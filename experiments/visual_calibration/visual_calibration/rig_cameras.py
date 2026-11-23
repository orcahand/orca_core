"""The rig's cameras, owned by one process on the rig Mac (the console's backend): every enabled
camera of config.yaml `cameras:` opened once (capture.SharedCamera), told apart, its UVC controls
locked, and a still from each on request. The rig calibration's sweeps and the hand run take
every photo through here; the page only shows a preview of it.

Which OpenCV index is which camera: AVFoundation numbers its devices in its own order and may
renumber them on a replug, and two BRIOs look the same to OpenCV. So the index comes from the
camera's USB serial (serial -> USB location -> AVFoundation unique id -> its place in the device
list), and `preflight` proves it before a photo counts:

- blink: each camera's exposure is cut through uvcc, which selects the camera by its serial; the
  one stream that darkens is that camera (a clean swap of the opened streams is taken as found);
- tiles (the tower on the plate, rig calibration only): each camera's height above the plate,
  from PnP on the tiles it sees, must match its mount (cam A ~250 mm, cam B ~470 mm). A camera
  on the other mount, or serials swapped in config.yaml, fails here.

A camera missing, a stream that does not answer the blink, or a camera at the other's height:
CameraIdentityError, and no still is handed out until a preflight passes. A capture reopened
since (a replug) needs a new preflight.
"""
from __future__ import annotations

import json
import subprocess
import threading
import time
from collections.abc import Callable, Mapping

import numpy as np

from visual_calibration.rig_config import DEFAULT_CAMERA
from visual_calibration.uvc import CameraSettingsError, Uvcc, ensure, find_device, usb_devices, wanted

BLINK_CONTROL = "absolute_exposure_time"
BLINK_FACTOR = 8            # the exposure divided by this: the camera's own stream drops to ~1/8
BLINK_SETTLE_S = 0.5        # frames this long after a control is set show it
BLINK_MIN_DROP = 0.4        # the blinked stream darkens at least this share of its mean ...
BLINK_MAX_OTHER = 0.15      # ... and every other stream at most this share
HEIGHT_TOL_MM = 60.0        # PnP height (nominal lens) vs the mount's
MIN_CORNERS = 12            # tile corners a camera must see for the tile check


class CameraIdentityError(RuntimeError):
    """A camera missing, not identifiable, or not where config.yaml puts it: nothing is captured."""


def avfoundation_devices(run: Callable = subprocess.run) -> list[dict]:
    """AVFoundation's video devices in its order: {name, unique_id} (opens nothing)."""
    p = run(["system_profiler", "SPCameraDataType", "-json"], capture_output=True, text=True, timeout=30)
    try:
        return [{"name": c.get("_name"), "unique_id": c.get("spcamera_unique-id")}
                for c in json.loads(p.stdout or "{}").get("SPCameraDataType", [])]
    except ValueError:
        return []


def avfoundation_index(dev: Mapping, av: list[dict]) -> int | None:
    """A USB camera's place among AVFoundation's video devices (unique id = location + vendor + product);
    usually OpenCV's index, which the blink proves."""
    uid = f"0x{dev.get('location', 0):x}{dev.get('vendor', 0):04x}{dev.get('product', 0):04x}"
    return next((i for i, c in enumerate(av) if str(c.get("unique_id")).lower() == uid), None)


def _gray_mean(img: np.ndarray) -> float:
    return float(img[::8, ::8].mean())


class MockCamera:
    """--mock: a flat grey frame at the camera's size, nothing opened."""

    def __init__(self, index: int, size: tuple[int, int]):
        self.index, self.size, self.opens, self.frames = index, tuple(size), 1, 0
        self._stopped = False

    def still(self, after: float | None = None, timeout: float = 10.0) -> np.ndarray:
        self.frames += 1
        return np.full((self.size[1], self.size[0], 3), 90 + 40 * self.index, np.uint8)

    def stats(self) -> dict:
        return {"index": self.index, "size": list(self.size), "opens": self.opens, "frames": self.frames, "open": True}

    def close(self) -> None:
        self._stopped = True


def _shared(index: int, size: tuple[int, int]):
    from visual_calibration.capture import SharedCamera
    return SharedCamera.get(index, size)


class RigCameras:
    """Every enabled camera of the rig (rig.active_cameras), opened once, identified, locked."""

    def __init__(self, rig, open_camera: Callable | None = None, uvcc: Callable | None = None,
                 devices: Callable[[], list[dict]] | None = None, av_devices: Callable[[], list[dict]] | None = None,
                 mock: bool = False):
        self.rig, self.mock = rig, mock
        self.names = list(rig.active_cameras)
        self._open_camera = open_camera or (MockCamera if mock else _shared)
        self._uvcc = None if mock else (uvcc or Uvcc.from_config)
        self._devices = devices or usb_devices
        self._av = av_devices or avfoundation_devices
        self.indices: dict[str, int] = {}
        self.cams: dict = {}
        self.verified: dict | None = None
        self._seen: dict = {}
        self.lock = threading.RLock()            # a preflight (it blinks) never overlaps a still

    # ---- config
    def config(self, cam: str) -> dict:
        return self.rig.cameras[cam]

    def size(self, cam: str) -> tuple[int, int]:
        return tuple(self.config(cam).get("resolution", (3840, 2160)))

    def serial(self, cam: str):
        return ((self.config(cam).get("uvc") or {}).get("device") or {}).get("serial")

    def controls(self, cam: str) -> Uvcc | None:
        uvc = self.config(cam).get("uvc")
        return self._uvcc(uvc) if self._uvcc is not None and uvc else None

    # ---- opening
    def resolve(self) -> dict[str, int]:
        """The OpenCV index of every camera: from its serial where config.yaml has one, else its `index`."""
        if self.mock:
            return {c: int(self.config(c).get("index", i)) for i, c in enumerate(self.names)}
        unnamed = [c for c in self.names if not self.serial(c)]
        if len(self.names) > 1 and unnamed:
            raise CameraIdentityError(
                f"camera {', '.join(unnamed)}: no serial; two BRIOs are told apart by it: set "
                f"cameras.<c>.uvc.device.serial in config.yaml (python -m visual_calibration.sweep --list-cameras)")
        if unnamed:
            out = {c: int(self.config(c).get("index", 0)) for c in self.names}
        else:
            usb, av, out = self._devices(), self._av(), {}
            for c in self.names:
                d = self.config(c)["uvc"]["device"]
                try:
                    dev = find_device(d["vendor"], d["product"], d.get("serial"), d.get("location"), usb)
                except CameraSettingsError as e:
                    raise CameraIdentityError(f"camera {c} (serial {d.get('serial')}) is not plugged in: {e}") from e
                i = avfoundation_index(dev, av)
                if i is None:
                    raise CameraIdentityError(f"camera {c} (serial {d.get('serial')}) is on USB but not among "
                                              f"the Mac's video devices")
                out[c] = i
        if len(set(out.values())) != len(out):
            raise CameraIdentityError(f"two cameras at one index: {out}")
        return out

    def _alive(self) -> bool:
        return bool(self.cams) and all(not getattr(cam, "_stopped", False) for cam in self.cams.values())

    def open(self) -> dict:
        """Open every camera once (the process's shared capture); the indices."""
        with self.lock:
            if not self._alive():
                self.indices = self.resolve()
                self.cams = {c: self._open_camera(i, self.size(c)) for c, i in self.indices.items()}
                self.verified = None
            return self.indices

    def _fingerprint(self) -> dict:
        return {c: (id(cam), getattr(cam, "opens", 0)) for c, cam in self.cams.items()}

    @property
    def ok(self) -> bool:
        """A preflight passed and no capture was reopened since."""
        return self.verified is not None and self._alive() and self._seen == self._fingerprint()

    def _require(self) -> None:
        if not self.ok:
            raise CameraIdentityError("the cameras are not checked (or one was reopened): run the camera preflight")

    # ---- the checks
    def lock_controls(self, write: bool = True) -> dict:
        """Every camera's UVC controls to config.yaml (uvc.ensure); raises CameraSettingsError."""
        return {c: ensure(u, self.config(c)["uvc"], write) for c in self.names if (u := self.controls(c)) is not None}

    def blink(self) -> tuple[dict[str, str], dict]:
        """Which opened stream each camera is: its exposure cut through uvcc (selected by serial), the
        one stream that darkens. {camera: stream}, and every stream's drop per blink."""
        found, drops = {}, {}
        settled = time.monotonic()
        for c in self.names:
            u, want = self.controls(c), wanted(self.config(c).get("uvc") or {}).get(BLINK_CONTROL)
            if u is None or want is None:
                raise CameraIdentityError(f"camera {c}: no uvc {BLINK_CONTROL} in config.yaml, it cannot be told apart")
            base = {s: _gray_mean(cam.still(after=settled)) for s, cam in self.cams.items()}
            u.set(BLINK_CONTROL, max(1, int(want) // BLINK_FACTOR))
            try:
                t = time.monotonic() + BLINK_SETTLE_S
                now = {s: _gray_mean(cam.still(after=t)) for s, cam in self.cams.items()}
            finally:
                u.set(BLINK_CONTROL, want)
                settled = time.monotonic() + BLINK_SETTLE_S
            d = drops[c] = {s: round(1.0 - now[s] / max(base[s], 1e-6), 3) for s in self.cams}
            dark = [s for s, v in d.items() if v >= BLINK_MIN_DROP]
            if len(dark) != 1 or any(v > BLINK_MAX_OTHER for s, v in d.items() if s != dark[0]):
                raise CameraIdentityError(f"camera {c}: dimming it (serial {self.serial(c)}) darkened "
                                          f"{'stream ' + ', '.join(dark) if dark else 'no stream'} (drops {d}): "
                                          "a camera missing, or the light changing")
            found[c] = dark[0]
        if sorted(found.values()) != sorted(self.cams):
            raise CameraIdentityError(f"two cameras darkened the same stream: {found}")
        return found, drops

    def tile_check(self, detector, frames: Mapping[str, np.ndarray] | None = None) -> dict:
        """Each camera's height above the plate from PnP on its tiles, against its mount. Needs the tower."""
        from visual_calibration.overlay import camera_model
        from visual_calibration.rig_calibration import Photo, _pnp, _points
        frames = frames or {c: self.cams[c].still() for c in self.names}
        expected = {c: float((self.rig.nominal_of(c) or {"position": [0, 0, np.nan]})["position"][2]) for c in self.names}
        out = {}
        for c in self.names:
            dets = detector.detect(frames[c])
            n = int(sum(len(d.ids) for d in dets))
            if n < MIN_CORNERS:
                raise CameraIdentityError(f"camera {c} sees {n} tile corners: the tower on the plate, camera {c} "
                                          "plugged in and aimed at it?")
            pose, rms = _pnp(camera_model(self.rig, cam=c)[0], *_points(detector.obj, Photo(0.0, dets, c, c)))
            h = float(pose.translation[2])
            nearest = min((k for k in expected if np.isfinite(expected[k])), key=lambda k: abs(expected[k] - h), default=c)
            out[c] = {"height_mm": round(h, 1), "expected_mm": expected[c], "tiles": len(dets), "corners": n,
                      "pnp_px": round(rms, 2)}
            if nearest != c or not abs(h - expected[c]) <= HEIGHT_TOL_MM:
                raise CameraIdentityError(
                    f"camera {c} is {h:.0f} mm above the plate, its mount {expected[c]:.0f} mm"
                    + (f": that is camera {nearest}'s height (cameras swapped?)" if nearest != c else ""))
        return out

    def preflight(self, detector=None, write: bool = True) -> dict:
        """Open, lock the controls, prove which stream is which camera (blink; with `detector` and the
        tower on the plate, also its height). Raises CameraIdentityError / CameraSettingsError."""
        with self.lock:
            self.open()
            self.verified = None
            report: dict = {"indices": dict(self.indices)}
            if self.mock:
                report["identified_by"] = "mock"
            else:
                report["controls"] = {c: r["set"] for c, r in self.lock_controls(write).items()}
                if all(self.controls(c) is not None for c in self.names):
                    found, report["blink"] = self.blink()
                    if any(s != c for c, s in found.items()):
                        report["reordered"] = {c: s for c, s in found.items() if s != c}
                        cams, idx = dict(self.cams), dict(self.indices)
                        self.cams = {c: cams[found[c]] for c in self.names}
                        self.indices = {c: idx[found[c]] for c in self.names}
                        report["indices"] = dict(self.indices)
                    self.lock_controls(write)          # the blinked exposure is back
                    report["identified_by"] = "serial"
                elif len(self.names) > 1:
                    raise CameraIdentityError("two cameras without uvc blocks cannot be told apart")
                else:
                    report["identified_by"] = "index"
                if detector is not None:
                    report["tiles"] = self.tile_check(detector)
            report["at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            self.verified, self._seen = report, self._fingerprint()
            return report

    # ---- stills
    def stills(self, after: float | None = None) -> dict[str, np.ndarray]:
        """A frame from every camera, each captured after `after` (default now)."""
        with self.lock:
            self._require()
            t = time.monotonic() if after is None else after
            return {c: self.cams[c].still(after=t) for c in self.names}

    def still(self, cam: str, after: float | None = None) -> np.ndarray:
        with self.lock:
            self._require()
            if cam not in self.cams:
                raise KeyError(cam)
            return self.cams[cam].still(after=after)

    def preview(self, cam: str, max_side: int = 1600) -> np.ndarray:
        """The newest frame, scaled down (no preflight needed: the operator aims by it)."""
        if cam not in self.names:
            raise KeyError(cam)
        self.open()
        img = self.cams[cam].still()
        s = max_side / max(img.shape[:2])
        if s >= 1:
            return img
        import cv2
        return cv2.resize(img, (round(img.shape[1] * s), round(img.shape[0] * s)), interpolation=cv2.INTER_AREA)

    def status(self) -> dict:
        return {"cameras": [{"name": c, "index": self.indices.get(c), "serial": self.serial(c),
                             "resolution": list(self.size(c)),
                             "stats": self.cams[c].stats() if c in self.cams and hasattr(self.cams[c], "stats") else None}
                            for c in self.names],
                "checked": self.ok, "preflight": self.verified, "mock": self.mock}

    def close(self) -> None:
        with self.lock:
            for cam in self.cams.values():
                cam.close()
            self.cams, self.verified = {}, None


def encode(img: np.ndarray, fmt: str = "png", quality: int = 95) -> bytes:
    """PNG (lossless, the calibration's photos) or JPEG bytes."""
    import cv2
    if fmt in ("jpg", "jpeg"):
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    else:
        ok, buf = cv2.imencode(".png", img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    if not ok:
        raise ValueError(f"could not encode a {fmt}")
    return buf.tobytes()
