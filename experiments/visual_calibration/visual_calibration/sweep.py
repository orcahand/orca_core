"""A sweep on the rig from the command line: the camera check, the zero check, a
photo at every stop and the first stop again, then park.

The camera check runs before anything moves: the UVC controls are set to
config.yaml camera.uvc and read back (uvc.ensure); then one frame at the
reference stop (where the plate is parked) is held against
``camera_reference.yaml``: sharpness, tile scale (focus), brightness, clipping.
Either failing aborts. At the end the controls are read again.

    PYTHONPATH=. uv run --with opencv-contrib-python-headless --with dynamixel-sdk \\
        python -m visual_calibration.sweep OUT [--step 7.5] [--save-reference]

Writes OUT/stop_<deg>.png, OUT/repeat_<deg>.png and OUT/sweep.yaml (read-back
angles, stillness, the camera before and after).
"""
from __future__ import annotations

import datetime
import json
import time
from collections.abc import Callable
from pathlib import Path

import numpy as np
from orca_core.utils.utils import read_yaml, write_yaml_atomic

from visual_calibration.uvc import CameraSettingsError, Uvcc, compare, differences, ensure, frame_stats, wanted

REFERENCE_FILE = "camera_reference.yaml"
REFERENCE_STOP = 90.0
STILL_PX = 0.3                  # corner motion between two frames: the plate has stopped
STILL_TRIES = 3
ZERO_TOL_DEG = 0.5


class Webcam:
    """The rig camera through OpenCV (AVFoundation on macOS), at full resolution."""

    def __init__(self, index: int = 0, size: tuple[int, int] = (3840, 2160)):
        self.index, self.size = index, tuple(size)
        self._open()

    def _open(self) -> None:
        import cv2
        self.cap = cv2.VideoCapture(self.index, cv2.CAP_AVFOUNDATION)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        for _ in range(30):
            self.cap.read()

    def grab(self) -> np.ndarray:
        for _ in range(3):
            for _ in range(4):
                self.cap.read()
            ok, img = self.cap.read()
            if ok and img.shape[1::-1] == self.size:
                return img
            self.cap.release()
            self._open()
        raise RuntimeError(f"no {self.size[0]}x{self.size[1]} frame from camera {self.index}")

    def close(self) -> None:
        self.cap.release()


def _gray(img: np.ndarray) -> np.ndarray:
    import cv2
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def motion_px(a, b) -> float | None:
    """RMS motion of the corners two detections share."""
    d = []
    for x in a:
        y = next((y for y in b if y.tile == x.tile), None)
        if y is not None:
            _, ia, ib = np.intersect1d(x.ids, y.ids, return_indices=True)
            d.append(x.uv[ia] - y.uv[ib])
    return float(np.sqrt((np.vstack(d) ** 2).sum(1).mean())) if d and sum(map(len, d)) else None


def reference_path(rig) -> Path:
    return Path(rig.path) / REFERENCE_FILE


def check_camera(rig, uvcc: Uvcc | None, grab: Callable, detector, at_deg: float, write: bool = True) -> dict:
    """The camera check before the motor moves. Raises CameraSettingsError."""
    settings = ensure(uvcc, rig.raw["camera"]["uvc"], write) if uvcc else None
    gray = _gray(grab())
    stats = frame_stats(gray, detector.detect(gray), detector.obj, rig.raw["calibration_object"].get("letters"))
    ref = read_yaml(str(reference_path(rig))) if reference_path(rig).exists() else None
    problems = []
    if ref:
        if abs(ref["stop"] - at_deg) > 0.5:
            problems.append(f"the reference is at {ref['stop']}°, the plate at {at_deg:.2f}°")
        else:
            problems = compare(stats, ref)
    if problems:
        raise CameraSettingsError("camera check: " + "; ".join(problems))
    return {"uvc": settings, "frame": stats, "at_deg": at_deg, "reference": bool(ref)}


def save_reference(rig, check: dict) -> Path:
    path = reference_path(rig)
    write_yaml_atomic(str(path), {"stop": float(round(check["at_deg"], 1)),
                                  "date": datetime.datetime.now().isoformat(timespec="seconds"),
                                  "uvc": wanted(rig.raw["camera"]["uvc"]), **check["frame"]})
    return path


def _photo(t, grab, detector, angle: float, settle_s: float) -> tuple[np.ndarray, float, float | None]:
    """Move, then frames until two in a row agree (the plate has stopped)."""
    s = t.move(angle, settle_s=settle_s, hold=True)
    prev = grab()
    for _ in range(STILL_TRIES):
        img = grab()
        m = motion_px(detector.detect(_gray(prev)), detector.detect(_gray(img)))
        if m is None or m <= STILL_PX:
            break
        time.sleep(1.0)
        prev = grab()
    return img, s.angle, m


def run(rig, t, grab: Callable, detector, out: Path, stops: list[float], uvcc: Uvcc | None = None,
        settle_s: float = 1.0, write: bool = True, zero_check: bool = True,
        reference_stop: float = REFERENCE_STOP, save_ref: bool = False, imwrite: Callable | None = None,
        hand: bool = False) -> dict:
    """The camera check, the zero check, the stops and the first again, park at the reference stop.
    hand: the hand is on the plate, not the object: the controls are checked, the tile frame is not.
    The torque holds between the stops and is off at the end, however it ends."""
    try:
        return _run(rig, t, grab, detector, out, stops, uvcc, settle_s, write, zero_check, reference_stop,
                    save_ref, imwrite, hand)
    finally:
        t.release()


def _run(rig, t, grab, detector, out, stops, uvcc, settle_s, write, zero_check, reference_stop, save_ref,
         imwrite, hand) -> dict:
    if imwrite is None:
        import cv2
        imwrite = lambda p, img: cv2.imwrite(str(p), img)
    out.mkdir(parents=True, exist_ok=True)
    at = t.angle()
    log: dict = {"started": datetime.datetime.now().isoformat(timespec="seconds"), "stops": {}}
    if uvcc:
        log["camera"] = {"uvc": ensure(uvcc, rig.raw["camera"]["uvc"], write)}       # nothing has moved
    if not hand:
        if abs(at - reference_stop) > 0.5:
            t.move(reference_stop, settle_s=settle_s, hold=True)
        log["camera"] = {**check_camera(rig, None, grab, detector, t.angle()), **log.get("camera", {})}
    if zero_check and t.zero_from == "calibration":
        cal_zero = t.zero
        t.move(10.0, hold=True)
        t.home(hold=True)
        log["zero_vs_calibration_deg"] = float(np.degrees(t.zero - cal_zero) / t.ratio)
        if abs(log["zero_vs_calibration_deg"]) > ZERO_TOL_DEG:
            raise RuntimeError(f"the zero moved {log['zero_vs_calibration_deg']:+.2f}°: recalibrate the motor first")
        t.zero, t.zero_from = cal_zero, "calibration"
    for i, a in enumerate([*stops, stops[0]]):
        name = f"stop_{a:07.2f}" if i < len(stops) else f"repeat_{a:07.2f}"
        img, angle, moved = _photo(t, grab, detector, a, settle_s)
        imwrite(out / f"{name}.png", img)
        log["stops"][name] = {"stop": a, "angle": angle, "motion_px": moved}
        print(f"{name} at {angle:7.2f}" + ("" if moved is None else f", motion {moved:.2f} px"), flush=True)
    t.move(reference_stop, settle_s=0.5)
    if uvcc:
        after = uvcc.export()
        log["camera"]["uvc_after"] = after
        log["camera"]["changed"] = {k: list(v) for k, v in differences(wanted(rig.raw["camera"]["uvc"]), after).items()}
    if save_ref and not hand:
        log["camera"]["reference_saved"] = str(save_reference(rig, log["camera"]))
    (out / "angles.json").write_text(json.dumps({k: v["angle"] for k, v in log["stops"].items()}, indent=1))
    write_yaml_atomic(str(out / "sweep.yaml"), _plain(log))
    return log


def _plain(x):
    if isinstance(x, dict):
        return {str(k): _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_plain(v) for v in x]
    return x.item() if isinstance(x, np.generic) else x


def main() -> None:
    import argparse

    from visual_calibration.calibration_object import CalibrationObject
    from visual_calibration.detect import TileDetector
    from visual_calibration.rig_config import RigConfig
    from visual_calibration.turntable import Turntable

    ap = argparse.ArgumentParser(description="Camera check, then a photo at every stop (the rig's turntable).")
    ap.add_argument("out", type=Path)
    ap.add_argument("--step", type=float, help="deg between stops (default turntable.yaml stops.step)")
    ap.add_argument("--verify-only", action="store_true", help="do not set the camera controls, only check them")
    ap.add_argument("--no-zero-check", action="store_true")
    ap.add_argument("--save-reference", action="store_true", help="this sweep's check frame becomes the reference")
    ap.add_argument("--hand", action="store_true", help="the hand is on the plate: no tile frame check")
    args = ap.parse_args()
    rig = RigConfig.load()
    tt = rig.raw["turntable"]
    st = tt.get("stops") or {}
    step = args.step or float(st.get("step", 15.0))
    stops = [float(st.get("start", 0.0)) + step * k for k in range(int((float(tt["travel"]) - 5.0) // step) + 1)]
    detector = TileDetector(CalibrationObject.from_config(rig.raw["calibration_object"]))
    uvcc = Uvcc.from_config(rig.raw["camera"]["uvc"])
    cam = Webcam(0, tuple(rig.raw["camera"]["resolution"]))
    t = Turntable.open(tt, calibration_path=rig.turntable_calibration_path)
    try:
        run(rig, t, cam.grab, detector, args.out, stops, uvcc, settle_s=max(1.0, float(st.get("settle_s", 1.0))),
            write=not args.verify_only, zero_check=not args.no_zero_check, save_ref=args.save_reference,
            hand=args.hand)
    finally:
        cam.close()
        t.close()
        print("torque off", flush=True)


if __name__ == "__main__":
    main()
