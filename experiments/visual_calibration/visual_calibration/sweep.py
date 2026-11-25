"""A sweep on the rig from the command line: the camera check, the zero check, a
photo at every stop and the first stop again, then park.

The camera check runs before anything moves: the cameras are opened and told apart
(rig_cameras.RigCameras.preflight: by serial, proven by a blink of each one's exposure, which
writes and restores that one control even with --verify-only), the UVC controls are set to
config.yaml camera.uvc and read back (uvc.ensure); then one frame at the
reference stop (where the plate is parked) is held against
``camera_reference.yaml``: sharpness, tile scale (focus), brightness, clipping.
Either failing aborts. At the end the controls are read again.

    PYTHONPATH=. uv run --with opencv-contrib-python-headless --with dynamixel-sdk \\
        python -m visual_calibration.sweep OUT [--step 7.5] [--save-reference]
    ... python -m visual_calibration.sweep --list-cameras     # serials, USB locations, AVFoundation order
    ... python -m visual_calibration.sweep --assign-cameras   # write them into config.yaml (camera_assign)
    ... python -m visual_calibration.sweep --assign-cameras --swap   # after Check cameras found A and B swapped
    ... python -m visual_calibration.sweep --quick            # the per-session check, nothing but the park stop

Writes OUT/stop_<deg>.png, OUT/repeat_<deg>.png and OUT/sweep.yaml (read-back
angles, stillness, the camera before and after). Several cameras (config.yaml cameras:,
the enabled ones): each is checked against its own reference (camera_reference.yaml for
"a", camera_reference_<cam>.yaml for the others) and grabbed at every stop after the plate
has stopped (stillness from the first camera): OUT/stop_<deg>_<cam>.png.
"""
from __future__ import annotations

import datetime
import json
import time
from collections.abc import Callable
from pathlib import Path

import numpy as np
from orca_core.utils.utils import read_yaml, write_yaml_atomic

from visual_calibration.rig_config import DEFAULT_CAMERA
from visual_calibration.uvc import CameraSettingsError, Uvcc, compare, differences, ensure, frame_stats, wanted

REFERENCE_FILE = "camera_reference.yaml"
REFERENCE_STOP = 90.0
STILL_PX = 0.3                  # corner motion between two frames: the plate has stopped
STILL_TRIES = 3
ZERO_TOL_DEG = 0.5


class Webcam:
    """The rig camera through OpenCV (AVFoundation on macOS), at full resolution: the process's
    one shared capture (visual_calibration.capture), never reopened per still."""

    def __init__(self, index: int = 0, size: tuple[int, int] = (3840, 2160)):
        from visual_calibration.capture import SharedCamera
        self.index, self.size = index, tuple(size)
        self.shared = SharedCamera.get(index, self.size)

    def grab(self) -> np.ndarray:
        """A frame captured after this call (nothing buffered before it)."""
        return self.shared.still()

    def close(self) -> None:
        self.shared.close()


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


def reference_path(rig, cam: str = DEFAULT_CAMERA) -> Path:
    return Path(rig.path) / (REFERENCE_FILE if cam == DEFAULT_CAMERA else f"camera_reference_{cam}.yaml")


def _suffix(cam: str) -> str:
    return "" if cam == DEFAULT_CAMERA else f"_{cam}"


def check_camera(rig, uvcc: Uvcc | None, grab: Callable, detector, at_deg: float, write: bool = True,
                 cam: str = DEFAULT_CAMERA, detections: list | None = None) -> dict:
    """The camera check before the motor moves. Raises CameraSettingsError."""
    settings = ensure(uvcc, rig.cameras[cam]["uvc"], write) if uvcc else None
    gray = _gray(grab())
    dets = detector.detect(gray)
    if detections is not None:
        detections[:] = dets
    stats = frame_stats(gray, dets, detector.obj, rig.raw["calibration_object"].get("letters"))
    path = reference_path(rig, cam)
    ref = read_yaml(str(path)) if path.exists() else None
    problems = []
    if ref:
        if abs(ref["stop"] - at_deg) > 0.5:
            problems.append(f"the reference is at {ref['stop']}°, the plate at {at_deg:.2f}°")
        else:
            problems = compare(stats, ref)
    if problems:
        raise CameraSettingsError(("camera check: " if cam == DEFAULT_CAMERA else f"camera {cam} check: ")
                                  + "; ".join(problems))
    return {"uvc": settings, "frame": stats, "at_deg": at_deg, "reference": bool(ref)}


def save_reference(rig, check: dict, cam: str = DEFAULT_CAMERA) -> Path:
    path = reference_path(rig, cam)
    write_yaml_atomic(str(path), {"stop": float(round(check["at_deg"], 1)),
                                  "date": datetime.datetime.now().isoformat(timespec="seconds"),
                                  "uvc": wanted(rig.cameras[cam]["uvc"]), **check["frame"]})
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


def run(rig, t, grab: Callable | dict[str, Callable], detector, out: Path, stops: list[float],
        uvcc: Uvcc | dict[str, Uvcc] | None = None,
        settle_s: float = 1.0, write: bool = True, zero_check: bool = True,
        reference_stop: float = REFERENCE_STOP, save_ref: bool = False, imwrite: Callable | None = None,
        hand: bool = False) -> dict:
    """The camera check, the zero check, the stops and the first again, park at the reference stop.
    grab / uvcc: camera "a"'s, or one per camera {name: ...} (every camera at every stop).
    hand: the hand is on the plate, not the object: the controls are checked, the tile frame is not.
    The torque holds between the stops and is off at the end, however it ends."""
    try:
        return _run(rig, t, grab, detector, out, stops, uvcc, settle_s, write, zero_check, reference_stop,
                    save_ref, imwrite, hand)
    finally:
        t.release()


def _cams(x) -> dict:
    return dict(x) if isinstance(x, dict) else ({DEFAULT_CAMERA: x} if x is not None else {})


def _run(rig, t, grab, detector, out, stops, uvcc, settle_s, write, zero_check, reference_stop, save_ref,
         imwrite, hand) -> dict:
    if imwrite is None:
        import cv2
        imwrite = lambda p, img: cv2.imwrite(str(p), img)
    grabs, uvccs = _cams(grab), _cams(uvcc)
    first, others = next(iter(grabs)), [c for c in grabs if c != next(iter(grabs))]
    out.mkdir(parents=True, exist_ok=True)
    at = t.angle()
    log: dict = {"started": datetime.datetime.now().isoformat(timespec="seconds"), "stops": {}}
    cams_log: dict = {c: {} for c in grabs}
    for c, u in uvccs.items():
        cams_log.setdefault(c, {})["uvc"] = ensure(u, rig.cameras[c]["uvc"], write)       # nothing has moved
    if not hand:
        if abs(at - reference_stop) > 0.5:
            t.move(reference_stop, settle_s=settle_s, hold=True)
        for c, g in grabs.items():
            cams_log[c] = {**check_camera(rig, None, g, detector, t.angle(), cam=c), **cams_log[c]}
    if cams_log.get(DEFAULT_CAMERA):
        log["camera"] = cams_log[DEFAULT_CAMERA]
    if any(cams_log.get(c) for c in cams_log if c != DEFAULT_CAMERA):
        log["cameras"] = {c: v for c, v in cams_log.items() if c != DEFAULT_CAMERA}
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
        img, angle, moved = _photo(t, grabs[first], detector, a, settle_s)
        imwrite(out / f"{name}{_suffix(first)}.png", img)
        log["stops"][name + _suffix(first)] = {"stop": a, "angle": angle, "motion_px": moved}
        for c in others:                                           # the plate has stopped: the first camera says so
            imwrite(out / f"{name}_{c}.png", grabs[c]())
            log["stops"][f"{name}_{c}"] = {"stop": a, "angle": angle, "motion_px": None, "camera": c}
        print(f"{name} at {angle:7.2f}" + ("" if moved is None else f", motion {moved:.2f} px"), flush=True)
    t.move(reference_stop, settle_s=0.5)
    for c, u in uvccs.items():
        after = u.export()
        cams_log[c]["uvc_after"] = after
        cams_log[c]["changed"] = {k: list(v) for k, v in differences(wanted(rig.cameras[c]["uvc"]), after).items()}
    if save_ref and not hand:
        for c in grabs:
            cams_log[c]["reference_saved"] = str(save_reference(rig, cams_log[c], c))
    (out / "angles.json").write_text(json.dumps({k: v["angle"] for k, v in log["stops"].items()}, indent=1))
    write_yaml_atomic(str(out / "sweep.yaml"), _plain(log))
    return log


def _plain(x):
    if isinstance(x, dict):
        return {str(k): _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_plain(v) for v in x]
    return x.item() if isinstance(x, np.generic) else x


def quick(rig, t, grabs: dict[str, Callable], detector, reference_stop: float = REFERENCE_STOP,
          settle_s: float = 1.0) -> dict:
    """The per-session check, before a hand session: one frame per camera at the park stop,
    against its reference frame (sharpness, scale, light) and against the applied calibration
    (checks.quick_check: the tiles where it puts them). Moves only to the park stop."""
    from visual_calibration.checks import quick_check
    if abs(t.angle() - reference_stop) > 0.5:
        t.move(reference_stop, settle_s=settle_s, hold=True)
    at, frames, problems = t.angle(), {}, {}
    for c, g in grabs.items():
        frames[c] = []
        try:
            check_camera(rig, None, g, detector, at, cam=c, detections=frames[c])
        except CameraSettingsError as e:
            problems[c] = str(e)
    out = quick_check(rig, frames, at)
    out["reference"] = problems
    return out


def list_cameras(run=None) -> dict:
    """The USB cameras (serial, location, address: for config.yaml cameras.<c>.uvc.device) and
    AVFoundation's video devices (their order is usually OpenCV's index). Opens nothing."""
    import subprocess

    from visual_calibration.camera_assign import usb_cameras
    from visual_calibration.rig_cameras import avfoundation_devices
    run = run or subprocess.run
    av = avfoundation_devices(run)
    return {"usb": usb_cameras(run, av=av), "avfoundation": av}


def main() -> None:
    import argparse

    from visual_calibration.calibration_object import CalibrationObject
    from visual_calibration.detect import TileDetector
    from visual_calibration.rig_cameras import RigCameras
    from visual_calibration.rig_config import RigConfig
    from visual_calibration.turntable import Turntable

    ap = argparse.ArgumentParser(description="Camera check, then a photo at every stop (the rig's turntable).")
    ap.add_argument("out", type=Path, nargs="?")
    ap.add_argument("--list-cameras", action="store_true", help="the cameras' serials and indices, nothing moves")
    ap.add_argument("--assign-cameras", nargs="*", type=int, metavar="N",
                    help="write the plugged-in USB cameras into config.yaml as cameras a, b (N: the listed "
                         "cameras to take, in that order); nothing opened, nothing moves")
    ap.add_argument("--swap", action="store_true", help="with --assign-cameras: trade cameras a and b")
    ap.add_argument("--quick", action="store_true", help="the per-session check: one frame per camera at the park stop")
    ap.add_argument("--step", type=float, help="deg between stops (default turntable.yaml stops.step)")
    ap.add_argument("--verify-only", action="store_true", help="do not set the camera controls, only check them")
    ap.add_argument("--no-zero-check", action="store_true")
    ap.add_argument("--save-reference", action="store_true", help="this sweep's check frame becomes the reference")
    ap.add_argument("--hand", action="store_true", help="the hand is on the plate: no tile frame check")
    args = ap.parse_args()
    if args.list_cameras:
        print(json.dumps(list_cameras(), indent=1))
        return
    if args.assign_cameras is not None or args.swap:
        from visual_calibration.camera_assign import AssignError, assign
        try:
            print("\n".join(assign(RigConfig.load(), args.assign_cameras, do_swap=args.swap)))
        except AssignError as e:
            raise SystemExit(str(e)) from None
        return
    if not args.quick and args.out is None:
        ap.error("OUT is needed for a sweep")
    rig = RigConfig.load()
    tt = rig.raw["turntable"]
    st = tt.get("stops") or {}
    step = args.step or float(st.get("step", 15.0))
    stops = [float(st.get("start", 0.0)) + step * k for k in range(int((float(tt["travel"]) - 5.0) // step) + 1)]
    detector = TileDetector(CalibrationObject.from_config(rig.raw["calibration_object"]))
    cfg = {c: rig.cameras[c] for c in rig.active_cameras}
    uvcc = {c: Uvcc.from_config(v["uvc"]) for c, v in cfg.items()}
    cams = RigCameras(rig)
    print(json.dumps(_plain(cams.preflight(write=not args.verify_only)), indent=1), flush=True)   # which stream is which
    grabs = {c: (lambda c=c: cams.still(c, after=time.monotonic())) for c in cfg}
    t = Turntable.open(tt, calibration_path=rig.turntable_calibration_path)
    try:
        if args.quick:
            print(json.dumps(_plain(quick(rig, t, grabs, detector)), indent=1))
            return
        run(rig, t, grabs if len(grabs) > 1 else grabs[DEFAULT_CAMERA], detector, args.out, stops,
            uvcc if len(uvcc) > 1 else uvcc[DEFAULT_CAMERA], settle_s=max(1.0, float(st.get("settle_s", 1.0))),
            write=not args.verify_only, zero_check=not args.no_zero_check, save_ref=args.save_reference,
            hand=args.hand)
    finally:
        cams.close()
        t.close()
        print("torque off", flush=True)


if __name__ == "__main__":
    main()
