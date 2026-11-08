"""Structured warnings from the solves, for the UI: {code, severity, message, fix, values}.

Severity: "info" (worth knowing), "warning" (the result is usable but something is
off), "error" (do not apply it). Thresholds: config.yaml `checks`.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np
from orca_core.kinematics.transforms import Transform

from visual_calibration.calibration_object import CalibrationObject
from visual_calibration.mounting import coverage
from visual_calibration.rig_config import RigConfig

DEFAULT_CHECKS = {"max_fit_px": 1.0, "max_photo_median_px": 1.0, "warn_photo_px": 2.0, "max_held_out_px": 1.0,
                  "max_hold_px": 1.0, "max_tower_tilt_deg": 0.5, "max_tower_offset_mm": 2.0,
                  "max_camera_off_mm": 5.0, "max_camera_off_deg": 2.0, "min_tile_corners": 8,
                  "max_stop_correction_deg": 1.0, "min_photo_corners": 24}


def warning(code: str, severity: str, message: str, fix: str = "", **values) -> dict:
    return {"code": code, "severity": severity, "message": message, "fix": fix, "values": values}


def limits(rig: RigConfig) -> dict:
    return {**DEFAULT_CHECKS, **(rig.raw.get("checks") or {})}


def turntable(rig: RigConfig, calibration: Mapping | None) -> list[dict]:
    if (calibration or {}).get("zero_in_turn_deg") is None:
        return [warning("turntable_uncalibrated", "warning", "The turntable motor is not calibrated: the plate "
                        "angles are not known.", "Run Calibrate motor (python -m visual_calibration.turntable calibrate).")]
    return []


def _r(v, n=2):
    return [round(float(x), n) for x in v] if isinstance(v, (list, tuple, np.ndarray)) else round(float(v), n)


def solve_warnings(rig: RigConfig, step: str, result: Mapping, photos: Sequence) -> list[dict]:
    """Everything worth flagging in a 1a ("camera") or 1b ("rotation") result."""
    lim, out = limits(rig), []
    obj = CalibrationObject.from_config(rig.raw["calibration_object"])
    fit = result.get("camera_fit" if step == "camera" else "fit") or {}
    rms = fit.get("reprojection_px")
    if rms is not None and rms > lim["max_fit_px"]:
        out.append(warning("residual_high", "warning", f"The fit is {rms:.2f} px, the target {lim['max_fit_px']:g} px.",
                           "Lock the focus on the tower, light it evenly, and retake the photos with few corners.",
                           rms_px=_r(rms), target_px=lim["max_fit_px"]))

    per = result.get("per_photo_px") or {}
    high = {k: v for k, v in per.items() if v > lim["warn_photo_px"]}
    if high:
        out.append(warning("photo_residual", "warning", f"{len(high)} photo(s) above {lim['warn_photo_px']:g} px "
                           f"(worst {max(high, key=high.get)}: {max(high.values()):.2f} px).",
                           "Usually a photo that sees one or two tiles only; retake it or leave it.", photos=high))

    m = result.get("mounting")
    if m:
        if m.get("direction_known") and m.get("direction", 1) < 0:
            out.append(warning("plate_direction", "error", "The plate turns the other way than the motor config says.",
                               "Set motor.gear_ratio to -1.0 in turntable.yaml and calibrate the motor again.",
                               direction=-1))
        elif not m.get("snapped"):
            out.append(warning("mounting_unusual", "warning",
                               f"The tower sits at {m['measured']:.0f}°, not at one of the connector's mountings.",
                               "Check that the Franka connector is bolted on square; the found angle is used as is.",
                               yaw_deg=_r(m["yaw"], 1), measured_deg=_r(m["measured"], 1), side0_stop_deg=_r(m["side0_stop"], 1)))
        else:
            out.append(warning("mounting_found", "info",
                               f"Tower mounted at {m['yaw']:.0f}°: side 0 faces the camera at {m['side0_stop']:.0f}°.",
                               yaw_deg=_r(m["yaw"], 1), measured_deg=_r(m["measured"], 1), side0_stop_deg=_r(m["side0_stop"], 1)))

    tower = result.get("tower")
    if tower:
        bound = rig.search.get("tower", {})
        tilt, off = np.array(tower["tilt"], float), np.array(tower["offset"], float)
        t, o = float(np.hypot(*tilt)), float(np.hypot(*off))
        at_bound = any(abs(v) >= 0.98 * b for v, b in zip(tilt, bound.get("tilt", {}).get("bound", [np.inf] * 2))) or \
            any(abs(v) >= 0.98 * b for v, b in zip(off, bound.get("offset", {}).get("bound", [np.inf] * 2)))
        if t > lim["max_tower_tilt_deg"] or at_bound:
            out.append(warning("tower_tilt", "error" if at_bound else "warning",
                               f"The tower leans {t:.1f}° to the turning axis" + (" (at the solver's limit)" if at_bound else "") + ".",
                               "Reseat the tower flat and fully down on the Franka connector (a tight fit tilts it); "
                               "if it stays, the plate's axis runs untrue.", tilt_deg=_r(tilt), total_deg=_r(t)))
        if o > lim["max_tower_offset_mm"]:
            out.append(warning("tower_offset", "error" if at_bound else "warning",
                               f"The tower stands {o:.1f} mm off the turning axis.",
                               "Check the Franka connector is centred on the plate.", offset_mm=_r(off), total_mm=_r(o)))

    corr = {float(k): float(v) for k, v in (result.get("stop_angle_corrections") or {}).items()}
    worst = max(corr, key=lambda k: abs(corr[k]), default=None)
    if worst is not None and abs(corr[worst]) > lim["max_stop_correction_deg"]:
        out.append(warning("stop_angle", "warning", f"Stop {worst:g}° is {corr[worst]:+.2f}° off its motor angle.",
                           "Check the motor holds the plate (PWM cap, backlash) and that the plate turns freely.",
                           worst_stop=worst, correction_deg=_r(corr[worst], 3)))
    cam, nominal = result.get("camera"), rig.camera_nominal
    if cam and cam.get("translation") is not None and nominal:
        d = np.array(cam["translation"], float) - np.array(nominal["position"], float)
        view = np.array(nominal["look_at"], float) - np.array(nominal["position"], float)
        z = Transform.from_xyz_rpy(cam["translation"], np.radians(cam["rotation"])).rotation[:, 2]
        deg = float(np.degrees(np.arccos(np.clip(z @ view / np.linalg.norm(view), -1, 1))))
        mm = float(np.linalg.norm(d))
        if mm > lim["max_camera_off_mm"] or deg > lim["max_camera_off_deg"]:
            out.append(warning("camera_off_cad", "warning",
                               f"The camera is {mm:.0f} mm and {deg:.1f}° from where the mount setting puts it.",
                               "Check camera.mount (distance, height) in config.yaml against the holes used. A camera "
                               "found low by a few mm can also be the tower sitting high (the two cannot be told apart).",
                               mm=_r(mm, 1), deg=_r(deg), delta_mm=_r(d, 1)))
        elif abs(d[2]) > 2.0:
            out.append(warning("camera_height", "info",
                               f"The camera comes out {d[2]:+.1f} mm from its CAD height: the camera, or the tower "
                               "sitting that much the other way (held at the CAD height).", delta_mm=_r(d, 1)))

    sightings: dict[str, list[int]] = {}
    weak_photos = []
    for p in photos:
        n = sum(len(d.ids) for d in p.detections)
        if n < lim["min_photo_corners"]:
            weak_photos.append(p.name or f"{p.stop_deg:g}")
        for d in p.detections:
            sightings.setdefault(d.tile, []).append(len(d.ids))
    weak = {t: _r(np.mean(v), 1) for t, v in sightings.items() if np.mean(v) < lim["min_tile_corners"]}
    if weak or weak_photos:
        out.append(warning("weak_tiles", "info" if not weak else "warning",
                           (f"Tiles found with few corners: {', '.join(sorted(weak))}. " if weak else "") +
                           (f"{len(weak_photos)} photos with fewer than {lim['min_photo_corners']} corners." if weak_photos else ""),
                           "More light and a locked focus; photos of one side-on tile add little.",
                           tiles=weak, photos=weak_photos))

    seen = {obj.tiles[t].side for t in sightings}
    missing = [s for s in (0, 2) if s not in seen] + ([] if seen & {1, 3} else ["1 or 3"])
    stops = sorted({p.stop_deg for p in photos})
    plan = coverage(obj, stops, [float(m["yaw"])] if m else rig.raw["calibration_object"].get("mountings", [0.0]))
    if missing or plan["warnings"]:
        out.append(warning("stop_coverage", "warning",
                           (f"No photo shows side(s) {', '.join(map(str, missing))}. " if missing else "") +
                           " ".join(plan["warnings"]),
                           "Add stops so every side faces the camera at least once.",
                           missing_sides=missing, stops=_r(stops, 1)))
    return out


def as_structured(warnings) -> list[dict]:
    """Old results kept plain strings."""
    return [w if isinstance(w, dict) else warning("note", "warning", str(w)) for w in warnings or []]



def camera_off(rig: RigConfig, cam: Mapping) -> tuple[float, float]:
    """The solved camera from its CAD pose: mm, deg."""
    n = rig.camera_nominal
    view = np.array(n["look_at"], float) - np.array(n["position"], float)
    z = Transform.from_xyz_rpy(cam["translation"], np.radians(cam["rotation"])).rotation[:, 2]
    return (float(np.linalg.norm(np.array(cam["translation"], float) - n["position"])),
            float(np.degrees(np.arccos(np.clip(z @ view / np.linalg.norm(view), -1, 1)))))


def verdict(rig: RigConfig, rotation: Mapping, held_out_px: float | None, hold_px: float | None) -> dict:
    """Pass/fail of a 1a + 1b run against config.yaml `checks`: {name: {value, limit, ok}}.
    Only these decide whether a run is applied; a photo above warn_photo_px is a warning."""
    lim = limits(rig)
    per = list((rotation.get("per_photo_px") or {}).values())
    corr = [abs(float(v)) for v in (rotation.get("stop_angle_corrections") or {}).values()]
    tw = rotation.get("tower") or rotation.get("axis") or {}
    mm, deg = camera_off(rig, rotation["camera"])
    rows = {
        "1a held-out px": (held_out_px, lim["max_held_out_px"]),
        "1b fit px": ((rotation.get("fit") or {}).get("reprojection_px"), lim["max_fit_px"]),
        "1b median per photo px": (float(np.median(per)) if per else None, lim["max_photo_median_px"]),
        "hold px": (hold_px, lim["max_hold_px"]),
        "stop angle deg": (max(corr, default=None), lim["max_stop_correction_deg"]),
        "tower tilt deg": (float(np.hypot(*tw["tilt"])) if tw else None, lim["max_tower_tilt_deg"]),
        "tower offset mm": (float(np.hypot(*tw["offset"])) if tw else None, lim["max_tower_offset_mm"]),
        "camera vs CAD mm": (mm, lim["max_camera_off_mm"]),
        "camera vs CAD deg": (deg, lim["max_camera_off_deg"]),
    }
    return {k: {"value": None if v is None else round(float(v), 3), "limit": float(lim_),
                "ok": v is not None and v <= lim_} for k, (v, lim_) in rows.items()}
