"""Where the rig's calibration stands, per step, for the UI: the motor calibration,
1a camera, 1b rotation, 2 hand. Each step: status, time, key numbers (residual vs.
target, frames, corners) and structured warnings (visual_calibration.checks)."""
from __future__ import annotations

from pathlib import Path

from orca_core.utils.utils import read_yaml

from visual_calibration import checks
from visual_calibration.rig_config import RigConfig
from visual_calibration.session import Session

TITLES = {"motor": "Motor calibration", "camera": "1a · camera", "rotation": "1b · rotation", "hand": "2 · hand"}


def _motor(rig: RigConfig) -> dict:
    c = read_yaml(rig.turntable_calibration_path) or {}
    done = c.get("zero_in_turn_deg") is not None
    return {"id": "motor", "title": TITLES["motor"], "status": "done" if done else "missing",
            "time": c.get("calibrated_at"),
            "numbers": {"travel_deg": c.get("travel_deg"), "travel_error_deg": c.get("travel_error_deg"),
                        "homing_spread_deg": c.get("homing_spread_deg"),
                        "stops": len(c.get("stops_motor_deg") or {})} if done else {},
            "warnings": checks.turntable(rig, c)}


def _session_step(rig: RigConfig, kind: str, sessions: list[Session], applied: str | None) -> dict:
    s = next((x for x in sessions if x.meta["step"] == kind), None)
    out = {"id": kind, "title": TITLES[kind], "status": "none", "time": None, "session": None, "numbers": {},
           "warnings": []}
    if s is None:
        return out
    photos, result = s.meta["photos"], s.result()
    fit = (result or {}).get("camera_fit" if kind == "camera" else "fit") or {}
    out.update(session=s.id, time=(result or {}).get("calibrated_at") or s.meta["created"],
               status="applied" if applied == s.id else "solved" if result else "photos" if photos else "empty",
               numbers={"residual_px": fit.get("reprojection_px"), "target_px": checks.limits(rig)["max_fit_px"],
                        "frames": len(photos), "corners": int(sum(p.get("corners", 0) for p in photos))},
               warnings=checks.as_structured((result or {}).get("warnings")))
    if kind == "rotation" and result:
        out["numbers"]["tower"] = result.get("tower")
    if kind == "hand":
        out["note"] = "solving the hand is not built yet"
    return out


def calibration_status(rig: RigConfig, base: Path) -> dict:
    cal = rig.load_calibration()
    sessions = Session.list(base)
    return {"rig": rig.name, "version": rig.version, "calibrated_at": cal.get("calibrated_at"),
            "calibration_path": rig.calibration_path,
            "mounting": cal.get("mounting"),
            "steps": [_motor(rig)] + [_session_step(rig, k, sessions, cal.get(f"{k}_session"))
                                      for k in ("camera", "rotation", "hand")]}
