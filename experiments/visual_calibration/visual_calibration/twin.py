"""The rig's digital twin: Shea's meshes (orca_files `calibration_rig/twin/`: twin.json,
one GLB per part, rig-frame mm, z up) and the live state to pose them.

twin.json's groups: static and camera fixed, rotor Rz(plate), object Rz(plate) ·
Rz(mounting yaw). The state gives those as 4×4 matrices (rig frame, mm), the CAD
next to the solved: for the object the solved one is Rz(plate + stop correction) ·
offset · tilt · Rz(tower yaw) (overlay.plate_in_rig · object_in_plate)."""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
from orca_core.kinematics.transforms import Transform

from visual_calibration.overlay import object_in_plate, plate_in_rig
from visual_calibration.rig_calibration import _rz
from visual_calibration.rig_config import RigConfig


def twin_dir() -> Path:
    env = os.environ.get("ORCA_RIG_TWIN")
    return Path(env) if env else Path(__file__).resolve().parents[4] / "orca_files" / "calibration_rig" / "twin"


def twin_info() -> dict | None:
    f = twin_dir() / "twin.json"
    return json.loads(f.read_text()) if f.is_file() else None


def _m(t: Transform) -> list:
    return t.matrix.round(6).tolist()


def _rot_z(deg: float) -> Transform:
    return Transform.from_rotation_translation(_rz(np.array([deg]))[0], np.zeros(3))


def twin_state(rig: RigConfig, angle: float | None) -> dict:
    cal = rig.load_calibration()
    a = 0.0 if angle is None else float(angle)
    mounting = cal.get("mounting")
    tower = cal.get("tower")
    nominal = rig.camera_nominal or {}
    cam = cal.get("camera") or {}
    solved_cam = Transform.from_xyz_rpy(cam["translation"], np.radians(cam["rotation"])) \
        if cam.get("translation") is not None and cam.get("rotation") is not None else None
    axis = cal.get("axis") or {}
    solved_axis = None
    if tower or cal.get("stop_angle_corrections"):
        # The turning axis seen from the tower (object frame): where the CAD tower would have to lean.
        inv = (plate_in_rig(rig, 0.0) @ object_in_plate(rig)).inverse()
        solved_axis = {"point_object": inv.apply_to_points(np.zeros((1, 3)))[0].round(3).tolist(),
                       "direction_object": (inv.rotation @ [0.0, 0.0, 1.0]).round(6).tolist(),
                       "tilt_deg": axis.get("tilt"), "offset_mm": axis.get("offset")}
    return {
        "angle": None if angle is None else round(a, 2),
        "mounting": mounting,
        "tower": tower,
        "groups": {"rotor": _m(_rot_z(a)),
                   "object_cad": _m(_rot_z(a) @ _rot_z(float((mounting or {}).get("yaw", 0.0)))),
                   "object_solved": _m(plate_in_rig(rig, a) @ object_in_plate(rig)) if tower else None},
        "camera": {"cad": {**nominal, **({"twin": (twin_info() or {}).get("camera")} if twin_info() else {})},
                   "solved": _m(solved_cam) if solved_cam is not None else None,
                   "frame": "OpenCV camera: x right, y down, z along the view; matrix = camera -> rig, mm"},
        "axis": {"cad": {"point": [0.0, 0.0, 0.0], "direction": [0.0, 0.0, 1.0]}, "solved": solved_axis},
        "twin_available": twin_info() is not None,
    }
