"""What the rig model expects the camera to see, for a live overlay on the video.

Before calibration it uses the CAD: the camera pose from `camera.mount`, the
lens from the BRIO's field of view (`camera.diagonal_fov_deg`), the tiles at
their CAD seats, no axis error. Each part switches to the solved value once
calibration.yaml has it (lens from step 1a, camera/axis/stops from 1b). So the
overlay is a sanity check before calibrating (the object's mounting not yet found, wrong mount
hole or camera mode) and a proof after it: everything rigid must line up.

Pixels are in the raw camera image (as the camera sends it, before the page
rotates it for display); lengths mm.
"""
from __future__ import annotations

import numpy as np
from orca_core.kinematics.transforms import Transform

from visual_calibration.calibration_object import CalibrationObject
from visual_calibration.camera import Lens, look_at, nominal_lens
from visual_calibration.rig_calibration import _rot, _rz, lens_from_calibration, tile_poses_from_calibration
from visual_calibration.rig_config import RigConfig


def _roll(eye: np.ndarray, target: np.ndarray, lens: Lens, display_rotation: float) -> Transform:
    """The nominal camera pose, rolled about its axis so that the rig's up shows
    as up once the page turns the raw image by `display_rotation` (clockwise)."""
    r = np.radians(display_rotation)
    turn = np.array([[np.cos(r), -np.sin(r)], [np.sin(r), np.cos(r)]])    # clockwise on screen (y down)
    base = look_at(eye, target, up=(0.0, 0.0, 1.0))
    best, score = base, -np.inf
    for k in range(4):
        R = base.rotation @ _rot(np.array([0.0, 0.0, 90.0 * k]))
        pose = Transform.from_rotation_translation(R, base.translation)
        uv = lens.project(pose.inverse().apply_to_points(np.array([target, target + [0, 0, 50.0]])))
        d = turn @ (uv[1] - uv[0])
        if -d[1] / np.linalg.norm(d) > score:
            best, score = pose, -d[1] / np.linalg.norm(d)
    return best


def camera_model(rig: RigConfig, display_rotation: float = 90.0) -> tuple[Lens, Transform, dict]:
    """Lens and camera pose (in the rig frame), solved where calibration.yaml has them."""
    cal = rig.load_calibration()
    cam_cfg = rig.raw["camera"]
    size = tuple(cam_cfg.get("resolution", (3840, 2160)))
    calibrated_lens = bool((cal.get("lens") or {}).get("fx"))
    lens = lens_from_calibration(cal) if calibrated_lens else \
        nominal_lens(size, float(cam_cfg.get("diagonal_fov_deg", 78.0)))
    c = cal.get("camera") or {}
    calibrated_pose = c.get("translation") is not None and c.get("rotation") is not None
    if calibrated_pose:
        pose = Transform.from_xyz_rpy(c["translation"], np.deg2rad(c["rotation"]))
    else:
        n = rig.camera_nominal
        pose = _roll(np.array(n["position"], float), np.array(n["look_at"], float), lens, display_rotation)
    return lens, pose, {"lens": "calibrated" if calibrated_lens else "nominal",
                        "camera": "calibrated" if calibrated_pose else "nominal"}


def plate_in_rig(rig: RigConfig, angle_deg: float) -> Transform:
    """The plate frame at `angle_deg` in the rig frame: the stop correction
    (interpolated between stops) and the axis tilt/offset from calibration.yaml."""
    cal = rig.load_calibration()
    corr = {float(k): float(v) for k, v in (cal.get("stop_angle_corrections") or {}).items()}
    c = float(np.interp(angle_deg, sorted(corr), [corr[k] for k in sorted(corr)])) if corr else 0.0
    axis = cal.get("axis") or {}
    tilt, off = np.array(axis.get("tilt") or [0.0, 0.0], float), np.array(axis.get("offset") or [0.0, 0.0], float)
    M = _rot(np.array([tilt[0], 0, 0])) @ _rot(np.array([0, tilt[1], 0]))
    return Transform.from_rotation_translation(_rz(np.array([angle_deg + c]))[0], np.zeros(3)) @ \
        Transform.from_rotation_translation(M, np.array([off[0], off[1], 0.0]))


def hand_in_plate(rig: RigConfig) -> Transform:
    """URDF base frame (mm) in the plate frame. reference.hand_mount is the hand on the
    CAD connector seat; the hand sits on the same rods as the tower, so it turns with the
    tower's found mounting (object_in_plate)."""
    m = (rig.raw.get("reference") or {}).get("hand_mount") or {}
    return object_in_plate(rig) @ Transform.from_xyz_rpy(m.get("translation", [0, 0, 0]),
                                                         np.deg2rad(m.get("rotation", [0, 0, 0])))


def mounting(rig: RigConfig) -> dict | None:
    """How the object sits on the plate (calibration.yaml, from the photos), or None: the CAD seats."""
    return rig.load_calibration().get("mounting")


def object_in_plate(rig: RigConfig) -> Transform:
    """The tower's turn on the plate: 1b's solved yaw, else the found mounting, else the CAD."""
    cal = rig.load_calibration()
    yaw = float((cal.get("tower") or {}).get("yaw", (mounting(rig) or {}).get("yaw", 0.0)))
    return Transform.from_rotation_translation(_rz(np.array([yaw]))[0], np.zeros(3))


def overlay(rig: RigConfig, angle_deg: float, display_rotation: float = 90.0) -> dict:
    lens, cam, source = camera_model(rig, display_rotation)
    plate = plate_in_rig(rig, angle_deg)
    cam_from_plate = cam.inverse() @ plate
    cam_from_object = cam_from_plate @ object_in_plate(rig)
    obj = CalibrationObject.from_config(rig.raw["calibration_object"])
    solved = tile_poses_from_calibration(rig.load_calibration())
    s, eye = obj.spec, cam_from_object.inverse().translation

    def px(points_object):
        p = cam_from_object.apply_to_points(np.atleast_2d(points_object))
        return None if (p[:, 2] <= 1.0).any() else lens.project(p).round(1).tolist()

    w, h = s.size[0], s.size[1]
    n = s.squares[0] * s.square_size
    lines = [((s.margin + k * s.square_size, s.margin), (s.margin + k * s.square_size, s.margin + n)) for k in range(s.squares[0] + 1)] + \
            [((s.margin, s.margin + k * s.square_size), (s.margin + n, s.margin + k * s.square_size)) for k in range(s.squares[1] + 1)]
    tiles = []
    for name, tile in obj.tiles.items():
        pose = solved.get(name, tile.pose)
        to3 = lambda q: pose.apply_to_points(np.array([[x, y, 0.0] for x, y in q]))
        facing = float(-pose.rotation[:, 2] @ (eye - pose.apply_to_points(np.array([[w / 2, h / 2, 0.0]]))[0])) > 0
        outline = px(to3([(0, 0), (w, 0), (w, h), (0, h)]))
        if outline is None:
            continue
        tiles.append({"tile": name, "facing": facing, "outline": outline,
                      "grid": [px(to3(seg)) for seg in lines]})
    return {"angle": angle_deg, "mounting": mounting(rig),
            "source": {**source, "tiles": "calibrated" if solved else "cad",
                       "mounting": "photos" if mounting(rig) else "cad",
                                          "axis": "calibrated" if rig.load_calibration().get("stop_angle_corrections") else "nominal"},
            "lens": {"fx": lens.fx, "fy": lens.fy, "cx": lens.cx, "cy": lens.cy, "size": list(lens.size),
                     "distortion": list(lens.distortion)},
            "camera_from_plate": cam_from_plate.matrix.round(6).tolist(),
            "plate_from_hand": hand_in_plate(rig).matrix.round(6).tolist(),
            "tiles": tiles}
