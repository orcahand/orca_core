"""The live overlay: before calibration from the CAD, after it from calibration.yaml."""
import numpy as np

from visual_calibration.overlay import camera_model, overlay
from visual_calibration.rig_config import RigConfig


def test_nominal_overlay_shows_side_0_upright_at_its_stop(uncalibrated):
    rig = uncalibrated
    o = overlay(rig, rig.raw["calibration_object"]["side0_stop"], display_rotation=90)
    assert o["source"]["camera"] == "nominal"
    facing = {t["tile"]: np.array(t["outline"]) for t in o["tiles"] if t["facing"]}
    assert {"tile_r0_s0", "tile_r1_s0", "tile_r2_s0"} <= set(facing)
    w, h = o["lens"]["size"]
    for tile in ("tile_r1_s0", "tile_r2_s0"):                                   # row 0 tilts up, for camera b
        assert (facing[tile] > 0).all() and (facing[tile] < [w, h]).all()      # in the frame (90° mode)
    # turned 90° clockwise for display, row 2 must be above row 0: raw x smaller (left = up)
    assert facing["tile_r2_s0"][:, 0].mean() < facing["tile_r0_s0"][:, 0].mean()


def test_calibrated_camera_replaces_the_cad(monkeypatch):
    rig = RigConfig.load()
    cal = {"lens": {"fx": 3000.0, "fy": 3000.0, "cx": 1920.0, "cy": 1080.0, "image_size": [3840, 2160],
                    "distortion": [0, 0, 0, 0, 0]},
           "camera": {"translation": [240.0, 0.0, 259.0], "rotation": [-90.0, 0.0, -90.0]}}
    monkeypatch.setattr(RigConfig, "load_calibration", lambda self: cal)
    lens, pose, source = camera_model(rig)
    assert source == {"lens": "calibrated", "camera": "calibrated"} and lens.fx == 3000.0
    assert np.allclose(pose.translation, [240.0, 0.0, 259.0])


def test_overlay_follows_the_found_mounting(monkeypatch):
    rig = RigConfig.load()
    monkeypatch.setattr(RigConfig, "load_calibration", lambda self: {"mounting": {"yaw": 180.0, "side0_stop": 0.0}})
    o = overlay(rig, 0.0, display_rotation=90)
    assert o["source"]["mounting"] == "photos"
    assert {"tile_r0_s0", "tile_r1_s0", "tile_r2_s0"} <= {t["tile"] for t in o["tiles"] if t["facing"]}


def test_hand_turns_with_the_found_mounting(uncalibrated, monkeypatch):
    """The hand sits on the tower's rods: found at 90°, the palm faces the camera at plate 180°."""
    from visual_calibration.overlay import hand_in_plate, plate_in_rig
    from orca_core.kinematics.transforms import Transform
    rig = uncalibrated
    monkeypatch.setattr(type(rig), "load_calibration", lambda self: {"mounting": {"yaw": 90.0, "side0_stop": 90.0}})
    cad = Transform.from_xyz_rpy([0, 0, 0], np.deg2rad(rig.raw["reference"]["hand_mount"]["rotation"])).rotation
    palm = cad.T @ [0.0, 1.0, 0.0]                                       # the CAD palm: plate +y on the CAD seat
    assert np.allclose((plate_in_rig(rig, 180.0) @ hand_in_plate(rig)).rotation @ palm, [1, 0, 0], atol=1e-6)
