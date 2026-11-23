"""Display rotation from a frame, the calibration status, the twin state."""
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from visual_calibration.calibration_object import CalibrationObject
from visual_calibration.detect import TileDetector, display_rotation
from visual_calibration.rig_config import RigConfig


def tile_frame():
    """tile_r1_s0 upright on a page (its top is the frame's top)."""
    obj = CalibrationObject.from_config(RigConfig.load().raw["calibration_object"])
    t = obj.tiles["tile_r1_s0"]
    board = cv2.aruco.CharucoBoard((5, 5), 14.0, 10.5, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250),
                                   t.marker_ids(obj.spec))
    page = np.full((1000, 1400), 255, np.uint8)
    page[150:850, 350:1050] = board.generateImage((700, 700), marginSize=0)
    return obj, page


@pytest.mark.parametrize("k, rotation", [(0, 0), (1, 90), (2, 180), (3, 270)])
def test_display_rotation_puts_the_tower_up(k, rotation):
    """np.rot90 k times turns the frame counter-clockwise; turning it back is k·90° clockwise."""
    obj, page = tile_frame()
    det = TileDetector(obj).detect(np.ascontiguousarray(np.rot90(page, k)))
    assert len(det) == 1
    o = display_rotation(obj, det)
    assert o["rotation"] == rotation and o["agreement"] > 0.99
    assert display_rotation(obj, []) is None


def test_status_without_sessions(tmp_path, monkeypatch):
    from visual_calibration.status import calibration_status
    monkeypatch.setattr(RigConfig, "turntable_calibration_path", property(lambda self: str(tmp_path / "none.yaml")))
    s = calibration_status(RigConfig.load(), tmp_path)
    steps = {x["id"]: x for x in s["steps"]}
    assert list(steps) == ["motor", "camera", "rotation", "hand"]
    assert steps["motor"]["status"] == "missing" and steps["motor"]["warnings"][0]["code"] == "turntable_uncalibrated"
    assert steps["camera"]["status"] == "none"


def test_status_after_a_solve(tmp_path, monkeypatch):
    from test_rig_calibration import SIZE, synthetic
    from visual_calibration.session import Session
    from visual_calibration.status import calibration_status

    _, photos, *_ = synthetic(np.random.default_rng(6), stops=tuple(30.0 * k for k in range(11)))
    monkeypatch.setattr(Session, "photos", lambda self: photos)
    cam = Session.create(tmp_path, "camera")
    cam.meta["image_size"] = list(SIZE)
    cam.meta["photos"] = [{"name": p.name, "stop_deg": p.stop_deg, "corners": sum(len(d.ids) for d in p.detections)}
                          for p in photos]
    cam._save()
    cam.solve()
    step = {x["id"]: x for x in calibration_status(RigConfig.load(), tmp_path)["steps"]}["camera"]
    assert step["status"] == "solved" and step["numbers"]["frames"] == 11 and step["numbers"]["corners"] > 300
    assert step["numbers"]["residual_px"] < 0.35 and step["numbers"]["target_px"] == 1.0
    assert any(w["code"] == "mounting_found" for w in step["warnings"])


def test_twin_state_cad_and_solved(monkeypatch, uncalibrated):
    from visual_calibration.twin import twin_state
    rig = uncalibrated
    s = twin_state(rig, 90.0)
    assert s["groups"]["object_solved"] is None and s["camera"]["solved"] is None
    assert np.allclose(np.array(s["groups"]["rotor"])[:2, :2], [[0, -1], [1, 0]], atol=1e-6)
    cal = {"mounting": {"yaw": 90.0, "side0_stop": 90.0}, "axis": {"tilt": [1.6, -0.6], "offset": [6.0, 3.5]},
           "tower": {"tilt": [1.6, -0.6], "offset": [6.0, 3.5], "yaw": 90.4},
           "stop_angle_corrections": {0.0: 0.0, 90.0: 0.1},
           "camera": {"translation": [240.0, 0.0, 259.0], "rotation": [-90.0, 0.0, -90.0]}}
    monkeypatch.setattr(RigConfig, "load_calibration", lambda self: cal)
    s = twin_state(rig, 90.0)
    obj = np.array(s["groups"]["object_solved"])
    assert np.allclose(obj[:3, 3], [-3.5 * np.cos(np.radians(0.1)) - 6.0 * np.sin(np.radians(0.1)), 6.0, 0.0], atol=0.05)
    assert np.allclose(np.array(s["groups"]["object_cad"])[:2, :2], [[-1, 0], [0, -1]], atol=1e-6)   # 90 + 90
    ax = s["axis"]["solved"]
    assert abs(np.degrees(np.arccos(ax["direction_object"][2])) - np.hypot(1.6, 0.6)) < 0.01
    assert np.allclose(np.array(s["camera"]["solved"])[:3, 3], [240.0, 0.0, 259.0])
