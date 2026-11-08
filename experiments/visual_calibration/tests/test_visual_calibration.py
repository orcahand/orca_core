import numpy as np
import pytest
from orca_core.kinematics.transforms import Transform

from visual_calibration.camera import Camera, Lens, look_at
from visual_calibration.hand_model import description_root


def test_projection_matches_pinhole():
    lens = Lens(1000.0, 1000.0, 500.0, 400.0, (1000, 800))
    cam = Camera(lens, look_at([0.0, -1.0, 0.0], [0.0, 0.0, 0.0]))
    uv = cam.project(np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.05]]))
    assert np.allclose(uv[0], [500.0, 400.0])
    assert np.allclose(uv[1], [600.0, 350.0])          # x right, z up -> image up
    assert np.allclose(Camera(lens, Transform.identity()).project(np.array([[0.0, 0.0, 2.0]])), [[500.0, 400.0]])


@pytest.mark.skipif(description_root() is None, reason="orcahand_description checkout not found")
def test_e0_recovers_index_angles():
    from visual_calibration import e0
    from visual_calibration.hand_model import HandModel

    model, rig = HandModel.load("right"), e0.nominal_rig()
    r = e0.trial_errors(rig, rig, model, e0.POSES["half"], e0.ALL3, [45.0 * k for k in range(8)],
                        e0.points(model, "tip+pip"), 0.5, 3, np.random.default_rng(1))
    assert max(r["max_deg"].values()) < 0.3
    assert r["wrong_over_2deg"] == 0


def _rendered_tile(first_marker=60, px_per_mm=10, origin=(300, 200)):
    """A white page with one tile's pattern (no border), and where its corners are."""
    cv2 = pytest.importorskip("cv2")
    from visual_calibration.calibration_object import CalibrationObject
    from visual_calibration.rig_config import RigConfig

    obj = CalibrationObject.from_config(RigConfig.load().raw["calibration_object"])
    s = obj.spec
    board = cv2.aruco.CharucoBoard(s.squares, s.square_size, s.marker_size,
                                   cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, s.dictionary)),
                                   np.arange(first_marker, first_marker + s.markers_per_tile))
    side = round(s.squares[0] * s.square_size * px_per_mm)
    page = np.full((1200, 1600), 255, np.uint8)
    page[origin[1]:origin[1] + side, origin[0]:origin[0] + side] = board.generateImage((side, side), marginSize=0)
    corners = (s.corners[:, :2] - s.margin) * px_per_mm + origin - 0.5     # pixel centres
    return obj, page, corners


def test_tile_detector_finds_a_rendered_tile():
    from visual_calibration.detect import TileDetector

    obj, page, corners = _rendered_tile()
    (d,) = TileDetector(obj).detect(page)
    assert d.tile == "tile_r1_s1" and len(d.ids) == len(corners) and d.markers == obj.spec.markers_per_tile
    assert np.abs(d.uv - corners[d.ids]).max() < 0.2


def test_session_stores_photos_and_detections(tmp_path):
    cv2 = pytest.importorskip("cv2")
    from visual_calibration.session import Session

    _, page, _ = _rendered_tile()
    s = Session.create(tmp_path, "camera")
    entry, dets = s.add_photo(cv2.imencode(".png", page)[1].tobytes(), 15.0)
    assert entry["tiles"] == 1 and entry["corners"] == 16 and len(dets) == 1
    s.add_photo(cv2.imencode(".png", page)[1].tobytes(), 15.0)          # retaken: replaces
    again = Session(s.root)
    assert [p.stop_deg for p in again.photos()] == [15.0] and again.meta["image_size"] == [1600, 1200]
    assert again.photos()[0].detections[0].tile == "tile_r1_s1"
    with pytest.raises(ValueError):
        again.apply()
