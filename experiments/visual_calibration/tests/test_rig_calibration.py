"""Step 1 of visual calibration on synthetic photos: known lens, camera, axis,
stop and tile errors in, the fit must get them back."""
import numpy as np
import pytest

pytest.importorskip("cv2")

from orca_core.kinematics.transforms import Transform

from visual_calibration.calibration_object import CalibrationObject
from visual_calibration.camera import Lens, look_at
from visual_calibration.detect import TileDetection
from visual_calibration.mounting import coverage, mounting_from_tiles
from visual_calibration.rig_calibration import Photo, _rot, _rz, calibrate_rig
from visual_calibration.rig_config import RigConfig

SIZE = (3840, 2160)
LENS = Lens(3480.0, 3475.0, 1935.0, 1072.0, SIZE, (0.06, -0.12, 0.0004, -0.0003, 0.0))
TILT, OFFSET = np.array([0.3, -0.2]), np.array([0.4, -0.3])


def synthetic(rng, noise_px=0.2, stops=tuple(15.0 * k for k in range(22)), yaw=0.0, direction=1,
              tilt=TILT, offset=OFFSET, lift=0.0, scales=None):
    """Photos of the object turned `yaw` on the plate (0: the CAD seats, side 0 to the camera at 180°);
    direction -1: the plate turns the other way than the stop angles say; tilt/offset: the tower
    on the axis; lift: the tower sitting that much higher than the CAD."""
    obj = CalibrationObject.from_config(RigConfig.load().raw["calibration_object"])
    eye = np.array([240.02, 0.0, 259.41])                          # the CAD mount: 260 mm, 3.12° down
    cam = look_at(eye, [0.0, 0.0, 246.33], up=(0.0, 1.0, 0.0))     # on its side: the tile column centred
    true_tiles = {n: t.pose if n == obj.reference else
                  t.pose @ Transform.from_rotation_translation(_rot(rng.normal(0, 0.3, 3)), rng.normal(0, 0.3, 3))
                  for n, t in obj.tiles.items()}
    corr = {s: (0.0 if s == 0 else rng.normal(0, 0.15)) for s in stops}
    M = _rot(np.array([tilt[0], 0, 0])) @ _rot(np.array([0, tilt[1], 0])) @ _rz(np.array([yaw]))[0]
    shift = np.array([*offset, 0.0]) + M @ [0.0, 0.0, lift]
    photos = []
    for s in stops:
        Rz = _rz(np.array([direction * s + corr[s]]))[0]
        dets = []
        for n, pose in true_tiles.items():
            normal = Rz @ M @ -pose.rotation[:, 2]
            centre = Rz @ (M @ pose.apply_to_points(obj.spec.corners).mean(0) + shift)
            if normal @ (eye - centre) / np.linalg.norm(eye - centre) < np.cos(np.deg2rad(65)):
                continue
            P = obj.spec.corners
            if scales and n in scales:
                c = P.mean(0)
                P = c + (P - c) * scales[n]
            X = (Rz @ (pose.apply_to_points(P) @ M.T + shift).T).T
            uv = LENS.project(cam.inverse().apply_to_points(X))
            ok = np.all((uv > 20) & (uv < np.array(SIZE) - 20), axis=1)
            if ok.sum() >= 6:
                ids = np.flatnonzero(ok)
                dets.append(TileDetection(n, ids, uv[ok] + rng.normal(0, noise_px, (len(ids), 2)), 12))
        photos.append(Photo(s, dets, f"stop_{s:g}"))
    return obj, photos, cam, corr, true_tiles


def test_rig_calibration_recovers_the_rig():
    rng = np.random.default_rng(3)
    obj, photos, cam, corr, true_tiles = synthetic(rng)
    r = calibrate_rig(obj, photos, SIZE)

    assert r.rms_px < 0.35
    assert abs(r.lens.fx / LENS.fx - 1) < 2e-3
    assert np.linalg.norm(r.camera.translation - cam.translation) < 0.3
    assert np.allclose(r.axis_tilt, TILT, atol=0.05) and np.allclose(r.axis_offset, OFFSET, atol=0.15)
    assert max(abs(r.stop_corrections[s] - c) for s, c in corr.items()) < 0.08
    for n, t in r.tile_poses.items():
        assert np.linalg.norm(t.translation - true_tiles[n].translation) < 0.2, n

    # What the hand fit needs: points where the hand will be land where they should.
    # (The principal point and tangential distortion trade off a little; that is fine.)
    q = np.random.default_rng(0)
    a, rad = q.uniform(0, 2 * np.pi, 2000), q.uniform(0, 80, 2000)
    X = np.column_stack([rad * np.cos(a), rad * np.sin(a), q.uniform(130, 360, 2000)])
    true = LENS.project(cam.inverse().apply_to_points(X))
    seen = np.all((true > 0) & (true < SIZE), axis=1)
    err = np.linalg.norm(r.lens.project(r.camera.inverse().apply_to_points(X)) - true, axis=1)[seen]
    assert err.mean() < 0.5 and err.max() < 2.0


def test_tilted_tower_at_90_is_solved():
    """The first rig: the tower bolted on at 90°, 0.4° past it, tilted ~1.7° and 7 mm off the axis."""
    tilt, offset = np.array([1.6, -0.6]), np.array([6.0, 3.5])
    obj, photos, cam, corr, true_tiles = synthetic(np.random.default_rng(11), yaw=90.4, tilt=tilt, offset=offset)
    r = calibrate_rig(obj, photos, SIZE)
    assert r.mounting.yaw == 90.0 and not r.mounting.warnings()
    assert r.rms_px < 0.35 and np.linalg.norm(r.camera.translation - cam.translation) < 0.3
    assert np.allclose(r.axis_tilt, tilt, atol=0.05) and np.allclose(r.axis_offset, offset, atol=0.2)
    assert abs(r.tower_yaw - 90.4) < 0.05                        # the camera is at its CAD azimuth here
    t = r.to_calibration()["tower"]
    assert abs(t["yaw_from_mounting"] - 0.4) < 0.05 and "held" in t["height"]

    from visual_calibration.checks import solve_warnings
    rig = RigConfig.load()
    w = {x["code"]: x for x in solve_warnings(rig, "rotation", r.to_calibration(), photos)}
    assert w["tower_tilt"]["severity"] == "warning" and abs(w["tower_tilt"]["values"]["total_deg"] - 1.71) < 0.05
    assert w["tower_offset"]["values"]["total_mm"] > 6.5 and w["mounting_found"]["values"]["side0_stop_deg"] == 90.0
    assert {"code", "severity", "message", "fix", "values"} <= set(w["tower_tilt"])
    assert "residual_high" not in w and "stop_coverage" not in w
    off = r.to_calibration()
    off["camera"] = {**off["camera"], "translation": [v + d for v, d in zip(off["camera"]["translation"], [0, 15.0, -6.0])]}
    off["fit"] = {**off["fit"], "reprojection_px": 2.5}
    w = {x["code"]: x for x in solve_warnings(rig, "rotation", off, photos[:5])}
    assert w["camera_off_cad"]["values"]["mm"] > 15 and w["residual_high"]["values"]["rms_px"] == 2.5
    assert w["stop_coverage"]["values"]["missing_sides"]


def test_tower_height_goes_into_the_camera():
    obj, photos, cam, *_ = synthetic(np.random.default_rng(12), lift=8.0)
    r = calibrate_rig(obj, photos, SIZE)
    assert r.rms_px < 0.35                                       # not seen in the pixels ...
    assert abs(r.camera.translation[2] - (cam.translation[2] - 8.0)) < 0.3   # ... the camera comes out 8 mm low


def test_hand_turned_stops_are_found():
    rng = np.random.default_rng(4)
    obj, photos, *_ = synthetic(rng, stops=tuple(15.0 * k for k in range(22)))
    off = {p.stop_deg: (0.0 if p.stop_deg == 0 else rng.uniform(-4, 4)) for p in photos}
    nominal = [Photo(p.stop_deg + off[p.stop_deg], p.detections, p.name) for p in photos]
    r = calibrate_rig(obj, nominal, SIZE, hand_turned=True)
    assert r.rms_px < 0.35
    assert max(abs(r.stop_corrections[s + off[s]] + off[s]) for s in off) < 0.5   # it undoes the typed angles


@pytest.mark.parametrize("yaw, side0", [(0.0, 180.0), (180.0, 0.0), (90.0, 90.0)])
def test_tower_any_way_round_on_the_connector(yaw, side0):
    obj, photos, cam, corr, true_tiles = synthetic(np.random.default_rng(5), yaw=yaw)
    m = mounting_from_tiles(obj, photos)
    assert m.snapped and m.yaw == yaw and m.side0_stop == side0 and m.direction == 1 and not m.warnings()
    r = calibrate_rig(obj, photos, SIZE)
    assert r.mounting.yaw == yaw and r.mounting.source == "camera" and r.to_calibration()["mounting"]["side0_stop"] == side0
    assert r.rms_px < 0.35 and np.linalg.norm(r.camera.translation - cam.translation) < 0.3
    assert np.allclose(r.axis_tilt, TILT, atol=0.05) and np.allclose(r.axis_offset, OFFSET, atol=0.15)
    assert max(abs(r.stop_corrections[s] - c) for s, c in corr.items()) < 0.08
    for n, t in r.tile_poses.items():                                  # tiles stay in the object frame
        assert np.linalg.norm(t.translation - true_tiles[n].translation) < 0.2, n


def test_any_mounting_angle():
    obj, photos, cam, *_ = synthetic(np.random.default_rng(9), yaw=45.0)
    m = mounting_from_tiles(obj, photos)
    assert not m.snapped and abs(m.yaw - 45.0) < 10 and "not at one of the connector's mountings" in m.warnings()[0]
    r = calibrate_rig(obj, photos, SIZE)
    assert abs(r.mounting.yaw - 45.0) < 0.5 and abs(r.mounting.side0_stop - 135.0) < 0.5
    assert r.rms_px < 0.5 and np.linalg.norm(r.camera.translation - cam.translation) < 3.0


def test_plate_turning_the_other_way_is_caught():
    obj, photos, *_ = synthetic(np.random.default_rng(10), direction=-1)
    m = mounting_from_tiles(obj, photos)
    assert m.direction == -1 and m.direction_known and "gear_ratio to -1.0" in m.warnings()[0]
    with pytest.raises(ValueError, match="gear_ratio to -1.0"):
        calibrate_rig(obj, photos, SIZE)


def test_stops_show_every_side_for_any_mounting():
    obj = CalibrationObject.from_config(RigConfig.load().raw["calibration_object"])
    c = coverage(obj, RigConfig.load().stops.angles)
    assert c["mountings"] == {0.0: [], 90.0: [], 180.0: [], 270.0: []} and c["any"]["ok"] and not c["warnings"]
    few = coverage(obj, [0.0, 15.0, 30.0], max_view_deg=40.0)
    assert few["mountings"][0.0] == [0, "1 or 3"] and few["warnings"]


def test_sessions_camera_then_rotation_then_apply(tmp_path, monkeypatch):
    from visual_calibration.rig_config import RigConfig
    from visual_calibration.session import Session

    _, photos, cam, *_ = synthetic(np.random.default_rng(6), stops=tuple(30.0 * k for k in range(11)))
    monkeypatch.setattr(Session, "photos", lambda self: photos)
    rotation = Session.create(tmp_path, "rotation")
    rotation.meta["image_size"] = list(SIZE)
    with pytest.raises(ValueError, match="camera step first"):
        rotation.solve()
    camera = Session.create(tmp_path / "x", "camera")          # another folder: the same second as `rotation`
    camera.meta["image_size"] = list(SIZE)
    camera.solve()
    assert camera.result()["camera_fit"]["reprojection_px"] < 0.35
    assert camera.result()["mounting"]["side0_stop"] == 180.0 and {w["severity"] for w in camera.result()["warnings"]} <= {"info"}
    (tmp_path / "x" / camera.id).rename(tmp_path / camera.id)
    rotation.solve()
    r = rotation.result()
    assert r["fit"]["reprojection_px"] < 0.35 and r["camera_session"] == camera.id
    assert r["mounting"]["yaw"] == 0.0 and r["mounting"]["source"] == "camera"
    assert np.linalg.norm(np.array(r["camera"]["translation"]) - cam.translation) < 0.3

    saved = {}
    monkeypatch.setattr(RigConfig, "save_calibration", lambda self, c, keep_history=True: saved.update(c))
    camera = Session(tmp_path / camera.id)
    camera.apply()
    assert saved["lens"] == camera.result()["lens"] and saved["camera_session"] == camera.id


def test_camera_then_rotation():
    from visual_calibration.rig_calibration import calibrate_camera

    rng = np.random.default_rng(7)
    obj, photos, cam, corr, true_tiles = synthetic(rng)
    c = calibrate_camera(obj, photos, SIZE)
    assert c.rms_px < 0.35 and abs(c.lens.fx / LENS.fx - 1) < 2e-3
    for n, t in c.tile_poses.items():
        assert np.linalg.norm(t.translation - true_tiles[n].translation) < 0.2, n

    r = calibrate_rig(obj, photos, SIZE, lens=c.lens, fixed_lens=True, tile_poses=c.tile_poses)
    assert r.rms_px < 0.35 and r.lens == c.lens
    assert np.linalg.norm(r.camera.translation - cam.translation) < 0.3
    assert np.allclose(r.axis_tilt, TILT, atol=0.05) and np.allclose(r.axis_offset, OFFSET, atol=0.15)
    assert max(abs(r.stop_corrections[s] - v) for s, v in corr.items()) < 0.08


def test_camera_prior_from_the_cad():
    obj, photos, cam, *_ = synthetic(np.random.default_rng(8))
    near = {"position": (cam.translation + [3.0, -2.0, 4.0]).tolist(), "look_at": [0.0, 0.0, 245.0], "sigma_mm": 10.0}
    r = calibrate_rig(obj, photos, SIZE, camera_prior=near)
    c, t = r.camera.translation, cam.translation                         # a weak prior does not drag it,
    assert abs(np.hypot(*c[:2]) - np.hypot(*t[:2])) < 0.3 and abs(c[2] - t[2]) < 0.3
    assert abs(np.arctan2(c[1], c[0]) - np.arctan2(*near["position"][1::-1])) < 1e-6   # but sets the yaw gauge
    with pytest.raises(ValueError, match="from where the rig config puts it"):
        calibrate_rig(obj, photos, SIZE, camera_prior={**near, "position": (cam.translation + [0, 0, 60.0]).tolist()})


def test_held_out_photos():
    from visual_calibration.rig_calibration import calibrate_camera, held_out_rms
    obj, photos, *_ = synthetic(np.random.default_rng(13), stops=tuple(15.0 * k for k in range(22)))
    c = calibrate_camera(obj, photos[1::2], SIZE)
    assert all(held_out_rms(obj, c.lens, c.tile_poses, p) < 0.4 for p in photos[::2] if p.detections)


def test_read_back_angles_instead_of_the_stops(tmp_path, monkeypatch):
    """The plate stops a little off each nominal stop; the solves take the motor's read-back angle."""
    from visual_calibration.session import Session
    rng = np.random.default_rng(14)
    stops = tuple(15.0 * k for k in range(22))
    read = {s: s + (0.0 if s == 0 else rng.uniform(-0.4, 0.4)) for s in stops}
    obj, true_photos, cam, corr, _ = synthetic(rng, stops=tuple(read.values()))
    nominal = [Photo(s, p.detections, f"stop_{s:07.2f}") for s, p in zip(stops, true_photos)]
    s = Session.create(tmp_path, "camera")
    s.meta["photos"] = [{"name": p.name, "stop_deg": p.stop_deg, "plate_deg": read[p.stop_deg]} for p in nominal]
    monkeypatch.setattr(Session, "detections", lambda self, name: next(p.detections for p in nominal if p.name == name))
    assert [p.stop_deg for p in s.photos()] == list(read.values())
    r = calibrate_rig(obj, s.photos(), SIZE)
    assert r.rms_px < 0.35 and max(abs(r.stop_corrections[a] - corr[a]) for a in read.values()) < 0.08
    rn = calibrate_rig(obj, nominal, SIZE)                       # the nominal stops: corrections take it up
    assert max(abs(v) for v in rn.stop_corrections.values()) > 0.3


def test_tile_print_scale():
    from visual_calibration.rig_calibration import DEFAULT_SEARCH, calibrate_camera, held_out_rms
    obj, photos, *_ = synthetic(np.random.default_rng(5), noise_px=0.1, scales={"tile_r0_s0": 1.006, "tile_r2_s2": 0.995})
    plain = calibrate_camera(obj, photos[1::2], SIZE)
    search = {"tile": {**DEFAULT_SEARCH["tile"], "scale": {"bound": 0.02, "sigma": 0.01}}}
    c = calibrate_camera(obj, photos[1::2], SIZE, search=search)
    assert c.tile_scales["tile_r0_s0"] == pytest.approx(1.006, abs=6e-4)
    assert c.tile_scales["tile_r2_s2"] == pytest.approx(0.995, abs=6e-4)
    assert c.tile_scales[obj.reference] == 1.0 and c.rms_px < plain.rms_px
    held = [held_out_rms(obj, c.lens, c.tile_poses, p, c.tile_scales) for p in photos[::2] if p.detections]
    assert max(held) < 0.3
