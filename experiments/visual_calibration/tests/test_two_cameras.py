"""Two cameras on synthetic photos of the two-camera tower (v2 seats): cam A level at the 260
hole, cam B on the side rail 45° down; known lenses, poses, axis, stops and tiles in, the
joint fit must get both cameras back."""
import shutil

import numpy as np
import pytest
import yaml

pytest.importorskip("cv2")

from orca_core.kinematics.transforms import Transform

from visual_calibration.calibration_object import CalibrationObject
from visual_calibration.camera import Lens, look_at
from visual_calibration.detect import TileDetection
from visual_calibration.rig_calibration import Photo, _rot, _rz, calibrate_camera, calibrate_rig, camera_block
from visual_calibration.rig_config import RigConfig, get_rig_path

SIZE = (3840, 2160)
LENSES = {"a": Lens(2284.0, 2283.9, 1910.0, 1033.0, SIZE, (0.03, -0.05, 0.0002, 0.0004, 0.01)),
          "b": Lens(2290.0, 2291.0, 1925.0, 1041.0, SIZE, (0.04, -0.07, -0.0003, 0.0002, 0.02))}
EYES = {"a": ([268.0, 0.0, 249.0], [0.0, 0.0, 249.0], (0.0, 1.0, 0.0)),        # level, on its side
        "b": ([0.0, 220.0, 470.0], [0.0, 0.0, 250.0], (1.0, 0.0, 0.0))}        # side rail, 45° down, portrait
TILT, OFFSET = np.array([0.3, -0.2]), np.array([0.4, -0.3])


def tower_v2() -> CalibrationObject:
    raw = RigConfig.load().raw
    return CalibrationObject.from_config({**raw["calibration_object"], **raw["towers"]["v2"]})


def cameras() -> dict[str, Transform]:
    return {c: look_at(e, t, up=u) for c, (e, t, u) in EYES.items()}


def synthetic(rng, noise_px=0.2, stops=tuple(22.5 * k for k in range(15)), cams=("a", "b")):
    obj = tower_v2()
    poses = cameras()
    true_tiles = {n: t.pose if n == obj.reference else
                  t.pose @ Transform.from_rotation_translation(_rot(rng.normal(0, 0.3, 3)), rng.normal(0, 0.3, 3))
                  for n, t in obj.tiles.items()}
    corr = {s: (0.0 if s == 0 else rng.normal(0, 0.15)) for s in stops}
    M = _rot(np.array([TILT[0], 0, 0])) @ _rot(np.array([0, TILT[1], 0]))
    shift = np.array([*OFFSET, 0.0])
    photos = []
    for s in stops:
        Rz = _rz(np.array([s + corr[s]]))[0]
        for c in cams:
            eye, dets = poses[c].translation, []
            for n, pose in true_tiles.items():
                normal = Rz @ M @ -pose.rotation[:, 2]
                centre = Rz @ (M @ pose.apply_to_points(obj.spec.corners).mean(0) + shift)
                if normal @ (eye - centre) / np.linalg.norm(eye - centre) < np.cos(np.deg2rad(65)):
                    continue
                X = (Rz @ (pose.apply_to_points(obj.spec.corners) @ M.T + shift).T).T
                uv = LENSES[c].project(poses[c].inverse().apply_to_points(X))
                ok = np.all((uv > 20) & (uv < np.array(SIZE) - 20), axis=1)
                if ok.sum() >= 6:
                    ids = np.flatnonzero(ok)
                    dets.append(TileDetection(n, ids, uv[ok] + rng.normal(0, noise_px, (len(ids), 2)), 12))
            photos.append(Photo(s, dets, f"stop_{s:07.2f}" + ("" if c == "a" else f"_{c}"), c))
    return obj, photos, poses, corr, true_tiles


OFF = {"a": [3.0, 0.0, 4.0], "b": [2.0, -3.0, 4.0]}           # a's on its azimuth: it sets the gauge


def priors():
    return {c: {"position": (np.array(e) + OFF[c]).tolist(), "look_at": t, "sigma_mm": 10.0} for c, (e, t, _) in EYES.items()}


def deg(Ra, Rb) -> float:
    """The angle between two rotations."""
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1))))


def test_joint_fit_recovers_both_cameras():
    obj, photos, poses, corr, true_tiles = synthetic(np.random.default_rng(25))
    assert {p.camera for p in photos if p.detections} == {"a", "b"}
    r = calibrate_rig(obj, photos, SIZE, camera_priors=priors())
    assert list(r.cameras) == ["a", "b"] and r.camera is r.cameras["a"]
    assert r.rms_px < 0.35 and max(r.camera_rms.values()) < 0.35
    for c in "ab":
        assert abs(r.lenses[c].fx / LENSES[c].fx - 1) < 2e-3, c
        assert np.linalg.norm(r.cameras[c].translation - poses[c].translation) < 0.5, c
        assert deg(r.cameras[c].rotation, poses[c].rotation) < 0.3, c       # trades with the principal point
        # What the hand fit needs: points where the hand will be land where they should, in both cameras.
        q = np.random.default_rng(0)
        ang, rad = q.uniform(0, 2 * np.pi, 2000), q.uniform(0, 80, 2000)
        X = np.column_stack([rad * np.cos(ang), rad * np.sin(ang), q.uniform(130, 360, 2000)])
        true = LENSES[c].project(poses[c].inverse().apply_to_points(X))
        seen = np.all((true > 0) & (true < SIZE), axis=1)
        err = np.linalg.norm(r.lenses[c].project(r.cameras[c].inverse().apply_to_points(X)) - true, axis=1)[seen]
        assert err.mean() < 0.5 and err.max() < 2.0, c
    assert np.allclose(r.axis_tilt, TILT, atol=0.05) and np.allclose(r.axis_offset, OFFSET, atol=0.15)
    assert max(abs(r.stop_corrections[s] - v) for s, v in corr.items()) < 0.08
    for n, t in r.tile_poses.items():
        assert np.linalg.norm(t.translation - true_tiles[n].translation) < 0.25, n

    out = r.to_calibration()                         # camera a at the top level, b under cameras:
    assert "a" not in out["cameras"] and np.allclose(out["camera"]["translation"], r.cameras["a"].translation, atol=1e-3)
    assert np.allclose(camera_block(out, "b")["camera"]["translation"], r.cameras["b"].translation, atol=1e-3)
    assert set(out["cameras"]["b"]) == {"lens", "camera", "fit"}


def test_camera_step_then_rotation_with_two_cameras():
    obj, photos, poses, corr, _ = synthetic(np.random.default_rng(22))
    c = calibrate_camera(obj, photos, SIZE)
    assert c.rms_px < 0.35 and all(abs(c.lenses[k].fx / LENSES[k].fx - 1) < 2e-3 for k in "ab")
    r = calibrate_rig(obj, photos, SIZE, lenses=c.lenses, fixed_lens=True, tile_poses=c.tile_poses,
                      camera_priors=priors())
    assert r.lenses == c.lenses and r.rms_px < 0.35
    assert all(np.linalg.norm(r.cameras[k].translation - poses[k].translation) < 0.5 for k in "ab")


def test_one_camera_alone_is_the_old_solve():
    """Photos of camera a only: no cameras: block, the result as before cameras existed."""
    obj, photos, poses, *_ = synthetic(np.random.default_rng(23), cams=("a",))
    r = calibrate_rig(obj, photos, SIZE, camera_prior=priors()["a"])
    assert list(r.cameras) == ["a"] and "cameras" not in r.to_calibration()
    assert np.linalg.norm(r.camera.translation - poses["a"].translation) < 0.5


def make_rig(tmp_path, seed: int = 24):
    """The rig with tower v2, cam A at the 260 hole, cam B enabled and both cameras solved on synthetic photos."""
    path = tmp_path / "v1" / "camera-rig"
    shutil.copytree(get_rig_path(), path, ignore=shutil.ignore_patterns("camera_reference*.yaml"))
    cfg = yaml.safe_load((path / "config.yaml").read_text())
    cfg["tower"] = "v2"
    cfg["cameras"]["a"]["mount"]["distance"] = 260.0
    cfg["cameras"]["b"]["enabled"] = True
    (path / "config.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    obj, photos, poses, corr, _ = synthetic(np.random.default_rng(seed))
    r = calibrate_rig(obj, photos, SIZE, camera_priors=priors())
    rig = RigConfig.load(path=str(path))
    rig.save_calibration(r.to_calibration(), keep_history=False)
    return rig, obj, poses, r


@pytest.fixture
def two_camera_rig(tmp_path):
    return make_rig(tmp_path)


def test_rotation_session_with_lens_and_tiles_held(tmp_path, monkeypatch):
    """The re-check after a camera moved: 5 stops, lenses and tiles from calibration.yaml held."""
    from visual_calibration.session import Session
    rig, obj, poses, full = make_rig(tmp_path / "rig", seed=25)
    _, photos, *_ = synthetic(np.random.default_rng(25), stops=(0.0, 67.5, 135.0, 202.5, 270.0))   # the same tiles
    monkeypatch.setattr(Session, "photos", lambda self: photos)
    monkeypatch.setattr(Session, "rig", property(lambda self: rig))
    s = Session.create(tmp_path / "s", "rotation", hold=True)
    s.meta["image_size"] = list(SIZE)
    s.solve()
    out = s.result()
    assert out["held"] == ["lens", "tile_poses"] and "camera_session" not in out
    assert out["lens"]["fx"] == round(full.lenses["a"].fx, 3) and out["cameras"]["b"]["lens"]["fx"] == round(full.lenses["b"].fx, 3)
    assert out["fit"]["reprojection_px"] < 0.5
    assert np.linalg.norm(np.array(out["camera"]["translation"]) - poses["a"].translation) < 0.5
    assert np.linalg.norm(np.array(out["cameras"]["b"]["camera"]["translation"]) - poses["b"].translation) < 0.5


def test_load_frames_with_a_camera_field(tmp_path):
    import json

    import cv2

    from hand_study.estimate import frame, frame_key, key_name, load_frames
    views = [{"stop": 45.0, "angle": 45.4, "file": "v045.png"}, {"stop": 45.0, "angle": 45.4, "file": "v045_b.png", "camera": "b"},
             {"stop": 0.0, "angle": 0.1, "file": "v000.png"}]
    for v in views:
        cv2.imwrite(str(tmp_path / v["file"]), np.zeros((4, 4, 3), np.uint8))
    (tmp_path / "meta.json").write_text(json.dumps({"views": views}))
    f = load_frames(tmp_path)
    assert [len(x) for x in f] == [2, 2, 3] and f[2][2] == "b" and f[0][0] == 0.1
    assert [frame_key(x) for x in f] == [0.1, 45.4, ("b", 45.4)] and frame(f[0])[2] == "a"
    assert key_name(("b", 45.4)) == "b:45.4" and len(load_frames(tmp_path, cameras=["a"])) == 2


def test_hand_run_photographs_every_camera(tmp_path, monkeypatch):
    """The hand run through orca-ui's cameras: the preflight names them, each stop writes one view per
    camera, a still refused after a replug (409) re-runs the preflight once; the fit reads both."""
    import cv2

    from hand_study import hw
    from hand_study.collect import photograph
    from hand_study.estimate import load_frames
    monkeypatch.delenv("STUDY_FAKE_CAMERA", raising=False)
    calls, refuse = [], [1]

    def jpeg(cam):
        return cv2.imencode(".jpg", np.full((6, 8, 3), 50 if cam == "a" else 200, np.uint8))[1].tobytes()

    def call(path, body=None, timeout=120.0):
        calls.append(path)
        if path == "/visual-calibration/cameras/preflight":
            assert body == {"tiles": False}
            return {"indices": {"a": 1, "b": 0}, "identified_by": "serial"}
        assert path.startswith("/visual-calibration/cameras/stills?format=jpg") and body is None
        if refuse[0]:
            refuse[0] -= 1
            raise hw.ApiError(f'{path}: 409 {{"detail": "the cameras are not checked: run the camera preflight"}}')
        return {"cameras": {c: __import__("base64").b64encode(jpeg(c)).decode() for c in "ab"}}

    def fetch(path, timeout=60.0):
        raise hw.ApiError(f'{path}: 409 {{"detail": "the plate is turning"}}')       # not about the preflight

    monkeypatch.setattr(hw, "call", call)
    monkeypatch.setattr(hw, "fetch", fetch)
    cam = hw.Camera()
    views = photograph(cam, tmp_path, 45.0, 45.3) + photograph(cam, tmp_path, 90.0, 89.8)
    assert [v["file"] for v in views] == ["v045.jpg", "v045_b.jpg", "v090.jpg", "v090_b.jpg"]
    assert calls.count("/visual-calibration/cameras/preflight") == 2       # once more after the 409
    with pytest.raises(hw.ApiError, match="plate is turning"):
        cam.grab("b")                                                       # another 409: no new preflight
    assert calls.count("/visual-calibration/cameras/preflight") == 2
    (tmp_path / "meta.json").write_text(__import__("json").dumps({"views": views}))
    f = load_frames(tmp_path)
    assert [x[2] if len(x) > 2 else "a" for x in f] == ["a", "b", "a", "b"] and f[1][1].mean() > 150


def test_a_refused_camera_preflight_fails_the_hand_run(tmp_path, monkeypatch):
    """The camera preflight is part of the run: refused, the run is marked failed with the reason,
    nothing moved, the stream closed."""
    import json

    from hand_study import collect, hw
    events = []

    class Stream:
        def close(self):
            events.append("stream closed")

    class Hand:
        def __init__(self, stream):
            pass

        def torque(self, on):
            events.append(f"torque {on}")

    class Plate:
        def off(self):
            events.append("plate off")

    def refused():
        raise hw.ApiError('/visual-calibration/cameras/preflight: 409 {"detail": "camera b is not plugged in"}')

    monkeypatch.setattr(collect, "Stream", Stream)
    monkeypatch.setattr(collect, "Hand", Hand)
    monkeypatch.setattr(collect, "Turntable", Plate)
    monkeypatch.setattr(collect, "Camera", refused)
    (tmp_path / "plan.json").write_text(json.dumps({"angles": [0.0], "configs": [{"id": "c0", "q": {}}], "calibration": "x"}))
    with pytest.raises(hw.ApiError):
        collect.run(tmp_path, log=lambda *a: None)
    p = json.loads((tmp_path / "progress.json").read_text())
    assert p["state"] == "error" and "not plugged in" in p["error"]
    assert events == ["torque False", "plate off", "stream closed"]


def test_overlay_and_quick_check_per_camera(two_camera_rig):
    from visual_calibration.checks import quick_check, solve_warnings
    from visual_calibration.overlay import camera_model, object_in_plate, plate_in_rig

    rig, obj, poses, r = two_camera_rig
    assert rig.active_cameras == ["a", "b"] and len(obj.tiles) == 12
    for c in "ab":
        lens, pose, source = camera_model(rig, cam=c)
        assert source == {"lens": "calibrated", "camera": "calibrated"} and lens.fx == pytest.approx(r.lenses[c].fx, abs=1e-3)
        assert np.linalg.norm(pose.translation - poses[c].translation) < 0.5

    # One park-stop frame per camera: as calibrated -> ok; camera b knocked 1° -> advice.
    angle = 90.0
    T = plate_in_rig(rig, angle) @ object_in_plate(rig)

    def frames(knock=0.0):
        out = {}
        for c in "ab":
            lens, pose, _ = camera_model(rig, cam=c)
            if c == "b":
                pose = Transform.from_rotation_translation(pose.rotation @ _rot(np.array([knock, 0, 0])), pose.translation)
            dets = []
            for n, tile in r.tile_poses.items():
                to_cam = pose.inverse() @ T
                p = to_cam.apply_to_points(tile.apply_to_points(obj.spec.corners))
                out_normal = to_cam.rotation @ -tile.rotation[:, 2]
                if (p[:, 2] < 1).any() or out_normal @ -p.mean(0) / np.linalg.norm(p.mean(0)) < np.cos(np.deg2rad(65)):
                    continue
                uv = lens.project(p)
                ok = np.all((uv > 20) & (uv < np.array(SIZE) - 20), axis=1)
                if ok.sum() >= 6:
                    dets.append(TileDetection(n, np.flatnonzero(ok), uv[ok], 12))
            out[c] = dets
        return out

    good = quick_check(rig, frames(), angle)
    assert good["ok"] and all(v["px"] < 0.01 for v in good["cameras"].values()) and good["advice"] is None
    bad = quick_check(rig, frames(1.0), angle)
    assert not bad["ok"] and bad["cameras"]["b"]["px"] > 1.0 and bad["cameras"]["a"]["ok"]
    assert "5-stop sweep" in bad["advice"] and "camera b" in bad["advice"]

    w = solve_warnings(rig, "rotation", r.to_calibration(), [])
    assert not [x for x in w if x["code"] == "camera_off_cad"]       # both where their mounts put them
    moved = r.to_calibration()
    moved["cameras"]["b"]["camera"]["translation"][2] += 15.0
    w = [x for x in solve_warnings(rig, "rotation", moved, []) if x["code"] == "camera_off_cad"]
    assert len(w) == 1 and w[0]["values"]["camera"] == "b" and "cameras.b.mount" in w[0]["fix"]


def test_old_single_camera_config_loads_as_camera_a(tmp_path):
    path = tmp_path / "v1" / "camera-rig"
    shutil.copytree(get_rig_path(), path)
    cfg = yaml.safe_load((path / "config.yaml").read_text())
    new = RigConfig.load(path=str(path))
    cfg["camera"] = cfg.pop("cameras")["a"]
    cfg.pop("tower"), cfg.pop("towers")
    (path / "config.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    old = RigConfig.load(path=str(path))
    assert list(old.cameras) == ["a"] and old.active_cameras == ["a"]
    assert old.camera_nominal == new.camera_nominal and old.raw["camera"] == new.raw["camera"]
    assert old.raw["calibration_object"] == cfg["calibration_object"]          # no tower: key, the v1 seats


def test_uvcc_selects_a_brio_by_serial():
    from visual_calibration.uvc import CameraSettingsError, Uvcc

    devs = [{"name": "Logitech BRIO", "vendor": 1133, "product": 2142, "serial": "AAA", "location": 0x01100000, "address": 3},
            {"name": "Logitech BRIO", "vendor": 1133, "product": 2142, "serial": "BBB", "location": 0x01200000, "address": 7}]
    calls = []

    class P:
        returncode, stdout, stderr = 0, "{}", ""

    run = lambda cmd, **kw: (calls.append(cmd), P())[1]
    Uvcc.from_config({"device": {"vendor": 1133, "product": 2142, "serial": "BBB"}}, run=run, devices=lambda: devs).export()
    assert calls[-1][-3:] == ["--address", "7", "export"]
    Uvcc.from_config({"device": {"vendor": 1133, "product": 2142, "location": "0x01100000"}}, run=run, devices=lambda: devs).export()
    assert calls[-1][-3:] == ["--address", "3", "export"]
    Uvcc.from_config({"device": {"vendor": 1133, "product": 2142}}, run=run, devices=lambda: devs).export()
    assert "--address" not in calls[-1]                               # one camera: as before
    with pytest.raises(CameraSettingsError, match="0 USB cameras"):
        Uvcc.from_config({"device": {"vendor": 1133, "product": 2142, "serial": "CCC"}}, run=run, devices=lambda: devs).export()
