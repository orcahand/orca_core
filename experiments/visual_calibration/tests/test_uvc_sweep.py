"""The camera's UVC controls (a fake uvcc) and the sweep's camera check (mock turntable)."""
import json
import shutil
import subprocess

import numpy as np
import pytest
import yaml

from visual_calibration.rig_config import RigConfig, get_rig_path
from visual_calibration.uvc import CameraSettingsError, Uvcc, compare, ensure, frame_stats, wanted

cv2 = pytest.importorskip("cv2")


class FakeUvcc:
    """uvcc's CLI over a dict; `stuck` controls ignore writes (Logi Tune holding them)."""

    def __init__(self, state, stuck=()):
        self.state, self.stuck, self.sets = dict(state), set(stuck), []

    def __call__(self, cmd, **kw):
        args = cmd[cmd.index("--product") + 2:]
        out = ""
        if args[0] == "export":
            out = json.dumps(self.state)
        elif args[0] == "set":
            self.sets.append(args[1])
            if args[1] not in self.stuck:
                v = [int(x) for x in args[2:]]
                self.state[args[1]] = v if len(v) > 1 else v[0]
        return subprocess.CompletedProcess(cmd, 0, out, "")


def uvc_cfg():
    return RigConfig.load().raw["camera"]["uvc"]


def camera_as_found():
    return {**wanted(uvc_cfg()), "auto_focus": 1, "auto_white_balance_temperature": 1, "absolute_focus": 30,
            "white_balance_temperature": 4000, "gain": 12}


def test_ensure_sets_auto_flags_first_and_reads_back():
    fake = FakeUvcc(camera_as_found())
    r = ensure(Uvcc(1133, 2142, tool="uvcc", run=fake), uvc_cfg())
    assert fake.sets[:2] == ["auto_focus", "auto_white_balance_temperature"]
    assert set(fake.sets) == {"auto_focus", "auto_white_balance_temperature", "absolute_focus", "white_balance_temperature", "gain"}
    assert r["after"]["absolute_focus"] == uvc_cfg()["absolute_focus"] and r["before"]["gain"] == 12
    assert fake.sets.count("absolute_focus") == 2                      # from below: the lens has hysteresis


def test_ensure_aborts_when_a_control_does_not_take():
    fake = FakeUvcc(camera_as_found(), stuck={"absolute_focus"})
    with pytest.raises(CameraSettingsError, match="absolute_focus 30"):
        ensure(Uvcc(1133, 2142, tool="uvcc", run=fake), uvc_cfg())


def test_verify_only_never_writes():
    fake = FakeUvcc(camera_as_found())
    with pytest.raises(CameraSettingsError):
        ensure(Uvcc(1133, 2142, tool="uvcc", run=fake), uvc_cfg(), write=False)
    assert fake.sets == []


def test_values_an_auto_flag_overrides_are_not_held():
    w = wanted({"device": {}, "white_balance_temperature": 5000, "auto_white_balance_temperature": 1, "gain": 0})
    assert w == {"auto_white_balance_temperature": 1, "gain": 0}


def _page(blur=0, gain=1.0, zoom=1.0):
    from visual_calibration.calibration_object import CalibrationObject
    obj = CalibrationObject.from_config(RigConfig.load().raw["calibration_object"])
    s = obj.spec
    board = cv2.aruco.CharucoBoard(s.squares, s.square_size, s.marker_size,
                                   cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, s.dictionary)),
                                   np.arange(60, 60 + s.markers_per_tile))
    side = round(s.squares[0] * s.square_size * 10)
    page = np.full((1200, 1600), 255, np.uint8)
    page[200:200 + side, 300:300 + side] = board.generateImage((side, side), marginSize=0)
    page = (page.astype(float) * 0.75 + 20).astype(np.uint8)      # the white margins not clipped
    if zoom != 1.0:
        page = cv2.warpAffine(page, np.array([[zoom, 0, 0], [0, zoom, 0]], float), page.shape[::-1], borderValue=211)
    if blur:
        page = cv2.GaussianBlur(page, (0, 0), blur)
    return obj, np.clip(page * gain, 0, 255).astype(np.uint8)


def test_frame_check_catches_blur_brightness_and_focus_breathing():
    from visual_calibration.detect import TileDetector
    obj, page = _page()
    det = TileDetector(obj)
    ref = frame_stats(page, det.detect(page), obj)
    assert compare(ref, ref) == [] and ref["clipped"] == 0
    (t,) = ref["tiles"].values()
    assert t["scale"] == pytest.approx(10 * obj.spec.square_size, rel=0.01)
    for kw, what in (({"blur": 2.0}, "sharpness"), ({"gain": 0.6}, "brightness"), ({"zoom": 1.03}, "scale")):
        _, img = _page(**kw)
        assert any(what in p for p in compare(frame_stats(img, det.detect(img), obj), ref)), kw


@pytest.fixture
def rig(tmp_path):
    path = tmp_path / "v1" / "camera-rig"
    shutil.copytree(get_rig_path(), path, ignore=shutil.ignore_patterns("camera_reference.yaml"))
    return RigConfig.load(path=str(path))


def _sweep(rig, tmp_path, img, fake, **kw):
    from visual_calibration import sweep
    from visual_calibration.detect import TileDetector
    from visual_calibration.turntable import Turntable
    t = Turntable.open(rig.raw["turntable"], mock=True)
    t.home()
    t.move(90.0)
    obj, _ = _page()
    saved = []
    t.held = []

    def grab():
        t.held.append(t.torque)
        return img
    try:
        log = sweep.run(rig, t, grab, TileDetector(obj), tmp_path / "out", [0.0, 45.0, 90.0],
                        Uvcc(1133, 2142, tool="uvcc", run=fake), settle_s=0.0,
                        imwrite=lambda p, im: saved.append(p.name), **kw)
    except CameraSettingsError:
        return t, None, saved
    finally:
        t.released = not t.torque
        t.close()
    return t, log, saved


def test_sweep_checks_the_camera_then_takes_every_stop(rig, tmp_path):
    _, page = _page()
    fake = FakeUvcc(camera_as_found())
    t, log, saved = _sweep(rig, tmp_path, page, fake, save_ref=True)
    assert saved == ["stop_0000.00.png", "stop_0045.00.png", "stop_0090.00.png", "repeat_0000.00.png"]
    assert log["camera"]["uvc"]["set"]["absolute_focus"] == uvc_cfg()["absolute_focus"] and log["camera"]["changed"] == {}
    assert all(s["motion_px"] == 0 for s in log["stops"].values())
    assert yaml.safe_load((tmp_path / "out" / "sweep.yaml").read_text())["stops"]["stop_0045.00"]["angle"] == pytest.approx(45, abs=0.1)
    ref = yaml.safe_load(open(f"{rig.path}/camera_reference.yaml"))
    assert ref["stop"] == 90.0 and ref["uvc"]["absolute_focus"] == uvc_cfg()["absolute_focus"]
    assert t.held and all(t.held[1:]) and t.released          # held at every stop's photos, off at the end


def test_a_blurred_camera_stops_the_sweep_before_the_motor_moves(rig, tmp_path):
    _, page = _page()
    _sweep(rig, tmp_path, page, FakeUvcc(camera_as_found()), save_ref=True)
    _, blurred = _page(blur=2.0)
    fake = FakeUvcc(camera_as_found())
    moves = []
    from visual_calibration.turntable import Turntable
    orig = Turntable.move
    Turntable.move = lambda self, a, settle_s=0.0, **kw: (moves.append(a), orig(self, a, settle_s, **kw))[1]
    try:
        _, log, saved = _sweep(rig, tmp_path / "2", blurred, fake)
    finally:
        Turntable.move = orig
    assert log is None and saved == [] and moves == [90.0]          # only the test's own move to the parked stop


def test_hand_sweep_checks_the_controls_only(rig, tmp_path):
    empty = np.full((1200, 1600), 128, np.uint8)
    _, log, saved = _sweep(rig, tmp_path, empty, FakeUvcc(camera_as_found()), hand=True)
    assert len(saved) == 4 and "frame" not in log["camera"] and log["camera"]["uvc"]["after"]["gain"] == 0
    assert all(s["motion_px"] is None for s in log["stops"].values())


def test_two_camera_sweep_grabs_both_with_a_reference_each(rig, tmp_path):
    from visual_calibration import sweep
    from visual_calibration.detect import TileDetector
    from visual_calibration.turntable import Turntable
    _, page = _page()
    t = Turntable.open(rig.raw["turntable"], mock=True)
    t.home()
    t.move(90.0)
    obj, _ = _page()
    saved, grabbed = [], []
    fakes = {"a": FakeUvcc(camera_as_found()), "b": FakeUvcc(camera_as_found())}
    grab = lambda c: (lambda: (grabbed.append(c), page)[1])
    try:
        log = sweep.run(rig, t, {"a": grab("a"), "b": grab("b")}, TileDetector(obj), tmp_path / "out", [0.0, 45.0],
                        {c: Uvcc(1133, 2142, tool="uvcc", run=f) for c, f in fakes.items()}, settle_s=0.0,
                        imwrite=lambda p, im: saved.append(p.name), save_ref=True)
    finally:
        t.close()
    assert saved == ["stop_0000.00.png", "stop_0000.00_b.png", "stop_0045.00.png", "stop_0045.00_b.png",
                     "repeat_0000.00.png", "repeat_0000.00_b.png"]
    assert log["camera"]["uvc"]["after"]["gain"] == 0 and log["cameras"]["b"]["uvc"]["after"]["gain"] == 0
    assert log["stops"]["stop_0045.00_b"]["camera"] == "b" and log["stops"]["stop_0045.00_b"]["angle"] == log["stops"]["stop_0045.00"]["angle"]
    assert (tmp_path / "v1" / "camera-rig" / "camera_reference_b.yaml").exists() and sweep.reference_path(rig).exists()
    assert "b" in grabbed and fakes["b"].sets
