"""The rig's camera service on fakes: two BRIOs told apart by serial (uvcc) while OpenCV numbers
the streams in its own order; the blink proves which stream is which; the tile check catches a
camera on the other mount; nothing is captured before a passing preflight."""
import shutil

import numpy as np
import pytest
import yaml

pytest.importorskip("cv2")

from visual_calibration import rig_cameras as rc
from visual_calibration.rig_cameras import CameraIdentityError, RigCameras, avfoundation_index
from visual_calibration.rig_config import RigConfig, get_rig_path
from visual_calibration.uvc import wanted

VENDOR, PRODUCT = 1133, 2142
LOCATION = {"SA": 0x14100000, "SB": 0x14200000}


class World:
    """USB (serial -> controls) and AVFoundation (index -> the device behind it)."""

    def __init__(self, rig, streams=("SA", "SB"), profiler=("SA", "SB"), plugged=("SA", "SB")):
        self.streams, self.profiler, self.plugged = list(streams), list(profiler), set(plugged)
        self.controls = {s: wanted(rig.cameras[c]["uvc"]) for c, s in (("a", "SA"), ("b", "SB"))}
        self.opened = []

    def devices(self):
        return [{"name": "BRIO", "vendor": VENDOR, "product": PRODUCT, "serial": s, "location": LOCATION[s], "address": i}
                for i, s in enumerate(sorted(self.plugged))]

    def av(self):
        return [{"name": "BRIO", "unique_id": f"0x{LOCATION[s]:x}{VENDOR:04x}{PRODUCT:04x}"} for s in self.profiler]

    def open(self, index, size):
        cam = FakeStream(self, index)
        self.opened.append(cam)
        return cam

    def uvcc(self, uvc):
        return FakeUvcc(self, uvc["device"]["serial"])


class FakeStream:
    def __init__(self, world, index):
        self.world, self.index, self.opens, self._stopped = world, index, 1, False

    def still(self, after=None, timeout=10.0):
        if self._stopped:
            raise RuntimeError("closed")
        serial = self.world.streams[self.index]
        exposure = self.world.controls.get(serial, {}).get("absolute_exposure_time", 100)
        return np.full((24, 32, 3), min(250, 1.2 * exposure) + 10 * self.index, np.uint8)

    def close(self):
        self._stopped = True


class FakeUvcc:
    def __init__(self, world, serial):
        self.world, self.serial = world, serial

    def export(self):
        return dict(self.world.controls[self.serial])

    def set(self, control, value):
        self.world.controls[self.serial][control] = value


@pytest.fixture
def rig(tmp_path):
    """The rig with camera B enabled and serials set, nothing calibrated."""
    path = tmp_path / "v1" / "camera-rig"
    shutil.copytree(get_rig_path(), path, ignore=shutil.ignore_patterns("camera_reference*.yaml"))
    (path / "calibration.yaml").write_text("calibrated_at: null\nhistory: []\n")
    cfg = yaml.safe_load((path / "config.yaml").read_text())
    cfg["cameras"]["b"]["enabled"] = True
    for c, s in (("a", "SA"), ("b", "SB")):
        cfg["cameras"][c]["uvc"]["device"]["serial"] = s
    (path / "config.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    return RigConfig.load(path=str(path))


def service(rig, world):
    return RigCameras(rig, open_camera=world.open, uvcc=world.uvcc, devices=world.devices, av_devices=world.av)


def test_serial_picks_the_index_and_the_blink_proves_it(rig):
    world = World(rig, streams=("SB", "SA"), profiler=("SB", "SA"))      # AVFoundation lists cam B first
    cams = service(rig, world)
    with pytest.raises(CameraIdentityError, match="preflight"):
        cams.stills()                                                    # nothing before the check
    r = cams.preflight()
    assert cams.indices == {"a": 1, "b": 0} and r["identified_by"] == "serial" and "reordered" not in r
    assert r["blink"]["a"]["a"] > 0.8 and r["blink"]["a"]["b"] == 0.0
    assert world.controls["SA"]["absolute_exposure_time"] == 100         # the blink put it back
    frames = cams.stills()
    assert set(frames) == {"a", "b"} and frames["a"][0, 0, 0] == 130 and frames["b"][0, 0, 0] == 120


def test_a_stream_order_unlike_the_profilers_is_found_by_the_blink(rig):
    world = World(rig, streams=("SB", "SA"), profiler=("SA", "SB"))      # OpenCV's order is not the profiler's
    cams = service(rig, world)
    r = cams.preflight()
    assert r["reordered"] == {"a": "b", "b": "a"} and cams.indices == {"a": 1, "b": 0}
    assert cams.stills()["a"][0, 0, 0] == 130                            # index 1: cam A's frames


def test_refuses_a_missing_or_silent_camera(rig):
    with pytest.raises(CameraIdentityError, match="camera b .*not plugged in"):
        service(rig, World(rig, plugged=("SA",))).preflight()
    world = World(rig, streams=("SA", "FACETIME"))                       # index 1 is another camera
    cams = service(rig, world)
    with pytest.raises(CameraIdentityError, match="camera b: dimming it .*no stream"):
        cams.preflight()
    with pytest.raises(CameraIdentityError):
        cams.stills()


def test_two_cameras_need_serials(rig):
    rig.raw["cameras"]["b"]["uvc"]["device"]["serial"] = None
    with pytest.raises(CameraIdentityError, match="no serial"):
        service(rig, World(rig)).preflight()


def test_a_reopened_capture_needs_a_new_preflight(rig):
    cams = service(rig, World(rig))
    cams.preflight()
    cams.stills()
    cams.cams["b"].opens += 1                                            # the reader reopened the device
    with pytest.raises(CameraIdentityError):
        cams.stills()
    cams.preflight()
    assert set(cams.stills()) == {"a", "b"}


def test_tile_check_catches_cameras_on_each_others_mounts(rig):
    from test_two_cameras import synthetic
    _, photos, *_ = synthetic(np.random.default_rng(3), stops=(90.0,))
    dets = {p.camera: p.detections for p in photos}

    class Detector:
        obj = rig_obj(rig)

        def detect(self, img):
            return dets["ab"[int(img[0, 0, 0])]]

    cams = service(rig, World(rig))
    frames = {"a": np.zeros((2, 2, 3), np.uint8), "b": np.ones((2, 2, 3), np.uint8)}
    r = cams.tile_check(Detector(), frames)
    assert abs(r["a"]["height_mm"] - 249) < 30 and abs(r["b"]["height_mm"] - 470) < 30
    with pytest.raises(CameraIdentityError, match="camera a is .*camera b's height"):
        cams.tile_check(Detector(), {"a": frames["b"], "b": frames["a"]})


def rig_obj(rig):
    from visual_calibration.calibration_object import CalibrationObject
    return CalibrationObject.from_config(rig.raw["calibration_object"])


def test_mock_and_preview():
    rig = RigConfig.load()
    cams = RigCameras(rig, mock=True)
    assert cams.preflight()["identified_by"] == "mock"
    img = cams.preview("a", max_side=480)
    assert max(img.shape[:2]) == 480 and set(cams.stills()) == set(rig.active_cameras)
    assert rc.encode(img, "jpg")[:2] == b"\xff\xd8" and rc.encode(img)[:4] == b"\x89PNG"


def test_avfoundation_index():
    dev = {"vendor": VENDOR, "product": PRODUCT, "location": LOCATION["SB"]}
    av = [{"unique_id": "0x1100000005ac8600"}, {"unique_id": f"0x{LOCATION['SB']:x}046D085E"}]
    assert avfoundation_index(dev, av) == 1 and avfoundation_index({**dev, "location": 1}, av) is None
