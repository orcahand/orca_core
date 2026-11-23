"""The rig's camera service on fakes: two BRIOs told apart by serial (uvcc) while OpenCV numbers
the streams by AVFoundation unique id; the blink proves which stream is which; the tile check
catches a camera on the other mount; every preflight starts from config.yaml and the USB bus as
they are now; nothing is captured before a passing preflight."""
import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml

pytest.importorskip("cv2")

from visual_calibration import rig_cameras as rc
from visual_calibration.rig_cameras import CameraIdentityError, RigCameras, UnknownCamera, avfoundation_index
from visual_calibration.rig_config import RigConfig, get_rig_path
from visual_calibration.uvc import wanted

VENDOR, PRODUCT = 1133, 2142
FACETIME = "47B4B64B70674B9CAD2BAE273A71F4B5"          # the built-in camera: listed first, sorted last


class World:
    """USB (serial -> location, controls) and AVFoundation (system_profiler's list; OpenCV's streams by index)."""

    def __init__(self, rig, streams=("SA", "SB", "FACETIME"), plugged=("SA", "SB")):
        self.streams, self.plugged = list(streams), set(plugged)
        self.location = {"SA": 0x14100000, "SB": 0x14200000}
        self.extra: list[str] = []                     # other video devices' unique ids
        self.controls = {s: wanted(rig.cameras[c]["uvc"]) for c, s in (("a", "SA"), ("b", "SB"))}
        self.listings = 0

    def uid(self, serial):
        return f"0x{self.location[serial]:x}{VENDOR:04x}{PRODUCT:04x}"

    def devices(self):
        self.listings += 1
        return [{"name": "BRIO", "vendor": VENDOR, "product": PRODUCT, "serial": s, "location": self.location[s],
                 "address": i} for i, s in enumerate(sorted(self.plugged))]

    def av(self):
        return [{"name": "FaceTime HD Camera", "unique_id": FACETIME}, *({"unique_id": u} for u in self.extra),
                *({"name": "BRIO", "unique_id": self.uid(s)} for s in sorted(self.plugged, reverse=True))]

    def open(self, index, size):
        return FakeStream(self, index)

    def uvcc(self, uvc):
        return FakeUvcc(self, uvc["device"]["serial"])


class FakeStream:
    def __init__(self, world, index):
        self.world, self.index, self.opens, self._stopped = world, index, 1, False
        self.on_still = None

    def still(self, after=None, timeout=10.0):
        if self._stopped:
            raise RuntimeError("closed")
        if self.on_still:
            self.on_still()
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


def edit_config(rig, edit):
    path = Path(rig.path) / "config.yaml"
    cfg = yaml.safe_load(path.read_text())
    edit(cfg)
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))


@pytest.fixture
def rig(tmp_path):
    """The rig with both cameras enabled and their serials set, nothing calibrated."""
    path = tmp_path / "v1" / "camera-rig"
    shutil.copytree(get_rig_path(), path, ignore=shutil.ignore_patterns("camera_reference*.yaml"))
    (path / "calibration.yaml").write_text("calibrated_at: null\nhistory: []\n")
    rig = RigConfig.load(path=str(path))

    def edit(cfg):
        cfg["cameras"]["b"]["enabled"] = True
        for c, s in (("a", "SA"), ("b", "SB")):
            cfg["cameras"][c]["uvc"]["device"]["serial"] = s
    edit_config(rig, edit)
    return RigConfig.load(path=str(path))


def service(rig, world):
    return RigCameras(rig, open_camera=world.open, uvcc=world.uvcc, devices=world.devices, av_devices=world.av)


def test_serial_picks_the_index_and_the_blink_proves_it(rig):
    world = World(rig)                       # system_profiler lists FaceTime, B, A; OpenCV sorts by unique id
    cams = service(rig, world)
    with pytest.raises(CameraIdentityError, match="preflight"):
        cams.stills()                                                    # nothing before the check
    r = cams.preflight()
    assert cams.indices == {"a": 0, "b": 1} and r["identified_by"] == "serial" and "reordered" not in r
    assert r["blink"]["a"]["a"] > 0.8 and r["blink"]["a"]["b"] == 0.0
    assert world.controls["SA"]["absolute_exposure_time"] == 100         # the blink put it back
    frames = cams.stills()
    assert set(frames) == {"a", "b"} and frames["a"][0, 0, 0] == 120 and frames["b"][0, 0, 0] == 130
    with pytest.raises(UnknownCamera):
        cams.still("c")


def test_a_stream_order_unlike_the_unique_ids_is_found_by_the_blink(rig):
    world = World(rig, streams=("SB", "SA", "FACETIME"))
    cams = service(rig, world)
    r = cams.preflight()
    assert r["reordered"] == {"a": "b", "b": "a"} and cams.indices == {"a": 1, "b": 0}
    assert cams.stills()["a"][0, 0, 0] == 130                            # index 1: cam A's frames


def test_refuses_a_missing_or_silent_camera(rig):
    with pytest.raises(CameraIdentityError, match="camera b .*not plugged in"):
        service(rig, World(rig, plugged=("SA",))).preflight()
    cams = service(rig, World(rig, streams=("SA", "FACETIME", "SB")))   # index 1 is another camera
    with pytest.raises(CameraIdentityError, match="camera b: dimming it .*no stream"):
        cams.preflight()
    with pytest.raises(CameraIdentityError):
        cams.stills()


def test_both_cameras_with_serials_are_required(rig):
    edit_config(rig, lambda cfg: cfg["cameras"]["b"]["uvc"]["device"].update(serial=None))
    cams = service(rig, World(rig))
    with pytest.raises(CameraIdentityError, match="camera b: no serial"):
        cams.preflight()
    with pytest.raises(CameraIdentityError, match="no serial"):
        cams.lock_controls()                                             # the controls too: by serial only
    edit_config(rig, lambda cfg: cfg["cameras"]["b"]["uvc"]["device"].update(serial="SB"))
    assert cams.preflight()["indices"] == {"a": 0, "b": 1}              # set after startup: picked up
    edit_config(rig, lambda cfg: cfg["cameras"]["b"].update(enabled=False))
    with pytest.raises(CameraIdentityError, match="need 2"):
        cams.preflight()


def test_a_replug_renumbers_and_the_next_preflight_follows(rig):
    world = World(rig)
    cams = service(rig, world)
    cams.preflight()
    a, b = cams.cams["a"], cams.cams["b"]
    world.location["SB"] = 0x14300000                                    # B on another port: a new unique id,
    world.extra = ["0x1420ffff0000aaaa"]                                 # sorted after another device's
    world.streams = ["SA", "X", "SB", "FACETIME"]
    b.opens += 1                                                         # the old capture noticed the unplug
    with pytest.raises(CameraIdentityError):
        cams.stills()
    r = cams.preflight()
    assert r["indices"] == {"a": 0, "b": 2} and cams.cams["a"] is a and cams.cams["b"] is not b and b._stopped
    assert set(cams.stills()) == {"a", "b"}


def test_a_capture_reopened_during_the_stills_is_refused(rig):
    cams = service(rig, World(rig))
    cams.preflight()
    cams.stills()

    def reopen():
        cams.cams["a"].opens += 1
    cams.cams["b"].on_still = reopen
    with pytest.raises(CameraIdentityError, match="reopened while"):
        cams.stills()
    cams.cams["b"].on_still = None
    with pytest.raises(CameraIdentityError):
        cams.still("a")
    cams.preflight()
    assert set(cams.stills()) == {"a", "b"}


def test_previews_reuse_the_device_listing_while_a_camera_is_missing(rig):
    world = World(rig, plugged=("SA",))
    cams = service(rig, world)
    for _ in range(3):
        with pytest.raises(CameraIdentityError, match="not plugged in"):
            cams.preview("a")
    assert world.listings == 1
    with pytest.raises(CameraIdentityError):
        cams.preflight()                                                 # a preflight always lists afresh
    assert world.listings == 2


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


def test_avfoundation_index_sorts_by_unique_id():
    w = World.__new__(World)
    w.location = {"SA": 0x14100000, "SB": 0x14200000}
    dev = {"vendor": VENDOR, "product": PRODUCT, "location": w.location["SB"]}
    av = [{"unique_id": FACETIME}, {"unique_id": w.uid("SB")}, {"unique_id": w.uid("SA")}]
    assert avfoundation_index(dev, av) == 1 and avfoundation_index({**dev, "location": w.location["SA"]}, av) == 0
    assert avfoundation_index({**dev, "location": 1}, av) is None
