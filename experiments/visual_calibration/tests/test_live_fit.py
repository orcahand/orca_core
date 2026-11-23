"""The camera-only hand pose fit on synthetic frames: the model rendered at a known pose."""
import numpy as np
import pytest

from visual_calibration.hand_model import description_root

cv2 = pytest.importorskip("cv2")
pytestmark = pytest.mark.skipif(description_root() is None, reason="orcahand_description checkout not found")

PALM_ON = 270.0          # on the uncalibrated rig (CAD mounting) the palm faces the camera here


def synth(f, x, angle, size=(540, 960)):
    """A dark hand (each link its own shade) on a light, noisy background."""
    from visual_calibration.live_fit import scaled
    lab = f.label(x, angle, scaled(f.lens, size[1] / f.lens.size[0]), size)
    rng = np.random.default_rng(1)
    img = 170 + rng.normal(0, 6, size)
    tones = np.r_[170, np.random.default_rng(3).uniform(30, 75, 40)]
    img = np.where(lab > 0, tones[lab], img)
    return cv2.GaussianBlur(np.clip(img, 0, 255).astype(np.uint8), (3, 3), 0)


@pytest.fixture(scope="module")
def fit(tmp_path_factory):
    import shutil

    from visual_calibration.live_fit import LiveHandFit
    from visual_calibration.rig_config import RigConfig, get_rig_path
    path = tmp_path_factory.mktemp("rig") / "v1" / "camera-rig"
    shutil.copytree(get_rig_path(), path, ignore=shutil.ignore_patterns("camera_reference.yaml"))
    (path / "calibration.yaml").write_text("calibrated_at: null\nhistory: []\n")
    return LiveHandFit(RigConfig.load(path=str(path)))


def test_mount_only(fit):
    fit.reset()
    true = np.zeros(fit.x.size)
    true[:6] = [1.5, -1.0, 2.0, 2.0, -1.5, 1.0]
    img = synth(fit, true, PALM_ON)
    for _ in range(8):
        r = fit.step(img, PALM_ON, {}, iters=3)
    assert r["cost_px"]["fit"] < r["cost_px"]["cad"] and r["outlines"]["fit"]
    assert np.abs(fit.x[:6] - true[:6]).max() < 2.0    # rotation and depth trade a little; the prior pulls towards the CAD
    assert r["segmentation"]["png"] and r["joint_table"] == [] and len(r["plate_from_hand"]) == 4
    assert np.allclose(np.array(r["plate_from_hand"])[:3, 3], fit.hand_in_plate(fit.x).translation, atol=1e-3)


def test_joints_palm_on(fit):
    """Seen palm-on: the wrist, the spread and the thumb's base come out within a few degrees;
    every joint called good is right, and the bent fingers' flexion is flagged (it can't be told
    from over-stretched from this side) with a plate angle that shows it better."""
    fit.reset()
    q = dict(wrist=5, index_abd=8, ring_abd=-6, thumb_abd=10, thumb_mcp=20, thumb_dip=15,
             index_mcp=25, index_pip=15, middle_pip=30, ring_mcp=35, pinky_mcp=20, pinky_pip=20)
    true = np.zeros(fit.x.size)
    true[:6] = [1.0, -0.5, 1.0, 1.0, -1.0, 0.5]
    for k, v in q.items():
        true[6 + fit.joints.index(k)] = v
    img = synth(fit, true, PALM_ON)
    for i in range(14):
        r = fit.step(img, PALM_ON, iters=3, solve_joints=True, search=i % 4 == 0)
    table = {row["name"]: row for row in r["joint_table"]}
    err = {j: table[j]["deg"] - q.get(j, 0.0) for j in table}
    for j in ("wrist", "pinky_abd", "ring_abd", "middle_abd", "index_abd", "thumb_cmc", "thumb_abd"):
        assert abs(err[j]) < 5.0, (j, err[j])
    for j, row in table.items():
        assert row["confidence"] != "good" or abs(err[j]) < 5.0, (j, row, err[j])
    flagged = {j for j, row in table.items() if row["confidence"] == "poor"}
    assert {"index_mcp", "index_pip", "middle_pip", "pinky_pip"} <= flagged
    assert r["suggest"] and r["suggest"]["better"] and abs(r["suggest"]["angle"] - PALM_ON) >= 30


@pytest.fixture(scope="module")
def fit2(tmp_path_factory):
    """The uncalibrated rig with cam B on the side rail, solved at its CAD pose (portrait, 45° down)."""
    import shutil

    import yaml

    from visual_calibration.camera import look_at, nominal_lens
    from visual_calibration.live_fit import LiveHandFit
    from visual_calibration.rig_calibration import _lens_dict, plain, rpy_deg
    from visual_calibration.rig_config import RigConfig, get_rig_path
    path = tmp_path_factory.mktemp("rig2") / "v1" / "camera-rig"
    shutil.copytree(get_rig_path(), path, ignore=shutil.ignore_patterns("camera_reference*.yaml"))
    cfg = yaml.safe_load((path / "config.yaml").read_text())
    cfg["cameras"]["b"]["enabled"] = True
    (path / "config.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    rig = RigConfig.load(path=str(path))
    n = rig.nominal_of("b")
    pose = look_at(n["position"], n["look_at"], up=(1.0, 0.0, 0.0))
    b = {"lens": _lens_dict(nominal_lens((3840, 2160), 90.0), 0.0),
         "camera": {"translation": list(pose.translation), "rotation": rpy_deg(pose.rotation).tolist()}}
    (path / "calibration.yaml").write_text(yaml.safe_dump(plain({"calibrated_at": None, "history": [], "cameras": {"b": b}})))
    return LiveHandFit(RigConfig.load(path=str(path)))


def test_views_of_both_cameras_join_the_mount_fit(fit2):
    """A view from cam B kept, then cam A's frames: the mount comes out closer than from cam A alone."""
    from visual_calibration.live_fit import scaled
    assert set(fit2.cameras) == {"a", "b"}
    true = np.zeros(fit2.x.size)
    true[:6] = [1.5, -1.0, 2.0, 2.0, -1.5, 1.0]
    lab_b = fit2.label(true, PALM_ON, scaled(fit2.lens, 960 / fit2.lens.size[0]), (540, 960), cam="b")
    assert (lab_b > 0).sum() > 1000                         # cam B sees the hand from above the side
    img_a = synth(fit2, true, PALM_ON)

    fit2.reset()
    fit2.clear_views()
    for _ in range(6):
        fit2.step(img_a, PALM_ON, {}, iters=3)
    alone = np.abs(fit2.x[:6] - true[:6]).max()

    fit2.reset()
    for _ in range(3):
        rb = fit2.step(synth2(fit2, true, PALM_ON, "b"), PALM_ON, {}, iters=3, cam="b")
    assert fit2.keep_view() == 1 and fit2.views[0].camera == "b"
    for _ in range(6):
        r = fit2.step(img_a, PALM_ON, {}, iters=3)
    assert r["view_cameras"] == ["b"] and r["camera"] == "a" and rb["camera"] == "b"
    both = np.abs(fit2.x[:6] - true[:6]).max()
    assert both < 1.5 and both < 0.8 * alone, (both, alone)


def synth2(f, x, angle, cam, size=(540, 960)):
    from visual_calibration.live_fit import scaled
    lab = f.label(x, angle, scaled(f.lens, size[1] / f.lens.size[0]), size, cam=cam)
    rng = np.random.default_rng(2)
    img = 170 + rng.normal(0, 6, size)
    tones = np.r_[170, np.random.default_rng(3).uniform(30, 75, 40)]
    img = np.where(lab > 0, tones[lab], img)
    return cv2.GaussianBlur(np.clip(img, 0, 255).astype(np.uint8), (3, 3), 0)
