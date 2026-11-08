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
