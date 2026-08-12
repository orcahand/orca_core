"""Tests for the camera-free contact-calibration path (tools/abs_calibration).

Pins the session format's contact events, the capture detectors, the
estimator's contact residuals (tip-press coincidence and abd-block
manifolds, verified to recover injected encoder offsets with no camera
data), the base-pose gauge pinning, and — when the description repo is
checked out — the URDF-vs-packaged-model frame agreement the manifold
builder depends on.
"""

import os
import sys

import numpy as np
import pytest

TOOLS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "tools", "abs_calibration",
)
sys.path.insert(0, TOOLS_DIR)

from contact_detect import (StallDetector, TipPressDetector,  # noqa: E402
                            taxel_centroid)
from dataset import ContactEvent, Dataset, load_session, save_session  # noqa: E402
from estimator import Estimator, PriorConfig  # noqa: E402
from model import DISTAL_LINK, JOINT_ROMS, KinematicModel  # noqa: E402

DESCRIPTION_URDF = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "orcahand_description", "v2", "models", "urdf", "orcahand_right.urdf")


@pytest.fixture(scope="module")
def nominal():
    return KinematicModel.load("right")


def _empty_ds(m, joints, contacts):
    return Dataset(
        m=m, dot_obs={}, dir_obs={}, frame_axes={},
        dir_frames=np.array([], int), plate_obs=[], sweep_frames={},
        joints=joints, contact_obs=contacts,
    )


# ---- session format ----------------------------------------------------------

def test_contact_session_roundtrip(tmp_path):
    joints = ["a", "b"]
    events = [
        ContactEvent(frame=0, kind="abd_block", body_a="a", body_b="b",
                     push_current_ma=80.0),
        ContactEvent(frame=1, kind="tip_press", body_a="index", body_b="thumb",
                     p_a=np.array([0.001, 0.002, 0.003]),
                     p_b=np.array([-0.001, 0.0, 0.002]),
                     force_a=0.8, force_b=1.1),
    ]
    ds = _empty_ds(np.zeros((2, 2)), joints, events)
    save_session(ds, str(tmp_path / "s"))
    back = load_session(str(tmp_path / "s"))
    assert len(back.contact_obs) == 2
    e = back.contact_obs[1]
    assert e.kind == "tip_press" and e.body_b == "thumb"
    assert np.allclose(e.p_a, [0.001, 0.002, 0.003])
    assert back.contact_obs[0].push_current_ma == pytest.approx(80.0)
    assert e.force_b == pytest.approx(1.1)


def test_v1_session_still_loads(tmp_path):
    ds = _empty_ds(np.zeros((1, 1)), ["a"], [])
    save_session(ds, str(tmp_path / "s"))
    os.remove(str(tmp_path / "s" / "contacts.yaml"))
    manifest = str(tmp_path / "s" / "session.yaml")
    text = open(manifest).read().replace("format: 2", "format: 1")
    open(manifest, "w").write(text)
    back = load_session(str(tmp_path / "s"))
    assert back.contact_obs == []


# ---- capture detectors -------------------------------------------------------

def test_stall_detector_fires_on_block_not_motion():
    det = StallDetector()
    cmd, meas = 0.0, 0.0
    for _ in range(20):                      # free motion: tracks with lag
        cmd += 0.4
        meas = cmd - 0.3
        assert not det.update(cmd, meas)
    fired_at = None
    for i in range(10):                      # blocked: measured frozen
        cmd += 0.4
        if det.update(cmd, meas):
            fired_at = i
            break
    assert fired_at is not None and fired_at <= 4  # onset, minimal force


def test_tip_press_detector_capture_and_cooldown():
    det = TipPressDetector()
    m = np.zeros(3)
    fires = [det.update(1.0, 1.0, m) for _ in range(12)]
    assert sum(fires) == 1                   # one capture per press
    assert not det.update(1.0, 1.0, m)       # still cooling down


def test_taxel_centroid_weighted():
    pos = np.array([[0.0, 0, 0], [0.01, 0, 0], [0.02, 0, 0]])
    forces = np.array([[0, 0, 1.0], [0, 0, 3.0], [0, 0, 0.0]])
    c, resultant = taxel_centroid(pos, forces)
    assert c[0] == pytest.approx(0.0075)
    assert resultant == pytest.approx(4.0)


# ---- estimator: contact residuals -------------------------------------------

def _tip_press_truth(nominal, rng, offsets, n_events=36):
    """Coincident-pad events synthesized from a perturbed-encoder truth."""
    joints = nominal.joint_names
    jidx = {j: i for i, j in enumerate(joints)}
    pairs = [("thumb", "index"), ("thumb", "middle"), ("index", "middle"),
             ("thumb", "ring"), ("middle", "ring"), ("ring", "pinky")]
    rows, events = [], []
    for k in range(n_events):
        q = np.array([rng.uniform(lo + 5, hi - 5)
                      for lo, hi in (JOINT_ROMS[j] for j in joints)])
        fa, fb = pairs[k % len(pairs)]
        poses = nominal.link_poses({j: np.array([q[i]])
                                    for i, j in enumerate(joints)})
        p_a = rng.uniform(-0.004, 0.004, 3)
        mnt_a, mnt_b = nominal.sensor_mounts[fa], nominal.sensor_mounts[fb]
        Ra, ta = poses[DISTAL_LINK[fa]]
        world = Ra[0] @ (mnt_a.rotation @ p_a + mnt_a.translation) + ta[0]
        Rb, tb = poses[DISTAL_LINK[fb]]
        p_link_b = Rb[0].T @ (world - tb[0])
        p_b = mnt_b.rotation.T @ (p_link_b - mnt_b.translation)
        m_row = q + np.array([offsets.get(j, 0.0) for j in joints])
        rows.append(m_row)
        events.append(ContactEvent(frame=k, kind="tip_press", body_a=fa,
                                   body_b=fb, p_a=p_a, p_b=p_b,
                                   force_a=1.2, force_b=1.2))
    return _empty_ds(np.asarray(rows), joints, events), jidx


def test_tip_press_recovers_offsets_without_cameras(nominal):
    rng = np.random.default_rng(3)
    offsets = {"thumb_cmc": 1.6, "thumb_abd": -1.2, "thumb_mcp": 0.9,
               "index_mcp": -1.4, "index_pip": 1.1, "middle_mcp": 0.7}
    ds, jidx = _tip_press_truth(nominal, rng, offsets)
    # Contact-only identifiability leans on the prior by design (doc section
    # 5); wide mode keeps the pull small enough to verify the residual math.
    prior = PriorConfig(mode={j: "wide" for j in nominal.joint_names})
    est = Estimator(nominal, offsets_only=True, prior=prior)
    result = est.solve(ds, est.initial_x({}))
    assert est._pin_base  # no world-frame class present
    for j, off in offsets.items():
        assert result.b[jidx[j], 0] == pytest.approx(-off, abs=0.3), j
    # The wrist is invisible to self-contact: it must sit on its prior.
    assert abs(result.b[jidx["wrist"], 0]) < 0.3
    assert result.sigma_b0_deg["wrist"] > 3 * result.sigma_b0_deg["index_mcp"]


def test_abd_block_recovers_offsets_with_synthetic_manifolds(nominal):
    # One contact curve constrains only a COMBINATION of the pair's offsets
    # (both drive directions probe the same manifold); the second flexion
    # posture touches a different mesh patch, and the two curves' differing
    # slopes are what make the individual offsets identifiable. This is why
    # the capture protocol runs every pair at two postures.
    rng = np.random.default_rng(4)
    joints = nominal.joint_names
    jidx = {j: i for i, j in enumerate(joints)}
    a, b = "ring_abd", "pinky_abd"
    posture_polys = {15.0: np.array([0.002, -1.15, 12.0]),
                     40.0: np.array([0.003, -0.55, 4.0])}
    manifolds = [
        {"pair": sorted((a, b)), "arg": a, "out": b, "poly": list(poly),
         "arg_range": list(JOINT_ROMS[a]), "sigma_deg": 0.3,
         "posture": {"ring_mcp": posture}}
        for posture, poly in posture_polys.items()
    ]
    off = {a: 2.1, b: -1.7}
    rows, events = [], []
    for k in range(12):
        posture = 15.0 if k % 2 == 0 else 40.0
        poly = posture_polys[posture]
        qa = rng.uniform(JOINT_ROMS[a][0] + 3, JOINT_ROMS[a][1] - 3)
        qb = float(np.polyval(poly, qa))
        q = np.zeros(len(joints))
        q[jidx[a]], q[jidx[b]] = qa, qb
        q[jidx["ring_mcp"]] = posture
        m = q.copy()
        m[jidx[a]] += off[a]
        m[jidx[b]] += off[b]
        rows.append(m)
        events.append(ContactEvent(frame=k, kind="abd_block",
                                   body_a=a, body_b=b))
    ds = _empty_ds(np.asarray(rows), joints, events)
    prior = PriorConfig(mode={a: "wide", b: "wide"})
    est = Estimator(nominal, offsets_only=True, prior=prior,
                    manifolds=manifolds)
    result = est.solve(ds, est.initial_x({}))
    assert result.b[jidx[a], 0] == pytest.approx(-off[a], abs=0.3)
    assert result.b[jidx[b], 0] == pytest.approx(-off[b], abs=0.3)
    assert not est.skipped_contacts


def test_abd_block_without_manifold_is_skipped_not_guessed(nominal):
    joints = nominal.joint_names
    ds = _empty_ds(np.zeros((1, len(joints))), joints, [
        ContactEvent(frame=0, kind="abd_block",
                     body_a="ring_abd", body_b="pinky_abd")])
    est = Estimator(nominal, offsets_only=True, prior=PriorConfig())
    est.solve(ds, est.initial_x({}))
    assert est.skipped_contacts
    assert est.skipped_contacts[0][3] == "no manifold"


# ---- manifold builder frame agreement (needs the description checkout) ------

@pytest.mark.skipif(not os.path.exists(DESCRIPTION_URDF),
                    reason="orcahand_description not checked out")
def test_urdf_fk_matches_packaged_model(nominal):
    from build_contact_manifold import UrdfHand, cross_check_fk
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(DESCRIPTION_URDF))))
    hand = UrdfHand(DESCRIPTION_URDF, root)
    hand.calibrate_signs(nominal)
    assert cross_check_fk(hand, nominal) < 1e-5
    assert all(hand.q_sign[f + "_abd"] == -1.0
               for f in ("index", "middle", "ring", "pinky"))
