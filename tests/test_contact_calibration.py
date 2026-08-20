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
    # The protocol's achievable pairs: pads face palmward, so only the
    # opposing thumb meets a finger pad-to-pad — the thumb is the hub.
    pairs = [("thumb", "index"), ("thumb", "middle"),
             ("thumb", "ring"), ("thumb", "pinky")]
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


# ---- manual abd capture gates (record_contacts, hardware-free) --------------

from record_contacts import (evaluate_manual_sample, find_manifold,  # noqa: E402
                             manual_plan, manual_expected, ABD_PAIRS,
                             MANUAL_VARIANTS)

MAN = {
    "pair": ["middle_abd", "ring_abd"], "arg": "ring_abd", "out": "middle_abd",
    "poly": [0.0, 0.5, 3.0], "arg_range": [-12.0, 15.0], "sigma_deg": 2.0,
    "posture": {"ring_mcp": 15.0, "middle_mcp": 15.0,
                "ring_pip": 8.0, "middle_pip": 8.0},
    "posture_tol_deg": 3.0,
}
# ring_abd moves toward middle by decreasing angle (right hand); middle
# toward ring by increasing.
MANUAL_SIGNS = {("ring_abd", "middle_abd"): -1.0,
                ("middle_abd", "ring_abd"): +1.0}
MANUAL_ROMS = {"ring_abd": (-13.0, 13.0), "middle_abd": (-13.0, 13.0)}
EXPECTED = dict(MAN["posture"])


def _sample(ring_abd=-12.5, middle_abd=None, **over):
    if middle_abd is None:
        middle_abd = float(np.polyval(MAN["poly"], ring_abd))
    mean = {"ring_abd": ring_abd, "middle_abd": middle_abd, **EXPECTED}
    mean.update(over)
    std = {j: 0.05 for j in mean}
    return mean, std


def test_manual_sample_on_manifold_passes():
    mean, std = _sample()
    ok, notes = evaluate_manual_sample(
        "ring_abd", "middle_abd", mean, std, EXPECTED, [MAN],
        MANUAL_SIGNS, MANUAL_ROMS)
    assert ok
    assert any("manifold deviation +0.00" in n for n in notes)
    assert any("braced at stop" in n for n in notes)


def test_manual_sample_gates_reject():
    mean, std = _sample()
    std["ring_abd"] = 0.6
    ok, notes = evaluate_manual_sample(
        "ring_abd", "middle_abd", mean, std, EXPECTED, [MAN],
        MANUAL_SIGNS, MANUAL_ROMS)
    assert not ok and any("unsteady" in n for n in notes)

    mean, std = _sample(ring_mcp=21.0)
    ok, notes = evaluate_manual_sample(
        "ring_abd", "middle_abd", mean, std, EXPECTED, [MAN],
        MANUAL_SIGNS, MANUAL_ROMS)
    assert not ok and any("posture off" in n for n in notes)

    mean, std = _sample(ring_abd=-20.0)  # e.g. a misplaced encoder magnet
    ok, notes = evaluate_manual_sample(
        "ring_abd", "middle_abd", mean, std, EXPECTED, [MAN],
        MANUAL_SIGNS, MANUAL_ROMS)
    assert not ok and any("solver would skip" in n for n in notes)

    mean, std = _sample()
    del mean["ring_abd"]
    ok, notes = evaluate_manual_sample(
        "ring_abd", "middle_abd", mean, std, EXPECTED, [MAN],
        MANUAL_SIGNS, MANUAL_ROMS)
    assert not ok and "no encoder reading" in notes[0]


def test_manual_sample_without_manifolds_still_captures():
    mean, std = _sample()
    ok, notes = evaluate_manual_sample(
        "ring_abd", "middle_abd", mean, std, EXPECTED, [],
        MANUAL_SIGNS, MANUAL_ROMS)
    assert ok
    assert not any("manifold deviation" in n for n in notes)


def test_find_manifold_is_direction_specific_with_edge_slack():
    mean, _ = _sample()
    assert find_manifold([MAN], "ring_abd", "middle_abd", mean)[0] is MAN
    assert find_manifold([MAN], "middle_abd", "ring_abd", mean)[0] is None
    mean["ring_abd"] = -13.2  # braced just past the fitted grid edge
    assert find_manifold([MAN], "ring_abd", "middle_abd", mean)[0] is MAN
    mean["ring_abd"] = -14.0
    man, why = find_manifold([MAN], "ring_abd", "middle_abd", mean)
    assert man is None and "outside manifold range" in why


def test_estimator_manifold_gate_admits_edge_slack(nominal):
    est = Estimator(nominal, manifolds=[MAN])
    jidx = {j: i for i, j in enumerate(est.joints)}
    m = np.zeros(len(est.joints))
    for j, v in MAN["posture"].items():
        m[jidx[j]] = v
    m[jidx["ring_abd"]] = 16.0  # 1 deg past the fitted edge: admitted
    assert est._match_manifold(sorted(MAN["pair"]), m, jidx)[0] is not None
    m[jidx["ring_abd"]] = 17.0  # past the slack: still refused
    assert est._match_manifold(sorted(MAN["pair"]), m, jidx)[0] is None


def test_manual_plan_covers_pairs_directions_variants():
    plan = manual_plan()
    assert len(plan) == len(ABD_PAIRS) * 2 * len(MANUAL_VARIANTS)
    assert len(set(plan)) == len(plan)
    for a, b in ABD_PAIRS:
        for variant in MANUAL_VARIANTS:
            assert (a, b, variant) in plan and (b, a, variant) in plan


def test_manual_expected_hardstop_postures():
    roms = {}
    for f in ("ring", "pinky"):
        roms[f + "_mcp"] = (-25.0, 100.0)
        roms[f + "_pip"] = (-15.0, 107.0)
    # Both straight: everything at the extension end.
    exp = manual_expected(roms, "ring_abd", "pinky_abd", "extended")
    assert exp == {"ring_mcp": -25.0, "ring_pip": -15.0,
                   "pinky_mcp": -25.0, "pinky_pip": -15.0}
    # Flexed: braced finger straight, moved finger mcp at the flexion
    # stop, its pip deliberately ungated.
    exp = manual_expected(roms, "ring_abd", "pinky_abd", "flexed")
    assert exp == {"ring_mcp": -25.0, "ring_pip": -15.0,
                   "pinky_mcp": 100.0}
    assert "pinky_pip" not in exp


def test_has_manifold_matches_commanded_posture():
    from record_contacts import has_manifold
    assert has_manifold([MAN], "ring_abd", "middle_abd", EXPECTED)
    assert not has_manifold([MAN], "middle_abd", "ring_abd", EXPECTED)
    at40 = {j: (40.0 if j.endswith("_mcp") else v) for j, v in EXPECTED.items()}
    assert not has_manifold([MAN], "ring_abd", "middle_abd", at40)


def test_manual_plan_groups_by_pair():
    plan = manual_plan()
    pair_seq = [frozenset((p, q)) for p, q, _ in plan]
    seen = []
    for pair in pair_seq:
        if not seen or seen[-1] != pair:
            seen.append(pair)
    assert len(seen) == len(ABD_PAIRS)  # each pair forms one contiguous block


def test_derive_pair_signs_chains_toward_thumb():
    from record_contacts import derive_pair_signs
    # Right-hand model convention: toward thumb = decreasing everywhere.
    t = {f"{f}_abd": -1.0 for f in ("index", "middle", "ring", "pinky")}
    s = derive_pair_signs(t)
    assert s[("pinky_abd", "ring_abd")] == -1.0   # toward thumb side
    assert s[("ring_abd", "pinky_abd")] == +1.0   # away from thumb side
    assert s[("middle_abd", "index_abd")] == -1.0
    assert s[("index_abd", "middle_abd")] == +1.0
    # An encoder-inverted index flips exactly its own entries.
    t["index_abd"] = +1.0
    s = derive_pair_signs(t)
    assert s[("index_abd", "middle_abd")] == -1.0
    assert s[("middle_abd", "index_abd")] == -1.0


def test_model_toward_thumb_convention_is_decreasing_on_right():
    """Pins the packaged model's lateral convention: on the right hand,
    decreasing abd angle moves every finger toward the thumb. The hardware
    probe is compared against exactly this."""
    from record_contacts import (compute_approach_signs,
                                 compute_toward_thumb_signs)
    neutral = {}
    ts = compute_toward_thumb_signs("right", neutral)
    assert ts == {f"{f}_abd": -1.0
                  for f in ("index", "middle", "ring", "pinky")}
    ps = compute_approach_signs("right", neutral)
    from record_contacts import derive_pair_signs
    assert derive_pair_signs(ts) == ps


def test_abd_frame_flips_into_model_frame():
    from record_contacts import abd_frame_flips, apply_frame
    model = {f"{f}_abd": -1.0 for f in ("index", "middle", "ring", "pinky")}
    roms = {j: tuple(JOINT_ROMS[j]) for j in model}
    # Built-in measured convention (toward thumb = increasing) flips all
    # four; symmetric ROMs need no offset, index's asymmetric one aligns
    # the mirrored config stops onto the model's (-30..25 -> -5).
    flips = abd_frame_flips(model, roms)
    assert flips["ring_abd"] == (-1.0, 0.0)
    assert flips["index_abd"] == (-1.0, -5.0)
    out = apply_frame({"ring_abd": 20.0, "index_abd": -30.0,
                       "ring_mcp": 15.0}, flips)
    assert out["ring_abd"] == -20.0 and out["ring_mcp"] == 15.0
    assert out["index_abd"] == 25.0  # config stop lands on the model stop
    # A measured override that matches the model cancels that joint's flip.
    flips = abd_frame_flips(model, roms, {"index_abd": -1.0})
    assert flips["index_abd"] == (1.0, 0.0)
    assert flips["ring_abd"][0] == -1.0


def test_find_manifold_reports_the_reason_that_applies():
    # A posture mismatch from another posture family must not mask the
    # intended entry's failure reason.
    other = dict(MAN, posture={"ring_mcp": 40.0, "middle_mcp": 40.0},
                 arg_range=[-12.0, 15.0])
    mean, _ = _sample(ring_abd=-20.0)  # matches MAN's posture, out of range
    man, why = find_manifold([MAN, other], "ring_abd", "middle_abd", mean)
    assert man is None and "outside manifold range" in why


def test_session_joint_order_need_not_match_the_model(nominal):
    # Capture tools store encoder columns in the hand's own joint order, which
    # differs from the model's; the estimator must permute rather than index
    # one order with the other's positions.
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
        qa = rng.uniform(JOINT_ROMS[a][0] + 3, JOINT_ROMS[a][1] - 3)
        m = np.zeros(len(joints))
        m[jidx[a]] = qa + off[a]
        m[jidx[b]] = float(np.polyval(posture_polys[posture], qa)) + off[b]
        m[jidx["ring_mcp"]] = posture
        rows.append(m)
        events.append(ContactEvent(frame=k, kind="abd_block",
                                   body_a=a, body_b=b))
    m_model = np.asarray(rows)

    perm = list(rng.permutation(len(joints)))
    shuffled = [joints[i] for i in perm]
    assert shuffled != list(joints)
    ds = _empty_ds(m_model[:, perm], shuffled, events)

    prior = PriorConfig(mode={a: "wide", b: "wide"})
    est = Estimator(nominal, offsets_only=True, prior=prior,
                    manifolds=manifolds)
    result = est.solve(ds, est.initial_x({}))
    assert not est.skipped_contacts
    assert result.b[jidx[a], 0] == pytest.approx(-off[a], abs=0.3)
    assert result.b[jidx[b], 0] == pytest.approx(-off[b], abs=0.3)


def test_contact_session_without_wrist_solves_with_wrist_filled(nominal):
    # A hand whose encoder loop excludes the wrist records 16 columns; the
    # wrist is common to both chains of every self-contact, so contact
    # residuals are invariant to it and it is filled at nominal.
    rng = np.random.default_rng(3)
    offsets = {"thumb_cmc": 1.6, "thumb_mcp": 0.9, "index_mcp": -1.4}
    ds, jidx = _tip_press_truth(nominal, rng, offsets)
    keep = [j for j in ds.joints if j != "wrist"]
    cols = [list(ds.joints).index(j) for j in keep]
    ds_nw = _empty_ds(ds.m[:, cols], keep, ds.contact_obs)

    prior = PriorConfig(mode={j: "wide" for j in nominal.joint_names})
    est = Estimator(nominal, offsets_only=True, prior=prior)
    result = est.solve(ds_nw, est.initial_x({}))
    assert est.filled_joints == ["wrist"]
    for j, off in offsets.items():
        assert result.b[jidx[j], 0] == pytest.approx(-off, abs=0.3), j
    assert abs(result.b[jidx["wrist"], 0]) < 0.3  # stays on its prior

    # A missing FINGER joint does not cancel: hard error.
    keep2 = [j for j in ds.joints if j != "index_mcp"]
    cols2 = [list(ds.joints).index(j) for j in keep2]
    ds_bad = _empty_ds(ds.m[:, cols2], keep2, ds.contact_obs)
    est2 = Estimator(nominal, offsets_only=True, prior=prior)
    with pytest.raises(ValueError, match="index_mcp"):
        est2.solve(ds_bad, est2.initial_x({}))

    # With world-frame observations even a missing wrist is a hard error.
    ds_dots = Dataset(
        m=ds.m[:, cols], dot_obs={"index_pip": np.zeros((len(ds.m), 2, 3))},
        dir_obs={}, frame_axes={}, dir_frames=np.array([], int),
        plate_obs=[], sweep_frames={}, joints=keep, contact_obs=[],
    )
    est3 = Estimator(nominal, offsets_only=True, prior=prior)
    with pytest.raises(ValueError, match="wrist"):
        est3.solve(ds_dots, est3.initial_x({}))


def test_raw_calibration_map_folds_frame_and_offset():
    from solve_session import raw_calibration_map
    meta = {"abd_frame": {"index_abd": [-1, -5.0], "ring_abd": [-1, 0.0]}}
    joints = ["index_abd", "ring_abd", "ring_mcp"]
    b = np.array([[2.0], [-1.5], [0.25]])
    m = raw_calibration_map(meta, joints, b)
    # Model-frame map: calibrated = sign*raw + (frame offset + b0).
    # Hand-frame delta (for URDF/display consumers): sign*b0, frame cancels.
    assert m["index_abd"] == {"raw_sign": -1, "raw_offset_deg": -3.0,
                              "hand_frame_delta_deg": -2.0}
    assert m["ring_abd"] == {"raw_sign": -1, "raw_offset_deg": -1.5,
                             "hand_frame_delta_deg": 1.5}
    assert m["ring_mcp"] == {"raw_sign": 1, "raw_offset_deg": 0.25,
                             "hand_frame_delta_deg": 0.25}
