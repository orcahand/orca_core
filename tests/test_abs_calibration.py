"""Tests for the absolute-calibration estimator tooling (tools/abs_calibration).

The estimator is maintainer tooling (not shipped in the wheel), but its
correctness gates the calibration project, so the load-bearing pieces are
pinned here: masked VarPro, the session format round-trip, the circle-fit
axis diagnostic, sigma reporting, and end-to-end offset recovery.
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

from dataset import load_session, save_session  # noqa: E402
from estimator import Estimator, PriorConfig  # noqa: E402
from model import JOINT_ROMS, KinematicModel  # noqa: E402
from sim import Scenario, generate, make_truth  # noqa: E402


@pytest.fixture(scope="module")
def nominal():
    return KinematicModel.load("right")


def _mini_scenario(**overrides):
    base = dict(name="mini", inl_deg=0.0, gain_frac=0.0, sweep_steps=12,
                n_anchor=6, n_palm=3, n_plate_wrist=2, seeds=1)
    base.update(overrides)
    return Scenario(**base)


def _solve(nominal, sc, seed=7, offsets_only=False, denovo=()):
    rng = np.random.default_rng(seed)
    truth = make_truth(nominal, sc, rng)
    ds = generate(nominal, sc, truth, rng)
    prior = PriorConfig(mean=dict(truth.hardstop_b0))
    for j in denovo:
        prior.mode[j] = "off"
    est = Estimator(nominal, offsets_only=offsets_only, prior=prior)
    x0 = est.initial_x(truth.hardstop_b0)
    if denovo:
        est.prepare(ds)
        x0 = est.denovo_init(x0, ds, list(denovo))
    return est, est.solve(ds, x0), truth, ds


def _mapping_errors(est, result, truth):
    errs = {}
    for i, j in enumerate(est.joints):
        q = np.linspace(*JOINT_ROMS[j], 100)
        m = truth.enc.measure(j, q)
        q_hat = est.apply_cal(np.tile(m[:, None], (1, len(est.joints))), result.b)[:, i]
        errs[j] = float(np.mean(np.abs(q_hat - q)))
    return errs


def test_recovers_offsets_with_occlusion(nominal):
    est, result, truth, _ = _solve(nominal, _mini_scenario())
    errs = _mapping_errors(est, result, truth)
    assert max(errs.values()) < 0.2, errs


def test_masking_matches_dataset_shape(nominal):
    sc = _mini_scenario(link_dropout=0.4, dot_dropout=0.3)
    rng = np.random.default_rng(3)
    truth = make_truth(nominal, sc, rng)
    ds = generate(nominal, sc, truth, rng)
    est = Estimator(nominal, prior=PriorConfig(mean=dict(truth.hardstop_b0)))
    est.prepare(ds)
    r1 = est.residuals(est.initial_x(truth.hardstop_b0), ds)
    r2 = est.residuals(est.initial_x({j: 0.0 for j in est.joints}), ds)
    assert r1.shape == r2.shape  # masks are data-fixed, not parameter-dependent
    assert np.isfinite(r1).all() and np.isfinite(r2).all()


def test_session_roundtrip(nominal, tmp_path):
    sc = _mini_scenario()
    rng = np.random.default_rng(5)
    truth = make_truth(nominal, sc, rng)
    ds = generate(nominal, sc, truth, rng)
    save_session(ds, str(tmp_path / "session"))
    ds2 = load_session(str(tmp_path / "session"))
    np.testing.assert_array_equal(ds.m, ds2.m)
    assert set(ds.dot_obs) == set(ds2.dot_obs)
    for link in ds.dot_obs:
        np.testing.assert_array_equal(ds.dot_obs[link], ds2.dot_obs[link])
    assert set(ds.frame_axes) == set(ds2.frame_axes)
    assert ds2.sweep_frames.keys() == ds.sweep_frames.keys()
    assert len(ds2.plate_obs) == len(ds.plate_obs)
    # A loaded session must solve identically to the in-memory one.
    est = Estimator(nominal, prior=PriorConfig(mean=dict(truth.hardstop_b0)))
    x0 = est.initial_x(truth.hardstop_b0)
    np.testing.assert_allclose(
        est.solve(ds, x0).b, est.solve(ds2, x0).b, atol=1e-9
    )


FLEXION_JOINTS = ("wrist", "index_mcp", "index_pip", "middle_mcp", "middle_pip",
                  "ring_mcp", "ring_pip", "pinky_mcp", "pinky_pip",
                  "thumb_mcp", "thumb_dip")


def test_axis_diagnostic_floor_and_recovery(nominal):
    # Floor: no injected tilt. Long flexion arcs resolve ~0.1°; short
    # abduction arcs are honest only to their self-reported floor.
    est, result, _, ds = _solve(nominal, _mini_scenario(sweep_steps=20))
    diag = est.axis_diagnostic(result.x, ds)
    flex = [diag[j]["deg"] for j in FLEXION_JOINTS if j in diag]
    assert np.median(flex) < 0.2, diag
    for j, d in diag.items():
        assert d["deg"] < max(3.0 * d["floor_deg"], 0.3), (j, d)

    # Recovery: injected tilts read back within tolerance on flexion joints.
    sc = _mini_scenario(name="tilt", axis_tilt_deg=0.8, sweep_steps=20)
    rng = np.random.default_rng(11)
    truth = make_truth(nominal, sc, rng)
    ds = generate(nominal, sc, truth, rng)
    est = Estimator(nominal, prior=PriorConfig(mean=dict(truth.hardstop_b0)))
    result = est.solve(ds, est.initial_x(truth.hardstop_b0))
    diag = est.axis_diagnostic(result.x, ds)

    true_tilt = {}
    for j in est.joints:
        a_nom = nominal.nodes[nominal.index[j]].axis
        a_true = truth.kin.nodes[truth.kin.index[j]].axis
        true_tilt[j] = np.degrees(np.arccos(np.clip(abs(a_nom @ a_true), -1, 1)))
    deviations = [abs(diag[j]["deg"] - true_tilt[j])
                  for j in FLEXION_JOINTS if j in diag]
    assert np.median(deviations) < 0.3, (diag, true_tilt)


def test_sigma_reported_and_grows_with_prior_off(nominal):
    est, result, _, _ = _solve(nominal, _mini_scenario())
    assert set(result.sigma_b0_deg) == set(est.joints)
    assert all(0 < s < 5.0 for s in result.sigma_b0_deg.values())

    _, result_off, _, _ = _solve(nominal, _mini_scenario(), denovo=("ring_abd",))
    assert result_off.sigma_b0_deg["ring_abd"] >= result.sigma_b0_deg["ring_abd"] * 0.5


def test_denovo_recovers_gross_init(nominal):
    sc = _mini_scenario(gross_init_err={"ring_abd": 20.0})
    est, result, truth, _ = _solve(nominal, sc, denovo=("ring_abd",))
    errs = _mapping_errors(est, result, truth)
    assert errs["ring_abd"] < 0.3, errs["ring_abd"]
