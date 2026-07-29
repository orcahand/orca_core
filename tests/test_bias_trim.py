"""Tests for the controller's slow feed-forward bias: the leak rate, the
closed-loop handoff (bias takes over the DC term, the fast integral unwinds,
no visible transient), the gates, the clamp with its warning, and reset."""

from __future__ import annotations

import logging

import numpy as np
import pytest

from orca_core.control import JointController
from orca_core.control.constants import DEFAULT_BIAS_TAU_S
from orca_core.hardware.sensing.constants import ENCODER_LSB_DEG


N = 4
DT = 0.01


def _make(
    *,
    Kp=1.0,
    Ki=8.0,
    correction_max_deg=20.0,
    i_clamp_deg=20.0,
    bias_tau_s=1.0,
    bias_max_deg=3.0,
    joint_names=None,
):
    controller = JointController(
        num_joints=N,
        joint_names=joint_names,
        bias_tau_s=bias_tau_s,
        bias_max_deg=bias_max_deg,
    )
    controller.set_gains(
        Kp=Kp, Ki=Ki,
        correction_max_deg=correction_max_deg,
        i_clamp_deg=i_clamp_deg,
    )
    return controller


class _Plant:
    """First-order joint whose map is wrong by a constant ``offset_deg``: it
    approaches ``target + trim - offset`` with time constant ``tau_s``, so the
    loop settles only once the total trim equals the offset."""

    def __init__(self, offset_deg, target_deg=0.0, tau_s=0.05):
        self.target = np.full(N, float(target_deg))
        self.offset = np.full(N, float(offset_deg))
        self.tau_s = float(tau_s)
        self.measured = self.target - self.offset
        self.trim = np.zeros(N)

    def run(self, controller, cycles, dt=DT):
        """Advance ``cycles`` closed-loop cycles, returning the per-cycle
        history of (error, fast correction, bias)."""
        alpha = dt / (self.tau_s + dt)
        history = []
        for _ in range(cycles):
            commanded = self.target + self.trim - self.offset
            self.measured += alpha * (commanded - self.measured)
            correction = controller.step(self.target, self.measured, dt)
            bias = controller.bias_deg
            self.trim = correction + bias
            history.append((self.target - self.measured, correction.copy(), bias))
        return history


def _run_plant(controller, offset_deg, cycles, target_deg=0.0, dt=DT):
    return _Plant(offset_deg, target_deg).run(controller, cycles, dt)


# ---------------------------------------------------------------------------
# Leak rate and closed-loop handoff
# ---------------------------------------------------------------------------


def test_leak_is_integral_output_over_tau():
    """One cycle accumulates exactly ``Ki·ierr · dt/tau`` into the bias."""
    controller = _make(Kp=0.0, Ki=1.0, bias_tau_s=10.0)
    # First step integrates err=1.0 for dt, leaving ierr = dt.
    controller.step(np.ones(N), np.zeros(N), DT)
    expected = 1.0 * DT * (DT / 10.0)
    np.testing.assert_allclose(controller.bias_deg, expected, rtol=1e-9)


def test_bias_absorbs_offset_and_integral_unwinds():
    """The bias converges on the persistent map error and the fast integrator
    hands it over — the point of the whole mechanism."""
    controller = _make(bias_tau_s=1.0)
    offset = 2.0
    _run_plant(controller, offset, cycles=3000)  # 30 s at tau = 1 s

    np.testing.assert_allclose(controller.bias_deg, offset, atol=0.02)
    integral_out = controller.get_state()["ierr_deg"] * 8.0
    assert np.all(np.abs(integral_out) < 0.05)


def test_handoff_leaves_no_visible_transient():
    """No explicit drain of the integral is needed: the feedback path unwinds
    it as the bias takes over. At the production time constant the handoff
    costs less pose error than one encoder LSB, so it is unobservable."""
    controller = _make(bias_tau_s=DEFAULT_BIAS_TAU_S)
    history = _run_plant(controller, 2.0, cycles=6000)  # 60 s
    # Skip the fast loop's own convergence (~1 s); the rest is the handoff.
    settled_errors = np.abs(np.array([err for err, _, _ in history[100:]]))
    assert settled_errors.max() < ENCODER_LSB_DEG


def test_disabled_when_tau_is_none():
    controller = _make(bias_tau_s=None)
    _run_plant(controller, 2.0, cycles=500)
    np.testing.assert_array_equal(controller.bias_deg, np.zeros(N))


# ---------------------------------------------------------------------------
# Gates
# ---------------------------------------------------------------------------


def test_saturated_channel_does_not_leak():
    """A blocked or externally-loaded joint pins the correction at its clamp;
    its integral is holding a force, not a map error, so nothing is banked."""
    controller = _make(Kp=1.0, Ki=8.0, correction_max_deg=1.0, i_clamp_deg=20.0)
    for _ in range(200):
        controller.step(np.full(N, 10.0), np.zeros(N), DT)
    np.testing.assert_array_equal(controller.bias_deg, np.zeros(N))


def test_wound_up_integral_does_not_leak():
    """A joint held off target by an external force winds its integral into
    the ±i_clamp without necessarily clamping the output; that trim is a held
    force, not a map error, so it must not be banked either."""
    controller = _make(
        Kp=0.0, Ki=8.0, correction_max_deg=100.0, i_clamp_deg=1.0, bias_tau_s=1.0
    )
    for _ in range(100):
        controller.step(np.full(N, 5.0), np.zeros(N), DT)
    np.testing.assert_allclose(controller.get_state()["ierr_deg"], 1.0)
    assert np.all(np.abs(controller.get_state()["last_correction_deg"]) < 100.0)

    pinned_bias = controller.bias_deg
    for _ in range(500):
        controller.step(np.full(N, 5.0), np.zeros(N), DT)
    np.testing.assert_array_equal(controller.bias_deg, pinned_bias)


def test_saturation_gate_is_per_joint():
    controller = _make(
        Kp=1.0,
        Ki=8.0,
        correction_max_deg=np.array([1.0, 20.0, 20.0, 20.0]),
        i_clamp_deg=20.0,
    )
    for _ in range(200):
        controller.step(np.full(N, 10.0), np.zeros(N), DT)
    bias = controller.bias_deg
    assert bias[0] == 0.0
    assert np.all(bias[1:] > 0.0)


def test_frozen_integral_also_freezes_the_leak():
    """A stale encoder freezes the integrator; banking a trim computed from
    angles that are no longer arriving would be worse than doing nothing."""
    controller = _make()
    plant = _Plant(2.0)
    plant.run(controller, 100)
    before = controller.bias_deg
    controller.freeze_integral()
    plant.run(controller, 100)
    np.testing.assert_array_equal(controller.bias_deg, before)
    controller.unfreeze_integral()
    plant.run(controller, 100)
    assert np.all(controller.bias_deg > before)


# ---------------------------------------------------------------------------
# Clamp, reset, validation
# ---------------------------------------------------------------------------


def test_bias_is_clamped_and_warns_once(caplog):
    joints = ["index_mcp", "index_pip", "middle_mcp", "middle_pip"]
    controller = _make(bias_tau_s=0.05, bias_max_deg=0.5, joint_names=joints)
    with caplog.at_level(logging.WARNING, logger="orca_core.control.joint_controller"):
        _run_plant(controller, 5.0, cycles=2000)

    np.testing.assert_allclose(np.abs(controller.bias_deg), 0.5)
    assert controller.get_state()["bias_at_clamp"].all()
    warnings = [r for r in caplog.records if "hit the ±0.5° clamp" in r.message]
    assert len(warnings) == N
    assert all(any(j in r.getMessage() for j in joints) for r in warnings)


def test_negative_offset_clamps_negative():
    controller = _make(bias_tau_s=0.05, bias_max_deg=0.5)
    _run_plant(controller, -5.0, cycles=2000)
    np.testing.assert_allclose(controller.bias_deg, -0.5)


def test_reset_keeps_bias_but_reset_bias_clears_it():
    """``reset()`` runs on every rebase; the bias is feed-forward knowledge
    about the hand, not transient loop state, so it must survive."""
    controller = _make()
    _run_plant(controller, 2.0, cycles=500)
    learned = controller.bias_deg
    assert np.all(learned > 0.0)

    controller.reset()
    np.testing.assert_array_equal(controller.bias_deg, learned)
    np.testing.assert_array_equal(controller.get_state()["ierr_deg"], np.zeros(N))

    controller.reset_bias()
    np.testing.assert_array_equal(controller.bias_deg, np.zeros(N))
    assert not controller.get_state()["bias_at_clamp"].any()


def test_reset_bias_rearms_the_clamp_warning(caplog):
    controller = _make(bias_tau_s=0.05, bias_max_deg=0.5)
    _run_plant(controller, 5.0, cycles=2000)
    controller.reset_bias()
    caplog.clear()
    with caplog.at_level(logging.WARNING, logger="orca_core.control.joint_controller"):
        _run_plant(controller, 5.0, cycles=2000)
    assert len([r for r in caplog.records if "clamp" in r.message]) == N


def test_constructor_validates_bias_parameters():
    with pytest.raises(ValueError, match="bias_tau_s"):
        JointController(num_joints=N, bias_tau_s=0.0)
    with pytest.raises(ValueError, match="bias_max_deg"):
        JointController(num_joints=N, bias_max_deg=-1.0)
    with pytest.raises(ValueError, match="joint_names"):
        JointController(num_joints=N, joint_names=["a", "b"])
