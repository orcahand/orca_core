"""Unit tests for ``JointPIDController``."""

from __future__ import annotations

import numpy as np

from orca_core.control import JointPIDController
from orca_core.control.constants import MAX_LOOP_DT_S


def _make(num_joints=4, *, Kp=0.0, Ki=0.0, Kd=0.0, i_clamp_mA=1e9, i_max_mA=1e9, tau_s=0.0):
    pid = JointPIDController(num_joints=num_joints)
    pid.set_gains(
        Kp=Kp, Ki=Ki, Kd=Kd, i_clamp_mA=i_clamp_mA, i_max_mA=i_max_mA,
        derivative_filter_tau_s=tau_s,
    )
    pid.set_target(np.zeros(num_joints))
    return pid


def test_proportional_per_channel_gain():
    """P-term math + per-channel gain broadcast in one shot."""
    pid = _make(Kp=np.array([100.0, 200.0, 300.0, 400.0]))
    pid.set_target(np.full(4, 0.1))
    out = pid.step(np.zeros(4), dt=0.005)
    np.testing.assert_allclose(out, np.array([10.0, 20.0, 30.0, 40.0]))


def test_integral_accumulates_and_clamps():
    pid = _make(Ki=1.0, i_clamp_mA=0.05)
    pid.set_target(np.full(4, 1.0))
    for _ in range(3):
        pid.step(np.zeros(4), dt=0.01)
    np.testing.assert_allclose(pid.get_state()["ierr"], np.full(4, 0.03))
    for _ in range(1000):
        pid.step(np.zeros(4), dt=0.01)
    np.testing.assert_allclose(pid.get_state()["ierr"], np.full(4, 0.05))


def test_output_clipped_at_i_max():
    pid = _make(Kp=1e6, i_max_mA=300.0)
    pid.set_target(np.full(4, 1.0))
    np.testing.assert_allclose(pid.step(np.zeros(4), dt=0.005), np.full(4, 300.0))
    pid.set_target(np.full(4, -1.0))
    np.testing.assert_allclose(pid.step(np.zeros(4), dt=0.005), np.full(4, -300.0))


def test_reset_is_bumpless():
    """After ``reset()`` with target == measured, the next step outputs ~0."""
    pid = _make(Kp=200.0, Ki=10.0, Kd=5.0)
    measured = np.full(4, 0.7)
    pid.set_target(np.full(4, 0.2))
    for _ in range(5):
        pid.step(np.zeros(4), dt=0.01)
    pid.set_target(measured.copy())
    pid.reset()
    np.testing.assert_allclose(pid.step(measured, dt=0.005), np.zeros(4))
    np.testing.assert_allclose(pid.get_target(), measured)


def test_freeze_integral_halts_accumulation():
    pid = _make(Kp=10.0, Ki=1.0)
    pid.set_target(np.full(4, 1.0))
    for _ in range(3):
        pid.step(np.zeros(4), dt=0.01)
    frozen = pid.get_state()["ierr"].copy()
    pid.freeze_integral()
    pid.step(np.zeros(4), dt=0.01)
    np.testing.assert_allclose(pid.get_state()["ierr"], frozen)


def test_dt_clamped_at_floor_and_ceiling():
    """``dt=0`` must not divide-by-zero; huge ``dt`` clamps so the
    derivative-on-measurement term stays finite when the joint moves."""
    pid = _make(Kd=1.0)
    pid.set_target(np.full(4, 1.0))
    pid.step(np.zeros(4), dt=0.0)
    assert np.isfinite(pid.get_state()["last_u_mA"]).all()

    pid.reset()
    pid.step(np.zeros(4), dt=0.005)            # establishes prev_measured
    pid.step(np.full(4, 1.0), dt=10.0)          # measurement steps by 1 rad
    # derr = -(1.0 - 0.0) / clamp(10.0) = -1/MAX_LOOP_DT_S = -20.0; Kd=1 → -20 mA
    np.testing.assert_allclose(
        pid.get_state()["last_u_mA"], np.full(4, -1.0 / MAX_LOOP_DT_S)
    )


def test_derivative_on_measurement_no_kick_on_target_step():
    """A pure target step must not inject any D-term contribution.
    Validates derivative-on-measurement: derr is driven by motion only."""
    pid = _make(Kd=10.0)  # No Kp/Ki — output should be zero on a target step
    pid.step(np.zeros(4), dt=0.005)
    pid.set_target(np.full(4, 0.5))
    out = pid.step(np.zeros(4), dt=0.005)
    np.testing.assert_allclose(out, np.zeros(4))


def test_derivative_filter_attenuates_single_lsb_pulse():
    """A one-sample 1-LSB encoder noise pulse on a steady target must not
    propagate at full Kd into the output. With τ=5 ms at dt=5 ms,
    α = 0.5 → first-cycle attenuation 50%."""
    pid = _make(Kd=100.0, tau_s=0.005)
    for _ in range(3):
        pid.step(np.zeros(4), dt=0.005)
    pulse = np.full(4, 0.0004)  # ~1 LSB
    out = pid.step(pulse, dt=0.005)
    raw_derr_term = -100.0 * (0.0004 / 0.005)        # = -8.0 mA at full Kd, no filter
    np.testing.assert_allclose(out, np.full(4, 0.5 * raw_derr_term))


def test_conditional_integration_freezes_at_saturation_then_unwinds():
    """The integrator must stop winding once the output saturates with the
    same sign as err, then unwind again when the err sign flips. With
    Ki=10, i_max=10, dt=0.01, err=1: ierr saturates at 1.0 after 100 steps."""
    pid = _make(Ki=10.0, i_clamp_mA=1e9, i_max_mA=10.0)
    pid.set_target(np.full(4, 1.0))
    for _ in range(200):
        pid.step(np.zeros(4), dt=0.01)
    saturated_ierr = pid.get_state()["ierr"].copy()
    np.testing.assert_allclose(saturated_ierr, np.full(4, 1.0))

    pid.set_target(np.full(4, -1.0))
    pid.step(np.zeros(4), dt=0.01)
    assert np.all(pid.get_state()["ierr"] < saturated_ierr)
