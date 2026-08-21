"""Unit tests for ``JointController``."""

from __future__ import annotations

import threading

import numpy as np
import pytest

from orca_core.control import JointController


def _make(
    num_joints: int = 4,
    *,
    Kp=0.0,
    Ki=0.0,
    correction_max_deg=20.0,
):
    controller = JointController(num_joints=num_joints)
    controller.set_gains(
        Kp=Kp, Ki=Ki, correction_max_deg=correction_max_deg,
    )
    return controller


def test_constructor_rejects_non_positive_num_joints():
    with pytest.raises(ValueError, match="num_joints"):
        JointController(num_joints=0)


def test_proportional_per_channel_gain():
    controller = _make(Kp=np.array([0.1, 0.2, 0.3, 0.4]))
    out = controller.step(np.full(4, 0.1), np.zeros(4), dt=0.01)
    np.testing.assert_allclose(out, np.array([0.01, 0.02, 0.03, 0.04]))


def test_integral_accumulates_then_freezes_at_the_output_clamp():
    """Conditional integration is the only anti-windup, so stored integral is
    bounded by correction_max_deg / Ki — the invariant that makes
    correction_max_deg, not Ki, set how much windup a hold can bank."""
    controller = _make(Kp=0.0, Ki=1.0, correction_max_deg=0.05)
    target = np.full(4, 1.0)
    for _ in range(3):
        controller.step(target, np.zeros(4), dt=0.01)
    np.testing.assert_allclose(controller.get_state()["ierr_deg"], np.full(4, 0.03))
    for _ in range(1000):
        controller.step(target, np.zeros(4), dt=0.01)
    np.testing.assert_allclose(controller.get_state()["ierr_deg"], np.full(4, 0.05))


def test_ki_zero_channel_banks_no_integral():
    """A pure-P phase must not wind the integrator: without this, raising Ki
    at runtime (the tuning flow) discharges the banked error as a correction
    pinned at the clamp for a long unwind."""
    controller = _make(num_joints=1, Kp=0.5, Ki=0.0, correction_max_deg=15.0)
    for _ in range(6000):  # 60 s at 100 Hz, err 5°
        controller.step(np.array([5.0]), np.zeros(1), dt=0.01)
    assert controller.get_state()["ierr_deg"][0] == 0.0

    controller.set_gains(Kp=0.5, Ki=5.0, correction_max_deg=15.0)
    out = controller.step(np.array([5.0]), np.zeros(1), dt=0.01)
    assert out[0] == pytest.approx(0.5 * 5.0, abs=0.5)


def test_retained_integral_contribution_is_independent_of_ki():
    """Ki changes how fast the integrator winds, not how much it retains."""
    correction_max = 12.0
    for ki in (2.0, 6.0, 20.0):
        controller = _make(num_joints=1, Kp=0.0, Ki=ki,
                           correction_max_deg=correction_max)
        for _ in range(4000):
            controller.step(np.array([5.0]), np.zeros(1), dt=0.01)
        contribution = ki * controller.get_state()["ierr_deg"][0]
        assert contribution == pytest.approx(correction_max, rel=0.02)


def test_output_clipped_at_correction_max():
    controller = _make(Kp=1e6, correction_max_deg=10.0)
    out_pos = controller.step(np.full(4, 1.0), np.zeros(4), dt=0.005)
    np.testing.assert_allclose(out_pos, np.full(4, 10.0))
    out_neg = controller.step(np.full(4, -1.0), np.zeros(4), dt=0.005)
    np.testing.assert_allclose(out_neg, np.full(4, -10.0))


def test_conditional_integration_freezes_at_saturation_then_unwinds():
    """When the output saturates with the same sign as err, the integrator
    must stop winding; flipping the err sign must let it unwind again."""
    controller = _make(Ki=10.0, correction_max_deg=10.0)
    target = np.full(4, 1.0)
    for _ in range(200):
        controller.step(target, np.zeros(4), dt=0.01)
    saturated_ierr = controller.get_state()["ierr_deg"].copy()
    assert np.all(saturated_ierr > 0)

    controller.step(-target, np.zeros(4), dt=0.01)
    assert np.all(controller.get_state()["ierr_deg"] < saturated_ierr)


def test_freeze_integral_halts_accumulation():
    controller = _make(Kp=0.1, Ki=1.0)
    target = np.full(4, 1.0)
    for _ in range(3):
        controller.step(target, np.zeros(4), dt=0.01)
    frozen = controller.get_state()["ierr_deg"].copy()
    assert np.any(frozen != 0)
    controller.freeze_integral()
    controller.step(target, np.zeros(4), dt=0.01)
    np.testing.assert_allclose(controller.get_state()["ierr_deg"], frozen)


def test_reset_zeros_state():
    controller = _make(Kp=0.1, Ki=1.0)
    target = np.full(4, 1.0)
    for _ in range(5):
        controller.step(target, np.zeros(4), dt=0.01)
    assert np.any(controller.get_state()["ierr_deg"] != 0)
    controller.reset()
    state = controller.get_state()
    np.testing.assert_allclose(state["ierr_deg"], np.zeros(4))
    np.testing.assert_allclose(state["last_correction_deg"], np.zeros(4))


@pytest.mark.parametrize(
    "bad_field",
    ["Kp", "Ki", "correction_max_deg"],
)
def test_set_gains_rejects_negative_values(bad_field):
    """Every gain must be non-negative; the validator must reject each one
    individually so we don't regress to "only Kp is checked"."""
    controller = JointController(num_joints=4)
    kwargs = {"Kp": 0.1, "Ki": 0.1, "correction_max_deg": 0.1}
    kwargs[bad_field] = -1.0
    with pytest.raises(ValueError, match=bad_field):
        controller.set_gains(**kwargs)


def test_step_rejects_wrong_shape():
    controller = _make(num_joints=4, Kp=1.0)
    with pytest.raises(ValueError, match="target"):
        controller.step(np.zeros(3), np.zeros(4), dt=0.01)
    with pytest.raises(ValueError, match="measured"):
        controller.step(np.zeros(4), np.zeros(3), dt=0.01)


def test_reset_and_set_gains_are_safe_during_concurrent_steps():
    """``reset()``/``set_gains()`` run from another thread while the loop
    thread is inside ``step()``; the integrator must stay within its clamp
    and no cycle may observe half-installed gains."""
    # Kp*err + Ki*ierr saturates at 20.0 with err=5.0, so ierr cannot pass 1.5.
    correction_max = 20.0
    max_ierr = (correction_max - 1.0 * 5.0) / 10.0
    controller = _make(Kp=1.0, Ki=10.0, correction_max_deg=correction_max)
    stop = threading.Event()
    errors: list[Exception] = []

    def stepper():
        target = np.full(4, 5.0)
        zeros = np.zeros(4)
        try:
            while not stop.is_set():
                controller.step(target, zeros, dt=0.01)
        except Exception as exc:  # surfaced in the main thread below
            errors.append(exc)

    thread = threading.Thread(target=stepper)
    thread.start()
    try:
        for _ in range(300):
            controller.reset()
            controller.set_gains(
                Kp=1.0, Ki=10.0, correction_max_deg=correction_max
            )
    finally:
        stop.set()
        thread.join(timeout=5.0)

    assert not errors
    assert not thread.is_alive()
    assert np.all(np.abs(controller.get_state()["ierr_deg"]) <= max_ierr + 1e-9)
