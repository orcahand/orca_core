"""Unit tests for ``JointController``."""

from __future__ import annotations

import numpy as np
import pytest

from orca_core.control import JointController


def _make(
    num_joints: int = 4,
    *,
    Kp=0.0,
    Ki=0.0,
    correction_max_deg=20.0,
    i_clamp_deg=20.0,
):
    controller = JointController(num_joints=num_joints)
    controller.set_gains(
        Kp=Kp, Ki=Ki,
        correction_max_deg=correction_max_deg,
        i_clamp_deg=i_clamp_deg,
    )
    return controller


def test_constructor_rejects_non_positive_num_joints():
    with pytest.raises(ValueError, match="num_joints"):
        JointController(num_joints=0)


def test_proportional_per_channel_gain():
    controller = _make(Kp=np.array([0.1, 0.2, 0.3, 0.4]))
    out = controller.step(np.full(4, 0.1), np.zeros(4), dt=0.01)
    np.testing.assert_allclose(out, np.array([0.01, 0.02, 0.03, 0.04]))


def test_integral_accumulates_and_clamps():
    controller = _make(Ki=1.0, i_clamp_deg=0.05)
    target = np.full(4, 1.0)
    for _ in range(3):
        controller.step(target, np.zeros(4), dt=0.01)
    np.testing.assert_allclose(controller.get_state()["ierr_deg"], np.full(4, 0.03))
    for _ in range(1000):
        controller.step(target, np.zeros(4), dt=0.01)
    np.testing.assert_allclose(controller.get_state()["ierr_deg"], np.full(4, 0.05))


def test_output_clipped_at_correction_max():
    controller = _make(Kp=1e6, correction_max_deg=10.0)
    out_pos = controller.step(np.full(4, 1.0), np.zeros(4), dt=0.005)
    np.testing.assert_allclose(out_pos, np.full(4, 10.0))
    out_neg = controller.step(np.full(4, -1.0), np.zeros(4), dt=0.005)
    np.testing.assert_allclose(out_neg, np.full(4, -10.0))


def test_conditional_integration_freezes_at_saturation_then_unwinds():
    """When the output saturates with the same sign as err, the integrator
    must stop winding; flipping the err sign must let it unwind again."""
    controller = _make(Ki=10.0, correction_max_deg=10.0, i_clamp_deg=1e9)
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


def test_set_gains_rejects_negative_values():
    controller = JointController(num_joints=4)
    with pytest.raises(ValueError, match="Kp"):
        controller.set_gains(Kp=-1.0, Ki=0.0, correction_max_deg=0.1, i_clamp_deg=0.1)


def test_step_rejects_wrong_shape():
    controller = _make(num_joints=4, Kp=1.0)
    with pytest.raises(ValueError, match="target"):
        controller.step(np.zeros(3), np.zeros(4), dt=0.01)
    with pytest.raises(ValueError, match="measured"):
        controller.step(np.zeros(4), np.zeros(3), dt=0.01)
