"""Unit tests for ``CascadedJointController``."""

from __future__ import annotations

import math

import numpy as np
import pytest

from orca_core.control import CascadedJointController


def _make(
    num_joints: int = 4,
    *,
    Kp=0.0,
    Ki=0.0,
    correction_max_rad=math.radians(20),
    i_clamp_rad=math.radians(20),
):
    controller = CascadedJointController(num_joints=num_joints)
    controller.set_gains(
        Kp=Kp, Ki=Ki,
        correction_max_rad=correction_max_rad,
        i_clamp_rad=i_clamp_rad,
    )
    return controller


def test_constructor_rejects_non_positive_num_joints():
    with pytest.raises(ValueError, match="num_joints"):
        CascadedJointController(num_joints=0)
    with pytest.raises(ValueError, match="num_joints"):
        CascadedJointController(num_joints=-1)


def test_proportional_per_channel_gain():
    controller = _make(Kp=np.array([0.1, 0.2, 0.3, 0.4]))
    target = np.full(4, 0.1)
    measured = np.zeros(4)
    out = controller.step(target, measured, dt=0.01)
    np.testing.assert_allclose(out, np.array([0.01, 0.02, 0.03, 0.04]))


def test_integral_accumulates_and_clamps():
    controller = _make(Ki=1.0, i_clamp_rad=0.05)
    target = np.full(4, 1.0)
    measured = np.zeros(4)
    for _ in range(3):
        controller.step(target, measured, dt=0.01)
    np.testing.assert_allclose(controller.get_state()["ierr_rad"], np.full(4, 0.03))
    for _ in range(1000):
        controller.step(target, measured, dt=0.01)
    np.testing.assert_allclose(controller.get_state()["ierr_rad"], np.full(4, 0.05))


def test_output_clipped_at_correction_max():
    controller = _make(Kp=1e6, correction_max_rad=math.radians(10))
    target = np.full(4, 1.0)
    out_pos = controller.step(target, np.zeros(4), dt=0.005)
    np.testing.assert_allclose(out_pos, np.full(4, math.radians(10)))
    out_neg = controller.step(-target, np.zeros(4), dt=0.005)
    np.testing.assert_allclose(out_neg, np.full(4, -math.radians(10)))


def test_conditional_integration_freezes_at_saturation_then_unwinds():
    """When the output saturates with the same sign as err, the integrator
    must stop winding; flipping the err sign must let it unwind again."""
    controller = _make(
        Ki=10.0, correction_max_rad=math.radians(10), i_clamp_rad=1e9,
    )
    target = np.full(4, 1.0)
    for _ in range(200):
        controller.step(target, np.zeros(4), dt=0.01)
    saturated_ierr = controller.get_state()["ierr_rad"].copy()
    assert np.all(saturated_ierr > 0)

    controller.step(-target, np.zeros(4), dt=0.01)
    assert np.all(controller.get_state()["ierr_rad"] < saturated_ierr)


def test_freeze_integral_halts_accumulation():
    controller = _make(Kp=0.1, Ki=1.0)
    target = np.full(4, 1.0)
    for _ in range(3):
        controller.step(target, np.zeros(4), dt=0.01)
    frozen = controller.get_state()["ierr_rad"].copy()
    assert np.any(frozen != 0)
    controller.freeze_integral()
    assert controller.integral_frozen
    controller.step(target, np.zeros(4), dt=0.01)
    np.testing.assert_allclose(controller.get_state()["ierr_rad"], frozen)
    controller.unfreeze_integral()
    assert not controller.integral_frozen


def test_reset_zeros_state():
    """``Kp=0.1`` and ``err=1`` keeps ``Kp*err = 0.1`` below the default
    ``correction_max_rad ≈ 0.349``, so conditional integration permits
    winding and ``ierr`` accumulates over five steps."""
    controller = _make(Kp=0.1, Ki=1.0)
    target = np.full(4, 1.0)
    for _ in range(5):
        controller.step(target, np.zeros(4), dt=0.01)
    assert np.any(controller.get_state()["ierr_rad"] != 0)
    controller.reset()
    state = controller.get_state()
    np.testing.assert_allclose(state["ierr_rad"], np.zeros(4))
    np.testing.assert_allclose(state["last_correction_rad"], np.zeros(4))


def test_dt_is_clamped_so_zero_dt_does_not_blow_up_integrator():
    controller = _make(Ki=1.0)
    controller.step(np.full(4, 1.0), np.zeros(4), dt=0.0)
    assert np.all(np.isfinite(controller.get_state()["ierr_rad"]))


def test_set_gains_rejects_negative_values():
    controller = CascadedJointController(num_joints=4)
    with pytest.raises(ValueError, match="Kp"):
        controller.set_gains(Kp=-1.0, Ki=0.0, correction_max_rad=0.1, i_clamp_rad=0.1)
    with pytest.raises(ValueError, match="correction_max_rad"):
        controller.set_gains(Kp=1.0, Ki=0.0, correction_max_rad=-0.1, i_clamp_rad=0.1)


def test_step_rejects_wrong_shape():
    controller = _make(num_joints=4, Kp=1.0)
    with pytest.raises(ValueError, match="target"):
        controller.step(np.zeros(3), np.zeros(4), dt=0.01)
    with pytest.raises(ValueError, match="measured"):
        controller.step(np.zeros(4), np.zeros(3), dt=0.01)
