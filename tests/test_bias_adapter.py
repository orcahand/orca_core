"""Tests for ``BiasAdapter``: gated transfer of persistent PI trim into a
per-joint feed-forward bias — each gate individually, the clamp with its
warning, drain bookkeeping, and reset."""

from __future__ import annotations

import logging

import numpy as np
import pytest

from orca_core.control import BiasAdapter, JointController


N = 4
DT = 0.01


def _adapter(**overrides) -> BiasAdapter:
    params = dict(
        tau_s=10.0,
        max_bias_deg=3.0,
        deadband_deg=0.05,
        settle_s=0.5,
        vel_max_deg_s=2.0,
        vel_filter_tau_s=0.05,
    )
    params.update(overrides)
    return BiasAdapter(N, **params)


def _settle(adapter, target, measured, integral, saturated=None, cycles=None):
    """Run enough quasistatic cycles to pass the settle gate (plus the
    first-cycle initialisation), returning the last transfer."""
    if saturated is None:
        saturated = np.zeros(N, dtype=bool)
    if cycles is None:
        cycles = int(adapter._settle_s / DT) + 2
    transferred = np.zeros(N)
    for _ in range(cycles):
        transferred = adapter.step(target, measured, integral, saturated, DT)
    return transferred


def test_transfers_integral_fraction_when_all_gates_pass():
    adapter = _adapter()
    target = np.zeros(N)
    measured = np.zeros(N)
    integral = np.full(N, 1.5)
    transferred = _settle(adapter, target, measured, integral)
    np.testing.assert_allclose(transferred, integral * DT / 10.0)
    assert np.all(adapter.bias_deg > 0)


def test_first_cycle_transfers_nothing():
    adapter = _adapter(settle_s=0.0)
    out = adapter.step(
        np.zeros(N), np.zeros(N), np.ones(N), np.zeros(N, dtype=bool), DT
    )
    np.testing.assert_array_equal(out, 0.0)


def test_target_change_resets_settle_gate_per_joint():
    adapter = _adapter()
    target = np.zeros(N)
    measured = np.zeros(N)
    integral = np.full(N, 1.0)
    _settle(adapter, target, measured, integral)

    moved_target = target.copy()
    moved_target[1] = 5.0
    transferred = adapter.step(
        moved_target, measured, integral, np.zeros(N, dtype=bool), DT
    )
    assert transferred[1] == 0.0
    assert all(transferred[i] > 0.0 for i in range(N) if i != 1)


def test_velocity_gate_blocks_moving_joint():
    adapter = _adapter()
    target = np.zeros(N)
    integral = np.full(N, 1.0)
    saturated = np.zeros(N, dtype=bool)
    measured = np.zeros(N)
    transferred = np.zeros(N)
    for i in range(int(adapter._settle_s / DT) + 2):
        measured[2] = 0.1 * i  # joint 2 moves at 10 deg/s
        transferred = adapter.step(target, measured, integral, saturated, DT)
    assert transferred[2] == 0.0
    assert all(transferred[i] > 0.0 for i in range(N) if i != 2)


def test_saturated_gate_blocks_transfer():
    adapter = _adapter()
    saturated = np.zeros(N, dtype=bool)
    saturated[0] = True
    transferred = _settle(
        adapter, np.zeros(N), np.zeros(N), np.full(N, 1.0), saturated
    )
    assert transferred[0] == 0.0
    assert np.all(transferred[1:] > 0.0)


def test_deadband_blocks_small_integral():
    adapter = _adapter(deadband_deg=0.5)
    integral = np.array([0.4, 0.6, -0.4, -0.6])
    transferred = _settle(adapter, np.zeros(N), np.zeros(N), integral)
    assert transferred[0] == 0.0 and transferred[2] == 0.0
    assert transferred[1] > 0.0 and transferred[3] < 0.0


def test_bias_clamps_and_warns_once(caplog):
    adapter = _adapter(tau_s=0.02, max_bias_deg=1.0, settle_s=0.0)
    target = np.zeros(N)
    integral = np.full(N, 10.0)
    saturated = np.zeros(N, dtype=bool)
    with caplog.at_level(logging.WARNING):
        for _ in range(50):
            adapter.step(target, target, integral, saturated, DT)
    assert np.all(adapter.bias_deg <= 1.0)
    clamp_warnings = [r for r in caplog.records if "clamp" in r.getMessage()]
    assert len(clamp_warnings) == N  # once per joint, not per cycle


def test_reset_zeroes_state():
    adapter = _adapter()
    _settle(adapter, np.zeros(N), np.zeros(N), np.full(N, 1.0))
    assert np.any(adapter.bias_deg != 0.0)
    adapter.reset()
    assert np.all(adapter.bias_deg == 0.0)
    state = adapter.get_state()
    assert np.all(state["settle_elapsed_s"] == 0.0)
    # After reset the next cycle re-initialises and transfers nothing.
    out = adapter.step(
        np.zeros(N), np.zeros(N), np.ones(N), np.zeros(N, dtype=bool), DT
    )
    np.testing.assert_array_equal(out, 0.0)


def test_shape_and_param_validation():
    with pytest.raises(ValueError):
        BiasAdapter(0)
    with pytest.raises(ValueError):
        BiasAdapter(N, joint_names=["a"])
    with pytest.raises(ValueError):
        BiasAdapter(N, tau_s=0.0)
    adapter = _adapter()
    with pytest.raises(ValueError):
        adapter.step(np.zeros(3), np.zeros(N), np.zeros(N), np.zeros(N, bool), DT)


def test_drain_matches_transfer_keeps_command_continuous():
    """Transferring δ into the bias while draining δ from the integral must
    leave correction + bias unchanged (up to the one-cycle overlap)."""
    controller = JointController(N)
    controller.set_gains(Kp=0.5, Ki=2.0, correction_max_deg=20.0, i_clamp_deg=20.0)
    adapter = _adapter(settle_s=0.0)

    target = np.full(N, 5.0)
    measured = np.zeros(N)
    adapter.step(target, measured, controller.integral_output,
                 np.zeros(N, bool), DT)  # init cycle
    for _ in range(100):
        correction = controller.step(target, measured, DT)
        pre_total = controller.integral_output + adapter.bias_deg
        transferred = adapter.step(
            target, measured, controller.integral_output, np.zeros(N, bool), DT
        )
        controller.drain_integral(transferred)
        # Draining must exactly cancel the bias gain in the integral term,
        # so their sum — the trim actually applied — is unchanged.
        post_total = controller.integral_output + adapter.bias_deg
        np.testing.assert_allclose(post_total, pre_total, atol=1e-12)
    assert np.all(adapter.bias_deg > 0.0)


def test_ki_zero_never_transfers():
    controller = JointController(N)
    controller.set_gains(Kp=1.0, Ki=0.0, correction_max_deg=20.0, i_clamp_deg=20.0)
    adapter = _adapter(settle_s=0.0)
    target = np.full(N, 5.0)
    measured = np.zeros(N)
    for _ in range(50):
        controller.step(target, measured, DT)
        transferred = adapter.step(
            target, measured, controller.integral_output, np.zeros(N, bool), DT
        )
        controller.drain_integral(transferred)
    assert np.all(adapter.bias_deg == 0.0)
