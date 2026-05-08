"""Watchdog and jitter-monitor tests for ``JointLoopThread``.

Encoder-freshness tiers: 10 / 50 / 200 / 1000 ms.
Loop-period jitter monitor: warn after 10 consecutive >2× target,
e-stop on a single >10× target.
"""

from __future__ import annotations

import logging
import os
import shutil

import numpy as np
import pytest

from orca_core.control import JointPIDController
from orca_core.control.constants import (
    WATCHDOG_DROP_TORQUE_MS,
    WATCHDOG_HOLD_MS,
    WATCHDOG_STOP_LOOP_MS,
    WATCHDOG_WARN_MS,
)
from orca_core.control.joint_loop import JointLoopThread

from tests._loop_helpers import (
    StaticEncoderSource,
    encoder_reading_from_joint_angles,
    make_calibrated_hand,
)


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand_right", "config.yaml"
)


@pytest.fixture
def calibrated_hand(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    hand = make_calibrated_hand(str(config_path))
    yield hand
    hand.disconnect()


def _make_loop(hand, encoder_source, *, Kp=200.0, Ki=10.0, target_hz=200):
    pid = JointPIDController(num_joints=len(hand._encoder_backed_joints()))
    pid.set_gains(Kp=Kp, Ki=Ki, Kd=0.0, i_clamp_mA=1e9, i_max_mA=500.0)
    loop = JointLoopThread(hand, encoder_source, pid, target_hz=target_hz)
    loop.prime_for_step()
    loop.set_target({j: 0.1 for j in hand._encoder_backed_joints()})
    return loop


def _static_zero(hand, freshness_ms: float = 0.0) -> StaticEncoderSource:
    return StaticEncoderSource(
        encoder_reading_from_joint_angles(
            {j: 0.0 for j in hand._encoder_backed_joints()}
        ),
        freshness_ms=freshness_ms,
    )


def _jitter_records(caplog) -> list:
    return [
        r for r in caplog.records
        if "jitter" in r.getMessage().lower() and r.levelno == logging.WARNING
    ]


def _freshness_records(caplog) -> list:
    return [
        r for r in caplog.records
        if "freshness" in r.getMessage().lower() and r.levelno == logging.WARNING
    ]


def test_tier1_warns_with_rate_limit_but_continues_normal_control(calibrated_hand, caplog):
    encoder = _static_zero(calibrated_hand, freshness_ms=WATCHDOG_WARN_MS + 5)
    loop = _make_loop(calibrated_hand, encoder)

    with caplog.at_level(logging.WARNING):
        for _ in range(50):
            loop.step_once(dt=0.001)

    assert np.all(loop._pid.get_state()["ierr"] > 0.0)
    assert any(abs(v) > 1.0 for v in calibrated_hand._motor_client.get_last_currents_mA().values())
    assert 1 <= len(_freshness_records(caplog)) <= 3


def test_tier2_freezes_integral(calibrated_hand):
    encoder = _static_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    for _ in range(3):
        loop.step_once(dt=0.005)
    ierr_before = loop._pid.get_state()["ierr"].copy()

    encoder.freshness_ms = WATCHDOG_HOLD_MS + 10
    for _ in range(10):
        loop.step_once(dt=0.005)
    np.testing.assert_allclose(loop._pid.get_state()["ierr"], ierr_before)
    assert loop._pid.integral_frozen


def test_tier2_recovery_unfreezes_integral(calibrated_hand):
    """When freshness drops back below the hold threshold the integrator
    must resume; otherwise a transient dropout latches the loop into a
    degraded mode forever."""
    encoder = _static_zero(calibrated_hand, freshness_ms=WATCHDOG_HOLD_MS + 10)
    loop = _make_loop(calibrated_hand, encoder)
    for _ in range(5):
        loop.step_once(dt=0.005)
    assert loop._pid.integral_frozen

    encoder.freshness_ms = 1.0
    for _ in range(5):
        loop.step_once(dt=0.005)
    assert not loop._pid.integral_frozen
    assert np.all(loop._pid.get_state()["ierr"] > 0.0)


def test_tier3_writes_zero_current_each_cycle(calibrated_hand):
    encoder = _static_zero(calibrated_hand, freshness_ms=WATCHDOG_DROP_TORQUE_MS + 50)
    loop = _make_loop(calibrated_hand, encoder)
    for _ in range(5):
        loop.step_once(dt=0.005)

    last = calibrated_hand._motor_client.get_last_currents_mA()
    for joint in calibrated_hand._encoder_backed_joints():
        motor_id = calibrated_hand.config.joint_to_motor_map[joint]
        assert last[motor_id] == 0.0
    assert loop.get_stats()["e_stops"] >= 1


def test_tier4_sets_fallback_and_restores_modes(calibrated_hand):
    """Tier-4 captures motor modes at prime time and writes them back on
    e-stop, so a freshness timeout returns the bus to whatever mode the
    rest of the system expected."""
    encoder_motor_ids = [
        calibrated_hand.config.joint_to_motor_map[j]
        for j in calibrated_hand._encoder_backed_joints()
    ]
    pre_modes = calibrated_hand._motor_client.get_operating_modes()
    encoder = _static_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)

    # Loop has snapshotted the pre-loop modes; simulate an external mode change
    # that the e-stop must undo.
    calibrated_hand._motor_client.set_operating_mode_per_motor(
        encoder_motor_ids, [0] * len(encoder_motor_ids)
    )
    encoder.freshness_ms = WATCHDOG_STOP_LOOP_MS + 100
    loop.step_once(dt=0.005)

    assert loop.get_stats()["fallback_active"] is True
    restored = calibrated_hand._motor_client.get_operating_modes()
    for mid in encoder_motor_ids:
        assert restored[mid] == pre_modes[mid]


def test_jitter_monitor_warns_after_streak_and_estops_on_pathological(calibrated_hand, caplog):
    """Three contracts in one test: the streak resets on a fast cycle (so
    healthy hands don't warn after a single hiccup); ten consecutive slow
    cycles emit a warning; a single >10× target cycle triggers e-stop."""
    encoder = _static_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    period = loop._target_period

    with caplog.at_level(logging.WARNING):
        for _ in range(5):
            loop._record_loop_period(period * 3.0)
        loop._record_loop_period(period)
        for _ in range(5):
            loop._record_loop_period(period * 3.0)
    assert _jitter_records(caplog) == []

    caplog.clear()
    with caplog.at_level(logging.WARNING):
        for _ in range(15):
            loop._record_loop_period(period * 3.0)
    assert len(_jitter_records(caplog)) >= 1

    loop2 = _make_loop(calibrated_hand, _static_zero(calibrated_hand))
    loop2._record_loop_period(loop2._target_period * 12.0)
    assert loop2.get_stats()["fallback_active"] is True
