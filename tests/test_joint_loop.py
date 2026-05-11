"""Tests for ``JointLoopThread``: bumpless start, full data path through
encoder → controller → vectorised joint→motor → ``write_desired_pos``,
watchdog tiers, agreement with the open-loop kinematics, and the jitter
monitor.
"""

from __future__ import annotations

import logging
import os
import shutil

import numpy as np
import pytest

from orca_core.control import JointController, JointLoopThread
from orca_core.control.constants import (
    WATCHDOG_DROP_TORQUE_MS,
    WATCHDOG_HOLD_MS,
    WATCHDOG_STOP_LOOP_MS,
    WATCHDOG_WARN_MS,
)

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


def _make_loop(
    hand,
    encoder_source,
    *,
    Kp=0.1,
    Ki=0.0,
    correction_max_deg=15.0,
    target_hz=100,
):
    controller = JointController(num_joints=len(hand._encoder_backed_joints()))
    controller.set_gains(
        Kp=Kp, Ki=Ki,
        correction_max_deg=correction_max_deg,
        i_clamp_deg=correction_max_deg,
    )
    return JointLoopThread(hand, encoder_source, controller, target_hz=target_hz)


def _static_at_zero(hand, freshness_ms: float = 0.0) -> StaticEncoderSource:
    return StaticEncoderSource(
        encoder_reading_from_joint_angles(
            {j: 0.0 for j in hand._encoder_backed_joints()}
        ),
        freshness_ms=freshness_ms,
    )


def _expected_motor_pos(hand, joint: str, joint_command_deg: float) -> float:
    """Mirror :meth:`OrcaHand._joint_to_motor_pos`: joint_roms is degrees,
    joint_to_motor_ratios is motor-rad per joint-deg."""
    motor_id = hand.config.joint_to_motor_map[joint]
    lower = hand.motor_limits_dict[motor_id][0]
    ratio = hand.calibration.joint_to_motor_ratios_dict[motor_id]
    if hand.config.joint_inversion_dict.get(joint, False):
        rom_upper = hand.config.joint_roms_dict[joint][1]
        base = lower + (rom_upper - joint_command_deg) * ratio
    else:
        rom_lower = hand.config.joint_roms_dict[joint][0]
        base = lower + (joint_command_deg - rom_lower) * ratio
    return base + hand._wrap_offsets_dict.get(motor_id, 0.0)


def test_bumpless_first_step_writes_base_motor_target_with_no_correction(calibrated_hand):
    """At ``prime_for_step`` the target latches to the measured pose, so
    the first ``step_once`` produces a zero correction and writes the
    base motor target for the (decoded) measured pose."""
    measured_angle_deg = 3.0
    encoder = StaticEncoderSource(
        encoder_reading_from_joint_angles(
            {j: measured_angle_deg for j in calibrated_hand._encoder_backed_joints()}
        )
    )
    loop = _make_loop(calibrated_hand, encoder, Kp=1.0)
    loop.prime_for_step()
    primed_targets = {
        joint: loop._target_deg[loop._joint_names.index(joint)]
        for joint in calibrated_hand._encoder_backed_joints()
    }
    loop.step_once(dt=0.01)

    correction = loop.get_correction()
    for joint in calibrated_hand._encoder_backed_joints():
        assert correction[joint] == pytest.approx(0.0, abs=1e-9)
        motor_id = calibrated_hand.config.joint_to_motor_map[joint]
        expected = _expected_motor_pos(calibrated_hand, joint, primed_targets[joint])
        actual = calibrated_hand._motor_client._pos[motor_id]
        assert actual == pytest.approx(expected, abs=1e-6)


def test_step_once_applies_correction_to_motor_target(calibrated_hand):
    """Full data path: encoder → controller → vectorised joint→motor →
    ``write_desired_pos``. Each motor command equals the kinematics of the
    corrected joint command, with inversion respected."""
    encoder = _static_at_zero(calibrated_hand)
    Kp, target_offset_deg = 0.5, 5.0
    loop = _make_loop(calibrated_hand, encoder, Kp=Kp)
    loop.prime_for_step()
    loop.set_target(
        {j: target_offset_deg for j in calibrated_hand._encoder_backed_joints()}
    )
    loop.step_once(dt=0.01)

    expected_correction = Kp * target_offset_deg
    correction = loop.get_correction()
    for joint in calibrated_hand._encoder_backed_joints():
        assert correction[joint] == pytest.approx(expected_correction, abs=1e-9)
        motor_id = calibrated_hand.config.joint_to_motor_map[joint]
        expected_motor = _expected_motor_pos(
            calibrated_hand, joint, target_offset_deg + expected_correction
        )
        assert calibrated_hand._motor_client._pos[motor_id] == pytest.approx(
            expected_motor, abs=1e-6
        )


def test_step_once_with_no_reading_writes_nothing(calibrated_hand):
    loop = _make_loop(calibrated_hand, StaticEncoderSource(reading=None))
    loop.prime_for_step()
    pos_before = dict(calibrated_hand._motor_client._pos)
    loop.step_once(dt=0.01)

    stats = loop.get_stats()
    assert stats["cycles_no_reading"] == 1
    assert stats["cycles_ok"] == 0
    assert calibrated_hand._motor_client._pos == pos_before


def test_tier1_warns_with_rate_limit_but_continues_normal_control(calibrated_hand, caplog):
    encoder = _static_at_zero(calibrated_hand, freshness_ms=WATCHDOG_WARN_MS + 5)
    loop = _make_loop(calibrated_hand, encoder, Kp=0.5, Ki=1.0)
    loop.prime_for_step()
    loop.set_target({j: 5.0 for j in calibrated_hand._encoder_backed_joints()})

    with caplog.at_level(logging.WARNING):
        for _ in range(50):
            loop.step_once(dt=0.001)

    assert np.all(loop._controller.get_state()["ierr_deg"] > 0.0)
    freshness_records = [
        r for r in caplog.records
        if "freshness" in r.getMessage().lower() and r.levelno == logging.WARNING
    ]
    assert 1 <= len(freshness_records) <= 3


def test_tier2_freezes_integrator_and_recovers(calibrated_hand):
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder, Kp=0.05, Ki=1.0)
    loop.prime_for_step()
    loop.set_target({j: 5.0 for j in calibrated_hand._encoder_backed_joints()})
    for _ in range(3):
        loop.step_once(dt=0.005)
    ierr_before = loop._controller.get_state()["ierr_deg"].copy()

    encoder.freshness_ms = WATCHDOG_HOLD_MS + 10
    for _ in range(10):
        loop.step_once(dt=0.005)
    np.testing.assert_allclose(loop._controller.get_state()["ierr_deg"], ierr_before)
    assert loop._controller.integral_frozen

    encoder.freshness_ms = 1.0
    loop.step_once(dt=0.005)
    assert not loop._controller.integral_frozen


def test_tier3_zeroes_trim_writes_base_motor_target_only(calibrated_hand):
    """At >200 ms freshness the trim is zeroed: write the base motor
    target for the (uncorrected) joint target. The motor PID holds the
    un-trimmed pose."""
    encoder = _static_at_zero(
        calibrated_hand, freshness_ms=WATCHDOG_DROP_TORQUE_MS + 50
    )
    loop = _make_loop(calibrated_hand, encoder, Kp=10.0)
    loop.prime_for_step()
    target_deg = 4.0
    loop.set_target({j: target_deg for j in calibrated_hand._encoder_backed_joints()})
    loop.step_once(dt=0.005)

    for joint in calibrated_hand._encoder_backed_joints():
        motor_id = calibrated_hand.config.joint_to_motor_map[joint]
        expected = _expected_motor_pos(calibrated_hand, joint, target_deg)
        assert calibrated_hand._motor_client._pos[motor_id] == pytest.approx(
            expected, abs=1e-6
        )
    assert loop.get_stats()["e_stops"] >= 1


def test_tier4_sets_fallback_active(calibrated_hand):
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    loop.prime_for_step()

    encoder.freshness_ms = WATCHDOG_STOP_LOOP_MS + 100
    loop.step_once(dt=0.005)

    assert loop.get_stats()["fallback_active"] is True


def test_motor_pos_matches_open_loop_kinematics(calibrated_hand):
    """The loop's vectorised joint→motor mapping must agree with
    :meth:`OrcaHand._joint_to_motor_pos` for the same joint target. Guards
    against re-introducing a unit-scale bug on motor writes."""
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    loop.prime_for_step()

    encoder_joints = calibrated_hand._encoder_backed_joints()
    target_deg = {
        joint: deg
        for joint, deg in zip(encoder_joints, [5.0, -5.0, 28.0, -28.0][: len(encoder_joints)])
    }
    target_array = np.array(
        [target_deg[j] for j in loop._joint_names], dtype=np.float64
    )
    loop_motor = loop._joint_to_motor_pos(target_array)

    open_loop_motor = calibrated_hand._joint_to_motor_pos(target_deg)
    motor_id_to_idx = calibrated_hand.config.motor_id_to_idx_dict
    for idx, joint in enumerate(loop._joint_names):
        motor_id = loop._motor_ids[idx]
        expected = open_loop_motor[motor_id_to_idx[motor_id]]
        assert loop_motor[idx] == pytest.approx(expected, abs=1e-9)


def test_jitter_monitor_estops_after_pathological_streak(calibrated_hand):
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    loop.prime_for_step()

    loop._record_loop_period(loop._target_period * 12.0)
    assert loop.get_stats()["fallback_active"] is False
    loop._record_loop_period(loop._target_period)
    for _ in range(4):
        loop._record_loop_period(loop._target_period * 12.0)
    assert loop.get_stats()["fallback_active"] is False
    loop._record_loop_period(loop._target_period * 12.0)
    assert loop.get_stats()["fallback_active"] is True
