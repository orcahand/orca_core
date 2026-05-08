"""Tests for ``JointLoopThread``: bumpless start, full data path through
encoder → PID → ``sync_write_current``, and thread lifecycle. Watchdog
tiers and the jitter monitor live in ``test_joint_loop_watchdog.py``.
"""

from __future__ import annotations

import os
import shutil
import time

import pytest

from orca_core.control import JointPIDController
from orca_core.control.joint_loop import JointLoopThread

from tests._loop_helpers import (
    CoupledPlantEncoderSource,
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


def _make_loop(hand, encoder_source, *, Kp=200.0, Ki=0.0, Kd=0.0, target_hz=200):
    pid = JointPIDController(num_joints=len(hand._encoder_backed_joints()))
    pid.set_gains(Kp=Kp, Ki=Ki, Kd=Kd, i_clamp_mA=1e9, i_max_mA=500.0)
    return JointLoopThread(hand, encoder_source, pid, target_hz=target_hz)


def _static_at_zero(hand) -> StaticEncoderSource:
    return StaticEncoderSource(
        encoder_reading_from_joint_angles(
            {j: 0.0 for j in hand._encoder_backed_joints()}
        )
    )


def test_bumpless_first_step_writes_zero_currents(calibrated_hand):
    """At ``prime_for_step`` the PID target latches to the measured pose, so
    the next ``step_once`` produces no torque even with non-zero gains."""
    encoder = StaticEncoderSource(
        encoder_reading_from_joint_angles(
            {j: 0.3 for j in calibrated_hand._encoder_backed_joints()}
        )
    )
    loop = _make_loop(calibrated_hand, encoder)
    loop.prime_for_step()
    loop.step_once(dt=0.005)

    last = calibrated_hand._motor_client.get_last_currents_mA()
    for joint in calibrated_hand._encoder_backed_joints():
        motor_id = calibrated_hand.config.joint_to_motor_map[joint]
        assert last[motor_id] == pytest.approx(0.0, abs=1e-9)


def test_step_once_applies_motor_polarity(calibrated_hand):
    """The full data path: encoder → PID → ``sync_write_current``. Each
    motor's commanded mA equals ``polarity_sign * Kp * err``, where
    ``polarity_sign`` comes from ``joint_inversion_dict``."""
    encoder = _static_at_zero(calibrated_hand)
    Kp, target_offset = 200.0, 0.1
    loop = _make_loop(calibrated_hand, encoder, Kp=Kp)
    loop.prime_for_step()
    loop.set_target({j: target_offset for j in calibrated_hand._encoder_backed_joints()})
    loop.step_once(dt=0.005)

    last = calibrated_hand._motor_client.get_last_currents_mA()
    for joint in calibrated_hand._encoder_backed_joints():
        motor_id = calibrated_hand.config.joint_to_motor_map[joint]
        sign = -1.0 if calibrated_hand.config.joint_inversion_dict.get(joint) else 1.0
        assert last[motor_id] == pytest.approx(sign * Kp * target_offset, abs=1e-9)


def test_step_once_with_no_reading_writes_nothing(calibrated_hand):
    loop = _make_loop(calibrated_hand, StaticEncoderSource(reading=None))
    loop.prime_for_step()
    loop.step_once(dt=0.005)

    stats = loop.get_stats()
    assert stats["cycles_no_reading"] == 1
    assert stats["cycles_ok"] == 0
    assert calibrated_hand._motor_client.get_last_currents_mA() == {}


def test_step_response_against_mock_plant(calibrated_hand):
    """Closed-loop sanity: with a mass-damper plant in the feedback path,
    the joints should converge toward a non-zero target. Validates the
    encoder → PID → motor → plant → encoder loop, not controller tuning.
    """
    encoder = CoupledPlantEncoderSource(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder, Kp=300.0, Ki=20.0, Kd=2.0)
    loop.prime_for_step()
    loop.set_target({j: 0.2 for j in calibrated_hand._encoder_backed_joints()})

    dt = 0.005
    for _ in range(400):
        loop.step_once(dt=dt)
        encoder.advance(dt)

    for joint, value in encoder.joint_pos.items():
        assert value == pytest.approx(0.2, abs=0.05), f"{joint} did not converge"


def test_thread_starts_runs_and_stops(calibrated_hand):
    loop = _make_loop(calibrated_hand, _static_at_zero(calibrated_hand))
    loop.start()
    time.sleep(0.05)
    assert loop.stop(timeout=1.0)
    assert loop.get_stats()["cycles_ok"] > 0


def test_wrist_motor_excluded_from_current_writes(calibrated_hand):
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    loop.prime_for_step()
    loop.set_target({j: 0.1 for j in calibrated_hand._encoder_backed_joints()})
    loop.step_once(dt=0.005)

    wrist_motor_id = calibrated_hand.config.joint_to_motor_map["wrist"]
    assert wrist_motor_id not in calibrated_hand._motor_client.get_last_currents_mA()
