"""Lifecycle and routing tests for ``OrcaHandJointFeedback``: connect
starts the loop without touching motor operating modes, disconnect tears
down cleanly, calibration gate raises on missing encoder calibration,
and the joint-position public API routes encoder joints through the loop
and the wrist through the inherited motor-position path.
"""

from __future__ import annotations

import logging
import os
import shutil

import pytest

from orca_core.constants import CURRENT, MODE_MAP, WRIST
from orca_core.control import JointController, JointLoopThread
from orca_core import JointFeedbackConnectError
from orca_core.joint_position import OrcaJointPositions

from tests._hand_feedback_helpers import make_calibrated_joint_feedback_hand
from tests.conftest import wait_until


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


@pytest.fixture
def joint_feedback_config(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    return str(config_path)


def test_connect_starts_loop_without_touching_operating_modes(joint_feedback_config):
    """The loop runs as an outer PI on top of the motor's internal position
    PID
    """
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    success, _ = hand.connect()
    try:
        assert success
        assert isinstance(hand._loop, JointLoopThread)
        assert isinstance(hand._controller, JointController)
        assert hand._loop._thread is not None and hand._loop._thread.is_alive()

        for motor_id in hand._encoder_motor_ids():
            assert (
                hand._motor_client._operating_mode[motor_id]
                != MODE_MAP[CURRENT]
            )

        wait_until(lambda: hand._encoder_client.get_latest() is not None)
    finally:
        hand.disconnect()


def test_disconnect_stops_loop_and_clears_state(joint_feedback_config):
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    modes_after_connect = dict(hand._motor_client._operating_mode)
    loop_thread = hand._loop._thread

    hand.disconnect()

    assert hand._loop is None
    assert hand._controller is None
    assert hand._encoder_client is None
    assert hand._encoder_link is None
    assert not loop_thread.is_alive()
    assert dict(hand._motor_client._operating_mode) == modes_after_connect


def test_connect_raises_when_encoder_calibration_missing(joint_feedback_config):
    hand = make_calibrated_joint_feedback_hand(
        joint_feedback_config, install_encoder_calibration=False,
    )
    try:
        with pytest.raises(JointFeedbackConnectError, match="joint-encoder calibration"):
            hand.connect()
        assert hand._loop is None
        assert hand._encoder_client is None
        assert hand._encoder_link is None
    finally:
        hand.disconnect()


def test_connect_skips_uncalibrated_joints_and_keeps_loop(joint_feedback_config):
    """One joint with incomplete motor calibration must not block the
    feedback tier: the loop closes on the calibrated rest, the victim is
    reported as skipped, and its targets take the open-loop motor path."""
    import dataclasses as dc

    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    victim = hand._encoder_backed_joints()[0]
    victim_motor = hand.config.joint_to_motor_map[victim]
    hand.calibration = dc.replace(
        hand.calibration,
        joint_to_motor_ratios_dict={
            **hand.calibration.joint_to_motor_ratios_dict, victim_motor: 0.0,
        },
        calibrated=False,
    )

    ok, msg = hand.connect()
    try:
        assert ok
        assert f"motor-only: {victim}" in msg
        assert hand.loop_skipped_joints == [victim]
        assert victim not in hand.loop_joint_names
        assert set(hand.loop_joint_names) == (
            set(hand._encoder_backed_joints()) - {victim}
        )

        # The skipped joint routes open-loop; a loop joint still hits the loop.
        loop_joint = hand.loop_joint_names[0]
        hand.set_joint_positions(
            OrcaJointPositions.from_dict({victim: 10.0, loop_joint: 30.0}),
        )
        assert victim not in hand._loop._joint_names
        loop_idx = hand._loop._joint_names.index(loop_joint)
        assert hand._loop._target_deg[loop_idx] == pytest.approx(30.0)
    finally:
        hand.disconnect()


def test_set_joint_positions_routes_wrist_and_encoder_joints(joint_feedback_config):
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    try:
        encoder_joint = hand._encoder_backed_joints()[0]
        wrist_motor_id = hand.config.joint_to_motor_map[WRIST]
        wrist_pos_before = hand._motor_client._pos[wrist_motor_id]

        hand.set_joint_positions(
            OrcaJointPositions.from_dict({encoder_joint: 30.0, WRIST: 5.0}),
        )

        loop_idx = hand._loop._joint_names.index(encoder_joint)
        assert hand._loop._target_deg[loop_idx] == pytest.approx(30.0)
        assert hand._motor_client._pos[wrist_motor_id] != wrist_pos_before
    finally:
        hand.disconnect()


def test_get_joint_positions_wrist_comes_from_motor_position(joint_feedback_config):
    """Encoder joints come from the loop's measurement; the wrist value
    has to come from the motor-position read (the wrist isn't in the
    loop). Drive the wrist motor to a known position post-connect and
    check the converted joint angle propagates through _get_joint_positions.
    """
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    try:
        wrist_motor_id = hand.config.joint_to_motor_map[WRIST]
        wrist_idx = hand.config.motor_id_to_idx_dict[wrist_motor_id]

        # Park the wrist motor at a non-trivial position so the converted
        # joint angle is distinct from the post-connect default.
        motor_pos = hand.get_motor_pos().copy()
        motor_pos[wrist_idx] = 0.25
        hand._motor_client._pos[wrist_motor_id] = 0.25

        # Compute the expected joint angle through the inherited mapping.
        expected_wrist = hand._motor_to_joint_pos(motor_pos)[WRIST]

        positions = hand._get_joint_positions().as_dict()
        assert WRIST in positions
        assert positions[WRIST] == pytest.approx(expected_wrist, abs=1e-9)
    finally:
        hand.disconnect()


def test_estop_falls_back_to_open_loop_joint_io(joint_feedback_config):
    """After the watchdog e-stop the dead loop must not swallow commands or
    serve frozen angles: joint I/O reroutes through the inherited open-loop
    motor path."""
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    try:
        encoder_joint = hand._encoder_backed_joints()[0]
        motor_id = hand.config.joint_to_motor_map[encoder_joint]

        hand._loop._trigger_estop("test")
        wait_until(lambda: not hand._loop._thread.is_alive())

        # Commands reach the motors directly instead of the dead loop.
        pos_before = hand._motor_client._pos[motor_id]
        target_before = hand._loop._target_deg.copy()
        rom_lower = hand.config.joint_roms_dict[encoder_joint][0]
        hand.set_joint_positions(
            OrcaJointPositions.from_dict({encoder_joint: rom_lower})
        )
        assert hand._motor_client._pos[motor_id] != pos_before
        assert hand._loop._target_deg == pytest.approx(target_before)

        # Reads come from the motors, not the loop's frozen measurement.
        hand._motor_client._pos[motor_id] = 0.3
        expected = hand._motor_to_joint_pos(hand.get_motor_pos())[encoder_joint]
        got = hand._get_joint_positions().as_dict()[encoder_joint]
        assert got == pytest.approx(expected, abs=1e-9)
    finally:
        hand.disconnect()


def test_teardown_warns_when_loop_join_times_out(joint_feedback_config, caplog):
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    orig_stop = hand._loop.stop
    hand._loop.stop = lambda timeout=1.0: orig_stop(timeout) and False

    with caplog.at_level(logging.WARNING, logger="orca_core.hardware_hand_sensing"):
        hand.disconnect()

    assert any("did not stop" in record.message for record in caplog.records)
