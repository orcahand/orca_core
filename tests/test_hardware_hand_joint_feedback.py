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
from tests._helpers import wait_until


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


@pytest.fixture
def joint_feedback_config(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    return str(config_path)


@pytest.fixture
def left_joint_feedback_config(tmp_path):
    with open(REAL_CONFIG) as f:
        text = f.read()
    config_path = tmp_path / "config.yaml"
    config_path.write_text(text.replace("type: right", "type: left"))
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
    # Disconnect discards the hand's motor client; keep a handle to check
    # the teardown left the motors' operating modes untouched.
    motor_client = hand._motor_client
    modes_after_connect = dict(motor_client._operating_mode)
    loop_thread = hand._loop._thread

    hand.disconnect()

    assert hand._loop is None
    assert hand._controller is None
    assert hand._encoder_client is None
    assert hand._encoder_link is None
    assert not loop_thread.is_alive()
    assert dict(motor_client._operating_mode) == modes_after_connect


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


def test_skipped_joint_stays_in_reads_and_interpolated_moves(
    joint_feedback_config, caplog
):
    """A joint skipped from the loop (missing encoder anchor) must still be
    reported by get_joint_position() via the motor path and reached by
    interpolated (num_steps>1) moves, and the skip must be logged."""
    import dataclasses as dc

    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    victim = hand._encoder_backed_joints()[0]
    encoder_cal = dict(hand.calibration.joint_encoder_calibration_dict)
    del encoder_cal[victim]
    hand.calibration = dc.replace(
        hand.calibration, joint_encoder_calibration_dict=encoder_cal
    )

    with caplog.at_level(logging.WARNING, logger="orca_core.hardware_hand_sensing"):
        ok, _ = hand.connect()
    try:
        assert ok
        assert hand.loop_skipped_joints == [victim]
        assert any("open-loop" in r.getMessage() for r in caplog.records)

        positions = hand.get_joint_position().as_dict()
        assert victim in positions

        victim_motor = hand.config.joint_to_motor_map[victim]
        rom = hand.config.joint_roms_dict[victim]
        current = positions[victim]
        target = rom[0] if abs(current - rom[0]) >= abs(current - rom[1]) else rom[1]
        pos_before = hand._motor_client._pos[victim_motor]
        hand.set_joint_positions(
            OrcaJointPositions.from_dict({victim: target}),
            num_steps=5, step_size=0.0,
        )
        assert hand._motor_client._pos[victim_motor] != pos_before
    finally:
        hand.disconnect()


def test_second_connect_is_noop_and_orphans_nothing(joint_feedback_config):
    """A second connect() must be a no-op success — not re-attach the
    encoder stack and orphan the first loop thread and link."""
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    try:
        loop, thread = hand._loop, hand._loop._thread
        link, client = hand._encoder_link, hand._encoder_client
        pump = hand._encoder_pump

        ok, msg = hand.connect()
        assert ok and msg == "Already connected"
        assert hand._loop is loop
        assert hand._loop._thread is thread and thread.is_alive()
        assert hand._encoder_link is link
        assert hand._encoder_client is client
        assert hand._encoder_pump is pump
    finally:
        hand.disconnect()
    assert not thread.is_alive()


def test_connect_refuses_left_hand_config(left_joint_feedback_config):
    """Closed-loop control is unvalidated for left-hand assemblies: connect
    must refuse before opening the motor bus, any link, or the loop — and
    the refusal must name the concrete escape hatches."""
    hand = make_calibrated_joint_feedback_hand(left_joint_feedback_config)
    with pytest.raises(JointFeedbackConnectError) as excinfo:
        hand.connect()
    message = str(excinfo.value)
    assert "left" in message
    assert "connect(engage_feedback=False)" in message
    assert "orcahand-left" in message
    assert not hand.is_connected()
    assert hand._loop is None
    assert hand._encoder_client is None
    assert hand._encoder_link is None
    assert hand._encoder_pump is None


def test_left_connect_without_feedback_opens_motor_bus_only(
    left_joint_feedback_config,
):
    """``engage_feedback=False`` is the left-hand escape hatch: the motor bus
    opens for open-loop control and the encoder stack stays untouched."""
    hand = make_calibrated_joint_feedback_hand(left_joint_feedback_config)
    success, msg = hand.connect(engage_feedback=False)
    try:
        assert success, msg
        assert hand.is_connected()
        assert hand._loop is None
        assert hand._encoder_client is None
        assert hand._encoder_link is None
        assert hand._encoder_pump is None
    finally:
        hand.disconnect()


def test_connect_without_feedback_refuses_while_the_loop_runs(joint_feedback_config):
    """``engage_feedback=False`` promises an open-loop hand; on a hand whose
    loop already runs it must say so instead of reporting success — otherwise
    the caller goes on to a maintenance routine the loop then refuses."""
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    assert hand.connect()[0]
    try:
        success, msg = hand.connect(engage_feedback=False)
        assert not success
        assert "disconnect()" in msg
        assert hand._loop is not None
    finally:
        hand.disconnect()


def test_open_loop_connect_after_disconnect_admits_calibration(joint_feedback_config):
    """The documented recovery: disconnect, reconnect open-loop, calibrate."""
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    assert hand.connect()[0]
    hand.disconnect()
    success, msg = hand.connect(engage_feedback=False)
    try:
        assert success, msg
        assert hand._loop is None
        hand.calibrate(joints=["index_mcp"])
    finally:
        hand.disconnect()


def test_tension_jitter_and_calibrate_refused_while_loop_runs(joint_feedback_config):
    """The maintenance routines drive the same motors as the 100 Hz loop and
    must be refused while it runs."""
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    try:
        with pytest.raises(RuntimeError, match="tension"):
            hand.tension()
        with pytest.raises(RuntimeError, match="jitter"):
            hand.jitter()
        with pytest.raises(RuntimeError, match="calibrate"):
            hand.calibrate()
    finally:
        hand.disconnect()


def test_init_joints_tolerates_connect_admitted_partial_calibration(
    joint_feedback_config,
):
    """init_joints() must honor what connect() admitted: skipped joints stay
    open-loop instead of triggering a refused calibrate() mid-sequence, and
    force_calibrate is rejected up front, before torque is touched."""
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
    hand.connect()
    try:
        assert hand.loop_skipped_joints == [victim]
        hand.init_joints(move_to_neutral=False)

        hand.disable_torque()
        torque_before = dict(hand._motor_client._torque_enabled)
        with pytest.raises(RuntimeError, match="force_calibrate"):
            hand.init_joints(force_calibrate=True)
        assert dict(hand._motor_client._torque_enabled) == torque_before
    finally:
        hand.disconnect()


def test_measured_joints_and_correction_raise_after_estop(joint_feedback_config):
    """After the watchdog e-stop the loop's measurement is frozen; the facade
    must raise instead of serving that dead data as if it were live."""
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    try:
        assert hand.get_measured_joints()

        hand._loop._trigger_estop("test")
        wait_until(lambda: not hand._loop._thread.is_alive())

        with pytest.raises(RuntimeError, match="e-stopped"):
            hand.get_measured_joints()
        with pytest.raises(RuntimeError, match="e-stopped"):
            hand.get_loop_correction()
    finally:
        hand.disconnect()


def test_encoder_link_health_reports_port_death(joint_feedback_config):
    """The public health accessor must surface link liveness and the latched
    port-dead state without callers reaching into private link attributes."""
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    assert hand.get_encoder_link_health() is None
    hand.connect()
    try:
        health = hand.get_encoder_link_health()
        assert health.connected
        assert not health.port_dead
        assert health.port_error is None
        assert health.stats.frames_routed

        hand._encoder_link.simulate_port_death()
        wait_until(lambda: hand.get_encoder_link_health().port_dead)
        health = hand.get_encoder_link_health()
        assert not health.connected
        assert health.port_error
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
