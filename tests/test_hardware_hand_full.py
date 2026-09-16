"""Lifecycle tests for ``OrcaHandFull`` — the hand that runs tactile sensing
and closed-loop joint feedback together.

The focus is the shared-link orchestration: when the tactile and encoder
streams resolve to the same port, both clients ride one
``HandSerialLink`` (demuxed by frame type), and ``disconnect`` tears the
whole stack down once without error.
"""

from __future__ import annotations

import dataclasses as dc
import os

import numpy as np
import pytest
import yaml

from orca_core.calibration import JointEncoderCal
from orca_core.control import JointLoopThread
from orca_core.hardware.sensing.constants import AUTO_ENC_NUM_JOINTS
from orca_core import JointFeedbackConnectError, MockOrcaHandFull

from tests._encoder_helpers import make_encoder_frame
from tests._helpers import wait_until


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RIGHT_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


@pytest.fixture
def full_config(tmp_path):
    """A config with both joint-encoder support (from orcahand-joint-right) and a
    tactile ``sensors:`` block, as a full hand requires."""
    with open(RIGHT_CONFIG) as f:
        cfg = yaml.safe_load(f)
    cfg["sensors"] = {
        "port": "auto",
        "finger_to_sensor_id": {
            "thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4,
        },
    }
    path = tmp_path / "config.yaml"
    with open(path, "w") as f:
        yaml.safe_dump(cfg, f)
    return str(path)


def make_full_hand(config_path: str) -> MockOrcaHandFull:
    """Mock full hand wired for a successful shared-link connect: the mock
    pins both 'auto' ports to the same in-memory port, so
    ``resolve_sensing_ports`` reports ``shared`` and both clients attach to
    one link (fed by the built-in encoder frame pump)."""
    frame = make_encoder_frame(
        raw_counts=np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    )

    class _Bound(MockOrcaHandFull):
        def _mock_encoder_frame(self) -> bytes:
            return frame

    hand = _Bound(config_path=config_path)

    motor_limits = {mid: [-0.5, 0.5] for mid in hand.config.motor_ids}
    ratios = {mid: 0.01 for mid in hand.config.motor_ids}
    encoder_cal = {
        joint: JointEncoderCal(enc_at_anchor_count=0)
        for joint in hand._encoder_backed_joints()
    }
    hand.calibration = dc.replace(
        hand.calibration,
        motor_limits_dict=motor_limits,
        joint_to_motor_ratios_dict=ratios,
        joint_encoder_calibration_dict=encoder_cal,
        calibrated=True,
        wrist_calibrated=True,
    )
    return hand


def test_full_connect_shares_one_link_for_both_streams(full_config):
    hand = make_full_hand(full_config)
    success, msg = hand.connect()
    try:
        assert success, msg
        # Joint loop is live.
        assert isinstance(hand._loop, JointLoopThread)
        assert hand._loop._thread is not None and hand._loop._thread.is_alive()
        # Tactile client is attached...
        assert hand._tactile_client is not None
        # ...to the *same* link as the encoder stream, and the dedicated
        # tactile link was never opened (so teardown closes the link once).
        assert hand._tactile_client._link is hand._encoder_link
        assert hand._tactile_link is None
        # Encoder frames are flowing through the shared link.
        wait_until(lambda: hand._encoder_client.get_latest() is not None)
    finally:
        hand.disconnect()


def test_full_disconnect_tears_down_cleanly(full_config):
    hand = make_full_hand(full_config)
    hand.connect()
    shared_link = hand._encoder_link

    hand.disconnect()

    assert hand._loop is None
    assert hand._controller is None
    assert hand._encoder_client is None
    assert hand._tactile_client is None
    assert hand._encoder_link is None
    assert not shared_link.is_connected


def test_full_connect_rolls_back_on_missing_encoder_calibration(full_config):
    """No encoder calibration → connect raises and leaves nothing running."""
    hand = make_full_hand(full_config)
    hand.calibration = dc.replace(
        hand.calibration, joint_encoder_calibration_dict={}
    )
    with pytest.raises(JointFeedbackConnectError, match="joint-encoder calibration"):
        hand.connect()
    assert hand._loop is None
    assert hand._encoder_client is None
    assert hand._tactile_client is None


@pytest.fixture
def left_full_config(full_config, tmp_path):
    with open(full_config) as f:
        cfg = yaml.safe_load(f)
    cfg["type"] = "left"
    left_dir = tmp_path / "left"
    left_dir.mkdir()
    left_path = left_dir / "config.yaml"
    with open(left_path, "w") as f:
        yaml.safe_dump(cfg, f)
    return str(left_path)


def test_full_connect_refuses_left_config(left_full_config):
    """Closed-loop control is unvalidated for left-hand assemblies: connect
    must refuse before opening the motor bus, any link, or the loop — and
    the refusal must name the concrete escape hatches."""
    from orca_core import JointFeedbackConnectError

    hand = make_full_hand(left_full_config)
    with pytest.raises(JointFeedbackConnectError) as excinfo:
        hand.connect()
    message = str(excinfo.value)
    assert "left" in message
    assert "connect(engage_feedback=False)" in message
    assert "orcahand-touch-left" in message
    assert not hand.is_connected()
    assert hand._loop is None
    assert hand._encoder_link is None
    assert hand._tactile_client is None
    assert hand._tactile_link is None


def test_full_left_connect_without_feedback_gets_motors_and_tactile(left_full_config):
    """``engage_feedback=False`` is the left-hand escape hatch: motors and
    tactile connect open-loop, and neither the encoder link nor the loop is
    touched."""
    hand = make_full_hand(left_full_config)
    success, msg = hand.connect(engage_feedback=False)
    try:
        assert success, msg
        assert hand.is_connected()
        assert hand._loop is None
        assert hand._encoder_link is None
        assert hand._encoder_client is None
        assert hand._tactile_client is not None
        assert hand.get_tactile_configuration() is not None
        hand.start_tactile_stream()
        hand.stop_tactile_stream()
    finally:
        hand.disconnect()


def test_full_connect_without_feedback_refuses_while_the_loop_runs(full_config):
    """``engage_feedback=False`` promises an open-loop hand; on a hand whose
    loop already runs it must say so instead of reporting success."""
    hand = make_full_hand(full_config)
    assert hand.connect()[0]
    try:
        success, msg = hand.connect(engage_feedback=False)
        assert not success
        assert "disconnect()" in msg
        assert hand._loop is not None
    finally:
        hand.disconnect()


def test_full_reconnect_open_loop_after_disconnect(full_config):
    """The documented recovery from the refusal above."""
    hand = make_full_hand(full_config)
    assert hand.connect()[0]
    hand.disconnect()
    success, msg = hand.connect(engage_feedback=False)
    try:
        assert success, msg
        assert hand._loop is None
        assert hand._tactile_client is not None
    finally:
        hand.disconnect()


def test_full_failed_motor_connect_preserves_sensors_only_tactile(
    full_config, monkeypatch
):
    """Sensors-first bring-up: tactile is live via connect_sensors_only()
    while the motors are unpowered. A failing motor connect must report the
    failure without tearing the live tactile session down."""
    from orca_core.hardware_hand import OrcaHand

    hand = make_full_hand(full_config)
    ok, _ = hand.connect_sensors_only()
    assert ok
    link, client = hand._tactile_link, hand._tactile_client

    monkeypatch.setattr(
        OrcaHand,
        "connect",
        lambda self, interactive=True: (False, "motor bus unpowered"),
    )
    try:
        success, msg = hand.connect()
        assert not success
        assert "motor bus unpowered" in msg
        assert hand._tactile_client is client
        assert hand._tactile_link is link and link.is_connected
        assert hand.get_tactile_configuration() is not None
    finally:
        monkeypatch.undo()
        hand.disconnect()


def test_full_second_connect_is_noop_and_orphans_nothing(full_config):
    """A second connect() must be a no-op success — not re-attach either
    stream and orphan the shared link, tactile client, or loop thread."""
    hand = make_full_hand(full_config)
    hand.connect()
    try:
        loop, thread = hand._loop, hand._loop._thread
        link, tactile = hand._encoder_link, hand._tactile_client

        ok, msg = hand.connect()
        assert ok and msg == "Already connected"
        assert hand._loop is loop
        assert hand._loop._thread is thread and thread.is_alive()
        assert hand._encoder_link is link
        assert hand._tactile_client is tactile
        assert hand._tactile_link is None
    finally:
        hand.disconnect()
    assert not thread.is_alive()


def test_full_connect_after_sensors_only_orphans_no_link(full_config):
    """connect() after connect_sensors_only() re-resolves the topology; the
    sensors-only tactile link must be closed, not silently orphaned while
    health accessors keep reporting on it."""
    hand = make_full_hand(full_config)
    ok, _ = hand.connect_sensors_only()
    assert ok
    sensors_only_link = hand._tactile_link
    assert sensors_only_link.is_connected

    ok, _ = hand.connect()
    try:
        assert ok
        assert not sensors_only_link.is_connected
        assert hand._tactile_link is None  # shared-link topology
        assert hand.get_tactile_link_health().connected
    finally:
        hand.disconnect()


def test_full_link_health_and_mock_seam_use_shared_link(full_config):
    """With both streams on one port, tactile health and the mock feed seam
    must resolve to the shared encoder link."""
    hand = make_full_hand(full_config)
    assert hand.get_tactile_link_health() is None
    assert hand.get_encoder_link_health() is None
    hand.connect()
    try:
        assert hand._tactile_link is None  # shared-link topology
        tactile_health = hand.get_tactile_link_health()
        encoder_health = hand.get_encoder_link_health()
        assert tactile_health is not None and tactile_health.connected
        assert encoder_health is not None and encoder_health.connected
        assert hand.tactile_mock_link is hand._encoder_link
    finally:
        hand.disconnect()
