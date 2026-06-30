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
from typing import List

import numpy as np
import pytest
import yaml

from orca_core.calibration import JointEncoderCal
from orca_core.control import JointLoopThread
from orca_core.hardware.sensing.constants import AUTO_ENC_NUM_JOINTS
from orca_core.hardware_hand_joint_feedback import MockOrcaHandFull

from tests._encoder_helpers import make_encoder_frame
from tests._hand_feedback_helpers import _LinkFramePump
from tests.conftest import wait_until


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


class _PumpedMockOrcaHandFull(MockOrcaHandFull):
    """``MockOrcaHandFull`` that feeds AA A9 frames to the encoder link so
    ``start_stream`` sees a fresh frame within its first-frame timeout."""

    _frame_for_pump: bytes = b""

    def _create_encoder_link(self, port: str):
        link = super()._create_encoder_link(port)
        pump = _LinkFramePump(link, type(self)._frame_for_pump)
        self._pumps.append(pump)
        pump.start()
        return link

    def disconnect(self):
        for pump in self._pumps:
            pump.stop()
        self._pumps.clear()
        return super().disconnect()


def make_full_hand(config_path: str) -> MockOrcaHandFull:
    """Mock full hand wired for a successful shared-link connect: tactile and
    encoder ports point at the same mock device, so ``resolve_sensing_ports``
    reports ``shared`` and both clients attach to one link."""
    frame = make_encoder_frame(
        raw_counts=np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    )

    class _Bound(_PumpedMockOrcaHandFull):
        _frame_for_pump = frame

    hand = _Bound(config_path=config_path)
    hand._pumps: List[_LinkFramePump] = []
    hand.config = dc.replace(
        hand.config,
        encoder_serial_port="/dev/ttyMOCK",
        sensor_port="/dev/ttyMOCK",
    )

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
    with pytest.raises(Exception):
        hand.connect()
    assert hand._loop is None
    assert hand._encoder_client is None
    assert hand._tactile_client is None
