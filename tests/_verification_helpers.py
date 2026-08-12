"""Shared fixtures for the producer-bench verification routines."""

import os
import shutil

import pytest

from orca_core import MockOrcaHand
from orca_core.hardware.sensing.constants import joint_encoder_polarity_for_side
from orca_core.utils import update_yaml

from tests._encoder_helpers import MockJointEncoderSource

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JOINT_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


@pytest.fixture
def joint_hand(tmp_path):
    """A connected mock hand with joint feedback enabled, plus its model dir."""
    config_path = tmp_path / "config.yaml"
    shutil.copy(JOINT_CONFIG, config_path)
    update_yaml(str(config_path), "use_joint_feedback", True)

    hand = MockOrcaHand(config_path=str(config_path))
    success, msg = hand.connect()
    assert success, msg
    try:
        yield hand, tmp_path
    finally:
        hand.stop_task()
        hand.disconnect()


def motor_direction(hand, joint):
    """Sign of the motor travel a positive joint move produces."""
    lower, upper = hand.config.joint_roms_dict[joint]
    idx = hand.config.motor_id_to_idx_dict[hand.config.joint_to_motor_map[joint]]
    at_lower = hand._joint_to_motor_pos({joint: lower})[idx]
    at_upper = hand._joint_to_motor_pos({joint: upper})[idx]
    return 1 if at_upper >= at_lower else -1


def encoder_source(hand, *, dead_slots=frozenset(), flip=()):
    """A mock encoder whose mounting matches this hand's polarity table.

    The mock synthesises counts from motor position, so a joint whose motor
    runs backwards relative to the joint needs its mock mounting flipped for
    the whole chain to agree with the table. ``flip`` inverts a joint on top
    of that, simulating a slot mounted the wrong way round.
    """
    table = joint_encoder_polarity_for_side(hand.config.type)
    polarities = {}
    for joint in hand.config.joint_to_motor_map:
        sign = table.get(joint, 1) * motor_direction(hand, joint)
        polarities[joint] = -sign if joint in flip else sign
    return MockJointEncoderSource(
        hand._motor_client,
        hand.config.joint_to_motor_map,
        polarities=polarities,
        dead_slots=dead_slots,
    )
