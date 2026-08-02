"""Literal pins on the shipped hardware wiring tables.

These tables describe how the hardware is physically assembled, so nothing in
the code can derive them: a wrong entry is a wrong hand, not a failing
computation. Every other test reads these tables to build its expectations, so
this file is the only place their values are stated independently. Update a
literal here only alongside a hardware change, never to make a test pass.
"""

from orca_core.hardware.feetech_client import POSITION_DIRECTION
from orca_core.hardware.sensing.constants import (
    ENCODER_SLOT_TO_JOINT,
    JOINT_ENCODER_POLARITY,
    JOINT_ENCODER_POLARITY_BY_SIDE,
    JOINT_TO_ENCODER_SLOT,
)


def test_joint_to_encoder_slot_matches_the_wiring_harness():
    """Slot index of each joint encoder on the connector board."""
    assert JOINT_TO_ENCODER_SLOT == {
        "thumb_dip": 0,
        "thumb_mcp": 1,
        "thumb_abd": 2,
        "thumb_cmc": 3,
        "index_pip": 4,
        "index_mcp": 5,
        "index_abd": 6,
        "middle_pip": 7,
        "middle_mcp": 8,
        "middle_abd": 9,
        "ring_pip": 10,
        "ring_mcp": 11,
        "ring_abd": 12,
        "pinky_pip": 13,
        "pinky_mcp": 14,
        "pinky_abd": 15,
        "wrist": 16,
    }


def test_encoder_slot_to_joint_inverts_without_collisions():
    assert ENCODER_SLOT_TO_JOINT == {v: k for k, v in JOINT_TO_ENCODER_SLOT.items()}
    assert len(ENCODER_SLOT_TO_JOINT) == len(JOINT_TO_ENCODER_SLOT)


def test_right_hand_encoder_polarity_matches_the_validated_assembly():
    """Sign of each encoder, fixed by mounting and magnet orientation."""
    assert JOINT_ENCODER_POLARITY == {
        "thumb_cmc": -1,
        "thumb_abd": -1,
        "thumb_mcp": 1,
        "thumb_dip": -1,
        "index_abd": 1,
        "index_mcp": 1,
        "index_pip": -1,
        "middle_abd": 1,
        "middle_mcp": 1,
        "middle_pip": -1,
        "ring_abd": 1,
        "ring_mcp": 1,
        "ring_pip": -1,
        "pinky_abd": 1,
        "pinky_mcp": 1,
        "pinky_pip": -1,
        "wrist": 1,
    }


def test_only_the_right_side_has_a_validated_polarity_table():
    assert set(JOINT_ENCODER_POLARITY_BY_SIDE) == {"right"}
    assert JOINT_ENCODER_POLARITY_BY_SIDE["right"] is JOINT_ENCODER_POLARITY


def test_every_polarity_entry_has_an_encoder_slot():
    assert set(JOINT_ENCODER_POLARITY) <= set(JOINT_TO_ENCODER_SLOT)


def test_feetech_position_direction_is_inverted():
    """Feetech servos count position opposite to the joint convention."""
    assert POSITION_DIRECTION == -1
