"""Per-side encoder polarity in ``OrcaHand._raw_to_joint_angle``.

Each side must decode exactly as its own measured polarity table dictates;
a side with no table must fail loudly at decode time instead of silently
reusing another side's signs.
"""

import dataclasses as dc
import os
import shutil

import numpy as np
import pytest
from orca_core import MockOrcaHand
from orca_core.calibration import JointEncoderCal
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    ENCODER_LSB_DEG,
    JOINT_ENCODER_POLARITY,
    JOINT_ENCODER_POLARITY_LEFT,
    JOINT_TO_ENCODER_SLOT,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "v2")
JOINT_MODEL_CONFIG = os.path.join(MODELS_DIR, "orcahand-joint-right", "config.yaml")
LEFT_JOINT_MODEL_CONFIG = os.path.join(
    MODELS_DIR, "orcahand-joint-left", "config.yaml"
)

# thumb_cmc and index_abd carry opposite signs on the two sides, so a hand that
# borrowed the other side's table decodes them backwards and fails.
DECODED_JOINTS = ["thumb_cmc", "index_mcp", "pinky_pip"]
LEFT_DECODED_JOINTS = ["thumb_cmc", "index_abd", "index_mcp", "pinky_pip"]


def _anchor_count(index: int) -> int:
    return 1000 + 100 * index


def _hand_with_anchors(config_path, tmp_path, joints):
    shutil.copy(config_path, tmp_path / "config.yaml")
    hand = MockOrcaHand(config_path=str(tmp_path / "config.yaml"))
    encoder_cal = {
        joint: JointEncoderCal(enc_at_anchor_count=_anchor_count(i))
        for i, joint in enumerate(joints)
    }
    hand.calibration = dc.replace(
        hand.calibration, joint_encoder_calibration_dict=encoder_cal
    )
    return hand


@pytest.fixture
def right_hand(tmp_path):
    return _hand_with_anchors(JOINT_MODEL_CONFIG, tmp_path, DECODED_JOINTS)


@pytest.fixture
def left_hand(tmp_path):
    return _hand_with_anchors(
        LEFT_JOINT_MODEL_CONFIG, tmp_path, LEFT_DECODED_JOINTS
    )


def _raw_frame(joints=DECODED_JOINTS) -> np.ndarray:
    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    for i, joint in enumerate(joints):
        raw[JOINT_TO_ENCODER_SLOT[joint]] = _anchor_count(i) + 37
    return raw


def test_right_hand_decode_wires_slot_anchor_and_rom_per_joint(right_hand):
    """Each joint decodes from its own slot, anchor and ROM upper bound.

    The expected angle is written out in degrees rather than re-derived through
    ``encoder_to_joint_angle``, so a bug in that math fails here too. The table
    values themselves are pinned in test_hardware_constants.py.
    """
    assert right_hand.config.type == "right"
    signs = {JOINT_ENCODER_POLARITY[j] for j in DECODED_JOINTS}
    assert signs == {1, -1}, "regression joints must exercise both signs"

    angles = right_hand._raw_to_joint_angle(_raw_frame())
    assert set(angles) == set(DECODED_JOINTS)

    for joint in DECODED_JOINTS:
        anchor_angle = right_hand.config.joint_roms_dict[joint][1]
        offset_deg = JOINT_ENCODER_POLARITY[joint] * 37 * ENCODER_LSB_DEG
        assert angles[joint] == pytest.approx(anchor_angle + offset_deg)


def test_left_hand_decode_uses_the_left_polarity_table(left_hand):
    """A left hand decodes with its own signs, not the right hand's.

    ``thumb_cmc`` and ``index_abd`` are signed oppositely on the two sides, so
    a hand that fell back to the right table gets them backwards here.
    """
    assert left_hand.config.type == "left"
    flipped = [j for j in LEFT_DECODED_JOINTS
               if JOINT_ENCODER_POLARITY_LEFT[j] != JOINT_ENCODER_POLARITY[j]]
    assert flipped, "regression joints must include a side-dependent sign"

    angles = left_hand._raw_to_joint_angle(_raw_frame(LEFT_DECODED_JOINTS))
    assert set(angles) == set(LEFT_DECODED_JOINTS)

    for joint in LEFT_DECODED_JOINTS:
        anchor_angle = left_hand.config.joint_roms_dict[joint][1]
        offset_deg = JOINT_ENCODER_POLARITY_LEFT[joint] * 37 * ENCODER_LSB_DEG
        assert angles[joint] == pytest.approx(anchor_angle + offset_deg)


@pytest.mark.parametrize("side", ["middle", None])
def test_unrecognised_side_raises_at_decode_time(right_hand, side):
    """Decoding without a measured table names the side and the fix."""
    right_hand.config = dc.replace(right_hand.config, type=side)
    with pytest.raises(ValueError, match="polarity") as excinfo:
        right_hand._raw_to_joint_angle(_raw_frame())
    assert str(side) in str(excinfo.value)


def test_unrecognised_side_without_encoder_calibration_decodes_nothing(right_hand):
    """An empty calibration never consults the polarity table, so the
    calibration anchor pass (which stores raw counts only) works on any side."""
    right_hand.config = dc.replace(right_hand.config, type="middle")
    right_hand.calibration = dc.replace(
        right_hand.calibration, joint_encoder_calibration_dict={}
    )
    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    assert right_hand._raw_to_joint_angle(raw) == {}
