"""Per-side encoder polarity in ``OrcaHand._raw_to_joint_angle``.

Right hands must decode exactly as the validated right-hand polarity table
dictates; sides without a validated table must fail loudly at decode time
instead of silently reusing right-hand signs.
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
    JOINT_ENCODER_POLARITY,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware.sensing.encoder_protocol import encoder_to_joint_angle

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JOINT_MODEL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)

DECODED_JOINTS = ["thumb_cmc", "index_mcp", "pinky_pip"]


def _anchor_count(index: int) -> int:
    return 1000 + 100 * index


@pytest.fixture
def right_hand(tmp_path):
    shutil.copy(JOINT_MODEL_CONFIG, tmp_path / "config.yaml")
    hand = MockOrcaHand(config_path=str(tmp_path / "config.yaml"))
    encoder_cal = {
        joint: JointEncoderCal(enc_at_anchor_count=_anchor_count(i))
        for i, joint in enumerate(DECODED_JOINTS)
    }
    hand.calibration = dc.replace(
        hand.calibration, joint_encoder_calibration_dict=encoder_cal
    )
    return hand


def _raw_frame() -> np.ndarray:
    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    for i, joint in enumerate(DECODED_JOINTS):
        raw[JOINT_TO_ENCODER_SLOT[joint]] = _anchor_count(i) + 37
    return raw


def test_right_hand_decode_matches_the_validated_right_table(right_hand):
    """Identical output to decoding straight from the right-hand table, for
    joints of both polarity signs."""
    assert right_hand.config.type == "right"
    signs = {JOINT_ENCODER_POLARITY[j] for j in DECODED_JOINTS}
    assert signs == {1, -1}, "regression joints must exercise both signs"

    raw = _raw_frame()
    angles = right_hand._raw_to_joint_angle(raw)
    assert set(angles) == set(DECODED_JOINTS)

    encoder_cal = right_hand.calibration.joint_encoder_calibration_dict
    for joint in DECODED_JOINTS:
        expected = encoder_to_joint_angle(
            raw[np.array([JOINT_TO_ENCODER_SLOT[joint]])],
            np.array([encoder_cal[joint].enc_at_anchor_count], dtype=np.int64),
            np.array([JOINT_ENCODER_POLARITY[joint]], dtype=np.int64),
            np.array(
                [right_hand.config.joint_roms_dict[joint][1]], dtype=np.float64
            ),
        )
        assert angles[joint] == float(expected[0])


@pytest.mark.parametrize("side", ["left", None])
def test_unvalidated_side_raises_at_decode_time(right_hand, side):
    right_hand.config = dc.replace(right_hand.config, type=side)
    with pytest.raises(KeyError, match="polarity"):
        right_hand._raw_to_joint_angle(_raw_frame())


def test_unvalidated_side_without_encoder_calibration_decodes_nothing(right_hand):
    """An empty calibration never consults the polarity table, so the
    calibration anchor pass (which stores raw counts only) works on any side."""
    right_hand.config = dc.replace(right_hand.config, type="left")
    right_hand.calibration = dc.replace(
        right_hand.calibration, joint_encoder_calibration_dict={}
    )
    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    assert right_hand._raw_to_joint_angle(raw) == {}
