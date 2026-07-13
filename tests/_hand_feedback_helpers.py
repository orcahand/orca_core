"""Fixtures for ``OrcaHandJointFeedback`` lifecycle tests.

The hand under test owns a real ``JointEncoderClient`` on a
``MockHandSerialLink``; the mock hand's built-in frame pump feeds AA A9
frames so ``start_stream`` returns within its first-frame timeout.
"""

from __future__ import annotations

import dataclasses as dc
from typing import Optional

import numpy as np

from orca_core.calibration import JointEncoderCal
from orca_core.hardware.sensing.constants import AUTO_ENC_NUM_JOINTS
from orca_core import MockOrcaHandJointFeedback

from tests._encoder_helpers import make_encoder_frame


def make_calibrated_joint_feedback_hand(
    config_path: str,
    raw_counts: Optional[np.ndarray] = None,
    install_encoder_calibration: bool = True,
) -> MockOrcaHandJointFeedback:
    """Build a mock hand wired for a successful joint-feedback connect.

    The built-in mock frame pump is pointed at a frame carrying
    ``raw_counts``. With ``install_encoder_calibration=False`` the motor
    calibration is installed but the encoder calibration block is emptied,
    so ``connect()`` raises ``JointFeedbackConnectError``.
    """
    if raw_counts is None:
        raw_counts = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)

    frame = make_encoder_frame(raw_counts=raw_counts)

    class _Bound(MockOrcaHandJointFeedback):
        def _mock_encoder_frame(self) -> bytes:
            return frame

    hand = _Bound(config_path=config_path)

    motor_limits = {mid: [-0.5, 0.5] for mid in hand.config.motor_ids}
    ratios = {mid: 0.01 for mid in hand.config.motor_ids}
    encoder_cal = (
        {
            joint: JointEncoderCal(enc_at_anchor_count=0)
            for joint in hand._encoder_backed_joints()
        }
        if install_encoder_calibration
        else {}
    )
    hand.calibration = dc.replace(
        hand.calibration,
        motor_limits_dict=motor_limits,
        joint_to_motor_ratios_dict=ratios,
        joint_encoder_calibration_dict=encoder_cal,
        calibrated=True,
        wrist_calibrated=True,
    )
    return hand
