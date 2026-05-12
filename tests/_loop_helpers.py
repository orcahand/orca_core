"""Helpers for joint-loop tests: a calibrated mock hand, a static encoder
source, and an encoder-reading builder that round-trips joint angles
through the encoder protocol.
"""

from __future__ import annotations

import dataclasses as dc
import time
from typing import Dict

import numpy as np

from orca_core.calibration import JointEncoderCal
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    ENCODER_COUNTS_PER_REV,
    ENCODER_LSB_DEG,
    JOINT_ENCODER_POLARITY,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware.sensing.types import EncoderReading
from orca_core.hardware_hand import MockOrcaHand


def make_calibrated_hand(config_path: str) -> MockOrcaHand:
    """Connect a ``MockOrcaHand`` and install motor + encoder calibration
    with anchor_count=0 entries for every encoder-backed joint. Polarity is
    looked up from ``JOINT_ENCODER_POLARITY`` at angle-decode time."""
    hand = MockOrcaHand(config_path=config_path)
    hand.connect()
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
    hand._compute_wrap_offsets_dict()
    return hand


def encoder_reading_from_joint_angles(
    hand: MockOrcaHand, joint_angles_deg: Dict[str, float],
) -> EncoderReading:
    """Build an ``EncoderReading`` whose raw counts decode to the given
    joint angles (degrees) for the given hand's calibration.

    The decode reads the anchor angle from ``hand.config.joint_roms_dict[j][1]``
    and the anchor count from ``hand.calibration.joint_encoder_calibration_dict[j]``,
    so the round-trip through :func:`encoder_to_joint_angle` recovers the
    input angles regardless of polarity, anchor count, or ROM offset.
    """
    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    encoder_dict = hand.calibration.joint_encoder_calibration_dict
    for joint, angle_deg in joint_angles_deg.items():
        slot = JOINT_TO_ENCODER_SLOT[joint]
        polarity = JOINT_ENCODER_POLARITY[joint]
        anchor_count = encoder_dict[joint].enc_at_anchor_count
        anchor_angle_deg = hand.config.joint_roms_dict[joint][1]
        delta_counts = int(round((angle_deg - anchor_angle_deg) / (polarity * ENCODER_LSB_DEG)))
        raw[slot] = (anchor_count + delta_counts) % ENCODER_COUNTS_PER_REV
    return EncoderReading(
        raw_counts=raw,
        parity_ok=np.ones(AUTO_ENC_NUM_JOINTS, dtype=bool),
        angle_error=np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool),
        err_byte=0,
        timestamp=time.monotonic(),
    )


class StaticEncoderSource:
    """Encoder client that always returns one canned reading. Each call
    rebuilds the timestamp from ``freshness_ms`` so the watchdog sees a
    steady simulated age regardless of how long ago the reading was set.
    """

    def __init__(
        self,
        reading: EncoderReading | None = None,
        freshness_ms: float = 0.0,
    ):
        self._reading = reading
        self.freshness_ms = float(freshness_ms)

    def set_reading(self, reading: EncoderReading | None) -> None:
        self._reading = reading

    def get_latest_encoder_reading(self) -> EncoderReading | None:
        if self._reading is None:
            return None
        return dc.replace(
            self._reading,
            timestamp=time.monotonic() - self.freshness_ms / 1000.0,
        )
