"""Helpers for joint-loop tests: a calibrated mock hand, a static encoder
source, and a mass-damper plant that closes the loop in software.
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
    JOINT_ENCODER_POLARITY,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware.sensing.types import EncoderReading
from orca_core.hardware_hand import MockOrcaHand


_COUNTS_PER_RAD = ENCODER_COUNTS_PER_REV / (2.0 * np.pi)


def make_calibrated_hand(config_path: str) -> MockOrcaHand:
    """Connect a ``MockOrcaHand`` and install motor + encoder calibration
    with anchor_count=0 entries for every encoder-backed joint. Polarity is
    looked up from ``JOINT_ENCODER_POLARITY`` at angle-decode time."""
    hand = MockOrcaHand(config_path=config_path)
    hand.connect()
    motor_limits = {mid: [-0.5, 0.5] for mid in hand.config.motor_ids}
    ratios = {mid: 0.01 for mid in hand.config.motor_ids}
    encoder_cal = {
        joint: JointEncoderCal(
            enc_at_anchor_count=0,
            anchor_angle_rad=0.0,
        )
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


def encoder_reading_from_joint_angles(joint_angles: Dict[str, float]) -> EncoderReading:
    """Build an ``EncoderReading`` whose raw counts decode to ``joint_angles``
    under anchor=0 calibration. Applies the per-joint
    :data:`JOINT_ENCODER_POLARITY` so the round-trip
    ``encoder_to_joint_angle(reading.raw_counts, ...)`` recovers the input
    angles regardless of which joints are mounted with inverted polarity.
    """
    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    for joint, angle in joint_angles.items():
        slot = JOINT_TO_ENCODER_SLOT[joint]
        polarity = JOINT_ENCODER_POLARITY[joint]
        raw[slot] = int(round(angle * polarity * _COUNTS_PER_RAD)) % ENCODER_COUNTS_PER_REV
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


class CoupledPlantEncoderSource:
    """Encoder source backed by a per-joint mass-damper plant.

    ``advance(dt)`` reads the last commanded mA off the mock motor client,
    integrates ``joint_acc = current/inertia - damping*joint_vel`` per
    joint, and rebuilds the encoder reading from the new positions. The
    integrated joint pos is also mirrored back onto ``mock_dxl._pos`` so
    code paths that read motor position stay consistent.
    """

    def __init__(
        self,
        hand: MockOrcaHand,
        inertia: float = 1.0,
        damping: float = 30.0,
    ):
        self._hand = hand
        self._inertia = float(inertia)
        self._damping = float(damping)
        self._joint_pos: Dict[str, float] = {
            joint: 0.0 for joint in hand._encoder_backed_joints()
        }
        self._joint_vel: Dict[str, float] = {joint: 0.0 for joint in self._joint_pos}
        self._last_reading = encoder_reading_from_joint_angles(self._joint_pos)

    def advance(self, dt: float) -> None:
        currents = self._hand._motor_client.get_last_currents_mA()
        inversion = self._hand.config.joint_inversion_dict
        joint_to_motor = self._hand.config.joint_to_motor_map
        for joint in list(self._joint_pos):
            motor_id = joint_to_motor.get(joint)
            if motor_id is None:
                continue
            polarity = -1.0 if inversion.get(joint, False) else 1.0
            joint_current = polarity * float(currents.get(motor_id, 0.0))
            accel = (joint_current / self._inertia) - self._damping * self._joint_vel[joint]
            self._joint_vel[joint] += accel * dt
            self._joint_pos[joint] += self._joint_vel[joint] * dt
            self._hand._motor_client._pos[motor_id] = polarity * self._joint_pos[joint]
        self._last_reading = encoder_reading_from_joint_angles(self._joint_pos)

    def get_latest_encoder_reading(self) -> EncoderReading:
        return self._last_reading

    @property
    def joint_pos(self) -> Dict[str, float]:
        return dict(self._joint_pos)
