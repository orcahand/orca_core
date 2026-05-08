# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import dataclasses
from dataclasses import dataclass, field
from typing import Dict, List, Literal

from .utils.utils import read_yaml


@dataclass(frozen=True)
class JointEncoderCal:
    """Per-joint absolute-encoder calibration.

    ``joint_angle = polarity * Δenc_wrapped + anchor_angle_rad``, where
    ``Δenc_wrapped`` is the 14-bit-aware wrap-corrected delta from
    ``enc_at_anchor_count``. ``anchor_angle_rad`` equals one endpoint of
    ``joint_roms_dict[joint]``; ``anchor_end`` records which (``"min"`` or
    ``"max"``).
    """

    enc_at_anchor_count: int
    polarity: int
    anchor_angle_rad: float
    anchor_end: Literal["min", "max"]


@dataclass(frozen=True)
class CalibrationResult:
    """Immutable snapshot of a hand's calibration state.

    Produced by the calibration routine and stored on the hand instance.
    Replacing the instance attribute is the only way to update calibration
    state — the internals are never mutated in place.

    Attributes:
        motor_limits_dict: Maps motor ID → ``[lower, upper]`` hard limits (rad).
            Values are ``None`` before the corresponding joint is calibrated.
        joint_to_motor_ratios_dict: Maps motor ID → rad/rad gear ratio.
            Zero before calibration.
        joint_encoder_calibration_dict: Maps joint name → :class:`JointEncoderCal`.
            Empty for hands without joint encoders.
        calibrated: ``True`` when all joints have been fully calibrated.
        wrist_calibrated: ``True`` when the wrist joint has been calibrated.
    """

    motor_limits_dict: Dict[int, List]
    joint_to_motor_ratios_dict: Dict[int, float]
    calibrated: bool
    wrist_calibrated: bool
    joint_encoder_calibration_dict: Dict[str, JointEncoderCal] = field(default_factory=dict)

    @classmethod
    def empty(cls, motor_ids: List[int]) -> "CalibrationResult":
        """Return a blank (uncalibrated) result for the given motor IDs."""
        return cls(
            motor_limits_dict={mid: [None, None] for mid in motor_ids},
            joint_to_motor_ratios_dict={mid: 0.0 for mid in motor_ids},
            calibrated=False,
            wrist_calibrated=False,
            joint_encoder_calibration_dict={},
        )

    @classmethod
    def from_calibration_path(
        cls,
        calibration_path: str,
        motor_ids: List[int],
    ) -> "CalibrationResult":
        """Load calibration state from a ``calibration.yaml`` file.

        Returns an :meth:`empty` result for any fields absent from the file.

        Args:
            calibration_path: Absolute path to ``calibration.yaml``.
            motor_ids: Ordered list of motor IDs; used to build the dicts.
        """
        calibration = read_yaml(calibration_path) or {}

        motor_limits_raw = calibration.get("motor_limits", {})
        motor_limits_dict = {
            mid: motor_limits_raw.get(mid, [None, None]) for mid in motor_ids
        }

        ratios_raw = calibration.get("joint_to_motor_ratios", {})
        joint_to_motor_ratios_dict = {
            mid: ratios_raw.get(mid, 0.0) for mid in motor_ids
        }

        encoder_raw = calibration.get("joint_encoder_calibration", {}) or {}
        joint_encoder_calibration_dict = {
            joint: JointEncoderCal(
                enc_at_anchor_count=int(entry["enc_at_anchor_count"]),
                polarity=int(entry["polarity"]),
                anchor_angle_rad=float(entry["anchor_angle_rad"]),
                anchor_end=str(entry["anchor_end"]),
            )
            for joint, entry in encoder_raw.items()
        }

        return cls(
            motor_limits_dict=motor_limits_dict,
            joint_to_motor_ratios_dict=joint_to_motor_ratios_dict,
            calibrated=calibration.get("calibrated", False) or False,
            wrist_calibrated=calibration.get("wrist_calibrated", False) or False,
            joint_encoder_calibration_dict=joint_encoder_calibration_dict,
        )


def joint_encoder_calibration_to_yaml(
    joint_encoder_calibration_dict: Dict[str, JointEncoderCal],
) -> Dict[str, Dict]:
    return {
        joint: dataclasses.asdict(cal)
        for joint, cal in joint_encoder_calibration_dict.items()
    }
