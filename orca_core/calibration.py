# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import dataclasses
from dataclasses import dataclass, field
from typing import Dict, List

from .utils.utils import read_yaml


SENSOR_CAL_SOURCE_HARDSTOP = "hardstop"
SENSOR_CAL_SOURCE_FIXTURE = "fixture"

MEASURED_ROM_HARD_TOL_DEG = 8.0
MEASURED_ROM_WARN_TOL_DEG = 4.0


@dataclass(frozen=True)
class JointEncoderCal:
    """Per-joint absolute-encoder calibration.

    ``joint_angle = polarity * scale * Δenc_wrapped + zero_angle_deg``, where
    ``Δenc_wrapped`` is the wrap-corrected 14-bit encoder delta from
    ``zero_count`` and ``polarity`` is looked up from
    :data:`JOINT_ENCODER_POLARITY` (hardware-fixed by mounting + magnet
    orientation).

    ``source`` records where the reference pose came from:
    ``"hardstop"`` — the calibration sweep stalled the motor at the joint's
    ROM upper, so ``zero_angle_deg`` is the configured ROM upper and inherits
    the hardstop's manufacturing tolerance. ``"fixture"`` — the joint was held
    at externally known angles, so ``zero_angle_deg`` (and ``scale``) are
    independent of the hardstops.
    """

    zero_count: int
    zero_angle_deg: float
    scale: float = 1.0
    source: str = SENSOR_CAL_SOURCE_HARDSTOP


@dataclass(frozen=True)
class CalibrationResult:
    """Immutable snapshot of a hand's calibration state.

    Produced by the calibration routine and stored on the hand instance.
    Replacing the instance attribute is the only way to update calibration
    state — the internals are never mutated in place.

    Attributes:
        motor_limits_dict: Maps motor ID → ``[lower, upper]`` motor-shaft
            hard limits (motor radians). Values are ``None`` before the
            corresponding joint is calibrated.
        joint_to_motor_ratios_dict: Maps motor ID → motor-rad per joint-deg
            gear ratio. Zero before calibration.
        joint_encoder_calibration_dict: Maps joint name → :class:`JointEncoderCal`.
            Empty for hands without joint encoders.
        joint_roms_measured_dict: Maps joint name → ``[lower, upper]`` joint
            ROM in degrees as actually measured at the hardstops with
            absolutely calibrated joint encoders. Empty until a
            fixture-calibrated hand runs calibration; config ROMs remain the
            nominal fallback.
        calibrated: ``True`` when all joints have been fully calibrated.
        wrist_calibrated: ``True`` when the wrist joint has been calibrated.
    """

    motor_limits_dict: Dict[int, List]
    joint_to_motor_ratios_dict: Dict[int, float]
    calibrated: bool
    wrist_calibrated: bool
    joint_encoder_calibration_dict: Dict[str, JointEncoderCal] = field(default_factory=dict)
    joint_roms_measured_dict: Dict[str, List[float]] = field(default_factory=dict)

    @classmethod
    def empty(cls, motor_ids: List[int]) -> "CalibrationResult":
        """Return a blank (uncalibrated) result for the given motor IDs."""
        return cls(
            motor_limits_dict={mid: [None, None] for mid in motor_ids},
            joint_to_motor_ratios_dict={mid: 0.0 for mid in motor_ids},
            calibrated=False,
            wrist_calibrated=False,
            joint_encoder_calibration_dict={},
            joint_roms_measured_dict={},
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
        joint_encoder_calibration_dict = {}
        for joint, entry in encoder_raw.items():
            cal = _parse_joint_encoder_entry(joint, entry)
            if cal is not None:
                joint_encoder_calibration_dict[joint] = cal

        measured_raw = calibration.get("joint_roms_measured", {}) or {}
        joint_roms_measured_dict = {}
        for joint, rom in measured_raw.items():
            if not isinstance(rom, (list, tuple)) or len(rom) != 2:
                print(
                    f"\033[93mWarning: dropping malformed joint_roms_measured "
                    f"entry for {joint}: {rom!r}\033[0m"
                )
                continue
            joint_roms_measured_dict[joint] = [float(rom[0]), float(rom[1])]

        return cls(
            motor_limits_dict=motor_limits_dict,
            joint_to_motor_ratios_dict=joint_to_motor_ratios_dict,
            calibrated=calibration.get("calibrated", False) or False,
            wrist_calibrated=calibration.get("wrist_calibrated", False) or False,
            joint_encoder_calibration_dict=joint_encoder_calibration_dict,
            joint_roms_measured_dict=joint_roms_measured_dict,
        )


def _parse_joint_encoder_entry(joint: str, entry: Dict) -> JointEncoderCal | None:
    """Parse one ``joint_encoder_calibration:`` YAML entry. Entries that
    don't match the current format are dropped with a warning; the joint
    then reads as uncalibrated and a recalibration rewrites it.
    """
    if "zero_count" in entry and "zero_angle_deg" in entry:
        return JointEncoderCal(
            zero_count=int(entry["zero_count"]),
            zero_angle_deg=float(entry["zero_angle_deg"]),
            scale=float(entry.get("scale", 1.0)),
            source=str(entry.get("source", SENSOR_CAL_SOURCE_HARDSTOP)),
        )

    print(
        f"\033[93mWarning: dropping unrecognized joint_encoder_calibration "
        f"entry for {joint}: {sorted(entry)}\033[0m"
    )
    return None


def joint_encoder_calibration_to_yaml(
    joint_encoder_calibration_dict: Dict[str, JointEncoderCal],
) -> Dict[str, Dict]:
    return {
        joint: dataclasses.asdict(cal)
        for joint, cal in joint_encoder_calibration_dict.items()
    }


def clamp_measured_roms(
    measured: Dict[str, List[float]],
    config_roms: Dict[str, List[float]],
    hard_tol_deg: float = MEASURED_ROM_HARD_TOL_DEG,
    warn_tol_deg: float = MEASURED_ROM_WARN_TOL_DEG,
) -> Dict[str, List[float]]:
    """Sanity-bound sensor-measured ROMs against the configured nominals
    before they are persisted.

    Config ROMs are the safety envelope: each measured endpoint is clamped
    to its configured counterpart ± ``hard_tol_deg`` so a bad sweep can
    never silently widen (or wildly shrink) a joint's travel. Deviations
    beyond ``warn_tol_deg`` print a warning. Joints unknown to the config,
    or whose clamped ROM collapses to zero/negative span, are dropped.
    """
    clamped: Dict[str, List[float]] = {}
    for joint, rom in measured.items():
        config_rom = config_roms.get(joint)
        if config_rom is None:
            print(
                f"\033[93mWarning: dropping measured ROM for unknown joint "
                f"{joint}.\033[0m"
            )
            continue

        bounded = []
        for measured_end, config_end in zip(rom, config_rom):
            deviation = measured_end - config_end
            if abs(deviation) > warn_tol_deg:
                print(
                    f"\033[93mWarning: measured ROM endpoint for {joint} deviates "
                    f"{deviation:+.1f}° from the configured {config_end:.1f}° "
                    f"(clamped to ±{hard_tol_deg:.1f}°). Check the hardstop, the "
                    f"sensor calibration, and the configured ROM.\033[0m"
                )
            low = config_end - hard_tol_deg
            high = config_end + hard_tol_deg
            bounded.append(max(low, min(high, float(measured_end))))

        if bounded[1] - bounded[0] <= 0:
            print(
                f"\033[93mWarning: dropping measured ROM for {joint}: clamped "
                f"range [{bounded[0]:.1f}, {bounded[1]:.1f}]° is empty.\033[0m"
            )
            continue
        clamped[joint] = bounded

    return clamped
