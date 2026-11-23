# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import dataclasses
import math
import numbers
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, Iterable, List, Mapping, Optional, Tuple

from .constants import JOINTS
from .utils.utils import read_yaml

if TYPE_CHECKING:
    from .hand_config import OrcaHandConfig


@dataclass(frozen=True)
class JointEncoderCal:
    """Per-joint absolute-encoder anchor.

    ``joint_angle = polarity * Δenc_wrapped + anchor_angle_deg``, where
    ``Δenc_wrapped`` is the wrap-corrected 14-bit encoder delta from
    ``enc_at_anchor_count``, ``polarity`` comes from the hand side's polarity
    table via :func:`joint_encoder_polarity_for_side` (hardware-fixed by
    mounting + magnet orientation), and ``anchor_angle_deg`` is the joint's ROM upper from
    :attr:`OrcaHandConfig.joint_roms_dict` (the pose the calibration sweep
    stalls the motor at).
    """

    enc_at_anchor_count: int


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
            gear ratio. Zero before calibration. Derived, never stored: a hand
            computes it from the motor limits and the joint ranges in force
            (see :func:`_derive_joint_to_motor_ratios`).
        joint_encoder_calibration_dict: Maps joint name → :class:`JointEncoderCal`.
            Empty for hands without joint encoders.
        joint_roms_measured_dict: Maps joint name → ``[lower, upper]`` joint ROM
            in degrees, with the span between the hardstops measured by the
            joint encoders rather than taken from the config nominal. The upper
            endpoint equals the config ROM upper — it is the anchor pose the
            encoder frame is pinned to, so only the lower endpoint moves. Empty
            for hands without joint encoders; config ROMs are the fallback.
        motor_travel_measured_dict: Maps joint name → the motor-shaft travel,
            in degrees, recorded between that joint's two hardstops. Keyed by
            joint rather than by motor because the travel is set by the joint's
            tendon spool. Compared against the ``joint_motor_travel`` baseline
            in ``config.yaml`` to catch a joint that stalled short of its
            hardstop; see :mod:`orca_core.maintenance.motor_travel`.
        drive_current_measured_dict: Maps joint name → ``{"flex": mA,
            "extend": mA}``, the mean current each joint drew while being
            driven onto that direction's hardstop. Recorded for information
            only — nothing reads it back to make a decision. A joint drawing
            far more than its neighbours is the signature of an over-tensioned
            tendon. Absent for joints that were skipped or never pressed a stop.
        calibrated: ``True`` when all joints have been fully calibrated.
        wrist_calibrated: ``True`` when the wrist joint has been calibrated.
    """

    motor_limits_dict: Dict[int, List]
    joint_to_motor_ratios_dict: Dict[int, float]
    calibrated: bool
    wrist_calibrated: bool
    joint_encoder_calibration_dict: Dict[str, JointEncoderCal] = field(default_factory=dict)
    joint_roms_measured_dict: Dict[str, List[float]] = field(default_factory=dict)
    motor_travel_measured_dict: Dict[str, float] = field(default_factory=dict)
    drive_current_measured_dict: Dict[str, Dict[str, float]] = field(default_factory=dict)

    def _motor_has_limits(self, motor_id: int) -> bool:
        """Whether the motor has a non-empty limits list with no None entries."""
        limits = self.motor_limits_dict.get(motor_id)
        return bool(limits) and all(limit is not None for limit in limits)

    def _motor_has_ratio(self, motor_id: int) -> bool:
        """Whether the motor has a usable (present and nonzero) joint-to-motor ratio."""
        ratio = self.joint_to_motor_ratios_dict.get(motor_id)
        return ratio is not None and ratio != 0

    def _motor_ready(self, motor_id: int) -> bool:
        """Whether the motor has both limits and a usable joint-to-motor ratio."""
        return self._motor_has_limits(motor_id) and self._motor_has_ratio(motor_id)

    def _all_motors_ready(self) -> bool:
        """Whether every motor in the result is :meth:`_motor_ready`."""
        motor_ids = set(self.motor_limits_dict) | set(self.joint_to_motor_ratios_dict)
        return all(self._motor_ready(motor_id) for motor_id in motor_ids)

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
            motor_travel_measured_dict={},
            drive_current_measured_dict={},
        )

    @classmethod
    def from_calibration_path(
        cls,
        calibration_path: str,
        motor_ids: List[int],
        config: "Optional[OrcaHandConfig]" = None,
    ) -> "CalibrationResult":
        """Load calibration state from a ``calibration.yaml`` file.

        Returns an :meth:`empty` result for any fields absent from the file.

        Args:
            calibration_path: Absolute path to ``calibration.yaml``.
            motor_ids: Ordered list of motor IDs; used to build the dicts.
            config: The hand's config. Given, the joint-to-motor ratios are
                derived from the motor limits and its joint ranges, and joints
                whose motor mapping changed since the calibration read as
                uncalibrated. Without it, ratios a file from an older
                orca_core stored are taken as they are.
        """
        calibration = read_yaml(calibration_path) or {}
        result = result_from_mapping(calibration, motor_ids)
        if config is None:
            return result
        result, _ = in_force(result, calibration.get(FINGERPRINT), config)
        return result


def _limit_pair(motor_id, limits):
    if limits is None:
        return [None, None]
    if isinstance(limits, (list, tuple)) and len(limits) == 2 and all(
        value is None or (isinstance(value, numbers.Real) and not isinstance(value, bool)
                          and math.isfinite(value))
        for value in limits
    ):
        return list(limits)
    raise ValueError(f"motor {motor_id}'s limits {limits!r} are not a pair of numbers")


def result_from_mapping(calibration: Mapping, motor_ids: List[int]) -> CalibrationResult:
    """Parse a calibration document (from disk or a hand's board) into a result.

    Ratios are taken from the document only where an older one stored them; a
    hand replaces them with derived ones (:func:`in_force`).

    Raises:
        ValueError: a motor's limits are not a pair of numbers (or ``None``).
    """
    motor_limits_raw = calibration.get("motor_limits") or {}
    motor_limits_dict = {
        mid: _limit_pair(mid, motor_limits_raw.get(mid, [None, None])) for mid in motor_ids
    }

    ratios_raw = calibration.get("joint_to_motor_ratios") or {}
    joint_to_motor_ratios_dict = {
        mid: ratios_raw.get(mid, 0.0) for mid in motor_ids
    }

    encoder_raw = calibration.get("joint_encoder_calibration", {}) or {}
    joint_encoder_calibration_dict = {
        joint: JointEncoderCal(
            enc_at_anchor_count=int(entry["enc_at_anchor_count"]),
        )
        for joint, entry in encoder_raw.items()
    }

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

    travel_raw = calibration.get("motor_travel_measured", {}) or {}
    motor_travel_measured_dict = {}
    for joint, travel in travel_raw.items():
        try:
            motor_travel_measured_dict[joint] = float(travel)
        except (TypeError, ValueError):
            print(
                f"\033[93mWarning: dropping malformed motor_travel_measured "
                f"entry for {joint}: {travel!r}\033[0m"
            )

    current_raw = calibration.get("drive_current_measured", {}) or {}
    drive_current_measured_dict = {}
    for joint, per_direction in current_raw.items():
        if not isinstance(per_direction, dict):
            print(
                f"\033[93mWarning: dropping malformed drive_current_measured "
                f"entry for {joint}: {per_direction!r}\033[0m"
            )
            continue
        parsed = {}
        for direction, value in per_direction.items():
            try:
                parsed[str(direction)] = float(value)
            except (TypeError, ValueError):
                print(
                    f"\033[93mWarning: dropping malformed drive_current_measured "
                    f"value for {joint} {direction}: {value!r}\033[0m"
                )
        if parsed:
            drive_current_measured_dict[joint] = parsed

    return CalibrationResult(
        motor_limits_dict=motor_limits_dict,
        joint_to_motor_ratios_dict=joint_to_motor_ratios_dict,
        calibrated=calibration.get("calibrated", False) or False,
        wrist_calibrated=calibration.get("wrist_calibrated", False) or False,
        joint_encoder_calibration_dict=joint_encoder_calibration_dict,
        joint_roms_measured_dict=joint_roms_measured_dict,
        motor_travel_measured_dict=motor_travel_measured_dict,
        drive_current_measured_dict=drive_current_measured_dict,
    )


def joint_encoder_calibration_to_yaml(
    joint_encoder_calibration_dict: Dict[str, JointEncoderCal],
) -> Dict[str, Dict]:
    return {
        joint: dataclasses.asdict(cal)
        for joint, cal in joint_encoder_calibration_dict.items()
    }


FINGERPRINT = "fingerprint"
"""Key of the per-joint record of what each joint's limits were taken against."""


def joint_fingerprints(config: "OrcaHandConfig") -> Dict[str, str]:
    """Per joint, what its motor limits mean under ``config``: the signed motor
    it is driven by and the directions its calibration steps drive it, e.g.
    ``"+7 extend,flex"``. Limits recorded under another value are not this
    joint's under this config."""
    directions: Dict[str, set] = {}
    for step in config.calibration_sequence or []:
        for joint, direction in (step.get(JOINTS) or {}).items():
            directions.setdefault(str(joint), set()).add(str(direction))
    prints = {}
    for joint, motor_id in config.joint_to_motor_map.items():
        sign = "-" if config.joint_inversion_dict.get(joint, False) else "+"
        prints[joint] = f"{sign}{int(motor_id)} {','.join(sorted(directions.get(joint, ())))}"
    return prints


def _mismatched_joints(
    stored: Optional[Mapping[str, str]], current: Mapping[str, str]
) -> List[str]:
    """Joints whose stored fingerprint differs from ``current``. A calibration
    that recorded none (from an older orca_core) is taken as it is."""
    if not isinstance(stored, Mapping):
        return []
    return sorted(joint for joint, value in current.items() if stored.get(joint) != value)


def _derive_joint_to_motor_ratios(
    motor_limits_dict: Mapping[int, Optional[List]],
    joint_to_motor_map: Mapping[str, int],
    joint_roms: Mapping[str, List[float]],
) -> Dict[int, float]:
    """Motor-rad per joint-deg for every motor: its measured motor travel over
    its joint's range. 0.0 where the limits or the range are missing."""
    joint_of = {int(motor_id): joint for joint, motor_id in joint_to_motor_map.items()}
    ratios = {}
    for motor_id, limits in motor_limits_dict.items():
        ratios[motor_id] = 0.0
        joint = joint_of.get(int(motor_id))
        rom = joint_roms.get(joint) if joint is not None else None
        if not limits or len(limits) != 2 or any(v is None for v in limits) or not rom:
            continue
        span = float(rom[1]) - float(rom[0])
        if span:
            ratios[motor_id] = float(limits[1] - limits[0]) / span
    return ratios


def _ratio_roms(
    config: "OrcaHandConfig",
    joint_roms_measured: Mapping[str, List[float]],
    prefer_config: Iterable[str] = (),
) -> Dict[str, List[float]]:
    """The joint ranges ratios are fitted against: an encoder-measured span
    where one was recorded, else the config's range. Joints in
    ``prefer_config`` (ranges measured for this hand) always take the config's."""
    roms = dict(config.joint_roms_dict)
    keep = set(prefer_config)
    for joint, rom in joint_roms_measured.items():
        if joint in roms and joint not in keep:
            roms[joint] = list(rom)
    return roms


def in_force(
    stored: CalibrationResult,
    stored_fingerprint: Optional[Mapping[str, str]],
    config: "OrcaHandConfig",
    prefer_config: Iterable[str] = (),
) -> Tuple[CalibrationResult, List[str]]:
    """``stored`` as ``config`` uses it, and the joints it had to drop.

    A joint whose fingerprint changed reads as uncalibrated: its limits were
    taken against another motor or direction and would drive it wrong. Every
    ratio is derived from the limits and the ranges in force, so a calibration
    made under other joint ranges still maps correctly.
    """
    dropped = _mismatched_joints(stored_fingerprint, joint_fingerprints(config))
    limits = {motor_id: list(value) if value is not None else [None, None]
              for motor_id, value in stored.motor_limits_dict.items()}
    for joint in dropped:
        motor_id = config.joint_to_motor_map.get(joint)
        if motor_id in limits:
            limits[motor_id] = [None, None]
    ratios = _derive_joint_to_motor_ratios(
        limits,
        config.joint_to_motor_map,
        _ratio_roms(config, stored.joint_roms_measured_dict, prefer_config),
    )
    result = dataclasses.replace(
        stored,
        motor_limits_dict=limits,
        joint_to_motor_ratios_dict=ratios,
        calibrated=bool(stored.calibrated) and not dropped,
    )
    return result, dropped
