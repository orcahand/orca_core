# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import math
import os
from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from .joint_position import OrcaJointPosition
from .utils.utils import get_model_path, read_yaml


@dataclass
class HandConfig:
    """Parsed, validated configuration for a single ORCA hand instance.

    All fields are populated by :meth:`from_config_path` from the hand's
    ``config.yaml`` and ``calibration.yaml`` files. The object is also
    applied directly to the owning hand instance via :meth:`apply_to_instance`
    so that every attribute is accessible from the hand directly.

    Attributes:
        model_path: Absolute path to the model directory.
        config_path: Absolute path to ``config.yaml``.
        calibration_path: Absolute path to ``calibration.yaml``.
        baudrate: Serial baudrate for the motor bus (default ``3_000_000``).
        port: Serial port device string (e.g. ``/dev/ttyUSB0``).
        max_current: Maximum allowable motor current in mA.
        control_mode: Active control mode string; one of
            ``"current_based_position"``, ``"position"``, ``"current"``,
            ``"velocity"``, or ``"multi_turn_position"``.
        type: Optional hand variant identifier string (e.g. ``"right"``).
        motor_type: Motor driver type; ``"dynamixel"`` or ``"feetech"``.
        calib_current: Motor current used during calibration (mA).
        wrist_calib_current: Motor current used for wrist calibration (mA).
        calib_step_size: Angular increment per calibration step (radians).
        calib_step_period: Sleep time between calibration steps (seconds).
        calib_threshold: Position stability threshold for limit detection (rad).
        calib_num_stable: Number of stable readings required to declare a limit.
        calib_sequence: Ordered list of calibration steps loaded from config.
        calibrated: ``True`` when all joints have been calibrated.
        wrist_calibrated: ``True`` when the wrist joint has been calibrated.
        neutral_position: Mapping from joint name to neutral position (rad).
        motor_ids: Ordered list of motor IDs on the bus.
        joint_ids: Ordered list of joint name strings.
        motor_id_to_idx_dict: Maps motor ID → index in :attr:`motor_ids`.
        motor_limits_dict: Maps motor ID → ``[lower, upper]`` hard limits (rad).
            Values may be ``None`` before calibration.
        joint_to_motor_ratios_dict: Maps motor ID → rad/rad gear ratio.
        joint_to_motor_map: Maps joint name → motor ID.
        joint_roms_dict: Maps joint name → ``[min, max]`` ROM bounds (rad).
        joint_inversion_dict: Maps joint name → ``True`` when the motor
            direction is inverted relative to the joint convention.
        motor_to_joint_dict: Maps motor ID → joint name.
    """

    model_path: str
    config_path: str
    calibration_path: str
    baudrate: int
    port: str
    max_current: int
    control_mode: str
    type: str | None
    motor_type: str
    calib_current: int
    wrist_calib_current: int
    calib_step_size: float
    calib_step_period: float
    calib_threshold: float
    calib_num_stable: int
    calib_sequence: List[dict]
    calibrated: bool
    wrist_calibrated: bool
    neutral_position: Dict[str, float]
    motor_ids: List[int]
    joint_ids: List[str]
    motor_id_to_idx_dict: Dict[int, int]
    motor_limits_dict: Dict[int, List[float | None]]
    joint_to_motor_ratios_dict: Dict[int, float]
    joint_to_motor_map: Dict[str, int]
    joint_roms_dict: Dict[str, List[float]]
    joint_inversion_dict: Dict[str, bool]
    motor_to_joint_dict: Dict[int, str]

    @classmethod
    def from_config_path(
        cls,
        config_path: str | None = None,
        calibration_path: str | None = None,
    ) -> "HandConfig":
        """Load and construct a :class:`HandConfig` from YAML files.

        When *config_path* is ``None`` the first model found under the package's
        ``models/`` directory is used. The calibration file is expected to live
        alongside ``config.yaml`` unless *calibration_path* is specified.

        Args:
            config_path: Path to a ``config.yaml`` file. Pass ``None`` to use
                the auto-discovered default model.
            calibration_path: Path to ``calibration.yaml``. Defaults to the
                file in the same directory as *config_path*.

        Returns:
            Fully populated :class:`HandConfig` instance.

        Raises:
            ValueError: If *config_path* does not point to a ``config.yaml``
                file.
            FileNotFoundError: If the resolved config file does not exist.
        """
        resolved_model_path, resolved_config_path, resolved_calibration_path = cls._resolve_paths(
            config_path=config_path,
            calibration_path=calibration_path,
        )

        config = read_yaml(resolved_config_path) or {}
        calib = read_yaml(resolved_calibration_path) or {}

        motor_ids = list(config.get("motor_ids", []))
        joint_ids = list(config.get("joint_ids", []))
        joint_to_motor_map = dict(config.get("joint_to_motor_map", {}))
        joint_roms_dict = dict(config.get("joint_roms", {}))

        joint_inversion_dict = {}
        for joint, motor_id in joint_to_motor_map.items():
            if motor_id < 0 or math.copysign(1, motor_id) < 0:
                joint_inversion_dict[joint] = True
                joint_to_motor_map[joint] = int(abs(motor_id))
            else:
                joint_inversion_dict[joint] = False

        joint_to_motor_map = {joint: int(motor_id) for joint, motor_id in joint_to_motor_map.items()}
        motor_id_to_idx_dict = {motor_id: idx for idx, motor_id in enumerate(motor_ids)}
        motor_to_joint_dict = {motor_id: joint for joint, motor_id in joint_to_motor_map.items()}

        motor_limits_from_calib = calib.get("motor_limits", {})
        motor_limits_dict = {
            motor_id: motor_limits_from_calib.get(motor_id, [None, None]) for motor_id in motor_ids
        }

        joint_to_motor_ratios_from_calib = calib.get("joint_to_motor_ratios", {})
        joint_to_motor_ratios_dict = {
            motor_id: joint_to_motor_ratios_from_calib.get(motor_id, 0.0) for motor_id in motor_ids
        }

        return cls(
            model_path=resolved_model_path,
            config_path=resolved_config_path,
            calibration_path=resolved_calibration_path,
            baudrate=config.get("baudrate", 3000000),
            port=config.get("port", "/dev/ttyUSB0"),
            max_current=config.get("max_current", 300),
            control_mode=config.get("control_mode", "current_position"),
            type=config.get("type", None),
            motor_type=config.get("motor_type", "dynamixel"),
            calib_current=config.get("calib_current", 200),
            wrist_calib_current=config.get("wrist_calib_current", 100),
            calib_step_size=config.get("calib_step_size", 0.1),
            calib_step_period=config.get("calib_step_period", 0.01),
            calib_threshold=config.get("calib_threshold", 0.01),
            calib_num_stable=config.get("calib_num_stable", 20),
            calib_sequence=list(config.get("calib_sequence", [])),
            calibrated=calib.get("calibrated", False),
            wrist_calibrated=calib.get("wrist_calibrated", False),
            neutral_position=dict(config.get("neutral_position", {})),
            motor_ids=motor_ids,
            joint_ids=joint_ids,
            motor_id_to_idx_dict=motor_id_to_idx_dict,
            motor_limits_dict=motor_limits_dict,
            joint_to_motor_ratios_dict=joint_to_motor_ratios_dict,
            joint_to_motor_map=joint_to_motor_map,
            joint_roms_dict=joint_roms_dict,
            joint_inversion_dict=joint_inversion_dict,
            motor_to_joint_dict=motor_to_joint_dict,
        )

    @staticmethod
    def _resolve_paths(
        config_path: str | None,
        calibration_path: str | None,
    ) -> tuple[str, str, str]:
        if config_path is None:
            resolved_model_path = get_model_path()
            resolved_config_path = os.path.join(resolved_model_path, "config.yaml")
        else:
            if os.path.isdir(config_path) or os.path.basename(config_path) != "config.yaml":
                raise ValueError("config_path must point to a config.yaml file.")

            resolved_config_path = os.path.abspath(config_path)
            if not os.path.exists(resolved_config_path):
                raise FileNotFoundError(f"Configuration file not found: {resolved_config_path}")
            resolved_model_path = os.path.dirname(resolved_config_path)

        if calibration_path is None:
            resolved_calibration_path = os.path.join(resolved_model_path, "calibration.yaml")
        else:
            resolved_calibration_path = os.path.abspath(calibration_path)

        return resolved_model_path, resolved_config_path, resolved_calibration_path

    def apply_to_instance(self, instance) -> None:
        """Copy every config field onto *instance* as an attribute.

        Called during hand initialisation so that ``hand.baudrate``,
        ``hand.joint_ids``, etc. are available directly on the hand object.

        Args:
            instance: Target object to receive the config attributes.
        """
        for field_name in self.__dataclass_fields__:
            setattr(instance, field_name, getattr(self, field_name))

    @property
    def calib_path(self) -> str:
        """Alias for :attr:`calibration_path`."""
        return self.calibration_path

    def validate_shared(self) -> None:
        """Validate configuration fields required by all hand backends.

        Checks that at least one joint is defined, that every joint has a
        corresponding ROM entry, and that all ROMs are valid (two-element
        ranges with positive width).

        Raises:
            ValueError: On any configuration inconsistency.
        """
        if not self.joint_ids:
            raise ValueError("At least one joint ID must be configured.")

        for joint in self.joint_ids:
            if joint not in self.joint_roms_dict:
                raise ValueError(f"ROM for joint {joint} is not defined.")

        for joint, rom in self.joint_roms_dict.items():
            if len(rom) != 2 or rom[1] - rom[0] <= 0:
                raise ValueError(f"ROM for joint {joint} is not valid.")
            if joint not in self.joint_ids:
                raise ValueError(f"Joint {joint} in ROMs is not defined.")

    def validate_hardware(self) -> None:
        """Validate configuration fields required by the hardware backend.

        Extends :meth:`validate_shared` with hardware-specific checks:
        motor/joint count parity, valid control mode, current limits, and the
        integrity of both the joint-to-motor map and the calibration sequence.

        Raises:
            ValueError: On any hardware-specific configuration inconsistency.
        """
        if len(self.motor_ids) != len(self.joint_ids):
            raise ValueError("Number of motor IDs and joints do not match.")

        if len(self.motor_ids) != len(self.joint_to_motor_map):
            raise ValueError("Number of motor IDs and joints do not match.")

        if self.control_mode not in [
            "current_position",
            "current_velocity",
            "position",
            "multi_turn_position",
            "current_based_position",
        ]:
            raise ValueError("Invalid control mode.")

        if self.max_current < self.calib_current:
            raise ValueError("Max current should be greater than the calibration current.")

        for joint, motor_id in self.joint_to_motor_map.items():
            if joint not in self.joint_ids:
                raise ValueError(f"Joint {joint} is not defined.")
            if motor_id not in self.motor_ids:
                raise ValueError(f"Motor ID {motor_id} is not in the motor IDs list.")

        for step in self.calib_sequence:
            for joint, direction in step["joints"].items():
                if joint not in self.joint_ids:
                    raise ValueError(f"Joint {joint} is not defined.")
                if direction not in ["flex", "extend"]:
                    raise ValueError(f"Invalid direction for joint {joint}.")

    def normalize_joint_position(
        self,
        joint_pos: OrcaJointPosition | dict | np.ndarray,
    ) -> OrcaJointPosition:
        """Clamp joint positions to their configured ROM bounds.

        Accepts any of the common position representations and returns an
        :class:`~orca_core.OrcaJointPosition` where every value is clipped to
        ``[min_rom, max_rom]`` for the corresponding joint.

        Args:
            joint_pos: Current or commanded joint positions as an
                :class:`~orca_core.OrcaJointPosition`, a ``dict`` mapping joint
                names to values, or a 1-D ``np.ndarray`` aligned with
                :attr:`joint_ids`.

        Returns:
            Normalised :class:`~orca_core.OrcaJointPosition` safe to send to
            the hardware.

        Raises:
            ValueError: If *joint_pos* is not one of the accepted types.
        """
        if isinstance(joint_pos, OrcaJointPosition):
            raw_joint_pos = joint_pos.as_dict()
        elif isinstance(joint_pos, dict):
            raw_joint_pos = dict(joint_pos)
        elif isinstance(joint_pos, np.ndarray):
            raw_joint_pos = OrcaJointPosition.from_ndarray(joint_pos, self.joint_ids).as_dict()
        else:
            raise ValueError("joint_pos must be an OrcaJointPosition, dict, or ndarray.")

        normalized = {}
        for joint, pos in raw_joint_pos.items():
            if joint not in self.joint_ids:
                continue
            normalized[joint] = self._normalize_joint_value(joint, pos)

        return OrcaJointPosition.from_dict(normalized)

    def normalize_joint_command(
        self,
        joint_pos: OrcaJointPosition | dict | np.ndarray,
    ) -> Dict[str, float]:
        """Clamp positions and return them as a plain ``dict``.

        Convenience wrapper around :meth:`normalize_joint_position` for callers
        that need a ``dict`` rather than an :class:`~orca_core.OrcaJointPosition`.

        Args:
            joint_pos: Joint positions in any accepted representation.

        Returns:
            Dict mapping joint name → clamped position (rad).
        """
        return self.normalize_joint_position(joint_pos).as_dict()

    def _normalize_joint_value(self, joint_name: str, pos: float) -> float:
        min_pos, max_pos = self.joint_roms_dict[joint_name]
        return max(min_pos, min(max_pos, float(pos)))
