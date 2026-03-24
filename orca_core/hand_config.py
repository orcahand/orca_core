# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import os
from dataclasses import dataclass, field
from typing import Dict, List, Literal

from .constants import CONTROL_MODES, JOINT_IDS, JOINT_ROM_DICT, JOINT_TO_MOTOR_MAP, MOTOR_IDS
from .joint_position import OrcaJointPositions
from .utils.utils import get_model_path, read_yaml


class HandConfigValidationError(ValueError):
    """Raised when a hand configuration is structurally invalid."""


def _resolve_config_path(config_path: str | None) -> str:
    """Resolve the canonical ``config.yaml`` path."""
    if config_path is None:
        resolved = os.path.join(get_model_path(), "config.yaml")
    elif os.path.basename(config_path) != "config.yaml":
        raise ValueError("config_path must point to a config.yaml file.")
    else:
        resolved = os.path.abspath(config_path)

    if not os.path.exists(resolved):
        raise FileNotFoundError(f"config.yaml not found: {resolved}")

    return resolved


def _resolve_calibration_path(config_path: str, calibration_path: str | None) -> str:
    """Resolve the companion ``calibration.yaml`` path for a config file."""
    if calibration_path is not None:
        return os.path.abspath(calibration_path)
    return os.path.join(os.path.dirname(config_path), "calibration.yaml")


def _canonical_joint_to_motor_map(
    raw_joint_to_motor_map: Dict[str, int],
) -> tuple[Dict[str, int], Dict[str, bool]]:
    """Normalize signed YAML motor mappings into absolute IDs plus inversion flags."""
    normalized_map: Dict[str, int] = {}
    inversion_dict: Dict[str, bool] = {}
    for joint, motor_id in raw_joint_to_motor_map.items():
        motor_id = int(motor_id)
        inversion_dict[joint] = motor_id < 0
        normalized_map[joint] = abs(motor_id)
    return normalized_map, inversion_dict


@dataclass(frozen=True)
class BaseHandConfig:
    """Base joint-space configuration for a hand model."""

    config_path: str
    type: Literal["left", "right"] | None = None
    joint_ids: List[str] = field(default_factory=list)
    joint_roms_dict: Dict[str, List[float]] = field(default_factory=dict)
    neutral_position: Dict[str, float] = field(default_factory=dict)

    @property
    def model_path(self) -> str:
        """Directory containing the hand assets described by ``config_path``."""
        return os.path.dirname(self.config_path)

    @classmethod
    def from_config_path(cls, config_path: str | None = None) -> "BaseHandConfig":
        """Load shared hand metadata from ``config.yaml``."""
        resolved_config_path = _resolve_config_path(config_path)
        config = read_yaml(resolved_config_path) or {}

        kwargs = {"config_path": resolved_config_path}
        if "type" in config:
            kwargs["type"] = config["type"]
        if JOINT_IDS in config:
            kwargs["joint_ids"] = list(config[JOINT_IDS])
        if JOINT_ROM_DICT in config:
            kwargs["joint_roms_dict"] = dict(config[JOINT_ROM_DICT])
        if "neutral_position" in config:
            kwargs["neutral_position"] = dict(config["neutral_position"])

        return cls(**kwargs)

    def validate(self) -> None:
        """Validate shared joint-space configuration."""
        if not self.joint_ids:
            raise HandConfigValidationError("At least one joint must be configured.")

        for joint in self.joint_ids:
            if joint not in self.joint_roms_dict:
                raise HandConfigValidationError(f"ROM for joint {joint} is not defined.")

        for joint, rom in self.joint_roms_dict.items():
            if joint not in self.joint_ids:
                raise HandConfigValidationError(f"Joint {joint} in ROMs is not defined.")
            if len(rom) != 2 or rom[1] - rom[0] <= 0:
                raise HandConfigValidationError(f"ROM {rom} for joint {joint} is not valid.")

        for joint in self.neutral_position:
            if joint not in self.joint_ids:
                raise HandConfigValidationError(
                    f"Neutral position entry for joint {joint} is not defined."
                )

    def clamp_joint_positions(
        self,
        joint_pos: OrcaJointPositions,
    ) -> OrcaJointPositions:
        """Clamp joint positions to the configured ROM bounds."""
        if not isinstance(joint_pos, OrcaJointPositions):
            raise HandConfigValidationError(
                "joint_pos must be an OrcaJointPositions instance."
            )

        normalized = {}
        for joint, pos in joint_pos:
            if joint not in self.joint_ids:
                raise HandConfigValidationError(
                    f"Joint {joint} is not defined among the hand's joint IDs: {self.joint_ids}"
                )
            normalized[joint] = self._clamp_joint_value(joint, pos)

        return OrcaJointPositions.from_dict(normalized)

    def _clamp_joint_value(self, joint_name: str, joint_pos: float) -> float:
        min_pos, max_pos = self.joint_roms_dict[joint_name]
        return max(min_pos, min(max_pos, float(joint_pos)))


@dataclass(frozen=True)
class OrcaHandConfig(BaseHandConfig):
    """ORCA hand configuration layered on top of the shared base spec."""

    calibration_path: str = ""
    baudrate: int = 3_000_000
    port: str = "/dev/ttyUSB0"
    max_current: int = 300  # mA
    control_mode: str = "current_based_position"
    motor_type: str = "dynamixel"
    motor_ids: List[int] = field(default_factory=list)
    joint_to_motor_map: Dict[str, int] = field(default_factory=dict)
    joint_inversion_dict: Dict[str, bool] = field(default_factory=dict)
    calibration_current: int = 200  # mA
    wrist_calibration_current: int = 100  # mA
    calibration_step_size: float = 0.1  # rad
    calibration_step_period: float = 0.01  # s
    calibration_threshold: float = 0.01  # rad
    calibration_num_stable: int = 20
    calibration_sequence: List[dict] = field(default_factory=list)

    @property
    def motor_id_to_idx_dict(self) -> Dict[int, int]:
        """Map motor ID to its index in ``motor_ids``."""
        return {motor_id: idx for idx, motor_id in enumerate(self.motor_ids)}

    @property
    def motor_to_joint_dict(self) -> Dict[int, str]:
        """Map motor ID to joint name."""
        return {motor_id: joint for joint, motor_id in self.joint_to_motor_map.items()}

    @classmethod
    def from_config_path(
        cls,
        config_path: str | None = None,
        calibration_path: str | None = None,
    ) -> "OrcaHandConfig":
        """Load hardware-backed ORCA hand configuration from canonical YAML keys."""
        resolved_config_path = _resolve_config_path(config_path)
        resolved_calibration_path = _resolve_calibration_path(
            resolved_config_path, calibration_path
        )

        config = read_yaml(resolved_config_path)  # {} when file is not found

        kwargs = {"config_path": resolved_config_path, "calibration_path": resolved_calibration_path}

        if "type" in config:
            kwargs["type"] = config["type"]
        if JOINT_IDS in config:
            kwargs["joint_ids"] = list(config[JOINT_IDS])
        if JOINT_ROM_DICT in config:
            kwargs["joint_roms_dict"] = dict(config[JOINT_ROM_DICT])
        if "neutral_position" in config:
            kwargs["neutral_position"] = dict(config["neutral_position"])
        if "baudrate" in config:
            kwargs["baudrate"] = int(config["baudrate"])
        if "port" in config:
            kwargs["port"] = config["port"]
        if "max_current" in config:
            kwargs["max_current"] = int(config["max_current"])
        if "control_mode" in config:
            kwargs["control_mode"] = config["control_mode"]
        if "motor_type" in config:
            kwargs["motor_type"] = config["motor_type"]
        if MOTOR_IDS in config:
            kwargs["motor_ids"] = [int(motor_id) for motor_id in config[MOTOR_IDS]]
        if JOINT_TO_MOTOR_MAP in config:
            joint_to_motor_map, joint_inversion_dict = _canonical_joint_to_motor_map(
                dict(config[JOINT_TO_MOTOR_MAP])
            )
            kwargs["joint_to_motor_map"] = joint_to_motor_map
            kwargs["joint_inversion_dict"] = joint_inversion_dict
        if "calibration_current" in config:
            kwargs["calibration_current"] = int(config["calibration_current"])
        if "wrist_calibration_current" in config:
            kwargs["wrist_calibration_current"] = int(
                config["wrist_calibration_current"]
            )
        if "calibration_step_size" in config:
            kwargs["calibration_step_size"] = float(config["calibration_step_size"])
        if "calibration_step_period" in config:
            kwargs["calibration_step_period"] = float(config["calibration_step_period"])
        if "calibration_threshold" in config:
            kwargs["calibration_threshold"] = float(config["calibration_threshold"])
        if "calibration_num_stable" in config:
            kwargs["calibration_num_stable"] = int(config["calibration_num_stable"])
        if "calibration_sequence" in config:
            kwargs["calibration_sequence"] = list(config["calibration_sequence"])

        return cls(**kwargs)

    def validate_config(self) -> None:
        """Validate the ORCA hand configuration."""
        super().validate()

        if len(self.motor_ids) != len(self.joint_ids):
            raise HandConfigValidationError(
                "Number of motor IDs and joints do not match."
            )

        if len(self.motor_ids) != len(self.joint_to_motor_map):
            raise HandConfigValidationError(
                "Number of motor IDs and joint-to-motor mappings do not match."
            )

        if self.control_mode not in CONTROL_MODES:
            raise HandConfigValidationError("Invalid control mode.")

        if self.max_current < self.calibration_current:
            raise HandConfigValidationError(
                "Max current should be greater than the calibration current."
            )

        for joint, motor_id in self.joint_to_motor_map.items():
            if joint not in self.joint_ids:
                raise HandConfigValidationError(f"Joint {joint} is not defined.")
            if motor_id not in self.motor_ids:
                raise HandConfigValidationError(
                    f"Motor ID {motor_id} is not in the motor IDs list."
                )

        for step in self.calibration_sequence:
            joints = step.get("joints")
            if not isinstance(joints, dict):
                raise HandConfigValidationError(
                    "Each calibration step must contain a 'joints' mapping."
                )
            for joint, direction in joints.items():
                if joint not in self.joint_ids:
                    raise HandConfigValidationError(f"Joint {joint} is not defined.")
                if direction not in ["flex", "extend"]:
                    raise HandConfigValidationError(
                        f"Invalid direction for joint {joint}."
                    )

    def __post_init__(self) -> None:
        self.validate_config()


HandConfig = BaseHandConfig
