# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Dict, List, Union

from .utils.utils import get_model_path, read_yaml


@dataclass
class HandSpec:
    model_path: str
    config_path: str
    calib_path: str
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
    def from_model_path(cls, model_path: str | None = None) -> "HandSpec":
        resolved_model_path = get_model_path(model_path)
        config_path = os.path.join(resolved_model_path, "config.yaml")
        calib_path = os.path.join(resolved_model_path, "calibration.yaml")

        config = read_yaml(config_path) or {}
        calib = read_yaml(calib_path) or {}

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
            config_path=config_path,
            calib_path=calib_path,
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

    def apply_to_instance(self, instance) -> None:
        for field_name in self.__dataclass_fields__:
            setattr(instance, field_name, getattr(self, field_name))

    def validate_shared(self) -> None:
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

    def normalize_joint_command(
        self, joint_pos: Union[dict, list]
    ) -> Dict[str, float | None]:
        if isinstance(joint_pos, dict):
            normalized = {}
            for joint, pos in joint_pos.items():
                if joint not in self.joint_ids:
                    continue
                normalized[joint] = self._normalize_joint_value(joint, pos)
            return normalized

        if isinstance(joint_pos, list):
            if len(joint_pos) != len(self.joint_ids):
                raise ValueError("Length of joint_pos list must match the number of joint_ids.")
            return {
                joint: self._normalize_joint_value(joint, pos)
                for joint, pos in zip(self.joint_ids, joint_pos)
            }

        raise ValueError("joint_pos must be a dict or a list.")

    def _normalize_joint_value(self, joint_name: str, pos: float | None) -> float | None:
        if pos is None:
            return None

        min_pos, max_pos = self.joint_roms_dict[joint_name]
        return max(min_pos, min(max_pos, float(pos)))
