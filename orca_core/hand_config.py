# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import os
from dataclasses import dataclass
from typing import Dict, List, Literal

from .constants import (
    MOTOR_IDS, JOINT_IDS, JOINT_TO_MOTOR_MAP, JOINT_ROM_DICT
)

from .joint_position import OrcaJointPositions
from .utils.utils import get_model_path, read_yaml


@dataclass(frozen=True)
class HandConfig:
    """Config for a single ORCA hand.

    All fields are populated by :meth:`from_config_path` from the hand's
    ``config.yaml`` and ``calibration.yaml`` files. The object is also
    applied directly to the owning hand instance via :meth:`apply_to_instance`
    so that every attribute is accessible from the hand directly.

    Attributes:
        Static config (always required, sourced from ``config.yaml``):
            model_path: Absolute path to the model directory.
            config_path: Absolute path to ``config.yaml``.
            calibration_path: Absolute path to ``calibration.yaml``.
            baudrate: Serial baudrate for the motor bus.
            port: Serial port device string (e.g. ``/dev/ttyUSB0``).
            max_current: Maximum allowable motor current in mA.
            control_mode: Active control mode string; one of
                ``"current_based_position"``, ``"position"``, ``"current"``,
                ``"velocity"``, or ``"multi_turn_position"``.
            type: Hand variant identifier (e.g. ``"right"``).
            motor_type: Motor driver type; ``"dynamixel"`` or ``"feetech"``.
            motor_ids: Ordered list of motor IDs on the bus.
            joint_ids: Ordered list of joint name strings.
            motor_id_to_idx_dict: Maps motor ID → index in :attr:`motor_ids`.
            motor_to_joint_dict: Maps motor ID → joint name.
            joint_to_motor_map: Maps joint name → motor ID.
            joint_roms_dict: Maps joint name → ``[min, max]`` ROM bounds (rad).
            joint_inversion_dict: Maps joint name → ``True`` when the motor
                direction is inverted relative to the joint convention.
            neutral_position: Mapping from joint name to neutral position (rad).
            calibration_current: Motor current used during calibration (mA).
            wrist_calibration_current: Motor current used for wrist calibration (mA).
            calibration_step_size: Angular increment per calibration step (rad).
            calibration_step_period: Sleep time between calibration steps (seconds).
            calibration_threshold: Position stability threshold for limit detection (rad).
            calibration_num_stable: Number of stable readings required to declare a limit.
            calibration_sequence: Ordered list of calibration steps.

        Calibration state (absent until calibration runs, written to ``calibration.yaml``):
            calibrated: ``True`` when all joints have been calibrated.
            wrist_calibrated: ``True`` when the wrist joint has been calibrated.
            motor_limits_dict: Maps motor ID → ``[lower, upper]`` hard limits (rad).
                Values are ``None`` before calibration.
            joint_to_motor_ratios_dict: Maps motor ID → rad/rad gear ratio.
                Values are ``0.0`` before calibration.
    """

    # ------------------------------------------------------------------
    # Static config — always required, sourced from config.yaml
    # ------------------------------------------------------------------
    # NOTE: Shared across Hardware and Sim classes
    config_path: str
    neutral_position: Dict[str, float]  # TODO: turn into OrcaJointPositions
    
    # NOTE: From here onwards, it's all hardware-specific config
    model_path: str
    calibration_path: str

    baudrate: int
    port: str
    max_current: int
    control_mode: str
    type: Literal["left", "right"] | None
    motor_type: str

    motor_ids: List[int]
    joint_ids: List[str]
    motor_id_to_idx_dict: Dict[int, int]
    motor_to_joint_dict: Dict[int, str]
    joint_to_motor_map: Dict[str, int]
    joint_roms_dict: Dict[str, List[float]]
    joint_inversion_dict: Dict[str, bool]

    calibration_current: int
    wrist_calibration_current: int
    calibration_step_size: float
    calibration_step_period: float
    calibration_threshold: float
    calibration_num_stable: int
    calibration_sequence: List[dict]

    # ------------------------------------------------------------------
    # Calibration state — absent until calibration runs, written to
    # calibration.yaml by the calibration routine
    # ------------------------------------------------------------------
    calibrated: bool
    wrist_calibrated: bool
    motor_limits_dict: Dict[int, List[float | None]]
    joint_to_motor_ratios_dict: Dict[int, float]

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
        calibration = read_yaml(resolved_calibration_path) or {}

        motor_ids = list(config.get(MOTOR_IDS, []))
        joint_ids = list(config.get(JOINT_IDS, []))
        joint_to_motor_map = dict(config.get(JOINT_TO_MOTOR_MAP, {}))
        joint_roms_dict = dict(config.get(JOINT_ROM_DICT, {}))

        joint_inversion_dict = {}
        for joint, motor_id in joint_to_motor_map.items():
            if motor_id < 0:
                joint_inversion_dict[joint] = True
                joint_to_motor_map[joint] = int(abs(motor_id))
            else:
                joint_inversion_dict[joint] = False

        joint_to_motor_map = {joint: int(motor_id) for joint, motor_id in joint_to_motor_map.items()}
        motor_id_to_idx_dict = {motor_id: idx for idx, motor_id in enumerate(motor_ids)}
        motor_to_joint_dict = {motor_id: joint for joint, motor_id in joint_to_motor_map.items()}

        motor_limits_from_calibration = calibration.get("motor_limits", {})
        motor_limits_dict = {
            motor_id: motor_limits_from_calibration.get(motor_id, [None, None]) for motor_id in motor_ids
        }

        joint_to_motor_ratios_from_calibration = calibration.get("joint_to_motor_ratios", {})
        joint_to_motor_ratios_dict = {
            motor_id: joint_to_motor_ratios_from_calibration.get(motor_id, 0.0) for motor_id in motor_ids
        }

        return cls(
            model_path=resolved_model_path,
            config_path=resolved_config_path,
            calibration_path=resolved_calibration_path,
            baudrate=config.get("baudrate"),
            port=config.get("port"),
            max_current=config.get("max_current"),
            control_mode=config.get("control_mode"),
            type=config.get("type"),
            motor_type=config.get("motor_type"),
            calibration_current=config.get("calibration_current"),
            wrist_calibration_current=config.get("wrist_calibration_current"),
            calibration_step_size=config.get("calibration_step_size"),
            calibration_step_period=config.get("calibration_step_period"),
            calibration_threshold=config.get("calibration_threshold"),
            calibration_num_stable=config.get("calibration_num_stable"),
            calibration_sequence=list(config.get("calibration_sequence")),
            calibrated=calibration.get("calibrated"),
            wrist_calibrated=calibration.get("wrist_calibrated"),
            neutral_position=dict(config.get("neutral_position")),
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
        """Resolve the paths to the "model" folder, containing both the ``config.yaml``, and ``calibration.yaml`` files.

        Args:
            config_path: Path to a ``config.yaml`` file. Pass ``None`` to use
                the auto-discovered default model.
            calibration_path: Path to ``calibration.yaml``. Defaults to the
                file in the same directory as *config_path*.
        """
        if config_path is None:
            resolved_config_path = os.path.join(get_model_path(), "config.yaml")
        else:
            if os.path.basename(config_path) != "config.yaml":
                raise ValueError("config_path must point to a config.yaml file.")
            resolved_config_path = os.path.abspath(config_path)

        if not os.path.exists(resolved_config_path):
            raise FileNotFoundError(f"config.yaml not found: {resolved_config_path}")

        resolved_model_path = os.path.dirname(resolved_config_path)
        resolved_calibration_path = (
            os.path.abspath(calibration_path)
            if calibration_path is not None
            else os.path.join(resolved_model_path, "calibration.yaml")
        )
        return resolved_model_path, resolved_config_path, resolved_calibration_path


    def validate(self):
        """Validate configuration fields always required.

        In particular, it checks that
        * at least one joint is defined
        * every joint has a corresponding ROM entry
        * all ROMs are valid (two-element ranges with positive width)
        * every joint has a corresponding motor ID
        * every motor ID has a corresponding joint
        """
        if not self.joint_ids:
            raise ValueError("At least one joint must be configured.")

        for joint, rom in self.joint_roms_dict.items():
            if len(rom) != 2 or rom[1] - rom[0] <= 0:
                raise ValueError(f"ROM for joint {joint} is not valid.")
            
            if joint not in self.joint_ids:
                raise ValueError(f"Joint {joint} in ROMs is not defined.")


    def validate_hardware(self) -> None:
        # TODO: Move to the hardware hand class
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

        if self.max_current < self.calibration_current:
            raise ValueError("Max current should be greater than the calibration current.")

        for joint, motor_id in self.joint_to_motor_map.items():
            if joint not in self.joint_ids:
                raise ValueError(f"Joint {joint} is not defined.")
            if motor_id not in self.motor_ids:
                raise ValueError(f"Motor ID {motor_id} is not in the motor IDs list.")

        for step in self.calibration_sequence:
            for joint, direction in step["joints"].items():
                if joint not in self.joint_ids:
                    raise ValueError(f"Joint {joint} is not defined.")
                if direction not in ["flex", "extend"]:
                    raise ValueError(f"Invalid direction for joint {joint}.")

    def clamp_joint_positions(
        self,
        joint_pos: OrcaJointPositions
    ) -> OrcaJointPositions:
        """Clamp joint positions to their configured ROM bounds.

        Accepts any of the common position representations and returns an
        :class:`~orca_core.OrcaJointPosition` where every value is clipped to
        ``[min_rom, max_rom]`` for the corresponding joint.

        Args:
            joint_pos: Current or commanded joint positions, as an
                :class:`~orca_core.OrcaJointPosition`.

        Returns:
            Normalised :class:`~orca_core.OrcaJointPosition` safe to send to
            the hardware.

        Raises:
            ValueError: If *joint_pos* is not one of the accepted types.
        """

        normalized = {}
        for joint, pos in joint_pos:
            if joint not in self.joint_ids:
                raise ValueError(f"Joint {joint} is not defined among the hand's joint IDs: {self.joint_ids}")

            normalized[joint] = self._clip_joint_value(joint, pos)

        return OrcaJointPositions.from_dict(normalized)

    def _clip_joint_value(self, joint_name: str, pos: float) -> float:
        min_pos, max_pos = self.joint_roms_dict[joint_name]
        return max(min_pos, min(max_pos, float(pos)))

