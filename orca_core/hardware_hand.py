# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import dataclasses
import math
import os
import threading
import time
from collections import deque
from threading import RLock
from typing import Dict, List, Union

import numpy as np

from .base_hand import BaseHand
from .calibration import CalibrationResult
from .hand_config import OrcaHandConfig, OrcaHandTouchConfig
from .hardware.dynamixel_client import DynamixelClient
from .hardware.feetech_client import FeetechClient
from .hardware.mock_tactile_client import MockTactileClient
from .hardware.motor_client import MotorClient
from .hardware.sensing.types import ResultantReading, TactileReading, TaxelReading
from .hardware.tactile_client import TactileClient
from .utils.utils import (
    auto_detect_port,
    find_single_usb_serial_port,
    get_and_choose_port,
    read_yaml,
    update_yaml,
)

from .constants import (
    SUPPORTED_MOTOR_TYPES,
    MOTOR_BAUD_RATES,
    MODE_MAP,
    WRIST_MODE_VALUE,
    CURRENT_BASED_POSITION,
    CURRENT,
    WRIST,
    FLEX,
    EXTEND,
    JOINTS,
    STEP,
    TINY_SLEEP,
    JOINT_TO_MOTOR_RATIOS,
    MOTOR_LIMITS_DICT,
    WRIST_CALIBRATED,
    CALIBRATED,
    NUM_STEPS,
    POSITION,
    STEP_SIZE,
)

from .joint_position import OrcaJointPositions


class OrcaHand(BaseHand):
    """ORCA hand class.

    Extends :class:`~orca_core.BaseHand` with a full lifecycle for a physical
    hand: connection management, torque control, multi-mode motor control,
    (automatic) calibration, and background task execution.

    The recommended usage pattern is:

    >>> from orca_core import OrcaHand, OrcaJointPositions
    >>> hand = OrcaHand()
    >>> hand.connect()
    >>> hand.init_joints()  # enables torque, calibrates if needed
    >>> hand.set_joint_positions(OrcaJointPositions({"index_mcp": 0.5}))
    >>> hand.disconnect()

    Args:
        config_path: Path to a ``config.yaml`` file. Defaults to the bundled
            model when ``None``.
    """

    config_cls = OrcaHandConfig

    def __init__(
        self,
        config_path: str | None = None,
        calibration_path: str | None = None,
        model_version: str | None = None,
        model_name: str | None = None,
        config: OrcaHandConfig | None = None,
    ):
        super().__init__(
            config_path=config_path,
            config=config,
            calibration_path=calibration_path,
            model_version=model_version,
            model_name=model_name,
        )

        self._wrap_offsets_dict: Dict[int, float] = None
        self._motor_client: MotorClient = None
        self._motor_lock: RLock = RLock()

        self._task_thread: threading.Thread = None
        self._task_stop_event = threading.Event()
        self._lock = threading.Lock()
        self._current_task = None

        self.calibration = CalibrationResult.from_calibration_path(
            self.config.calibration_path, self.config.motor_ids
        )
        self._sanity_check()
        self.is_calibrated(verbose=True)

    def __del__(self):
        self.disconnect()

    # ------------------------------------------------------------------
    # Calibration state — thin views onto self.calibration
    # ------------------------------------------------------------------

    @property
    def motor_limits_dict(self) -> Dict[int, list]:
        return self.calibration.motor_limits_dict

    @property
    def joint_to_motor_ratios_dict(self) -> Dict[int, float]:
        return self.calibration.joint_to_motor_ratios_dict

    @property
    def calibrated(self) -> bool:
        return self.calibration.calibrated

    @property
    def wrist_calibrated(self) -> bool:
        return self.calibration.wrist_calibrated

    def _create_motor_client(
        self,
        motor_type: str,
        port: str,
        baudrate: int,
    ) -> MotorClient:
        if motor_type == "dynamixel":
            return DynamixelClient(self.config.motor_ids, port, baudrate)

        if motor_type == "feetech":
            return FeetechClient(self.config.motor_ids, port, baudrate)

        raise ValueError(
            f"Unknown motor_type: {motor_type}. "
            f"Expected one of [{', '.join(SUPPORTED_MOTOR_TYPES)}]."
        )

    def _resolve_port(self) -> str | None:
        """Pick the serial port to talk to: explicit yaml > VID match > sole adapter."""
        if self.config.port and os.path.exists(self.config.port):
            return self.config.port
        if self.config.port:
            print(
                f"Configured port {self.config.port} not present; "
                "falling back to USB auto-detection."
            )
        # No yaml port (or the configured one is gone). Try VID-based detection
        # for any motor family, then fall back to "the only adapter present".
        for motor_type in SUPPORTED_MOTOR_TYPES:
            port = auto_detect_port(motor_type)
            if port:
                return port
        return find_single_usb_serial_port()

    def _trial_probe(self, port: str) -> tuple[str | None, int | None]:
        """Probe ``port`` until a (motor_type, baudrate) combination responds.

        Iterates each motor family × the baudrates listed in
        :data:`~orca_core.constants.MOTOR_BAUD_RATES`. If ``motor_type`` or
        ``baudrate`` is pinned in yaml, that dimension is fixed and the
        probe only iterates the other.
        """
        candidates = {
            "dynamixel": DynamixelClient,
            "feetech": FeetechClient,
        }
        motor_types = (
            [self.config.motor_type]
            if self.config.motor_type
            else list(SUPPORTED_MOTOR_TYPES)
        )
        for motor_type in motor_types:
            client_cls = candidates.get(motor_type)
            if client_cls is None:
                continue
            baudrates = (
                [self.config.baudrate]
                if self.config.baudrate is not None
                else list(MOTOR_BAUD_RATES.get(motor_type, []))
            )
            for baudrate in baudrates:
                print(f"Probing {motor_type} on {port} @ {baudrate} baud...")
                try:
                    if client_cls.probe(port, baudrate, self.config.motor_ids):
                        print(f"  -> {motor_type} responded at {baudrate} baud.")
                        return motor_type, baudrate
                except Exception as e:
                    print(f"  -> probe errored: {e}")
        return None, None

    def connect(self) -> tuple[bool, str]:
        """Open connection to the motor bus.

        Resolves (motor_type, port, baudrate) at connect time:

        1. Find a port: explicit ``port`` in yaml wins; else VID-based
           auto-detection; else the lone attached USB serial adapter; else
           an interactive picker.
        2. Find the motor family / baudrate by pinging each candidate from
           :data:`~orca_core.constants.MOTOR_BAUD_RATES` until one responds.
           Explicit ``motor_type``/``baudrate`` in yaml short-circuit the
           corresponding axis.

        Returns:
            A ``(success, message)`` tuple where *success* is ``True`` on a
            successful connection.
        """
        port = self._resolve_port()
        if port is None:
            print("Please select a port from available devices:")
            port = get_and_choose_port()
            if port is None:
                return False, "Connection failed: No port selected"

        motor_type, baudrate = self._trial_probe(port)
        if motor_type is None or baudrate is None:
            return False, (
                f"Connection failed: no motor responded on {port}. "
                "Check power and wiring."
            )

        try:
            self._motor_client = self._create_motor_client(motor_type, port, baudrate)
            with self._motor_lock:
                self._motor_client.connect()

            self._persist_resolved_driver(motor_type, port, baudrate)
            return True, f"Connection successful ({motor_type} @ {port}, {baudrate} baud)"

        except Exception as e:
            self._motor_client = None
            return False, f"Connection failed on {port}: {str(e)}"

    def _persist_resolved_driver(self, motor_type: str, port: str, baudrate: int) -> None:
        """Persist auto-detected driver fields to config.yaml.

        Each field is only written when it was missing from yaml. Once written,
        the next connect short-circuits the probe and uses the yaml values
        directly. Users who want to swap motor families on the same hand can
        clear the yaml fields and re-run to trigger a fresh probe.
        """
        updates = {}
        if not self.config.port or self.config.port != port:
            updates["port"] = port
        if self.config.motor_type is None:
            updates["motor_type"] = motor_type
        if self.config.baudrate is None:
            updates["baudrate"] = baudrate
        if not updates:
            return
        self.config = dataclasses.replace(self.config, **updates)
        for key, value in updates.items():
            update_yaml(self.config.config_path, key, value)
        print(
            f"Wrote auto-detected {', '.join(updates.keys())} to "
            f"{os.path.basename(self.config.config_path)}."
        )

    def disconnect(self) -> tuple[bool, str]:
        """Disable torque and close the serial connection.

        Safe to call even when the hand is already disconnected.

        Returns:
            A ``(success, message)`` tuple.
        """
        try:
            if self._motor_client is None:
                return True, "Disconnected successfully"
            with self._motor_lock:
                self.disable_torque()
                time.sleep(0.1)
                self._motor_client.disconnect()
            return True, "Disconnected successfully"
        except Exception as e:
            return False, f"Disconnection failed: {str(e)}"

    def is_connected(self) -> bool:
        """Return ``True`` if the motor client is connected.

        Returns:
            Connection status as a boolean.
        """
        return self._motor_client is not None and self._motor_client.is_connected

    def enable_torque(self, motor_ids: List[int] = None):
        """Enable torque on the specified motors.

        Args:
            motor_ids: List of motor IDs to enable. Defaults to all motors.
        """
        motor_ids = self.config.motor_ids if motor_ids is None else motor_ids

        with self._motor_lock:
            self._motor_client.set_torque_enabled(motor_ids, True)

    def disable_torque(self, motor_ids: List[int] = None):
        """Disable torque on the specified motors.

        Args:
            motor_ids: List of motor IDs to disable. Defaults to all motors.
        """
        motor_ids = self.config.motor_ids if motor_ids is None else motor_ids

        with self._motor_lock:
            self._motor_client.set_torque_enabled(motor_ids, False)

    def set_max_current(self, current: Union[float, List[float]]):
        """Set the maximum allowable current for the motors.

        Args:
            current: Either a single float applied to all motors, or a list of
                per-motor current values (mA). If a list, its length must match
                the number of configured motors.

        Raises:
            ValueError: If *current* is a list with the wrong length.
        """
        if isinstance(current, list):
            if len(current) != len(self.config.motor_ids):
                raise ValueError(
                    "Number of currents do not match the number of motors."
                )

            with self._motor_lock:
                self._motor_client.write_desired_current(self.config.motor_ids, current)

        with self._motor_lock:
            self._motor_client.write_desired_current(
                self.config.motor_ids, current * np.ones(len(self.config.motor_ids))
            )

    def set_control_mode(self, mode: str, motor_ids: List[int] = None):
        """Switch the operating mode of the specified motors.

        The wrist motor is always kept in ``multi_turn_position`` mode (4) when
        *mode* would otherwise be ``current_based_position`` (5) or
        ``current`` (0), because those modes are incompatible with the wrist
        joint's range of motion.

        Args:
            mode: One of ``"current"``, ``"velocity"``, ``"position"``,
                ``"multi_turn_position"``, or ``"current_based_position"``.
            motor_ids: Motors to reconfigure. Defaults to all motors.

        Raises:
            ValueError: If *mode* is not recognised or *motor_ids* contains
                unknown IDs.
        """
        mode_value = MODE_MAP.get(mode)
        if mode_value is None:
            raise ValueError("Invalid control mode.")

        with self._motor_lock:
            if motor_ids is None:
                motor_ids = self.config.motor_ids
            elif not all(motor_id in self.config.motor_ids for motor_id in motor_ids):
                raise ValueError("Invalid motor IDs.")

        if mode_value in (MODE_MAP[CURRENT_BASED_POSITION], MODE_MAP[CURRENT]):
            wrist_motor_id = self.config.joint_to_motor_map.get("wrist")
            if wrist_motor_id is not None:
                motor_ids_without_wrist = [
                    motor_id for motor_id in motor_ids if motor_id != wrist_motor_id
                ]
                self._motor_client.set_operating_mode(
                    motor_ids_without_wrist, mode_value
                )

                if wrist_motor_id in motor_ids:
                    self._motor_client.set_operating_mode(
                        [wrist_motor_id], WRIST_MODE_VALUE
                    )

                return

        self._motor_client.set_operating_mode(motor_ids, mode_value)

    def get_motor_pos(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Read raw motor positions from the bus.

        Args:
            as_dict: When ``True`` returns a ``dict`` keyed by motor ID.
                Defaults to ``False`` (returns an array ordered by
                :attr:`motor_ids`).

        Returns:
            Motor positions in radians as an array or dict.
        """
        with self._motor_lock:
            motor_pos = self._motor_client.read_position_velocity_current().position

            if as_dict:
                return {
                    motor_id: pos
                    for motor_id, pos in zip(self.config.motor_ids, motor_pos)
                }

            return motor_pos

    def get_motor_current(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Read the present current drawn by each motor.

        Args:
            as_dict: When ``True`` returns a ``dict`` keyed by motor ID.

        Returns:
            Motor currents (mA) as an array, or dict.
        """
        with self._motor_lock:
            motor_current = self._motor_client.read_position_velocity_current().current

            if as_dict:
                return {
                    motor_id: current
                    for motor_id, current in zip(self.config.motor_ids, motor_current)
                }

            return motor_current

    def wait_for_motion(self, timeout: float = 5.0) -> None:
        """Block until all motors have settled at their commanded position.

        No-op for motor types fast enough that callers don't need to wait
        (e.g., Dynamixel). Feetech polls a per-motor moving flag.

        Args:
            timeout: Max seconds to wait.

        Raises:
            MotionTimeoutError: If motors fail to settle within ``timeout``.
        """
        if not self._motor_client.waits_for_motion:
            return
        with self._motor_lock:
            self._motor_client.wait_for_motion_complete(timeout=timeout)

    def get_motor_temp(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Read the present temperature of each motor.

        Args:
            as_dict: When ``True`` returns a ``dict`` keyed by motor ID.

        Returns:
            Motor temperatures in °C as an array or dict.
        """
        with self._motor_lock:
            motor_temp = self._motor_client.read_temperature()

            if as_dict:
                return {
                    motor_id: temp
                    for motor_id, temp in zip(self.config.motor_ids, motor_temp)
                }

            return motor_temp

    def _get_joint_positions(self) -> OrcaJointPositions:
        motor_pos = self.get_motor_pos()
        return OrcaJointPositions.from_dict(self._motor_to_joint_pos(motor_pos))

    def _set_joint_positions(self, joint_pos: OrcaJointPositions) -> bool:
        motor_pos = self._joint_to_motor_pos(joint_pos.as_dict())
        self._set_motor_pos(motor_pos)
        return True

    def init_joints(self, force_calibrate: bool = False, move_to_neutral: bool = True):
        """Prepare the hand for operation.

        Enables torque, sets the configured control mode and current limit,
        runs calibration if needed, computes wrap offsets, and optionally
        moves to the neutral position.

        Args:
            force_calibrate: Force a fresh calibration even if the hand is
                already calibrated (default ``False``).
            move_to_neutral: Move to the configured neutral pose at the end
                of initialization (default ``True``). Set to ``False`` when
                the caller will immediately command a different pose.
        """
        self.enable_torque()
        self.set_control_mode(self.config.control_mode)
        self.set_max_current(self.config.max_current)

        if not self.calibrated or force_calibrate:
            self.calibrate()

        self._compute_wrap_offsets_dict()

        if move_to_neutral:
            control_mode = self.config.control_mode
            self.set_control_mode(POSITION)  # neutral position is given in POSITION mode
            self.set_joint_positions(
                OrcaJointPositions.from_dict(self.config.neutral_position),
                num_steps=NUM_STEPS
            )
            self.set_control_mode(control_mode)

    def is_calibrated(self, verbose: bool = False) -> bool:
        """Check whether all joints have been fully calibrated.

        Args:
            verbose: When ``True``, prints a warning for each uncalibrated
                motor instead of returning early.

        Returns:
            ``True`` if every motor has valid limits and a non-zero
            joint-to-motor ratio.
        """
        overall_calibrated = True
        uncalibrated_messages = []
        motors_with_warnings = set()

        for motor_id, limits in self.motor_limits_dict.items():
            if any(limit is None for limit in limits):
                overall_calibrated = False
                if not verbose:
                    return False
                joint_name = self.config.motor_to_joint_dict.get(motor_id, "Unknown")
                uncalibrated_messages.append(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing motor limits).\033[0m"
                )
                motors_with_warnings.add(motor_id)

        for motor_id, ratio in self.calibration.joint_to_motor_ratios_dict.items():
            if ratio is None or ratio == 0.0:
                overall_calibrated = False
                if not verbose:
                    return False
                if motor_id not in motors_with_warnings:
                    joint_name = self.config.motor_to_joint_dict.get(
                        motor_id, "Unknown"
                    )
                    uncalibrated_messages.append(
                        f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m"
                    )
                    motors_with_warnings.add(motor_id)

        if verbose:
            for msg in uncalibrated_messages:
                print(msg)

        return overall_calibrated

    def calibrate(
        self,
        blocking: bool = True,
        force_wrist: bool = False,
        joints: list | None = None,
    ):
        """Run the joint calibration routine.

        Drives each joint to its mechanical limits in the sequence defined by
        ``calibration_sequence`` in ``config.yaml``, records the motor positions at
        each limit, and persists the resulting motor limits and joint-to-motor
        ratios to ``calibration.yaml``.

        Args:
            blocking: When True, runs to completion before returning.
            force_wrist: Recalibrate the wrist even if it's already calibrated.
            joints: When given, only calibrate steps that touch any of these
                joint names (others are skipped). Useful for re-running just
                one finger or joint without re-calibrating the whole hand.

        On completion ``self.calibration`` is replaced with a fresh
        :class:`~orca_core.CalibrationResult`. Partial progress is written to
        disk after every step so an interrupted run is never fully lost.
        """
        if blocking:
            self._task_stop_event.clear()
            result = self._calibrate(force_wrist=force_wrist, joints=joints)
            if result is not None:
                self.calibration = result
        else:
            self._start_task(self._calibrate_and_apply, force_wrist=force_wrist, joints=joints)

    def _build_calibration_result(
        self,
        motor_limits: Dict[int, list],
        joint_to_motor_ratios: Dict[int, float],
        wrist_calibrated: bool,
    ) -> CalibrationResult:
        calibrated = all(
            limits[0] is not None and limits[1] is not None
            for limits in motor_limits.values()
        ) and all(
            ratio is not None and ratio != 0.0
            for ratio in joint_to_motor_ratios.values()
        )

        return CalibrationResult(
            motor_limits_dict={
                motor_id: list(limits) for motor_id, limits in motor_limits.items()
            },
            joint_to_motor_ratios_dict=dict(joint_to_motor_ratios),
            calibrated=calibrated,
            wrist_calibrated=wrist_calibrated,
        )

    def _calibrate(
        self,
        force_wrist: bool = False,
        joints: list | None = None,
    ) -> CalibrationResult | None:
        """Execute the calibration routine and return a :class:`~orca_core.CalibrationResult`.

        Drives each joint through its mechanical limits following ``calibration_sequence``
        from ``config.yaml``, records motor positions at each limit, and persists
        the resulting motor limits and joint-to-motor ratios to ``calibration.yaml``
        after every step. Returns ``None`` on early exit (stop event triggered).

        Wrist calibration logic:
        - Wrist is calibrated independently of fingers (tracked by `wrist_calibrated` in calibration file).
        - Uses a higher calibration current.
        - If already calibrated (and calibration run is not forcing), skip wrist steps.
        - If missing from sequence, is calibrated.
        - If force_wrist=True, always include wrist in calibration steps.
        """
        wrist_in_sequence = any(
            "wrist" in step[JOINTS] for step in self.config.calibration_sequence
        )
        calibration_sequence = list(self.config.calibration_sequence)

        if self.wrist_calibrated and not force_wrist:
            if wrist_in_sequence:
                print(
                    "WARNING: Wrist is already calibrated. Skipping wrist calibration. Use --force-wrist to override."
                )
            calibration_sequence = [
                step for step in calibration_sequence if WRIST not in step[JOINTS]
            ]
        elif not wrist_in_sequence:
            # Adds wrist to calibration sequence
            calibration_sequence.append(
                {STEP: len(calibration_sequence) + 1, JOINTS: {WRIST: FLEX}}
            )
            calibration_sequence.append(
                {STEP: len(calibration_sequence) + 1, JOINTS: {WRIST: EXTEND}}
            )

        if joints is not None:
            joints_set = set(joints)
            filtered = []
            for step in calibration_sequence:
                step_joints = {
                    j: d for j, d in step[JOINTS].items() if j in joints_set
                }
                if step_joints:
                    filtered.append({STEP: step[STEP], JOINTS: step_joints})
            print(
                f"Calibrating {len(joints_set)} joint(s) across {len(filtered)} "
                f"step(s) (out of {len(calibration_sequence)} total)."
            )
            calibration_sequence = filtered

        # Deep-copy current limits. Joints not touched by this run keep their
        # values; joints in this run accumulate their new limits in
        # ``pending_limits`` and only overwrite ``motor_limits`` once both
        # flex AND extend have been measured. That way an interrupted run
        # never destroys a previously-good calibration.
        motor_limits = {
            motor_id: list(limits)
            for motor_id, limits in self.calibration.motor_limits_dict.items()
        }
        pending_limits: Dict[int, list] = {
            motor_id: [None, None] for motor_id in self.config.motor_ids
        }
        joint_to_motor_ratios = dict(self.calibration.joint_to_motor_ratios_dict)

        self._compute_wrap_offsets_dict()

        motors_with_initial_offset = set()
        motors_with_final_offset = set()
        
        calibrated_joints: dict = {}

        # Calibration is always done in current-based position mode.
        self.set_control_mode(CURRENT_BASED_POSITION)
        self.set_max_current(self.config.calibration_current)

        for step in calibration_sequence:
            self.disable_torque()

            if self._task_stop_event.is_set():
                return None

            desired_increment, motor_reached_limit, directions = {}, {}, {}
            position_buffers, calibrated_joints, position_logs, current_log = (
                {},
                {},
                {},
                {},
            )

            for joint, direction in step[JOINTS].items():
                self.enable_torque(motor_ids=[self.config.joint_to_motor_map[joint]])
                print(
                    "Enabling torque for the following motor: ",
                    self.config.joint_to_motor_map[joint],
                )

                if self._task_stop_event.is_set():
                    return None

                self.set_max_current(
                    self.config.calibration_current
                    if joint != WRIST
                    else self.config.wrist_calibration_current
                )

                motor_id = self.config.joint_to_motor_map[joint]
                sign = 1 if direction == FLEX else -1
                if self.config.joint_inversion_dict.get(joint, False):
                    sign = -sign

                directions[motor_id] = sign
                self._wrap_offsets_dict[motor_id] = 0.0
                position_buffers[motor_id] = deque(
                    maxlen=self.config.calibration_num_stable
                )
                position_logs[motor_id] = []
                current_log[motor_id] = []
                motor_reached_limit[motor_id] = False

                if (
                    self._motor_client.requires_offset_calibration
                    and motor_id not in motors_with_initial_offset
                ):
                    self._motor_client.calibrate_offset(motor_id, upper=(sign < 0))
                    motors_with_initial_offset.add(motor_id)

            while (
                not all(motor_reached_limit.values())
                and not self._task_stop_event.is_set()
            ):
                desired_increment = {}
                for motor_id, reached_limit in motor_reached_limit.items():
                    if not reached_limit:
                        desired_increment[motor_id] = (
                            directions[motor_id] * self.config.calibration_step_size
                        )

                self._set_motor_pos(desired_increment, rel_to_current=True)
                time.sleep(self.config.calibration_step_period)
                curr_pos = self.get_motor_pos()
                curr_current = self.get_motor_current()

                for motor_id in desired_increment.keys():
                    if not motor_reached_limit[motor_id]:
                        idx = self.config.motor_id_to_idx_dict[motor_id]
                        position_buffers[motor_id].append(curr_pos[idx])
                        position_logs[motor_id].append(float(curr_pos[idx]))
                        current_log[motor_id].append(float(curr_current[idx]))

                        if len(
                            position_buffers[motor_id]
                        ) == self.config.calibration_num_stable and np.allclose(
                            position_buffers[motor_id],
                            position_buffers[motor_id][0],
                            atol=self.config.calibration_threshold,
                        ):
                            motor_reached_limit[motor_id] = True
                            if WRIST in self.config.motor_to_joint_dict[motor_id]:
                                avg_limit = float(np.mean(position_buffers[motor_id]))
                            else:
                                self.disable_torque([motor_id])
                                time.sleep(TINY_SLEEP)
                                avg_limit = float(self.get_motor_pos()[idx])
                            print(
                                f"Motor {motor_id} corresponding to joint {self.config.motor_to_joint_dict[motor_id]} reached the limit at {avg_limit} rad."
                            )
                            if directions[motor_id] == 1:
                                pending_limits[motor_id][1] = avg_limit
                            if directions[motor_id] == -1:
                                pending_limits[motor_id][0] = avg_limit

                            if (
                                self._motor_client.requires_offset_calibration
                                and motor_id not in motors_with_final_offset
                            ):
                                is_positive = directions[motor_id] > 0
                                self._motor_client.calibrate_offset(
                                    motor_id, upper=is_positive
                                )
                                time.sleep(TINY_SLEEP)
                                new_limit = float(self.get_motor_pos()[idx])
                                pending_limits[motor_id][1 if is_positive else 0] = (
                                    new_limit
                                )
                                print(
                                    f"  (Offset adjusted: limit now at {new_limit} rad)"
                                )
                                motors_with_final_offset.add(motor_id)

                            self.enable_torque([motor_id])

            for joint in step[JOINTS].keys():
                motor_id = self.config.joint_to_motor_map[joint]
                if (
                    pending_limits[motor_id][0] is None
                    or pending_limits[motor_id][1] is None
                ):
                    # Only one direction has completed for this joint; leave
                    # motor_limits alone so an interrupted run doesn't lose
                    # the previously-known good values.
                    continue

                # Both directions measured — commit to motor_limits.
                motor_limits[motor_id] = list(pending_limits[motor_id])
                delta_motor = motor_limits[motor_id][1] - motor_limits[motor_id][0]
                delta_joint = (
                    self.config.joint_roms_dict[joint][1]
                    - self.config.joint_roms_dict[joint][0]
                )
                joint_to_motor_ratios[motor_id] = float(delta_motor / delta_joint)
                print("Joint calibrated: ", joint)
                calibrated_joints[joint] = 0.0

            # Persist partial progress after every step so an interrupted run
            # never loses the work already done.
            update_yaml(
                self.config.calibration_path,
                JOINT_TO_MOTOR_RATIOS,
                joint_to_motor_ratios,
            )
            update_yaml(self.config.calibration_path, MOTOR_LIMITS_DICT, motor_limits)

            step_wrist_calibrated = self.calibration.wrist_calibrated or (
                WRIST in calibrated_joints
            )
            self.calibration = self._build_calibration_result(
                motor_limits=motor_limits,
                joint_to_motor_ratios=joint_to_motor_ratios,
                wrist_calibrated=step_wrist_calibrated,
            )
            update_yaml(
                self.config.calibration_path,
                WRIST_CALIBRATED,
                self.calibration.wrist_calibrated,
            )
            update_yaml(
                self.config.calibration_path,
                CALIBRATED,
                self.calibration.calibrated,
            )

            if calibrated_joints:
                self.set_joint_positions(
                    calibrated_joints, num_steps=NUM_STEPS, step_size=STEP_SIZE
                )

            # TODO(fracapuano): Is this necessary?
            time.sleep(0.1)

        new_wrist_calibrated = self.calibration.wrist_calibrated
        if any(WRIST in step[JOINTS] for step in calibration_sequence):
            new_wrist_calibrated = True
            update_yaml(self.config.calibration_path, WRIST_CALIBRATED, True)

        final_result = self._build_calibration_result(
            motor_limits=motor_limits,
            joint_to_motor_ratios=joint_to_motor_ratios,
            wrist_calibrated=new_wrist_calibrated,
        )
        self.calibration = final_result
        update_yaml(self.config.calibration_path, CALIBRATED, final_result.calibrated)

        if calibrated_joints:
            self.set_joint_positions(
                calibrated_joints, num_steps=NUM_STEPS, step_size=TINY_SLEEP
            )

        self.set_max_current(self.config.max_current)

        return final_result

    def set_neutral_position(self, num_steps: int = NUM_STEPS, step_size: float = STEP_SIZE):
        control_mode = self.config.control_mode
        self.set_control_mode(POSITION)
        super().set_neutral_position(num_steps, step_size)
        self.set_control_mode(control_mode)
    
    def _compute_wrap_offsets_dict(self):
        """Detect per-motor encoder wrap-arounds and store correction offsets.

        Reads current motor positions and checks whether any motor has drifted
        more than 1/4pi beyond its configured limits, which indicates the
        rotary encoder has wrapped around a full revolution. For each such motor
        a ±2pi offset is recorded in `_wrap_offsets_dict` so callers can shift
        the raw reading back into the expected operating range. Motors with no
        configured limits, or whose positions are within tolerance, receive an
        offset of 0.
        """
        motor_pos = self.get_motor_pos()

        lower_limit = np.array(
            [self.motor_limits_dict[motor_id][0] for motor_id in self.config.motor_ids]
        )
        higher_limit = np.array(
            [self.motor_limits_dict[motor_id][1] for motor_id in self.config.motor_ids]
        )

        offsets = {}
        for idx, motor_id in enumerate(self.config.motor_ids):
            if lower_limit[idx] is None or higher_limit[idx] is None:
                offsets[motor_id] = 0.0
                continue

            if motor_pos[idx] < lower_limit[idx] - 0.25 * np.pi:
                print(
                    f"Motor ID {motor_id} is out of bounds: {lower_limit[idx]} < {motor_pos[idx]} < {higher_limit[idx]}"
                )
                offsets[motor_id] = -2 * np.pi
            elif motor_pos[idx] > higher_limit[idx] + 0.25 * np.pi:
                print(
                    f"Motor ID {motor_id} is out of bounds: {lower_limit[idx]} < {motor_pos[idx]} < {higher_limit[idx]}"
                )
                offsets[motor_id] = +2 * np.pi
            else:
                offsets[motor_id] = 0.0

        print(f"Offsets: {offsets}")
        self._wrap_offsets_dict = offsets

    def _set_motor_pos(
        self, desired_pos: Union[dict, np.ndarray, list], rel_to_current: bool = False
    ):
        with self._motor_lock:
            if (
                rel_to_current
            ):  # TODO(fracapuano): split in two methods for delta-set or absolute-set
                current_positions = self.get_motor_pos()

            motor_ids_to_write = []
            positions_to_write = []

            if isinstance(desired_pos, dict):
                for motor_id, pos_val in desired_pos.items():
                    if motor_id not in self.config.motor_ids:
                        print(
                            f"Warning: Motor ID {motor_id} in desired_pos dict is not in self.config.motor_ids. Skipping."
                        )
                        continue
                    if pos_val is None or math.isnan(pos_val):
                        continue

                    pos_to_write = float(pos_val)
                    if rel_to_current:
                        pos_to_write += current_positions[
                            self.config.motor_id_to_idx_dict[motor_id]
                        ]

                    motor_ids_to_write.append(motor_id)
                    positions_to_write.append(pos_to_write)

                if not motor_ids_to_write:
                    return
                positions_to_write = np.array(positions_to_write, dtype=float)

            elif isinstance(desired_pos, (np.ndarray, list)):
                if len(desired_pos) != len(self.config.motor_ids):
                    raise ValueError(
                        f"Length of desired_pos (list/ndarray) ({len(desired_pos)}) must match the number of configured motor_ids ({len(self.config.motor_ids)})."
                    )

                for idx, pos_val in enumerate(desired_pos):
                    if pos_val is None or math.isnan(pos_val):
                        continue

                    motor_ids_to_write.append(self.config.motor_ids[idx])
                    if rel_to_current:
                        positions_to_write.append(
                            float(pos_val) + current_positions[idx]
                        )
                    else:
                        positions_to_write.append(float(pos_val))

                if not motor_ids_to_write:
                    print(
                        "\033[93mWarning: All positions in desired_pos (list/array) were None. No motor commands sent.\033[0m"
                    )
                    return

                positions_to_write = np.array(positions_to_write, dtype=float)

            else:
                raise ValueError("desired_pos must be a dict, np.ndarray, or list.")

            self._motor_client.write_desired_pos(motor_ids_to_write, positions_to_write)

    def _motor_to_joint_pos(self, motor_pos: np.ndarray) -> dict:
        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()

        joint_pos = {}
        for idx, pos in enumerate(motor_pos):
            motor_id = self.config.motor_ids[idx]
            joint_name = self.config.motor_to_joint_dict.get(motor_id)
            if any(limit is None for limit in self.motor_limits_dict[motor_id]):
                joint_pos[joint_name] = None
                print(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing motor limits).\033[0m"
                )
            elif self.calibration.joint_to_motor_ratios_dict[motor_id] == 0:
                joint_pos[joint_name] = None
                print(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m"
                )
            else:
                wrapped_pos = pos - self._wrap_offsets_dict.get(motor_id, 0.0)
                if self.config.joint_inversion_dict.get(joint_name, False):
                    joint_pos[joint_name] = (
                        self.config.joint_roms_dict[joint_name][1]
                        - (wrapped_pos - self.motor_limits_dict[motor_id][0])
                        / self.calibration.joint_to_motor_ratios_dict[motor_id]
                    )
                else:
                    joint_pos[joint_name] = (
                        self.config.joint_roms_dict[joint_name][0]
                        + (wrapped_pos - self.motor_limits_dict[motor_id][0])
                        / self.calibration.joint_to_motor_ratios_dict[motor_id]
                    )
        return joint_pos

    def _joint_to_motor_pos(self, joint_pos: dict) -> np.ndarray:
        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()

        motor_pos = [None] * len(self.config.motor_ids)

        for joint_name, pos in joint_pos.items():
            motor_id = self.config.joint_to_motor_map.get(joint_name)
            if motor_id is None:
                continue

            if pos is None:
                motor_pos[self.config.motor_id_to_idx_dict[motor_id]] = None
                continue

            if (
                self.motor_limits_dict[motor_id][0] is None
                or self.motor_limits_dict[motor_id][1] is None
                or self.calibration.joint_to_motor_ratios_dict[motor_id] == 0
            ):
                motor_pos[self.config.motor_id_to_idx_dict[motor_id]] = None
                print(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m"
                )
                continue

            if self.config.joint_inversion_dict.get(joint_name, False):
                motor_pos[self.config.motor_id_to_idx_dict[motor_id]] = (
                    self.motor_limits_dict[motor_id][0]
                    + (self.config.joint_roms_dict[joint_name][1] - pos)
                    * self.calibration.joint_to_motor_ratios_dict[motor_id]
                )
            else:
                motor_pos[self.config.motor_id_to_idx_dict[motor_id]] = (
                    self.motor_limits_dict[motor_id][0]
                    + (pos - self.config.joint_roms_dict[joint_name][0])
                    * self.calibration.joint_to_motor_ratios_dict[motor_id]
                )

            motor_pos[self.config.motor_id_to_idx_dict[motor_id]] += (
                self._wrap_offsets_dict.get(motor_id, 0.0)
            )

        return motor_pos

    def _sanity_check(self):
        for motor_limit in self.motor_limits_dict.values():
            if any(limit is None for limit in motor_limit):
                self.calibration = dataclasses.replace(
                    self.calibration, calibrated=False
                )
                update_yaml(self.config.calibration_path, "calibrated", False)

    def tension(self, move_motors: bool = True, blocking: bool = True):
        """Hold motors under current to allow manual tendon tensioning.

        Optionally pre-conditions the tendons with a short back-and-forth
        motion before entering the hold phase. Torque is disabled automatically
        on exit.

        Args:
            move_motors: When ``True``, execute a short flexion/extension cycle
                before holding (default ``True``).
            blocking: When ``True`` (default) blocks until the user interrupts
                with Ctrl-C. When ``False`` runs in a background thread.
        """
        if blocking:
            self._task_stop_event.clear()
            self._tension(move_motors)
        else:
            self._start_task(self._tension, move_motors)

    def jitter(
        self,
        motor_ids: List[int] = None,
        amplitude: float = 5.0,
        frequency: float = 10.0,
        duration: float = 3.0,
        include_wrist: bool = False,
        blocking: bool = True,
    ):
        """Apply a sinusoidal jitter to the motors for tendon seating.

        All motors oscillate around their current position with a sine wave.
        Amplitude is capped at 10° for safety.

        Args:
            motor_ids: Motors to jitter. Defaults to all non-wrist motors (or
                all motors when *include_wrist* is ``True``).
            amplitude: Peak-to-peak amplitude in degrees (default ``5.0``,
                max ``10.0``).
            frequency: Oscillation frequency in Hz (default ``10.0``).
            duration: Total jitter duration in seconds (default ``3.0``).
            include_wrist: Include the wrist motor when *motor_ids* is
                ``None`` (default ``False``).
            blocking: When ``True`` (default) blocks until jitter completes.
                When ``False`` runs in a background thread.

        Raises:
            ValueError: If *amplitude* exceeds 10°.
        """
        if blocking:
            self._task_stop_event.clear()
            self._jitter(motor_ids, amplitude, frequency, duration, include_wrist)
        else:
            self._start_task(
                self._jitter, motor_ids, amplitude, frequency, duration, include_wrist
            )

    def _jitter(
        self,
        motor_ids: List[int] = None,
        amplitude: float = 5.0,
        frequency: float = 10.0,
        duration: float = 3.0,
        include_wrist: bool = False,
    ):
        max_amplitude_deg = 10.0
        if amplitude > max_amplitude_deg:
            raise ValueError(
                f"Amplitude must be <= {max_amplitude_deg} degrees for safety. Got {amplitude}."
            )

        amplitude_rad = np.deg2rad(amplitude)

        if motor_ids is None:
            wrist_motor_id = self.config.joint_to_motor_map.get("wrist")
            motor_ids = [
                mid
                for mid in self.config.motor_ids
                if include_wrist or mid != wrist_motor_id
            ]

        start_positions = self.get_motor_pos(as_dict=True)
        start_pos_array = np.array([start_positions[mid] for mid in motor_ids])

        # Feetech (and similar) issue one bus transaction per motor per update.
        # Without a throttle, the inner loop floods the serial link and TxRx
        # fails ("no status packet" / "incorrect status packet").
        jitter_period_s = 0.01

        start_time = time.time()
        while (
            time.time() - start_time < duration and not self._task_stop_event.is_set()
        ):
            t = time.time() - start_time
            offset = amplitude_rad * math.sin(2 * math.pi * frequency * t)
            with self._motor_lock:
                self._motor_client.write_desired_pos(
                    motor_ids, start_pos_array + offset
                )
            time.sleep(jitter_period_s)

        with self._motor_lock:
            self._motor_client.write_desired_pos(motor_ids, start_pos_array)

    def _tension(self, move_motors: bool = True):
        # TODO(fracapuano): Move this to a standard stateless function
        control_mode = self.config.control_mode
        self.set_control_mode(CURRENT_BASED_POSITION)
        if move_motors:
            motors_to_move = [
                motor_id
                for joint, motor_id in self.config.joint_to_motor_map.items()
                if WRIST not in joint.lower() and motor_id in self.config.motor_ids
            ]
            self.set_max_current(self.config.calibration_current)

            duration = 5
            increment_per_step = 0.1
            motor_increments_right = {
                motor_id: increment_per_step for motor_id in motors_to_move
            }
            motor_increments_left = {
                motor_id: -increment_per_step for motor_id in motors_to_move
            }

            stall_threshold = 0.01
            stall_hold = 1.0

            for increments in (motor_increments_left, motor_increments_right):
                stall_start = None
                prev_pos = self.get_motor_pos()
                while not self._task_stop_event.is_set():
                    self._set_motor_pos(increments, rel_to_current=True)
                    time.sleep(0.1)
                    cur_pos = self.get_motor_pos()
                    if np.max(np.abs(cur_pos - prev_pos)) < stall_threshold:
                        if stall_start is None:
                            stall_start = time.time()
                        elif time.time() - stall_start >= stall_hold:
                            break
                    else:
                        stall_start = None
                    prev_pos = cur_pos

        # Gradually release torque so tendons don't snap back, then
        # re-engage at the relaxed position for a stable hold.
        max_cur = self.config.max_current
        self.set_max_current(max_cur)
        self.enable_torque()
        steps = 20
        for i in range(steps):
            if self._task_stop_event.is_set():
                break
            self.set_max_current(max_cur * (1 - (i + 1) / steps))
            time.sleep(1.0 / steps)
        self.disable_torque()
        time.sleep(0.05)
        self.set_max_current(max_cur)
        self.enable_torque()
        print("Holding motors. Please tension carefully. Press Ctrl+C to exit.")
        try:
            while not self._task_stop_event.is_set():
                time.sleep(0.1)
        finally:
            self.set_control_mode(control_mode)
            self.disable_torque()

    def _run_task(self, task_fn, *args, **kwargs):
        with self._lock:
            self._task_stop_event.clear()
            self._current_task = task_fn.__name__
            try:
                task_fn(*args, **kwargs)
            finally:
                self._current_task = None

    def _start_task(self, task_fn, *args, **kwargs):
        if self._task_thread and self._task_thread.is_alive():
            print(f"Task '{self._current_task}' is already running.")
            return

        self._task_thread = threading.Thread(
            target=self._run_task, args=(task_fn,) + args, kwargs=kwargs
        )
        self._task_thread.start()

    def stop_task(self):
        """Stops a background task like calibration, tensioning or jittering."""
        if self._task_thread and self._task_thread.is_alive():
            self._task_stop_event.set()
            self._task_thread.join()
            print("Task stopped.")
        else:
            print("No running task to stop.")


class OrcaHandTouch(OrcaHand):
    """ORCA hand with integrated tactile sensing.

    ``connect()`` opens both the motor bus and the sensor serial link;
    ``disconnect()`` tears down both.
    """

    config_cls = OrcaHandTouchConfig

    def __init__(
        self,
        config_path: str | None = None,
        calibration_path: str | None = None,
        model_version: str | None = None,
        model_name: str | None = None,
        config: OrcaHandTouchConfig | None = None,
    ):
        super().__init__(
            config_path=config_path,
            calibration_path=calibration_path,
            model_version=model_version,
            model_name=model_name,
            config=config,
        )
        self._tactile_client = None

    def _create_tactile_client(self):
        return TactileClient(
            port=self.config.sensor_port,
            baudrate=self.config.sensor_baudrate,
            finger_to_sensor_id=self.config.finger_to_sensor_id,
        )

    def _persist_sensor_port(self, chosen_port: str) -> None:
        """Write a new ``sensors.port`` value to ``config.yaml`` while preserving
        the rest of the ``sensors`` block (baudrate, finger_to_sensor_id)."""
        existing = read_yaml(self.config.config_path) or {}
        sensors = dict(existing.get("sensors") or {})
        sensors["port"] = chosen_port
        update_yaml(self.config.config_path, "sensors", sensors)

    def _connect_sensor_with_fallback(self) -> tuple[bool, str]:
        """Open the sensor serial link, mirroring the motor cascade.

        Tries the configured port first, then USB-VID auto-detection
        (``KNOWN_VIDS["tactile_sensor"]``). On a successful auto-detect the
        new port is written back to ``config.yaml``.
        """
        self._tactile_client = self._create_tactile_client()
        try:
            self._tactile_client.connect()
            if self._tactile_client.get_tactile_configuration() is not None:
                return True, f"Sensor connected on {self.config.sensor_port}"
            print(f"Port {self.config.sensor_port} opened but sensor did not respond; "
                  "trying auto-detection.")
            self._tactile_client.disconnect()
            self._tactile_client = None
        except Exception as e:
            print(f"Sensor connection failed on {self.config.sensor_port}: {e}")
            self._tactile_client = None

        chosen = auto_detect_port("tactile_sensor")
        if chosen:
            try:
                self.config = dataclasses.replace(self.config, sensor_port=chosen)
                self._tactile_client = self._create_tactile_client()
                self._tactile_client.connect()
                if self._tactile_client.get_tactile_configuration() is None:
                    print(f"Auto-detected port {chosen} opened but sensor did not respond.")
                    self._tactile_client.disconnect()
                    self._tactile_client = None
                else:
                    self._persist_sensor_port(chosen)
                    return True, f"Sensor connected on auto-detected {chosen}"
            except Exception as e:
                print(f"Auto-detected sensor port {chosen} also failed: {e}")
                self._tactile_client = None

        return False, (
            "Sensor connection failed: no usable port (set sensors.port in config.yaml "
            "or check that the sensor adapter is plugged in)"
        )

    def connect(self) -> tuple[bool, str]:
        success, msg = super().connect()
        if not success:
            return success, msg

        sensor_ok, sensor_msg = self._connect_sensor_with_fallback()
        if not sensor_ok:
            return False, f"{msg} | {sensor_msg}"
        return True, f"{msg} | {sensor_msg}"

    def connect_sensors_only(self) -> tuple[bool, str]:
        """Connect only the tactile sensor, skipping the motor bus.

        Useful for sensor bring-up and testing on a hand whose motors are not
        powered. After this call, tactile methods work; motor-control methods
        will fail because the motor client is not initialised.
        """
        return self._connect_sensor_with_fallback()

    def disconnect(self) -> None:
        if self._tactile_client is not None and self._tactile_client.is_connected:
            try:
                self._tactile_client.stop_auto_stream()
            except Exception:
                pass
            self._tactile_client.disconnect()
        self._tactile_client = None
        super().disconnect()

    def get_tactile_forces(self) -> ResultantReading | None:
        """Return the latest resultant ``ResultantReading``, or ``None`` if no frame yet."""
        forces, ts = self._tactile_client.get_auto_latest()
        if forces is None:
            return None
        return ResultantReading(forces=forces, timestamp=ts)

    def get_tactile_taxels(self) -> TaxelReading | None:
        """Return the latest per-taxel ``TaxelReading``, or ``None`` if no frame yet."""
        taxels, ts = self._tactile_client.get_auto_latest_taxels()
        if taxels is None:
            return None
        return TaxelReading(taxels=taxels, timestamp=ts)

    def get_tactile_data(self) -> TactileReading | None:
        """Return resultant and per-taxel forces from the same frame.

        Single locked snapshot of the auto-stream cache, so ``forces`` and
        ``taxels`` are guaranteed to share a timestamp. Either field is
        ``None`` if its stream mode is disabled. Returns ``None`` if no
        frame has arrived yet.
        """
        forces, taxels, ts = self._tactile_client.get_auto_latest_all()
        if forces is None and taxels is None:
            return None
        return TactileReading(
            forces=ResultantReading(forces=forces, timestamp=ts) if forces is not None else None,
            taxels=TaxelReading(taxels=taxels, timestamp=ts) if taxels is not None else None,
            timestamp=ts,
        )

    def start_tactile_stream(
        self, resultant: bool = True, taxels: bool = False, min_sensors: int = 1
    ) -> None:
        self._tactile_client.start_auto_stream(
            resultant=resultant, taxels=taxels, min_sensors=min_sensors,
        )

    def stop_tactile_stream(self) -> None:
        self._tactile_client.stop_auto_stream()

    def zero_tactile_sensors(self, num_samples: int = 100) -> dict:
        """Capture current readings as zero baseline and return offsets."""
        return self._tactile_client.capture_taxel_offsets(num_samples=num_samples)

    def clear_tactile_zero(self) -> None:
        self._tactile_client.clear_taxel_offsets()

    def get_tactile_configuration(self):
        return self._tactile_client.get_tactile_configuration()

    def get_tactile_stats(self):
        """Return ``AutoStreamStats`` for the running auto-stream."""
        return self._tactile_client.get_auto_stats()


class MockOrcaHand(OrcaHand):
    """Drop-in :class:`OrcaHand` backed by an in-memory mock motor client,
    for testing and prototyping.

    All methods behave identically to :class:`OrcaHand` but no serial
    port is opened and motor state is simulated in memory.
    """

    def _resolve_port(self) -> str:
        return self.config.port or "/dev/null"

    def _trial_probe(self, port: str) -> tuple[str, int]:
        # Mock hand has no real bus to probe — pretend Dynamixel @ 1M.
        return "dynamixel", 1_000_000

    def _create_motor_client(
        self,
        motor_type: str,
        port: str,
        baudrate: int,
    ) -> MotorClient:
        from .hardware.mock_dynamixel_client import MockDynamixelClient

        return MockDynamixelClient(
            self.config.motor_ids, self.config.port, self.config.baudrate
        )


class MockOrcaHandTouch(OrcaHandTouch):
    """Drop-in :class:`OrcaHandTouch` with in-memory mock motor + sensor clients (no serial I/O)."""

    def _resolve_port(self) -> str:
        return self.config.port or "/dev/null"

    def _trial_probe(self, port: str) -> tuple[str, int]:
        return "dynamixel", 1_000_000

    def _create_motor_client(
        self,
        motor_type: str,
        port: str,
        baudrate: int,
    ) -> MotorClient:
        from .hardware.mock_dynamixel_client import MockDynamixelClient

        return MockDynamixelClient(
            self.config.motor_ids, self.config.port, self.config.baudrate
        )

    def _create_tactile_client(self):
        return MockTactileClient(
            port="mock",
            baudrate=self.config.sensor_baudrate,
            finger_to_sensor_id=self.config.finger_to_sensor_id,
        )
