from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import replace
from threading import RLock
from typing import TYPE_CHECKING, Dict, List, Union

import numpy as np

from .drivers import create_motor_client
from .hardware.motor_client import MotorClient
from .profile import DEFAULT_PROFILE_NAME, HandProfile, load_legacy_calibration_data, load_profile, load_profile_from_path
from .state import FileStateStore, HandState, StateStore

if TYPE_CHECKING:
    from .hardware.dynamixel_client import DynamixelClient
    from .hardware.feetech_client import FeetechClient


class OrcaHand:
    """High-level joint-space controller for an ORCA hand profile."""

    def __init__(
        self,
        model_path: str | None = None,
        *,
        profile: HandProfile | None = None,
        profile_name: str | None = None,
        driver: MotorClient | None = None,
        state_store: StateStore | None = None,
        state: HandState | None = None,
        port: str | None = None,
    ):
        if profile is None:
            if model_path is not None:
                profile = load_profile_from_path(model_path)
            else:
                profile = load_profile(profile_name or DEFAULT_PROFILE_NAME)

        self.profile = profile
        self.model_path = self.profile.source_path
        self.config_path = f"{self.model_path}/config.yaml" if self.model_path else None
        self.calib_path = f"{self.model_path}/calibration.yaml" if self.model_path else None

        self._state_store = state_store or FileStateStore()
        self._driver_override = driver

        loaded_state = state or self._state_store.load(self.profile)
        if model_path is not None:
            legacy_data = load_legacy_calibration_data(model_path)
            if legacy_data and not loaded_state.calibrated:
                loaded_state = HandState.from_dict(legacy_data, profile=self.profile)
                if port is not None:
                    loaded_state.port = port

        self._state = loaded_state

        self.baudrate: int = self.profile.driver.baudrate
        self.port: str | None = port or self._state.port or self.profile.driver.port
        self.max_current: int = self.profile.max_current
        self.control_mode: str = self.profile.control_mode
        self.type: str | None = self.profile.hand_type
        self.motor_type: str = self.profile.driver.kind

        self.calib_current: int = self.profile.calib_current
        self.wrist_calib_current: int = self.profile.wrist_calib_current
        self.calib_step_size: float = self.profile.calib_step_size
        self.calib_step_period: float = self.profile.calib_step_period
        self.calib_threshold: float = self.profile.calib_threshold
        self.calib_num_stable: int = self.profile.calib_num_stable
        self.calib_sequence: List[dict] = self.profile.calib_sequence
        self.calibrated: bool = self._state.calibrated
        self.wrist_calibrated: bool = self._state.wrist_calibrated

        self.neutral_position: Dict[str, float] = dict(self.profile.neutral_position)

        self.motor_ids: List[int] = list(self.profile.motor_ids)
        self.joint_ids: List[str] = list(self.profile.joint_ids)
        self.motor_id_to_idx_dict: Dict[int, int] = {
            motor_id: i for i, motor_id in enumerate(self.motor_ids)
        }

        self.motor_limits_dict: Dict[int, List[float | None]] = {
            motor_id: list(self._state.motor_limits.get(motor_id, [None, None]))
            for motor_id in self.motor_ids
        }
        self.joint_to_motor_ratios_dict: Dict[int, float] = {
            motor_id: float(self._state.joint_to_motor_ratios.get(motor_id, 0.0))
            for motor_id in self.motor_ids
        }

        self.joint_to_motor_map: Dict[str, int] = dict(self.profile.joint_to_motor_map)
        self.joint_roms_dict: Dict[str, List[float]] = dict(self.profile.joint_roms)

        self.joint_inversion_dict: Dict[str, bool] = {}
        for joint, motor_id in self.joint_to_motor_map.items():
            if motor_id < 0 or math.copysign(1, motor_id) < 0:
                self.joint_inversion_dict[joint] = True
                self.joint_to_motor_map[joint] = int(abs(motor_id))
            else:
                self.joint_inversion_dict[joint] = False

        self.joint_to_motor_map = {
            joint: int(motor_id) for joint, motor_id in self.joint_to_motor_map.items()
        }
        self.motor_to_joint_dict: Dict[int, str] = {
            motor_id: joint for joint, motor_id in self.joint_to_motor_map.items()
        }

        self._wrap_offsets_dict: Dict[int, float] | None = None
        self._motor_client: MotorClient | None = None
        self._motor_lock: RLock = RLock()

        self._task_thread: threading.Thread | None = None
        self._task_stop_event = threading.Event()
        self._lock = threading.Lock()
        self._current_task: str | None = None

        self._sanity_check()
        self.is_calibrated(verbose=True)

    def __del__(self):
        try:
            self.disconnect()
        except Exception:
            pass

    @property
    def state(self) -> HandState:
        self._sync_runtime_state()
        return self._state

    def _sync_runtime_state(self) -> None:
        self._state.port = self.port
        self._state.calibrated = self.calibrated
        self._state.wrist_calibrated = self.wrist_calibrated
        self._state.motor_limits = {
            motor_id: list(self.motor_limits_dict[motor_id]) for motor_id in self.motor_ids
        }
        self._state.joint_to_motor_ratios = {
            motor_id: float(self.joint_to_motor_ratios_dict[motor_id]) for motor_id in self.motor_ids
        }

    def _persist_state(self) -> None:
        self._sync_runtime_state()
        self._state_store.save(self.profile, self._state)

    def _create_motor_client(self) -> MotorClient:
        return create_motor_client(
            self.profile,
            port=self.port,
            driver=self._driver_override,
        )

    def connect(self, port: str | None = None) -> tuple[bool, str]:
        if port is not None:
            self.port = port

        if self.port is None:
            return False, "Connection failed: no port configured"

        try:
            self._motor_client = self._create_motor_client()
            with self._motor_lock:
                self._motor_client.connect()
            self._persist_state()
            return True, f"Connection successful on {self.port}"
        except Exception as exc:
            self._motor_client = None
            return False, f"Connection failed on {self.port}: {exc}"

    def disconnect(self) -> tuple[bool, str]:
        if self._motor_client is None:
            return True, "Disconnected successfully"

        try:
            with self._motor_lock:
                try:
                    self.disable_torque()
                    time.sleep(0.1)
                except Exception:
                    pass
                self._motor_client.disconnect()
            return True, "Disconnected successfully"
        except Exception as exc:
            return False, f"Disconnection failed: {exc}"
        finally:
            self._motor_client = None

    def is_connected(self) -> bool:
        return self._motor_client.is_connected if self._motor_client else False

    def enable_torque(self, motor_ids: List[int] | None = None):
        if motor_ids is None:
            motor_ids = self.motor_ids
        with self._motor_lock:
            self._motor_client.set_torque_enabled(motor_ids, True)

    def disable_torque(self, motor_ids: List[int] | None = None):
        if motor_ids is None:
            motor_ids = self.motor_ids
        with self._motor_lock:
            self._motor_client.set_torque_enabled(motor_ids, False)

    def set_max_current(self, current: Union[float, List[float]]):
        if isinstance(current, list):
            if len(current) != len(self.motor_ids):
                raise ValueError("Number of currents do not match the number of motors.")
            with self._motor_lock:
                self._motor_client.write_desired_current(self.motor_ids, current)
        else:
            with self._motor_lock:
                self._motor_client.write_desired_current(
                    self.motor_ids,
                    current * np.ones(len(self.motor_ids)),
                )

    def set_control_mode(self, mode: str, motor_ids: List[int] | None = None):
        mode_map = {
            "current": 0,
            "velocity": 1,
            "position": 3,
            "multi_turn_position": 4,
            "current_based_position": 5,
        }

        mode_value = mode_map.get(mode)
        if mode_value is None:
            raise ValueError("Invalid control mode.")

        with self._motor_lock:
            if motor_ids is None:
                motor_ids = self.motor_ids
            elif not all(motor_id in self.motor_ids for motor_id in motor_ids):
                raise ValueError("Invalid motor IDs.")

        if mode_value in (0, 5):
            wrist_motor_id = self.joint_to_motor_map.get("wrist")
            if wrist_motor_id is not None:
                motor_ids_without_wrist = [
                    motor_id for motor_id in motor_ids if motor_id != wrist_motor_id
                ]
                self._motor_client.set_operating_mode(motor_ids_without_wrist, mode_value)
                if wrist_motor_id in motor_ids:
                    self._motor_client.set_operating_mode([wrist_motor_id], 4)
            else:
                self._motor_client.set_operating_mode(motor_ids, mode_value)
        else:
            self._motor_client.set_operating_mode(motor_ids, mode_value)

    def get_motor_pos(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        with self._motor_lock:
            motor_pos = self._motor_client.read_pos_vel_cur()[0]
            if as_dict:
                return {motor_id: pos for motor_id, pos in zip(self.motor_ids, motor_pos)}
            return motor_pos

    def get_motor_current(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        with self._motor_lock:
            motor_current = self._motor_client.read_pos_vel_cur()[2]
            if as_dict:
                return {motor_id: current for motor_id, current in zip(self.motor_ids, motor_current)}
            return motor_current

    def get_motor_temp(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        with self._motor_lock:
            motor_temp = self._motor_client.read_temperature()
            if as_dict:
                return {motor_id: temp for motor_id, temp in zip(self.motor_ids, motor_temp)}
            return motor_temp

    def get_joint_pos(self, as_list: bool = True) -> Union[dict, list]:
        motor_pos = self.get_motor_pos()
        joint_pos = self._motor_to_joint_pos(motor_pos)
        if as_list:
            return [joint_pos[joint] for joint in self.joint_ids]
        return joint_pos

    def set_joint_pos(self, joint_pos: Union[dict, list], num_steps: int = 1, step_size: float = 1.0):
        if num_steps > 1:
            current_positions = self.get_joint_pos(as_list=False)

            if isinstance(joint_pos, list):
                if len(joint_pos) != len(self.joint_ids):
                    raise ValueError("Length of joint_pos list must match the number of joint_ids.")
                target_positions = {joint: pos for joint, pos in zip(self.joint_ids, joint_pos)}
            else:
                target_positions = joint_pos.copy()

            for step in range(num_steps + 1):
                t = step / num_steps
                interpolated_positions = {}
                for joint in self.joint_ids:
                    if joint in target_positions:
                        current_pos = current_positions[joint]
                        if current_pos is None or target_positions[joint] is None:
                            interpolated_positions[joint] = None
                            continue
                        target_pos = target_positions[joint]
                        interpolated_positions[joint] = current_pos * (1 - t) + target_pos * t
                    else:
                        interpolated_positions[joint] = current_positions[joint]

                motor_pos = self._joint_to_motor_pos(interpolated_positions)
                self._set_motor_pos(motor_pos)
                if step < num_steps:
                    time.sleep(step_size)
            return

        if isinstance(joint_pos, dict):
            motor_pos = self._joint_to_motor_pos(joint_pos)
        elif isinstance(joint_pos, list):
            if len(joint_pos) != len(self.joint_ids):
                raise ValueError("Length of joint_pos list must match the number of joint_ids.")
            joint_pos_dict = {joint: pos for joint, pos in zip(self.joint_ids, joint_pos)}
            motor_pos = self._joint_to_motor_pos(joint_pos_dict)
        else:
            raise ValueError("joint_pos must be a dict or a list.")

        self._set_motor_pos(motor_pos)

    def set_zero_position(self, num_steps: int = 25, step_size: float = 0.001):
        self.set_joint_pos({joint: 0 for joint in self.joint_ids}, num_steps=num_steps, step_size=step_size)

    def set_neutral_position(self, num_steps: int = 25, step_size: float = 0.001):
        if self.neutral_position is None:
            raise ValueError("Neutral position is not set for this profile.")
        self.set_joint_pos(self.neutral_position, num_steps=num_steps, step_size=step_size)

    def init_joints(self, calibrate: bool = False):
        self.enable_torque()
        self.set_control_mode(self.control_mode)
        self.set_max_current(self.max_current)

        if not self.calibrated or calibrate:
            self.calibrate()

        self._compute_wrap_offsets_dict()
        self.set_joint_pos(self.neutral_position)

    def is_calibrated(self, verbose: bool = False) -> bool:
        overall_calibrated = True
        uncalibrated_messages = []
        motors_with_warnings = set()

        for motor_id, limits in self.motor_limits_dict.items():
            if any(limit is None for limit in limits):
                overall_calibrated = False
                if not verbose:
                    return False
                joint_name = self.motor_to_joint_dict.get(motor_id, "Unknown")
                uncalibrated_messages.append(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing motor limits).\033[0m"
                )
                motors_with_warnings.add(motor_id)

        for motor_id, ratio in self.joint_to_motor_ratios_dict.items():
            if ratio is None or ratio == 0.0:
                overall_calibrated = False
                if not verbose:
                    return False
                if motor_id not in motors_with_warnings:
                    joint_name = self.motor_to_joint_dict.get(motor_id, "Unknown")
                    uncalibrated_messages.append(
                        f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m"
                    )
                    motors_with_warnings.add(motor_id)

        if verbose:
            for msg in uncalibrated_messages:
                print(msg)

        return overall_calibrated

    def calibrate(self, blocking: bool = True, force_wrist: bool = False):
        if blocking:
            self._task_stop_event.clear()
            self._calibrate(force_wrist=force_wrist)
        else:
            self._start_task(self._calibrate, force_wrist=force_wrist)

    def _calibrate(self, force_wrist: bool = False):
        wrist_in_sequence = any("wrist" in step["joints"] for step in self.calib_sequence)
        calib_sequence = list(self.calib_sequence)

        if self.wrist_calibrated and not force_wrist:
            if wrist_in_sequence:
                print("WARNING: Wrist is already calibrated. Skipping wrist calibration. Use --force-wrist to override.")
            calib_sequence = [step for step in calib_sequence if "wrist" not in step["joints"]]
        elif not wrist_in_sequence:
            calib_sequence.append({"step": len(calib_sequence) + 1, "joints": {"wrist": "flex"}})
            calib_sequence.append({"step": len(calib_sequence) + 1, "joints": {"wrist": "extend"}})

        motor_limits = self.motor_limits_dict.copy()

        self._compute_wrap_offsets_dict()
        for step in calib_sequence:
            for joint in step["joints"].keys():
                motor_id = self.joint_to_motor_map[joint]
                motor_limits[motor_id] = [None, None]
                self._wrap_offsets_dict[motor_id] = 0.0

        motors_with_initial_offset = set()
        motors_with_final_offset = set()

        self.set_control_mode("current_based_position")
        self.set_max_current(self.calib_current)

        calibrated_joints = {}

        for step in calib_sequence:
            self.disable_torque()

            if self._task_stop_event.is_set():
                return

            desired_increment = {}
            motor_reached_limit = {}
            directions = {}
            position_buffers = {}
            position_logs = {}
            current_log = {}
            calibrated_joints = {}

            for joint, direction in step["joints"].items():
                self.enable_torque(motor_ids=[self.joint_to_motor_map[joint]])
                print("Enabling torque for the following motor: ", self.joint_to_motor_map[joint])

                if self._task_stop_event.is_set():
                    return

                if joint == "wrist":
                    self.set_max_current(self.wrist_calib_current)
                else:
                    self.set_max_current(self.calib_current)

                motor_id = self.joint_to_motor_map[joint]
                sign = 1 if direction == "flex" else -1
                if self.joint_inversion_dict.get(joint, False):
                    sign = -sign
                directions[motor_id] = sign
                position_buffers[motor_id] = deque(maxlen=self.calib_num_stable)
                position_logs[motor_id] = []
                current_log[motor_id] = []
                motor_reached_limit[motor_id] = False

                if self._motor_client.requires_offset_calibration and motor_id not in motors_with_initial_offset:
                    self._motor_client.calibrate_offset(motor_id, upper=(sign < 0))
                    motors_with_initial_offset.add(motor_id)

            while not all(motor_reached_limit.values()) and not self._task_stop_event.is_set():
                for motor_id, reached_limit in motor_reached_limit.items():
                    if not reached_limit:
                        desired_increment[motor_id] = directions[motor_id] * self.calib_step_size

                self._set_motor_pos(desired_increment, rel_to_current=True)
                time.sleep(self.calib_step_period)
                curr_pos = self.get_motor_pos()

                for motor_id in desired_increment.keys():
                    if motor_reached_limit[motor_id]:
                        continue
                    position_buffers[motor_id].append(curr_pos[self.motor_id_to_idx_dict[motor_id]])
                    position_logs[motor_id].append(float(curr_pos[self.motor_id_to_idx_dict[motor_id]]))
                    current_log[motor_id].append(float(self.get_motor_current()[self.motor_id_to_idx_dict[motor_id]]))

                    if len(position_buffers[motor_id]) != self.calib_num_stable:
                        continue
                    if not np.allclose(
                        position_buffers[motor_id],
                        position_buffers[motor_id][0],
                        atol=self.calib_threshold,
                    ):
                        continue

                    motor_reached_limit[motor_id] = True
                    if "wrist" in self.motor_to_joint_dict[motor_id]:
                        avg_limit = float(np.mean(position_buffers[motor_id]))
                    else:
                        self.disable_torque([motor_id])
                        time.sleep(0.05)
                        avg_limit = float(self.get_motor_pos()[self.motor_id_to_idx_dict[motor_id]])

                    print(
                        f"Motor {motor_id} corresponding to joint {self.motor_to_joint_dict[motor_id]} reached the limit at {avg_limit} rad."
                    )
                    if directions[motor_id] == 1:
                        motor_limits[motor_id][1] = avg_limit
                    if directions[motor_id] == -1:
                        motor_limits[motor_id][0] = avg_limit

                    if self._motor_client.requires_offset_calibration and motor_id not in motors_with_final_offset:
                        is_positive = directions[motor_id] > 0
                        self._motor_client.calibrate_offset(motor_id, upper=is_positive)
                        time.sleep(0.05)
                        new_limit = float(self.get_motor_pos()[self.motor_id_to_idx_dict[motor_id]])
                        motor_limits[motor_id][1 if is_positive else 0] = new_limit
                        print(f"  (Offset adjusted: limit now at {new_limit} rad)")
                        motors_with_final_offset.add(motor_id)

                    self.enable_torque([motor_id])

            for joint in step["joints"].keys():
                motor_id = self.joint_to_motor_map[joint]
                if motor_limits[motor_id][0] is None or motor_limits[motor_id][1] is None:
                    continue
                delta_motor = motor_limits[motor_id][1] - motor_limits[motor_id][0]
                delta_joint = (
                    self.joint_roms_dict[self.motor_to_joint_dict[motor_id]][1]
                    - self.joint_roms_dict[self.motor_to_joint_dict[motor_id]][0]
                )
                self.joint_to_motor_ratios_dict[motor_id] = float(delta_motor / delta_joint)
                print("Joint calibrated: ", joint)
                calibrated_joints[joint] = 0.0

            self.motor_limits_dict = motor_limits
            self._persist_state()

            if calibrated_joints:
                self.set_joint_pos(calibrated_joints, num_steps=25, step_size=0.001)
            time.sleep(0.1)

        if any("wrist" in step["joints"] for step in calib_sequence):
            self.wrist_calibrated = True

        self.calibrated = self.is_calibrated()
        self._persist_state()
        if calibrated_joints:
            self.set_joint_pos(calibrated_joints, num_steps=25, step_size=0.001)
        self.set_max_current(self.max_current)

    def _compute_wrap_offsets_dict(self):
        motor_pos = self.get_motor_pos()

        lower_limit = np.array([self.motor_limits_dict[motor_id][0] for motor_id in self.motor_ids])
        higher_limit = np.array([self.motor_limits_dict[motor_id][1] for motor_id in self.motor_ids])

        offsets = {}
        for i, motor_id in enumerate(self.motor_ids):
            if lower_limit[i] is None or higher_limit[i] is None:
                offsets[motor_id] = 0.0
                continue

            if motor_pos[i] < lower_limit[i] - 0.25 * np.pi:
                print(f"Motor ID {motor_id} is out of bounds: {lower_limit[i]} < {motor_pos[i]} < {higher_limit[i]}")
                offsets[motor_id] = -2 * np.pi
            elif motor_pos[i] > higher_limit[i] + 0.25 * np.pi:
                print(f"Motor ID {motor_id} is out of bounds: {lower_limit[i]} < {motor_pos[i]} < {higher_limit[i]}")
                offsets[motor_id] = 2 * np.pi
            else:
                offsets[motor_id] = 0.0

        print(f"Offsets: {offsets}")
        self._wrap_offsets_dict = offsets

    def _set_motor_pos(self, desired_pos: Union[dict, np.ndarray, list], rel_to_current: bool = False):
        with self._motor_lock:
            if rel_to_current:
                current_positions = self.get_motor_pos()

            motor_ids_to_write = []
            positions_to_write = []

            if isinstance(desired_pos, dict):
                for motor_id, pos_val in desired_pos.items():
                    if motor_id not in self.motor_ids:
                        print(f"Warning: Motor ID {motor_id} in desired_pos dict is not in self.motor_ids. Skipping.")
                        continue
                    if pos_val is None or math.isnan(pos_val):
                        continue

                    pos_to_write = float(pos_val)
                    if rel_to_current:
                        pos_to_write += current_positions[self.motor_id_to_idx_dict[motor_id]]

                    motor_ids_to_write.append(motor_id)
                    positions_to_write.append(pos_to_write)

                if not motor_ids_to_write:
                    return

                positions_to_write = np.array(positions_to_write, dtype=float)
            elif isinstance(desired_pos, (np.ndarray, list)):
                if len(desired_pos) != len(self.motor_ids):
                    raise ValueError(
                        f"Length of desired_pos (list/ndarray) ({len(desired_pos)}) must match the number of configured motor_ids ({len(self.motor_ids)})."
                    )

                for i, pos_val in enumerate(desired_pos):
                    if pos_val is None or math.isnan(pos_val):
                        continue
                    motor_ids_to_write.append(self.motor_ids[i])
                    if rel_to_current:
                        positions_to_write.append(float(pos_val) + current_positions[i])
                    else:
                        positions_to_write.append(float(pos_val))

                if not motor_ids_to_write:
                    print("\033[93mWarning: All positions in desired_pos (list/array) were None. No motor commands sent.\033[0m")
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
            motor_id = self.motor_ids[idx]
            joint_name = self.motor_to_joint_dict.get(motor_id)
            if any(limit is None for limit in self.motor_limits_dict[motor_id]):
                joint_pos[joint_name] = None
                print(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing motor limits).\033[0m"
                )
            elif self.joint_to_motor_ratios_dict[motor_id] == 0:
                joint_pos[joint_name] = None
                print(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m"
                )
            else:
                wrapped_pos = pos - self._wrap_offsets_dict.get(motor_id, 0.0)
                if self.joint_inversion_dict.get(joint_name, False):
                    joint_pos[joint_name] = self.joint_roms_dict[joint_name][1] - (
                        (wrapped_pos - self.motor_limits_dict[motor_id][0]) / self.joint_to_motor_ratios_dict[motor_id]
                    )
                else:
                    joint_pos[joint_name] = self.joint_roms_dict[joint_name][0] + (
                        (wrapped_pos - self.motor_limits_dict[motor_id][0]) / self.joint_to_motor_ratios_dict[motor_id]
                    )
        return joint_pos

    def _joint_to_motor_pos(self, joint_pos: dict) -> np.ndarray:
        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()

        motor_pos = [None] * len(self.motor_ids)

        for joint_name, pos in joint_pos.items():
            motor_id = self.joint_to_motor_map.get(joint_name)
            if motor_id is None:
                continue

            if pos is None:
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = None
                continue

            if (
                self.motor_limits_dict[motor_id][0] is None
                or self.motor_limits_dict[motor_id][1] is None
                or self.joint_to_motor_ratios_dict[motor_id] == 0
            ):
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = None
                print(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m"
                )
                continue

            min_pos, max_pos = self.joint_roms_dict[joint_name]
            if pos < min_pos or pos > max_pos:
                pos = max(min_pos, min(max_pos, pos))

            if self.joint_inversion_dict.get(joint_name, False):
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = self.motor_limits_dict[motor_id][0] + (
                    self.joint_roms_dict[joint_name][1] - pos
                ) * self.joint_to_motor_ratios_dict[motor_id]
            else:
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = self.motor_limits_dict[motor_id][0] + (
                    pos - self.joint_roms_dict[joint_name][0]
                ) * self.joint_to_motor_ratios_dict[motor_id]

            motor_pos[self.motor_id_to_idx_dict[motor_id]] += self._wrap_offsets_dict.get(motor_id, 0.0)

        return motor_pos

    def _sanity_check(self):
        if len(self.motor_ids) != len(self.joint_ids):
            raise ValueError("Number of motor IDs and joints do not match.")

        if len(self.motor_ids) != len(self.joint_to_motor_map):
            raise ValueError("Number of motor IDs and joints do not match.")

        if self.control_mode not in [
            "current",
            "velocity",
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
            if joint not in self.joint_roms_dict:
                raise ValueError(f"ROM for joint {joint} is not defined.")
            if motor_id not in self.motor_ids:
                raise ValueError(f"Motor ID {motor_id} is not in the motor IDs list.")

        for joint, rom in self.joint_roms_dict.items():
            if rom[1] - rom[0] <= 0:
                raise ValueError(f"ROM for joint {joint} is not valid.")
            if joint not in self.joint_ids:
                raise ValueError(f"Joint {joint} in ROMs is not defined.")

        for step in self.calib_sequence:
            for joint, direction in step["joints"].items():
                if joint not in self.joint_ids:
                    raise ValueError(f"Joint {joint} is not defined.")
                if direction not in ["flex", "extend"]:
                    raise ValueError(f"Invalid direction for joint {joint}.")

        for motor_limit in self.motor_limits_dict.values():
            if any(limit is None for limit in motor_limit):
                self.calibrated = False

    def tension(self, move_motors: bool = False, blocking: bool = True):
        if blocking:
            self._task_stop_event.clear()
            self._tension(move_motors)
        else:
            self._start_task(self._tension, move_motors)

    def jitter(
        self,
        motor_ids: List[int] | None = None,
        amplitude: float = 5.0,
        frequency: float = 10.0,
        duration: float = 3.0,
        include_wrist: bool = False,
        blocking: bool = True,
    ):
        if blocking:
            self._task_stop_event.clear()
            self._jitter(motor_ids, amplitude, frequency, duration, include_wrist)
        else:
            self._start_task(self._jitter, motor_ids, amplitude, frequency, duration, include_wrist)

    def _jitter(
        self,
        motor_ids: List[int] | None = None,
        amplitude: float = 5.0,
        frequency: float = 10.0,
        duration: float = 3.0,
        include_wrist: bool = False,
    ):
        max_amplitude_deg = 10.0
        if amplitude > max_amplitude_deg:
            raise ValueError(f"Amplitude must be <= {max_amplitude_deg} degrees for safety. Got {amplitude}.")

        amplitude_rad = np.deg2rad(amplitude)

        if motor_ids is None:
            wrist_motor_id = self.joint_to_motor_map.get("wrist")
            motor_ids = [mid for mid in self.motor_ids if include_wrist or mid != wrist_motor_id]

        start_positions = self.get_motor_pos(as_dict=True)
        start_pos_array = np.array([start_positions[mid] for mid in motor_ids])

        start_time = time.time()
        while time.time() - start_time < duration and not self._task_stop_event.is_set():
            t = time.time() - start_time
            offset = amplitude_rad * math.sin(2 * math.pi * frequency * t)
            with self._motor_lock:
                self._motor_client.write_desired_pos(motor_ids, start_pos_array + offset)

        with self._motor_lock:
            self._motor_client.write_desired_pos(motor_ids, start_pos_array)

    def _tension(self, move_motors: bool = False):
        self.set_control_mode("current_based_position")
        if move_motors:
            motors_to_move = [
                motor_id
                for joint, motor_id in self.joint_to_motor_map.items()
                if "wrist" not in joint.lower() and motor_id in self.motor_ids
            ]
            self.set_max_current(self.calib_current)

            duration = 8
            increment_per_step = 0.1
            motor_increments_right = {motor_id: increment_per_step for motor_id in motors_to_move}
            motor_increments_left = {motor_id: -increment_per_step for motor_id in motors_to_move}

            start_time = time.time()
            while time.time() - start_time < duration:
                if self._task_stop_event.is_set():
                    break
                self._set_motor_pos(motor_increments_left, rel_to_current=True)
                time.sleep(0.1)

            start_time = time.time()
            while time.time() - start_time < duration:
                if self._task_stop_event.is_set():
                    break
                self._set_motor_pos(motor_increments_right, rel_to_current=True)
                time.sleep(0.1)

        self.set_max_current(self.max_current)
        self.enable_torque()
        print("Holding motors. Please tension carefully. Press Ctrl+C to exit.")
        try:
            while not self._task_stop_event.is_set():
                time.sleep(0.1)
        finally:
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
            target=self._run_task,
            args=(task_fn,) + args,
            kwargs=kwargs,
        )
        self._task_thread.start()

    def stop_task(self):
        if self._task_thread and self._task_thread.is_alive():
            self._task_stop_event.set()
            self._task_thread.join()
            print("Task stopped.")
        else:
            print("No running task to stop.")


def require_connection(func):
    def wrapper(self, *args, **kwargs):
        if not self._motor_client or not self._motor_client.is_connected:
            raise RuntimeError("Hand is not connected.")
        return func(self, *args, **kwargs)

    return wrapper


def require_calibration(func):
    def wrapper(self, *args, **kwargs):
        if not self.calibrated:
            raise RuntimeError("Hand is not calibrated. Please run .calibrate() first.")
        return func(self, *args, **kwargs)

    return wrapper


class MockOrcaHand(OrcaHand):
    """Testing helper that always uses the mock Dynamixel driver."""

    def __init__(self, *args, **kwargs):
        profile = kwargs.get("profile")
        model_path = kwargs.get("model_path")
        if model_path is None and args:
            model_path = args[0]
            args = args[1:]
        if profile is None:
            if model_path is not None:
                profile = load_profile_from_path(model_path)
            else:
                profile = load_profile(kwargs.get("profile_name") or DEFAULT_PROFILE_NAME)

        kwargs["profile"] = replace(
            profile,
            driver=replace(profile.driver, kind="mock_dynamixel"),
        )
        if model_path is not None:
            kwargs["model_path"] = model_path
        kwargs.pop("profile_name", None)
        super().__init__(*args, **kwargs)

    def connect(self, port: str | None = None) -> tuple[bool, str]:
        if port is not None:
            self.port = port
        if self.port is None:
            self.port = "/dev/mock"
        try:
            self._motor_client = self._create_motor_client()
            with self._motor_lock:
                self._motor_client.connect()
            self._persist_state()
            return True, "Mock connection successful"
        except Exception as exc:
            self._motor_client = None
            return False, f"Mock connection failed: {exc}"
