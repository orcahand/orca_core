# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

# # THERE ARE THREE MAIN THREADS:
# 1. task thread: high level, calibration, tensioning, etc.
# 2. control thread: low level, control the motors and pulls targets from command_queu
# 3. state thread: polls the hardware and updates the state dataclass

from errno import ECHILD
import atexit
import os
import time
import math
import threading
from typing import Dict, List, Union, Optional, Callable
from collections import deque
from threading import RLock
import numpy as np
import serial
from .hardware.dynamixel_client import DynamixelClient
from .hardware.mock_dynamixel_client import MockDynamixelClient
from .utils.utils import *
import copy 
from dataclasses import dataclass


@dataclass
class HandState:
    """State data for all motors"""
    motor_pos: np.ndarray = None
    motor_vel: np.ndarray = None
    motor_cur: np.ndarray = None
    motor_temp: np.ndarray = None

    joint_pos: np.ndarray = None 
    joint_vel: np.ndarray = None

    last_updated: float = 0.0

@dataclass
class ControlGoal:
    """Goal data for all motors (Currently only position is supported)"""
    motor_pos: np.ndarray = None
    joint_pos: np.ndarray = None # (only for debugging, never use this for control)


class OrcaHand:
    """OrcaHand class is used to abtract hardware control the hand of the robot with simple high level control methods in joint space."""
   
    def __init__(self, model_path: str = None):
        """Initialize the OrcaHand class.

        Args:
            model_path (str): The path to model_path folder, which includes the config.yaml and calibration.yaml 
        """
        self.model_path = get_model_path(model_path)
                
        # Load configurations from the YAML files
        self.config_path = os.path.join(self.model_path, "config.yaml")
        self.calib_path = os.path.join(self.model_path, "calibration.yaml")
        
        config = read_yaml(self.config_path)
        calib = read_yaml(self.calib_path)
            
        self.baudrate: int = config.get('baudrate', 3000000)
        self.port: str = config.get('port')
        if self.port is None:
            self.port = get_and_choose_dynamixel_port()
        update_yaml(self.config_path, 'port', self.port)

        self.max_current: int = config.get('max_current', 300)
        self.control_mode: str = config.get('control_mode', 'current_position')
        self.type: str = config.get('type', None) # left or right
        
        self.calib_current: str = config.get('calib_current', 200)
        self.wrist_calib_current: str = config.get('wrist_calib_current', 100)
        self.calib_step_size: float = config.get('calib_step_size', 0.1)
        self.calib_step_period: float = config.get('calib_step_period', 0.01)
        self.calib_threshold: float = config.get('calib_threshold', 0.01)
        self.calib_num_stable: int = config.get('calib_num_stable', 20)
        self.calib_sequence: Dict[str, Dict[str, str]] = config.get('calib_sequence', [])
        self._calibrated: bool = calib.get('calibrated', False)
     
        self.neutral_position: Dict[str, float] = config.get('neutral_position', {})
        
        self.motor_ids: List[int] = config.get('motor_ids', [])
        self.joint_ids: List[str] = config.get('joint_ids', [])

   
        motor_limits_from_calib_dict = calib.get('motor_limits', {})
        self.motor_limits_dict: Dict[int, List[float]] = {
            motor_id: motor_limits_from_calib_dict.get(motor_id, [None, None]) for motor_id in self.motor_ids}

        joint_to_motor_ratios_from_calib_dict = calib.get('joint_to_motor_ratios', {})
        self.joint_to_motor_ratios_dict: Dict[int, float] = {
            motor_id: joint_to_motor_ratios_from_calib_dict.get(motor_id, 0.0) for motor_id in self.motor_ids}
            
        self.joint_to_motor_map: Dict[str, float] = config.get('joint_to_motor_map', {})
        self.joint_roms_dict: Dict[str, List[float]] = config.get('joint_roms', {})
        
        self.joint_inversion_dict = {}
        for joint, motor_id in self.joint_to_motor_map.items():
            if motor_id < 0 or math.copysign(1, motor_id) < 0:
                self.joint_inversion_dict[joint] = True
                self.joint_to_motor_map[joint] = int(abs(motor_id))
            else:
                self.joint_inversion_dict[joint] = False
        
        # Helper dicts for mapping between motor and joint ids and indices
        self.joint_id_to_motor_id_dict = {k: int(v) for k, v in self.joint_to_motor_map.items()}
        self.motor_id_to_joint_id_dict = {v: k for k, v in self.joint_to_motor_map.items()}
        self.motor_id_to_idx_dict: Dict[int, int] = {motor_id: i for i, motor_id in enumerate(self.motor_ids)}
        self.joint_id_to_idx_dict: Dict[str, int] = {joint_id: i for i, joint_id in enumerate(self.joint_ids)}
        self.joint_id_to_motor_idx_dict: Dict[str, int] = {joint_id: self.motor_id_to_idx_dict[motor_id] for joint_id, motor_id in self.joint_to_motor_map.items()}
        self.motor_id_to_joint_idx_dict: Dict[int, int] = {motor_id: self.joint_id_to_idx_dict[joint_id] for joint_id, motor_id in self.joint_to_motor_map.items()}

        self._wrap_offsets_dict: Dict[int, float] = None

        self._dxl_client: DynamixelClient = None
        self._motor_lock: RLock = RLock()

        self._sanity_check()       
        self.check_calibrated(verbose=True)

        self.control_rate_hz = config.get('control_rate_hz', 1)
        self.state_rate_hz = config.get('state_rate_hz', 30)

        # Task thread to start and stop longer tasks like tensioning, calibration, etc. externally
        self._task_thread: threading.Thread = None
        self._task_stop_event = threading.Event()
        self._task_lock: RLock = RLock() 
        self._current_task = None

        # State thread to sync the state from the hardware
        self._hand_state_thread: threading.Thread = None
        self._hand_state_stop_event = threading.Event()
        self._hand_state_lock: RLock = RLock()
        self._hand_state = HandState()

        # Control thread to read current and goal and write to the hardware
        self._control_thread: threading.Thread = None
        self._control_stop_event = threading.Event()
        self._control_lock: RLock = RLock()
        self._control_goal = ControlGoal()
        self._control_goal_queue = deque(maxlen=1)

        # Add missing lock
        self._lock = RLock()

        atexit.register(self.disconnect)


    @staticmethod
    def add_two_numbers(a: int, b: int) -> int:
        return a + b

    @property
    def is_connected(self) -> bool:
        """Check if the hand is connected.

        Returns:
            bool: True if connected, False otherwise.
        """
        return self._dxl_client.is_connected if self._dxl_client else False

    @property
    def is_calibrated(self) -> bool:
        """Check if the hand is calibrated.

        Returns:
            bool: True if calibrated, False otherwise.
        """
        return self._calibrated

    @property
    def hand_state(self) -> HandState:
        """Get the current hand state.

        Returns:
            HandState: The current hand state.
        """
        with self._hand_state_lock:
            return self._hand_state
    
    @property
    def control_goal(self) -> ControlGoal:
        """Get the current control goal.

        Returns:
            ControlGoal: The current control goal.
        """
        with self._motor_lock:
            return self._control_goal    

    def connect(self) -> tuple[bool, str]:
        """Connect to the hardware, initialize the hand and start state and control threads.

        Returns:
            tuple[bool, str]: (Success status, message).
        """
        try:
            self._dxl_client = DynamixelClient(self.motor_ids, self.port, self.baudrate)
            with self._motor_lock:
                self._dxl_client.connect()
            
            self._start_threads()
            time.sleep(0.1)

            self.enable_torque()
            self.set_control_mode(self.control_mode)
            self.set_max_current(self.max_current)
            self._compute_wrap_offsets_dict()

            return True, "Connection successful"
        except serial.SerialException as e:
            self._dxl_client = None
            self.port = get_and_choose_dynamixel_port()
            update_yaml(self.config_path, 'port', self.port)
            return self.connect()
        except Exception as e:
            raise e
            self._dxl_client = None
            return False, f"Connection failed: {str(e)}"
        
    def disconnect(self) -> tuple[bool, str]:
        """Disconnect from the hand.

        Returns:
            tuple[bool, str]: (Success status, message).
        """
        self._stop_threads()

        try:
            with self._motor_lock:
                self.disable_torque()
                self._dxl_client.disconnect()
            return True, "Disconnected successfully"
        except Exception as e:
            return False, f"Disconnection failed: {str(e)}"

    def enable_torque(self, motor_ids: List[int] = None):
        """Enable torque for the motors.
        
        Args:
            motor_ids (list): List of motor IDs to enable the torque. If None, all motors connected will be enabled
        """
        if motor_ids is None:
            motor_ids = self.motor_ids
        with self._motor_lock:
            self._dxl_client.set_torque_enabled(motor_ids, True)        

    def disable_torque(self, motor_ids: List[int] = None):
        """Disable torque for the motors.
        
        Args:
            motor_ids (list): List of motor IDs to disable the torque. If None, all motors will be disabled.
        """
        if motor_ids is None:
            motor_ids = self.motor_ids
        with self._motor_lock:
            self._dxl_client.set_torque_enabled(motor_ids, False)

    def set_control_mode(self, mode: str, motor_ids: List[int] = None):
        """Set the control mode for the motors.
        
        Args:
            mode (str): Control mode.
                (0) current: Current control mode,
                (1) velocity: Velocity control mode,
                (3) position: Position control mode,
                (4) multi_turn_position: Multi-turn position control mode,
                (5) current_based_position: Current-based position control mode.
            motor_ids (list): List of motor IDs to set the control mode. If None, all motors will be set.
        """
        
        mode_map = {
            'current': 0,
            'velocity': 1,
            'position': 3,
            'multi_turn_position': 4,
            'current_based_position': 5
        }

        mode = mode_map.get(mode)
        if mode is None:
            raise ValueError("Invalid control mode.")
        
        with self._motor_lock:
            if motor_ids is None:
                motor_ids = self.motor_ids
            else:
                if not all(motor_id in self.motor_ids for motor_id in motor_ids):
                    raise ValueError("Invalid motor IDs.")
            self._dxl_client.set_operating_mode(motor_ids, mode)

    def _start_threads(self):
        """Start state and control threads"""

        if not self._hand_state_thread or not self._hand_state_thread.is_alive():
            self._hand_state_stop_event.clear()
            self._hand_state_thread = threading.Thread(
                target=self._hand_state_loop, 
                daemon=True,  
                name="HandStateThread"
            )
            self._hand_state_thread.start()
        
        # # Start control thread
        # if not self._control_thread or not self._control_thread.is_alive():
        #     self._control_stop_event.clear()
        #     self._control_thread = threading.Thread(
        #         target=self._control_loop, 
        #         daemon=True,  
        #         name="ControlThread"
        #     )
        #     self._control_thread.start()
    
    def _stop_threads(self):
        """Stop state and control threads"""
        # Signal threads to stop
        self._hand_state_stop_event.set()
        self._control_stop_event.set()
        
        # Wait for threads to finish (with timeout)
        if self._hand_state_thread and self._hand_state_thread.is_alive():
            self._hand_state_thread.join(timeout=1.0)  # Wait max 2 seconds
            if self._hand_state_thread.is_alive():
                print("WARNING: Hand state thread did not stop gracefully")
        
        # Only join control thread if it was started
        if (hasattr(self, '_control_thread') and 
            self._control_thread and 
            self._control_thread.is_alive()):
            self._control_thread.join(timeout=2.0)
            if self._control_thread.is_alive():
                print("WARNING: Control thread did not stop gracefully")

    def _hand_state_loop(self):
        """Background thread for syncing hand state from hardware"""
        while not self._hand_state_stop_event.is_set():
            start_time = time.time()

            try:
                with self._motor_lock:
                    motor_pos, motor_vel, motor_cur = self._dxl_client.read_pos_vel_cur()
                    motor_temp = self._dxl_client.read_temperature()

                with self._hand_state_lock:
                    self._hand_state.motor_pos = motor_pos
                    self._hand_state.motor_vel = motor_vel
                    self._hand_state.motor_cur = motor_cur
                    self._hand_state.motor_temp = motor_temp
                    self._hand_state.last_updated = time.time()

                    if self._calibrated:
                        self._hand_state.joint_pos = self._motor_to_joint_pos(self._hand_state.motor_pos)
                    else:
                        self._hand_state.joint_pos = None
                        self._hand_state.joint_vel = None
                        self._hand_state.joint_temp = None

                try:
                    goal = self._control_goal_queue.popleft()

                    assert len(goal.motor_pos) == len(self.motor_ids)

                    with self._motor_lock:
                        self._dxl_client.write_desired_pos(self.motor_ids, goal.motor_pos)

                except IndexError:
                    continue
                
                time_to_sleep = 1/self.state_rate_hz - (time.time() - start_time)

                if time_to_sleep > 0:
                    time.sleep(time_to_sleep)
                

            except Exception as e:
                raise e
                print(f"Error in hand state thread: {e}")
                time.sleep(0.1) 

    def _control_loop(self):
        """Background thread for control commands"""
        while not self._control_stop_event.is_set():

            start_time = time.time()

            try:
                goal = self._control_goal_queue.popleft()
                self._control_goal = goal

                assert len(self._control_goal.motor_pos) == len(self.motor_ids)
                with self._motor_lock:
                    self._dxl_client.write_desired_pos(self.motor_ids, self._control_goal.motor_pos)

                time_to_sleep = 1/self.control_rate_hz - (time.time() - start_time)
                if time_to_sleep > 0:
                    time.sleep(time_to_sleep)

            except IndexError:
                continue
            except Exception as e:
                print(f"Error in control thread: {e}")
                time.sleep(0.1)         
   
    def set_joint_pos(self, joint_pos: Union[dict, list, np.ndarray]):
        """Set the desired joint positions.
    
        Args:
            joint_pos (dict or list): If dict, it should be {joint_name: desired_position}.
                                    If list, it should contain positions in the order of joint_ids.
                                    If np.ndarray, it should contain positions in the order of joint_ids.
        """
        if isinstance(joint_pos, dict):
            pos = np.full(len(self.joint_ids), None)
            for joint, pos_val in joint_pos.items():
                if joint in self.joint_ids:
                    pos[self.joint_id_to_idx_dict[joint]] = pos_val
            
            motor_pos = self._joint_to_motor_pos(pos)
            goal = ControlGoal(motor_pos=motor_pos)
            self._control_goal_queue.append(goal)
        else:
            NotImplementedError("Setting joint positions as list is not supported yet.")

    def _set_motor_pos(self, motor_pos: np.ndarray, rel_to_current: bool = False):
        """Set the desired motor positions and check that they are within the limits.
        
        Args:
            motor_pos (np.ndarray): Desired motor positions.
            rel_to_current (bool): If True, the desired position is relative to the current position.
        """
        if rel_to_current:
            motor_pos = motor_pos + self.hand_state.motor_pos
            
        goal = ControlGoal(motor_pos=motor_pos)
        self._control_goal_queue.append(goal)

    def check_calibrated(self, verbose: bool = False) -> bool:
        """Check if the hand is calibrated .

        Args:
            verbose (bool): If True, print detailed calibration status for uncalibrated joints.

        Returns:
            bool: True if all joints are calibrated, False otherwise.
        """
        overall_calibrated = True
        uncalibrated_messages = []
        motors_with_warnings = set()

        for motor_id, limits in self.motor_limits_dict.items():
            if any(limit is None for limit in limits):
                overall_calibrated = False
                if not verbose:
                    return False 
                joint_name = self.motor_id_to_joint_id_dict.get(motor_id, "Unknown")
                uncalibrated_messages.append(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing motor limits).\033[0m")
                motors_with_warnings.add(motor_id)


        for motor_id, ratio in self.joint_to_motor_ratios_dict.items():
            if ratio is None or ratio == 0.0:
                overall_calibrated = False
                if not verbose:
                    return False
                if motor_id not in motors_with_warnings:
                    joint_name = self.motor_id_to_joint_id_dict.get(motor_id, "Unknown")
                    uncalibrated_messages.append(
                        f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m"
                    )
                    motors_with_warnings.add(motor_id)
        
        if verbose:
            for msg in uncalibrated_messages:
                print(msg)

        self._calibrated = overall_calibrated
        
        return overall_calibrated

    def calibrate(self, blocking: bool = True):
        if blocking:
            self._calibrate()
        else:
            self._start_task(self._calibrate)

    #### CHANGE LATER TO GENERAL UPDATE CONFIG METHODS ####

    def set_max_current(self, current: Union[float, np.ndarray]):
        """Set the maximum current for the motors.
        
        Args:
            current (int or np.ndarray): If np.ndarray, it should be the maximum current for each motor, otherwise it will be the same for all motors.
        """
        if isinstance(current, np.ndarray):
            if len(current) != len(self.motor_ids):
                raise ValueError("Number of currents do not match the number of motors.")
            with self._motor_lock:
                self._dxl_client.write_desired_current(self.motor_ids, current)
        elif isinstance(current, int):
            with self._motor_lock:
                self._dxl_client.write_desired_current(self.motor_ids, current*np.ones(len(self.motor_ids)))
        else:
            raise ValueError("Current must be an int or np.ndarray, not " + type(current))
     
    #### CHANGE LATER TO GENERAL UPDATE CONFIG METHODS ####

    def _calibrate(self):
            
        motor_limits = self.motor_limits_dict.copy()

        self._compute_wrap_offsets_dict()

        for step in self.calib_sequence:
            for joint in step["joints"].keys():
                motor_id = self.joint_id_to_motor_id_dict[joint]
                motor_limits[motor_id] = [None, None]
                self._wrap_offsets_dict[motor_id] = 0.0

        self.set_control_mode('current_based_position')
        currents = np.full(len(self.motor_ids), self.calib_current)
        currents[self.joint_id_to_motor_idx_dict['wrist']] = self.wrist_calib_current

        self.set_max_current(currents) 
        self.enable_torque()
        
        for step in self.calib_sequence:
            if self._task_stop_event.is_set():
                return

            motor_reached_limit, directions, position_buffers, motor_reached_limit, calibrated_joints, position_logs, current_log = {}, {}, {}, {}, {}, {}, {}
            desired_increment = np.zeros(len(self.motor_ids))

            for joint, direction in step["joints"].items(): 
                if self._task_stop_event.is_set():
                    return
                    
                motor_id = self.joint_id_to_motor_id_dict[joint]
                sign = 1 if direction == 'flex' else -1
                if self.joint_inversion_dict.get(joint, False):
                    sign = -sign
                directions[motor_id] = sign
                position_buffers[motor_id] = deque(maxlen=self.calib_num_stable)
                position_logs[motor_id] = []
                current_log[motor_id] = []
                motor_reached_limit[motor_id] = False
            
            while(not all(motor_reached_limit.values()) and not self._task_stop_event.is_set()):    
                moving_motors = []
                for motor_id, reached_limit in motor_reached_limit.items():
                    if not reached_limit:
                        desired_increment[self.motor_id_to_idx_dict[motor_id]] = directions[motor_id] * self.calib_step_size
                        moving_motors.append(motor_id)

                self._set_motor_pos(desired_increment, rel_to_current=True)
                time.sleep(self.calib_step_period)

                for motor_id in moving_motors:
                    if not motor_reached_limit[motor_id]:
                        curr_state = self.hand_state
                        motor_idx = self.motor_id_to_idx_dict[motor_id]
                        position_buffers[motor_id].append(float(curr_state.motor_pos[motor_idx]))
                        position_logs[motor_id].append(float(curr_state.motor_pos[motor_idx]))
                        current_log[motor_id].append(float(curr_state.motor_cur[motor_idx]))

                        if len(position_buffers[motor_id]) == self.calib_num_stable and np.allclose(position_buffers[motor_id], position_buffers[motor_id][0], atol=self.calib_threshold):
                            motor_reached_limit[motor_id] = True
                            if 'wrist' in joint or 'abd' in joint:
                                avg_limit = float(np.mean(position_buffers[motor_id]))
                            else:
                                self.set_max_current(50)
                                time.sleep(0.05)
                                curr_state = self.hand_state
                                avg_limit = float(curr_state.motor_pos[motor_idx])
                            print(f"Motor {motor_id} corresponding to joint {self.motor_id_to_joint_id_dict[motor_id]} reached the limit at {avg_limit} rad.")
                            if directions[motor_id] == 1:
                                motor_limits[motor_id][1] = avg_limit
                            if directions[motor_id] == -1:
                                motor_limits[motor_id][0] = avg_limit
                
            # find ratios of all motors that have been calibrated in this step
            for joint, direction in step["joints"].items(): 
                motor_id = self.joint_id_to_motor_id_dict[joint]
                if motor_limits[motor_id][0] is None or motor_limits[motor_id][1] is None:
                    continue
                delta_motor = motor_limits[motor_id][1] - motor_limits[motor_id][0]
                delta_joint = self.joint_roms_dict[self.motor_id_to_joint_id_dict[motor_id]][1] - self.joint_roms_dict[self.motor_id_to_joint_id_dict[motor_id]][0]
                self.joint_to_motor_ratios_dict[motor_id] = float(delta_motor / delta_joint) 
                print("Joint calibrated: ", joint)
                calibrated_joints[joint] = 0.0
  
            update_yaml(self.calib_path, 'joint_to_motor_ratios', self.joint_to_motor_ratios_dict)
            update_yaml(self.calib_path, 'motor_limits', motor_limits)
            self.motor_limits_dict = motor_limits
            if calibrated_joints:
                self.set_joint_pos(calibrated_joints)
                print(f"Calibrated joints: {calibrated_joints}")
            time.sleep(1)    
            
        self._calibrated = self.check_calibrated()
        update_yaml(self.calib_path, 'calibrated', self._calibrated)
        self.set_joint_pos(calibrated_joints)
        self.set_max_current(self.max_current)

    def _compute_wrap_offsets_dict(self):
        """
        Wrap offsets are corrections applied to motor positions to handle the circular nature of Dynamixel motor encoders.
        
        How it works: 
            - Each motor has a range of positions it can measure, typically 0-4095 units.
            - When you exceed 4095 or go below 0, the motor wraps around to the other end of the range.
            - We need to account for this so that when you move the motor, it doesn't wrap around
            - This is done by adding or subtracting 2*pi to the motor position when it exceeds the limits.
            - This is done by adding or subtracting 2*pi to the motor position when it exceeds the limits.  
        """

        curr_state = self.hand_state
        
        motor_pos = curr_state.motor_pos

        lower_limit = np.array([self.motor_limits_dict[motor_id][0] for motor_id in self.motor_ids])
        higher_limit = np.array([self.motor_limits_dict[motor_id][1] for motor_id in self.motor_ids])

        offsets = {}
        for i, motor_id in enumerate(self.motor_ids):
            if lower_limit[i] is None or higher_limit[i] is None:
                offsets[motor_id] = 0.0
                continue

            if motor_pos[i] < lower_limit[i] - 0.25 * np.pi: # Some buffer to compensate for noise/slack differences
                print(f"Motor ID {motor_id} is out of bounds: "
                    f"{lower_limit[i]} < {motor_pos[i]} < {higher_limit[i]}")
                offsets[motor_id] = -2 * np.pi

            elif motor_pos[i] > higher_limit[i] + 0.25 * np.pi: # Some buffer to compensate for noise/slack differences
                print(f"Motor ID {motor_id} is out of bounds: "
                    f"{lower_limit[i]} < {motor_pos[i]} < {higher_limit[i]}")
                offsets[motor_id] = +2 * np.pi

            else:
                offsets[motor_id] = 0.0

        self._wrap_offsets_dict = offsets

    def _motor_to_joint_pos(self, motor_pos: np.ndarray) -> np.ndarray:
        """Convert motor positions into joint positions.
        
        Args:
            motor_pos (np.ndarray): Motor positions.
        
        Returns:
            np.ndarray: Joint positions.
        """
        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()
    

        joint_pos = np.zeros(len(self.joint_ids))
        for idx, pos in enumerate(motor_pos):
            motor_id = self.motor_ids[idx]
            joint_name = self.motor_id_to_joint_id_dict.get(motor_id)
            if any(limit is None for limit in self.motor_limits_dict[motor_id]):
                joint_pos[idx] = None
                print(f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m")
            elif self.joint_to_motor_ratios_dict[motor_id] == 0:
                joint_pos[idx] = None
                print(f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m")
            else:
                wrapped_pos = pos - self._wrap_offsets_dict.get(motor_id, 0.0)
                
                if self.joint_inversion_dict.get(joint_name, False):
                    joint_pos[idx] = self.joint_roms_dict[joint_name][1] - (wrapped_pos - self.motor_limits_dict[motor_id][0]) / self.joint_to_motor_ratios_dict[motor_id]
                else:
                    joint_pos[idx] = self.joint_roms_dict[joint_name][0] + (wrapped_pos - self.motor_limits_dict[motor_id][0]) / self.joint_to_motor_ratios_dict[motor_id]
        
        return joint_pos
    
    def _joint_to_motor_pos(self, joint_pos: np.ndarray) -> np.ndarray:
        """Convert desired joint positions into motor commands. Additionally, clips the motor positions to the motor limits.
        Args:
            joint_pos (np.ndarray): Desired joint positions. If None, the motor position will not be updated.

        Returns:
            np.ndarray: Motor positions. This is always a list of length len(self.motor_ids).
        """

        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()

        with self._hand_state_lock:
            motor_pos = self._hand_state.motor_pos.copy()
                
        for idx, pos in enumerate(joint_pos):
            motor_id = self.joint_id_to_motor_id_dict[self.joint_ids[idx]]
            joint_name = self.motor_id_to_joint_id_dict.get(motor_id)

            if pos is None:
                continue

            if self.motor_limits_dict[motor_id][0] is None or self.motor_limits_dict[motor_id][1] is None or self.joint_to_motor_ratios_dict[motor_id] == 0:
                print(f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m")
                continue
            
            min_pos, max_pos = self.joint_roms_dict[joint_name]
            
            if pos < min_pos or pos > max_pos:
                pos = np.clip(pos, min_pos, max_pos)
                print(f"\033[93mWarning: Joint {joint_name} is out of bounds: {min_pos} < {pos} < {max_pos}.\033[0m")

            if self.joint_inversion_dict.get(joint_name, False):
                # Inverted: higher ROM value corresponds to lower motor position.
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = self.motor_limits_dict[motor_id][0] + (self.joint_roms_dict[joint_name][1] - pos) * self.joint_to_motor_ratios_dict[motor_id]
            else:
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = self.motor_limits_dict[motor_id][0] + (pos - self.joint_roms_dict[joint_name][0]) * self.joint_to_motor_ratios_dict[motor_id]  
            
            motor_pos[self.motor_id_to_idx_dict[motor_id]] += self._wrap_offsets_dict.get(motor_id, 0.0)

        motor_pos = np.clip(motor_pos, self.motor_limits_dict[motor_id][0], self.motor_limits_dict[motor_id][1])
            
        return motor_pos
    
    def _sanity_check(self):
        """Check if the configuration is correct and the IDs are consistent."""
        if len(self.motor_ids) != len(self.joint_ids):
            raise ValueError("Number of motor IDs and joints do not match.")
        
        if len(self.motor_ids) != len(self.joint_to_motor_map):
            raise ValueError("Number of motor IDs and joints do not match.")
        
        if self.control_mode not in ['current_position', 'current_velocity', 'position', 'multi_turn_position', 'current_based_position']:
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
                if direction not in ['flex', 'extend']:
                    raise ValueError(f"Invalid direction for joint {joint}.")
          
        
        for motor_limit in self.motor_limits_dict.values():
            if any(limit is None for limit in motor_limit):
                self._calibrated = False
                update_yaml(self.calib_path, 'calibrated', self._calibrated)

    def tension(self, move_motors: bool = False, blocking: bool = True):
        if blocking:
            self._tension(move_motors)
        else:
            self._start_task(self._tension, move_motors)

    def _tension(self, move_motors: bool = False):
        """Freeze the motors, so that the hand can be manually tensioned.
        
        Args:
            move_motors (bool): If True, the hand will move to all motors positively for 3 seconds to set some initial tension.
        """
        self.set_control_mode('current_based_position')
        if move_motors:
            self.set_max_current(self.calib_current)

            duration = 3
            increment_per_step = 0.08
            motor_increments = np.ones(len(self.motor_ids)) * increment_per_step
            motor_increments[self.joint_id_to_motor_idx_dict['wrist']] = 0.0

            start_time = time.time()
            while(time.time() - start_time < duration):
                if self._task_stop_event.is_set():
                    break
                self._set_motor_pos(motor_increments, rel_to_current=True)
                time.sleep(0.01)

        self.set_max_current(self.max_current)
      
        print("Holding motors. Please tension carefully. Press Ctrl+C to exit.")

        try:
            while not self._task_stop_event.is_set():
                time.sleep(0.1) 
        finally:
            self.disable_torque() 

    def record_waypoints(self, buffer: list, on_finish: Optional[Callable] = None, input_fn: Optional[Callable] = input, blocking: bool = True):
        """
        Start interactive waypoint recording in a thread-safe, interruptible way.
        Args:
            buffer (list): List to append captured waypoints (joint positions as list).
            on_capture (callable): Optional callback called after each capture (passed the new waypoint).
            on_finish (callable): Optional callback called when recording finishes or is stopped.
            blocking (bool): If True, run in the current thread. If False, run in a background thread.
        """
        if blocking:
            self._record_waypoints(buffer, on_finish, input_fn)
        else:
            self._start_task(self._record_waypoints, buffer, on_finish, input_fn)

    def _record_waypoints(self, buffer, on_finish, input_fn):
        try:
            print("Press Enter to capture a waypoint. Ctrl+C or stop_task() to quit.")
            while not self._task_stop_event.is_set():
                input_fn("Press Enter to capture current joint positions...")
                if self._task_stop_event.is_set():
                    break
                current_angles = self.hand_state.joint_pos
                buffer.append([float(angle) for angle in current_angles])
                print(f"Captured waypoint: {current_angles}")
        finally:
            if on_finish:
                on_finish()

    def replay_waypoints(self, waypoints: list, duration: float = 0.8, step_time: float = 0.02, max_iterations: int = 1000, mode: str = "linear", on_finish: Optional[Callable] = None, blocking: bool = True):
        """ Replay a sequence of waypoints by interpolating between them.

        Args:
            waypoints (list): List of joint position lists or dicts.
            duration (float): Total duration to interpolate between waypoints (seconds).
            step_time (float): Time per interpolation step (seconds).
            max_iterations (int): Maximum number of waypoint playback iterations.
            mode (str): Interpolation mode, e.g., "linear", "ease_in_out", etc.
            on_finish (callable): Optional callback called when playback finishes or is stopped.
            blocking (bool): If True, run in the current thread. If False, run in a background thread.
        """
        if blocking:
            self._replay_waypoints(waypoints, duration, step_time, max_iterations, mode, on_finish)
        else:
            self._start_task(self._replay_waypoints, waypoints, duration, step_time, max_iterations, mode, on_finish)

    def _replay_waypoints(self, waypoints: list, duration: float, step_time: float, max_iterations: int, mode: str, on_finish: Optional[Callable] = None):
        iteration_count = 0
        try:
            while not self._task_stop_event.is_set():
                if iteration_count >= max_iterations:
                    break
                for i, start in enumerate(waypoints):
                    print(f"Reached waypoint {i + 1}/{len(waypoints)}: {start}")
                    if self._task_stop_event.is_set():
                        break
                    end = waypoints[(i + 1) % len(waypoints)]
                    for position in interpolate_waypoints(start, end, duration, step_time, mode):
                        if self._task_stop_event.is_set():
                            break
                        self.set_joint_pos(position)
                        time.sleep(step_time)
                iteration_count += 1
        finally:
            if on_finish:
                on_finish()

    def _run_task(self, task_fn, *args, **kwargs):
        """Run a task in a separate thread, so that it can be stopped externally.
        
        Args:
            task_fn (function): The task function to run.
            *args: Additional arguments to pass to the task function.
            **kwargs: Additional keyword arguments to pass to the task function.
        """
        with self._lock:
            self._task_stop_event.clear()
            self._current_task = task_fn.__name__
            try:
                task_fn(*args, **kwargs)
            finally:
                self._current_task = None

    def _start_task(self, task_fn, *args, **kwargs):
        """Start a task in a separate thread, so that it can be stopped externally.
        
        Args:
            task_fn (function): The task function to run.
            *args: Additional arguments to pass to the task function.
            **kwargs: Additional keyword arguments to pass to the task function.
        """
        if self._task_thread and self._task_thread.is_alive():
            print(f"Task '{self._current_task}' is already running.")
            return

        self._task_thread = threading.Thread(target=self._run_task, args=(task_fn,) + args, kwargs=kwargs)
        self._task_thread.start()

    def stop_task(self):
        """Stop the currently running task.
        """
        if self._task_thread and self._task_thread.is_alive():
            self._task_stop_event.set()
            self._task_thread.join()
            print("Task stopped.")
        else:
            print("No running task to stop.")               


def require_connection(func):
    def wrapper(self, *args, **kwargs):
        if not self._dxl_client.is_connected():
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
    """MockOrcaHand class is used to simulate the OrcaHand class for testing."""
   
    def connect(self) -> tuple[bool, str]:
        """Connects to the mock Dynamixel client.

        Returns:
            tuple[bool, str]: A tuple containing a boolean indicating success or failure, 
                              and a string message.
        """
        try:
            self._dxl_client = MockDynamixelClient(self.motor_ids, self.port, self.baudrate)
            with self._motor_lock:
                self._dxl_client.connect()
            
            self._start_threads()
            time.sleep(0.1)

            self.enable_torque()
            self.set_control_mode(self.control_mode)
            self.set_max_current(self.max_current)
            self._compute_wrap_offsets_dict()

            return True, "Mock connection successful"
        except Exception as e:
            self._dxl_client = None
            return False, f"Mock connection failed: {str(e)}"
    
if __name__ == "__main__":
    # Example usage:
    hand = OrcaHand("/Users/ccc/dev/orca/orca_configs/orcahand_v1_right_clemens_stanford")
    status = hand.connect()

    if not status[0]:
        print(status[1])
        exit()

    start_time = time.time()

    hand.calibrate()
    time.sleep(1)
    hand.disconnect()

