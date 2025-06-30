# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import os
import time
import math
import threading
from typing import Dict, List, Union
from collections import deque
from threading import RLock
import numpy as np
from .hardware.dynamixel_client import DynamixelClient
from .hardware.mock_dynamixel_client import MockDynamixelClient
from .utils.yaml_utils import read_yaml, update_yaml # Changed from import *
from .utils.load_utils import get_model_path

class OrcaHand:
    """OrcaHand class is used to abtract hardware control the hand of the robot with simple high level control methods in joint space."""
   
    def __init__(self, model_path: str = None):
        """Initialize the OrcaHand class.

        Args:
            model_path (str): The path to model_path folder, which includes the config.yaml and calibration.yaml 
        """
        # Find the model directory if not provided
        self.model_path = get_model_path(model_path)
                
        # Load configurations from the YAML files
        self.config_path = os.path.join(self.model_path, "config.yaml")
        self.calib_path = os.path.join(self.model_path, "calibration.yaml")
        
        config = read_yaml(self.config_path)
        calib = read_yaml(self.calib_path)
            
        self.baudrate: int = config.get('baudrate', 3000000)
        self.port: str = config.get('port', '/dev/ttyUSB0')
        self.max_current: int = config.get('max_current', 300)
        self.control_mode: str = config.get('control_mode', 'current_position')
        self.type: str = config.get('type', None)
        
        self.calib_current: str = config.get('calib_current', 200)
        self.wrist_calib_current: str = config.get('wrist_calib_current', 100)
        self.calib_step_size: float = config.get('calib_step_size', 0.1)
        self.calib_step_period: float = config.get('calib_step_period', 0.01)
        self.calib_threshold: float = config.get('calib_threshold', 0.01)
        self.calib_num_stable: int = config.get('calib_num_stable', 20)
        self.calib_sequence: Dict[str, Dict[str, str]] = config.get('calib_sequence', [])
        self.calibrated: bool = calib.get('calibrated', False)
     
        self.neutral_position: Dict[str, float] = config.get('neutral_position', {})
        
        self.motor_ids: List[int] = config.get('motor_ids', [])
        self.joint_ids: List[str] = config.get('joint_ids', [])
        self.motor_id_to_idx_dict: Dict[int, int] = {motor_id: i for i, motor_id in enumerate(self.motor_ids)}

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
        
        self.joint_to_motor_map = {k: int(v) for k, v in self.joint_to_motor_map.items()} # This is to make IDs integers

        self.motor_to_joint_dict: Dict[int, str] = {v: k for k, v in self.joint_to_motor_map.items()}

        self._wrap_offsets_dict: Dict[int, float] = None

        self._dxl_client: DynamixelClient = None
        self._motor_lock: RLock = RLock()
        
        self._sanity_check()       
        self.is_calibrated(verbose=True)

    def __del__(self):
        """Destructor to disconnect from the hand."""
        self.disconnect()
        
    def connect(self) -> tuple[bool, str]:
        """Connect to the hand with the DynamixelClient.

        Returns:
            tuple[bool, str]: (Success status, message).
        """
        try:
            self._dxl_client = DynamixelClient(self.motor_ids, self.port, self.baudrate)
            with self._motor_lock:
                self._dxl_client.connect()
            return True, "Connection successful"
        except Exception as e:
            self._dxl_client = None
            return False, f"Connection failed: {str(e)}"
        
    def disconnect(self) -> tuple[bool, str]:
        """Disconnect from the hand.

        Returns:
            tuple[bool, str]: (Success status, message).
        """
        try:
            with self._motor_lock:
                self.disable_torque()
                time.sleep(0.1)
                self._dxl_client.disconnect()
            return True, "Disconnected successfully"
        except Exception as e:
            return False, f"Disconnection failed: {str(e)}"
        
    def is_connected(self) -> bool:
        """Check if the hand is connected.

        Returns:
            bool: True if connected, False otherwise.
        """
        return self._dxl_client.is_connected if self._dxl_client else False
        
    def enable_torque(self, motor_ids: List[int] = None):
        """Enable torque for the motors.
        
        Args:
            motor_ids (list): List of motor IDs to enable the torque. If None, all motors will be enabled
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
    
    def set_max_current(self, current: Union[float, List[float]]):
        """Set the maximum current for the motors.
        
        Args:
            current (int or list): If list, it should be the maximum current for each motor, otherwise it will be the same for all motors.
        """
        if isinstance(current, list):
            if len(current) != len(self.motor_ids):
                raise ValueError("Number of currents do not match the number of motors.")
            with self._motor_lock:
                self._dxl_client.write_desired_current(self.motor_ids, current)
        else:
            with self._motor_lock:
                self._dxl_client.write_desired_current(self.motor_ids, current*np.ones(len(self.motor_ids)))
        
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
            
    def get_motor_pos(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Get the current motor positions in radians (Note that this includes offsets of the motors).
        
        Args:
            as_dict (bool): If True, return the motor positions as a dictionary with motor IDs as keys.
                           If False, return as numpy array.
        
        Returns:
            Union[np.ndarray, dict]: Motor positions either as numpy array or dictionary {motor_id: position}.
        """
        with self._motor_lock:
            motor_pos = self._dxl_client.read_pos_vel_cur()[0]
            if as_dict:
                return {motor_id: pos for motor_id, pos in zip(self.motor_ids, motor_pos)}
            return motor_pos
        
    def get_motor_current(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Get the current motor currents in mA.
        
        Args:
            as_dict (bool): If True, return the motor currents as a dictionary with motor IDs as keys.
                           If False, return as numpy array.
        
        Returns:
            Union[np.ndarray, dict]: Motor currents either as numpy array or dictionary {motor_id: current}.
        """
        with self._motor_lock:
            motor_current = self._dxl_client.read_pos_vel_cur()[2]
            if as_dict:
                return {motor_id: current for motor_id, current in zip(self.motor_ids, motor_current)}
            return motor_current
        
    def get_motor_temp(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Get the current motor temperatures in Celsius.
        
        Args:
            as_dict (bool): If True, return the motor temperatures as a dictionary with motor IDs as keys.
                           If False, return as numpy array.
        
        Returns:
            Union[np.ndarray, dict]: Motor temperatures either as numpy array or dictionary {motor_id: temperature}.
        """
        with self._motor_lock:
            motor_temp = self._dxl_client.read_temperature()
            if as_dict:
                return {motor_id: temp for motor_id, temp in zip(self.motor_ids, motor_temp)}
            return motor_temp

    def get_joint_pos(self, as_list: bool = True) -> Union[dict, list]:
        """Get the current joint positions.
    
        Args:
            as_list (bool): If True, return the joint positions as a list in the order of joint_ids.
                            If False, return the joint positions as a dictionary.
    
        Returns:
            Union[dict, list]: Joint positions as a list [position1, position2, ...] in the order of joint_ids
                               or as a dictionary {joint_name: position}.
        """
        motor_pos = self.get_motor_pos()
        joint_pos = self._motor_to_joint_pos(motor_pos)
        if as_list:
            return [joint_pos[joint] for joint in self.joint_ids]
    
        return joint_pos
         
    def set_joint_pos(self, joint_pos: Union[dict, list], num_steps: int = 1, step_size: float = 1.0):
        """Set the desired joint positions. If nun_steps > 1, the hand will move to the target position in a smooth, gradual motion (depending also on step_size).
    
        Args:
            joint_pos (dict or list): If dict, it should be {joint_name: desired_position}.
                                    If list, it should contain positions in the order of joint_ids.
            num_steps (int): Number of steps to reach the target position. If 1, moves directly to target.
            step_size (float): Time to wait between steps in seconds.
        """
        
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
        else:
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
        """Set the hand to the zero position by moving all joints simultaneously to their zero positions
        in a smooth, gradual motion.
        
        Args:
            num_steps (int): Number of steps to reach the zero position.
            step_size (float): Step size for each joint.
        """
        self.set_joint_pos({joint: 0 for joint in self.joint_ids}, num_steps=num_steps, step_size=step_size)
        
    def set_neutral_position(self, num_steps: int = 25, step_size: float = 0.001):
        """Set the hand to the neutral position by moving all joints simultaneously to their neutral positions
        in a smooth, gradual motion.

        Args:
            num_steps (int): Number of steps to reach the neutral position.
            step_size (float): Step size for each joint.
        """
        if self.neutral_position is None:
            raise ValueError("Neutral position is not set. Please set the neutral position in the config.yaml file.")
        print(self.neutral_position)
        self.set_joint_pos(self.neutral_position, num_steps=num_steps, step_size=step_size)
        
    def init_joints(self, calibrate: bool = False
                    ):
        """Initialize the joints, enables torque, sets the control mode and sets to the zero position.
        If the hand is not calibrated, it will calibrate the hand. 
        
        Args:
            calibrate (bool): If True, the hand will be calibrated
        
        """
        self.enable_torque()
        self.set_control_mode(self.control_mode)
        self.set_max_current(self.max_current)
        
        if not self.calibrated or calibrate:
            self.calibrate()
   
        self._compute_wrap_offsets_dict()
        self.set_joint_pos(self.neutral_position)

    def is_calibrated(self, verbose: bool = False) -> bool:
        """Check if the hand is calibrated.

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
                joint_name = self.motor_to_joint_dict.get(motor_id, "Unknown")
                # Corrected escape sequence
                uncalibrated_messages.append(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing motor limits).\033[0m")
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
            if not uncalibrated_messages:
                print("All joints found to be calibrated.")
            else:
                for msg in uncalibrated_messages:
                    print(msg)
        
        return overall_calibrated

    def calibrate(self):
        """Calibrate the hand by moving the joints to their limits and setting the ROMs. 
        
        The proecess is mostly hardware independent and is defined in the config.yaml file.
        The motor position is increseed and decreased flexing and extending each joint while recording the reached limits.
        """        
        # Store the min and max values for each motor
        motor_limits = self.motor_limits_dict.copy()

        self._compute_wrap_offsets_dict()
        for step in self.calib_sequence:
            for joint in step["joints"].keys():
                motor_id = self.joint_to_motor_map[joint]
                motor_limits[motor_id] = [None, None]
                self._wrap_offsets_dict[motor_id] = 0.0

        # Set calibration control mode
        self.set_control_mode('current_based_position')
        self.set_max_current(self.calib_current)
        self.enable_torque()
        

        for step in self.calib_sequence:
            desired_increment, motor_reached_limit, directions, position_buffers, motor_reached_limit, calibrated_joints, position_logs, current_log = {}, {}, {}, {}, {}, {}, {}, {}

            for joint, direction in step["joints"].items(): 
                if joint == 'wrist':
                    self.set_max_current(self.wrist_calib_current)
                else:
                    self.set_max_current(self.calib_current)
                    
                motor_id = self.joint_to_motor_map[joint]
                sign = 1 if direction == 'flex' else -1
                if self.joint_inversion_dict.get(joint, False):
                    sign = -sign
                directions[motor_id] = sign
                position_buffers[motor_id] = deque(maxlen=self.calib_num_stable)
                position_logs[motor_id] = []
                current_log[motor_id] = []
                motor_reached_limit[motor_id] = False
            
            while(not all(motor_reached_limit.values())):                
                for motor_id, reached_limit in motor_reached_limit.items():
                    if not reached_limit:
                        desired_increment[motor_id] = directions[motor_id] * self.calib_step_size

                self._set_motor_pos(desired_increment, rel_to_current=True)
                time.sleep(self.calib_step_period)
                curr_pos = self.get_motor_pos()
                
                for motor_id in desired_increment.keys():
                    if not motor_reached_limit[motor_id]:
                        position_buffers[motor_id].append(curr_pos[self.motor_id_to_idx_dict[motor_id]])
                        position_logs[motor_id].append(float(curr_pos[self.motor_id_to_idx_dict[motor_id]]))
                        current_log[motor_id].append(float(self.get_motor_current()[self.motor_id_to_idx_dict[motor_id]]))

                        # Check if buffer is full and all values are close
                        if len(position_buffers[motor_id]) == self.calib_num_stable and np.allclose(position_buffers[motor_id], position_buffers[motor_id][0], atol=self.calib_threshold):
                            motor_reached_limit[motor_id] = True
                            # disable torque for the motor
                            if joint == 'wrist' or joint == 'thumb_abd': # don\'t disable because of gravity
                                avg_limit = float(np.mean(position_buffers[motor_id]))
                            else:
                                self.disable_torque([motor_id])
                                time.sleep(0.05)
                                avg_limit = float(self.get_motor_pos()[self.motor_id_to_idx_dict[motor_id]])
                            print(f"Motor {motor_id} corresponding to joint {self.motor_to_joint_dict[motor_id]} reached the limit at {avg_limit} rad.")
                            if directions[motor_id] == 1:
                                motor_limits[motor_id][1] = avg_limit
                            if directions[motor_id] == -1:
                                motor_limits[motor_id][0] = avg_limit
                            self.enable_torque([motor_id])
                
            # find ratios of all motors that have been calibrated in this step
            for joint, direction in step["joints"].items(): 
                motor_id = self.joint_to_motor_map[joint]
                if motor_limits[motor_id][0] is None or motor_limits[motor_id][1] is None:
                    continue
                delta_motor = motor_limits[motor_id][1] - motor_limits[motor_id][0]
                delta_joint = self.joint_roms_dict[self.motor_to_joint_dict[motor_id]][1] - self.joint_roms_dict[self.motor_to_joint_dict[motor_id]][0]
                self.joint_to_motor_ratios_dict[motor_id] = float(delta_motor / delta_joint) 
                print("Joint calibrated: ", joint)
                calibrated_joints[joint] = 0.0
  
            update_yaml(self.calib_path, 'joint_to_motor_ratios', self.joint_to_motor_ratios_dict)
            update_yaml(self.calib_path, 'motor_limits', motor_limits)
            self.motor_limits_dict = motor_limits
            if calibrated_joints:
                print("Setting calibrated joints")
                self.set_joint_pos(calibrated_joints, num_steps=25, step_size=0.001)
            time.sleep(0.1)    
            
        print("Is fully calibrated: ", self.is_calibrated())
        self.calibrated = self.is_calibrated()
        update_yaml(self.calib_path, 'calibrated', self.calibrated)
        self.set_joint_pos(calibrated_joints, num_steps=25, step_size=0.001)
        self.set_max_current(self.max_current)
       
    def calibrate_manual(self):
        
        raise NotImplementedError("Manual calibration is not implemented yet. Please use the automatic calibration method.")
        
        self.disable_torque()

        calibrated_joints = {}
        self._compute_wrap_offsets_dict()
        motor_limits = self.motor_limits_dict.copy()

        for step in self.calib_sequence:
            for joint in step["joints"].keys():
                motor_id = self.joint_to_motor_map[joint]
                motor_limits[motor_id] = [None, None]
                self._wrap_offsets_dict[motor_id] = 0.0

        for i, step in enumerate(self.calib_sequence, start=1):
            for joint, _ in step["joints"].items():
                motor_id = self.joint_to_motor_map[joint]

                print(f"Progress: {i}/{len(self.calib_sequence)}")
                print(f"\033[1;35mPlease flex joint {joint} corresponding to motor {motor_id} fully and press enter.\033[0m")
                input()
                flex_position = float(self.get_motor_pos()[self.motor_id_to_idx_dict[motor_id]])
                motor_limits[motor_id][1] = flex_position

                print(f"\033[1;35mPlease extend the joint {joint} corresponding to motor {motor_id} fully and press enter.\033[0m")
                input()
                extend_position = float(self.get_motor_pos()[self.motor_id_to_idx_dict[motor_id]])
                motor_limits[motor_id][0] = extend_position
                
                delta_motor = abs(motor_limits[motor_id][1] - motor_limits[motor_id][0])
                delta_joint = abs(self.joint_roms_dict[joint][1] - self.joint_roms_dict[joint][0])
                self.joint_to_motor_ratios_dict[motor_id] = float(delta_motor / delta_joint)

                calibrated_joints[joint] = 0.0

                print(f"Joint {joint} calibrated. Motor limits: {motor_limits[motor_id]} rad. Ratio: {self.joint_to_motor_ratios_dict[motor_id]}")
                update_yaml(self.calib_path, 'joint_to_motor_ratios', self.joint_to_motor_ratios_dict)
                update_yaml(self.calib_path, 'motor_limits', motor_limits)

                stop_flag = False

                def wait_for_enter_local():
                    nonlocal stop_flag
                    input()
                    stop_flag = True

                thread = threading.Thread(target=wait_for_enter_local, daemon=True)
                thread.start()

                while not stop_flag:
                    curr_pos = self.get_motor_pos()[self.motor_id_to_idx_dict[motor_id]]
                    joint_pos = self.get_joint_pos()[joint]
                    print(f"\rMotor Pos: {curr_pos}, Joint Pos: {joint_pos}", end="")
                    time.sleep(0.01)

                print()

        self.motor_limits_dict.update(motor_limits)
        update_yaml(self.calib_path, 'motor_limits', self.motor_limits_dict)
        update_yaml(self.calib_path, 'joint_to_motor_ratios', self.joint_to_motor_ratios_dict)

        self.set_joint_pos(calibrated_joints)
        time.sleep(1)
        self.set_max_current(self.max_current)

        print("Is fully calibrated: ", self.is_calibrated())
        self.calibrated = self.is_calibrated()
        update_yaml(self.calib_path, 'calibrated', self.calibrated)

        print("\033[1;33mMove away from the hand. Setting joints to 0 in:\033[0m")
        for i in range(3, 0, -1):
            print(f"\033[1;33m{i}\033[0m")
            time.sleep(1)

        self.set_joint_pos(calibrated_joints)
        time.sleep(1)

    def _compute_wrap_offsets_dict(self):
        """Read motor_pos positions once and figure out ±1-turn offsets so that
        pos + offset ∈ [motor_limits_lo, motor_limits_hi].
        """

        motor_pos = self.get_motor_pos()

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

        print(f"Offsets: {offsets}")

        self._wrap_offsets_dict = offsets

    def _set_motor_pos(self, desired_pos: Union[dict, np.ndarray, list], rel_to_current: bool = False):
        """Set the desired motor positions in radians.
        
        Args:
            desired_pos (dict or np.ndarray or list): 
                - If dict: {motor_id: desired_position}. Can be partial. Only motors in the dict will be commanded.
                - If np.ndarray or list: Desired positions for all motors in the order of self.motor_ids.
                                         None values will be skipped, and the corresponding motor won't be commanded.
            rel_to_current (bool): If True, the desired position is relative to the current position.
        """
        with self._motor_lock:
            current_positions = self.get_motor_pos() # np.ndarray of all motor positions

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
                        f"Length of desired_pos (list/ndarray) ({len(desired_pos)}) "
                        f"must match the number of configured motor_ids ({len(self.motor_ids)})."
                    )
                
                for i, pos_val in enumerate(desired_pos):
                    if pos_val is None or math.isnan(pos_val):
                        continue
                    else:
                        motor_ids_to_write.append(self.motor_ids[i])
                        current_pos_of_motor = current_positions[i]
                        if rel_to_current:
                            positions_to_write.append(float(pos_val) + current_pos_of_motor)
                        else:
                            positions_to_write.append(float(pos_val))
                
                if not motor_ids_to_write:
                    print("Info: All positions in desired_pos (list/array) were None. No motor commands sent.")
                    return

                motor_ids_to_write = motor_ids_to_write
                positions_to_write = np.array(positions_to_write, dtype=float)
            
            else:
                raise ValueError("desired_pos must be a dict, np.ndarray, or list.")
   
            self._dxl_client.write_desired_pos(motor_ids_to_write, positions_to_write)
    
    def _motor_to_joint_pos(self, motor_pos: np.ndarray) -> dict:
        """Convert motor positions into joint positions.
        
        Args:
            motor_pos (np.ndarray): Motor positions.
        
        Returns:
            dict: {joint_name: position}
        """
        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()

        joint_pos = {}
        for idx, pos in enumerate(motor_pos):
            motor_id = self.motor_ids[idx]
            joint_name = self.motor_to_joint_dict.get(motor_id)
            if any(limit is None for limit in self.motor_limits_dict[motor_id]):
                joint_pos[joint_name] = None #TODO: Add a warning here the probably the motor is not calibrated
            elif self.joint_to_motor_ratios_dict[motor_id] == 0:
                joint_pos[joint_name] = None #TODO: Add a warning here the probably the motor is not calibrated
            else:
                wrapped_pos = pos - self._wrap_offsets_dict.get(motor_id, 0.0)
                
                if self.joint_inversion_dict.get(joint_name, False):
                    joint_pos[joint_name] = self.joint_roms_dict[joint_name][1] - (wrapped_pos - self.motor_limits_dict[motor_id][0]) / self.joint_to_motor_ratios_dict[motor_id]
                else:
                    joint_pos[joint_name] = self.joint_roms_dict[joint_name][0] + (wrapped_pos - self.motor_limits_dict[motor_id][0]) / self.joint_to_motor_ratios_dict[motor_id]
        return joint_pos
    
    def _joint_to_motor_pos(self, joint_pos: dict) -> np.ndarray:
        """Convert desired joint positions into motor commands.
    
        Args:
            joint_pos (dict): {joint_name: desired_position}

        Returns:
            np.ndarray: Motor positions.
        """
        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()

        motor_pos = [None] * len(self.get_motor_pos())
                
        for joint_name, pos in joint_pos.items():
            motor_id = self.joint_to_motor_map.get(joint_name)
            if motor_id is None or pos is None:
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = None
                continue

            if self.motor_limits_dict[motor_id][0] is None or self.motor_limits_dict[motor_id][1] is None or self.joint_to_motor_ratios_dict[motor_id] == 0:
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = None
                print(f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m")
                continue
            
            min_pos, max_pos = self.joint_roms_dict[joint_name]
            
            # Clip the position if outside ROM and notify
            if pos < min_pos or pos > max_pos:
                clipped_pos = max(min_pos, min(max_pos, pos))
                # print(f"Clipping {joint_name} from {pos} to {clipped_pos} (ROM: {min_pos} to {max_pos})")
                pos = clipped_pos

            if self.joint_inversion_dict.get(joint_name, False):
                # Inverted: higher ROM value corresponds to lower motor position.
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = self.motor_limits_dict[motor_id][0] + (self.joint_roms_dict[joint_name][1] - pos) * self.joint_to_motor_ratios_dict[motor_id]
            else:
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = self.motor_limits_dict[motor_id][0] + (pos - self.joint_roms_dict[joint_name][0]) * self.joint_to_motor_ratios_dict[motor_id]  
            
            motor_pos[self.motor_id_to_idx_dict[motor_id]] += self._wrap_offsets_dict.get(motor_id, 0.0)
            
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
                self.calibrated = False
                update_yaml(self.calib_path, 'calibrated', False)
                

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
            return True, "Mock connection successful"
        except Exception as e:
            self._dxl_client = None
            return False, f"Mock connection failed: {str(e)}"
        
    
if __name__ == "__main__":
    # Example usage:
    hand = OrcaHand()
    status = hand.connect()
    hand.enable_torque()
    hand.calibrate()

    # Set the desired joint positions to 0
    hand.set_joint_pos({joint: 0 for joint in hand.joint_ids})
    hand.disable_torque()
    hand.disconnect()