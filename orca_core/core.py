import os
import sys
import time
from typing import Dict, List, Union
from collections import deque
from threading import RLock
from .hardware.dynamixel_client import *
from .utils.yaml_utils import *
from .utils.load_utils import get_model_path

class OrcaHand:
    """
    OrcaHand class is used to abtract hardware control the hand of the robot with simple high level control methods in joint space. 
   """
    def __init__(self, model_path: str = None):
        """
        Initialize the OrcaHand class.

        Args:
            orca_config (str): The path to the orca_config.yaml file, which includes static information like ROMs, motor IDs, etc. 
        """
        # Find the model directory if not provided
        self.model_path = get_model_path(model_path)
        # Load configurations from the YAML files
        self.config_path = os.path.join(self.model_path, "config.yaml")
        self.urdf_path = os.path.join(self.model_path, "urdf", "orcahand.urdf")
        self.mjco_path = os.path.join(self.model_path, "mujoco", "orcahand.xml")
        self.calib_path = os.path.join(self.model_path, "calibration.yaml")
        
        config = read_yaml(self.config_path)
        calib = read_yaml(self.calib_path)
            
        self.baudrate: int = config.get('baudrate', 3000000)
        self.port: str = config.get('port', '/dev/ttyUSB0')
        self.max_current: int = config.get('max_current', 300)
        self.control_mode: str = config.get('control_mode', 'current_position')
        
        self.calib_current: str = config.get('calib_current', 200)
        self.calib_step_size: float = config.get('calib_step_size', 0.1)
        self.calib_step_period: float = config.get('calib_step_period', 0.01)
        self.calib_threshold: float = config.get('calib_threshold', 0.01)
        self.calib_num_stable: int = config.get('calib_num_stable', 20)
        self.calib_sequence: Dict[str, Dict[str, str]] = config.get('calib_sequence', [])
        self.calibrated: bool = calib.get('calibrated', False)
        self.motor_limits: Dict[int, List[float]] = calib.get('motor_limits', {})
        self.joint_to_motor_ratios: Dict[int, float] = calib.get('joint_to_motor_ratios', {})
        
        self.motor_ids: List[int] = config.get('motor_ids', [])
        self.joint_ids: List[str] = config.get('joint_ids', [])
        self.joint_to_motor_map: Dict[str, int] = config.get('joint_to_motor_map', {})
        self.motor_to_joint_map: Dict[int, str] = {v: k for k, v in self.joint_to_motor_map.items()}
        self.joint_roms: Dict[str, List[float]] = config.get('joint_roms', {})
               
        self._dxl_client: DynamixelClient = None
        self._motor_lock: RLock = RLock()
        
        self._sanity_check()       
        
    def __del__(self):
        """
        Destructor to disconnect from the hand.
        """
        self.disconnect()
        
    def connect(self) -> tuple[bool, str]:
        """
        Connect to the hand with the DynamixelClient.
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
        """
        Disconnect from the hand.
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
        
    def enable_torque(self, motor_ids: List[int] = None):
        """
        Enable torque for the motors.
        
        Parameters:
        - motor_ids (list): List of motor IDs to enable the torque. If None, all motors will be
        enabled
        """
        if motor_ids is None:
            motor_ids = self.motor_ids
        with self._motor_lock:
            self._dxl_client.set_torque_enabled(motor_ids, True)        

    def disable_torque(self, motor_ids: List[int] = None):
        """
        Disable torque for the motors.
        
        Parameters:
        - motor_ids (list): List of motor IDs to disable the torque. If None, all motors will be disabled.
        """
        if motor_ids is None:
            motor_ids = self.motor_ids
        with self._motor_lock:
            self._dxl_client.set_torque_enabled(motor_ids, False)
    
    def set_max_current(self, current: Union[float, List[float]]):
        """
        Set the maximum current for the motors.
        
        Parameters:
        - current (int) or (list): If list, it should be the maximum current for each motor, otherwise it will be the same for all motors.
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
        """
        Set the control mode for the motors.
        
        Parameters:
        - mode (str): Control mode.
            current: Current control mode (0)
            velocity: Velocity control mode (1)
            position: Position control mode (3)
            multi_turn_position: Multi-turn position control mode (4)
            current_based_position: Current-based position control mode (5)
        - motor_ids (list): List of motor IDs to set the control mode. If None, all motors will be set.
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
            
    def get_motor_pos(self) -> np.ndarray:
        """
        Get the current motor positions in radians (Note that this includes offsets of the motors).
        
        Returns:
            np.ndarray: Motor positions.
        """
        with self._motor_lock:
            motor_pos = self._dxl_client.read_pos_vel_cur()[0]
            return motor_pos
        
    def get_motor_current(self) -> np.ndarray:
        with self._motor_lock:
            return self._dxl_client.read_pos_vel_cur()[2]
        
    def get_motor_temp(self) -> np.ndarray:
        with self._motor_lock:
            return self._dxl_client.read_temperature()

    def get_joint_pos(self) -> dict:
        """
        Get the current joint positions.
        Returns:
            dict: {joint_name: position}
        """
        motor_pos = self.get_motor_pos()
        joint_pos = self._motor_to_joint_pos(motor_pos)
        return joint_pos
         
    def set_joint_pos(self, joint_pos: dict):
        """
        Set the desired joint positions.
        
        Parameters:
        - joint_pos (dict): {joint_name: desired_position}
        """
        if not isinstance(joint_pos, dict):
            raise ValueError("joint_pos must be a dict.")
        motor_pos = self._joint_to_motor_pos(joint_pos)
        self._set_motor_pos(motor_pos)
        
    def init_joints(self, calibrate: bool = False
                    ):
        """
        Initialize the joints, enables torque, sets the control mode and sets to the zero position.
        If the hand is not calibrated, it will calibrate the hand. 
        
        Parameters:
        - calibrate (bool): If True, the hand will be calibrated
        
        """
        self.enable_torque()
        self.set_control_mode(self.control_mode)
        self.set_max_current(self.max_current)

        if not self.calibrated or calibrate:
            self.calibrate()
   
        self.set_joint_pos({joint: 0 for joint in self.joint_ids})
                   
    def is_calibrated(self) -> bool:
        """
        Check if the hand is calibrated.
        Returns:
            bool: True if calibrated, False otherwise.
        """
        for motor_limit in self.motor_limits.values():
            if any(limit is None for limit in motor_limit):
                return False
        return True
              
    def calibrate(self):
        """
        Calibrate the hand by moving the joints to their limits and setting the ROMs. The proecess is hardware independent and is defined in the config.yaml file.
        By increasing the motor position, the motor will turn counter-clockwise, flexing the joint.
        """        
        # Store the min and max values for each motor
        motor_limits = {motor_id: [None, None] for motor_id in self.motor_ids}

        # Set calibration control mode
        self.set_control_mode('current_based_position')
        self.set_max_current(self.calib_current)
        self.enable_torque()
        
        for step in self.calib_sequence:
            desired_increment, motor_reached_limit, directions, position_buffers, motor_reached_limit, calibrated_joints, position_logs, current_log = {}, {}, {}, {}, {}, {}, {}, {}
            
            for joint, direction in step["joints"].items():          
                motor_id = self.joint_to_motor_map[joint]
                directions[motor_id] = 1 if direction == 'flex' else -1
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
                        position_buffers[motor_id].append(curr_pos[motor_id - 1])
                        position_logs[motor_id].append(float(curr_pos[motor_id - 1]))
                        current_log[motor_id].append(float(self.get_motor_current()[motor_id - 1]))

                        # Check if buffer is full and all values are close
                        if len(position_buffers[motor_id]) == self.calib_num_stable and np.allclose(position_buffers[motor_id], position_buffers[motor_id][0], atol=self.calib_threshold):
                            motor_reached_limit[motor_id] = True
                            avg_limit = float(np.mean(position_buffers[motor_id]))
                            print(f"Motor {motor_id} corresponding to joint {self.motor_to_joint_map[motor_id]} reached the limit at {avg_limit} rad.")
                            if directions[motor_id] == 1:
                                motor_limits[motor_id][1] = avg_limit
                            if directions[motor_id] == -1:
                                motor_limits[motor_id][0] = avg_limit
                
            # find ratios of all motors that have been calibrated
            for motor_id, limits in motor_limits.items():
                if limits[0] is None or limits[1] is None:
                    continue
                delta_motor = limits[1] - limits[0]
                delta_joint = self.joint_roms[self.motor_to_joint_map[motor_id]][1] - self.joint_roms[self.motor_to_joint_map[motor_id]][0]
                self.joint_to_motor_ratios[motor_id] = float(delta_motor / delta_joint) 
                
                # Zero all joints that have been calibrated during this step
                calibrated_joints[self.motor_to_joint_map[motor_id]] = 0

            
            update_yaml(self.calib_path, 'joint_to_motor_ratios', self.joint_to_motor_ratios)
            update_yaml(self.calib_path, 'motor_limits', motor_limits)
            self.motor_limits = motor_limits
            self.set_joint_pos(calibrated_joints)
            time.sleep(1)    
            
        print("Is fully calibrated: ", self.is_calibrated())
        self.calibrated = self.is_calibrated()
        update_yaml(self.calib_path, 'calibrated', self.calibrated)
        self.set_joint_pos(calibrated_joints)
        self.set_max_current(self.max_current)
       
    def calibrate_manual(self):
        """
        Calibrate the hand manually by moving the joints to their limits
        """
        self.disable_torque()
        
        calibrated_joints = {}
        for i, step in enumerate(self.calib_sequence, start=1):
            for joint, _ in step["joints"].items():  
                motor_id = self.joint_to_motor_map[joint]
                print(f"Progress: {i}/{len(self.calib_sequence)}")
                print(f"\033[1;35mPlease flex joint {joint} corresponding to motor {motor_id} fully and press enter.\033[0m")
                input()
                self.motor_limits[motor_id][1] = float(self.get_motor_pos()[motor_id - 1])
                print(f"\033[1;35mPlease extend the joint {joint} corresponding to motor {motor_id} fully and press enter.\033[0m")
                input()
                self.motor_limits[motor_id][0] = float(self.get_motor_pos()[motor_id - 1])

                delta_motor = self.motor_limits[motor_id][1] - self.motor_limits[motor_id][0]
                delta_joint = self.joint_roms[self.motor_to_joint_map[motor_id]][1] - self.joint_roms[self.motor_to_joint_map[motor_id]][0]
                self.joint_to_motor_ratios[motor_id] = float(delta_motor / delta_joint) 
                calibrated_joints[self.motor_to_joint_map[motor_id]] = 0
            
                print(f"Joint {joint} calibrated. Motor limits: {self.motor_limits[motor_id]} rad. Ratio: {self.joint_to_motor_ratios[motor_id]}")
                update_yaml(self.calib_path, 'joint_to_motor_ratios', self.joint_to_motor_ratios)
                update_yaml(self.calib_path, 'motor_limits', self.motor_limits)
                
                count = 0
                while count < 5:
                    curr_pos = self.get_motor_pos()[motor_id - 1]
                    print(f"\rMotor Pos: {curr_pos}, Joint Pos: {self.get_joint_pos()[joint]}")
                    #sys.stdout.write(f"\rMotor Pos: {curr_pos:.4f}, Joint Pos: {self._motor_to_joint_pos([curr_pos]):.4f}")
                    #sys.stdout.flush()
                    time.sleep(1)
                    count += 1
                    print(count)
                    
                
        self.enable_torque()
        update_yaml(self.calib_path, 'joint_to_motor_ratios', self.joint_to_motor_ratios)
        update_yaml(self.calib_path, 'motor_limits', self.motor_limits)
        print("Is fully calibrated: ", self.is_calibrated())
        self.calibrated = self.is_calibrated()
        update_yaml(self.calib_path, 'calibrated', self.calibrated)
        
        print("\033[1;33mMove away from the hand. Setting joints to 0 in:\033[0m")
        for i in range(3, 0, -1):
            print(f"\033[1;33m{i}\033[0m")
            time.sleep(1)
            
        self.set_joint_pos(calibrated_joints)
        time.sleep(1)
        self.set_max_current(self.max_current)
        
        
    
    def _set_motor_pos(self, desired_pos: Union[dict, np.ndarray, list], rel_to_current: bool = False):
        """
        Set the desired motor positions in radians.
        
        Parameters:
        - desired_pos (dict or np.ndarray or list): Desired motor positions. If dict, it should be {motor_id: desired_position} and it can be partial. If np.ndarray or list, it should be the desired positions for all motors in the order of motor_ids.
        - rel_to_current (bool): If True, the desired position is relative to the current position.
        """
        with self._motor_lock:
            current_pos = self.get_motor_pos()

            if isinstance(desired_pos, dict):
                motor_pos_array = np.array([
                    desired_pos.get(motor_id, 0 if rel_to_current else current_pos[motor_id - 1]) for motor_id in self.motor_ids
                ])
            elif isinstance(desired_pos, np.ndarray):
                assert len(desired_pos) == len(self.motor_ids), "Number of motor positions do not match the number of motors."
                motor_pos_array = desired_pos.copy()
            elif isinstance(desired_pos, list):
                assert len(desired_pos) == len(self.motor_ids), "Number of motor positions do not match the number of motors."
                motor_pos_array = np.array(desired_pos)
            else:
                raise ValueError("desired_pos must be a dict or np.ndarray or list")

            if rel_to_current:
                motor_pos_array += current_pos

            self._dxl_client.write_desired_pos(self.motor_ids, motor_pos_array)
    
    def _motor_to_joint_pos(self, motor_pos: np.ndarray) -> dict:
        """
        Convert motor positions into joint positions.
        
        Parameters:
        - motor_pos (np.ndarray): Motor positions.
        
        Returns:
        - dict: {joint_name: position}
        """          
        joint_pos = {}
        for motor_id, pos in enumerate(motor_pos, start=1):
            joint_name = self.motor_to_joint_map.get(motor_id)
            if any(limit is None for limit in self.motor_limits[motor_id]):
                joint_pos[joint_name] = None
            else:
                joint_pos[joint_name] = self.joint_roms[joint_name][0] + (pos - self.motor_limits[motor_id][0]) / self.joint_to_motor_ratios[motor_id]
        return joint_pos
    
    def _joint_to_motor_pos(self, joint_pos: dict) -> np.ndarray:
        """
        Convert desired joint positions into motor commands.
    
        Parameters:
        - joint_pos (dict): {joint_name: desired_position}
        """
        motor_pos = self.get_motor_pos()
        
        for joint_name, pos in joint_pos.items():
            motor_id = self.joint_to_motor_map.get(joint_name)
            if motor_id is None:
                continue
            if self.motor_limits[motor_id][0] is None or self.motor_limits[motor_id][1] is None:
                raise ValueError(f"Motor {motor_id} corresponding to joint {joint_name} is not calibrated.")
            motor_pos[motor_id - 1] = self.motor_limits[motor_id][0] + (pos - self.joint_roms[joint_name][0]) * self.joint_to_motor_ratios[motor_id]
            
            
        return motor_pos
               
    def _sanity_check(self):
        """
        Check if the configuration is correct and the IDs are consistent.
        """
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
            if joint not in self.joint_roms:
                raise ValueError(f"ROM for joint {joint} is not defined.")
            if motor_id not in self.motor_ids:
                raise ValueError(f"Motor ID {motor_id} is not in the motor IDs list.")
            
        for joint, rom in self.joint_roms.items():
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
          
        
        for motor_limit in self.motor_limits.values():
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

if __name__ == "__main__":
    # Example usage:
    hand = OrcaHand()
    print(hand.model_path)#
    print(hand.config_path)
    status = hand.connect()
    print(status)
    hand.enable_torque()
    hand.calibrate()

    # Set the desired joint positions to 0
    hand.set_joint({joint: 0 for joint in hand.joint_ids})
    hand.disable_torque()
    hand.disconnect()
    
    
    
    
        
    
    