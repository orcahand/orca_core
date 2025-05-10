from orca_core import OrcaHand
import orca_core
import yaml
import argparse
import time
from pathlib import Path
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Load hand configurations from YAML files.")
    parser.add_argument("configuration", type=str, help="Name of the configuration to load.")
    args = parser.parse_args()
    configuration = args.configuration

    orca_core_path = Path(orca_core.__file__).parent
    configuration_path = orca_core_path / "configurations" / f"{configuration}.yaml"
    if not configuration_path.is_file():
        print(f"Configuration file {configuration_path} does not exist.")
        exit(1)

    with open(configuration_path, 'r') as file:
        hand_pos = yaml.safe_load(file)

    hand = OrcaHand()
    status = hand.connect()    
    hand.enable_torque()

    hand.set_joint_pos(hand_pos)
    time.sleep(1)

    if configuration == "italian":
        orca_core_path = Path(orca_core.__file__).parent
        model_path = orca_core_path / "models" / "orcahand_v1"

        calibration_yaml = model_path / "calibration.yaml"
        with open(calibration_yaml, 'r') as file:
            calibration = yaml.safe_load(file)
        
        config_yaml = model_path / "config.yaml"
        with open(config_yaml, 'r') as file:
            config = yaml.safe_load(file)

        wrist_motor_id = config["joint_to_motor_map"]["wrist"]
        wrist_lower, wrist_upper = np.array(calibration["motor_limits"][wrist_motor_id]) / np.pi * 180

        curr_joint_pos = hand.get_joint_pos()
        curr_wrist_pos = curr_joint_pos["wrist"]

        # Sinusoidal motion parameters
        frequency = 0.5
        amplitude = (wrist_upper - wrist_lower) / 2
        offset = (wrist_upper + wrist_lower) / 2
        duration = 6  # seconds
        timestep = 0.01  # 50 Hz loop

        # Compute initial phase to start at current wrist position
        # Clamp to avoid domain error in arcsin
        sin_arg = np.clip((curr_wrist_pos - offset) / amplitude, -1.0, 1.0)
        initial_phase = np.arcsin(sin_arg)

        # Time loop
        start_time = time.time()
        while time.time() - start_time < duration:
            t = time.time() - start_time
            wrist_angle = amplitude * np.sin(2 * np.pi * frequency * t)

            # Update joint position only for wrist
            target_joint_pos = curr_joint_pos.copy()
            target_joint_pos["wrist"] = wrist_angle

            hand.set_joint_pos(target_joint_pos)
            time.sleep(timestep)

        time.sleep(3)

    hand.disable_torque()
    hand.disconnect()
