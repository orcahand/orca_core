import time
import yaml
import numpy as np
import argparse
from orca_core import OrcaHand
import os

def main():
    parser = argparse.ArgumentParser(description='Replay continuous hand joint movements.')
    parser.add_argument("model_path", type=str, nargs="?", default=None, help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1_left)")
    parser.add_argument('--replay_file', type=str, required=True, help="Path to the replay file. Can be an absolute/relative path (e.g., 'replay_sequences/my_file.yaml'), or a plain filename which will be sought in 'project_root/replay_sequences/'.")
    args = parser.parse_args()

    user_input_replay_file = args.replay_file.strip()
    
    project_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    default_replay_dir_path = os.path.join(project_root, 'replay_sequences')

    if os.path.isabs(user_input_replay_file):
        full_filepath = user_input_replay_file
    elif os.sep in user_input_replay_file :
        full_filepath = os.path.join(project_root, user_input_replay_file)
    else:
        full_filepath = os.path.join(default_replay_dir_path, user_input_replay_file)
    
    full_filepath = os.path.abspath(full_filepath)

    try:
        with open(full_filepath, "r") as file:
            replay_data = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"File not found at the resolved path: {full_filepath}")
        return

    metadata = replay_data.get("metadata", {})
    if metadata.get("type") != "continuous":
        print("The file does not contain continuous recording data.")
        return

    sampling_frequency = metadata.get("sampling_frequency_hz")
    if sampling_frequency is None:
        print("Sampling frequency not found in metadata.")
        return

    print("Replaying with ", sampling_frequency, " Hz...")

    step_time = 1.0 / sampling_frequency

    waypoints = replay_data.get("angles", [])
    if not waypoints:
        print("No angles found in the file.")
        return

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    if "hand_type" in metadata:
        recorded_type = metadata["hand_type"]
        if recorded_type != getattr(hand, "type", None):
            print(f"Hand type mismatch: recorded={recorded_type}, connected={getattr(hand, 'type', 'unknown')}")
            return

    hand.enable_torque()
    print("Torque enabled. Starting real-time replay...")

    try:
        start_time = time.time()
        for i, pose in enumerate(waypoints):
            hand.set_joint_pos(pose)
            target_time = start_time + i * step_time
            now = time.time()
            if now < target_time:
                time.sleep(target_time - now)

    except KeyboardInterrupt:
        print("Replay interrupted.")

    finally:
        hand.disable_torque()
        print("Torque disabled.")

if __name__ == "__main__":
    main()
