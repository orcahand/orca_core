import time
import yaml
import numpy as np
import argparse
from orca_core import OrcaHand

def main():
    parser = argparse.ArgumentParser(description='Replay continuous hand joint movements.')
    parser.add_argument('model_path', type=str, help='Path to the OrcaHand model folder (e.g., /path/to/orcahand_v1_right)')
    parser.add_argument('replay_file', type=str, help='YAML file containing the recorded angles and metadata')
    args = parser.parse_args()

    try:
        with open(args.replay_file, "r") as file:
            replay_data = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"File {args.replay_file} not found.")
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
