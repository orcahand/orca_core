import time
import yaml
import numpy as np
import argparse
from orca_core import OrcaHand, MockOrcaHand
import os

def resolve_replay_file(user_input_filename, project_root):
    user_input_filename = user_input_filename.strip()
    default_replay_dir_path = os.path.join(project_root, 'replay_sequences')
    if os.path.isabs(user_input_filename):
        return os.path.abspath(user_input_filename)
    elif os.sep in user_input_filename:
        return os.path.abspath(os.path.join(project_root, user_input_filename))
    else:
        return os.path.abspath(os.path.join(default_replay_dir_path, user_input_filename))


def main():
    parser = argparse.ArgumentParser(description='Replay recorded hand movements')
    parser.add_argument('--step_time', type=float, default=0.02,
                      help='Timestep for interpolation (default: 0.02)')
    parser.add_argument("model_path", type=str, nargs="?", default=None, help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1)")
    parser.add_argument('--replay_file', type=str, required=True, help="Path to the replay file. Can be an absolute/relative path (e.g., 'replay_sequences/my_file.yaml'), or a plain filename which will be sought in 'project_root/replay_sequences/'.")
    args = parser.parse_args()
    
    project_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    replay_file_path = resolve_replay_file(args.replay_file, project_root)

    try:
        with open(replay_file_path, "r") as file:
            replay_data = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"File not found at the resolved path: {replay_file_path}")
        return

    waypoints = replay_data.get("waypoints", [])
    if not waypoints:
        print("No waypoints found in the file.")
        return
    
    hand = MockOrcaHand(model_path=args.model_path)     
    success, message = hand.connect()
    if not success:
        print(f"Failed to connect: {message}")
        return 1

    hand.enable_torque()
    print("Torque enabled. Starting replay...")

    interp_time = 0.8  # seconds between waypoints
    step_time = args.step_time
    mode = "ease_in_out"  # can be "linear" or "ease_in_out"
    max_iterations = 1000000  # or set as needed

    try:
        hand.replay_waypoints(
            waypoints,
            duration=interp_time,
            step_time=step_time,
            max_iterations=max_iterations,
            mode=mode,
            blocking=True
        )
    except KeyboardInterrupt:
        print("Replay interrupted.")
    except Exception as e:
        print(f"Error during replay: {e}")
    finally:
        hand.disable_torque()
        hand.disconnect()
        print("Replay finished. Disconnected from hand.")

if __name__ == "__main__":
    main()