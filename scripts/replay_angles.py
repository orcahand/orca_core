import time
import yaml
import numpy as np
import argparse
from orca_core import OrcaHand
import os

def linear_interp(t):
    return t

def ease_in_out(t):
    return 0.5 * (1 - np.cos(np.pi * t))

def interpolate_waypoints(start, end, duration, step_time, mode="linear"):
    n_steps = int(duration / step_time)
    interp_func = linear_interp if mode == "linear" else ease_in_out
    for i in range(n_steps + 1):
        t = i / n_steps
        alpha = interp_func(t)
        yield [(1 - alpha) * s + alpha * e for s, e in zip(start, end)]

def main():
    parser = argparse.ArgumentParser(description='Replay recorded hand movements')
    parser.add_argument('--step_time', type=float, default=0.02,
                      help='Timestep for interpolation (default: 0.02)')
    parser.add_argument("model_path", type=str, nargs="?", default=None, help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1)")
    parser.add_argument('--replay_file', type=str, required=True, help="Path to the replay file. Can be an absolute/relative path (e.g., 'replay_sequences/my_file.yaml'), or a plain filename which will be sought in 'project_root/replay_sequences/'.")
    args = parser.parse_args()
    
    user_input_filename = args.replay_file.strip()
    
    project_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    default_replay_dir_path = os.path.join(project_root, 'replay_sequences')

    if os.path.isabs(user_input_filename):
        full_filepath = user_input_filename
    elif os.sep in user_input_filename:
        full_filepath = os.path.join(project_root, user_input_filename)
    else:
        full_filepath = os.path.join(default_replay_dir_path, user_input_filename)
    
    full_filepath = os.path.abspath(full_filepath)
    
    try:
        with open(full_filepath, "r") as file:
            replay_data = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"File not found at the resolved path: {full_filepath}")
        return

    waypoints = replay_data.get("waypoints", [])
    if not waypoints:
        print("No waypoints found in the file.")
        return

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    hand.enable_torque()
    print("Torque enabled. Starting replay...")

    # --- Parameters ---
    interp_time = 0.5     # seconds between waypoints
    step_time = args.step_time     # timestep for interpolation
    mode = "ease_in_out" # can be "linear" or "ease_in_out"

    try:
        while True:  # Infinite loop
            for i in range(len(waypoints)):
                start = waypoints[i]
                end = waypoints[(i + 1) % len(waypoints)]  # Loop back to the first waypoint
                print(f"Interpolating {mode} from waypoint {i+1} to waypoint {(i+2) if (i+1) < len(waypoints) else 1}...")

                n_steps = int(interp_time / step_time)
                interp_func = linear_interp if mode == "linear" else ease_in_out
                start_time = time.time()

                for step in range(n_steps + 1):
                    t = step / n_steps
                    alpha = interp_func(t)
                    pose = [(1 - alpha) * s + alpha * e for s, e in zip(start, end)]
                    hand.set_joint_pos(pose)

                    # Compute target time for this step and wait only if ahead
                    target_time = start_time + step * step_time
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