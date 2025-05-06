import time
import yaml
import numpy as np
from orca_core import OrcaHand

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
    filename = input("Enter the filename of the replay sequence (e.g., replay_sequence_YYYYMMDD_HHMMSS.yaml): ")
    try:
        with open(filename, "r") as file:
            replay_data = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"File {filename} not found.")
        return

    waypoints = replay_data.get("waypoints", [])
    if not waypoints:
        print("No waypoints found in the file.")
        return

    hand = OrcaHand('/Users/ccc/dev/orca/orca_core/orca_core/models/orcahand_v1')
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    hand.enable_torque()
    print("Torque enabled. Starting replay...")

    # --- Parameters ---
    interp_time = 0.15     # seconds between waypoints
    step_time = 0.02     # timestep for interpolation
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