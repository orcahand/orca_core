import os
import time
import yaml
import argparse
from orca_core import OrcaHand
from datetime import datetime


def record_continuous_angles(hand, output_dir, sampling_frequency=50.0, duration=None):
    """
    Continuously records joint angles from the OrcaHand.

    Args:
        hand (OrcaHand): An instance of the OrcaHand.
        output_dir (str): Directory to save the output file. This directory will be created if it doesn't exist.
        sampling_frequency (float): Frequency in Hz to sample the joint angles.
        duration (float or None): Duration in seconds to record. If None, records until KeyboardInterrupt.
    """

    os.makedirs(output_dir, exist_ok=True)

    prefix = input("Enter a prefix for the output file name (optional): ").strip()
    interval = 1.0 / sampling_frequency
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_dir, f"{prefix + '_' if prefix else ''}continuous_angles_{timestamp}.yaml")

    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    hand.set_neutral_position()
    time.sleep(1)
    hand.disable_torque()
    
    input("Hand is in neutral position. Press Enter to start recording...")
    print("Recording... Press Ctrl+C to stop.")

    data = {
        "metadata": {
            "type": "continuous",
            "hand_type": hand.type,
            "created_at": timestamp,
            "sampling_frequency_hz": sampling_frequency
        },
        "angles": []
    }

    start_time = time.time()

    try:
        while True:
            if duration and (time.time() - start_time) > duration:
                break
            angles = hand.get_joint_pos(as_list=True)
            data["angles"].append([float(angle) for angle in angles])
            print("Recording...", end="\r")
            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nRecording stopped by user.")

    with open(filename, "w") as f:
        yaml.dump(data, f)

    print(f"Continuous angle recording saved to {filename}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record continuous joint angles from the OrcaHand.")
    parser.add_argument("model_path", type=str, nargs="?", default=None, help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1_left)")
    parser.add_argument("--frequency", type=float, default=50.0, help="Sampling frequency in Hz")
    parser.add_argument("--duration", type=float, default=None, help="Recording duration in seconds (optional)")
    parser.add_argument("--output_dir", type=str, default=None, help="Directory to save the output file, relative to the project root. Defaults to 'replay_sequences/' at project root if not specified.")
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)

    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, '..'))
    if args.output_dir:
        final_output_dir = os.path.abspath(os.path.join(project_root, args.output_dir))
    else:
        final_output_dir = os.path.join(project_root, 'replay_sequences')
    
    record_continuous_angles(hand, output_dir=final_output_dir, sampling_frequency=args.frequency, duration=args.duration)
