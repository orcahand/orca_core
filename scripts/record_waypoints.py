import time
import yaml
from orca_core import OrcaHand, MockOrcaHand
import argparse
import os
from orca_core.utils.utils import update_yaml

def main():
    parser = argparse.ArgumentParser(description="Record waypoints for the ORCA Hand.")
    parser.add_argument("model_path", type=str, nargs="?", default=None, help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1_left)")
    parser.add_argument("--output_dir", type=str, default=None, help="Directory to save the replay sequence. Defaults to 'replay_sequences/' at the project root.")

    args = parser.parse_args()

    user_prefix = input("Enter a prefix for the replay filename (e.g., 'my_capture').\n"
                        "It will be saved as 'output_dir/YOUR_PREFIX_replay_sequence_TIMESTAMP.yaml'.\n"
                        "If empty, defaults to 'output_dir/replay_sequence_TIMESTAMP.yaml': ").strip()
    
    if user_prefix.endswith(".yaml"):
        user_prefix = user_prefix[:-5]
    elif user_prefix.endswith(".yml"):
        user_prefix = user_prefix[:-4]
    
    hand = MockOrcaHand(model_path=args.model_path)     
    success, message = hand.connect()
    if not success:
        print(f"Failed to connect: {message}")
        return 1
    
    hand.init_joints()
    time.sleep(1)
    hand.disable_torque()
    print("Torque disabled. Ready to record motor angles.")

    replay_buffer = []
    try:
        hand.record_waypoints(replay_buffer, input_fn=input, blocking=True)
    except KeyboardInterrupt:
        print("\nRecording interrupted by user.")

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename_core = f"{user_prefix}_replay_sequence_{timestamp}.yaml" if user_prefix else f"replay_sequence_{timestamp}.yaml"

    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    output_directory = os.path.abspath(args.output_dir) if args.output_dir else os.path.join(project_root, 'replay_sequences')
    os.makedirs(output_directory, exist_ok=True)

    output_filepath = os.path.join(output_directory, filename_core)
    update_yaml(output_filepath, "waypoints", replay_buffer)

    hand.disconnect()
    print(f"Replay sequence saved to {output_filepath} and disconnected from hand.")

if __name__ == "__main__":
    main()