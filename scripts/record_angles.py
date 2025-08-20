import time
import yaml
import threading
from orca_core import OrcaHand
import argparse
import os  # Added import

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
    
    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return
    
    hand.init_joints()
    time.sleep(1
    )

    hand.disable_torque()
    print("Torque disabled. Ready to record motor angles.")

    replay_buffer = []
    print("Press Enter to capture a waypoint. Ctrl+C to quit.")
    
    def wait_for_enter(stop_flag):
        input()
        stop_flag.append(True)

    try:
        while True:
            stop_flag = []
            thread = threading.Thread(target=wait_for_enter, args=(stop_flag,), daemon=True)
            thread.start()

            while not stop_flag:
                current_angles = hand.get_joint_pos(as_list=True) 
                print("\rWaiting for input...", end="")
                time.sleep(0.1)

            print()  # newline after stopping
            current_angles = hand.get_joint_pos(as_list=True) 
            replay_buffer.append([float(angle) for angle in current_angles])  
            print(f"Captured waypoint: {current_angles}")

    except KeyboardInterrupt:
        print("\nRecording interrupted.")

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    
    if user_prefix:
        filename_core = f"{user_prefix}_replay_sequence_{timestamp}.yaml"
    else:
        filename_core = f"replay_sequence_{timestamp}.yaml"

    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, '..'))

    if args.output_dir:
        output_directory = os.path.abspath(args.output_dir)
    else:
        output_directory = os.path.join(project_root, 'replay_sequences')
    
    os.makedirs(output_directory, exist_ok=True) 

    output_filepath = os.path.join(output_directory, filename_core)

    with open(output_filepath, "w") as file:
        yaml.dump({"waypoints": replay_buffer}, file)

    print(f"Replay sequence saved to {output_filepath}")

if __name__ == "__main__":
    main()