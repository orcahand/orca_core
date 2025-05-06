import os
import time
import yaml
import threading
from orca_core import OrcaHand

def main():
    
    filename = input("Enter the filename to save the replay sequence (default: replay_sequence.yaml): ")
    
    hand = OrcaHand('/Users/ccc/dev/orca/orca_core/orca_core/models/orcahand_v1')
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return
    
    hand.init_joints()
    time.sleep(1)

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
                print(f"\rWaiting for input...", end="")
                time.sleep(0.1)

            print()  # newline after stopping
            current_angles = hand.get_joint_pos(as_list=True) 
            replay_buffer.append([float(angle) for angle in current_angles])  
            print(f"Captured waypoint: {current_angles}")

    except KeyboardInterrupt:
        print("\nRecording interrupted.")

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = filename + f"_replay_sequence_{timestamp}.yaml"
    with open(filename, "w") as file:
        yaml.dump({"waypoints": replay_buffer}, file)

    print(f"Replay sequence saved to {filename}")

if __name__ == "__main__":
    main()