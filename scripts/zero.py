import argparse
from orca_core import OrcaHand
import time 
def main():
    parser = argparse.ArgumentParser(
        description="Move only the wrist joint (motor 17) to zero position."
    )
    parser.add_argument(
        "model_path",
        type=str,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1)"
    )
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    hand.enable_torque()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    
    # Get current motor position
    current_pos = hand.get_motor_pos()
    
    while True:
        new_pos = current_pos.copy()
        new_pos[-1] += 0.1
        hand._set_motor_pos(new_pos)
        time.sleep(0.001)
    
 
    

if __name__ == "__main__":
    main()