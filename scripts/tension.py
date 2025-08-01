import argparse
from orca_core import OrcaHand
import time
import numpy as np

def main():
    parser = argparse.ArgumentParser(
        description="Enable torque and hold tension on the ORCA Hand. "
                    "Specify the path to the orcahand model folder."
    )
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    parser.add_argument('--move_motors', action='store_true', help='If set, move motors 1-16 continuously positively for 3 seconds with calibration current.')

    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    hand.enable_torque()

    hand.tension(args.move_motors)
    
    hand.disconnect()

if __name__ == "__main__":
    main()