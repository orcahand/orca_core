import argparse
from orca_core import OrcaHand

def main():
    parser = argparse.ArgumentParser(
        description="Enable torque and hold tension on the ORCA Hand. "
                    "Specify the path to the orcahand model folder."
    )
    parser.add_argument(
        "model_path",
        type=str,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1_left)"
    )
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)
    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    hand.enable_torque()
    print("Torque enabled. Press Ctrl+C to exit.")

    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\nExiting. Disabling torque.")
        hand.disable_torque()

if __name__ == "__main__":
    main()