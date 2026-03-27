import argparse
from orca_core import OrcaHand

def main():
    parser = argparse.ArgumentParser(
        description="Calibrate the ORCA Hand. Specify the path to the hand config.yaml file."
    )
    parser.add_argument(
        "config_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the hand config.yaml file (e.g., /path/to/orcahand_v1/config.yaml)"
    )
    parser.add_argument(
        "--force-wrist",
        action="store_true",
        help="Force wrist calibration even if already calibrated"
    )
    args = parser.parse_args()

    hand = OrcaHand(config_path=args.config_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    hand.calibrate(force_wrist=args.force_wrist)

if __name__ == "__main__":
    main()