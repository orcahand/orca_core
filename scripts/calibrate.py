

import argparse

from common import add_hand_arguments, connect_hand, create_hand, shutdown_hand


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the ORCA hand calibration routine."
    )
    add_hand_arguments(parser)
    parser.add_argument(
        "--force-wrist",
        action="store_true",
        help="Recalibrate the wrist even if it is already marked as calibrated.",
    )
    args = parser.parse_args()

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        print("Starting calibration...")
        hand.calibrate(force_wrist=args.force_wrist)
        print("Calibration complete.")
        return 0
    finally:
        shutdown_hand(hand)

if __name__ == "__main__":
    raise SystemExit(main())
