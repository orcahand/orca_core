#!/usr/bin/env python3
import argparse

from common import add_hand_arguments, connect_hand, create_hand, shutdown_hand


def main() -> int:
    parser = argparse.ArgumentParser(description="Move the hand to its neutral pose.")
    add_hand_arguments(parser)
    parser.add_argument(
        "--force-calibrate",
        action="store_true",
        help="Run calibration even if calibration.yaml already exists.",
    )
    args = parser.parse_args()

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        hand.init_joints(force_calibrate=args.force_calibrate or args.mock)
        print("Moving to neutral position...")
        hand.set_neutral_position()
        print("Reached neutral position.")
        return 0
    finally:
        shutdown_hand(hand)

if __name__ == "__main__":
    raise SystemExit(main())
