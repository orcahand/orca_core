

import argparse

from common import add_hand_arguments, connect_hand, create_hand, shutdown_hand


FINGER_TO_JOINTS = {
    "thumb": ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"],
    "index": ["index_abd", "index_mcp", "index_pip"],
    "middle": ["middle_abd", "middle_mcp", "middle_pip"],
    "ring": ["ring_abd", "ring_mcp", "ring_pip"],
    "pinky": ["pinky_abd", "pinky_mcp", "pinky_pip"],
    "wrist": ["wrist"],
}

ALL_JOINTS = [j for joints in FINGER_TO_JOINTS.values() for j in joints]


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
    parser.add_argument(
        "--fingers",
        type=str,
        nargs="+",
        choices=list(FINGER_TO_JOINTS.keys()),
        help="Fingers to calibrate (e.g., --fingers thumb index pinky)",
    )
    parser.add_argument(
        "--joints",
        type=str,
        nargs="+",
        choices=ALL_JOINTS,
        help="Individual joints to calibrate (e.g., --joints thumb_cmc index_mcp)",
    )
    args = parser.parse_args()

    if args.fingers and args.joints:
        parser.error("Cannot specify both --fingers and --joints. Use either one.")

    joints = None
    if args.fingers:
        joints = []
        for finger in args.fingers:
            joints.extend(FINGER_TO_JOINTS[finger])
        print(f"Calibrating fingers: {args.fingers}")
        print(f"Resolved joints: {joints}")
    elif args.joints:
        joints = args.joints
        print(f"Calibrating joints: {joints}")

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        print("Starting calibration...")
        hand.calibrate(force_wrist=args.force_wrist, joints=joints)
        print("Calibration complete.")
        return 0
    finally:
        shutdown_hand(hand)

if __name__ == "__main__":
    raise SystemExit(main())
