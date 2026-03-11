import argparse
from orca_core import OrcaHand

FINGER_TO_JOINTS = {
    "thumb": ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_pip", "thumb_dip"],
    "index": ["index_abd", "index_mcp", "index_pip"],
    "middle": ["middle_abd", "middle_mcp", "middle_pip"],
    "ring": ["ring_abd", "ring_mcp", "ring_pip"],
    "pinky": ["pinky_abd", "pinky_mcp", "pinky_pip"],
    "wrist": ["wrist"],
}

ALL_JOINTS = [j for joints in FINGER_TO_JOINTS.values() for j in joints]

def main():
    parser = argparse.ArgumentParser(
        description="Calibrate the ORCA Hand. Specify the path to the orcahand model folder."
    )
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1)"
    )
    parser.add_argument(
        "--force-wrist",
        action="store_true",
        help="Force wrist calibration even if already calibrated"
    )
    parser.add_argument(
        "--fingers",
        type=str,
        nargs="+",
        choices=list(FINGER_TO_JOINTS.keys()),
        help="Fingers to calibrate (e.g., --fingers thumb index pinky)"
    )
    parser.add_argument(
        "--joints",
        type=str,
        nargs="+",
        choices=ALL_JOINTS,
        help="Individual joints to calibrate (e.g., --joints thumb_cmc index_mcp)"
    )
    args = parser.parse_args()

    if args.fingers and args.joints:
        parser.error("Cannot specify both --fingers and --joints. Use one or the other.")

    # Resolve which joints to calibrate
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

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    hand.calibrate(force_wrist=args.force_wrist, joints=joints)

if __name__ == "__main__":
    main()