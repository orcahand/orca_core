

import argparse

from common import add_hand_arguments, connect_hand, create_hand, shutdown_hand
from orca_core.constants import NUM_STEPS, STEP_SIZE
    

def main() -> int:
    parser = argparse.ArgumentParser(description="Move all joints to zero.")
    add_hand_arguments(parser)
    parser.add_argument(
        "--force-calibrate",
        action="store_true",
        help="Run calibration even if calibration.yaml already exists.",
    )
    parser.add_argument("--num-steps", type=int, default=NUM_STEPS)
    parser.add_argument("--step-size", type=float, default=STEP_SIZE)
    args = parser.parse_args()

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        hand.init_joints(
            force_calibrate=args.force_calibrate or args.mock,
            move_to_neutral=False,
        )
        print("Moving all joints to zero...")
        hand.set_zero_position(num_steps=args.num_steps, step_size=args.step_size)
        print("Reached zero position.")
        return 0
    finally:
        shutdown_hand(hand)

if __name__ == "__main__":
    raise SystemExit(main())
