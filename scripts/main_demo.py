import argparse

from common import add_hand_arguments, connect_hand, create_hand, shutdown_hand
from orca_core.constants import STEPS_TO_NEUTRAL


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a simple open-close-pinch demo using the current hand config."
    )
    add_hand_arguments(parser)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--num-steps", type=int, default=8)
    parser.add_argument("--step-size", type=float, default=0.02)
    args = parser.parse_args()

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        hand.init_joints(False)

        print("Cycling through open_hand -> power_grasp -> pinch -> neutral")
        hand.run_demo(
            "main",
            cycles=args.cycles,
            num_steps=250
        )

        return 0
    except KeyboardInterrupt:
        print("\nDemo interrupted.")
        return 0
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
