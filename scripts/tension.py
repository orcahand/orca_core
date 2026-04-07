

import argparse

from common import add_hand_arguments, connect_hand, create_hand, shutdown_hand


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Hold the hand under tension for manual tendon setup."
    )
    add_hand_arguments(parser)
    parser.add_argument(
        "--move-motors",
        action="store_true",
        help="Run the built-in preconditioning motion before holding tension.",
    )
    args = parser.parse_args()

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        hand.tension(move_motors=args.move_motors)
        return 0
    except KeyboardInterrupt:
        print("\nTension interrupted.")
        return 0
    finally:
        shutdown_hand(hand)

if __name__ == "__main__":
    raise SystemExit(main())
