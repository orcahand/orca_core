import argparse

from common import add_hand_arguments, connect_hand, create_hand, shutdown_hand


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a demo focused on finger abduction and spread patterns."
    )
    add_hand_arguments(parser)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--num-steps", type=int, default=8)
    parser.add_argument("--step-size", type=float, default=0.02)
    args = parser.parse_args()

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        hand.init_joints(force_calibrate=args.mock)

        print("Cycling through fan_out -> fan_in -> spread_grasp -> neutral")
        hand.run_demo(
            "abduction",
            cycles=args.cycles,
            num_steps=args.num_steps,
            step_size=args.step_size,
        )
        return 0
    except KeyboardInterrupt:
        print("\nDemo interrupted.")
        return 0
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
