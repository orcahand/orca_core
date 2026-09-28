import argparse

from orca_core.utils.cli import (
    add_hand_arguments,
    connect_hand,
    create_hand_from_args,
    print_calibration_progress,
    shutdown_hand,
)

from demo_runner import run_demo


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a simple open-close-pinch demo using the current hand config."
    )
    add_hand_arguments(parser)
    parser.add_argument(
        "--cycles", type=int, default=3,
        help="Times the pose sequence is repeated.",
    )
    parser.add_argument(
        "--num-steps", type=int, default=8,
        help="Interpolation steps per pose transition; higher is smoother and slower.",
    )
    parser.add_argument(
        "--step-size", type=float, default=0.02,
        help="Seconds to pause between interpolation steps.",
    )
    args = parser.parse_args()

    hand = create_hand_from_args(args)
    try:
        connect_hand(hand)
        hand.init_joints(progress_callback=print_calibration_progress)

        print("Cycling through open_hand -> power_grasp -> pinch -> neutral")
        run_demo(
            hand,
            "main",
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
