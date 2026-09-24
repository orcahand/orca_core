import argparse

from orca_core.utils.cli import add_hand_arguments, connect_hand, create_hand_from_args, shutdown_hand

from demo_runner import run_demo


def main() -> int:
    """Play the packaged "main" demo sequence (open -> power grasp -> pinch -> neutral).

    See :meth:`~orca_core.base_hand.BaseHand.set_joint_positions` for the exact behaviour of
    ``--num-steps`` and ``--step-size``.

    Returns the process exit code: 0 after a full run or a Ctrl-C interrupt.
    """
    parser = argparse.ArgumentParser(
        description="Run a simple open-close-pinch demo using the current hand config."
    )
    add_hand_arguments(parser)
    parser.add_argument(
        "--cycles",
        type=int,
        default=3,
        help="Number of times the pose sequence is repeated. Default: 3.",
    )
    parser.add_argument(
        "--num-steps",
        type=int,
        default=8,
        help="Interpolation steps per pose transition; higher is smoother and slower. Default: 8.",
    )
    parser.add_argument(
        "--step-size",
        type=float,
        default=0.02,
        help="Seconds to pause between interpolation steps. Default: 0.02.",
    )
    args = parser.parse_args()

    hand = create_hand_from_args(args)

    if not hand.calibrated:
        print("Hand not calibrated. Running calibration...")

    try:
        connect_hand(hand)
        hand.init_joints(force_calibrate=args.mock)

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
