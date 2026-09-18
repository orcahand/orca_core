import argparse

from orca_core.utils.cli import add_hand_arguments, connect_hand, create_hand_from_args, shutdown_hand

from demo_runner import run_demo


def main() -> int:
    """Play the packaged "abduction" demo sequence (fan out -> fan in -> spread grasp -> neutral).

    See :meth:`~orca_core.base_hand.BaseHand.set_joint_positions` for the exact behaviour of
    ``--num-steps`` and ``--step-size``.

    Returns the process exit code: 0 after a full run or a Ctrl-C interrupt.
    """
    parser = argparse.ArgumentParser(
        description="Run a demo focused on finger abduction and spread patterns."
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
        help="Interpolation steps per pose transition; higher is smoother and slower. "
        "num_steps and step_size define the duration of each transition. Default: 8.",
    )
    parser.add_argument(
        "--step-size",
        type=float,
        default=0.02,
        help="Pause between interpolation steps. Default: 0.02 s.",
    )
    args = parser.parse_args()

    hand = create_hand_from_args(args)

    if not hand.calibrated:
        print("Hand not calibrated. Running calibration...")

    try:
        connect_hand(hand)
        hand.init_joints(force_calibrate=args.mock)

        print("Cycling through fan_out -> fan_in -> spread_grasp -> neutral")
        run_demo(
            hand,
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
