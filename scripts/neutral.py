import argparse

from orca_core.utils.cli import (
    add_hand_arguments,
    connect_hand,
    create_fleet_from_args,
    quiet_uncalibrated_warnings,
    shutdown_hand,
)


def _run_one(hand, args) -> None:
    connect_hand(hand)
    try:
        # init_joints() calibrates when the hand isn't already, or when
        # forced — quiet the per-step re-warn that routine's own reads would
        # otherwise repeat; construction's one-time banner stays visible,
        # since here (unlike calibrate.py/tension.py) it explains why a
        # calibration sweep is about to run at all.
        with quiet_uncalibrated_warnings():
            hand.init_joints(force_calibrate=args.force_calibrate)
        print("Moving to neutral position...")
        hand.set_neutral_position()
        print("Reached neutral position.")
    finally:
        shutdown_hand(hand)


def main() -> int:
    parser = argparse.ArgumentParser(description="Move the hand to its neutral pose.")
    add_hand_arguments(parser, all_flag=True)
    parser.add_argument(
        "--force-calibrate",
        action="store_true",
        help="Run calibration even if calibration.yaml already exists.",
    )
    parser.add_argument(
        "--sequential",
        action="store_true",
        help="With --all, move one hand at a time instead of together.",
    )
    args = parser.parse_args()

    fleet = create_fleet_from_args(args)
    if len(fleet) == 1:
        _run_one(fleet.only(), args)
        return 0

    results = fleet.run(lambda hand: _run_one(hand, args), parallel=not args.sequential)
    failed = {hand_id: err for hand_id, err in results.items() if isinstance(err, Exception)}
    for hand_id, err in failed.items():
        print(f"[{hand_id}] FAILED: {err}")
    return 1 if failed else 0

if __name__ == "__main__":
    raise SystemExit(main())
