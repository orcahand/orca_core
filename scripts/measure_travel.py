"""Measure each joint's motor travel and store it as the calibration baseline.

Drives every joint onto both of its hardstops, records how far the driving
motor turned between them, and writes the result to ``config.yaml`` under
``joint_motor_travel:``. The calibration routine then compares each fresh
measurement against that baseline and re-drives any joint that falls short at
a higher current — see ``orca_core/maintenance/calibration_routine.py``.

Run this on a freshly tensioned hand: the baseline records what the joint's
tendon spool geometry allows, so it must be taken when nothing is binding.
"""

import argparse
import dataclasses
import sys

from orca_core.maintenance.motor_travel import (
    measured_travel_by_joint,
    travel_deviation,
    write_joint_motor_travel,
)
from orca_core.utils.cli import (
    add_hand_arguments,
    create_hand_from_args,
    print_calibration_progress,
    shutdown_hand,
)


def _print_table(hand, measured: dict[str, float]) -> None:
    """Print measured travel per joint, against any baseline already stored."""
    print()
    print(f"{'joint':<14}{'motor':>6}{'travel deg':>12}{'baseline':>10}{'delta':>9}")
    print("-" * 51)
    for joint in hand.config.joint_ids:
        if joint not in measured:
            continue
        motor_id = hand.config.joint_to_motor_map[joint]
        travel = measured[joint]
        baseline = hand.config.expected_motor_travel_deg(joint)
        if baseline is None:
            print(f"{joint:<14}{motor_id:>6}{travel:>12.2f}{'-':>10}{'-':>9}")
        else:
            delta = travel_deviation(travel, baseline) * 100
            print(
                f"{joint:<14}{motor_id:>6}{travel:>12.2f}"
                f"{baseline:>10.2f}{delta:>8.0f}%"
            )
    print()


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Measure per-joint motor travel and write it to config.yaml as the "
            "joint_motor_travel baseline."
        )
    )
    add_hand_arguments(parser, feedback_flag=False)
    parser.add_argument(
        "--from-calibration",
        action="store_true",
        help="Derive the baseline from the stored calibration.yaml without "
        "moving the hand. Only trustworthy if that calibration was clean.",
    )
    parser.add_argument(
        "--current",
        type=int,
        default=None,
        help="Calibration current (mA) for the sweep. Raise it above the "
        "config value so a stiff joint still reaches its hardstop.",
    )
    parser.add_argument(
        "--force-wrist",
        action="store_true",
        help="Include the wrist even if it is already calibrated.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the measured travel without writing config.yaml.",
    )
    parser.add_argument(
        "--replace",
        action="store_true",
        help="Replace the stored baseline instead of updating only the joints "
        "measured in this run.",
    )
    args = parser.parse_args()

    hand = create_hand_from_args(args, engage_feedback=False, engage_sensors=False)

    # Deriving from the stored calibration moves nothing, so it must not need
    # the motor bus - it is the mode to reach for when the hand is unplugged.
    if not args.from_calibration:
        success, message = hand.connect(interactive=False)
        print(f"connect() -> success={success}, message={message}")
        if not success:
            print("Failed to connect to the hand.")
            sys.exit(1)

    try:
        if not args.from_calibration:
            if args.current is not None:
                hand.config = dataclasses.replace(
                    hand.config,
                    calibration_current=args.current,
                    max_current=max(args.current, hand.config.max_current),
                )
                print(f"Sweeping at {args.current} mA.")
            print("Driving every joint onto both hardstops...")
            hand.calibrate(
                force_wrist=args.force_wrist,
                progress_callback=print_calibration_progress,
            )

        measured = measured_travel_by_joint(hand.config, hand.calibration)
        if not measured:
            print("No joint has a complete motor-limit pair; nothing to record.")
            sys.exit(1)

        _print_table(hand, measured)

        if args.dry_run:
            print("Dry run: config.yaml not written.")
            return

        stored = write_joint_motor_travel(
            hand.config.config_path, measured, merge=not args.replace
        )
        print(
            f"Wrote joint_motor_travel for {len(stored)} joint(s) to "
            f"{hand.config.config_path}"
        )
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if not args.from_calibration:
            shutdown_hand(hand)


if __name__ == "__main__":
    main()
