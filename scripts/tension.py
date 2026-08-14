

import argparse

from orca_core.utils.cli import (
    add_hand_arguments,
    connect_hand,
    create_fleet_from_args,
    quiet_uncalibrated_warnings,
    shutdown_hand,
)


def _print_progress(event: dict) -> None:
    """Render tension progress events on the terminal."""
    phase = event.get("phase") if event.get("event") == "phase" else None
    if phase == "winding":
        print("Winding tendons taut...")
    elif phase == "holding":
        print("Holding motors for manual tensioning — press Ctrl+C when done.")


def _run_one(hand, args) -> None:
    connect_hand(hand)
    try:
        # A hand tensioning always needs (re)calibrating afterwards anyway,
        # so its current calibration state is not worth reporting mid-run.
        with quiet_uncalibrated_warnings():
            hand.tension(move_motors=args.move_motors, progress_callback=_print_progress)
    except KeyboardInterrupt:
        print("\nTension interrupted.")
    finally:
        shutdown_hand(hand)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Hold the hand under tension for manual tendon setup."
    )
    add_hand_arguments(parser, feedback_flag=False, all_flag=True)
    parser.add_argument(
        "--move-motors",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Run the built-in preconditioning motion before holding tension (default: True).",
    )
    args = parser.parse_args()

    # tension() refuses to run against a live joint loop; connect open-loop.
    # quiet=True drops construction's not-calibrated banner: tensioning
    # always needs a recalibration afterwards regardless of where it started.
    fleet = create_fleet_from_args(args, quiet=True, engage_feedback=False)
    if len(fleet) == 1:
        _run_one(fleet.only(), args)
        return 0

    # Always sequential, regardless of the fleet size: tension()'s hold blocks
    # until Ctrl+C, and Ctrl+C only ever reaches the main thread — a worker
    # thread holding torque would never see it and never release. One hand
    # at a time keeps that Ctrl+C in the thread that actually receives it.
    print(f"Tensioning {len(fleet)} hands one at a time — Ctrl+C moves on to the next.")
    results = fleet.run(lambda hand: _run_one(hand, args), parallel=False)
    failed = {hand_id: err for hand_id, err in results.items() if isinstance(err, Exception)}
    for hand_id, err in failed.items():
        print(f"[{hand_id}] FAILED: {err}")
    return 1 if failed else 0

if __name__ == "__main__":
    raise SystemExit(main())
