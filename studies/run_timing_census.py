"""Record the encoder stream and print how it was delivered.

Listens only: nothing is commanded and torque is never enabled.
"""

import argparse
import json
import sys
from pathlib import Path

from studies.analysis import timing
from studies.experiments.timing_census import (
    DEFAULT_DURATION_S,
    run_timing_census,
)
from studies.preconditions import PreconditionError
from studies.record import Dataset


DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "data"


def on_progress(event):
    kind = event["event"]
    if kind == "port_resolved":
        shared = " (tactile shares it)" if event["shared"] else ""
        print(f"Encoder port: {event['port']}{shared}")
    elif kind == "board_identified":
        print(f"Board: {event['board'] or 'did not identify itself'}")
    elif kind == "health_checked":
        print(f"Encoder slots healthy over {event['frames']} frames.")
        if event["constant_slots"]:
            print(f"  Not moving (expected at rest): {', '.join(event['constant_slots'])}")
    elif kind == "tactile_started":
        print(f"Tactile streaming on the same link: mode={event['mode']}")
    elif kind == "recording":
        print(f"Recording to {event['directory']}")
    elif kind == "progress":
        print(
            f"  {event['elapsed_s']:6.1f}s elapsed, {event['remaining_s']:6.1f}s left, "
            f"{event['frames']} frames, {event['rate_hz']} Hz"
        )
    elif kind == "census_done":
        print(
            f"Done: {event['frames']} frames, {event['dropped']} dropped by the "
            f"recorder, {event['stale']} arrivals the decoder rejected."
        )
    elif kind == "census_aborted":
        print(f"Aborted: {event['error']}")


def report(directory):
    dataset = Dataset(directory)
    summary = timing.summarise(dataset.table("frames"))

    print()
    print(f"Frames            {summary['frames']}")
    print(f"Rate              {summary['rate_hz']:.1f} Hz")

    gaps = summary["inter_arrival"]
    print()
    print("Gap between arrivals (ms)")
    for key in ("p50_ms", "p90_ms", "p99_ms", "p999_ms", "max_ms"):
        print(f"  {key[:-3]:<6} {gaps[key]:8.3f}")

    print()
    print("Gap as a multiple of the board's emit period")
    for multiple, count in sorted(summary["period_multiples"].items()):
        share = 100.0 * count / max(1, gaps["count"])
        print(f"  {multiple}x  {count:8d}  {share:6.2f}%")
    print(f"  not on a multiple: {summary['off_period_fraction'] * 100:.2f}%")

    after = summary["after_a_long_gap"]
    print()
    if after.get("n_late"):
        print(
            f"After a long gap ({after['n_late']} of them): next gap averages "
            f"{after['following_mean_ms']:.3f} ms -> reads as {after['reads_as']}"
        )
    else:
        print("No long gaps.")

    print()
    print("Loop watchdog tiers a running loop would have crossed")
    for tier, count in summary["watchdog_exposure"].items():
        print(f"  {tier:<10} {count}")

    faults = {
        name: slot
        for name, slot in summary["slots"].items()
        if slot["parity_bad"] or slot["angle_error"] or slot["impossible_jumps"]
    }
    print()
    if faults:
        print("Slots reporting faults")
        for name, slot in faults.items():
            print(
                f"  {name} ({slot['joint']}): parity {slot['parity_bad']}, "
                f"angle-error {slot['angle_error']}, jumps {slot['impossible_jumps']}"
            )
    else:
        print("All slots clean.")

    recorder = summary["recorder"]
    if recorder.get("missing"):
        print(f"\nThe recorder itself lost {recorder['missing']} rows.")

    summary_path = Path(directory) / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, default=str))
    print(f"\nSummary written to {summary_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None, help="Encoder port; discovered if omitted.")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument(
        "--label", default="loop_off", help="Recorded in the manifest to tell runs apart."
    )
    parser.add_argument(
        "--tactile",
        choices=("off", "resultant", "taxels"),
        default="off",
        help="Also stream tactile on the same link and measure the encoder "
        "stream under it. 'resultant' is what a hand runs by default; "
        "'taxels' is the heaviest payload the link carries.",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--skip-health-check",
        action="store_true",
        help="Record even while a slot reports a fault.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=None,
        help="Skip recording and summarise an existing dataset directory.",
    )
    args = parser.parse_args()

    if args.report is not None:
        report(args.report)
        return 0

    try:
        directory = run_timing_census(
            output_dir=args.output_dir,
            port=args.port,
            duration_s=args.duration,
            label=args.label,
            tactile_mode=args.tactile,
            check_health=not args.skip_health_check,
            progress_callback=on_progress,
        )
    except PreconditionError as error:
        print(f"\nCannot start: {error}")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130

    if directory is None:
        print("Nothing recorded.")
        return 1
    report(directory)
    return 0


if __name__ == "__main__":
    sys.exit(main())
