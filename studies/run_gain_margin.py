"""Raise one joint's loop gain until it oscillates, and say where that was.

Attended: the hand is driven deliberately toward instability, one joint at a
time. Ctrl-C returns the joint being probed to its base pose and stops after the
current dwell.
"""

import argparse
import json
import signal
import sys
import time
from pathlib import Path

from orca_core.hand_factory import load_hand

from studies.experiments.gain_margin import ARMS, run_gain_margin
from studies.preconditions import PreconditionError
from studies.record import FrameRecorder, JointAngleWindow, Ritual, record_frames


DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "data"


def on_progress(event):
    kind = event["event"]
    if kind == "run_started":
        print(f"Ramping {event['arm']} on: {', '.join(event['joints'])}")
    elif kind == "joint_started":
        print(
            f"\n{event['joint']}: base {event['base_pose_deg']}°, "
            f"excite ±{event['amplitude_deg']}°, from Kp={event['kp']} Ki={event['ki']}"
        )
    elif kind == "baseline_measured":
        print(
            f"  baseline swing {event['p2p_deg']}°, frame gap p99 "
            f"{event['frame_gap_p99_ms']} ms"
        )
    elif kind == "dwell_done":
        measured = event["measured"] or {}
        verdict = event["verdict"] or {}
        state = verdict.get("state", "baseline")
        if event["aborted"]:
            state += f" (cut short: {event['aborted']})"
        frequency = measured.get("frequency_hz")
        print(
            f"  Kp={event['kp']:<6} Ki={event['ki']:<6} "
            f"swing {measured.get('peak_to_peak_deg', float('nan')):.3f}° "
            f"at {frequency if frequency is not None else '—'} Hz, "
            f"clamped {event['clamp_fraction'] * 100:.0f}% -> {state}"
        )
    elif kind == "onset":
        low, high = event["bracket"]
        print(
            f"  ONSET at {event['arm']}={event['value']}, oscillating at "
            f"{event['frequency_hz']} Hz (last quiet at {low if low is not None else '—'})"
        )
    elif kind == "joint_done":
        print(f"  {event['joint']}: {event['outcome']} — {event['reason']}")


def report(result):
    print()
    print(f"{'joint':<12} {'outcome':<10} {'onset in':>14} {'Hz':>7} {'baseline':>9}")
    for margin in result["joints"]:
        bracket = margin["onset_bracket"]
        span = (
            f"{bracket[0]}–{bracket[1]}" if bracket and bracket[0] is not None
            else (str(margin["onset_value"]) if margin["onset_value"] is not None else "—")
        )
        frequency = margin["onset_frequency_hz"]
        print(
            f"{margin['joint']:<12} {margin['outcome']:<10} {span:>14} "
            f"{frequency if frequency is not None else '—':>7} "
            f"{margin['baseline_p2p_deg'] if margin['baseline_p2p_deg'] is not None else '—':>9}"
        )

    summary_path = Path(result["directory"]) / "summary.json"
    summary_path.write_text(json.dumps(result, indent=2, default=str))
    print(f"\nSummary written to {summary_path}")


def ritual_from(args) -> Ritual:
    """What was done to the tendons before this run, as timestamps."""
    now = time.time()

    def stamp(minutes):
        return None if minutes is None else now - minutes * 60.0

    if args.no_ritual_history:
        return Ritual(notes="operator declared no tendon history")
    return Ritual(
        tensioned_at=stamp(args.tensioned_minutes_ago),
        jittered_at=stamp(args.jittered_minutes_ago),
        calibrated_at=stamp(args.calibrated_minutes_ago),
        notes=args.notes,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "config_path",
        nargs="?",
        default=None,
        help="Path to config.yaml. Autodetected from the connected hand when omitted.",
    )
    parser.add_argument(
        "--joints",
        nargs="+",
        default=None,
        help="Joints to probe. Defaults to every joint the loop controls.",
    )
    parser.add_argument(
        "--arm",
        choices=ARMS,
        default="kp",
        help="'kp' ramps proportional gain with the integrator off; 'ki' ramps "
        "integral gain at the configured proportional gain.",
    )
    parser.add_argument(
        "--max-current",
        type=int,
        default=None,
        help="Motor current ceiling (mA) for the run; restored afterwards.",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--tensioned-minutes-ago", type=float, default=None)
    parser.add_argument("--jittered-minutes-ago", type=float, default=None)
    parser.add_argument("--calibrated-minutes-ago", type=float, default=None)
    parser.add_argument("--notes", default="", help="Anything else about the hand's state.")
    parser.add_argument(
        "--no-ritual-history",
        action="store_true",
        help="Record that no tendon history was declared, and run anyway.",
    )
    parser.add_argument("--yes", action="store_true", help="Skip the confirmation.")
    args = parser.parse_args()

    if not args.yes:
        print(
            "This drives one joint at a time past the gain its loop is stable "
            "at. Stay with the hand and be ready to cut power."
        )
        if input("Continue? [y/N] ").strip().lower() not in ("y", "yes"):
            return 1

    stopping = False

    def should_stop():
        return stopping

    def on_signal(signum, frame):
        nonlocal stopping
        stopping = True
        print("\nStopping after this dwell...")

    def watch_for_stop():
        """Both signals, because a joint left at a raised gain is not something
        to leave behind on the way out."""
        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)

    def stop_watching():
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)

    hand = load_hand(args.config_path)
    frames = FrameRecorder()
    window = JointAngleWindow.for_hand(hand)
    # Before connect: the encoder client is built there and never replaced.
    record_frames(hand, frames, window)

    connected, message = hand.connect()
    print(f"connect() -> {connected}: {message}")
    if not connected:
        return 1

    # Connecting starts the loop but leaves the motors limp, and a limp joint
    # measures nothing. Torque stays on afterwards: cutting it drops the
    # fingers onto whatever is under them.
    failed = hand.enable_torque()
    if failed:
        print(f"Motors that would not take torque: {failed}")
        hand.disconnect()
        return 1
    hand.rebase_loop()
    print(f"Loop joints: {', '.join(hand.loop_joint_names or [])}")

    watch_for_stop()
    try:
        result = run_gain_margin(
            hand,
            window,
            output_dir=args.output_dir,
            ritual=ritual_from(args),
            joints=args.joints,
            arm=args.arm,
            frame_recorder=frames,
            max_current_ma=args.max_current,
            progress_callback=on_progress,
            should_stop=should_stop,
        )
    except PreconditionError as error:
        print(f"\nCannot start: {error}")
        return 1
    finally:
        stop_watching()
        hand.disconnect()

    report(result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
