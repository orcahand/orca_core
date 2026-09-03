

import argparse
import gc
import time

import numpy as np
import yaml

from orca_core.control.control_auxiliary import DEFAULT_TRACE_HZ, LoopTracer, PacingMonitor
from orca_core.utils.cli import (
    add_hand_arguments,
    connect_hand,
    create_hand_from_args,
    pose_without,
    KeyListener,
    print_pacing_report,
    print_trace_report,
    resolve_input_path,
    shutdown_hand,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay a continuous joint recording.")
    add_hand_arguments(parser)
    parser.add_argument("--replay-file", type=str, required=True)
    parser.add_argument(
        "--skip-joints", nargs="+", default=(),
        help=(
            "Joints to leave uncommanded — they hold their pose. Use to replay "
            "a whole-hand recording on a hand with a joint out of action."
        ),
    )
    parser.add_argument(
        "--pacing-csv", default=None,
        help=(
            "Record when every command went out and how long it took, to this "
            "CSV. Use when the motion stutters: it separates a command that "
            "blocked from one the host never issued on time."
        ),
    )
    parser.add_argument(
        "--trace-csv", default=None,
        help=(
            "Record measured joint angles, the PI correction and the loop "
            "counters at loop rate, to this CSV. Answers what the loop was "
            "doing when the motion stuttered."
        ),
    )
    parser.add_argument(
        "--trace-hz", type=float, default=DEFAULT_TRACE_HZ,
        help=f"Trace sampling rate (default {DEFAULT_TRACE_HZ}).",
    )
    parser.add_argument(
        "--trace-temps", type=float, default=0.0,
        help=(
            "Also sample motor temperature at this rate in Hz. Costs a bus "
            "read per tick, so keep it at 1 or below. 0 (default) is off."
        ),
    )
    parser.add_argument(
        "--freeze-gc", action="store_true",
        help=(
            "Freeze and disable the garbage collector for the replay. If the "
            "stutter disappears, GC pauses were the cause."
        ),
    )
    args = parser.parse_args()

    if args.freeze_gc:
        gc.freeze()
        gc.disable()
        print("Garbage collector frozen and disabled for this replay.")

    replay_path = resolve_input_path(args.replay_file)
    try:
        replay_data = yaml.safe_load(replay_path.read_text(encoding="utf-8")) or {}
    except FileNotFoundError:
        print(f"Replay file not found: {replay_path}")
        return 1

    metadata = replay_data.get("metadata", {})
    if metadata.get("type") != "continuous":
        print("Replay file is not a continuous recording.")
        return 1

    sampling_frequency = metadata.get("sampling_frequency_hz")
    if sampling_frequency is None:
        print("Replay file is missing sampling_frequency_hz.")
        return 1

    waypoints = replay_data.get("angles", [])
    if not waypoints:
        print("Replay file does not contain any recorded frames.")
        return 1

    hand = create_hand_from_args(args)
    monitor = None
    tracer = None
    keys = None
    try:
        connect_hand(hand)
        hand.init_joints()

        expected_joint_ids = metadata.get("joint_ids")
        if expected_joint_ids is not None and expected_joint_ids != hand.config.joint_ids:
            raise ValueError("Replay joint order does not match the connected hand configuration.")

        if metadata.get("hand_type") not in (None, hand.config.type):
            raise ValueError(
                f"Replay was recorded for hand_type={metadata['hand_type']}, "
                f"but the connected config is {hand.config.type}."
            )

        print(f"Replaying {len(waypoints)} frames from {replay_path}")
        step_time = 1.0 / sampling_frequency
        monitor = PacingMonitor(target_period_s=step_time, hand=hand)
        if args.trace_csv:
            tracer = LoopTracer(
                hand, rate_hz=args.trace_hz, temp_hz=args.trace_temps
            )
            tracer.start()

            def mark_observation(key: str) -> None:
                if key == " ":
                    print(f"  marked at t={tracer.mark('stutter'):.2f} s")

            keys = KeyListener(mark_observation).start()
            if keys.listening:
                print("Press SPACE whenever you see a stutter to mark the trace.")
        start_time = time.monotonic()
        for index, pose in enumerate(waypoints):
            with monitor.command():
                hand.set_joint_positions(
                    pose_without(hand, pose, args.skip_joints)
                )
            target_time = start_time + index * step_time
            remaining = target_time - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        return 0
    except KeyboardInterrupt:
        print("\nReplay interrupted.")
        return 0
    finally:
        if keys is not None:
            keys.stop()
        if tracer is not None:
            tracer.stop()
            traced = tracer.write_csv(args.trace_csv) if args.trace_csv else None
            print_trace_report(tracer.summary(), traced)
        if monitor is not None:
            written = (
                monitor.write_csv(args.pacing_csv)
                if args.pacing_csv else None
            )
            print_pacing_report(monitor.stats(), written)
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
