

import argparse
import time
from pathlib import Path

import yaml

from orca_core.utils.cli import (
    add_hand_arguments,
    connect_hand,
    create_hand_from_args,
    prepare_output_dir,
    shutdown_hand,
)

# Fraction of dropped frames above which the recording is not worth saving,
# and how far the achieved rate may fall short of the requested one silently.
MAX_STALE_FRACTION = 0.1
RATE_TOLERANCE = 0.05


def _build_output_path(output_dir: Path, prefix: str) -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    stem = f"{prefix}_continuous_angles_{timestamp}" if prefix else f"continuous_angles_{timestamp}"
    return output_dir / f"{stem}.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Continuously record joint angles while manually moving the hand."
    )
    add_hand_arguments(parser, feedback_flag=False)
    parser.add_argument("--frequency", type=float, default=50.0)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--output-dir", type=str, default=None)
    parser.add_argument(
        "--force-calibrate",
        action="store_true",
        help="Run calibration even if calibration.yaml already exists.",
    )
    args = parser.parse_args()

    output_dir = prepare_output_dir(args.output_dir)
    prefix = input("Enter an optional filename prefix. Press Enter to skip: ").strip()
    output_path = _build_output_path(output_dir, prefix)
    interval = 1.0 / args.frequency

    # Recording backdrives the hand with torque off, which a running joint
    # loop would fight and wind its integrator up against.
    hand = create_hand_from_args(args, engage_feedback=False)
    data = {
        "metadata": {
            "type": "continuous",
            "created_at": time.strftime("%Y%m%d_%H%M%S"),
            "requested_frequency_hz": args.frequency,
            "sampling_frequency_hz": args.frequency,
        },
        "angles": [],
    }
    stale_frames = 0
    start_time = None
    elapsed = 0.0

    try:
        connect_hand(hand)
        hand.init_joints(force_calibrate=args.force_calibrate)
        hand.disable_torque()

        data["metadata"]["joint_ids"] = hand.config.joint_ids
        data["metadata"]["hand_type"] = hand.config.type

        input("Press Enter to start recording. Press Ctrl+C to stop.\n")
        start_time = time.monotonic()
        index = 0
        while True:
            elapsed = time.monotonic() - start_time
            if args.duration is not None and elapsed > args.duration:
                break
            pose = hand.get_joint_position().as_list(hand.config.joint_ids)
            # A failed bus read leaves cached values behind, which would replay
            # as real motion; drop the frame instead of recording a lie.
            if hand.last_read_ok:
                data["angles"].append([float(value) for value in pose])
            else:
                stale_frames += 1
            print(
                f"Recording... captured {len(data['angles'])} frames "
                f"({stale_frames} dropped)", end="\r",
            )
            index += 1
            remaining = start_time + index * interval - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nRecording stopped by user.")
    finally:
        if start_time is not None:
            elapsed = time.monotonic() - start_time
        frames = data["angles"]
        total = len(frames) + stale_frames
        stale_fraction = stale_frames / total if total else 0.0

        if frames and elapsed > 0:
            measured = len(frames) / elapsed
            data["metadata"]["sampling_frequency_hz"] = round(measured, 3)
            if measured < args.frequency * (1 - RATE_TOLERANCE):
                print(
                    f"\nWARNING: recorded at {measured:.1f} Hz, "
                    f"below the requested {args.frequency:.1f} Hz; "
                    "the file is labelled with the measured rate."
                )
        if stale_frames:
            print(
                f"\n{stale_frames}/{total} frames dropped on failed motor reads "
                f"({stale_fraction:.0%})."
            )

        if not frames:
            print("Nothing recorded; no file written.")
        elif stale_fraction > MAX_STALE_FRACTION:
            print(
                f"Refusing to save: more than {MAX_STALE_FRACTION:.0%} of frames "
                "were dropped. Check the motor chain wiring and baud rate."
            )
        else:
            output_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
            print(f"Saved {len(frames)} frames to {output_path}")
        shutdown_hand(hand)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
