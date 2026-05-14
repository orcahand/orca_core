

import argparse
import time
from pathlib import Path

import yaml

from common import (
    add_hand_arguments,
    connect_hand,
    create_hand,
    prepare_output_dir,
    shutdown_hand,
)


def _build_output_path(output_dir: Path, prefix: str) -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    stem = f"{prefix}_continuous_angles_{timestamp}" if prefix else f"continuous_angles_{timestamp}"
    return output_dir / f"{stem}.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Continuously record joint angles while manually moving the hand."
    )
    add_hand_arguments(parser)
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

    hand = create_hand(args.config_path, use_mock=args.mock)
    data = {
        "metadata": {
            "type": "continuous",
            "created_at": time.strftime("%Y%m%d_%H%M%S"),
            "sampling_frequency_hz": args.frequency,
        },
        "angles": [],
    }

    try:
        connect_hand(hand)
        hand.init_joints(force_calibrate=args.force_calibrate or args.mock)
        hand.disable_torque()

        data["metadata"]["joint_ids"] = hand.config.joint_ids
        data["metadata"]["hand_type"] = hand.config.type

        input("Press Enter to start recording. Press Ctrl+C to stop.\n")
        start_time = time.time()
        while True:
            if args.duration is not None and (time.time() - start_time) > args.duration:
                break
            pose = hand.get_joint_position().as_list(hand.config.joint_ids)
            data["angles"].append([float(value) for value in pose])
            print(f"Recording... captured {len(data['angles'])} frames", end="\r")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nRecording stopped by user.")
    finally:
        if data["angles"]:
            output_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
            print(f"Saved {len(data['angles'])} frames to {output_path}")
        shutdown_hand(hand)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
