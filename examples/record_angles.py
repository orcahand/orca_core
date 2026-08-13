

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

# Fraction of dropped captures above which the recording is not worth saving.
MAX_STALE_FRACTION = 0.1


def _build_output_path(output_dir: Path, prefix: str) -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    stem = f"{prefix}_replay_sequence_{timestamp}" if prefix else f"replay_sequence_{timestamp}"
    return output_dir / f"{stem}.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Record discrete joint-space waypoints by manually posing the hand."
    )
    add_hand_arguments(parser, feedback_flag=False)
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Directory where the replay YAML will be written.",
    )
    parser.add_argument(
        "--force-calibrate",
        action="store_true",
        help="Run calibration even if calibration.yaml already exists.",
    )
    args = parser.parse_args()

    output_dir = prepare_output_dir(args.output_dir)
    prefix = input(
        "Enter an optional filename prefix "
        "(for example 'pinch_demo'). Press Enter to skip: "
    ).strip()
    output_path = _build_output_path(output_dir, prefix)

    # Recording backdrives the hand with torque off, which a running joint
    # loop would fight and wind its integrator up against.
    hand = create_hand_from_args(args, engage_feedback=False)
    replay_buffer: list[list[float]] = []
    stale_captures = 0
    try:
        connect_hand(hand)
        hand.init_joints(force_calibrate=args.force_calibrate)
        hand.disable_torque()

        print("Torque disabled. Manually move the hand, then press Enter to capture a waypoint.")
        print("Press Ctrl+C when you are done recording.")

        while True:
            input("Capture waypoint")
            waypoint = hand.get_joint_position().as_list(hand.config.joint_ids)
            # A failed bus read leaves cached values behind, which would replay
            # as real motion; drop the capture instead of recording a lie.
            if not hand.last_read_ok:
                stale_captures += 1
                print("Motor read failed; waypoint discarded. Try again.")
                continue
            replay_buffer.append([float(value) for value in waypoint])
            print(f"Captured waypoint #{len(replay_buffer)}")
    except KeyboardInterrupt:
        print("\nRecording finished.")
    finally:
        total = len(replay_buffer) + stale_captures
        stale_fraction = stale_captures / total if total else 0.0
        if stale_captures:
            print(
                f"{stale_captures}/{total} captures dropped on failed motor reads "
                f"({stale_fraction:.0%})."
            )
        if replay_buffer and stale_fraction > MAX_STALE_FRACTION:
            print(
                f"Refusing to save: more than {MAX_STALE_FRACTION:.0%} of captures "
                "were dropped. Check the motor chain wiring and baud rate."
            )
        elif replay_buffer:
            payload = {
                "metadata": {
                    "type": "discrete_waypoints",
                    "created_at": time.strftime("%Y%m%d_%H%M%S"),
                    "joint_ids": hand.config.joint_ids,
                    "hand_type": hand.config.type,
                },
                "waypoints": replay_buffer,
            }
            output_path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
            print(f"Saved {len(replay_buffer)} waypoints to {output_path}")
        shutdown_hand(hand)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
