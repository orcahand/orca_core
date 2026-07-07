

import argparse
import time

import numpy as np
import yaml

from common import (
    add_hand_arguments,
    connect_hand,
    create_hand,
    resolve_input_path,
    shutdown_hand,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay a continuous joint recording.")
    add_hand_arguments(parser)
    parser.add_argument("--replay-file", type=str, required=True)
    args = parser.parse_args()

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

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        hand.init_joints(force_calibrate=args.mock)

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
        start_time = time.time()
        for index, pose in enumerate(waypoints):
            hand.set_joint_positions(np.asarray(pose, dtype=np.float64))
            target_time = start_time + index * step_time
            remaining = target_time - time.time()
            if remaining > 0:
                time.sleep(remaining)
        return 0
    except KeyboardInterrupt:
        print("\nReplay interrupted.")
        return 0
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
