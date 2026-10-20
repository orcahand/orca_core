

import argparse
import time

import numpy as np

from orca_core.utils.cli import (
    add_hand_arguments,
    check_recording_matches_hand,
    connect_hand,
    create_hand_from_args,
    load_recording,
    shutdown_hand,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay a continuous joint recording.")
    add_hand_arguments(parser)
    parser.add_argument("--replay-file", type=str, required=True)
    args = parser.parse_args()

    recording = load_recording(args.replay_file)
    if recording is None:
        return 1
    replay_path, replay_data = recording

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
    try:
        connect_hand(hand)
        hand.init_joints()
        check_recording_matches_hand(metadata, hand)

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
