

import argparse
import time

import numpy as np

from orca_core.utils import ease_in_out, linear_interp
from orca_core.utils.cli import (
    add_hand_arguments,
    check_recording_matches_hand,
    connect_hand,
    create_hand_from_args,
    load_recording,
    shutdown_hand,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay recorded waypoint poses.")
    add_hand_arguments(parser)
    parser.add_argument("--step-time", type=float, default=0.02)
    parser.add_argument("--transition-time", type=float, default=0.5)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument(
        "--mode",
        choices=["linear", "ease_in_out"],
        default="ease_in_out",
    )
    parser.add_argument("--replay-file", type=str, required=True)
    parser.add_argument(
        "--force",
        action="store_true",
        help="Replay even when the recording was made on the other hand side.",
    )
    args = parser.parse_args()

    recording = load_recording(args.replay_file)
    if recording is None:
        return 1
    replay_path, replay_data = recording

    waypoints = replay_data.get("waypoints", [])
    if not waypoints:
        print("No waypoints found in the replay file.")
        return 1

    metadata = replay_data.get("metadata", {})
    hand = create_hand_from_args(args)
    try:
        connect_hand(hand)
        hand.init_joints()
        check_recording_matches_hand(
            metadata, hand, force=args.force, force_flag="--force"
        )

        interp_func = linear_interp if args.mode == "linear" else ease_in_out
        wrist_idx = hand.config.joint_ids.index("wrist")
        print(f"Starting waypoint replay from {replay_path}")

        while True:
            for index, start in enumerate(waypoints):
                if not args.loop and index == len(waypoints) - 1:
                    final = np.asarray(start, dtype=np.float64)
                    final[wrist_idx] = 0.0
                    hand.set_joint_positions(final)
                    return 0
                end = waypoints[(index + 1) % len(waypoints)]
                n_steps = max(1, int(args.transition_time / args.step_time))
                start_time = time.time()

                for step in range(n_steps + 1):
                    alpha = interp_func(step / n_steps)
                    pose = [(1 - alpha) * s + alpha * e for s, e in zip(start, end)]
                    pose[wrist_idx] = 0.0
                    hand.set_joint_positions(np.asarray(pose, dtype=np.float64))

                    target_time = start_time + step * args.step_time
                    remaining = target_time - time.time()
                    if remaining > 0:
                        time.sleep(remaining)

                if not args.loop and index == len(waypoints) - 1:
                    return 0
    except KeyboardInterrupt:
        print("\nReplay interrupted.")
        return 0
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
