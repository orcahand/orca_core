

import argparse
import time

import numpy as np
import yaml

from orca_core.utils.cli import (
    add_hand_arguments,
    connect_hand,
    create_hand_from_args,
    resolve_input_path,
    shutdown_hand,
)


def linear_interp(t: float) -> float:
    return t


def ease_in_out(t: float) -> float:
    return 0.5 * (1 - np.cos(np.pi * t))


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

    replay_path = resolve_input_path(args.replay_file)
    try:
        replay_data = yaml.safe_load(replay_path.read_text(encoding="utf-8")) or {}
    except FileNotFoundError:
        print(f"Replay file not found: {replay_path}")
        return 1

    waypoints = replay_data.get("waypoints", [])
    if not waypoints:
        print("No waypoints found in the replay file.")
        return 1

    metadata = replay_data.get("metadata", {})
    hand = create_hand_from_args(args)
    try:
        connect_hand(hand)
        hand.init_joints()

        expected_joint_ids = metadata.get("joint_ids")
        if expected_joint_ids is not None and expected_joint_ids != hand.config.joint_ids:
            raise ValueError("Replay joint order does not match the connected hand configuration.")

        # Left and right hands share a joint order, so only hand_type catches a
        # sequence recorded on the mirrored hand.
        if not args.force and metadata.get("hand_type") not in (None, hand.config.type):
            raise ValueError(
                f"Replay was recorded for hand_type={metadata['hand_type']}, "
                f"but the connected config is {hand.config.type}. Pass --force to "
                "replay it anyway."
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
