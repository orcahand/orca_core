

import argparse

from common import add_hand_arguments, connect_hand, create_hand, shutdown_hand
from orca_core import OrcaJointPositions


def pose_from_fractions(hand, fractions: dict[str, float]) -> OrcaJointPositions:
    pose = dict(hand.config.neutral_position)
    for joint, fraction in fractions.items():
        if joint not in hand.config.joint_roms_dict:
            continue
        joint_min, joint_max = hand.config.joint_roms_dict[joint]
        pose[joint] = joint_min + fraction * (joint_max - joint_min)
    return OrcaJointPositions.from_dict(pose)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a simple open-close-pinch demo using the current hand config."
    )
    add_hand_arguments(parser)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--num-steps", type=int, default=8)
    parser.add_argument("--step-size", type=float, default=0.02)
    args = parser.parse_args()

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        hand.init_joints(force_calibrate=args.mock)

        demo_poses = {
            "open_hand": pose_from_fractions(
                hand,
                {
                    "thumb_cmc": 0.70,
                    "thumb_abd": 0.80,
                    "thumb_mcp": 0.85,
                    "thumb_dip": 0.75,
                    "index_abd": 0.10,
                    "middle_abd": 0.50,
                    "ring_abd": 0.70,
                    "pinky_abd": 0.85,
                    "index_mcp": 0.15,
                    "middle_mcp": 0.15,
                    "ring_mcp": 0.15,
                    "pinky_mcp": 0.15,
                    "index_pip": 0.10,
                    "middle_pip": 0.10,
                    "ring_pip": 0.10,
                    "pinky_pip": 0.10,
                    "wrist": 0.30,
                },
            ),
            "power_grasp": pose_from_fractions(
                hand,
                {
                    "thumb_cmc": 0.35,
                    "thumb_abd": 0.55,
                    "thumb_mcp": 0.20,
                    "thumb_dip": 0.85,
                    "index_mcp": 0.85,
                    "middle_mcp": 0.85,
                    "ring_mcp": 0.85,
                    "pinky_mcp": 0.85,
                    "index_pip": 0.90,
                    "middle_pip": 0.90,
                    "ring_pip": 0.90,
                    "pinky_pip": 0.90,
                    "wrist": 0.55,
                },
            ),
            "pinch": pose_from_fractions(
                hand,
                {
                    "thumb_cmc": 0.45,
                    "thumb_abd": 0.65,
                    "thumb_mcp": 0.40,
                    "thumb_dip": 0.75,
                    "index_abd": 0.30,
                    "index_mcp": 0.70,
                    "index_pip": 0.75,
                    "middle_mcp": 0.30,
                    "middle_pip": 0.20,
                    "ring_mcp": 0.20,
                    "ring_pip": 0.15,
                    "pinky_mcp": 0.20,
                    "pinky_pip": 0.15,
                    "wrist": 0.45,
                },
            ),
        }

        for name, pose in demo_poses.items():
            hand.register_position(name, pose)

        print("Cycling through open_hand -> power_grasp -> pinch -> neutral")
        for _ in range(args.cycles):
            for name in ("open_hand", "power_grasp", "pinch"):
                print(f"Moving to {name}")
                hand.set_named_position(name, num_steps=args.num_steps, step_size=args.step_size)
            hand.set_neutral_position(num_steps=args.num_steps, step_size=args.step_size)

        return 0
    except KeyboardInterrupt:
        print("\nDemo interrupted.")
        return 0
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
