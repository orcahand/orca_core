"""Runnable MockOrcaHand workflow demo.

This script is intentionally small and readable so it can double as a
walkthrough for the refactored hand API.

Run from the repository root with either:

    python -m orca_core.test

or:

    python orca_core/test.py
"""

import argparse
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orca_core import MockOrcaHand, OrcaJointPositions


PACKAGE_DIR = Path(__file__).resolve().parent
MODEL_DIR = PACKAGE_DIR / "models" / "v1" / "orcahand_right"
MODEL_CONFIG = MODEL_DIR / "config.yaml"

CONFIG_KEY_RENAMES = {
    "calib_current": "calibration_current",
    "calib_step_size": "calibration_step_size",
    "calib_step_period": "calibration_step_period",
    "calib_threshold": "calibration_threshold",
    "calib_num_stable": "calibration_num_stable",
    "calib_sequence": "calibration_sequence",
}


def normalize_demo_config(src: Path, dst: Path) -> None:
    data = yaml.safe_load(src.read_text(encoding="utf-8")) or {}
    for old_key, new_key in CONFIG_KEY_RENAMES.items():
        if old_key in data and new_key not in data:
            data[new_key] = data.pop(old_key)

    dst.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def build_demo_pose(hand: MockOrcaHand, fractions: dict[str, float]) -> OrcaJointPositions:
    pose = {}
    for joint, fraction in fractions.items():
        min_pos, max_pos = hand.config.joint_roms_dict[joint]
        pose[joint] = min_pos + fraction * (max_pos - min_pos)
    return OrcaJointPositions.from_dict(pose)


def build_full_demo_pose(
    hand: MockOrcaHand,
    fractions: dict[str, float],
) -> OrcaJointPositions:
    pose = dict(hand.config.neutral_position)
    for joint, fraction in fractions.items():
        min_pos, max_pos = hand.config.joint_roms_dict[joint]
        pose[joint] = min_pos + fraction * (max_pos - min_pos)
    return OrcaJointPositions.from_dict(pose)


def current_joint_dict(hand: MockOrcaHand) -> dict[str, float]:
    return hand._get_joint_positions().as_dict()


def assert_joint_subset(
    hand: MockOrcaHand,
    expected: OrcaJointPositions,
    atol: float = 1e-6,
) -> None:
    actual = current_joint_dict(hand)
    for joint, expected_value in expected.as_dict().items():
        actual_value = actual[joint]
        if actual_value is None or not np.isclose(actual_value, expected_value, atol=atol):
            raise AssertionError(
                f"Joint {joint} mismatch: expected {expected_value}, got {actual_value}."
            )


def print_joint_subset(label: str, joint_pos: dict[str, float], joints: list[str]) -> None:
    formatted = ", ".join(f"{joint}={joint_pos[joint]:.3f}" for joint in joints)
    print(f"{label}: {formatted}")


def run_workflow(reuse_calibration: bool = False) -> None:
    preview_joints = ["thumb_mcp", "index_mcp", "middle_mcp", "wrist"]

    with tempfile.TemporaryDirectory(prefix="orca_mock_workflow_") as temp_dir:
        temp_dir_path = Path(temp_dir)
        config_path = temp_dir_path / "config.yaml"
        calibration_path = temp_dir_path / "calibration.yaml"

        normalize_demo_config(MODEL_CONFIG, config_path)
        if reuse_calibration:
            bundled_calibration = MODEL_DIR / "calibration.yaml"
            if bundled_calibration.exists():
                shutil.copy(bundled_calibration, calibration_path)
        else:
            calibration_path.write_text("{}\n", encoding="utf-8")

        hand = MockOrcaHand(config_path=str(config_path))

        try:
            print("1. Connecting to the mock hand")
            success, message = hand.connect()
            print(f"   connect() -> success={success}, message={message}")
            if not success:
                raise RuntimeError(message)

            print("2. Initializing joints")
            hand.init_joints(force_calibrate=not reuse_calibration)
            if not hand.calibrated:
                raise AssertionError("Hand should be calibrated after init_joints().")

            neutral_joint_pos = current_joint_dict(hand)
            print_joint_subset("   neutral pose", neutral_joint_pos, preview_joints)

            print("3. Sending a typed joint-space command")
            pose_a = build_demo_pose(
                hand,
                {
                    "thumb_mcp": 0.75,
                    "index_mcp": 0.60,
                    "middle_mcp": 0.55,
                    "wrist": 0.35,
                },
            )
            hand.set_joint_positions(pose_a)
            assert_joint_subset(hand, pose_a)
            print_joint_subset("   pose A", current_joint_dict(hand), preview_joints)

            print("4. Interpolating to a second pose")
            pose_b = build_demo_pose(
                hand,
                {
                    "thumb_mcp": 0.25,
                    "index_mcp": 0.30,
                    "middle_mcp": 0.70,
                    "wrist": 0.65,
                },
            )
            hand.set_joint_positions(pose_b, num_steps=5, step_size=0.0)
            assert_joint_subset(hand, pose_b)
            print_joint_subset("   pose B", current_joint_dict(hand), preview_joints)

            print("5. Building a full pose as OrcaJointPositions")
            pose_c = build_full_demo_pose(
                hand,
                {
                    "thumb_mcp": 0.50,
                    "index_mcp": 0.45,
                    "middle_mcp": 0.40,
                    "wrist": 0.50,
                },
            )
            hand.set_joint_positions(pose_c)
            assert_joint_subset(hand, hand.config.clamp_joint_positions(pose_c))
            print_joint_subset("   pose C", current_joint_dict(hand), preview_joints)

            print("6. Registering and replaying a named pose")
            hand.register_position("demo_pose", pose_b)
            hand.set_named_position("demo_pose")
            assert_joint_subset(hand, pose_b)

            print("7. Running a short jitter pass")
            motor_pos_before = hand.get_motor_pos()
            hand.jitter(amplitude=1.0, frequency=3.0, duration=0.2, blocking=True)
            motor_pos_after = hand.get_motor_pos()
            np.testing.assert_allclose(motor_pos_after, motor_pos_before, atol=1e-6)

            print("8. Returning to neutral and disconnecting")
            hand.set_neutral_position(num_steps=5, step_size=0.0)
            neutral_pose = hand.config.clamp_joint_positions(
                OrcaJointPositions.from_dict(hand.config.neutral_position)
            )
            assert_joint_subset(hand, neutral_pose)

        finally:
            hand.stop_task()
            success, message = hand.disconnect()
            print(f"   disconnect() -> success={success}, message={message}")
            if hand.is_connected():
                raise AssertionError("Hand should be disconnected at the end of the workflow.")

    print("\nWorkflow completed successfully.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the MockOrcaHand workflow demo.")
    parser.add_argument(
        "--reuse-calibration",
        action="store_true",
        help="Reuse the bundled calibration file instead of forcing a fresh calibration in a temp directory.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_workflow(reuse_calibration=args.reuse_calibration)


if __name__ == "__main__":
    main()
