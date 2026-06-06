from orca_core import OrcaHand, OrcaJointPositions
from orca_core.hardware_hand import MockOrcaHand
import pytest


def test_import_and_instantiation(mock_config_dir):
    hand = MockOrcaHand(config_path=str(mock_config_dir / "config.yaml"))
    assert isinstance(hand, OrcaHand)


def test_orca_hand_exposes_refactored_methods(mock_config_dir):
    hand = MockOrcaHand(config_path=str(mock_config_dir / "config.yaml"))
    for method_name in [
        "connect",
        "disconnect",
        "init_joints",
        "enable_torque",
        "disable_torque",
        "set_control_mode",
        "set_max_current",
        "calibrate",
        "tension",
        "jitter",
        "pose_from_fractions",
        "register_position",
        "set_named_position",
        "play_named_positions",
        "run_demo",
        "get_joint_position",
    ]:
        assert hasattr(hand, method_name), f"Missing hardware method: {method_name}"


def test_mock_workflow_smoke(initialized_mock_hand):
    hand = initialized_mock_hand

    pose = OrcaJointPositions.from_dict(
        {
            "thumb_mcp": hand.config.neutral_position["thumb_mcp"] + 5.0,
            "index_mcp": hand.config.neutral_position["index_mcp"] + 5.0,
            "middle_mcp": hand.config.neutral_position["middle_mcp"] + 5.0,
            "wrist": hand.config.neutral_position["wrist"] + 3.0,
        }
    )

    hand.set_joint_positions(pose, num_steps=4, step_size=0.0)
    hand.register_position("smoke_pose", pose)
    hand.set_neutral_position(num_steps=4, step_size=0.0)
    hand.set_named_position("smoke_pose", num_steps=4, step_size=0.0)

    assert hand.is_connected()
    actual = hand.get_joint_position().as_dict()
    for joint, expected in pose.as_dict().items():
        assert actual[joint] == pytest.approx(expected, abs=1e-6)


def test_builtin_demo_positions_can_run_on_mock_hand(initialized_mock_hand):
    hand = initialized_mock_hand

    sequence = hand.run_demo("main", cycles=1, num_steps=1, step_size=0.0)
    assert sequence == ("open_hand", "power_grasp", "pinch")

    actual = hand.get_joint_position().as_dict()
    for joint, expected in hand.config.neutral_position.items():
        assert actual[joint] == pytest.approx(expected)
