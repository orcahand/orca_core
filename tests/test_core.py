from pathlib import Path

from orca_core import OrcaHand, OrcaJointPositions
from orca_core import MockOrcaHand
from orca_core.utils import get_model_path
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


def test_fraction_poses_can_be_sequenced_on_mock_hand(initialized_mock_hand):
    hand = initialized_mock_hand

    for name, fraction in (("open", 0.1), ("closed", 0.9)):
        hand.register_position(
            name, hand.pose_from_fractions({j: fraction for j in hand.config.joint_ids})
        )

    hand.play_named_positions(
        ["open", "closed"],
        cycles=1,
        num_steps=1,
        step_size=0.0,
        return_to_neutral=True,
    )

    actual = hand.get_joint_position().as_dict()
    for joint, expected in hand.config.neutral_position.items():
        assert actual[joint] == pytest.approx(expected)


def test_get_model_path_falls_back_across_versions_for_bare_model_name(tmp_path, monkeypatch):
    models_dir = tmp_path / "models"
    for version, name in [
        ("v1", "orcahand-right"),
        ("v1", "orcahand-left"),
        ("v2", "orcahand-right"),
    ]:
        model_dir = models_dir / version / name
        model_dir.mkdir(parents=True)
        (model_dir / "config.yaml").write_text("joint_ids: []\n", encoding="utf-8")
    monkeypatch.setattr("orca_core.utils.utils._get_models_dir", lambda: str(models_dir))

    assert Path(get_model_path("orcahand-left")) == models_dir / "v1" / "orcahand-left"
