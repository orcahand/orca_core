from pathlib import Path

import numpy as np
import pytest
import yaml

from orca_core.base_hand import BaseHand
from orca_core.joint_position import OrcaJointPositions


class DummyHand(BaseHand):
    def __init__(self, config_path):
        super().__init__(config_path=config_path)
        self.state = {joint: 0.0 for joint in self.config.joint_ids}
        self.command_log = []

    def _get_joint_positions(self) -> OrcaJointPositions:
        return OrcaJointPositions.from_dict(self.state)

    def _set_joint_positions(self, joint_pos: OrcaJointPositions) -> bool:
        self.command_log.append(joint_pos.as_dict())
        for joint, value in joint_pos:
            if value is not None:
                self.state[joint] = value
        return True


@pytest.fixture
def hand(tmp_path):
    config_path = tmp_path / "config.yaml"
    with open(config_path, "w", encoding="utf-8") as file:
        yaml.safe_dump(
            {
                "joint_ids": ["joint_a", "joint_b", "joint_c"],
                "joint_roms": {
                    "joint_a": [-1.0, 1.0],
                    "joint_b": [0.0, 2.0],
                    "joint_c": [-0.5, 0.5],
                },
                "neutral_position": {
                    "joint_a": 0.25,
                    "joint_b": 1.25,
                    "joint_c": -0.25,
                },
            },
            file,
            sort_keys=False,
        )
    with open(tmp_path / "calibration.yaml", "w", encoding="utf-8") as file:
        yaml.safe_dump({}, file, sort_keys=False)

    return DummyHand(str(config_path))


def test_accepts_dict_joint_commands(hand):
    hand.set_joint_positions({"joint_a": 0.5, "joint_b": 1.0})
    assert hand.get_joint_position().as_dict()["joint_a"] == 0.5
    assert hand.get_joint_position().as_dict()["joint_b"] == 1.0


def test_accepts_orca_joint_position_commands(hand):
    hand.set_joint_positions(
        OrcaJointPositions.from_dict({"joint_a": 0.5, "joint_c": -0.2})
    )
    assert hand.get_joint_position().as_dict()["joint_a"] == 0.5
    assert hand.get_joint_position().as_dict()["joint_c"] == -0.2


def test_accepts_ndarray_joint_commands_in_joint_order(hand):
    hand.set_joint_positions(np.array([0.1, 0.2, 0.3]))
    assert hand.get_joint_position().as_dict() == {
        "joint_a": 0.1,
        "joint_b": 0.2,
        "joint_c": 0.3,
    }


def test_wrong_length_ndarray_raises(hand):
    with pytest.raises(ValueError):
        hand.set_joint_positions(np.array([0.1, 0.2]))


def test_joint_commands_are_clipped_to_joint_rom(hand):
    hand.set_joint_positions({"joint_a": 5.0, "joint_b": -1.0, "joint_c": 0.75})
    assert hand.get_joint_position().as_dict() == {
        "joint_a": 1.0,
        "joint_b": 0.0,
        "joint_c": 0.5,
    }


def test_partial_commands_preserve_unspecified_joints(hand):
    hand.set_joint_positions({"joint_a": 0.4, "joint_b": 0.9, "joint_c": -0.2})
    hand.set_joint_positions({"joint_b": 1.5})
    assert hand.get_joint_position().as_dict() == {
        "joint_a": 0.4,
        "joint_b": 1.5,
        "joint_c": -0.2,
    }


def test_get_joint_position_as_list_preserves_order(hand):
    hand.set_joint_positions({"joint_a": -0.1, "joint_b": 1.1, "joint_c": 0.2})
    assert hand.get_joint_position().as_list(hand.config.joint_ids) == [-0.1, 1.1, 0.2]


def test_get_joint_position_returns_typed_wrapper(hand):
    hand.set_joint_positions({"joint_a": 0.4, "joint_b": 1.2, "joint_c": -0.1})
    joint_pos = hand.get_joint_position()
    assert isinstance(joint_pos, OrcaJointPositions)
    assert joint_pos.as_dict() == {
        "joint_a": 0.4,
        "joint_b": 1.2,
        "joint_c": -0.1,
    }


def test_set_zero_position_commands_all_zeroes(hand):
    hand.set_joint_positions({"joint_a": 0.4, "joint_b": 1.5, "joint_c": -0.2})
    hand.set_zero_position()
    assert hand.get_joint_position().as_dict() == {
        "joint_a": 0.0,
        "joint_b": 0.0,
        "joint_c": 0.0,
    }


def test_set_neutral_position_uses_configured_neutral_pose(hand):
    hand.set_neutral_position()
    assert hand.get_joint_position().as_dict() == {
        "joint_a": 0.25,
        "joint_b": 1.25,
        "joint_c": -0.25,
    }


def test_pose_from_fractions_defaults_to_neutral_for_unspecified_joints(hand):
    pose = hand.pose_from_fractions({"joint_a": 0.75, "joint_c": 0.25})
    assert pose.as_dict() == {
        "joint_a": 0.5,
        "joint_b": 1.25,
        "joint_c": -0.25,
    }


def test_play_named_positions_reuses_registered_poses(hand):
    positions = {
        "pose_one": OrcaJointPositions.from_dict({"joint_a": -0.5, "joint_b": 0.5, "joint_c": 0.0}),
        "pose_two": OrcaJointPositions.from_dict({"joint_a": 1.0, "joint_b": 2.0, "joint_c": -0.5}),
    }
    for name, joint_pos in positions.items():
        hand.register_position(name, joint_pos)

    hand.play_named_positions(
        list(positions.keys()),
        cycles=2,
        num_steps=1,
        step_size=0.0,
    )

    assert hand.get_joint_position().as_dict() == {
        "joint_a": 1.0,
        "joint_b": 2.0,
        "joint_c": -0.5,
    }


def test_set_neutral_position_without_configured_neutral_pose_is_noop(tmp_path):
    config_path = tmp_path / "config.yaml"
    with open(config_path, "w", encoding="utf-8") as file:
        yaml.safe_dump(
            {
                "joint_ids": ["joint_a", "joint_b"],
                "joint_roms": {"joint_a": [-1.0, 1.0], "joint_b": [0.0, 2.0]},
            },
            file,
            sort_keys=False,
        )

    hand = DummyHand(str(config_path))
    hand.set_joint_positions({"joint_a": 0.3, "joint_b": 1.2})
    hand.set_neutral_position()
    assert hand.get_joint_position().as_dict() == {"joint_a": 0.3, "joint_b": 1.2}


def test_interpolation_reaches_same_final_pose(hand):
    target = {"joint_a": -0.5, "joint_b": 0.5, "joint_c": 0.25}
    hand.set_joint_positions(target, num_steps=4, step_size=0.0)
    assert hand.get_joint_position().as_dict() == target
    assert len(hand.command_log) == 5


def test_directory_config_path_is_rejected(tmp_path):
    with pytest.raises(ValueError):
        DummyHand(str(Path(tmp_path)))
