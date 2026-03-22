import os
import shutil
import tempfile
import unittest

import yaml

from orca_core import BaseHand, OrcaJointPosition


class DummyHand(BaseHand):
    def __init__(self, config_path):
        super().__init__(config_path=config_path)
        self.state = {joint: 0.0 for joint in self.joint_ids}
        self.command_log = []

    def _get_joint_pos_impl(self) -> dict:
        return self.state.copy()

    def _set_joint_pos_impl(self, joint_pos: dict) -> None:
        self.command_log.append(joint_pos.copy())
        for joint, value in joint_pos.items():
            if value is not None:
                self.state[joint] = value


class TestBaseHandContract(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.config_path = os.path.join(self.temp_dir, "config.yaml")
        with open(os.path.join(self.temp_dir, "config.yaml"), "w", encoding="utf-8") as file:
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
        with open(os.path.join(self.temp_dir, "calibration.yaml"), "w", encoding="utf-8") as file:
            yaml.safe_dump({}, file, sort_keys=False)

        self.hand = DummyHand(self.config_path)

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    def test_accepts_dict_joint_commands(self):
        self.hand.set_joint_pos({"joint_a": 0.5, "joint_b": 1.0})
        self.assertEqual(self.hand.get_joint_pos(as_list=False)["joint_a"], 0.5)
        self.assertEqual(self.hand.get_joint_pos(as_list=False)["joint_b"], 1.0)

    def test_accepts_orca_joint_position_commands(self):
        self.hand.set_joint_pos(OrcaJointPosition.from_dict({"joint_a": 0.5, "joint_c": -0.2}))
        self.assertEqual(self.hand.get_joint_pos(as_list=False)["joint_a"], 0.5)
        self.assertEqual(self.hand.get_joint_pos(as_list=False)["joint_c"], -0.2)

    def test_accepts_list_joint_commands_in_joint_order(self):
        self.hand.set_joint_pos([0.1, 0.2, 0.3])
        self.assertEqual(self.hand.get_joint_pos(as_list=False), {"joint_a": 0.1, "joint_b": 0.2, "joint_c": 0.3})

    def test_wrong_length_list_raises(self):
        with self.assertRaises(ValueError):
            self.hand.set_joint_pos([0.1, 0.2])

    def test_joint_commands_are_clipped_to_joint_rom(self):
        self.hand.set_joint_pos({"joint_a": 5.0, "joint_b": -1.0, "joint_c": 0.75})
        self.assertEqual(
            self.hand.get_joint_pos(as_list=False),
            {"joint_a": 1.0, "joint_b": 0.0, "joint_c": 0.5},
        )

    def test_partial_commands_preserve_unspecified_joints(self):
        self.hand.set_joint_pos({"joint_a": 0.4, "joint_b": 0.9, "joint_c": -0.2})
        self.hand.set_joint_pos({"joint_b": 1.5})
        self.assertEqual(
            self.hand.get_joint_pos(as_list=False),
            {"joint_a": 0.4, "joint_b": 1.5, "joint_c": -0.2},
        )

    def test_get_joint_pos_as_list_preserves_order(self):
        self.hand.set_joint_pos({"joint_a": -0.1, "joint_b": 1.1, "joint_c": 0.2})
        self.assertEqual(self.hand.get_joint_pos(as_list=True), [-0.1, 1.1, 0.2])

    def test_get_joint_position_returns_typed_wrapper(self):
        self.hand.set_joint_pos({"joint_a": 0.4, "joint_b": 1.2, "joint_c": -0.1})
        joint_pos = self.hand.get_joint_position()
        self.assertIsInstance(joint_pos, OrcaJointPosition)
        self.assertEqual(joint_pos.as_dict(), {"joint_a": 0.4, "joint_b": 1.2, "joint_c": -0.1})

    def test_set_zero_position_commands_all_zeroes(self):
        self.hand.set_joint_pos({"joint_a": 0.4, "joint_b": 1.5, "joint_c": -0.2})
        self.hand.set_zero_position()
        self.assertEqual(self.hand.get_joint_pos(as_list=False), {"joint_a": 0.0, "joint_b": 0.0, "joint_c": 0.0})

    def test_set_neutral_position_uses_configured_neutral_pose(self):
        self.hand.set_neutral_position()
        self.assertEqual(
            self.hand.get_joint_pos(as_list=False),
            {"joint_a": 0.25, "joint_b": 1.25, "joint_c": -0.25},
        )

    def test_set_neutral_position_requires_neutral_pose_in_config(self):
        missing_neutral_dir = tempfile.mkdtemp()
        try:
            missing_neutral_config_path = os.path.join(missing_neutral_dir, "config.yaml")
            with open(missing_neutral_config_path, "w", encoding="utf-8") as file:
                yaml.safe_dump(
                    {
                        "joint_ids": ["joint_a", "joint_b"],
                        "joint_roms": {"joint_a": [-1.0, 1.0], "joint_b": [0.0, 2.0]},
                    },
                    file,
                    sort_keys=False,
                )
            with open(os.path.join(missing_neutral_dir, "calibration.yaml"), "w", encoding="utf-8") as file:
                yaml.safe_dump({}, file, sort_keys=False)

            hand = DummyHand(missing_neutral_config_path)
            with self.assertRaises(ValueError):
                hand.set_neutral_position()
        finally:
            shutil.rmtree(missing_neutral_dir)

    def test_interpolation_reaches_same_final_pose(self):
        target = {"joint_a": -0.5, "joint_b": 0.5, "joint_c": 0.25}
        self.hand.set_joint_pos(target, num_steps=4, step_size=0.0)
        self.assertEqual(self.hand.get_joint_pos(as_list=False), target)
        self.assertEqual(len(self.hand.command_log), 5)

    def test_directory_config_path_is_rejected(self):
        with self.assertRaises(ValueError):
            DummyHand(self.temp_dir)
