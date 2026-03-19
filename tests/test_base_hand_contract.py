import os
import shutil
import tempfile
import unittest

import yaml

from orca_core import BaseHand


class DummyHand(BaseHand):
    def __init__(self, model_path):
        super().__init__(model_path=model_path)
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

        self.hand = DummyHand(self.temp_dir)

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    def test_accepts_dict_joint_commands(self):
        self.hand.set_joint_pos({"joint_a": 0.5, "joint_b": 1.0})
        self.assertEqual(self.hand.get_joint_pos(as_list=False)["joint_a"], 0.5)
        self.assertEqual(self.hand.get_joint_pos(as_list=False)["joint_b"], 1.0)

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

    def test_interpolation_reaches_same_final_pose(self):
        target = {"joint_a": -0.5, "joint_b": 0.5, "joint_c": 0.25}
        self.hand.set_joint_pos(target, num_steps=4, step_size=0.0)
        self.assertEqual(self.hand.get_joint_pos(as_list=False), target)
        self.assertEqual(len(self.hand.command_log), 5)
