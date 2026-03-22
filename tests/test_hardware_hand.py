import os
import shutil
import tempfile
import unittest

import numpy as np

from orca_core import MockOrcaHand


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "orcahand_v1_right")
REAL_CONFIG = os.path.join(MODEL_DIR, "config.yaml")
REAL_CALIB = os.path.join(MODEL_DIR, "calibration.yaml")


class TestHardwareHand(unittest.TestCase):
    def setUp(self):
        self.hand = MockOrcaHand()
        success, msg = self.hand.connect()
        self.assertTrue(success, f"Failed to connect mock hand: {msg}")

    def tearDown(self):
        self.hand.stop_task()
        self.hand.disconnect()

    def test_connect_disconnect_updates_connection_state(self):
        self.assertTrue(self.hand.is_connected())
        success, _ = self.hand.disconnect()
        self.assertTrue(success)
        self.assertFalse(self.hand.is_connected())

    def test_set_control_mode_preserves_wrist_special_case(self):
        wrist_motor_id = self.hand.joint_to_motor_map.get("wrist")
        self.hand.set_control_mode("current_based_position")

        for motor_id in self.hand.motor_ids:
            expected_mode = 4 if motor_id == wrist_motor_id else 5
            self.assertEqual(self.hand._motor_client._operating_mode[motor_id], expected_mode)

    def test_set_max_current_supports_scalar_and_list(self):
        self.hand.set_max_current(123.0)
        self.assertTrue(all(current == 123.0 for current in self.hand._motor_client._cur.values()))

        desired_currents = [float(idx) for idx in range(len(self.hand.motor_ids))]
        self.hand.set_max_current(desired_currents)
        for motor_id, desired in zip(self.hand.motor_ids, desired_currents):
            self.assertEqual(self.hand._motor_client._cur[motor_id], desired)

    def test_disconnect_disables_torque(self):
        self.hand.disconnect()
        self.assertTrue(all(not enabled for enabled in self.hand._motor_client._torque_enabled.values()))


class TestHardwareHandRoundTrip(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.config_path = os.path.join(self.temp_dir, "config.yaml")
        shutil.copy(REAL_CONFIG, self.config_path)
        shutil.copy(REAL_CALIB, os.path.join(self.temp_dir, "calibration.yaml"))
        self.hand = MockOrcaHand(self.config_path)
        success, msg = self.hand.connect()
        self.assertTrue(success, f"Failed to connect mock hand: {msg}")
        self.hand.calibrate()

    def tearDown(self):
        self.hand.stop_task()
        self.hand.disconnect()
        shutil.rmtree(self.temp_dir)

    def test_calibrated_joint_command_round_trips_through_motor_mapping(self):
        target = {}
        for joint in self.hand.joint_ids[:3]:
            min_pos, max_pos = self.hand.joint_roms_dict[joint]
            target[joint] = (min_pos + max_pos) / 2.0

        self.hand.set_joint_pos(target)
        actual = self.hand.get_joint_pos(as_list=False)
        for joint, expected in target.items():
            self.assertAlmostEqual(actual[joint], expected, places=5)

    def test_calibrated_list_command_round_trips(self):
        command = []
        for joint in self.hand.joint_ids:
            min_pos, max_pos = self.hand.joint_roms_dict[joint]
            command.append((min_pos + max_pos) / 4.0)

        self.hand.set_joint_pos(command)
        actual = np.array(self.hand.get_joint_pos(as_list=True))
        np.testing.assert_allclose(actual, np.array(command), atol=1e-6)
