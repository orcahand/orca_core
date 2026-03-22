import os
import shutil
import tempfile
import unittest

from orca_core import FileStateStore, MockOrcaHand, load_profile_from_path
from orca_core.utils import read_yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "orcahand_v1_right")

EXPECTED_LIMITS = [-1.0, 1.0]


class TestOrcaHandCalibration(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.profile_dir = os.path.join(self.temp_dir, "profile")
        shutil.copytree(MODEL_DIR, self.profile_dir)
        self.state_dir = os.path.join(self.temp_dir, "state")
        self.profile = load_profile_from_path(self.profile_dir)
        self.state_store = FileStateStore(self.state_dir)

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    def test_calibration_persists_runtime_state(self):
        hand = MockOrcaHand(profile=self.profile, state_store=self.state_store)
        hand.connect()

        self.assertFalse(hand.calibrated, "Hand should not be marked as calibrated before calibration")

        hand.calibrate()

        state_path = self.state_store.path_for(self.profile)
        self.assertTrue(os.path.exists(state_path), "Runtime state should be written to the state store")

        state_data = read_yaml(str(state_path))
        self.assertTrue(state_data["calibrated"])

        for motor_id, ratio in hand.joint_to_motor_ratios_dict.items():
            self.assertNotEqual(ratio, 0, f"Joint to motor ratio for motor {motor_id} should not be 0")
            self.assertGreaterEqual(hand.motor_limits_dict[motor_id][0], EXPECTED_LIMITS[0])
            self.assertLessEqual(hand.motor_limits_dict[motor_id][1], EXPECTED_LIMITS[1])
