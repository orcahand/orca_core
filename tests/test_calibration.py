import unittest
import tempfile
import os
import shutil
import yaml
from orca_core import MockOrcaHand
from orca_core.utils import read_yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "orcahand_v1_right")
REAL_CONFIG = os.path.join(MODEL_DIR, "config.yaml")
REAL_CALIB = os.path.join(MODEL_DIR, "calibration.yaml")

EXPECTED_LIMITS = [-1.0, 1.0]

class TestOrcaHandCalibration(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.config_path = os.path.join(self.temp_dir, "config.yaml")
        self.calib_path = os.path.join(self.temp_dir, "calibration.yaml")
        shutil.copy(REAL_CONFIG, self.config_path)
        shutil.copy(REAL_CALIB, self.calib_path)

    def tearDown(self):
        shutil.rmtree(self.temp_dir)
        
    def check_calibrated(self, hand):
        self.assertTrue(hand.calibrated, "Hand should be marked as calibrated")

        calib = read_yaml(self.calib_path)
        
        motor_limits_file = calib.get('motor_limits', {})
        motor_limits = hand.motor_limits_dict
        if motor_limits != motor_limits_file:
            print(f"motor_limits from hand: {motor_limits}")
            print(f"motor_limits from file: {motor_limits_file}")
        self.assertEqual(motor_limits, motor_limits_file, "Motor limits do not match between hand and calibration file")
        
        joint_to_motor_ratios_file = calib.get('joint_to_motor_ratios', {})
        joint_to_motor_ratios = hand.joint_to_motor_ratios_dict
        if joint_to_motor_ratios != joint_to_motor_ratios_file:
            print(f"joint_to_motor_ratios from hand: {joint_to_motor_ratios}")
            print(f"joint_to_motor_ratios from file: {joint_to_motor_ratios_file}")
        self.assertEqual(joint_to_motor_ratios, joint_to_motor_ratios_file, "Joint to motor ratios do not match between hand and calibration file")
        
        for mid, ratio in joint_to_motor_ratios.items():
            self.assertNotEqual(ratio, 0, f"Joint to motor ratio for motor {mid} should not be 0")
            self.assertGreaterEqual(motor_limits[mid][0], EXPECTED_LIMITS[0], f"Motor {mid} min limit is below expected")
            self.assertLessEqual(motor_limits[mid][1], EXPECTED_LIMITS[1], f"Motor {mid} max limit is above expected")


    def test_calibration_yaml_missing(self):
        os.remove(self.calib_path)
        hand = MockOrcaHand(self.temp_dir)
        hand.connect()
    
        self.assertFalse(hand.calibrated, "Hand should not be marked as calibrated before calibration")
        
        hand.calibrate()

        self.assertTrue(os.path.exists(self.calib_path), "calibration.yaml should be created")
        
        self.check_calibrated(hand)

if __name__ == "__main__":
    unittest.main()
