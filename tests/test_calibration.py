import os
import shutil

import pytest
from orca_core import MockOrcaHand
from orca_core.utils import read_yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "orcahand_v1_right")
REAL_CONFIG = os.path.join(MODEL_DIR, "config.yaml")
REAL_CALIB = os.path.join(MODEL_DIR, "calibration.yaml")

EXPECTED_LIMITS = [-1.0, 1.0]


@pytest.fixture
def calib_dir(tmp_path):
    config_path = tmp_path / "config.yaml"
    calib_path = tmp_path / "calibration.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    shutil.copy(REAL_CALIB, calib_path)
    return tmp_path


def check_calibrated(hand, calib_path):
    assert hand.calibrated, "Hand should be marked as calibrated"

    calib = read_yaml(str(calib_path))

    motor_limits_file = calib.get('motor_limits', {})
    motor_limits = hand.motor_limits_dict
    assert motor_limits == motor_limits_file, "Motor limits do not match between hand and calibration file"

    joint_to_motor_ratios_file = calib.get('joint_to_motor_ratios', {})
    joint_to_motor_ratios = hand.joint_to_motor_ratios_dict
    assert joint_to_motor_ratios == joint_to_motor_ratios_file, "Joint to motor ratios do not match between hand and calibration file"

    for mid, ratio in joint_to_motor_ratios.items():
        assert ratio != 0, f"Joint to motor ratio for motor {mid} should not be 0"
        assert motor_limits[mid][0] >= EXPECTED_LIMITS[0], f"Motor {mid} min limit is below expected"
        assert motor_limits[mid][1] <= EXPECTED_LIMITS[1], f"Motor {mid} max limit is above expected"


def test_calibration_yaml_missing(calib_dir):
    calib_path = calib_dir / "calibration.yaml"
    os.remove(calib_path)
    hand = MockOrcaHand(str(calib_dir))
    hand.connect()

    assert not hand.calibrated, "Hand should not be marked as calibrated before calibration"

    hand.calibrate()

    assert calib_path.exists(), "calibration.yaml should be created"

    check_calibrated(hand, calib_path)
