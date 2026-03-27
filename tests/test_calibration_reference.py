"""Verify that calibration on mock hardware produces identical results every time.

Compares calibration output against a checked-in reference file
(tests/reference/calibration_expected.yaml). If any test here fails,
calibration behavior has changed and needs to be investigated.
"""

import os
import shutil
import tempfile

import pytest
import yaml

from orca_core.hardware_hand import MockOrcaHand
from orca_core.utils import read_yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "v2", "orcahand_right")
REFERENCE_PATH = os.path.join(REPO_ROOT, "tests", "reference", "calibration_expected.yaml")


# ── fixtures ──


@pytest.fixture(scope="module")
def calibration_result():
    """Run a full from-scratch calibration on MockOrcaHand and return the persisted YAML."""
    tmp = tempfile.mkdtemp()
    shutil.copy(os.path.join(CONFIG_DIR, "config.yaml"), os.path.join(tmp, "config.yaml"))
    calib_path = os.path.join(tmp, "calibration.yaml")

    hand = MockOrcaHand(config_path=os.path.join(tmp, "config.yaml"))
    hand.connect()
    hand.calibrate()
    hand.disconnect()

    result = read_yaml(calib_path)
    shutil.rmtree(tmp)
    return result


@pytest.fixture(scope="module")
def reference():
    """Load the checked-in reference calibration output."""
    assert os.path.exists(REFERENCE_PATH), (
        f"Reference file not found at {REFERENCE_PATH}."
    )
    with open(REFERENCE_PATH) as f:
        return yaml.safe_load(f)


# ── tests ──


def test_calibration_marks_hand_calibrated(calibration_result, reference):
    assert calibration_result["calibrated"] == reference["calibrated"]


def test_wrist_calibrated_flag(calibration_result, reference):
    assert calibration_result["wrist_calibrated"] == reference["wrist_calibrated"]


def test_motor_limits_match_reference(calibration_result, reference):
    actual = calibration_result["motor_limits"]
    expected = reference["motor_limits"]

    assert set(actual.keys()) == set(expected.keys()), (
        f"Motor ID mismatch: got {sorted(actual.keys())}, expected {sorted(expected.keys())}"
    )
    for motor_id in expected:
        assert actual[motor_id] == pytest.approx(expected[motor_id], abs=1e-9), (
            f"Motor {motor_id} limits: got {actual[motor_id]}, expected {expected[motor_id]}"
        )


def test_joint_to_motor_ratios_match_reference(calibration_result, reference):
    actual = calibration_result["joint_to_motor_ratios"]
    expected = reference["joint_to_motor_ratios"]

    assert set(actual.keys()) == set(expected.keys()), (
        f"Motor ID mismatch: got {sorted(actual.keys())}, expected {sorted(expected.keys())}"
    )
    for motor_id in expected:
        assert actual[motor_id] == pytest.approx(expected[motor_id], abs=1e-9), (
            f"Motor {motor_id} ratio: got {actual[motor_id]}, expected {expected[motor_id]}"
        )
