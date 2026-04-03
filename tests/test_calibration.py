import pytest
from orca_core.hardware_hand import MockOrcaHand
from orca_core.utils import read_yaml

EXPECTED_LIMITS = [-1.0, 1.0]


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


def test_calibration_yaml_missing(mock_config_dir):
    calib_path = mock_config_dir / "calibration.yaml"
    hand = MockOrcaHand(config_path=str(mock_config_dir / "config.yaml"))
    success, msg = hand.connect()
    assert success, f"Failed to connect mock hand: {msg}"

    try:
        assert (
            not hand.calibrated
        ), "Hand should not be marked as calibrated before calibration"

        hand.calibrate()

        assert calib_path.exists(), "calibration.yaml should be created"

        check_calibrated(hand, calib_path)
    finally:
        hand.stop_task()
        hand.disconnect()
