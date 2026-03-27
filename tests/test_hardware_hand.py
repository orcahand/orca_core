import os
import shutil

import numpy as np
import pytest
from orca_core.hardware_hand import MockOrcaHand

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "v2", "orcahand_right")
REAL_CONFIG = os.path.join(MODEL_DIR, "config.yaml")


@pytest.fixture
def mock_hand():
    hand = MockOrcaHand(config_path=REAL_CONFIG)
    success, msg = hand.connect()
    assert success, f"Failed to connect: {msg}"
    yield hand
    hand.stop_task()
    hand.disconnect()


@pytest.fixture
def calibrated_hand(tmp_path):
    shutil.copy(REAL_CONFIG, tmp_path / "config.yaml")
    hand = MockOrcaHand(config_path=str(tmp_path / "config.yaml"))
    success, msg = hand.connect()
    assert success, f"Failed to connect: {msg}"
    hand.calibrate()
    yield hand
    hand.stop_task()
    hand.disconnect()


def test_connect_disconnect_updates_connection_state(mock_hand):
    assert mock_hand.is_connected()
    success, _ = mock_hand.disconnect()
    assert success
    assert not mock_hand.is_connected()


def test_set_control_mode_preserves_wrist_special_case(mock_hand):
    wrist_motor_id = mock_hand.config.joint_to_motor_map.get("wrist")
    mock_hand.set_control_mode("current_based_position")
    for motor_id in mock_hand.config.motor_ids:
        expected_mode = 4 if motor_id == wrist_motor_id else 5
        assert mock_hand._motor_client._operating_mode[motor_id] == expected_mode


def test_set_max_current_supports_scalar_and_list(mock_hand):
    mock_hand.set_max_current(123.0)
    assert all(current == 123.0 for current in mock_hand._motor_client._cur.values())
    desired_currents = [float(idx) for idx in range(len(mock_hand.config.motor_ids))]
    mock_hand.set_max_current(desired_currents)
    for motor_id, desired in zip(mock_hand.config.motor_ids, desired_currents):
        assert mock_hand._motor_client._cur[motor_id] == desired


def test_disconnect_disables_torque(mock_hand):
    mock_hand.disconnect()
    assert all(not enabled for enabled in mock_hand._motor_client._torque_enabled.values())


def test_calibrated_joint_command_round_trips_through_motor_mapping(calibrated_hand):
    target = {}
    for joint in calibrated_hand.config.joint_ids[:3]:
        min_pos, max_pos = calibrated_hand.config.joint_roms_dict[joint]
        target[joint] = (min_pos + max_pos) / 2.0
    calibrated_hand.set_joint_positions(target)
    actual = calibrated_hand.get_joint_position().as_dict()
    for joint, expected in target.items():
        assert actual[joint] == pytest.approx(expected, abs=1e-5)


def test_calibrated_list_command_round_trips(calibrated_hand):
    command = []
    for joint in calibrated_hand.config.joint_ids:
        min_pos, max_pos = calibrated_hand.config.joint_roms_dict[joint]
        command.append((min_pos + max_pos) / 4.0)
    calibrated_hand.set_joint_positions(np.array(command))
    actual = calibrated_hand.get_joint_position().as_array(calibrated_hand.config.joint_ids)
    np.testing.assert_allclose(actual, np.array(command), atol=1e-6)
