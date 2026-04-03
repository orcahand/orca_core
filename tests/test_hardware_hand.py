import numpy as np
import pytest


def test_connect_disconnect_updates_connection_state(connected_mock_hand):
    assert connected_mock_hand.is_connected()
    success, _ = connected_mock_hand.disconnect()
    assert success
    assert not connected_mock_hand.is_connected()


def test_set_control_mode_preserves_wrist_special_case(connected_mock_hand):
    wrist_motor_id = connected_mock_hand.config.joint_to_motor_map.get("wrist")
    connected_mock_hand.set_control_mode("current_based_position")
    for motor_id in connected_mock_hand.config.motor_ids:
        expected_mode = 4 if motor_id == wrist_motor_id else 5
        assert connected_mock_hand._motor_client._operating_mode[motor_id] == expected_mode


def test_set_max_current_supports_scalar_and_list(connected_mock_hand):
    connected_mock_hand.set_max_current(123.0)
    assert all(
        current == 123.0 for current in connected_mock_hand._motor_client._cur.values()
    )
    desired_currents = [
        float(idx) for idx in range(len(connected_mock_hand.config.motor_ids))
    ]
    connected_mock_hand.set_max_current(desired_currents)
    for motor_id, desired in zip(connected_mock_hand.config.motor_ids, desired_currents):
        assert connected_mock_hand._motor_client._cur[motor_id] == desired


def test_disconnect_disables_torque(connected_mock_hand):
    connected_mock_hand.disconnect()
    assert all(
        not enabled
        for enabled in connected_mock_hand._motor_client._torque_enabled.values()
    )


def test_calibrated_joint_command_round_trips_through_motor_mapping(
    initialized_mock_hand,
):
    calibrated_hand = initialized_mock_hand
    target = {}
    for joint in calibrated_hand.config.joint_ids[:3]:
        min_pos, max_pos = calibrated_hand.config.joint_roms_dict[joint]
        target[joint] = (min_pos + max_pos) / 2.0
    calibrated_hand.set_joint_positions(target)
    actual = calibrated_hand.get_joint_position().as_dict()
    for joint, expected in target.items():
        assert actual[joint] == pytest.approx(expected, abs=1e-5)


def test_calibrated_list_command_round_trips(initialized_mock_hand):
    calibrated_hand = initialized_mock_hand
    command = []
    for joint in calibrated_hand.config.joint_ids:
        min_pos, max_pos = calibrated_hand.config.joint_roms_dict[joint]
        command.append((min_pos + max_pos) / 4.0)
    calibrated_hand.set_joint_positions(np.array(command))
    actual = calibrated_hand.get_joint_position().as_array(calibrated_hand.config.joint_ids)
    np.testing.assert_allclose(actual, np.array(command), atol=1e-6)
