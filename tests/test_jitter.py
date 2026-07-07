import time
import inspect

import numpy as np
import pytest
from unittest.mock import patch


@pytest.fixture
def wrist_motor_id(connected_mock_hand):
    return connected_mock_hand.config.joint_to_motor_map.get("wrist")


def test_amplitude_exceeds_max_raises_error(connected_mock_hand):
    with pytest.raises(ValueError):
        connected_mock_hand.jitter(amplitude=15.0, duration=0.1)


def test_amplitude_at_max_allowed(connected_mock_hand):
    connected_mock_hand.jitter(amplitude=10.0, duration=0.1)


def test_amplitude_below_max(connected_mock_hand):
    connected_mock_hand.jitter(amplitude=3.0, duration=0.1)


def test_default_frequency_is_fast(connected_mock_hand):
    sig = inspect.signature(connected_mock_hand.jitter)
    default_freq = sig.parameters["frequency"].default
    assert default_freq >= 10.0


def test_duration_respected(connected_mock_hand):
    start = time.time()
    connected_mock_hand.jitter(duration=1.0, amplitude=2.0)
    elapsed = time.time() - start
    assert elapsed == pytest.approx(1.0, abs=0.3)


def test_short_duration(connected_mock_hand):
    start = time.time()
    connected_mock_hand.jitter(duration=0.3, amplitude=2.0)
    elapsed = time.time() - start
    assert elapsed < 1.0


def test_custom_motor_ids(connected_mock_hand):
    target_id = connected_mock_hand.config.motor_ids[0]
    with patch.object(
        connected_mock_hand._motor_client,
        "write_desired_pos",
        wraps=connected_mock_hand._motor_client.write_desired_pos,
    ) as mock_write:
        connected_mock_hand.jitter(motor_ids=[target_id], amplitude=5.0, duration=0.2)
        assert len(mock_write.call_args_list) > 0
        for call_args in mock_write.call_args_list:
            assert call_args[0][0] == [target_id]


def test_default_excludes_wrist(connected_mock_hand, wrist_motor_id):
    assert wrist_motor_id is not None, "Config must have a wrist motor for this test"
    with patch.object(
        connected_mock_hand._motor_client,
        "write_desired_pos",
        wraps=connected_mock_hand._motor_client.write_desired_pos,
    ) as mock_write:
        connected_mock_hand.jitter(amplitude=5.0, duration=0.2)
        assert len(mock_write.call_args_list) > 0
        for call_args in mock_write.call_args_list:
            assert wrist_motor_id not in call_args[0][0]


def test_wrist_excluded_even_with_many_motors(connected_mock_hand, wrist_motor_id):
    non_wrist_ids = [
        mid for mid in connected_mock_hand.config.motor_ids if mid != wrist_motor_id
    ]
    with patch.object(
        connected_mock_hand._motor_client,
        "write_desired_pos",
        wraps=connected_mock_hand._motor_client.write_desired_pos,
    ) as mock_write:
        connected_mock_hand.jitter(amplitude=5.0, duration=0.2)
        assert len(mock_write.call_args_list) > 0
        commanded_ids = set(mock_write.call_args_list[0][0][0])
        assert commanded_ids == set(non_wrist_ids)


def test_include_wrist_flag(connected_mock_hand, wrist_motor_id):
    with patch.object(
        connected_mock_hand._motor_client,
        "write_desired_pos",
        wraps=connected_mock_hand._motor_client.write_desired_pos,
    ) as mock_write:
        connected_mock_hand.jitter(amplitude=5.0, duration=0.2, include_wrist=True)
        assert len(mock_write.call_args_list) > 0
        commanded_ids = set(mock_write.call_args_list[0][0][0])
        assert wrist_motor_id in commanded_ids


def test_works_without_calibration(connected_mock_hand):
    connected_mock_hand.jitter(amplitude=5.0, duration=0.2)


def test_returns_to_start_position(connected_mock_hand):
    positions_before = connected_mock_hand.get_motor_pos()
    connected_mock_hand.jitter(amplitude=5.0, duration=0.5)
    positions_after = connected_mock_hand.get_motor_pos()
    np.testing.assert_allclose(positions_after, positions_before, atol=1e-6)


def test_non_blocking_runs_in_background(connected_mock_hand):
    connected_mock_hand.jitter(amplitude=5.0, duration=5.0, blocking=False)
    time.sleep(0.2)
    assert connected_mock_hand._task_thread.is_alive()
    connected_mock_hand.stop_task()
    time.sleep(0.2)
    assert not connected_mock_hand._task_thread.is_alive()


def test_early_stop_returns_to_start(connected_mock_hand):
    positions_before = connected_mock_hand.get_motor_pos()
    connected_mock_hand.jitter(amplitude=5.0, duration=10.0, blocking=False)
    time.sleep(0.3)
    connected_mock_hand.stop_task()
    time.sleep(0.2)
    positions_after = connected_mock_hand.get_motor_pos()
    np.testing.assert_allclose(positions_after, positions_before, atol=1e-6)


def test_second_jitter_rejected(connected_mock_hand):
    connected_mock_hand.jitter(amplitude=5.0, duration=5.0, blocking=False)
    time.sleep(0.1)
    connected_mock_hand.jitter(amplitude=5.0, duration=5.0, blocking=False)
    assert connected_mock_hand._task_thread.is_alive()
    assert connected_mock_hand._current_task == "_jitter"
    connected_mock_hand.stop_task()
    time.sleep(0.2)
    assert not connected_mock_hand._task_thread.is_alive()
