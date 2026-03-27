import time

import pytest
from orca_core.hardware_hand import MockOrcaHand


@pytest.fixture
def mock_hand():
    hand = MockOrcaHand()
    success, msg = hand.connect()
    assert success, f"Failed to connect mock hand: {msg}"
    yield hand
    hand.stop_task()
    time.sleep(0.1)
    hand.disconnect()


def test_tension_move_motors_false(mock_hand):
    mock_hand.tension(move_motors=False, blocking=False)
    assert mock_hand._task_thread.is_alive()
    mock_hand.stop_task()
    time.sleep(0.1)
    assert not mock_hand._task_thread.is_alive()


def test_tension_move_motors_true(mock_hand):
    mock_hand.tension(move_motors=True, blocking=False)
    time.sleep(1)
    assert mock_hand._task_thread.is_alive()
    mock_hand.stop_task()
    time.sleep(0.1)
    assert not mock_hand._task_thread.is_alive()


def test_tension_interrupt_after_3_seconds(mock_hand):
    mock_hand.tension(move_motors=True, blocking=False)
    time.sleep(3)
    assert mock_hand._task_thread.is_alive(), "Tension task should still be running"
    mock_hand.stop_task()
    time.sleep(0.1)
    assert not mock_hand._task_thread.is_alive(), "Tension task should have stopped"


def test_second_tension_is_rejected(mock_hand):
    mock_hand.tension(move_motors=True, blocking=False)
    mock_hand.tension(move_motors=True, blocking=False)
    assert mock_hand._task_thread.is_alive()
    assert mock_hand._current_task == '_tension'
    mock_hand.stop_task()
    time.sleep(0.1)
    assert not mock_hand._task_thread.is_alive()
