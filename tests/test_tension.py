import time

def test_tension_move_motors_false(connected_mock_hand):
    connected_mock_hand.tension(move_motors=False, blocking=False)
    assert connected_mock_hand._task_thread.is_alive()
    connected_mock_hand.stop_task()
    assert not connected_mock_hand._task_thread.is_alive()


def test_tension_move_motors_true(connected_mock_hand):
    connected_mock_hand.tension(move_motors=True, blocking=False)
    time.sleep(1)
    assert connected_mock_hand._task_thread.is_alive()
    connected_mock_hand.stop_task()
    time.sleep(0.1)
    assert not connected_mock_hand._task_thread.is_alive()


def test_tension_interrupt_after_3_seconds(connected_mock_hand):
    connected_mock_hand.tension(move_motors=True, blocking=False)
    time.sleep(3)
    assert connected_mock_hand._task_thread.is_alive(), "Tension task should still be running"
    connected_mock_hand.stop_task()
    time.sleep(0.1)
    assert not connected_mock_hand._task_thread.is_alive(), "Tension task should have stopped"


def test_second_tension_is_rejected(connected_mock_hand):
    connected_mock_hand.tension(move_motors=True, blocking=False)
    connected_mock_hand.tension(move_motors=True, blocking=False)
    assert connected_mock_hand._task_thread.is_alive()
    assert connected_mock_hand._current_task == "_tension"
    connected_mock_hand.stop_task()
    time.sleep(0.1)
    assert not connected_mock_hand._task_thread.is_alive()
