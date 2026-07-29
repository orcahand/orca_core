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


def test_tension_emits_phase_events(connected_mock_hand):
    import threading

    events = []
    holding = threading.Event()

    def cb(event):
        events.append(event)
        if event.get("phase") == "holding":
            holding.set()

    thread = threading.Thread(
        target=lambda: connected_mock_hand.tension(
            blocking=True, progress_callback=cb
        )
    )
    thread.start()
    assert holding.wait(timeout=120), f"never reached holding: {events}"
    connected_mock_hand._task_stop_event.set()
    thread.join(timeout=10)
    assert not thread.is_alive()

    phases = [e["phase"] for e in events if e["event"] == "phase"]
    assert phases[0] == "winding"
    assert "ramp" in phases and "holding" in phases
    assert phases[-1] == "released"
    assert any(e["event"] == "winding_progress" for e in events)
