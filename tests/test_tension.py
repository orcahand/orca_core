import os
import shutil
import time

import pytest


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


def test_run_tension_exception_restores_hand_state(tmp_path):
    """An exception mid-winding must disable torque and restore the configured
    control mode and current limit, not leave the hand straining."""
    import orca_core
    from orca_core import MockOrcaHand
    from orca_core.maintenance.tensioning import run_tension
    from orca_core.utils import update_yaml

    model_config = os.path.join(
        os.path.dirname(orca_core.__file__),
        "models", "v2", "orcahand-right", "config.yaml",
    )
    config_path = tmp_path / "config.yaml"
    shutil.copy(model_config, config_path)
    update_yaml(str(config_path), "max_current", 250)
    update_yaml(str(config_path), "calibration_current", 200)
    update_yaml(str(config_path), "control_mode", "position")

    hand = MockOrcaHand(config_path=str(config_path))
    success, msg = hand.connect()
    assert success, msg
    try:
        def boom(*args, **kwargs):
            raise RuntimeError("bus died")

        hand.get_motor_pos = boom

        events = []
        with pytest.raises(RuntimeError, match="bus died"):
            run_tension(hand, move_motors=True, progress_callback=events.append)

        client = hand.motor_client
        assert not any(client._torque_enabled.values()), "torque left enabled"
        assert set(client._operating_mode.values()) == {3}, "control mode not restored"
        assert all(c == 250 for c in client._cur.values()), "current limit not restored"
        phases = [e["phase"] for e in events if e["event"] == "phase"]
        assert phases[-1] == "released"
    finally:
        del hand.get_motor_pos
        hand.disconnect()


def test_run_tension_holding_prints_nothing(connected_mock_hand, capsys):
    """Operator guidance is the CLI's job; the packaged routine must not print
    terminal-only instructions like the Ctrl+C hint."""
    from orca_core.maintenance.tensioning import run_tension

    capsys.readouterr()
    run_tension(connected_mock_hand, move_motors=False, should_stop=lambda: True)
    assert capsys.readouterr().out == ""
