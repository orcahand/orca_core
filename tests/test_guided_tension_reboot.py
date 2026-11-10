"""Guided tensioning around a motor reboot: a Dynamixel reboot re-reports a
multi-turn position within one turn, so scores and pulls must not jump a turn,
and only the hand's finger motors may be rebooted."""

import math
import threading

import pytest

from orca_core.constants import HARDWARE_VERSION_V21
from orca_core.maintenance import guided_tension as gt
from orca_core.maintenance.guided_tension import (
    FINISH,
    FINISH_ANYWAY,
    GOTO_TENSION,
    PULL_MORE,
    read_progress,
    reboot_and_hold,
    run_guided_tension,
)


@pytest.fixture
def hand(connected_mock_hand, monkeypatch):
    monkeypatch.setattr(gt, "SAMPLE_PERIOD_S", 0.002)
    h = connected_mock_hand
    h.config = h.config.with_hardware_version(HARDWARE_VERSION_V21)
    h.motor_client._min_pos, h.motor_client._max_motor_pos = -50.0, 50.0
    return h


def _reboot_one_turn_lower(client, latched=None):
    def reboot_motor(motor_id):
        client._torque_enabled[motor_id] = False
        client._pos[motor_id] -= 2 * math.pi
        if latched is not None:
            latched["on"] = False

    client.reboot_motor = reboot_motor


def _wait_spreads(seen, n=1):
    for _ in range(n):
        seen.clear()
        assert seen.wait(5)


def test_a_reboot_during_the_range_check_scores_the_real_sweep(hand):
    client = hand.motor_client
    target = hand.config.joint_to_motor_map["index_mcp"]
    latched = {"on": False}
    client.read_hardware_error = lambda mid: 0x20 if (mid == target and latched["on"]) else 0
    client.take_hardware_alerts = lambda: {target: 0xA0} if latched["on"] else {}
    _reboot_one_turn_lower(client, latched)
    seen, cleared = threading.Event(), threading.Event()

    def progress(event):
        if event["event"] == "spread":
            seen.set()
        elif event["event"] == "fault_cleared":
            cleared.set()

    def prompt(request):
        if request["action"] == "range_check" and request["stage"]["key"] == "index":
            centre = client._pos[target]
            for pos in (centre + math.radians(12), centre - math.radians(12), centre):
                client._pos[target] = pos
                _wait_spreads(seen)
            latched["on"] = True
            _wait_spreads(seen, 2)
            reboot_and_hold(hand, target)
            assert cleared.wait(5)
            _wait_spreads(seen, 5)
        if request["action"] == "score":
            return FINISH_ANYWAY if FINISH_ANYWAY in request["options"] else FINISH
        return request["options"][0]

    result = run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress, persist=True)
    row = next(m for m in result.spreads if m["joint"] == "index_mcp")
    assert row["status"] != "pass"
    assert read_progress(hand.config)["best"].get("index_mcp", 0.0) < 1.0


def test_pull_more_after_a_reboot_stays_within_the_turn_the_motor_reports(hand):
    client = hand.motor_client
    target = hand.config.joint_to_motor_map["index_mcp"]
    writes = []
    write = hand.write_motor_pos

    def spy(ids, pos):
        ids = list(ids)
        if target in ids:
            writes.append(float(pos[ids.index(target)]))
        return write(ids, pos)

    hand.write_motor_pos = spy
    _reboot_one_turn_lower(client)
    state = {"tension": 0}

    def prompt(request):
        if request["action"] == "pose" and state["tension"] == 0:
            return GOTO_TENSION
        if request["action"] == "tension":
            state["tension"] += 1
            if state["tension"] == 1:
                reboot_and_hold(hand, target)
                state["held"] = client._pos[target]
                writes.clear()
                return PULL_MORE
            raise KeyboardInterrupt
        return request["options"][0]

    with pytest.raises(KeyboardInterrupt):
        run_guided_tension(hand, prompt_callback=prompt, persist=False)
    assert writes and abs(math.degrees(writes[0] - state["held"])) < 45


@pytest.mark.parametrize("motor_id", ["wrist", 254, -1, 300])
def test_only_finger_motors_can_be_rebooted(hand, motor_id):
    rebooted = []
    hand.motor_client.reboot_motor = rebooted.append
    if motor_id == "wrist":
        motor_id = hand.config.joint_to_motor_map["wrist"]
    with pytest.raises(ValueError):
        reboot_and_hold(hand, motor_id)
    assert rebooted == []
