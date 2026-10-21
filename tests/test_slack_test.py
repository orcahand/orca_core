"""Slack test: a spool turns at the test current only as far as its slack lets
it; one that completes the half turn blocks calibration. Torque off afterwards."""

import math

import numpy as np
import pytest

from orca_core.maintenance import slack_test
from orca_core.maintenance.slack_test import SlackTestSettings, run_slack_test


class _FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s


class _Spools:
    """Each motor turns ``slack_deg`` freely from where it starts, in either
    direction, and not a degree more at a current under 60 mA."""

    def __init__(self, hand, slack_deg):
        self.hand = hand
        self.slack = slack_deg
        self.goal = {}
        self.current = {m: 0.0 for m in hand.config.motor_ids}
        self.start = {m: float(p) for m, p in zip(hand.config.motor_ids, hand.get_motor_pos())}

    def write_motor_pos(self, motor_ids, positions):
        for m, p in zip(motor_ids, positions):
            self.goal[int(m)] = float(p)

    def set_max_current(self, current):
        values = list(current) if isinstance(current, (list, tuple)) else [float(current)] * len(self.hand.config.motor_ids)
        self.current = dict(zip(self.hand.config.motor_ids, values))

    def read_pos(self, hand):
        out = []
        for m in hand.config.motor_ids:
            start, goal = self.start[m], self.goal.get(m, self.start[m])
            reach = math.radians(self.slack.get(m, 0.0) + max(0.0, self.current[m] - 60.0) * 0.2)
            delta = goal - start
            out.append(start + math.copysign(min(abs(delta), reach), delta) if delta else start)
        return np.array(out)


@pytest.fixture()
def spooled(connected_mock_hand, monkeypatch):
    hand = connected_mock_hand
    motor = hand.config.joint_to_motor_map
    spools = _Spools(hand, {motor["index_pip"]: 12.0, motor["ring_mcp"]: 400.0})
    monkeypatch.setattr(hand, "write_motor_pos", spools.write_motor_pos)
    monkeypatch.setattr(hand, "set_max_current", spools.set_max_current)
    monkeypatch.setattr(slack_test, "read_motor_pos_checked", spools.read_pos)
    return hand, spools


def test_slack_shows_as_free_rotation_and_a_half_turn_blocks(spooled):
    hand, spools = spooled
    events = []
    report = run_slack_test(hand, progress_callback=events.append, clock=_FakeClock())
    by = {r["joint"]: r for r in report["results"]}
    assert by["index_pip"]["plus_deg"] == pytest.approx(12.0, abs=0.1)
    assert by["index_pip"]["minus_deg"] == pytest.approx(12.0, abs=0.1)
    assert by["index_pip"]["total_deg"] == pytest.approx(24.0, abs=0.2)
    assert by["index_pip"]["status"] == "slack" and not by["index_pip"]["plus_capped"]
    assert by["middle_pip"]["total_deg"] == pytest.approx(0.0) and by["middle_pip"]["status"] == "taut"
    assert by["ring_mcp"]["plus_capped"] and by["ring_mcp"]["status"] == "unspool"
    assert by["ring_mcp"]["plus_deg"] <= 180.0
    assert report["blocked"] is True
    assert {by["index_pip"]["plus_sense"], by["index_pip"]["minus_sense"]} == {"flex", "extend"}
    assert "wrist" not in by
    # Torque is off and the configured limit is back at the end.
    assert not any(hand.motor_client._torque_enabled.values())
    assert spools.current[hand.config.joint_to_motor_map["index_pip"]] == hand.config.max_current
    kinds = [e["event"] for e in events]
    assert kinds[0] == "slack_plan" and kinds.count("slack_phase_done") == 2
    phases = [e["direction"] for e in events if e["event"] == "slack_phase"]
    assert phases == [1, -1]


def test_a_taut_hand_passes_and_a_stop_leaves_torque_off(spooled):
    hand, spools = spooled
    report = run_slack_test(hand, joints=["index_pip", "middle_pip"], clock=_FakeClock(),
                            settings=SlackTestSettings(slack_tolerance_deg=30.0))
    assert report["blocked"] is False
    assert {r["status"] for r in report["results"]} == {"taut"}
    calls = {"n": 0}

    def should_stop():
        calls["n"] += 1
        return calls["n"] > 3

    with pytest.raises(InterruptedError):
        run_slack_test(hand, should_stop=should_stop, clock=_FakeClock())
    assert not any(hand.motor_client._torque_enabled.values())
    with pytest.raises(ValueError):
        SlackTestSettings(max_turn_deg=360.0).validate()
