"""Slack test: with the joint held, a spool turns only as far as its slack lets
it; more than half a turn fails the joint and blocks calibration. Torque off
afterwards."""

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
    """With its joint held, each motor turns ``slack_deg`` freely from where it
    starts, in either direction, and no further."""

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
            reach = math.radians(self.slack.get(m, 0.0))
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


def test_held_joints_report_their_free_rotation_and_too_much_fails(spooled):
    hand, spools = spooled
    events, held = [], []
    report = run_slack_test(hand, progress_callback=events.append, clock=_FakeClock(),
                            hold_callback=lambda j: held.append(j) or True)
    by = {r["joint"]: r for r in report["results"]}
    assert held == report["joints"] and "wrist" not in by
    assert by["index_pip"]["plus_deg"] == pytest.approx(12.0, abs=0.1)
    assert by["index_pip"]["minus_deg"] == pytest.approx(12.0, abs=0.1)
    assert by["index_pip"]["status"] == "pass"
    assert by["middle_pip"]["total_deg"] == pytest.approx(0.0) and by["middle_pip"]["status"] == "pass"
    # Past the half turn the joint fails, and the spool is never wound much further.
    assert by["ring_mcp"]["status"] == "fail" and by["ring_mcp"]["total_deg"] <= 186.0
    assert report["blocked"] is True
    assert {by["index_pip"]["plus_sense"], by["index_pip"]["minus_sense"]} == {"flex", "extend"}
    # Torque is off and the configured limit is back at the end.
    assert not any(hand.motor_client._torque_enabled.values())
    assert spools.current[hand.config.joint_to_motor_map["index_pip"]] == hand.config.max_current
    kinds = [e["event"] for e in events]
    assert kinds[0] == "slack_plan" and kinds.count("slack_joint_done") == len(report["joints"])


def test_slack_adds_up_over_both_directions(spooled):
    hand, spools = spooled
    spools.slack[hand.config.joint_to_motor_map["index_pip"]] = 100.0
    report = run_slack_test(hand, joints=["index_pip"], clock=_FakeClock())
    (r,) = report["results"]
    assert r["plus_deg"] == pytest.approx(100.0, abs=0.1)
    assert r["status"] == "fail" and report["blocked"] is True


def test_a_skipped_joint_is_not_moved_and_a_stop_leaves_torque_off(spooled):
    hand, spools = spooled
    report = run_slack_test(hand, joints=["index_pip", "ring_mcp"], clock=_FakeClock(),
                            hold_callback=lambda j: j != "ring_mcp")
    by = {r["joint"]: r for r in report["results"]}
    assert by["ring_mcp"]["status"] == "skipped" and by["ring_mcp"]["total_deg"] == 0.0
    assert report["blocked"] is False
    calls = {"n": 0}

    def should_stop():
        calls["n"] += 1
        return calls["n"] > 3

    with pytest.raises(InterruptedError):
        run_slack_test(hand, should_stop=should_stop, clock=_FakeClock())
    assert not any(hand.motor_client._torque_enabled.values())
    with pytest.raises(ValueError):
        SlackTestSettings(max_free_deg=360.0).validate()


def test_back_discards_the_previous_joint_and_tests_it_again(spooled):
    hand, spools = spooled
    answers = iter([False, slack_test.HOLD_BACK, True, True])
    events, held = [], []
    report = run_slack_test(hand, joints=["index_pip", "ring_mcp"], clock=_FakeClock(),
                            progress_callback=events.append,
                            hold_callback=lambda j: held.append(j) or next(answers))
    assert held == ["index_pip", "ring_mcp", "index_pip", "ring_mcp"]
    by = {r["joint"]: r for r in report["results"]}
    assert by["index_pip"]["status"] == "pass" and by["index_pip"]["plus_deg"] == pytest.approx(12.0, abs=0.1)
    assert by["ring_mcp"]["status"] == "fail"
    assert [e["joint"] for e in events if e["event"] == "slack_joint_back"] == ["index_pip"]
