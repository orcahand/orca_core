"""Tension probe: slack shows as a low breakaway current (and, with the
take-up method, as free travel); a taut tendon shows neither. Both methods,
both directions, repeated passes, other motors held, torque off afterwards."""

import math

import pytest

from orca_core.maintenance import tension_probe
from orca_core.maintenance.tension_probe import ProbeSettings, run_tension_probe


class _FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s


class _Tendons:
    """Spring-with-slack model per motor: the motor turns ``slack_deg`` freely,
    then 0.2 deg more per mA above 60 mA."""

    def __init__(self, hand, slack_deg):
        self.hand = hand
        self.slack = slack_deg
        self.goal = {}
        self.current = {m: 0.0 for m in hand.config.motor_ids}
        self.currents_seen = []
        # Mid-way between the calibrated limits, where a real hand rests; the mock
        # itself powers up at zero, outside them.
        limits = hand.calibration.motor_limits_dict
        self.start = {
            m: (
                sum(limits[m]) / 2
                if limits.get(m) and None not in limits[m]
                else float(p)
            )
            for m, p in zip(hand.config.motor_ids, hand.get_motor_pos())
        }

    def write_motor_pos(self, motor_ids, positions):
        for m, p in zip(motor_ids, positions):
            self.goal[int(m)] = float(p)

    def set_max_current(self, current):
        values = (
            list(current)
            if isinstance(current, (list, tuple))
            else [float(current)] * len(self.hand.config.motor_ids)
        )
        self.current = dict(zip(self.hand.config.motor_ids, values))
        self.currents_seen.append(dict(self.current))

    def read_pos(self, hand):
        import numpy as np

        out = []
        for m in hand.config.motor_ids:
            start = self.start[m]
            goal = self.goal.get(m, start)
            reach = math.radians(
                self.slack.get(m, 0.0) + max(0.0, self.current[m] - 60.0) * 0.2
            )
            delta = goal - start
            out.append(
                start + math.copysign(min(abs(delta), reach), delta) if delta else start
            )
        return np.array(out)


@pytest.fixture()
def probed(connected_mock_hand, monkeypatch):
    hand = connected_mock_hand
    motor = hand.config.joint_to_motor_map
    tendons = _Tendons(hand, {motor["index_pip"]: 8.0, motor["middle_pip"]: 0.0, motor["ring_pip"]: 0.0})
    monkeypatch.setattr(hand, "write_motor_pos", tendons.write_motor_pos)
    monkeypatch.setattr(hand, "set_max_current", tendons.set_max_current)
    monkeypatch.setattr(tension_probe, "read_motor_pos_checked", tendons.read_pos)
    return hand, tendons


def test_slack_reads_as_free_travel_and_a_low_breakaway(probed):
    hand, tendons = probed
    events = []
    report = run_tension_probe(
        hand,
        joints=["index_pip", "middle_pip"],
        methods=["takeup", "breakaway"],
        settings=ProbeSettings(hold_current_ma=200),
        progress_callback=events.append,
        clock=_FakeClock(),
    )

    by = {(r["joint"], r["direction"], r["method"]): r for r in report["results"]}
    assert len(by) == 2 * 2 * 2
    slack_takeup = by[("index_pip", 1, "takeup")]
    assert (
        slack_takeup["free_deg"] == pytest.approx(8.0, abs=0.1)
        and slack_takeup["stalled"]
    )
    tight_takeup = by[("middle_pip", 1, "takeup")]
    assert (
        tight_takeup["free_deg"] == pytest.approx(0.0, abs=0.1)
        and tight_takeup["stalled"]
    )

    slack_break = by[("index_pip", -1, "breakaway")]
    assert slack_break["breakaway_ma"] == 10.0  # the slack moves at the first step
    assert slack_break["move_ma"] == 70.0  # 8 + (70-60)*0.2 = 10 deg
    tight_break = by[("middle_pip", -1, "breakaway")]
    assert tight_break["breakaway_ma"] == 65.0  # 1 deg needs 5 mA over 60
    assert tight_break["move_ma"] == 110.0
    assert (
        tight_break["curve"][0] == [10.0, 0.0] and tight_break["curve"][-1][1] >= 10.0
    )
    assert all(r["run"] == 1 for r in report["results"]) and report["repeats"] == 1
    assert {r["sense"] for r in report["results"]} == {"flex", "extend"}

    # Every other motor was held at the hold current while one was probed.
    other = hand.config.joint_to_motor_map["ring_pip"]
    probing = [
        c
        for c in tendons.currents_seen
        if c[hand.config.joint_to_motor_map["index_pip"]] == 60.0
    ]
    assert probing and all(c[other] == 200.0 for c in probing)
    # Torque is off and the configured limit is back at the end.
    assert not any(hand.motor_client._torque_enabled.values())
    assert tendons.currents_seen[-1][other] == hand.config.max_current
    kinds = [e["event"] for e in events]
    assert kinds[0] == "probe_plan" and kinds.count("probe_done") == 8


def test_stop_unwinds_with_torque_off(probed):
    hand, _ = probed
    calls = {"n": 0}

    def should_stop():
        calls["n"] += 1
        return calls["n"] > 3

    with pytest.raises(InterruptedError):
        run_tension_probe(
            hand, joints=["index_pip"], should_stop=should_stop, clock=_FakeClock()
        )
    assert not any(hand.motor_client._torque_enabled.values())


def test_the_probe_stays_clear_of_a_calibrated_limit():
    from orca_core.maintenance.tension_probe import travel_cap_deg

    settings = ProbeSettings(max_travel_deg=40.0, limit_margin_deg=10.0)
    # 25 deg of room above the centre: 25 - 10 margin = 15, below the 40 cap.
    assert travel_cap_deg([0.0, math.radians(25.0)], 0.0, 1, settings) == pytest.approx(
        15.0
    )
    assert (
        travel_cap_deg([-math.radians(80.0), math.radians(25.0)], 0.0, -1, settings)
        == 40.0
    )
    assert travel_cap_deg(None, 0.0, 1, settings) == 40.0
    # A motor reporting its position one turn away from the calibrated limits
    # (the encoder wrapped) still gets its real room, not "-312 deg".
    for turns in (-1, 1, 2):
        shifted = math.radians(-20.0) + turns * 2 * math.pi
        assert travel_cap_deg(
            [-math.radians(80.0), math.radians(25.0)], shifted, 1, settings
        ) == pytest.approx(35.0)
        assert travel_cap_deg(
            [-math.radians(80.0), math.radians(25.0)], shifted, -1, settings
        ) == pytest.approx(40.0)
    with pytest.raises(ValueError):
        ProbeSettings(play_deg=12.0, move_deg=10.0).validate()


def test_the_default_start_pose_is_mid_rom_and_hold_skips_the_move(probed, monkeypatch):
    hand, _ = probed
    moves = []
    monkeypatch.setattr(hand, "set_joint_positions", lambda pos, **kw: moves.append(dict(pos)))
    events = []
    run_tension_probe(hand, joints=["index_pip"], methods=["takeup"], clock=_FakeClock(),
                      progress_callback=events.append)
    assert moves and set(moves[0]) == {"index_pip"}
    rom = hand.config.joint_roms_dict["index_pip"]
    assert moves[0]["index_pip"] == pytest.approx((rom[0] + rom[1]) / 2)
    plan = next(e for e in events if e["event"] == "probe_plan")
    assert plan["pose"] == "mid"

    moves.clear()
    run_tension_probe(hand, joints=["index_pip"], methods=["takeup"], clock=_FakeClock(), pose="neutral")
    assert moves[0]["index_pip"] == pytest.approx(float(hand.config.neutral_position.get("index_pip", 0.0)))

    moves.clear()
    run_tension_probe(hand, joints=["index_pip"], methods=["takeup"], clock=_FakeClock(), pose="hold")
    assert moves == []
    with pytest.raises(ValueError):
        run_tension_probe(hand, joints=["index_pip"], clock=_FakeClock(), pose="sideways")


def test_posing_uses_the_pose_current_and_reports_joints_left_short(probed, monkeypatch):
    hand, tendons = probed
    motor = hand.config.joint_to_motor_map["index_pip"]
    seen = []
    monkeypatch.setattr(hand, "set_joint_positions", lambda pos, **kw: seen.append(dict(tendons.current)))
    # A stiff joint that the pose move cannot reach: the motor stays put.
    monkeypatch.setattr(hand, "_joint_to_motor_pos", lambda pos: [
        tendons.start[m] + math.radians(30.0) if m == motor else None
        for m in hand.config.motor_ids
    ])
    events = []
    run_tension_probe(hand, joints=["index_pip"], methods=["takeup"], clock=_FakeClock(),
                      settings=ProbeSettings(pose_current_ma=350.0), progress_callback=events.append)
    assert seen and seen[0][motor] == 350.0
    short = next(e for e in events if e["event"] == "pose_short")
    assert short["joints"] == {"index_pip": 30.0}
    # Only the probed motor gets the pose current; the rest hold.
    assert all(v == 200.0 for m, v in seen[0].items() if m != motor)


def test_repeats_run_the_whole_sequence_again_from_the_same_pose(probed):
    hand, tendons = probed
    events = []
    report = run_tension_probe(
        hand,
        joints=["index_pip"],
        methods=["breakaway"],
        repeats=3,
        progress_callback=events.append,
        clock=_FakeClock(),
    )
    assert report["repeats"] == 3 and len(report["results"]) == 2 * 3
    assert [r["run"] for r in report["results"]] == [1, 1, 2, 2, 3, 3]
    plan = next(e for e in events if e["event"] == "probe_plan")
    assert plan["total"] == 6 and plan["repeats"] == 3
    starts = [e for e in events if e["event"] == "probe_started"]
    assert [e["index"] for e in starts] == list(range(1, 7))
    assert [e["run"] for e in starts] == [1, 1, 2, 2, 3, 3]
    # The same tendon gives the same breakaway every pass.
    assert len({r["breakaway_ma"] for r in report["results"] if r["direction"] == 1}) == 1
    with pytest.raises(ValueError):
        run_tension_probe(hand, joints=["index_pip"], repeats=0, clock=_FakeClock())


def test_ignoring_the_limits_probes_every_side_the_full_cap(probed):
    hand, tendons = probed
    motor = hand.config.joint_to_motor_map["index_pip"]
    # Limits that leave no room on the positive side would skip that probe.
    hand.calibration.motor_limits_dict[motor] = [tendons.start[motor] - 1.0, tendons.start[motor] + 0.01]
    kept = run_tension_probe(hand, joints=["index_pip"], methods=["breakaway"], pose="hold", clock=_FakeClock())
    assert [r["skipped"] is not None for r in kept["results"]] == [True, False]
    free = run_tension_probe(hand, joints=["index_pip"], methods=["breakaway"], pose="hold",
                             clock=_FakeClock(), use_limits=False)
    assert all(r["skipped"] is None and r["cap_deg"] == 40.0 for r in free["results"])
    assert free["use_limits"] is False and kept["use_limits"] is True



def test_parallel_ramps_one_joint_kind_together_and_sends_each_home(probed):
    from orca_core.maintenance.tension_probe import probe_groups

    hand, tendons = probed
    motor = hand.config.joint_to_motor_map
    joints = ["index_pip", "ring_pip", "index_mcp", "thumb_dip"]
    # The thumb alone; the four fingers together per joint kind.
    assert probe_groups(joints) == [["thumb_dip"], ["index_mcp"], ["index_pip", "ring_pip"]]
    assert probe_groups(joints, parallel=False) == [[j] for j in joints]
    assert probe_groups(["index_abd", "middle_abd", "index_mcp", "middle_mcp"]) == [
        ["index_abd", "middle_abd"], ["index_mcp", "middle_mcp"]]
    events = []
    report = run_tension_probe(
        hand, joints=joints, methods=["breakaway"], parallel=True,
        progress_callback=events.append, clock=_FakeClock(),
    )
    assert report["parallel"] is True and len(report["results"]) == 8
    by = {(r["joint"], r["direction"]): r for r in report["results"]}
    # The same currents as one at a time: the slack joint moves at the first step.
    assert by[("index_pip", -1)]["breakaway_ma"] == 10.0 and by[("index_pip", -1)]["move_ma"] == 70.0
    assert by[("ring_pip", -1)]["breakaway_ma"] == 65.0 and by[("ring_pip", -1)]["move_ma"] == 110.0
    # The paired PIP motors ramped under one current limit at the same time ...
    pips = [motor[j] for j in ("index_pip", "ring_pip")]
    together = [c for c in tendons.currents_seen if all(c[m] == 60.0 for m in pips)]
    assert together
    # ... and a motor that had moved was sent home while the other kept ramping.
    after = [c for c in tendons.currents_seen if c[motor["ring_pip"]] == 100.0]
    assert after and all(c[motor["index_pip"]] == 200.0 for c in after)
    started = [e for e in events if e["event"] == "probe_started"]
    assert started[0]["group"] == ["thumb_dip"] and started[2]["group"] == ["index_mcp"]
    assert started[4]["group"] == ["index_pip", "ring_pip"]
    # A group moves one sense at a time: flex for both, then extend for both.
    assert [e["sense"] for e in started[4:8]] == ["flex", "flex", "extend", "extend"]
    assert [e["index"] for e in events if e["event"] == "probe_done"] == list(range(1, 9))
    assert all(abs(tendons.goal[m] - tendons.start[m]) < 1e-9 for m in pips)


def test_the_stroke_moves_a_whole_group_at_once_under_one_current(probed, monkeypatch):
    """The tension check's method: the group's motors are driven together,
    position-controlled, and each one's mean current over the stroke is the
    reading; nothing waits for a breakaway."""
    import numpy as np

    hand, tendons = probed
    motor = hand.config.joint_to_motor_map

    class _State:
        def __init__(self, position, current):
            self.position, self.current = position, current

    def read_state(h, clock):
        return _State(tendons.read_pos(h), np.array([tendons.current[m] for m in h.config.motor_ids]))

    monkeypatch.setattr(tension_probe, "_read_state", read_state)
    events = []
    report = run_tension_probe(
        hand, joints=["index_pip", "ring_pip"], methods=["stroke"], parallel=True, use_limits=False,
        settings=ProbeSettings(stroke_current_ma=150.0), progress_callback=events.append, clock=_FakeClock(),
    )
    by = {(r["joint"], r["sense"]): r for r in report["results"]}
    assert len(by) == 4 and all(r["method"] == "stroke" for r in report["results"])
    # Both motors ran under the stroke current in the same command, start to finish.
    pips = [motor["index_pip"], motor["ring_pip"]]
    together = [c for c in tendons.currents_seen if all(c[m] == 150.0 for m in pips)]
    assert together and all(c[motor["middle_pip"]] == 200.0 for c in together)
    # The fake reaches slack + 18° at 150 mA: the slack tendon gets further, both stall short of the cap.
    assert by[("index_pip", "flex")]["free_deg"] == pytest.approx(26.0, abs=0.6)
    assert by[("ring_pip", "flex")]["free_deg"] == pytest.approx(18.0, abs=0.6)
    assert all(r["stalled"] for r in report["results"])
    assert by[("index_pip", "flex")]["mean_ma"] == 150.0 and by[("index_pip", "flex")]["breakaway_ma"] == 150.0
    started = [e for e in events if e["event"] == "probe_started"]
    assert started[0]["group"] == ["index_pip", "ring_pip"]
    assert all(abs(tendons.goal[m] - tendons.start[m]) < 1e-9 for m in pips)
