"""Tension sweep: the current a motor draws limit to limit is recorded against
its position; the level is the tension, a low stretch after the reversal is
slack. Needs a calibrated hand; torque off afterwards."""

import math

import numpy as np
import pytest

from orca_core.maintenance import tension_sweep
from orca_core.maintenance.tension_sweep import SweepSettings, run_tension_sweep, summarize


class _FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s


class _Read:
    def __init__(self, position, current):
        self.position = position
        self.current = current


class _Tendon:
    """The motor follows its goal exactly. Its current is ``load_ma`` once it
    has turned ``dead_deg`` away from the last reversal, near zero before."""

    def __init__(self, hand, load_ma, dead_deg):
        self.hand = hand
        self.load = load_ma
        self.dead = dead_deg
        self.pos = {m: float(p) for m, p in zip(hand.config.motor_ids, hand.get_motor_pos())}
        self.reversal = dict(self.pos)
        self.last_dir = {m: 0 for m in hand.config.motor_ids}
        self.currents = []

    def write_motor_pos(self, motor_ids, positions):
        for m, p in zip(motor_ids, positions):
            m, p = int(m), float(p)
            d = 1 if p > self.pos[m] else -1 if p < self.pos[m] else self.last_dir[m]
            if d != self.last_dir[m]:
                self.reversal[m] = self.pos[m]
                self.last_dir[m] = d
            self.pos[m] = p

    def set_max_current(self, current):
        self.currents.append(list(current) if isinstance(current, (list, tuple)) else current)

    def read_pos(self, hand):
        return np.array([self.pos[m] for m in hand.config.motor_ids])

    def read_state(self, hand, **kw):
        cur = []
        for m in hand.config.motor_ids:
            moved = math.degrees(abs(self.pos[m] - self.reversal[m]))
            cur.append(self.load.get(m, 0.0) * self.last_dir[m] if moved >= self.dead.get(m, 0.0) else 3.0)
        return _Read(self.read_pos(hand), np.array(cur))


@pytest.fixture()
def swept(connected_mock_hand, monkeypatch):
    hand = connected_mock_hand
    motor = hand.config.joint_to_motor_map
    tendon = _Tendon(hand, {motor["index_pip"]: 120.0, motor["ring_pip"]: 60.0},
                     {motor["index_pip"]: 0.0, motor["ring_pip"]: 20.0})
    for j in ("index_pip", "ring_pip"):
        hand.calibration.motor_limits_dict[motor[j]] = [-0.5, 1.5]  # 114.6 deg of travel
    monkeypatch.setattr(type(hand), "calibrated", property(lambda self: True))
    monkeypatch.setattr(hand, "write_motor_pos", tendon.write_motor_pos)
    monkeypatch.setattr(hand, "set_max_current", tendon.set_max_current)
    monkeypatch.setattr(tension_sweep, "read_motor_pos_checked", tendon.read_pos)
    monkeypatch.setattr(tension_sweep, "_read_state_checked", tendon.read_state)
    monkeypatch.setattr(tension_sweep, "_move_to_pose", lambda *a, **k: {})
    return hand, tendon


def test_the_sweep_records_the_load_level_and_the_dead_band(swept):
    hand, tendon = swept
    events = []
    report = run_tension_sweep(hand, joints=["index_pip", "ring_pip"], repeats=2,
                               progress_callback=events.append, clock=_FakeClock(),
                               settings=SweepSettings(speed_deg_s=60.0, sample_period_s=0.05))
    assert report["repeats"] == 2 and len(report["results"]) == 2 * 2 * 2
    assert report["travel_deg"]["index_pip"] == pytest.approx(math.degrees(2.0), abs=0.01)
    by = {(r["joint"], r["direction"], r["run"]): r for r in report["results"]}
    tight = by[("index_pip", 1, 1)]
    assert tight["mean_ma"] == pytest.approx(120.0)
    assert tight["deadband_deg"] < 10.0 and tight["short_deg"] == pytest.approx(0.0, abs=0.01)
    assert tight["samples"] and tight["samples"][-1][0] == pytest.approx(math.degrees(2.0) - 2.0, abs=0.01)
    assert tight["start_deg"] == pytest.approx(2.0, abs=0.01)
    loose = by[("ring_pip", -1, 2)]
    assert loose["mean_ma"] == pytest.approx(60.0)
    assert loose["deadband_deg"] == pytest.approx(20.0, abs=3.5)
    assert {r["sense"] for r in report["results"]} == {"flex", "extend"}
    plan = next(e for e in events if e["event"] == "sweep_plan")
    assert plan["total"] == 8 and plan["repeats"] == 2
    done = [e for e in events if e["event"] == "sweep_done"]
    assert [e["index"] for e in done] == list(range(1, 9))
    assert isinstance(done[0]["result"]["samples"], list)  # the curve rides along for live scoring
    assert any(e["event"] == "sweep_sample" for e in events)
    # The swept motor ran at the sweep current while the rest held; torque off at the end.
    idx = hand.config.motor_id_to_idx_dict
    sweeping = [c for c in tendon.currents if isinstance(c, list) and c[idx[hand.config.joint_to_motor_map["index_pip"]]] == 300.0]
    assert sweeping and all(c[idx[hand.config.joint_to_motor_map["middle_pip"]]] == 200.0 for c in sweeping)
    assert not any(hand.motor_client._torque_enabled.values())


def test_the_sweep_needs_limits_and_summarize_reads_a_curve(connected_mock_hand):
    hand = connected_mock_hand
    hand.calibration.motor_limits_dict[hand.config.joint_to_motor_map["index_pip"]] = [None, None]
    with pytest.raises(ValueError, match="index_pip"):
        run_tension_sweep(hand, joints=["index_pip"], clock=_FakeClock())
    curve = [[d, 5.0 if d < 15 else 90.0] for d in np.arange(0.0, 100.0, 1.0)]
    s = summarize(curve, 1, 100.0, deadband_ma=25.0)
    assert s["mean_ma"] == pytest.approx(90.0) and s["end_ma"] == pytest.approx(90.0)
    assert s["deadband_deg"] == pytest.approx(15.0) and s["peak_ma"] == 90.0
    assert summarize(curve, 1, 100.0, 25.0, start_deg=-2.0)["deadband_deg"] == pytest.approx(17.0)
    assert summarize([], 1, 100.0, 25.0)["mean_ma"] is None
    with pytest.raises(ValueError):
        SweepSettings(speed_deg_s=0.0).validate()


def test_a_group_sweeps_together_and_a_motor_on_its_stop_ends_early(swept):
    hand, tendon = swept
    motor = hand.config.joint_to_motor_map
    # index_pip cannot pass 1.2 rad: it is on a stop 0.3 rad short of the far turnaround.
    original = tendon.write_motor_pos

    def blocked(motor_ids, positions):
        positions = [min(float(p), 1.2) if int(m) == motor["index_pip"] else float(p)
                     for m, p in zip(motor_ids, positions)]
        original(motor_ids, positions)

    tendon.write_motor_pos = blocked
    hand.write_motor_pos = blocked
    events = []
    report = run_tension_sweep(hand, joints=["index_pip", "ring_pip"], progress_callback=events.append,
                               clock=_FakeClock(), settings=SweepSettings(speed_deg_s=60.0))
    assert report["parallel"] is True
    plan = next(e for e in events if e["event"] == "sweep_plan")
    assert plan["groups"] == [["index_pip", "ring_pip"]]
    started = [e for e in events if e["event"] == "sweep_started"]
    assert started[0]["group"] == ["index_pip", "ring_pip"] and started[0]["index"] == 1 and started[1]["index"] == 2
    by = {(r["joint"], r["direction"]): r for r in report["results"]}
    stuck = by[("index_pip", 1)]
    assert stuck["short_deg"] == pytest.approx(math.degrees(0.3) - 2.0, abs=0.5)  # short of the turnaround inside the 2° margin
    # Done half a second after it stopped moving, not at the walk's deadline.
    assert stuck["duration_s"] < 4.0
    assert by[("ring_pip", 1)]["short_deg"] == pytest.approx(0.0, abs=0.01)
    assert report["results"][0]["samples"][-1][0] == pytest.approx(math.degrees(1.2 + 0.5), abs=0.5)


def test_the_thumb_and_the_fingers_are_parked_out_of_each_others_way(swept):
    """A thumb joint sweeps with the fingers straight; a finger group sweeps
    with the thumb straight; both flex first, then extend."""
    hand, tendon = swept
    motor = hand.config.joint_to_motor_map
    hand.calibration.motor_limits_dict[motor["thumb_dip"]] = [-0.5, 1.5]
    seen = {}

    def progress(e):
        if e["event"] == "sweep_started" and e["sense"] == "flex" and e["joint"] not in seen:
            seen[e["joint"]] = dict(tendon.pos)

    run_tension_sweep(hand, joints=["thumb_dip", "index_pip"], progress_callback=progress,
                      clock=_FakeClock(), settings=SweepSettings(speed_deg_s=60.0))
    from orca_core.maintenance.tension_probe import direction_for
    flex = {j: direction_for(hand.config, j, "flex") for j in ("thumb_dip", "index_pip", "ring_pip")}
    # Straight = the extend limit plus the park margin, on the extend side.
    def straight(j):
        return (-0.5 if flex[j] > 0 else 1.5) + flex[j] * math.radians(5.0)
    assert seen["thumb_dip"][motor["index_pip"]] == pytest.approx(straight("index_pip"), abs=0.02)
    assert seen["thumb_dip"][motor["ring_pip"]] == pytest.approx(straight("ring_pip"), abs=0.02)
    assert seen["index_pip"][motor["thumb_dip"]] == pytest.approx(straight("thumb_dip"), abs=0.02)
    # Each sweep starts at the extend-side turnaround and flexes first.
    assert seen["index_pip"][motor["index_pip"]] == pytest.approx(-0.5 + math.radians(2.0) if flex["index_pip"] > 0 else 1.5 - math.radians(2.0), abs=0.02)
