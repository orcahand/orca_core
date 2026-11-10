"""Guided tensioning: spreads are scored against the travel baseline, the wrist
is left alone, and every exit leaves the motors limp.

Fraction scoring (the default) keeps each joint's best score across rounds;
slack scoring counts only settled, twice-reached ends, in degrees, and carries
nothing over between rounds."""

import dataclasses
import glob
import importlib.util
import math
import os
import threading
from pathlib import Path

import pytest

import orca_core
from orca_core.constants import SUPPORTED_MOTOR_TYPES
from orca_core.hand_config import OrcaHandConfig
from orca_core.hardware.motor_factory import motor_client_class
from orca_core.maintenance import guided_tension
from orca_core.maintenance.guided_tension import (
    DONE,
    FINISH,
    FINISH_ANYWAY,
    SETTLE_SAMPLES,
    TENSION,
    MotorSpread,
    joint_currents,
    read_last_tension,
    read_progress,
    record_sample,
    run_guided_tension,
    tension_targets,
)
from orca_core.constants import HARDWARE_VERSION_V21
from orca_core.maintenance.tensioning import uncapped_currents


@pytest.fixture
def connected_mock_hand(connected_mock_hand, monkeypatch):
    """The guided routine scores against the v2.1 baseline, so run it as v2.1;
    sample fast so the settle-and-repeat sweeps stay quick."""
    monkeypatch.setattr(guided_tension, "SAMPLE_PERIOD_S", 0.002)
    hand = connected_mock_hand
    hand.config = hand.config.with_hardware_version(HARDWARE_VERSION_V21)
    return hand


def _drive(hand, drag, any_hardware=False):
    """Prompt answers that, in each stage, drag that stage's joints' motors by
    ``drag(joint)`` x their travel and back (fraction scoring: every sample counts)."""
    spread_seen = threading.Event()
    client = hand.motor_client
    targets = tension_targets(hand.config, any_hardware=any_hardware)

    def progress(event):
        if event["event"] == "spread":
            spread_seen.set()

    def prompt(p):
        if p["action"] == "range_check":
            assert all(client._torque_enabled[t.motor_id] for t in targets)
            for t in targets:
                if t.joint not in p["stage"]["joints"]:
                    continue
                half = math.radians(t.expected_deg * drag(t.joint)) / 2
                centre = client._pos[t.motor_id]
                for pos in (centre + half, centre - half, centre):
                    client._pos[t.motor_id] = pos
                    spread_seen.clear()
                    assert spread_seen.wait(5)
        return (
            FINISH
            if FINISH in p["options"]
            else (FINISH_ANYWAY if FINISH_ANYWAY in p["options"] else p["options"][0])
        )

    return prompt, progress


# ----- fraction scoring (default) -------------------------------------------


def test_wrist_is_never_a_target(connected_mock_hand):
    assert "wrist" not in {t.joint for t in tension_targets(connected_mock_hand.config)}


def test_partial_spread_scores_short_and_releases(connected_mock_hand):
    hand = connected_mock_hand
    prompt, progress = _drive(hand, lambda j: 0.9 if j == "index_mcp" else 0.4)
    result = run_guided_tension(
        hand, prompt_callback=prompt, progress_callback=progress, persist=True
    )

    by_joint = {m["joint"]: m for m in result.spreads}
    assert by_joint["index_mcp"]["status"] == "pass"
    assert by_joint["middle_mcp"]["status"] == "partial"
    assert by_joint["index_mcp"]["scoring"] == "fraction"
    assert by_joint["index_mcp"]["pass_fraction"] == 0.85
    assert not result.completed
    assert not any(hand.motor_client._torque_enabled.values())
    assert read_last_tension(hand.config) is None


def test_full_spread_completes_and_records(connected_mock_hand):
    hand = connected_mock_hand
    prompt, progress = _drive(hand, lambda j: 0.9)
    result = run_guided_tension(
        hand, prompt_callback=prompt, progress_callback=progress, persist=True
    )

    assert result.completed and result.rounds == 1
    last = read_last_tension(hand.config)
    assert last["spreads"]["index_mcp"] == pytest.approx(0.9, abs=0.01)
    assert last["scoring"] == "fraction" and last["pass_fraction"]["index_mcp"] == 0.85


def test_an_older_hand_is_refused_unless_overridden(connected_mock_hand):
    hand = connected_mock_hand
    hand.config = hand.config.with_hardware_version(2)
    with pytest.raises(ValueError, match="needs v2.1 hardware"):
        run_guided_tension(hand, prompt_callback=lambda p: p["options"][0])

    prompt, progress = _drive(hand, lambda j: 0.9, any_hardware=True)
    result = run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=False,
        any_hardware=True,
    )
    assert result.completed


def test_a_joint_asked_to_test_again_loses_its_kept_pass(connected_mock_hand):
    hand = connected_mock_hand
    prompt, progress = _drive(hand, lambda j: 0.9 if j == "index_mcp" else 0.2)
    run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress, persist=True, hand_key="ser-0003")
    asks = [{"index_mcp"}]
    prompt, progress = _drive(hand, lambda j: 0.0)
    result = run_guided_tension(
        hand, prompt_callback=prompt, progress_callback=progress, persist=True, hand_key="ser-0003",
        forget=lambda: asks.pop() if asks else set(),
    )
    by_joint = {m["joint"]: m for m in result.spreads}
    assert by_joint["index_mcp"]["kept"] is None and by_joint["index_mcp"]["status"] != "pass"
    assert read_progress(hand.config, "ser-0003")["best"]["index_mcp"] < 0.1


def test_stop_during_prompt_releases(connected_mock_hand):
    hand = connected_mock_hand
    stop = threading.Event()

    def prompt(p):
        stop.set()
        return p["options"][0]

    with pytest.raises(InterruptedError):
        run_guided_tension(hand, prompt_callback=prompt, should_stop=stop.is_set)
    assert not any(hand.motor_client._torque_enabled.values())


def test_progress_carries_over_per_hand_and_resets_from_scratch(connected_mock_hand):
    hand = connected_mock_hand
    prompt, progress = _drive(hand, lambda j: 0.9 if j == "index_mcp" else 0.2)
    run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0001",
    )
    assert read_progress(hand.config, "ser-0001")["best"]["index_mcp"] == pytest.approx(
        0.9, abs=0.01
    )
    assert read_progress(hand.config, "ser-0002") is None

    # A resumed run keeps index_mcp passed without moving it again.
    prompt, progress = _drive(hand, lambda j: 0.0)
    result = run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0001",
    )
    by_joint = {m["joint"]: m for m in result.spreads}
    assert by_joint["index_mcp"]["status"] == "pass"
    assert by_joint["index_mcp"]["kept"] == pytest.approx(0.9, abs=0.01)

    prompt, progress = _drive(hand, lambda j: 0.0)
    result = run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0001",
        resume=False,
    )
    assert {m["joint"]: m for m in result.spreads}["index_mcp"]["status"] == "low"


def test_stages_cover_every_finger_joint_and_skip_what_passed(connected_mock_hand):
    from orca_core.maintenance.guided_tension import DEFAULT_RANGE_STAGES

    hand = connected_mock_hand
    staged = [j for st in DEFAULT_RANGE_STAGES for j in st.joints]
    assert sorted(staged) == sorted(t.joint for t in tension_targets(hand.config))

    prompt, progress = _drive(
        hand, lambda j: 0.9 if j.startswith(("thumb", "index")) else 0.0
    )
    run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0003",
    )
    posed = []
    inner_prompt, progress = _drive(hand, lambda j: 0.0)

    def prompt2(p):
        if p["action"] == "pose":
            posed.append(p["stage"]["key"])
        return inner_prompt(p)

    run_guided_tension(
        hand,
        prompt_callback=prompt2,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0003",
    )
    assert posed == ["middle", "ring", "pinky"]


def test_back_and_goto_move_between_stages(connected_mock_hand):
    seen = []
    script = iter(
        [
            "Continue",
            "Next",
            "Back",
            "Continue",
            "Back",
            "goto:pinky",
            "Continue",
            "Score",
            "goto:middle",
            "Continue",
            "Next",
            "Continue",
            "Next",
            "Continue",
            "Score",
            FINISH_ANYWAY,
        ]
    )

    def prompt(p):
        seen.append((p["action"], p.get("stage", {}).get("key")))
        answer = next(script)
        assert answer in p["options"]
        return answer

    result = run_guided_tension(
        connected_mock_hand, prompt_callback=prompt, persist=False, resume=False
    )
    assert seen == [
        ("pose", "thumb"),
        ("range_check", "thumb"),
        ("pose", "index"),
        ("pose", "thumb"),
        ("range_check", "thumb"),
        ("pose", "thumb"),
        ("pose", "pinky"),
        ("range_check", "pinky"),
        ("score", None),
        ("pose", "middle"),
        ("range_check", "middle"),
        ("pose", "ring"),
        ("range_check", "ring"),
        ("pose", "pinky"),
        ("range_check", "pinky"),
        ("score", None),
    ]
    assert result.rounds == 1
    assert not any(connected_mock_hand.motor_client._torque_enabled.values())


def test_any_stage_and_the_tension_step_are_one_click_away(connected_mock_hand):
    hand = connected_mock_hand
    client = hand.motor_client
    targets = tension_targets(hand.config)
    seen, pulled = [], {}

    def prompt(p):
        seen.append((p["action"], p.get("stage", {}).get("key")))
        if p["action"] == "tension":
            pulled.update({t.joint: client._pos[t.motor_id] for t in targets})
        return answers.pop(0)

    # Jump straight to tension, pull once more, open round two on the ring, score, finish.
    answers = ["goto:tension", "Pull more", "goto:ring", "goto:score", FINISH_ANYWAY]
    centre = {t.joint: client._pos[t.motor_id] for t in targets}
    result = run_guided_tension(
        hand,
        prompt_callback=prompt,
        persist=False,
        resume=False,
        tension_offset_deg=15.0,
        tension_extra_deg=15.0,
    )
    assert seen == [
        ("pose", "thumb"),
        ("tension", None),
        ("tension", None),
        ("pose", "ring"),
        ("score", None),
    ]
    assert result.rounds == 2
    assert all(
        pulled[j] == pytest.approx(centre[j] + math.radians(30.0)) for j in centre
    )
    assert not any(client._torque_enabled.values())


def test_a_passed_stage_is_skipped_but_can_be_jumped_to(connected_mock_hand):
    hand = connected_mock_hand
    prompt, progress = _drive(
        hand, lambda j: 0.9 if j.startswith(("thumb", "index")) else 0.0
    )
    run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0004",
    )
    seen = []
    answers = [
        "goto:thumb",
        "Continue",
        "Next",
        "Continue",
        "Next",
        "Continue",
        "Next",
        "Continue",
        "Score",
        FINISH_ANYWAY,
    ]

    def prompt(p):
        seen.append((p["action"], p.get("stage", {}).get("key")))
        assert answers[0] in p["options"]
        return answers.pop(0)

    run_guided_tension(hand, prompt_callback=prompt, persist=True, hand_key="ser-0004")
    assert seen[:5] == [
        ("pose", "middle"),
        ("pose", "thumb"),
        ("range_check", "thumb"),
        ("pose", "middle"),
        ("range_check", "middle"),
    ]


def test_jumping_to_tension_with_no_pull_still_holds(connected_mock_hand):
    hand = connected_mock_hand
    targets = tension_targets(hand.config)
    held = []
    answers = ["goto:tension", "Done", "goto:score", FINISH_ANYWAY]

    def prompt(p):
        if p["action"] == "tension":
            held.append(
                all(hand.motor_client._torque_enabled[t.motor_id] for t in targets)
            )
        return answers.pop(0)

    run_guided_tension(
        hand, prompt_callback=prompt, persist=False, resume=False, tension_offset_deg=0
    )
    assert held == [True]
    assert not any(hand.motor_client._torque_enabled.values())


def test_hold_currents_come_from_the_config_and_a_named_joint_keeps_its_own(
    connected_mock_hand,
):
    config = dataclasses.replace(
        connected_mock_hand.config,
        guided_tension_currents_dict={
            "dynamixel": {"hold": {"mcp": 150, "thumb_mcp": 84}, "tension": 420}
        },
    )
    from_config = joint_currents(config)
    flat = joint_currents(config, 100)
    grouped = joint_currents(config, lambda j: 140 if j.endswith("_mcp") else 200)
    assert from_config["thumb_mcp"] == flat["thumb_mcp"] == grouped["thumb_mcp"] == 84
    assert (from_config["index_mcp"], flat["index_mcp"], grouped["index_mcp"]) == (
        150,
        100,
        140,
    )
    assert from_config["index_pip"] == 100 and config.tension_current == 420


V2_MODELS = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2")


def _packaged_config(model):
    return OrcaHandConfig.from_config_path(
        config_path=os.path.join(V2_MODELS, model, "config.yaml")
    )


def test_the_tension_current_follows_max_current_until_a_family_sets_one(connected_mock_hand):
    config = dataclasses.replace(connected_mock_hand.config, max_current=500)
    assert config.tension_current == 500
    pinned = dataclasses.replace(
        config, guided_tension_currents_dict={"dynamixel": {"hold": {}, "tension": 420}}
    )
    assert pinned.tension_current == 420


@pytest.mark.parametrize("family", SUPPORTED_MOTOR_TYPES)
def test_before_connecting_the_tension_current_is_the_familys_own(family):
    config = _packaged_config("orcahand-left")
    assert (
        config.guided_tension_currents_for(family)["tension"]
        == motor_client_class(family).default_max_current_ma
    )


def test_packaged_configs_list_only_what_differs_from_the_defaults():
    paths = sorted(glob.glob(os.path.join(V2_MODELS, "*", "config.yaml")))
    assert paths
    for path in paths:
        config = OrcaHandConfig.from_config_path(config_path=path)
        expected = (
            {"dynamixel": {"hold": {"thumb_mcp": 100}}} if "orcahand-touch-" in path else {}
        )
        assert config.guided_tension_currents_dict == expected, path


def test_the_tension_step_pulls_at_the_tension_current_then_holds_at_the_ceiling(
    connected_mock_hand, monkeypatch
):
    hand = connected_mock_hand
    hand.motor_client.current_ceilings_ma = {m: 700.0 + m for m in hand.config.motor_ids}
    written, seen = [], []
    write = hand.set_max_current

    def record_write(current):
        written.append(current)
        return write(current)

    monkeypatch.setattr(hand, "set_max_current", record_write)
    answers = ["goto:tension", "Done", "goto:score", FINISH_ANYWAY]

    def prompt(p):
        if p["action"] == "tension":
            seen.append((p["tension_current"], written[-2:]))
        return answers.pop(0)

    run_guided_tension(hand, prompt_callback=prompt, persist=False, resume=False)
    tension_ma = float(hand.config.max_current)
    assert seen == [(tension_ma, [tension_ma, uncapped_currents(hand)])]


def test_every_limit_write_is_read_back_from_the_motors_and_drawn_currents_reported(
    connected_mock_hand,
):
    """The log must show the limit each motor holds, not the one intended: the
    stage's joints at their hold current, every other finger frozen at the
    tension current, the pull at the tension current and the ratchet at the ceiling."""
    hand = connected_mock_hand
    config = hand.config
    hand.motor_client.current_ceilings_ma = {m: 700.0 + m for m in config.motor_ids}
    events, sampled = [], threading.Event()
    answers = ["Continue", "goto:tension", "Done", "goto:score", FINISH_ANYWAY]

    def progress(event):
        events.append(event)
        if event["event"] == "spread":
            sampled.set()

    def prompt(p):
        if p["action"] == "range_check":
            assert sampled.wait(5)
        return answers.pop(0)

    run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress,
                       persist=False, resume=False)

    limits = {e["phase"]: e for e in events if e["event"] == "current_limits"}
    stage = limits["range_check"]["stage"]
    holds = guided_tension.joint_currents(config)
    tension = float(config.tension_current)
    fingers = [j for j in holds if j != "wrist"]
    for joint in fingers:
        want = holds[joint] if joint in stage["joints"] else tension
        assert limits["range_check"]["limits"][joint] == {"asked": want, "held": want}, joint
        assert limits["pull"]["limits"][joint] == {"asked": tension, "held": tension}, joint
        ceiling = 700.0 + config.joint_to_motor_map[joint]
        assert limits["ratchet"]["limits"][joint]["held"] == ceiling, joint

    drawn = next(e for e in events if e["event"] == "currents_drawn")
    assert drawn["stage"]["key"] == stage["key"]
    assert set(fingers) <= set(drawn["drawn"])
    assert all({"peak", "mean"} == set(v) for v in drawn["drawn"].values())


def test_a_family_without_the_read_back_still_tensions(connected_mock_hand, monkeypatch):
    hand = connected_mock_hand

    def missing(motor_ids=None):
        raise NotImplementedError

    monkeypatch.setattr(hand.motor_client, "read_goal_currents", missing)
    events = []
    answers = ["Continue", "goto:tension", "Done", "goto:score", FINISH_ANYWAY]
    run_guided_tension(hand, prompt_callback=lambda p: answers.pop(0),
                       progress_callback=events.append, persist=False, resume=False)
    assert not [e for e in events if e["event"] == "current_limits"]


# ----- slack scoring ----------------------------------------------------------


class _Sweeper:
    """Moves the mock motors by hand: ``hold`` parks a motor until the sampler
    has seen it settle, so the position counts as an end."""

    def __init__(self, hand):
        self.client = hand.motor_client
        self.seen = threading.Event()
        self.last = None

    def progress(self, event):
        if event["event"] == "spread":
            self.last = event
            self.seen.set()

    def sample(self):
        self.seen.clear()
        assert self.seen.wait(5)

    def hold(self, motor, pos, samples=SETTLE_SAMPLES + 1):
        self.client._pos[motor] = pos
        for _ in range(samples):
            self.sample()

    def sweep(self, motor, half, visits=2):
        """Centre → +half → centre → -half ..., each end ``visits`` times."""
        centre = self.client._pos[motor]
        for _ in range(visits):
            for pos in (centre + half, centre, centre - half, centre):
                self.hold(motor, pos)

    def row(self, joint):
        return next(m for m in self.last["motors"] if m["joint"] == joint)


def _drive_slack(hand, drag, visits=2):
    """Prompt answers that, in each stage, sweep that stage's joints' motors
    over ``drag(joint)`` x their travel, reaching each end ``visits`` times."""
    sweeper = _Sweeper(hand)
    targets = tension_targets(hand.config)

    def prompt(p):
        if p["action"] == "range_check":
            for t in targets:
                if t.joint in p["stage"]["joints"] and drag(t.joint):
                    half = math.radians(t.expected_deg * drag(t.joint)) / 2
                    sweeper.sweep(t.motor_id, half, visits)
        return (
            FINISH
            if FINISH in p["options"]
            else (FINISH_ANYWAY if FINISH_ANYWAY in p["options"] else p["options"][0])
        )

    return prompt, sweeper.progress


def _feed(spread, offsets, track=None):
    track = track or guided_tension._EndTracker()
    for offset in offsets:
        record_sample(spread, track, offset, settle=True)
    return track


def test_an_end_needs_a_settled_motor_and_two_visits():
    spread = MotorSpread("index_mcp", 1, 100.0)
    # A flick to 60° that settles back at once never becomes an end.
    _feed(spread, [0.0] * 4 + [60.0] + [0.0] * 4)
    assert spread.plus_deg == 0.0
    # Resting at 48° makes it the + end, seen once; coming back confirms it.
    track = _feed(spread, [48.0] * 5)
    assert spread.plus_deg == 48.0 and spread.plus_hits == 1 and not spread.confirmed
    _feed(spread, [0.0] * 5 + [47.5] * 5, track)
    assert spread.plus_hits == 2
    # A clearly further settled position starts the count over.
    _feed(spread, [0.0] * 5 + [52.0] * 5, track)
    assert spread.plus_deg == 52.0 and spread.plus_hits == 1
    _feed(spread, [-49.0] * 5 + [0.0] * 5 + [-49.0] * 5, track)
    assert spread.minus_hits == 2 and not spread.confirmed
    _feed(spread, [0.0] * 5 + [52.0] * 5, track)
    assert spread.confirmed and spread.slack_deg == pytest.approx(-1.0)
    assert spread.status(4.0, "slack") == "pass"
    assert (
        dataclasses.replace(spread, minus_hits=1).status(4.0, "slack") == "unconfirmed"
    )
    # Fraction scoring never looks at the visit counts.
    assert dataclasses.replace(spread, minus_hits=1).status(0.85) == "pass"


def test_fraction_scoring_records_every_sample():
    spread = MotorSpread("index_mcp", 1, 100.0)
    track = guided_tension._EndTracker()
    record_sample(spread, track, 60.0)
    record_sample(spread, track, -30.0)
    assert (spread.plus_deg, spread.minus_deg, spread.spread_deg) == (60.0, -30.0, 90.0)


def test_slack_scoring_scores_slack_in_degrees_and_records_it(connected_mock_hand):
    hand = connected_mock_hand
    prompt, progress = _drive_slack(hand, lambda j: 0.98 if j == "index_mcp" else 0.4)
    result = run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        scoring="slack",
    )
    by_joint = {m["joint"]: m for m in result.spreads}
    assert by_joint["index_mcp"]["status"] == "pass"
    assert by_joint["index_mcp"]["pass_slack_deg"] == 4.0
    assert by_joint["middle_mcp"]["status"] == "partial"
    assert by_joint["middle_mcp"]["slack_deg"] == pytest.approx(
        0.6 * by_joint["middle_mcp"]["expected_deg"], abs=0.5
    )
    assert not result.completed

    prompt, progress = _drive_slack(hand, lambda j: 0.98)
    result = run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        scoring="slack",
    )
    assert result.completed
    last = read_last_tension(hand.config)
    assert last["scoring"] == "slack"
    assert last["slack_deg"]["index_mcp"] == pytest.approx(
        0.02 * last["expected_deg"]["index_mcp"], abs=0.5
    )


def test_slack_progress_records_the_last_round_and_nothing_carries_over(
    connected_mock_hand,
):
    hand = connected_mock_hand
    prompt, progress = _drive_slack(hand, lambda j: 0.98 if j == "index_mcp" else 0.2)
    run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0001",
        scoring="slack",
    )
    saved = read_progress(hand.config, "ser-0001", "slack")
    assert saved["rounds"] == 1 and saved["scores"]["index_mcp"]["status"] == "pass"
    assert saved["scores"]["index_mcp"]["slack_deg"] == pytest.approx(2.4, abs=0.5)
    # The fraction scoring's view of the same hand is untouched.
    assert read_progress(hand.config, "ser-0001") is None

    # The next run counts on, but index_mcp must earn its pass again.
    events = []
    prompt, progress = _drive_slack(hand, lambda j: 0.0)
    result = run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=lambda e: (events.append(e), progress(e)),
        persist=True,
        hand_key="ser-0001",
        scoring="slack",
    )
    resumed = next(e for e in events if e["event"] == "resumed")
    assert resumed["rounds"] == 1 and resumed["scores"]["index_mcp"]["status"] == "pass"
    assert {m["joint"]: m for m in result.spreads}["index_mcp"]["status"] == "low"
    assert read_progress(hand.config, "ser-0001", "slack")["rounds"] == 2

    prompt, progress = _drive_slack(hand, lambda j: 0.0)
    run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0001",
        resume=False,
        scoring="slack",
    )
    assert read_progress(hand.config, "ser-0001", "slack")["rounds"] == 1


def test_the_two_scorings_keep_separate_progress_for_one_hand(connected_mock_hand):
    hand = connected_mock_hand
    prompt, progress = _drive(hand, lambda j: 0.9 if j == "index_mcp" else 0.2)
    run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0009",
    )
    prompt, progress = _drive_slack(hand, lambda j: 0.2)
    run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0009",
        scoring="slack",
    )
    fraction = read_progress(hand.config, "ser-0009")
    slack = read_progress(hand.config, "ser-0009", "slack")
    assert (
        fraction["best"]["index_mcp"] == pytest.approx(0.9, abs=0.01)
        and fraction["rounds"] == 1
    )
    assert slack["scores"]["index_mcp"]["status"] == "low" and slack["rounds"] == 1
    # Finishing one scoring leaves the other's record alone.
    prompt, progress = _drive(hand, lambda j: 0.9)
    assert run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0009",
    ).completed
    assert read_progress(hand.config, "ser-0009") is None
    assert read_progress(hand.config, "ser-0009", "slack")["rounds"] == 1


def test_slack_scoring_checks_every_stage_every_round(connected_mock_hand):
    hand = connected_mock_hand
    prompt, progress = _drive_slack(
        hand, lambda j: 0.98 if j.startswith(("thumb", "index")) else 0.0
    )
    run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0003",
        scoring="slack",
    )
    posed = []
    inner_prompt, progress = _drive_slack(hand, lambda j: 0.0)

    def prompt2(p):
        if p["action"] == "pose":
            posed.append(p["stage"]["key"])
        return inner_prompt(p)

    run_guided_tension(
        hand,
        prompt_callback=prompt2,
        progress_callback=progress,
        persist=True,
        hand_key="ser-0003",
        scoring="slack",
    )
    assert posed == ["thumb", "index", "middle", "ring", "pinky"]


def test_an_end_reached_once_is_unconfirmed_until_the_sweep_repeats(
    connected_mock_hand,
):
    hand = connected_mock_hand
    sweeper = _Sweeper(hand)
    motor = hand.config.joint_to_motor_map["index_mcp"]
    half = math.radians(0.98 * hand.config.expected_motor_travel_deg("index_mcp")) / 2
    statuses = []

    def prompt(p):
        if p["action"] == "range_check" and p["stage"]["key"] == "index":
            sweeper.sweep(motor, half, visits=1)
            statuses.append(sweeper.row("index_mcp")["status"])
            sweeper.sweep(motor, half, visits=1)
            statuses.append(sweeper.row("index_mcp")["status"])
        return FINISH_ANYWAY if FINISH_ANYWAY in p["options"] else p["options"][0]

    run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=sweeper.progress,
        persist=False,
        scoring="slack",
    )
    assert statuses == ["unconfirmed", "pass"]


def test_a_slack_stage_is_measured_from_its_own_held_pose(connected_mock_hand):
    """The pose step moves the next finger with torque off; its sweep must be
    measured from where it is then held, not from the round's first centre."""
    hand = connected_mock_hand
    sweeper = _Sweeper(hand)
    motor = hand.config.joint_to_motor_map["index_mcp"]
    half = math.radians(0.98 * hand.config.expected_motor_travel_deg("index_mcp")) / 2

    def prompt(p):
        stage = p.get("stage", {}).get("key")
        if p["action"] == "pose" and stage == "index":
            sweeper.client._pos[motor] += math.radians(70.0)  # re-posed, torque off
        if p["action"] == "range_check" and stage == "index":
            sweeper.sweep(motor, half)
        return (
            FINISH
            if FINISH in p["options"]
            else (FINISH_ANYWAY if FINISH_ANYWAY in p["options"] else p["options"][0])
        )

    result = run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=sweeper.progress,
        persist=False,
        scoring="slack",
    )
    row = {m["joint"]: m for m in result.spreads}["index_mcp"]
    assert row["status"] == "pass" and row["slack_deg"] == pytest.approx(2.4, abs=0.5)


def test_a_flick_past_the_hardstop_is_not_a_slack_end(connected_mock_hand):
    hand = connected_mock_hand
    sweeper = _Sweeper(hand)
    motor = hand.config.joint_to_motor_map["index_mcp"]
    rows = []

    def prompt(p):
        if p["action"] == "range_check" and p["stage"]["key"] == "index":
            centre = sweeper.client._pos[motor]
            sweeper.hold(motor, centre + math.radians(60.0), samples=1)
            sweeper.hold(motor, centre)
            rows.append(sweeper.row("index_mcp"))
        return FINISH_ANYWAY if FINISH_ANYWAY in p["options"] else p["options"][0]

    run_guided_tension(
        hand,
        prompt_callback=prompt,
        progress_callback=sweeper.progress,
        persist=False,
        scoring="slack",
    )
    assert rows[0]["plus_deg"] < 2.0 and rows[0]["spread_deg"] < 2.0


def test_a_latched_motor_is_reported_and_holds_again_after_its_reboot(
    connected_mock_hand,
):
    """A latched motor answers the bus and acks torque enable but never
    energizes: it must be named, must not score, and must hold again once
    rebooted from outside the blocked prompt."""
    from orca_core.maintenance.guided_tension import reboot_and_hold

    hand = connected_mock_hand
    client = hand.motor_client
    target = hand.config.joint_to_motor_map["index_mcp"]
    latched = {"on": False}
    reboots = []
    client.read_hardware_error = lambda mid: (
        0x20 if (mid == target and latched["on"]) else 0
    )
    client.take_hardware_alerts = lambda: {target: 0xA0} if latched["on"] else {}

    def reboot_motor(mid):
        reboots.append(mid)
        client._torque_enabled[mid] = False
        latched["on"] = False

    client.reboot_motor = reboot_motor
    events = []
    seen = threading.Event()
    visits = {"range_check": 0, "tension": 0}

    def progress(event):
        events.append(event)
        if event["event"] in ("fault", "fault_cleared"):
            seen.set()

    def prompt(p):
        first = p["action"] in visits and visits[p["action"]] == 0
        if p["action"] in visits and p.get("stage", {}).get("key", "index") == "index":
            visits[p["action"]] += 1
        if first and p["action"] == "range_check" and p["stage"]["key"] == "index":
            seen.clear()
            latched["on"] = True
            assert seen.wait(5), "the latch was never reported"
            fault = [e for e in events if e["event"] == "fault"][-1]
            assert fault["motor"] == target and fault["joint"] == "index_mcp"
            assert fault["flags"] == client.decode_hardware_error(0x20)
            # Reported on the score sheet too, and never as a pass.
            spread = [e for e in events if e["event"] == "spread"][-1]
            row = next(m for m in spread["motors"] if m["joint"] == "index_mcp")
            assert row["status"] == "fault" and row["fault"] == fault["flags"]
            seen.clear()
            assert reboot_and_hold(hand, target) == []
            assert reboots == [target] and client._torque_enabled[target]
            assert seen.wait(5), "the clearing was never reported"
            cleared = [e for e in events if e["event"] == "fault_cleared"]
            assert [e["motor"] for e in cleared] == [target]
        if first and p["action"] == "tension":
            # No sampler runs here; the register is still swept.
            seen.clear()
            latched["on"] = True
            assert seen.wait(5), "a latch at the tension step went unnoticed"
            seen.clear()
            latched["on"] = False
            assert seen.wait(5), "a clear at the tension step went unnoticed"
            return DONE
        if p["action"] == "score":
            if visits["tension"] == 0 and TENSION in p["options"]:
                return TENSION
            return FINISH_ANYWAY if FINISH_ANYWAY in p["options"] else p["options"][0]
        return p["options"][0]

    result = run_guided_tension(
        hand, prompt_callback=prompt, progress_callback=progress
    )
    assert not result.completed
    assert all(m["fault"] is None for m in result.spreads)


def _guided_tension_script():
    path = Path(__file__).resolve().parents[1] / "scripts" / "guided_tension.py"
    spec = importlib.util.spec_from_file_location("guided_tension_script", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_live_line_shows_the_stages_joints_on_one_row():
    script = _guided_tension_script()
    motors = [{"joint": f"joint_{i}", "scoring": "fraction", "fraction": 0.5, "status": "low"} for i in range(16)]
    event = {"passed": 1, "scored": 16, "motors": motors}

    stage = ["joint_9", "joint_10", "joint_11", "joint_12"]
    line = script._live_line(event, 80, stage)
    assert len(script._ANSI_COLOUR.sub("", line)) < 80
    assert all(f"{joint}:" in line for joint in stage) and "joint_0:" not in line and "..." not in line

    every_joint = [motor["joint"] for motor in motors]
    line = script._live_line(event, 80, every_joint)
    assert len(script._ANSI_COLOUR.sub("", line)) < 80 and line.endswith("...")
