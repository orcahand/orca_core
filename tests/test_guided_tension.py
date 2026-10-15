"""Guided tensioning: spreads are scored against the travel baseline, the wrist
is left alone, and every exit leaves the motors limp."""

import dataclasses
import math
import threading

import pytest

from orca_core.maintenance.guided_tension import (
    FINISH,
    FINISH_ANYWAY,
    joint_currents,
    read_last_tension,
    run_guided_tension,
    tension_targets,
)
from orca_core.constants import HARDWARE_VERSION_V21


@pytest.fixture
def connected_mock_hand(connected_mock_hand):
    """The guided routine scores against the v2.1 baseline, so run it as v2.1."""
    hand = connected_mock_hand
    hand.config = hand.config.with_hardware_version(HARDWARE_VERSION_V21)
    return hand


def _drive(hand, drag, any_hardware=False):
    """Prompt answers that, in each stage, drag that stage's joints' motors by
    ``drag(joint)`` x their travel and back."""
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
        return FINISH if FINISH in p["options"] else (
            FINISH_ANYWAY if FINISH_ANYWAY in p["options"] else p["options"][0])

    return prompt, progress


def test_wrist_is_never_a_target(connected_mock_hand):
    assert "wrist" not in {t.joint for t in tension_targets(connected_mock_hand.config)}


def test_partial_spread_scores_short_and_releases(connected_mock_hand):
    hand = connected_mock_hand
    prompt, progress = _drive(hand, lambda j: 0.9 if j == "index_mcp" else 0.4)
    result = run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress, persist=True)

    by_joint = {m["joint"]: m for m in result.spreads}
    assert by_joint["index_mcp"]["status"] == "pass"
    assert by_joint["middle_mcp"]["status"] == "partial"
    assert not result.completed
    assert not any(hand.motor_client._torque_enabled.values())
    assert read_last_tension(hand.config) is None


def test_full_spread_completes_and_records(connected_mock_hand):
    hand = connected_mock_hand
    prompt, progress = _drive(hand, lambda j: 0.9)
    result = run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress, persist=True)

    assert result.completed and result.rounds == 1
    assert read_last_tension(hand.config)["spreads"]["index_mcp"] == pytest.approx(0.9, abs=0.01)


def test_an_older_hand_is_refused_unless_overridden(connected_mock_hand):
    hand = connected_mock_hand
    hand.config = hand.config.with_hardware_version(2)
    with pytest.raises(ValueError, match="needs v2.1 hardware"):
        run_guided_tension(hand, prompt_callback=lambda p: p["options"][0])

    prompt, progress = _drive(hand, lambda j: 0.9, any_hardware=True)
    result = run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress,
                                persist=False, any_hardware=True)
    assert result.completed


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
    from orca_core.maintenance.guided_tension import read_progress

    hand = connected_mock_hand
    prompt, progress = _drive(hand, lambda j: 0.9 if j == "index_mcp" else 0.2)
    run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress,
                       persist=True, hand_key="ser-0001")
    assert read_progress(hand.config, "ser-0001")["best"]["index_mcp"] == pytest.approx(0.9, abs=0.01)
    assert read_progress(hand.config, "ser-0002") is None

    # A resumed run keeps index_mcp passed without moving it again.
    prompt, progress = _drive(hand, lambda j: 0.0)
    result = run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress,
                                persist=True, hand_key="ser-0001")
    by_joint = {m["joint"]: m for m in result.spreads}
    assert by_joint["index_mcp"]["status"] == "pass"
    assert by_joint["index_mcp"]["kept"] == pytest.approx(0.9, abs=0.01)

    prompt, progress = _drive(hand, lambda j: 0.0)
    result = run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress,
                                persist=True, hand_key="ser-0001", resume=False)
    assert {m["joint"]: m for m in result.spreads}["index_mcp"]["status"] == "low"


def test_stages_cover_every_finger_joint_and_skip_what_passed(connected_mock_hand):
    from orca_core.maintenance.guided_tension import DEFAULT_RANGE_STAGES

    hand = connected_mock_hand
    staged = [j for st in DEFAULT_RANGE_STAGES for j in st.joints]
    assert sorted(staged) == sorted(t.joint for t in tension_targets(hand.config))

    prompt, progress = _drive(hand, lambda j: 0.9 if j.startswith("thumb") or j.startswith("index") else 0.0)
    run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress,
                       persist=True, hand_key="ser-0003")
    posed = []
    inner_prompt, progress = _drive(hand, lambda j: 0.0)

    def prompt2(p):
        if p["action"] == "pose":
            posed.append(p["stage"]["key"])
        return inner_prompt(p)

    run_guided_tension(hand, prompt_callback=prompt2, progress_callback=progress,
                       persist=True, hand_key="ser-0003")
    assert posed == ["middle", "ring", "pinky"]


def test_back_and_goto_move_between_stages(connected_mock_hand):
    seen = []
    script = iter(["Continue", "Next", "Back", "Continue", "Back", "goto:pinky",
                   "Continue", "Score", "goto:middle", "Continue", "Next", "Continue", "Next",
                   "Continue", "Score", FINISH_ANYWAY])

    def prompt(p):
        seen.append((p["action"], p.get("stage", {}).get("key")))
        answer = next(script)
        assert answer in p["options"]
        return answer

    result = run_guided_tension(connected_mock_hand, prompt_callback=prompt, persist=False,
                                resume=False)
    assert seen == [
        ("pose", "thumb_index"), ("range_check", "thumb_index"),
        ("pose", "middle"), ("pose", "thumb_index"), ("range_check", "thumb_index"),
        ("pose", "thumb_index"), ("pose", "pinky"), ("range_check", "pinky"), ("score", None),
        ("pose", "middle"), ("range_check", "middle"), ("pose", "ring"), ("range_check", "ring"),
        ("pose", "pinky"), ("range_check", "pinky"), ("score", None),
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
    result = run_guided_tension(hand, prompt_callback=prompt, persist=False, resume=False,
                                tension_offset_deg=15.0, tension_extra_deg=15.0)
    assert seen == [("pose", "thumb_index"), ("tension", None), ("tension", None),
                    ("pose", "ring"), ("score", None)]
    assert result.rounds == 2
    assert all(pulled[j] == pytest.approx(centre[j] + math.radians(30.0)) for j in centre)
    assert not any(client._torque_enabled.values())


def test_a_passed_stage_is_skipped_but_can_be_jumped_to(connected_mock_hand):
    hand = connected_mock_hand
    prompt, progress = _drive(hand, lambda j: 0.9 if j.startswith(("thumb", "index")) else 0.0)
    run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress,
                       persist=True, hand_key="ser-0004")
    seen = []
    answers = ["goto:thumb_index", "Continue", "Next", "Continue", "Next", "Continue", "Next",
               "Continue", "Score", FINISH_ANYWAY]

    def prompt(p):
        seen.append((p["action"], p.get("stage", {}).get("key")))
        assert answers[0] in p["options"]
        return answers.pop(0)

    run_guided_tension(hand, prompt_callback=prompt, persist=True, hand_key="ser-0004")
    assert seen[:5] == [("pose", "middle"), ("pose", "thumb_index"), ("range_check", "thumb_index"),
                        ("pose", "middle"), ("range_check", "middle")]


def test_jumping_to_tension_with_no_pull_still_holds(connected_mock_hand):
    hand = connected_mock_hand
    targets = tension_targets(hand.config)
    held = []
    answers = ["goto:tension", "Done", "goto:score", FINISH_ANYWAY]

    def prompt(p):
        if p["action"] == "tension":
            held.append(all(hand.motor_client._torque_enabled[t.motor_id] for t in targets))
        return answers.pop(0)

    run_guided_tension(hand, prompt_callback=prompt, persist=False, resume=False,
                       tension_offset_deg=0)
    assert held == [True]
    assert not any(hand.motor_client._torque_enabled.values())


def test_hold_currents_come_from_the_config_and_a_named_joint_keeps_its_own(connected_mock_hand):
    config = dataclasses.replace(connected_mock_hand.config, guided_tension_currents_dict={
        "dynamixel": {"hold": {"mcp": 150, "thumb_mcp": 84}, "tension": 420}})
    from_config = joint_currents(config)
    flat = joint_currents(config, 100)
    grouped = joint_currents(config, lambda j: 140 if j.endswith("_mcp") else 200)
    assert from_config["thumb_mcp"] == flat["thumb_mcp"] == grouped["thumb_mcp"] == 84
    assert (from_config["index_mcp"], flat["index_mcp"], grouped["index_mcp"]) == (150, 100, 140)
    assert from_config["index_pip"] == 100 and config.tension_current == 420
