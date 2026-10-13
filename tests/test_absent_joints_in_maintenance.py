"""A hand with fingers missing from the bus: the maintenance routines leave
those joints out, say so, and score only what is there."""

import math
import threading

import pytest

from orca_core.constants import HARDWARE_VERSION_V21
from orca_core.maintenance import calibration_routine
from orca_core.maintenance.guided_tension import (
    DEFAULT_RANGE_STAGES,
    EXCLUDED_NO_MOTOR,
    FINISH,
    FINISH_ANYWAY,
    excluded_joints,
    run_guided_tension,
    score,
    tension_targets,
)
from orca_core.maintenance.tensioning import finger_motor_ids, run_tension

MISSING_FINGERS = ("thumb", "ring", "pinky")


def _silence(hand, fingers=MISSING_FINGERS):
    """Declare every motor of ``fingers`` unplugged on the mock bus."""
    silent = {m for j, m in hand.config.joint_to_motor_map.items() if j.split("_")[0] in fingers}
    hand.motor_client.silent_motor_ids = silent
    return silent


@pytest.fixture
def partial_hand(connected_mock_hand):
    hand = connected_mock_hand
    hand.config = hand.config.with_hardware_version(HARDWARE_VERSION_V21)
    _silence(hand)
    return hand


def test_the_hand_names_its_absent_joints(partial_hand):
    joints = partial_hand.absent_joints
    assert joints and all(j.split("_")[0] in MISSING_FINGERS for j in joints)
    assert set(partial_hand.absent_motor_ids) == partial_hand.motor_client.silent_motor_ids
    assert "wrist" not in joints


def test_targets_and_the_hold_leave_absent_joints_out(partial_hand):
    hand = partial_hand
    targets = tension_targets(hand.config, absent_joints=hand.absent_joints)
    assert {t.joint.split("_")[0] for t in targets} == {"index", "middle"}
    held = finger_motor_ids(hand.config, hand.absent_motor_ids)
    assert set(held) == {t.motor_id for t in targets}


def test_the_score_sheet_lists_the_excluded_joints_with_a_reason(partial_hand):
    hand = partial_hand
    excluded = excluded_joints(hand)
    assert len(excluded) == 10
    assert all(e["reason"] == EXCLUDED_NO_MOTOR and e["joint"] in hand.absent_joints for e in excluded)
    sheet = score(tension_targets(hand.config, absent_joints=hand.absent_joints), excluded=excluded)
    assert sheet["scored"] == 6 and [e["joint"] for e in sheet["excluded"]] == [e["joint"] for e in excluded]


def test_guided_tension_runs_the_present_fingers_only_and_reports_the_rest(partial_hand):
    hand = partial_hand
    client = hand.motor_client
    seen = {"stages": [], "absent": None, "scores": [], "spread_excluded": None}
    spread_seen = threading.Event()
    targets = tension_targets(hand.config, absent_joints=hand.absent_joints)

    def progress(event):
        if event["event"] == "spread":
            spread_seen.set()
            seen["spread_excluded"] = event["excluded"]
        if event["event"] == "score":
            seen["scores"].append(event)

    def prompt(p):
        if p["action"] == "pose":
            seen["stages"].append(p["stage"]["key"])
            seen["absent"] = p["absent_joints"]
        if p["action"] == "range_check":
            assert not any(client._torque_enabled[m] for m in client.silent_motor_ids)
            for t in targets:
                if t.joint not in p["stage"]["joints"]:
                    continue
                half = math.radians(t.expected_deg * 0.95) / 2
                centre = client._pos[t.motor_id]
                for pos in (centre + half, centre - half, centre):
                    client._pos[t.motor_id] = pos
                    spread_seen.clear()
                    assert spread_seen.wait(5)
        return FINISH if FINISH in p["options"] else (
            FINISH_ANYWAY if FINISH_ANYWAY in p["options"] else p["options"][0])

    result = run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress, persist=False)

    assert result.completed
    assert seen["stages"] == ["thumb_index", "middle"], "ring and pinky have no motor; thumb+index keeps index"
    assert set(seen["absent"]) == set(hand.absent_joints)
    final = seen["scores"][-1]
    assert final["scored"] == 6 and final["passed"] == 6
    assert {e["joint"] for e in final["excluded"]} == set(hand.absent_joints)
    assert seen["spread_excluded"] and seen["spread_excluded"][0]["reason"] == EXCLUDED_NO_MOTOR


def test_a_hand_with_no_finger_answering_is_refused(connected_mock_hand):
    hand = connected_mock_hand
    hand.config = hand.config.with_hardware_version(HARDWARE_VERSION_V21)
    _silence(hand, ("thumb", "index", "middle", "ring", "pinky"))
    with pytest.raises(ValueError, match="no finger has a motor answering"):
        run_guided_tension(hand, prompt_callback=lambda p: p["options"][0], persist=False)


def test_the_plain_hold_pulls_only_the_present_fingers(partial_hand):
    hand = partial_hand
    client = hand.motor_client
    before = dict(client._pos)
    holding = threading.Event()
    hand.tension(move_motors=False, blocking=False, offset_deg=15.0,
                 progress_callback=lambda e: holding.set() if e.get("phase") == "holding" else None)
    assert holding.wait(10)
    assert hand.stop_task(timeout=10)
    moved = {m: math.degrees(client._pos[m] - before[m]) for m in hand.config.motor_ids}
    present = set(finger_motor_ids(hand.config, hand.absent_motor_ids))
    assert all(abs(moved[m] - 15.0) < 1e-6 for m in present)
    assert all(moved[m] == 0.0 for m in client.silent_motor_ids)


def test_calibration_skips_absent_joints_and_says_so(partial_hand):
    hand = partial_hand
    events = []
    result = calibration_routine.run_calibration(
        hand, joints=["index_mcp", "ring_mcp"], progress_callback=events.append, persist=False)
    skipped = [e for e in events if e["event"] == "joints_skipped"]
    assert skipped and skipped[0]["joints"] == ["ring_mcp"]
    assert "did not answer" in skipped[0]["reason"]
    assert set(result.motor_travel_measured_dict) == {"index_mcp"}, "only the present joint was driven"


def test_a_motor_that_never_meets_a_hardstop_is_given_up_after_two_turns(connected_mock_hand):
    """A slack or broken tendon lets the motor spin: the step stops it after
    MAX_DRIVE_TURNS and reports the joint, instead of driving until the
    wall-clock backstop."""
    import math

    from orca_core.constants import MAX_DRIVE_TURNS

    hand = connected_mock_hand
    client = hand.motor_client
    client._max_motor_pos, client._min_pos = 1e9, -1e9   # nothing stops this mock
    events = []
    result = calibration_routine.run_calibration(
        hand, joints=["index_mcp"], progress_callback=events.append, persist=False)
    given_up = [e for e in events if e["event"] == "hardstop_not_found"]
    assert given_up and all(e["joint"] == "index_mcp" for e in given_up)
    assert all(e["travel_deg"] >= math.degrees(MAX_DRIVE_TURNS * 2 * math.pi) for e in given_up)
    assert not [e for e in events if e["event"] == "drive_step_timeout"]
    assert hand.config.joint_to_motor_map["index_mcp"] not in result.motor_limits_dict or \
        "index_mcp" not in result.motor_travel_measured_dict


def test_a_wrist_that_overruns_is_driven_again_at_lower_output(connected_mock_hand):
    """The belt skips at full output, holds at 75 %: the wrist sweep backs its
    PWM off once and finds the stop instead of giving up."""
    from orca_core.constants import HARDWARE_VERSION_V21, WRIST_PWM_RETRY_SCALE

    hand = connected_mock_hand
    hand.config = hand.config.with_hardware_version(HARDWARE_VERSION_V21)
    client = hand.motor_client
    wrist = hand.config.joint_to_motor_map["wrist"]
    # Nothing stops the mock at full output; the first PWM cut brings the stops back.
    client._max_motor_pos, client._min_pos = 1e9, -1e9
    original = client.write_pwm_fraction

    def capped(motor_ids, fraction):
        original(motor_ids, fraction)
        if fraction < 1.0:
            client._max_motor_pos, client._min_pos = 3.0, -3.0

    client.write_pwm_fraction = capped
    events = []
    result = calibration_routine.run_calibration(
        hand, joints=["wrist"], force_wrist=True, progress_callback=events.append, persist=False)

    retries = [e for e in events if e["event"] == "overrun_retry"]
    assert retries and retries[0]["joint"] == "wrist" and retries[0]["pwm_fraction"] == WRIST_PWM_RETRY_SCALE
    expected = hand.config.expected_motor_travel_deg("wrist")
    assert retries[0]["limit_deg"] == pytest.approx(1.5 * expected)
    assert not [e for e in events if e["event"] == "hardstop_not_found"]
    assert wrist in result.motor_limits_dict and None not in result.motor_limits_dict[wrist]
    assert client.pwm_fraction[wrist] == 1.0, "full output restored after the step"


def test_a_finger_that_overruns_its_baseline_is_given_up_without_a_pwm_retry(connected_mock_hand):
    from orca_core.constants import HARDWARE_VERSION_V21

    hand = connected_mock_hand
    hand.config = hand.config.with_hardware_version(HARDWARE_VERSION_V21)
    client = hand.motor_client
    client._max_motor_pos, client._min_pos = 1e9, -1e9
    events = []
    calibration_routine.run_calibration(
        hand, joints=["index_mcp"], progress_callback=events.append, persist=False)
    given_up = [e for e in events if e["event"] == "hardstop_not_found"]
    assert given_up and given_up[0]["limit_deg"] == pytest.approx(720.0), "a tendon joint keeps the two-turn cap"
    assert not [e for e in events if e["event"] == "overrun_retry"]


def test_the_other_fingers_are_frozen_and_not_sampled_during_a_stage(connected_mock_hand):
    """During a stage only its joints yield at the hold current and only they
    are recorded; every other finger holds at the tension current and its
    spread does not change however it is moved."""
    from orca_core.constants import HARDWARE_VERSION_V21

    hand = connected_mock_hand
    hand.config = hand.config.with_hardware_version(HARDWARE_VERSION_V21)
    client = hand.motor_client
    motor_of = hand.config.joint_to_motor_map
    seen = {}
    spread_seen = threading.Event()

    def progress(event):
        if event["event"] == "spread":
            spread_seen.set()
            seen["motors"] = {m["joint"]: m for m in event["motors"]}

    def prompt(p):
        if p["action"] == "range_check" and p["stage"]["key"] == "thumb_index":
            seen["prompt"] = p
            seen["currents"] = dict(client._cur)
            # Shove a ring motor: it is not this stage's, so its spread must not move.
            client._pos[motor_of["ring_mcp"]] += 1.0
            spread_seen.clear(); assert spread_seen.wait(5)
            spread_seen.clear(); assert spread_seen.wait(5)
            return "goto:score"
        if p["action"] == "score":
            return FINISH_ANYWAY
        return p["options"][0]

    run_guided_tension(hand, prompt_callback=prompt, progress_callback=progress, persist=False,
                       hold_current=100, tension_current=400)

    p = seen["prompt"]
    assert set(p["hold_current"]) == set(p["stage"]["joints"])
    assert p["freeze_current"] == 400
    assert seen["currents"][motor_of["index_mcp"]] == pytest.approx(100)
    assert seen["currents"][motor_of["ring_mcp"]] == pytest.approx(400)
    assert seen["motors"]["ring_mcp"]["spread_deg"] == 0.0
