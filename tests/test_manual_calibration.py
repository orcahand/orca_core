"""Hands-on hardstop capture: the operator sets the pose, not the motors."""

import pytest

from orca_core.hardware_hand import MockOrcaHand
from orca_core.maintenance.calibration_routine import run_calibration


@pytest.fixture()
def hand():
    h = MockOrcaHand()
    h.connect()
    yield h
    h.disconnect()


def _events(collected, kind):
    return [e for e in collected if e["event"] == kind]


def test_prompts_for_both_ends_of_every_joint(hand):
    asked = []
    run_calibration(hand, joints=["index_mcp"], manual=True, persist=False,
                    prompt_callback=lambda r: asked.append(
                        (r["action"], r["joint"], r["direction"])) or "record")
    assert asked == [
        ("capture_limit", "index_mcp", "flex"),
        ("capture_limit", "index_mcp", "extend"),
    ]


def test_records_the_position_the_operator_is_holding(hand):
    motor_id = hand.config.joint_to_motor_map["index_mcp"]
    idx = hand.config.motor_id_to_idx_dict[motor_id]
    held = []

    def prompt(request):
        held.append(float(hand.get_motor_pos()[idx]))
        return "record"

    events = []
    run_calibration(hand, joints=["index_mcp"], manual=True, persist=False,
                    prompt_callback=prompt, progress_callback=events.append)

    recorded = _events(events, "limit_recorded")
    assert [e["bound"] for e in recorded] == ["upper", "lower"]
    assert [e["limit"] for e in recorded] == pytest.approx(held)


def test_torque_stays_off_so_the_joints_can_be_moved(hand):
    torqued = []
    prompt = lambda r: torqued.append(  # noqa: E731
        hand.motor_client.is_torque_enabled(r["motor"])
        if hasattr(hand.motor_client, "is_torque_enabled") else False) or "record"
    run_calibration(hand, joints=["index_mcp"], manual=True, persist=False,
                    prompt_callback=prompt)
    assert not any(torqued), "a joint held by its motor cannot be moved by hand"


def test_skip_leaves_that_end_unrecorded_and_the_joint_uncommitted(hand):
    events = []
    run_calibration(
        hand, joints=["index_mcp"], manual=True, persist=False,
        prompt_callback=lambda r: "skip" if r["direction"] == "extend" else "record",
        progress_callback=events.append)

    assert len(_events(events, "limit_recorded")) == 1
    assert len(_events(events, "manual_capture_skipped")) == 1
    # One bound is not a calibration: the joint keeps what it already had.
    assert not _events(events, "joint_calibrated")


def test_abort_ends_the_run_without_a_result(hand):
    before = dict(hand.calibration.motor_limits_dict)
    result = run_calibration(hand, joints=["index_mcp"], manual=True,
                             persist=False,
                             prompt_callback=lambda r: "abort")
    assert result is None
    assert hand.calibration.motor_limits_dict == before


def test_an_unrecognised_answer_skips_rather_than_recording(hand):
    """A garbled answer must not be read as consent to record a limit."""
    events = []
    run_calibration(hand, joints=["index_mcp"], manual=True, persist=False,
                    prompt_callback=lambda r: "yes please",
                    progress_callback=events.append)
    assert not _events(events, "limit_recorded")
    assert len(_events(events, "manual_capture_skipped")) == 2


def test_manual_mode_requires_a_prompt_callback(hand):
    with pytest.raises(ValueError, match="prompt_callback"):
        run_calibration(hand, joints=["index_mcp"], manual=True, persist=False)


def test_manual_mode_never_re_drives_a_short_joint(hand):
    """The re-drive is a motor-driven sweep; there are no motors driving here.

    Exercised directly: on a mock nothing moves, so a whole run reports zero
    travel and stops at the did-not-move floor before this branch is reached.
    """
    import math

    from orca_core.maintenance.calibration_routine import (
        _DriveState,
        _resolve_short_travel,
    )

    motor_id = hand.config.joint_to_motor_map["index_mcp"]
    baseline = hand.config.expected_motor_travel_deg("index_mcp")
    # Short enough to be flagged, far enough above the floor to be re-drivable.
    half = math.radians(baseline / 2)
    state = _DriveState(pending_limits={motor_id: [0.0, half]})

    events = []
    assert _resolve_short_travel(
        hand, "index_mcp", motor_id, state=state, encoder_pass=None,
        motor_travel_measured={}, boosted_joints={},
        progress_callback=events.append, should_stop=lambda: False,
        manual=True)

    kinds = [e["event"] for e in events]
    assert "travel_retry_started" not in kinds
    disabled = [e for e in events if e["event"] == "travel_retry_disabled"]
    assert disabled and disabled[0]["reason"] == "manual"


# ----- a sweep that never moved ----------------------------------------------


def test_a_motionless_sweep_is_reported_per_direction(hand, monkeypatch):
    """The step that did nothing says so, not the one two steps later."""
    motor_id = hand.config.joint_to_motor_map["index_mcp"]
    # Freeze this motor: every write to it is dropped, so it holds position
    # while the sweep walks its goal — a blocked tendon, from the bus's side.
    original = hand._set_motor_pos

    def frozen(desired_pos, rel_to_current=False):
        if isinstance(desired_pos, dict):
            desired_pos = {k: v for k, v in desired_pos.items()
                           if k != motor_id}
            if not desired_pos:
                return None
        return original(desired_pos, rel_to_current=rel_to_current)

    monkeypatch.setattr(hand, "_set_motor_pos", frozen)

    events = []
    run_calibration(hand, joints=["index_mcp"], persist=False,
                    progress_callback=events.append)

    stalled = _events(events, "sweep_no_motion")
    assert [e["direction"] for e in stalled] == ["flex", "extend"]
    assert all(e["joint"] == "index_mcp" for e in stalled)
    assert all(e["moved_deg"] < 3.0 for e in stalled)


def test_a_real_sweep_reports_no_motionless_step(hand):
    events = []
    run_calibration(hand, joints=["index_mcp"], persist=False,
                    progress_callback=events.append)
    assert not _events(events, "sweep_no_motion")
