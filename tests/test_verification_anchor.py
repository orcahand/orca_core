"""Tests for the anchor repeatability routine.

Driven end to end on ``MockOrcaHand``: the mock's anchors are deterministic,
so a clean hand must score a zero spread, and the interesting cases are the
ones where a joint fails to sample at all.
"""

import pytest

from orca_core.hardware.sensing.constants import JOINT_TO_ENCODER_SLOT
from orca_core.utils import read_yaml
from orca_core.verification import run_anchor_repeatability
from orca_core.verification.anchor_repeatability import (
    VERDICT_GOOD,
    VERDICT_UNRELIABLE,
)

from tests._verification_helpers import encoder_source, joint_hand  # noqa: F401


def test_repeatability_scores_a_stable_anchor_good(joint_hand):
    hand, _ = joint_hand
    encoder = encoder_source(hand)
    hand.calibrate(joint_encoder_client=encoder, persist=False)

    result = run_anchor_repeatability(hand, joint_encoder_client=encoder, repeats=2)

    assert result.completed_passes == 2
    assert result.thresholds == {
        "spread_good_deg": 0.30,
        "spread_marginal_deg": 1.00,
    }
    assert result.joints
    for stats in result.joints:
        assert stats.anchored_passes == 2
        assert stats.verdict == VERDICT_GOOD, stats
        assert stats.peak_to_peak_deg == pytest.approx(0.0, abs=0.05)
    assert result.failed == []


def test_a_rejected_span_is_still_reported_with_its_value(joint_hand):
    """The calibration routine drops a span that misses its tolerance and
    falls back to the config nominal, so the measured value survives only in
    the event stream. A hand shipping with one is running a joint map known
    to be wrong, which is unanswerable if the number is not recorded."""
    hand, _ = joint_hand
    encoder = encoder_source(hand)

    result = run_anchor_repeatability(hand, joint_encoder_client=encoder, repeats=1)

    rejected = [s for s in result.joints if s.spans_rejected]
    accepted = [s for s in result.joints if s.measured_span_mean_deg and not s.spans_rejected]
    assert rejected, "the mock's spans miss nominal on most joints"
    assert accepted, "and land inside tolerance on at least one"

    for stats in rejected:
        assert stats.measured_span_mean_deg is not None, stats.joint
        assert stats.spans_rejected == 1
        # The joint kept its config nominal, wrong by exactly this much.
        assert stats.span_error_deg == pytest.approx(
            stats.measured_span_mean_deg - stats.nominal_span_deg, abs=1e-3
        )


def test_reported_nominal_span_comes_from_the_config_rom(joint_hand):
    hand, _ = joint_hand
    result = run_anchor_repeatability(
        hand, joint_encoder_client=encoder_source(hand), repeats=1
    )
    for stats in result.joints:
        lower, upper = hand.config.joint_roms_dict[stats.joint]
        assert stats.nominal_span_deg == pytest.approx(upper - lower, abs=1e-3)


def test_repeatability_persists_nothing_and_restores_calibration(joint_hand):
    hand, tmp_path = joint_hand
    encoder = encoder_source(hand)
    hand.calibrate(joint_encoder_client=encoder, persist=True)
    baseline = hand.calibration
    before = read_yaml(str(tmp_path / "calibration.yaml"))

    run_anchor_repeatability(hand, joint_encoder_client=encoder, repeats=2)

    assert hand.calibration is baseline
    assert read_yaml(str(tmp_path / "calibration.yaml")) == before
    assert not any(hand.motor_client._torque_enabled.values())


def test_repeatability_marks_a_failing_anchor_unreliable(joint_hand):
    """A joint whose anchor sample fails must not present as clean, however
    tight the spread of the passes that did read."""
    hand, _ = joint_hand
    dead = JOINT_TO_ENCODER_SLOT["ring_mcp"]
    encoder = encoder_source(hand, dead_slots=frozenset({dead}))

    result = run_anchor_repeatability(hand, joint_encoder_client=encoder, repeats=2)

    ring_mcp = next(s for s in result.joints if s.joint == "ring_mcp")
    assert ring_mcp.sample_failures > 0
    assert ring_mcp.verdict == VERDICT_UNRELIABLE
    assert ring_mcp in result.failed


def test_repeatability_stopped_before_any_pass_returns_none(joint_hand):
    hand, _ = joint_hand
    baseline = hand.calibration
    result = run_anchor_repeatability(
        hand,
        joint_encoder_client=encoder_source(hand),
        repeats=3,
        should_stop=lambda: True,
    )
    assert result is None
    assert hand.calibration is baseline


def test_repeatability_rejects_a_non_positive_repeat_count(joint_hand):
    hand, _ = joint_hand
    with pytest.raises(ValueError, match="repeats must be positive"):
        run_anchor_repeatability(
            hand, joint_encoder_client=encoder_source(hand), repeats=0
        )


def test_repeatability_forwards_the_calibration_event_stream(joint_hand):
    hand, _ = joint_hand
    events = []
    run_anchor_repeatability(
        hand,
        joint_encoder_client=encoder_source(hand),
        repeats=1,
        progress_callback=events.append,
    )
    names = [e["event"] for e in events]
    assert names[0] == "repeatability_started"
    assert names[-1] == "repeatability_done"
    assert "pass_started" in names and "pass_done" in names
    inner = [e for e in events if e["event"] == "calibration_event"]
    assert inner, "the inner calibration events must stay visible"
    assert {e["payload"]["event"] for e in inner} >= {"calibration_started"}
