"""Tests for the common shape every verification step's result takes.

A plan runner and a report only ever see this face, so what matters is that
every step satisfies it, that ``measurements()`` is flat and addressable, and
that a number a step could not take stays absent rather than becoming a zero
someone later aggregates.
"""

import pytest

from orca_core.hardware.sensing.constants import JOINT_TO_ENCODER_SLOT
from orca_core.verification import (
    StepResult,
    flat_measurements,
    run_anchor_repeatability,
    run_encoder_mapping_sweep,
)
from orca_core.verification.anchor_repeatability import VERDICT_UNRELIABLE

from tests._verification_helpers import encoder_source, joint_hand  # noqa: F401


# ---------------------------------------------------------------------------
# flat_measurements
# ---------------------------------------------------------------------------


def test_keys_are_dotted_metric_then_joint():
    assert flat_measurements("traverse_current_mean_ma", {"index_mcp": 142}) == {
        "traverse_current_mean_ma.index_mcp": 142.0
    }


def test_an_unmeasured_value_is_absent_not_zero():
    """A missing number aggregated as a zero is a fleet statistic that lies."""
    flat = flat_measurements("span_deg", {"index_mcp": None, "ring_pip": 3.0})
    assert flat == {"span_deg.ring_pip": 3.0}


# ---------------------------------------------------------------------------
# Both steps against the protocol
# ---------------------------------------------------------------------------


@pytest.fixture
def calibrated(joint_hand):
    hand, _ = joint_hand
    encoder = encoder_source(hand)
    hand.calibrate(joint_encoder_client=encoder, persist=False)
    return hand, encoder


def test_mapping_sweep_result_satisfies_the_protocol(calibrated):
    hand, encoder = calibrated
    result = run_encoder_mapping_sweep(
        hand,
        joint_encoder_client=encoder,
        joints=["index_mcp"],
        magnitude_tolerance=100.0,
    )
    assert isinstance(result, StepResult)
    assert result.passed
    assert result.messages == []
    assert result.thresholds["crosstalk_limit_deg"] > 0


def test_anchor_result_satisfies_the_protocol(calibrated):
    hand, encoder = calibrated
    result = run_anchor_repeatability(
        hand, joint_encoder_client=encoder, repeats=2
    )
    assert isinstance(result, StepResult)
    assert result.passed
    assert result.thresholds["spread_good_deg"] > 0


def test_mapping_measurements_are_named_per_joint(calibrated):
    hand, encoder = calibrated
    result = run_encoder_mapping_sweep(
        hand,
        joint_encoder_client=encoder,
        joints=["index_mcp", "ring_pip"],
        magnitude_tolerance=100.0,
    )
    measurements = result.measurements()
    assert measurements["commanded_deg.index_mcp"] == pytest.approx(
        result.sweeps[0].commanded_deg
    )
    assert "measured_deg.ring_pip" in measurements
    assert measurements["crosstalk_max_deg.index_mcp"] >= 0.0
    assert all(isinstance(v, float) for v in measurements.values())


def test_a_dead_slot_fails_the_step_and_records_the_zero_it_measured(calibrated):
    """A slot that answers but never moves has a real measurement of 0°, and
    the runner learns it failed from ``passed``, not from the number."""
    hand, _ = calibrated
    dead = JOINT_TO_ENCODER_SLOT["ring_mcp"]
    result = run_encoder_mapping_sweep(
        hand,
        joint_encoder_client=encoder_source(hand, dead_slots=frozenset({dead})),
        joints=["ring_mcp"],
        magnitude_tolerance=100.0,
    )
    assert not result.passed
    assert result.messages
    assert result.measurements()["measured_deg.ring_mcp"] == pytest.approx(0.0)


def test_anchor_measurements_carry_the_spread_and_the_span(calibrated):
    hand, encoder = calibrated
    result = run_anchor_repeatability(
        hand, joint_encoder_client=encoder, repeats=2
    )
    measurements = result.measurements()
    assert measurements["anchor_spread_p2p_deg.index_mcp"] == pytest.approx(
        0.0, abs=0.05
    )
    assert measurements["anchored_passes.index_mcp"] == 2.0
    assert "nominal_span_deg.index_mcp" in measurements
    assert all(isinstance(v, float) for v in measurements.values())


def test_a_failed_anchor_reuses_the_calibration_routines_own_wording(joint_hand):
    """The routine already states what it saw and what it expected; a step
    that paraphrased it would put two descriptions of one fault in the field."""
    hand, _ = joint_hand
    dead = JOINT_TO_ENCODER_SLOT["ring_mcp"]
    encoder = encoder_source(hand, dead_slots=frozenset({dead}))

    result = run_anchor_repeatability(hand, joint_encoder_client=encoder, repeats=2)

    ring_mcp = next(s for s in result.joints if s.joint == "ring_mcp")
    assert ring_mcp.verdict == VERDICT_UNRELIABLE
    assert ring_mcp.messages
    message = ring_mcp.messages[0]
    assert "failed its anchor sample on 2 of 2 passes" in message
    assert f"slot {dead} did not track the sweep" in message
    assert not result.passed
    assert message in result.messages


def test_a_rejected_span_is_reported_without_failing_the_step(calibrated):
    """The gate on a rejected span is J2's; this step still says it happened,
    because the joint is left on a configured ROM known to be wrong."""
    hand, encoder = calibrated
    result = run_anchor_repeatability(
        hand, joint_encoder_client=encoder, repeats=1
    )
    rejected = [s for s in result.joints if s.spans_rejected]
    assert rejected, "the mock's spans miss nominal on most joints"
    for stats in rejected:
        assert any("span rejected on" in m for m in stats.messages), stats.joint
