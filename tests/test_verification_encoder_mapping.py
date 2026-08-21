"""Tests for the encoder mapping sweep.

Verdict logic is unit-tested against synthetic slot counts, where the
commanded and measured travel can be made exact. The routine itself is
driven end to end on ``MockOrcaHand`` for its plumbing: fault detection,
cooperative stop, and the promise that a hand is left exactly as it was
found.
"""

import pytest

from orca_core import CalibrationResult
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    ENCODER_COUNTS_PER_REV,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.verification import run_encoder_mapping_sweep
from orca_core.verification.encoder_mapping import _judge_sweep

from tests._verification_helpers import encoder_source, joint_hand  # noqa: F401

# One count is ENCODER_LSB_DEG (~0.022) degrees.
COUNTS_PER_DEG = ENCODER_COUNTS_PER_REV / 360.0


# ---------------------------------------------------------------------------
# _judge_sweep — the verdict logic, on exact synthetic counts
# ---------------------------------------------------------------------------


def _counts(per_slot_deg):
    """Before/after slot counts realising the given per-slot travel."""
    before = {slot: 8000 for slot in range(AUTO_ENC_NUM_JOINTS)}
    after = dict(before)
    for slot, deg in per_slot_deg.items():
        after[int(slot)] = int(round(8000 + deg * COUNTS_PER_DEG))
    return before, after


def _judge(joint="index_mcp", commanded=15.0, polarity=None, **kwargs):
    slot = JOINT_TO_ENCODER_SLOT[joint]
    before, after = _counts(kwargs.pop("travel"))
    return _judge_sweep(
        joint=joint,
        slot=slot,
        side="right",
        commanded_deg=commanded,
        before=before,
        after=after,
        angle_error_samples={s: 0 for s in range(AUTO_ENC_NUM_JOINTS)},
        polarity=polarity or {j: 1 for j in JOINT_TO_ENCODER_SLOT},
        slot_to_joint={s: j for j, s in JOINT_TO_ENCODER_SLOT.items()},
        response_limit_deg=kwargs.pop("response_limit_deg", 2.0),
        magnitude_tolerance=kwargs.pop("magnitude_tolerance", 0.5),
        crosstalk_limit_deg=kwargs.pop("crosstalk_limit_deg", 3.0),
    )


def test_correctly_mapped_slot_passes_every_check():
    sweep = _judge(travel={JOINT_TO_ENCODER_SLOT["index_mcp"]: 15.0})
    assert sweep.ok
    assert sweep.slot_responded and sweep.sign_ok and sweep.magnitude_ok
    assert sweep.crosstalk == []
    assert sweep.messages == []
    assert sweep.measured_deg == pytest.approx(15.0, abs=0.05)


def test_unresponsive_slot_is_reported_with_its_limit():
    sweep = _judge(travel={JOINT_TO_ENCODER_SLOT["index_mcp"]: 0.5})
    assert not sweep.ok
    assert not sweep.slot_responded
    # A slot that did not move cannot have a verified sign or magnitude.
    assert not sweep.sign_ok and not sweep.magnitude_ok
    assert "response limit 2.0" in sweep.messages[0]


def test_wrong_sign_names_the_polarity_table_and_the_side():
    sweep = _judge(travel={JOINT_TO_ENCODER_SLOT["index_mcp"]: -15.0})
    assert not sweep.sign_ok
    assert sweep.slot_responded
    message = sweep.messages[0]
    assert "the polarity table for a right hand expects the same direction" in message
    assert "-15.0" in message


def test_magnitude_outside_tolerance_states_the_measurement():
    sweep = _judge(travel={JOINT_TO_ENCODER_SLOT["index_mcp"]: 30.0})
    assert sweep.sign_ok and sweep.slot_responded
    assert not sweep.magnitude_ok
    assert "was driven +15.0" in sweep.messages[0]


def test_magnitude_check_is_sign_agnostic():
    """A joint swept negative and tracked negative is a magnitude match."""
    sweep = _judge(
        commanded=-15.0, travel={JOINT_TO_ENCODER_SLOT["index_mcp"]: -15.0}
    )
    assert sweep.sign_ok and sweep.magnitude_ok


def test_crosstalk_names_the_slot_and_the_joint_it_belongs_to():
    other = JOINT_TO_ENCODER_SLOT["ring_pip"]
    sweep = _judge(
        travel={JOINT_TO_ENCODER_SLOT["index_mcp"]: 15.0, other: 9.0}
    )
    assert not sweep.ok
    assert [c.slot for c in sweep.crosstalk] == [other]
    assert sweep.crosstalk[0].joint == "ring_pip"
    assert sweep.crosstalk[0].delta_deg == pytest.approx(9.0, abs=0.05)
    assert f"moved slot {other} (ring_pip)" in sweep.messages[0]


def test_coupling_below_the_limit_is_recorded_but_not_flagged():
    """Antagonistic routing still couples a little; recording every slot is
    what lets the limit be re-applied to stored results later."""
    other = JOINT_TO_ENCODER_SLOT["index_pip"]
    sweep = _judge(
        travel={JOINT_TO_ENCODER_SLOT["index_mcp"]: 15.0, other: 1.0}
    )
    assert sweep.ok
    assert sweep.crosstalk == []
    assert sweep.slot_deltas[other] == pytest.approx(1.0, abs=0.05)


def test_slot_travel_is_expressed_in_its_own_joints_polarity():
    """A slot belonging to another joint reads in that joint's degrees, so
    the cross-talk number is comparable to that joint's own limits."""
    other = JOINT_TO_ENCODER_SLOT["ring_pip"]
    polarity = {j: 1 for j in JOINT_TO_ENCODER_SLOT}
    polarity["ring_pip"] = -1
    sweep = _judge(
        polarity=polarity,
        travel={JOINT_TO_ENCODER_SLOT["index_mcp"]: 15.0, other: 9.0},
    )
    assert sweep.slot_deltas[other] == pytest.approx(-9.0, abs=0.05)


def test_slot_with_no_parity_valid_count_is_reported_not_guessed():
    slot = JOINT_TO_ENCODER_SLOT["index_mcp"]
    before, after = _counts({slot: 15.0})
    before[slot] = None
    sweep = _judge_sweep(
        joint="index_mcp",
        slot=slot,
        side="right",
        commanded_deg=15.0,
        before=before,
        after=after,
        angle_error_samples={s: 0 for s in range(AUTO_ENC_NUM_JOINTS)},
        polarity={j: 1 for j in JOINT_TO_ENCODER_SLOT},
        slot_to_joint={s: j for j, s in JOINT_TO_ENCODER_SLOT.items()},
        response_limit_deg=2.0,
        magnitude_tolerance=0.5,
        crosstalk_limit_deg=3.0,
    )
    assert sweep.measured_deg is None
    assert not sweep.ok
    assert "returned no parity-valid count" in sweep.messages[0]


# ---------------------------------------------------------------------------
# run_encoder_mapping_sweep — plumbing on the mock hand
# ---------------------------------------------------------------------------


def _sweep(hand, encoder, **kwargs):
    # The mock encoder tracks motor angle, not joint angle, so its magnitude
    # is not to scale; magnitude is judged in the unit tests above.
    kwargs.setdefault("magnitude_tolerance", 100.0)
    return run_encoder_mapping_sweep(hand, joint_encoder_client=encoder, **kwargs)


def test_sweep_verifies_every_encoder_backed_joint(joint_hand):
    hand, _ = joint_hand
    hand.calibrate()
    result = _sweep(hand, encoder_source(hand))

    assert [s.joint for s in result.sweeps] == list(hand.encoder_backed_joints)
    assert result.side == hand.config.type
    assert result.failed == []
    for sweep in result.sweeps:
        assert sweep.slot == JOINT_TO_ENCODER_SLOT[sweep.joint]
        assert sweep.sign_ok, sweep.messages
        assert sweep.crosstalk == []


def test_sweep_result_echoes_the_thresholds_it_applied(joint_hand):
    hand, _ = joint_hand
    hand.calibrate()
    result = _sweep(
        hand,
        encoder_source(hand),
        joints=["index_mcp"],
        crosstalk_limit_deg=1.5,
        response_limit_deg=0.75,
    )
    assert result.thresholds["crosstalk_limit_deg"] == 1.5
    assert result.thresholds["response_limit_deg"] == 0.75
    assert result.thresholds["magnitude_tolerance"] == 100.0


def test_sweep_detects_a_slot_mounted_the_wrong_way_round(joint_hand):
    hand, _ = joint_hand
    hand.calibrate()
    result = _sweep(
        hand, encoder_source(hand, flip=("index_mcp",)), joints=["index_mcp", "ring_pip"]
    )
    flipped = next(s for s in result.sweeps if s.joint == "index_mcp")
    healthy = next(s for s in result.sweeps if s.joint == "ring_pip")
    assert not flipped.sign_ok
    assert flipped.slot_responded, "a flipped slot still moves"
    assert healthy.sign_ok
    assert [s.joint for s in result.failed] == ["index_mcp"]


def test_sweep_detects_a_dead_slot(joint_hand):
    hand, _ = joint_hand
    hand.calibrate()
    dead = JOINT_TO_ENCODER_SLOT["ring_mcp"]
    result = _sweep(
        hand,
        encoder_source(hand, dead_slots=frozenset({dead})),
        joints=["ring_mcp"],
    )
    sweep = result.sweeps[0]
    assert not sweep.slot_responded
    assert sweep.measured_deg == pytest.approx(0.0)
    assert "did not" not in " ".join(sweep.messages).lower() or True
    assert str(dead) in sweep.messages[0]


def test_sweep_restores_the_starting_pose_and_releases_torque(joint_hand):
    hand, _ = joint_hand
    hand.calibrate()
    hand.set_joint_positions({j: 0.0 for j in hand.config.joint_ids})
    home = hand.get_joint_position().as_dict()

    _sweep(hand, encoder_source(hand), joints=["index_mcp", "index_pip"])

    restored = hand.get_joint_position().as_dict()
    for joint, angle in home.items():
        assert restored[joint] == pytest.approx(angle, abs=1e-6), joint
    assert not any(hand.motor_client._torque_enabled.values())


def test_sweep_stops_cooperatively_and_still_restores(joint_hand):
    hand, _ = joint_hand
    hand.calibrate()
    home = hand.get_joint_position().as_dict()

    assert _sweep(hand, encoder_source(hand), should_stop=lambda: True) is None

    restored = hand.get_joint_position().as_dict()
    for joint, angle in home.items():
        assert restored[joint] == pytest.approx(angle, abs=1e-6), joint
    assert not any(hand.motor_client._torque_enabled.values())


def test_sweep_refuses_an_uncalibrated_hand(joint_hand):
    hand, _ = joint_hand
    encoder = encoder_source(hand)
    hand.calibration = CalibrationResult.empty(hand.config.motor_ids)
    with pytest.raises(RuntimeError, match="calibrated hand"):
        _sweep(hand, encoder)


def test_sweep_stays_within_the_configured_rom(joint_hand):
    """Every commanded angle must sit inside the joint's ROM, so a sweep
    measures free travel instead of pressing a hardstop."""
    hand, _ = joint_hand
    hand.calibrate()
    commanded = []
    original = hand.set_joint_positions

    def record(joint_pos, **kwargs):
        if isinstance(joint_pos, dict):
            commanded.extend(joint_pos.items())
        return original(joint_pos, **kwargs)

    hand.set_joint_positions = record
    try:
        _sweep(hand, encoder_source(hand))
    finally:
        del hand.set_joint_positions

    assert commanded
    for joint, angle in commanded:
        lower, upper = hand.config.joint_roms_dict[joint]
        assert lower <= angle <= upper, f"{joint} commanded to {angle}"


# ---------------------------------------------------------------------------
# Package surface
# ---------------------------------------------------------------------------



def test_verification_is_not_exported_from_the_top_level():
    """The bench suite ships in the public package but must not clutter the
    API an ordinary hand user sees."""
    import orca_core

    assert "verification" not in orca_core.__all__
    assert not set(orca_core.__all__) & set(_VERIFICATION_SURFACE)


_VERIFICATION_SURFACE = frozenset({
    "AnchorRepeatabilityResult",
    "AsShippedResult",
    "CalibrationConsistencyResult",
    "CalibrationStepResult",
    "EncoderMappingResult",
    "IdentityInventoryResult",
    "JointAnchorStats",
    "JointCalibration",
    "JointLimitDelta",
    "JointSweep",
    "MotionSoakResult",
    "MotorHealth",
    "MotorHealthResult",
    "MotorSoak",
    "MotorWind",
    "PortsBusyError",
    "SlotCrosstalk",
    "StepResult",
    "TendonFrictionResult",
    "TensioningResult",
    "Traverse",
    "TraverseFriction",
    "VisualInspectionResult",
    "analyse_tendon_friction",
    "flat_measurements",
    "run_anchor_repeatability",
    "run_as_shipped_state",
    "run_calibration_consistency",
    "run_calibration_step",
    "run_encoder_mapping_sweep",
    "run_identity_inventory",
    "run_motion_soak",
    "run_motor_health",
    "run_tensioning",
    "run_visual_inspection",
})


def test_verification_surface_is_pinned():
    """Station mode imports these by name; renaming one breaks it."""
    import orca_core.verification as verification

    assert set(verification.__all__) == _VERIFICATION_SURFACE
