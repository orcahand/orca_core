"""Tests for the base-hand verification steps (B0-B8).

Every step is driven end to end on ``MockOrcaHand``. What is checked is the
step contract rather than the physics: thresholds echoed, a hand left as it
was found, infrastructure raising while a measurement outcome returns, and
failure text that states the measurement and the limit.
"""

import pytest

from orca_core import MockOrcaHand
from orca_core.hand_factory import HandDetection
from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo
from orca_core.verification import (
    PortsBusyError,
    StepResult,
    Traverse,
    analyse_tendon_friction,
    run_as_shipped_state,
    run_calibration_consistency,
    run_calibration_step,
    run_identity_inventory,
    run_motion_soak,
    run_motor_health,
    run_tensioning,
    run_visual_inspection,
)

from tests._verification_helpers import joint_hand  # noqa: F401


@pytest.fixture
def hand(joint_hand):
    hand, _ = joint_hand
    return hand


def _detection(**overrides) -> HandDetection:
    fields = dict(
        model_name="orcahand-joint-right",
        side="right",
        has_tactile=False,
        has_encoders=True,
        motor_port="/dev/mock-motor",
        identity=OrcaBoardInfo(role="motor", side="right", serial="ser-0001"),
        declared_config=1,
        probed_tactile=False,
        probed_encoders=True,
    )
    fields.update(overrides)
    return HandDetection(**fields)


# ---------------------------------------------------------------------------
# B0 identity and inventory
# ---------------------------------------------------------------------------


def test_identity_passes_when_the_bus_holds_exactly_the_configured_motors(hand):
    result = run_identity_inventory(
        hand.config, detection=_detection(), motor_client=hand.motor_client
    )
    assert isinstance(result, StepResult)
    assert result.passed, result.messages
    assert result.found_motor_ids == sorted(hand.config.motor_ids)
    assert result.hand_id == "ser-0001"
    assert result.measurements()["motors_missing"] == 0.0


def test_identity_scans_exactly_the_ids_this_hand_configures(hand):
    """A motor that was never assigned answers on its factory default, which
    is inside the configured span, so the default scan does not go wider."""
    scanned = []
    hand.motor_client.scan_for_motors = lambda port, id_range, baud_rates=None: (
        scanned.append(id_range) or []
    )
    run_identity_inventory(
        hand.config, detection=_detection(), motor_client=hand.motor_client
    )
    assert scanned == [(min(hand.config.motor_ids), max(hand.config.motor_ids))]


def test_identity_names_missing_and_unexpected_motors_separately(hand):
    stranger = MockOrcaHand(config_path=hand.config.config_path)
    stranger._motor_client = type(hand.motor_client)(
        [1, 2, 25], port="mock"
    )
    stranger._motor_client.connect()
    try:
        # Widened past the configured span, which is how a bench hunts a motor
        # assigned outside the set.
        result = run_identity_inventory(
            hand.config,
            detection=_detection(),
            motor_client=stranger.motor_client,
            id_range=(1, 30),
        )
    finally:
        stranger._motor_client.disconnect()

    assert not result.passed
    assert result.unexpected_motor_ids == [25]
    assert result.missing_motor_ids
    joined = " ".join(result.messages)
    assert "factory-default ID" in joined
    assert "25" in joined


def test_identity_raises_on_a_busy_port_rather_than_failing_the_hand(hand):
    """A CDC held by another process reads as an absent capability, so a
    result taken now would understate the hand. The runner turns this into
    verdict `error`, never `fail`."""
    with pytest.raises(PortsBusyError, match="held by another process"):
        run_identity_inventory(
            hand.config,
            detection=_detection(busy_ports=("/dev/cu.busy",)),
            motor_client=hand.motor_client,
        )


def test_identity_reports_a_capability_the_board_declares_but_cannot_show(hand):
    result = run_identity_inventory(
        hand.config,
        detection=_detection(
            has_tactile=True, probed_tactile=False,
            missing_capabilities=("tactile",),
        ),
        motor_client=hand.motor_client,
    )
    assert not result.passed
    assert "reports config 1" in result.messages[0]


# ---------------------------------------------------------------------------
# B1 motor health
# ---------------------------------------------------------------------------


def test_motor_health_scores_a_clean_cold_bus(hand):
    result = run_motor_health(hand, bulk_reads=5)
    assert isinstance(result, StepResult)
    assert result.passed, result.messages
    assert result.failed_reads == 0
    assert result.bulk_reads == 5
    measurements = result.measurements()
    assert measurements["failed_bulk_reads"] == 0.0
    assert measurements["temperature_c.index_mcp"] == pytest.approx(25.0)
    assert measurements["supply_voltage_v.index_mcp"] == pytest.approx(12.0)


def test_motor_health_names_the_error_bits_without_guessing_a_cause(hand):
    faulted = hand.config.motor_ids[0]
    hand.motor_client._hardware_error[faulted] = 0x21

    result = run_motor_health(hand, bulk_reads=2)

    assert not result.passed
    message = result.messages[0]
    assert "0x21" in message
    assert "input_voltage, overload" in message
    # A byte says a fault is latched, not what broke it.
    assert "tendon" not in message and "because" not in message


def test_motor_health_counts_dropped_status_packets(hand):
    hand.motor_client._pos_vel_cur_reader.last_read_ok = False

    result = run_motor_health(hand, bulk_reads=10)

    assert result.failed_reads == 10
    assert not result.passed
    assert "10 of 10 bulk reads returned no status packet" in result.messages[-1]


def test_motor_health_flags_the_outlier_motor_not_the_peak(hand):
    hot = hand.config.motor_ids[0]
    hand.motor_client._temp[hot] = 25.0 + 20.0

    result = run_motor_health(hand, bulk_reads=2, max_temperature_c=100.0)

    assert not result.passed
    assert "span 20.0°C" in " ".join(result.messages)
    assert result.measurements()["temperature_spread_c"] == pytest.approx(20.0)


def test_motor_health_echoes_the_thresholds_it_applied(hand):
    result = run_motor_health(hand, bulk_reads=3, max_temperature_c=42.0)
    assert result.thresholds["max_temperature_c"] == 42.0
    assert result.thresholds["bulk_reads"] == 3.0


def test_motor_health_leaves_torque_untouched(hand):
    """Read-only: a step that energised the motors to measure them would
    change the thing it is measuring."""
    hand.enable_torque()
    run_motor_health(hand, bulk_reads=2)
    assert all(hand.motor_client._torque_enabled.values())


# ---------------------------------------------------------------------------
# B3 calibration
# ---------------------------------------------------------------------------


def test_calibration_step_records_spans_ratios_and_duration(hand):
    result = run_calibration_step(hand, persist=False)
    assert isinstance(result, StepResult)
    assert result.calibrated and result.wrist_calibrated
    assert result.passed, result.messages

    measurements = result.measurements()
    assert measurements["motor_span_rad.index_mcp"] > 0
    assert "joint_to_motor_ratio.index_mcp" in measurements
    assert measurements["duration_s"] >= 0
    assert result.traverses, "the traverses B5 consumes must be harvested"


def test_calibration_step_gates_a_span_only_when_given_a_band(hand):
    unbanded = run_calibration_step(hand, persist=False)
    span = next(j for j in unbanded.joints if j.joint == "index_mcp").motor_span_rad
    assert unbanded.passed

    banded = run_calibration_step(
        hand, persist=False,
        motor_span_limits_rad={"index_mcp": (span + 1.0, span + 2.0)},
    )
    assert not banded.passed
    message = " ".join(banded.messages)
    assert "motor rad between its limits" in message
    assert "stalls before reaching one" in message


def test_calibration_step_stopped_early_returns_none(hand):
    assert run_calibration_step(hand, persist=False, should_stop=lambda: True) is None


# ---------------------------------------------------------------------------
# B5 tendon friction
# ---------------------------------------------------------------------------


def _traverse(currents, positions=None, joint="index_mcp", direction=1):
    positions = positions or [0.1 * i for i in range(len(currents))]
    return Traverse(
        motor=1, joint=joint, step_index=0, direction=direction,
        reached_limit=True, positions=positions, currents=currents,
    )


def test_friction_excludes_the_terminal_stall_from_the_average():
    """Current at the hardstop measures the hardstop, not the friction of
    getting there, so a long stall must not drag the mean up."""
    moving = [100.0] * 6
    stalled = [400.0] * 6
    positions = [0.1 * i for i in range(6)] + [0.6] * 6

    result = analyse_tendon_friction(
        [_traverse(moving + stalled, positions=positions)]
    )

    assert result.traverses[0].current_mean_ma == pytest.approx(100.0)
    assert result.traverses[0].current_p90_ma == pytest.approx(100.0)
    # Six intervals moved; the six stalled ones contributed nothing.
    assert result.traverses[0].moving_samples == 6


def test_friction_keys_each_direction_separately():
    """The two directions of one joint are different measurements — one hand
    measured 88 mA one way against 179 mA the other."""
    result = analyse_tendon_friction([
        _traverse([88.0] * 6, direction=1),
        _traverse([179.0] * 6, direction=-1),
    ])
    measurements = result.measurements()
    assert measurements["traverse_current_mean_ma.index_mcp.flex"] == pytest.approx(88.0)
    assert measurements["traverse_current_mean_ma.index_mcp.extend"] == pytest.approx(179.0)


def test_friction_is_recorded_but_not_gated_without_a_band():
    result = analyse_tendon_friction([_traverse([500.0] * 6)])
    assert result.passed, "no fleet band means no gate"
    assert result.measurements()["traverse_current_mean_ma.index_mcp.flex"] == 500.0


def test_friction_names_over_tension_only_when_a_band_is_exceeded():
    result = analyse_tendon_friction(
        [_traverse([500.0] * 6)], current_bands_ma={"index_mcp": (60.0, 260.0)}
    )
    assert not result.passed
    message = result.messages[0]
    assert "drew 500 mA mean through its flex traverse (expected 60–260)" in message
    assert "over-tensioned tendon" in message
    assert result.thresholds["traverse_current_mean_ma.index_mcp.max"] == 260.0


def test_friction_consumes_a_calibration_step_result(hand):
    """B5 costs no bench time because it re-reads what B3 already drove."""
    calibration = run_calibration_step(hand, persist=False)
    result = analyse_tendon_friction(calibration.traverses)
    assert result.traverses
    assert any(t.current_mean_ma is not None for t in result.traverses)


# ---------------------------------------------------------------------------
# B4 calibration consistency
# ---------------------------------------------------------------------------


def test_consistency_scores_a_repeatable_hand_and_restores_calibration(hand):
    baseline = run_calibration_step(hand, persist=False)
    before = hand.calibration

    result = run_calibration_consistency(hand, baseline=baseline)

    assert isinstance(result, StepResult)
    assert result.passed, result.messages
    assert hand.calibration is before, "the repeat must not become the calibration"
    assert not any(hand.motor_client._torque_enabled.values())
    assert result.measurements()["passes"] == 2.0


def test_consistency_reports_a_joint_whose_limits_moved(hand):
    baseline = run_calibration_step(hand, persist=False)
    shifted = next(j for j in baseline.joints if j.joint == "index_mcp")
    moved = type(shifted)(
        joint=shifted.joint, motor=shifted.motor,
        lower_rad=(shifted.lower_rad or 0.0) - 1.0, upper_rad=shifted.upper_rad,
        motor_span_rad=shifted.motor_span_rad, ratio=shifted.ratio,
    )
    baseline = type(baseline)(
        **{**baseline.__dict__, "joints": [moved]}
    )

    result = run_calibration_consistency(hand, baseline=baseline)

    assert not result.passed
    assert "apart across two calibrations" in result.messages[0]


# ---------------------------------------------------------------------------
# B6 motion soak
# ---------------------------------------------------------------------------


def test_soak_cycles_and_reports_a_clean_hand(hand):
    hand.calibrate()
    result = run_motion_soak(
        hand, duration_s=0.0, hold_s=0.0, sample_interval_s=0.0, num_steps=1
    )
    assert isinstance(result, StepResult)
    assert result.cycles >= 1
    assert result.passed, result.messages
    assert not any(hand.motor_client._torque_enabled.values())


def test_soak_stops_early_when_a_motor_overheats(hand):
    hand.calibrate()
    hand.motor_client._temp[hand.config.motor_ids[0]] = 95.0

    result = run_motion_soak(
        hand, duration_s=5.0, hold_s=0.0, sample_interval_s=0.0, num_steps=1
    )

    assert not result.passed
    assert "stopped early" in " ".join(result.messages)
    assert result.peak_temperature_c >= 70.0


def test_soak_counts_a_rebooted_motor(hand):
    hand.calibrate()
    overloaded = hand.config.motor_ids[0]
    hand.motor_client._hardware_error[overloaded] = 0x20

    result = run_motion_soak(
        hand, duration_s=0.0, hold_s=0.0, sample_interval_s=0.0, num_steps=1
    )

    assert result.reboots == [overloaded]
    assert not result.passed
    assert "latching an overload" in " ".join(result.messages)


def test_soak_needs_at_least_two_poses(hand):
    with pytest.raises(ValueError, match="at least two poses"):
        run_motion_soak(hand, poses=[{}], duration_s=0.0)


# ---------------------------------------------------------------------------
# B7 visual inspection
# ---------------------------------------------------------------------------


def test_inspection_records_the_operator_verdict_and_note(hand):
    hand.calibrate()
    result = run_visual_inspection(
        hand,
        prompt_callback=lambda p: {"confirmed": False, "note": "ring finger binds"},
        sweep=False,
    )
    assert not result.passed
    assert result.note == "ring finger binds"
    assert "ring finger binds" in result.messages[0]
    assert not any(hand.motor_client._torque_enabled.values())


def test_inspection_without_an_operator_is_not_a_confirmation(hand):
    hand.calibrate()
    result = run_visual_inspection(hand, sweep=False)
    assert not result.confirmed
    assert "no operator prompt" in result.note


def test_inspection_asks_only_after_the_hand_is_posed(hand):
    hand.calibrate()
    events = []
    run_visual_inspection(
        hand,
        prompt_callback=lambda p: events.append(p["action"]) or True,
        sweep=True,
        sweep_hold_s=0.0,
        progress_callback=lambda e: events.append(e["event"]),
    )
    assert events.index("posed_neutral") < events.index("confirm_visual_inspection")


# ---------------------------------------------------------------------------
# B2 tensioning
# ---------------------------------------------------------------------------


def test_tensioning_prompts_once_per_round_and_records_each_wind(hand):
    prompts = []
    result = run_tensioning(
        hand, rounds=2, prompt_callback=lambda p: prompts.append(p)
    )

    assert isinstance(result, StepResult)
    assert [p["round"] for p in prompts] == [1, 2]
    assert result.rounds == 2
    assert result.motors
    assert any(m.wind_rad is not None for m in result.motors)
    assert "rounds" in result.measurements()


def test_tensioning_prompts_only_once_the_routine_is_holding(hand):
    """Prompting during the wind would ask the operator to act on a hand that
    is still moving."""
    seen = []
    run_tensioning(
        hand,
        rounds=1,
        prompt_callback=lambda p: seen.append("prompt"),
        progress_callback=lambda e: (
            seen.append("holding")
            if e.get("payload", {}).get("phase") == "holding"
            else None
        ),
    )
    assert seen.index("holding") < seen.index("prompt")


def test_tensioning_rejects_a_non_positive_round_count(hand):
    with pytest.raises(ValueError, match="rounds must be positive"):
        run_tensioning(hand, rounds=0)


# ---------------------------------------------------------------------------
# B8 as-shipped state
# ---------------------------------------------------------------------------


def test_as_shipped_hashes_what_the_hand_leaves_with(joint_hand):
    hand, tmp_path = joint_hand
    hand.calibrate(persist=True)

    result = run_as_shipped_state(hand)

    assert isinstance(result, StepResult)
    assert result.passed, result.messages
    assert result.torque_off and result.at_neutral
    assert len(result.config_sha256) == 64
    assert len(result.calibration_sha256) == 64
    assert not any(hand.motor_client._torque_enabled.values())


def test_as_shipped_says_so_when_there_is_nothing_to_diff_against(hand):
    """A hand shipping without a calibration.yaml cannot be compared with
    whatever comes back."""
    result = run_as_shipped_state(hand)
    assert result.calibration_sha256 is None
    assert not result.passed
    assert "nothing to diff" in result.messages[0]


def test_as_shipped_releases_torque_even_when_posing_raises(hand):
    def boom(*args, **kwargs):
        raise RuntimeError("bus died mid-pose")

    hand.enable_torque()
    hand.set_neutral_position = boom
    with pytest.raises(RuntimeError, match="bus died"):
        run_as_shipped_state(hand)
    assert not any(hand.motor_client._torque_enabled.values())
