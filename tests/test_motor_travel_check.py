"""Tests for the per-joint motor-travel baseline and the short-travel re-drive.

An over-tensioned tendon stalls the calibration drive before the hardstop, so
the joint's two motor limits come out closer together than its spool geometry
allows. These cover the baseline (config side) and the higher-current re-drive
the calibration routine answers a short measurement with.
"""

import math
import os
import shutil

import pytest
import yaml

from orca_core import MockOrcaHand
from orca_core.hand_config import HandConfigValidationError, OrcaHandConfig
from orca_core.maintenance import calibration_routine
from orca_core.maintenance.motor_travel import (
    measured_travel_by_joint,
    motor_travel_deg,
    travel_deviation,
    write_joint_motor_travel,
)
from orca_core.utils.utils import read_yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "v2", "orcahand-right")


@pytest.fixture
def model_dir(tmp_path):
    shutil.copy(os.path.join(MODEL_DIR, "config.yaml"), tmp_path / "config.yaml")
    return tmp_path


def _write_config(model_dir, **overrides):
    """Patch the copied config.yaml with ``overrides`` and return its path."""
    path = model_dir / "config.yaml"
    doc = read_yaml(str(path))
    doc.update(overrides)
    path.write_text(yaml.dump(doc, sort_keys=False))
    return str(path)


# ---------------------------------------------------------------------------
# motor_travel helpers
# ---------------------------------------------------------------------------


def test_motor_travel_deg_converts_a_limit_pair():
    assert motor_travel_deg([0.0, math.pi]) == pytest.approx(180.0)


def test_motor_travel_deg_is_unsigned():
    """Limits recorded lower-first or upper-first describe the same travel."""
    assert motor_travel_deg([math.pi, 0.0]) == pytest.approx(180.0)


@pytest.mark.parametrize("limits", [None, [], [1.0], [None, 1.0], [1.0, None]])
def test_motor_travel_deg_rejects_incomplete_limits(limits):
    assert motor_travel_deg(limits) is None


def test_travel_deviation_is_a_signed_fraction():
    assert travel_deviation(75.0, 100.0) == pytest.approx(-0.25)
    assert travel_deviation(125.0, 100.0) == pytest.approx(0.25)


def test_measured_travel_is_keyed_by_joint_not_motor(model_dir):
    """The baseline follows the joint, so a remapped motor ID keeps its value."""
    path = _write_config(model_dir)
    hand = MockOrcaHand(config_path=path)
    hand.connect()
    try:
        travel = measured_travel_by_joint(hand.config, hand.calibration)
    finally:
        hand.disconnect()

    assert set(travel) <= set(hand.config.joint_ids)
    assert all(value > 0 for value in travel.values())


def test_measured_travel_skips_half_calibrated_joints(model_dir):
    path = _write_config(model_dir)
    hand = MockOrcaHand(config_path=path)
    hand.connect()
    try:
        motor_id = hand.config.joint_to_motor_map["index_mcp"]
        limits = dict(hand.calibration.motor_limits_dict)
        limits[motor_id] = [None, limits[motor_id][1]]
        hand.calibration = type(hand.calibration)(
            **{
                **hand.calibration.__dict__,
                "motor_limits_dict": limits,
            }
        )
        travel = measured_travel_by_joint(hand.config, hand.calibration)
    finally:
        hand.disconnect()

    assert "index_mcp" not in travel
    assert "index_pip" in travel


# ---------------------------------------------------------------------------
# write_joint_motor_travel
# ---------------------------------------------------------------------------


def test_write_joint_motor_travel_merges_and_keeps_other_keys(model_dir):
    path = _write_config(model_dir, joint_motor_travel={"index_mcp": 100.0})

    stored = write_joint_motor_travel(path, {"index_pip": 90.0})

    assert stored == {"index_mcp": 100.0, "index_pip": 90.0}
    doc = read_yaml(path)
    assert doc["joint_motor_travel"] == stored
    assert doc["max_current"] == 300  # untouched neighbours survive the rewrite


def test_write_joint_motor_travel_replace_drops_unmeasured_joints(model_dir):
    path = _write_config(model_dir, joint_motor_travel={"index_mcp": 100.0})

    stored = write_joint_motor_travel(path, {"index_pip": 90.0}, merge=False)

    assert stored == {"index_pip": 90.0}


def test_write_joint_motor_travel_follows_canonical_joint_order(model_dir):
    path = _write_config(model_dir, joint_motor_travel={})

    write_joint_motor_travel(path, {"pinky_pip": 88.0, "wrist": 167.0})

    doc = read_yaml(path)
    assert list(doc["joint_motor_travel"]) == ["wrist", "pinky_pip"]


# ---------------------------------------------------------------------------
# config validation
# ---------------------------------------------------------------------------


def test_config_reads_the_travel_baseline(model_dir):
    path = _write_config(model_dir, joint_motor_travel={"index_mcp": 123.4})
    config = OrcaHandConfig.from_config_path(path)

    assert config.expected_motor_travel_deg("index_mcp") == pytest.approx(123.4)
    assert config.expected_motor_travel_deg("index_pip") is None


def test_config_rejects_a_baseline_for_an_unknown_joint(model_dir):
    path = _write_config(model_dir, joint_motor_travel={"index_knuckle": 100.0})
    with pytest.raises(HandConfigValidationError, match="index_knuckle"):
        OrcaHandConfig.from_config_path(path)


@pytest.mark.parametrize("travel", [0, -10.0])
def test_config_rejects_a_non_positive_baseline(model_dir, travel):
    path = _write_config(model_dir, joint_motor_travel={"index_mcp": travel})
    with pytest.raises(HandConfigValidationError, match="positive"):
        OrcaHandConfig.from_config_path(path)


@pytest.mark.parametrize("margin", [0, 1, 1.5, -0.1])
def test_config_rejects_a_margin_outside_the_unit_interval(model_dir, margin):
    path = _write_config(model_dir, calibration_travel_margin=margin)
    with pytest.raises(HandConfigValidationError, match="calibration_travel_margin"):
        OrcaHandConfig.from_config_path(path)


def test_config_rejects_a_retry_current_that_is_not_a_boost(model_dir):
    """A retry at the nominal current would re-drive with no extra torque."""
    path = _write_config(model_dir, calibration_retry_current=300)
    with pytest.raises(HandConfigValidationError, match="must exceed"):
        OrcaHandConfig.from_config_path(path)


def test_retry_current_defaults_to_a_multiple_of_the_calibration_current(model_dir):
    path = _write_config(model_dir, calibration_current=200, max_current=300)
    config = OrcaHandConfig.from_config_path(path)

    assert config.calibration_retry_current_resolved == 300


def test_retry_current_ramps_across_attempts(model_dir):
    # calibration_max_current is what bounds the ramp, so the top of it has to
    # fit under that ceiling to be reached.
    path = _write_config(
        model_dir,
        calibration_max_current=400,
        calibration_current=200,
        calibration_retry_current=400,
        calibration_travel_retries=2,
    )
    config = OrcaHandConfig.from_config_path(path)

    assert config.retry_current_for_attempt(1) == pytest.approx(300.0)
    assert config.retry_current_for_attempt(2) == pytest.approx(400.0)
    with pytest.raises(ValueError):
        config.retry_current_for_attempt(3)


# ---------------------------------------------------------------------------
# the calibration routine's short-travel re-drive
# ---------------------------------------------------------------------------


def _events(calls, name):
    return [event for event in calls if event["event"] == name]


def _run(hand, joints=None):
    calls = []
    result = calibration_routine.run_calibration(
        hand,
        joints=joints,
        progress_callback=calls.append,
        persist=False,
    )
    return result, calls


def _mock_hand(path):
    hand = MockOrcaHand(config_path=path)
    success, msg = hand.connect()
    assert success, msg
    return hand


def _baseline_from_a_clean_run(model_dir, joint):
    """Travel the mock actually achieves for ``joint``, as a realistic baseline."""
    hand = _mock_hand(_write_config(model_dir, joint_motor_travel={}))
    try:
        _run(hand, joints=[joint])
        return measured_travel_by_joint(hand.config, hand.calibration)[joint]
    finally:
        hand.disconnect()


def test_travel_within_margin_is_reported_and_not_re_driven(model_dir):
    achieved = _baseline_from_a_clean_run(model_dir, "index_mcp")
    path = _write_config(model_dir, joint_motor_travel={"index_mcp": achieved})

    hand = _mock_hand(path)
    try:
        _, calls = _run(hand, joints=["index_mcp"])
    finally:
        hand.disconnect()

    checked = _events(calls, "travel_checked")
    assert [event["joint"] for event in checked] == ["index_mcp"]
    assert checked[0]["within_margin"]
    assert not _events(calls, "travel_retry_started")


def test_a_joint_without_a_baseline_is_measured_but_not_checked(model_dir):
    path = _write_config(model_dir, joint_motor_travel={})

    hand = _mock_hand(path)
    try:
        result, calls = _run(hand, joints=["index_mcp"])
    finally:
        hand.disconnect()

    assert _events(calls, "travel_baseline_missing")
    assert not _events(calls, "travel_checked")
    assert result.motor_travel_measured_dict["index_mcp"] > 0


def test_short_travel_re_drives_at_a_higher_current(model_dir):
    """A baseline the nominal pass cannot meet triggers the boosted re-drive."""
    achieved = _baseline_from_a_clean_run(model_dir, "index_mcp")
    path = _write_config(
        model_dir,
        joint_motor_travel={"index_mcp": achieved * 2},
        calibration_travel_margin=0.1,
        calibration_current=200,
        calibration_retry_current=280,
        calibration_travel_retries=2,
        max_current=300,
    )

    hand = _mock_hand(path)
    currents = []
    original = hand.set_max_current
    hand.set_max_current = lambda current: (
        currents.append(current), original(current)
    )[1]
    try:
        _, calls = _run(hand, joints=["index_mcp"])
    finally:
        hand.disconnect()

    started = _events(calls, "travel_retry_started")
    assert [event["attempt"] for event in started] == [1, 2]
    assert [event["current"] for event in started] == [240.0, 280.0]
    # The boost is scoped to the re-drive: once the last one is over, the
    # nominal calibration current is restored before the run's final cleanup.
    assert {240.0, 280.0} <= set(currents)
    after_last_boost = currents[len(currents) - currents[::-1].index(280.0):]
    assert after_last_boost[0] == 200
    assert currents[-1] == 300  # run cleanup restores max_current

    exhausted = _events(calls, "travel_retry_exhausted")
    assert len(exhausted) == 1
    assert exhausted[0]["joint"] == "index_mcp"


def test_an_exhausted_re_drive_still_commits_the_best_limits(model_dir):
    achieved = _baseline_from_a_clean_run(model_dir, "index_mcp")
    path = _write_config(
        model_dir,
        joint_motor_travel={"index_mcp": achieved * 2},
        calibration_travel_margin=0.1,
    )

    hand = _mock_hand(path)
    try:
        result, _ = _run(hand, joints=["index_mcp"])
    finally:
        hand.disconnect()

    motor_id = hand.config.joint_to_motor_map["index_mcp"]
    assert None not in result.motor_limits_dict[motor_id]
    assert result.joint_to_motor_ratios_dict[motor_id] != 0.0
    assert result.motor_travel_measured_dict["index_mcp"] == pytest.approx(achieved)


def test_zero_retries_reports_the_shortfall_without_re_driving(model_dir):
    achieved = _baseline_from_a_clean_run(model_dir, "index_mcp")
    path = _write_config(
        model_dir,
        joint_motor_travel={"index_mcp": achieved * 2},
        calibration_travel_margin=0.1,
        calibration_travel_retries=0,
    )

    hand = _mock_hand(path)
    try:
        _, calls = _run(hand, joints=["index_mcp"])
    finally:
        hand.disconnect()

    assert _events(calls, "travel_retry_disabled")
    assert not _events(calls, "travel_retry_started")


def test_excess_travel_is_flagged_but_never_re_driven(model_dir):
    """More current cannot shorten a span, so over-travel only warns."""
    achieved = _baseline_from_a_clean_run(model_dir, "index_mcp")
    path = _write_config(
        model_dir,
        joint_motor_travel={"index_mcp": achieved / 2},
        calibration_travel_margin=0.1,
    )

    hand = _mock_hand(path)
    try:
        _, calls = _run(hand, joints=["index_mcp"])
    finally:
        hand.disconnect()

    excess = _events(calls, "travel_excess")
    assert [event["joint"] for event in excess] == ["index_mcp"]
    assert excess[0]["deviation"] > 0
    assert not _events(calls, "travel_retry_started")


def test_a_multi_turn_wrist_is_not_re_driven(model_dir):
    """Its mode ignores the current cap, so a re-drive would change nothing."""
    achieved = _baseline_from_a_clean_run(model_dir, "wrist")
    path = _write_config(
        model_dir,
        joint_motor_travel={"wrist": achieved * 2},
        calibration_travel_margin=0.1,
    )

    hand = _mock_hand(path)
    assert hand.motor_client.supports_multi_turn
    try:
        _, calls = _run(hand, joints=["wrist"])
    finally:
        hand.disconnect()

    assert [e["joint"] for e in _events(calls, "travel_retry_unavailable")] == ["wrist"]
    assert not _events(calls, "travel_retry_started")


def test_measured_travel_is_persisted_to_the_calibration_file(model_dir):
    path = _write_config(model_dir)
    hand = _mock_hand(path)
    try:
        calibration_routine.run_calibration(
            hand, joints=["index_mcp"], persist=True
        )
    finally:
        hand.disconnect()

    doc = read_yaml(hand.config.calibration_path)
    assert doc["motor_travel_measured"]["index_mcp"] > 0


def test_calibration_done_reports_the_joints_that_needed_a_boost(model_dir):
    path = _write_config(model_dir)
    hand = _mock_hand(path)
    try:
        _, calls = _run(hand, joints=["index_mcp"])
    finally:
        hand.disconnect()

    done = _events(calls, "calibration_done")
    assert done[0]["boosted_joints"] == {}
    assert done[0]["motor_travel_deg"]["index_mcp"] > 0


# ---------------------------------------------------------------------------
# Guards against a failed sweep poisoning the calibration
#
# A motor that never turns (a latched hardware-error shutdown refuses torque)
# used to sweep "successfully" onto a degenerate limit pair. That wrote a zero
# joint-to-motor ratio, which makes the joint uncommandable, and left the wrap
# detector comparing live positions against a single point — so one bad run
# corrupted every run after it.
# ---------------------------------------------------------------------------


def test_retry_current_is_capped_at_the_calibration_ceiling(model_dir):
    """The re-drive is bounded by calibration_max_current, not max_current."""
    path = _write_config(model_dir, calibration_max_current=300,
                         calibration_current=200,
                         calibration_travel_retries=1)
    config = OrcaHandConfig.from_config_path(config_path=path)
    # 200 * 1.5 = 300, exactly at the ceiling.
    assert config.calibration_retry_current_resolved == 300
    assert config.retry_current_for_attempt(1) == pytest.approx(300.0)

    # No headroom under the calibration ceiling: the resolved retry clamps.
    path = _write_config(model_dir, calibration_max_current=300,
                         calibration_current=300,
                         calibration_travel_retries=1)
    config = OrcaHandConfig.from_config_path(config_path=path)
    assert config.calibration_retry_current_resolved == 450
    assert config.retry_current_for_attempt(1) == pytest.approx(300.0)


def test_the_re_drive_may_exceed_max_current(model_dir):
    """Freeing an over-tensioned joint takes more torque than the hand is
    allowed in normal operation, so max_current does not bound the re-drive."""
    path = _write_config(model_dir, max_current=300, calibration_current=300,
                         calibration_max_current=500,
                         calibration_travel_retries=2)
    config = OrcaHandConfig.from_config_path(config_path=path)
    assert config.retry_current_for_attempt(1) == pytest.approx(375.0)
    assert config.retry_current_for_attempt(2) == pytest.approx(450.0)
    assert config.retry_current_for_attempt(2) > config.max_current


def test_a_calibration_ceiling_below_the_nominal_pass_is_rejected(model_dir):
    """It would clamp every attempt onto the current that already stalled."""
    path = _write_config(model_dir, calibration_current=300,
                         calibration_max_current=200)
    with pytest.raises(HandConfigValidationError):
        OrcaHandConfig.from_config_path(config_path=path)


def test_degenerate_limits_are_not_committed(model_dir):
    """A sweep whose two limits coincide is rejected, not recorded.

    Committing it would write ratio 0 and leave the joint uncommandable.
    """
    from orca_core.constants import MIN_MOTOR_TRAVEL_RAD

    assert motor_travel_deg([1.5, 1.5]) == pytest.approx(0.0)
    # The guard's threshold is what separates "no travel" from a real sweep.
    assert abs(1.5 - 1.5) < MIN_MOTOR_TRAVEL_RAD
    assert abs(2.6 - 1.5) > MIN_MOTOR_TRAVEL_RAD


def test_wrap_detection_ignores_a_degenerate_limit_pair():
    """A single-point limit pair must not be read as a wrap.

    With lower == upper the +/-0.25pi tolerance is a 45 deg window around one
    point, so almost any real position looks out of bounds and earns a
    spurious 2pi shift that corrupts every angle the motor reports.
    """
    hand = MockOrcaHand()
    hand.connect()
    try:
        motor_id = hand.config.motor_ids[0]
        pos = hand.get_motor_pos(as_dict=True)[motor_id]
        # A degenerate pair placed far from where the motor actually sits.
        hand.calibration.motor_limits_dict[motor_id] = [pos + 3.0, pos + 3.0]
        hand._compute_wrap_offsets_dict()
        assert hand._wrap_offsets_dict[motor_id] == 0.0
    finally:
        hand.disconnect()


def test_zero_travel_joints_are_not_re_driven_at_higher_current():
    """A joint at ~0% of baseline never moved; more current only cooks it."""
    from orca_core.constants import MIN_TRAVEL_FRACTION

    expected = 100.0
    assert 0.0 < expected * MIN_TRAVEL_FRACTION
    # Under the floor: no re-drive. Over it: a genuine short-travel re-drive.
    assert 2.0 < expected * MIN_TRAVEL_FRACTION
    assert 50.0 > expected * MIN_TRAVEL_FRACTION


# ---------------------------------------------------------------------------
# Hardstop detection: a stable position is not on its own a hardstop
