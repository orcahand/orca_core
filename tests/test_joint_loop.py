"""Tests for ``JointLoopThread``: bumpless start, full data path through
encoder → controller → vectorised joint→motor → ``write_desired_pos``,
watchdog tiers, agreement with the open-loop kinematics, and the jitter
monitor.
"""

from __future__ import annotations

import logging
import os
import shutil
import time

import numpy as np
import pytest

from orca_core.control import JointController, JointLoopThread
from orca_core.control.constants import (
    WATCHDOG_HOLD_BASE_MS,
    WATCHDOG_HOLD_MS,
    WATCHDOG_STOP_LOOP_MS,
    WATCHDOG_WARN_MS,
)

from tests._loop_helpers import (
    StaticEncoderSource,
    encoder_reading_from_joint_angles,
    make_calibrated_hand,
)


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


@pytest.fixture
def calibrated_hand(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    hand = make_calibrated_hand(str(config_path))
    yield hand
    hand.disconnect()


def _make_loop(
    hand,
    encoder_source,
    *,
    Kp=0.1,
    Ki=0.0,
    correction_max_deg=15.0,
    target_hz=100,
):
    controller = JointController(num_joints=len(hand._encoder_backed_joints()))
    controller.set_gains(
        Kp=Kp, Ki=Ki,
        correction_max_deg=correction_max_deg,
        i_clamp_deg=correction_max_deg,
    )
    return JointLoopThread(hand, encoder_source, controller, target_hz=target_hz)


def _static_at_zero(hand, freshness_ms: float = 0.0) -> StaticEncoderSource:
    return StaticEncoderSource(
        encoder_reading_from_joint_angles(
            hand, {j: 0.0 for j in hand._encoder_backed_joints()},
        ),
        freshness_ms=freshness_ms,
    )


def _expected_motor_pos(hand, joint: str, joint_command_deg: float) -> float:
    """Open-loop scalar mapping, for cross-checking the loop's vectorised one."""
    motor_id = hand.config.joint_to_motor_map[joint]
    idx = hand.config.motor_id_to_idx_dict[motor_id]
    return hand._joint_to_motor_pos({joint: joint_command_deg})[idx]


def test_bumpless_first_step_leaves_motors_where_they_are(calibrated_hand):
    """At ``prime_for_step`` the target latches to the measured pose and the
    feed-forward bias absorbs the motor↔encoder calibration offset, so the
    first ``step_once`` produces a zero correction and leaves every motor at
    its current position (no startup lurch)."""
    measured_angle_deg = 3.0
    encoder = StaticEncoderSource(
        encoder_reading_from_joint_angles(
            calibrated_hand,
            {j: measured_angle_deg for j in calibrated_hand._encoder_backed_joints()},
        )
    )
    loop = _make_loop(calibrated_hand, encoder, Kp=1.0)
    loop.prime_for_step()
    pos_before = {
        calibrated_hand.config.joint_to_motor_map[joint]: calibrated_hand._motor_client._pos[
            calibrated_hand.config.joint_to_motor_map[joint]
        ]
        for joint in calibrated_hand._encoder_backed_joints()
    }
    loop.step_once(dt=0.01)

    correction = loop.get_correction()
    for joint in calibrated_hand._encoder_backed_joints():
        assert correction[joint] == pytest.approx(0.0, abs=1e-9)
        motor_id = calibrated_hand.config.joint_to_motor_map[joint]
        actual = calibrated_hand._motor_client._pos[motor_id]
        assert actual == pytest.approx(pos_before[motor_id], abs=1e-6)


def test_step_once_applies_correction_to_motor_target(calibrated_hand):
    """Full data path: encoder → controller → vectorised joint→motor →
    ``write_desired_pos``. Each motor command equals the kinematics of the
    corrected joint command, with inversion respected."""
    encoder = _static_at_zero(calibrated_hand)
    Kp, target_offset_deg = 0.5, 5.0
    loop = _make_loop(calibrated_hand, encoder, Kp=Kp)
    loop.prime_for_step()
    loop.set_target(
        {j: target_offset_deg for j in calibrated_hand._encoder_backed_joints()}
    )
    loop.step_once(dt=0.01)

    # The encoder round-trip is quantised to one 14-bit LSB (~0.022°/LSB),
    # so the measured pose can land up to that much off zero; tolerance
    # follows the LSB scaled by Kp.
    expected_correction = Kp * target_offset_deg
    enc_quantisation_deg = 360.0 / 16384
    correction_tol = Kp * enc_quantisation_deg
    correction = loop.get_correction()
    for joint in calibrated_hand._encoder_backed_joints():
        assert correction[joint] == pytest.approx(expected_correction, abs=correction_tol)
        motor_id = calibrated_hand.config.joint_to_motor_map[joint]
        bias = loop._motor_bias[loop._joint_names.index(joint)]
        expected_motor = _expected_motor_pos(
            calibrated_hand, joint, target_offset_deg + correction[joint],
        ) + bias
        assert calibrated_hand._motor_client._pos[motor_id] == pytest.approx(
            expected_motor, abs=1e-6
        )


def test_rebase_is_bumpless_after_motors_moved(calibrated_hand):
    """After the motors are moved out from under a running loop (torque off)
    and the integrator has wound up, ``rebase()`` re-anchors so the next
    ``step_once`` leaves the motors where they are — no lurch."""
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder, Kp=1.0, Ki=8.0)
    loop.prime_for_step()
    # Wind up the integrator by holding a target away from the measured pose.
    loop.set_target({j: 8.0 for j in calibrated_hand._encoder_backed_joints()})
    for _ in range(20):
        loop.step_once(dt=0.01)
    assert np.any(loop._controller.get_state()["ierr_deg"] != 0.0)

    # Simulate the hand being posed by hand while torque was off.
    for mid in loop._motor_ids:
        calibrated_hand._motor_client._pos[mid] = 0.123

    loop.rebase()
    assert np.allclose(loop._controller.get_state()["ierr_deg"], 0.0)
    pos_before = dict(calibrated_hand._motor_client._pos)
    loop.step_once(dt=0.01)
    assert calibrated_hand._motor_client._pos == pytest.approx(pos_before, abs=1e-6)


def test_step_once_with_no_reading_writes_nothing(calibrated_hand):
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    loop.prime_for_step()
    encoder.set_reading(None)
    pos_before = dict(calibrated_hand._motor_client._pos)
    loop.step_once(dt=0.01)

    stats = loop.get_stats()
    assert stats["cycles_no_reading"] == 1
    assert stats["cycles_ok"] == 0
    assert calibrated_hand._motor_client._pos == pos_before


def test_prime_rejects_missing_encoder_reading(calibrated_hand):
    loop = _make_loop(calibrated_hand, StaticEncoderSource(reading=None))
    with pytest.raises(RuntimeError, match="no valid encoder reading"):
        loop.prime_for_step()


def test_tier1_warns_with_rate_limit_but_continues_normal_control(calibrated_hand, caplog):
    encoder = _static_at_zero(calibrated_hand, freshness_ms=WATCHDOG_WARN_MS + 5)
    loop = _make_loop(calibrated_hand, encoder, Kp=0.5, Ki=1.0)
    loop.prime_for_step()
    loop.set_target({j: 5.0 for j in calibrated_hand._encoder_backed_joints()})

    with caplog.at_level(logging.WARNING):
        for _ in range(50):
            loop.step_once(dt=0.001)

    assert np.all(loop._controller.get_state()["ierr_deg"] > 0.0)
    freshness_records = [
        r for r in caplog.records
        if "freshness" in r.getMessage().lower() and r.levelno == logging.WARNING
    ]
    assert 1 <= len(freshness_records) <= 3


def test_tier2_freezes_integrator_and_recovers(calibrated_hand):
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder, Kp=0.05, Ki=1.0)
    loop.prime_for_step()
    loop.set_target({j: 5.0 for j in calibrated_hand._encoder_backed_joints()})
    for _ in range(3):
        loop.step_once(dt=0.005)
    ierr_before = loop._controller.get_state()["ierr_deg"].copy()

    encoder.freshness_ms = WATCHDOG_HOLD_MS + 10
    for _ in range(10):
        loop.step_once(dt=0.005)
    np.testing.assert_allclose(loop._controller.get_state()["ierr_deg"], ierr_before)
    assert loop._controller.integral_frozen

    encoder.freshness_ms = 1.0
    loop.step_once(dt=0.005)
    assert not loop._controller.integral_frozen


def test_tier3_drops_trim_writes_base_motor_target_only(calibrated_hand):
    """At >200 ms freshness the PI trim is dropped: each cycle writes the
    base motor target for the (uncorrected) joint target so the motor's
    internal PID continues to hold the requested pose."""
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder, Kp=10.0)
    loop.prime_for_step()
    encoder.freshness_ms = WATCHDOG_HOLD_BASE_MS + 50
    target_deg = 4.0
    loop.set_target({j: target_deg for j in calibrated_hand._encoder_backed_joints()})
    loop.step_once(dt=0.005)

    for joint in calibrated_hand._encoder_backed_joints():
        motor_id = calibrated_hand.config.joint_to_motor_map[joint]
        bias = loop._motor_bias[loop._joint_names.index(joint)]
        expected = _expected_motor_pos(calibrated_hand, joint, target_deg) + bias
        assert calibrated_hand._motor_client._pos[motor_id] == pytest.approx(
            expected, abs=1e-6
        )
    stats = loop.get_stats()
    assert stats["cycles_held_base"] >= 1
    assert stats["e_stops"] == 0, "tier-3 must not count as an e-stop"


def test_tier4_sets_fallback_active(calibrated_hand):
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    loop.prime_for_step()

    encoder.freshness_ms = WATCHDOG_STOP_LOOP_MS + 100
    loop.step_once(dt=0.005)

    assert loop.get_stats()["fallback_active"] is True


def test_motor_pos_matches_open_loop_kinematics(calibrated_hand):
    """The loop's vectorised joint→motor mapping must agree with
    :meth:`OrcaHand._joint_to_motor_pos` for the same joint target. Guards
    against re-introducing a unit-scale bug on motor writes."""
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    loop.prime_for_step()

    encoder_joints = calibrated_hand._encoder_backed_joints()
    values = [5.0, -5.0, 28.0, -28.0]
    target_deg = {
        joint: values[i % len(values)] for i, joint in enumerate(encoder_joints)
    }
    target_array = np.array(
        [target_deg[j] for j in loop._joint_names], dtype=np.float64
    )
    loop_motor = loop._joint_to_motor_pos(target_array)

    open_loop_motor = calibrated_hand._joint_to_motor_pos(target_deg)
    motor_id_to_idx = calibrated_hand.config.motor_id_to_idx_dict
    for idx, joint in enumerate(loop._joint_names):
        motor_id = loop._motor_ids[idx]
        expected = open_loop_motor[motor_id_to_idx[motor_id]]
        assert loop_motor[idx] == pytest.approx(expected, abs=1e-9)


def test_jitter_monitor_estops_after_pathological_streak(calibrated_hand):
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    loop.prime_for_step()

    loop._record_loop_period(loop._target_period * 12.0)
    assert loop.get_stats()["fallback_active"] is False
    loop._record_loop_period(loop._target_period)
    for _ in range(4):
        loop._record_loop_period(loop._target_period * 12.0)
    assert loop.get_stats()["fallback_active"] is False
    loop._record_loop_period(loop._target_period * 12.0)
    assert loop.get_stats()["fallback_active"] is True


def test_pause_writes_holds_motors_but_keeps_measurements_live(calibrated_hand):
    """While writes are paused, ``step_once`` issues no motor command (so a
    concurrent round-trip op — e.g. a torque toggle — isn't stalled by the
    loop's sync_writes) yet still decodes the encoder into the measured pose.
    Resuming restores normal command output."""
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder, Kp=1.0)
    loop.prime_for_step()
    loop.set_target({j: 5.0 for j in calibrated_hand._encoder_backed_joints()})

    def motor_snapshot():
        return {
            mid: calibrated_hand._motor_client._pos[mid]
            for mid in (calibrated_hand.config.joint_to_motor_map[j]
                        for j in calibrated_hand._encoder_backed_joints())
        }

    before = motor_snapshot()
    loop.pause_writes()
    loop.step_once(dt=0.01)

    # No motor moved while paused ...
    assert motor_snapshot() == before
    # ... but the measured pose is still updated from the encoder stream.
    measured = loop.get_measured_joints()
    for j in calibrated_hand._encoder_backed_joints():
        assert measured[j] == pytest.approx(0.0, abs=0.05)  # within one encoder LSB

    loop.resume_writes()
    loop.step_once(dt=0.01)
    assert motor_snapshot() != before  # commands flow again


def test_prime_rejects_failed_motor_read(calibrated_hand):
    """A motor read the bus never answered must not be anchored into the
    feed-forward bias — that would command the affected motors toward
    absolute zero every cycle. The anchor raises instead."""
    reader = getattr(calibrated_hand._motor_client, "_pos_vel_cur_reader", None)
    if reader is None:
        pytest.skip("mock has no SDK reader to simulate a failed read")
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    reader.last_read_ok = False
    try:
        with pytest.raises(RuntimeError, match="anchoring the joint loop"):
            loop.prime_for_step()
    finally:
        reader.last_read_ok = True


def test_chip_flagged_joint_holds_previous_measurement(calibrated_hand):
    """A sample the encoder chip flags as invalid (parity fail or angle-error
    bit) must not feed the PI a phantom angle: the joint holds its previous
    measured value for the cycle and the occurrence is counted in stats."""
    joints = calibrated_hand._encoder_backed_joints()
    encoder = StaticEncoderSource(
        encoder_reading_from_joint_angles(calibrated_hand, {j: 3.0 for j in joints})
    )
    loop = _make_loop(calibrated_hand, encoder, Kp=1.0)
    loop.prime_for_step()

    parity_joint, error_joint = loop._joint_names[0], loop._joint_names[1]
    new_reading = encoder_reading_from_joint_angles(
        calibrated_hand, {j: 8.0 for j in joints}
    )
    new_reading.parity_ok[loop._slots[0]] = False
    new_reading.angle_error[loop._slots[1]] = True
    encoder.set_reading(new_reading)
    loop.step_once(dt=0.01)

    measured = loop.get_measured_joints()
    assert measured[parity_joint] == pytest.approx(3.0, abs=0.05)
    assert measured[error_joint] == pytest.approx(3.0, abs=0.05)
    for joint in loop._joint_names[2:]:
        assert measured[joint] == pytest.approx(8.0, abs=0.05)
    assert loop.get_stats()["joints_flagged_invalid"] == 2


def test_start_after_estop_clears_fallback_and_runs(calibrated_hand):
    """A deliberate ``start()`` after an e-stop must clear ``fallback_active``
    so the restarted loop actually reads encoders and writes motors."""
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    loop.prime_for_step()
    encoder.freshness_ms = WATCHDOG_STOP_LOOP_MS + 100
    loop.step_once(dt=0.005)
    assert loop.get_stats()["fallback_active"] is True

    encoder.freshness_ms = 0.0
    loop.start()
    try:
        assert loop.get_stats()["fallback_active"] is False
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and loop.get_stats()["cycles_ok"] == 0:
            time.sleep(0.01)
        assert loop.get_stats()["cycles_ok"] > 0
    finally:
        loop.stop()


def test_anchor_rejects_stale_encoder_reading(calibrated_hand):
    """A cached reading older than the watchdog hold tier must not be
    anchored: the pose it describes may be long gone (e.g. hand-posed with
    the stream stalled), so the anchor raises instead of latching it."""
    encoder = _static_at_zero(calibrated_hand, freshness_ms=WATCHDOG_HOLD_MS + 10)
    loop = _make_loop(calibrated_hand, encoder)
    with pytest.raises(RuntimeError, match="stale"):
        loop.prime_for_step()


def test_snapshot_rejects_selected_joint_without_motor_calibration(calibrated_hand):
    """With the default joints=None a joint carrying an encoder anchor but a
    zero joint-to-motor ratio must fail the snapshot loudly instead of
    becoming a silent no-op (or NaN targets for None limits)."""
    victim = calibrated_hand._encoder_backed_joints()[0]
    victim_motor = calibrated_hand.config.joint_to_motor_map[victim]
    calibrated_hand.calibration.joint_to_motor_ratios_dict[victim_motor] = 0.0
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    with pytest.raises(RuntimeError, match="joint-to-motor ratio"):
        loop.prime_for_step()


def test_anchor_reads_motor_flag_under_motor_lock(calibrated_hand):
    """The anchor's read-ok flag must be captured while the motor lock is
    still held, so a concurrent reader can't overwrite it in between."""
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    real_client = calibrated_hand._motor_client
    lock_owned_at_flag_read = []

    class _FlagProbe:
        def __getattr__(self, name):
            if name == "last_read_ok":
                lock_owned_at_flag_read.append(
                    calibrated_hand._motor_lock._is_owned()
                )
            return getattr(real_client, name)

    calibrated_hand._motor_client = _FlagProbe()
    try:
        loop.prime_for_step()
    finally:
        calibrated_hand._motor_client = real_client
    assert lock_owned_at_flag_read and all(lock_owned_at_flag_read)


def test_snapshot_rejects_unvalidated_hand_side(calibrated_hand, monkeypatch):
    """A hand side without a validated encoder polarity table must fail the
    snapshot instead of silently decoding with right-hand signs."""
    import dataclasses as dc

    monkeypatch.setattr(
        calibrated_hand, "config", dc.replace(calibrated_hand.config, type="left")
    )
    encoder = _static_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, encoder)
    with pytest.raises(ValueError, match="polarity"):
        loop.prime_for_step()


def test_offset_read_rejects_failed_bus_read(calibrated_hand):
    """A failed bulk read (reader reports not-ok) must not be turned into wrap
    offsets — the guard raises instead of baking -2π from stale cache."""
    reader = getattr(calibrated_hand._motor_client, "_pos_vel_cur_reader", None)
    if reader is None:
        pytest.skip("mock has no SDK reader to simulate a failed read")
    reader.last_read_ok = False
    with pytest.raises(RuntimeError, match="no status packets"):
        calibrated_hand._read_motor_pos_for_offsets(retries=2, retry_interval=0.0)
