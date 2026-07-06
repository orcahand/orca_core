"""Tests for the calibration waypoint fit: the linear fit and fold math
(round-trip exactness in both inversion cases), the settle-wait helper, and
the end-to-end calibrate() integration — fold applied for a sane scale
error, endpoint calibration kept for an insane one or when disabled."""

from __future__ import annotations

import os
import shutil
import time

import numpy as np
import pytest

from orca_core import calibration_joint_encoder as jec
from orca_core.calibration_joint_encoder import (
    JointEncoderCalibrationError,
    fit_linear_joint_map,
    fold_linear_correction,
    wait_for_settled_angles,
)
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    ENCODER_COUNTS_PER_REV,
    ENCODER_LSB_DEG,
    JOINT_ENCODER_POLARITY,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware.sensing.types import EncoderReading
from orca_core.hardware_hand import MockOrcaHand
from orca_core.utils import read_yaml, update_yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)

MOCK_MOTOR_LO, MOCK_MOTOR_HI = -1.0, 1.0  # MockDynamixelClient clamp bounds


@pytest.fixture(autouse=True)
def fast_waypoint_settle(monkeypatch):
    """Shrink the settle-detection timing so mock calibrations stay fast;
    the mock motors move instantly, so only the window length matters."""
    monkeypatch.setattr(jec, "WAYPOINT_SETTLE_WINDOW_S", 0.02)
    monkeypatch.setattr(jec, "WAYPOINT_SETTLE_POLL_S", 0.0005)
    monkeypatch.setattr(jec, "WAYPOINT_SETTLE_TIMEOUT_S", 0.5)
    monkeypatch.setattr(jec, "WAYPOINT_SETTLE_MIN_SAMPLES", 3)


# ---------------------------------------------------------------------------
# fold_linear_correction
# ---------------------------------------------------------------------------


def _plant_angle(motor, a, b, lower, ratio, rom_lo, rom_up, inverted):
    """Joint angle the encoder would read at ``motor`` for a plant whose
    response to the ORIGINAL map is ``measured = a·commanded + b``."""
    if inverted:
        commanded = rom_up - (motor - lower) / ratio
    else:
        commanded = rom_lo + (motor - lower) / ratio
    return a * commanded + b


@pytest.mark.parametrize("inverted", [False, True])
@pytest.mark.parametrize("a,b", [(0.92, 1.7), (1.1, -2.3), (1.0, 0.0)])
def test_fold_round_trip_lands_on_target(inverted, a, b):
    """Commanding θ through the folded map must land the plant exactly on θ."""
    lower, ratio, rom_lo, rom_up = -0.8, 0.025, -10.0, 80.0
    new_lower, new_upper, new_ratio = fold_linear_correction(
        a, b, lower, ratio, rom_lo, rom_up, inverted
    )
    assert new_upper == pytest.approx(new_lower + new_ratio * (rom_up - rom_lo))
    for theta in (-10.0, 0.0, 25.0, 61.5, 80.0):
        if inverted:
            motor = new_lower + (rom_up - theta) * new_ratio
        else:
            motor = new_lower + (theta - rom_lo) * new_ratio
        landed = _plant_angle(motor, a, b, lower, ratio, rom_lo, rom_up, inverted)
        assert landed == pytest.approx(theta, abs=1e-9)


def test_fold_identity_is_noop():
    lower, ratio, rom_lo, rom_up = -1.0, 0.02, 0.0, 90.0
    new_lower, new_upper, new_ratio = fold_linear_correction(
        1.0, 0.0, lower, ratio, rom_lo, rom_up, inverted=False
    )
    assert new_lower == pytest.approx(lower)
    assert new_ratio == pytest.approx(ratio)
    assert new_upper == pytest.approx(lower + ratio * (rom_up - rom_lo))


def test_fold_rejects_nonpositive_scale():
    with pytest.raises(ValueError, match="positive"):
        fold_linear_correction(0.0, 0.0, -1.0, 0.02, 0.0, 90.0, False)
    with pytest.raises(ValueError, match="positive"):
        fold_linear_correction(-0.5, 0.0, -1.0, 0.02, 0.0, 90.0, False)


# ---------------------------------------------------------------------------
# fit_linear_joint_map
# ---------------------------------------------------------------------------


def test_fit_recovers_exact_line():
    points = [(cmd, 0.93 * cmd + 1.4) for cmd in (10.0, 30.0, 50.0, 70.0)]
    a, b, residuals = fit_linear_joint_map(points)
    assert a == pytest.approx(0.93, abs=1e-12)
    assert b == pytest.approx(1.4, abs=1e-9)
    np.testing.assert_allclose(residuals, 0.0, atol=1e-9)


def test_fit_reports_nonlinearity_in_residuals():
    points = [(0.0, 0.0), (45.0, 45.0), (90.0, 92.0)]  # kink at the top
    a, b, residuals = fit_linear_joint_map(points)
    assert np.max(np.abs(residuals)) > 0.3


def test_fit_rejects_degenerate_input():
    with pytest.raises(ValueError, match="two"):
        fit_linear_joint_map([(10.0, 10.0)])
    with pytest.raises(ValueError, match="span"):
        fit_linear_joint_map([(10.0, 10.0), (10.2, 10.5), (10.4, 10.1)])


# ---------------------------------------------------------------------------
# wait_for_settled_angles
# ---------------------------------------------------------------------------


class ScriptedEncoderSource:
    """Serves one raw-count frame per ``get_latest`` from a script, repeating
    the final frame once exhausted (or forever cycling when ``cycle=True``)."""

    def __init__(self, frames, cycle: bool = False):
        self._frames = [np.asarray(f, dtype=np.uint16) for f in frames]
        self._cycle = bool(cycle)
        self._i = 0

    def get_latest(self):
        if not self._frames:
            return None
        idx = self._i % len(self._frames) if self._cycle else min(
            self._i, len(self._frames) - 1
        )
        self._i += 1
        return EncoderReading(
            raw_counts=self._frames[idx],
            parity_ok=np.ones(AUTO_ENC_NUM_JOINTS, dtype=bool),
            angle_error=np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool),
            error_byte=0,
            timestamp=time.monotonic(),
        )


def _frame_with_count(slot: int, count: int) -> np.ndarray:
    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    raw[slot] = count
    return raw


_SETTLE_ARGS = dict(
    slots=np.array([0]),
    anchors=np.array([1000]),
    polarities=np.array([1]),
    anchor_angles=np.array([90.0]),
    tol_deg=0.1,
    window_s=0.02,
    timeout_s=0.3,
    poll_period_s=0.0005,
    min_samples=3,
)


def test_settle_returns_mean_after_motion_stops():
    moving = [_frame_with_count(0, 1000 + 40 * i) for i in range(10)]
    settled = [_frame_with_count(0, 1400)]
    source = ScriptedEncoderSource(moving + settled)
    angles = wait_for_settled_angles(source, **_SETTLE_ARGS)
    expected = 90.0 + 400 * ENCODER_LSB_DEG
    assert angles[0] == pytest.approx(expected, abs=1e-9)


def test_settle_times_out_to_nan_when_never_stable():
    # ±60 counts ≈ ±1.3° of dither — never inside the 0.1° tolerance.
    frames = [_frame_with_count(0, 1000), _frame_with_count(0, 1060)]
    source = ScriptedEncoderSource(frames, cycle=True)
    angles = wait_for_settled_angles(source, **_SETTLE_ARGS)
    assert np.isnan(angles[0])


def test_settle_raises_without_any_frames():
    source = ScriptedEncoderSource([])
    with pytest.raises(JointEncoderCalibrationError, match="no encoder frames"):
        wait_for_settled_angles(source, **_SETTLE_ARGS)


# ---------------------------------------------------------------------------
# calibrate() integration
# ---------------------------------------------------------------------------


class ScaledMapEncoderSource:
    """Encoder counts consistent with the mock motor's physical map, with an
    injected scale error: the 'true' joint angle is ``alpha · map⁻¹(motor)``
    where the physical map spans the mock clamp bounds over the joint ROM.
    Because the anchor is sampled at the pressed flex pose through this same
    source, decoding yields exactly ``measured = alpha·commanded +
    (1 − alpha)·rom_up`` — a fit with known analytic outcome."""

    def __init__(self, hand, alpha: float):
        self._dxl = hand._motor_client
        self._config = hand.config
        self._alpha = float(alpha)

    def _true_angle(self, joint: str) -> float:
        motor_id = self._config.joint_to_motor_map[joint]
        motor = float(self._dxl._pos[motor_id])
        rom_lo, rom_up = self._config.joint_roms_dict[joint]
        ratio = (MOCK_MOTOR_HI - MOCK_MOTOR_LO) / (rom_up - rom_lo)
        if self._config.joint_inversion_dict.get(joint, False):
            angle = rom_up - (motor - MOCK_MOTOR_LO) / ratio
        else:
            angle = rom_lo + (motor - MOCK_MOTOR_LO) / ratio
        return self._alpha * angle

    def get_latest(self):
        raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
        for joint, slot in JOINT_TO_ENCODER_SLOT.items():
            if joint not in self._config.joint_to_motor_map or joint == "wrist":
                continue
            polarity = JOINT_ENCODER_POLARITY[joint]
            count = int(round(polarity * self._true_angle(joint) / ENCODER_LSB_DEG))
            raw[slot] = count % ENCODER_COUNTS_PER_REV
        return EncoderReading(
            raw_counts=raw,
            parity_ok=np.ones(AUTO_ENC_NUM_JOINTS, dtype=bool),
            angle_error=np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool),
            error_byte=0,
            timestamp=time.monotonic(),
        )


TEST_JOINTS = ["ring_mcp", "ring_pip"]


@pytest.fixture
def joint_feedback_hand(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    update_yaml(str(config_path), "use_joint_feedback", True)
    update_yaml(str(config_path), "joint_encoder_joints", TEST_JOINTS)
    hand = MockOrcaHand(config_path=str(config_path))
    hand.connect()
    yield hand
    hand.disconnect()


def _expected_fold(hand, joint: str, alpha: float):
    """Closed-form expected calibration values for ScaledMapEncoderSource."""
    rom_lo, rom_up = hand.config.joint_roms_dict[joint]
    endpoint_ratio = (MOCK_MOTOR_HI - MOCK_MOTOR_LO) / (rom_up - rom_lo)
    b = (1.0 - alpha) * rom_up
    return fold_linear_correction(
        alpha,
        b,
        MOCK_MOTOR_LO,
        endpoint_ratio,
        rom_lo,
        rom_up,
        hand.config.joint_inversion_dict.get(joint, False),
    )


def test_waypoint_fit_folds_scale_into_calibration(joint_feedback_hand, tmp_path):
    hand = joint_feedback_hand
    alpha = 0.95
    hand.calibrate(
        joint_encoder_client=ScaledMapEncoderSource(hand, alpha), joints=TEST_JOINTS
    )

    # Quantisation to whole encoder counts bounds each sample error by half
    # an LSB (~0.011°); through the fit and the deg→rad ratio (~0.02 rad/deg)
    # that is well under 1e-3 rad on the folded limits.
    limit_tol_rad = 2e-3
    persisted = read_yaml(str(tmp_path / "calibration.yaml"))
    for joint in TEST_JOINTS:
        motor_id = hand.config.joint_to_motor_map[joint]
        exp_lower, exp_upper, exp_ratio = _expected_fold(hand, joint, alpha)
        assert hand.motor_limits_dict[motor_id][0] == pytest.approx(
            exp_lower, abs=limit_tol_rad
        )
        assert hand.motor_limits_dict[motor_id][1] == pytest.approx(
            exp_upper, abs=limit_tol_rad
        )
        assert hand.joint_to_motor_ratios_dict[motor_id] == pytest.approx(
            exp_ratio, rel=0.005
        )
        assert persisted["motor_limits"][motor_id][0] == pytest.approx(
            exp_lower, abs=limit_tol_rad
        )
        assert persisted["joint_to_motor_ratios"][motor_id] == pytest.approx(
            exp_ratio, rel=0.005
        )


def test_waypoint_fit_out_of_bounds_keeps_endpoint_calibration(joint_feedback_hand):
    hand = joint_feedback_hand
    hand.calibrate(
        joint_encoder_client=ScaledMapEncoderSource(hand, alpha=0.5),
        joints=TEST_JOINTS,
    )

    for joint in TEST_JOINTS:
        motor_id = hand.config.joint_to_motor_map[joint]
        rom_lo, rom_up = hand.config.joint_roms_dict[joint]
        assert hand.motor_limits_dict[motor_id][0] == pytest.approx(MOCK_MOTOR_LO)
        assert hand.motor_limits_dict[motor_id][1] == pytest.approx(MOCK_MOTOR_HI)
        assert hand.joint_to_motor_ratios_dict[motor_id] == pytest.approx(
            (MOCK_MOTOR_HI - MOCK_MOTOR_LO) / (rom_up - rom_lo)
        )


def test_waypoint_fit_disabled_by_config(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    update_yaml(str(config_path), "use_joint_feedback", True)
    update_yaml(str(config_path), "joint_encoder_joints", TEST_JOINTS)
    update_yaml(str(config_path), "calibration_waypoint_fit", False)
    hand = MockOrcaHand(config_path=str(config_path))
    hand.connect()
    try:
        hand.calibrate(
            joint_encoder_client=ScaledMapEncoderSource(hand, alpha=0.95),
            joints=TEST_JOINTS,
        )
        for joint in TEST_JOINTS:
            motor_id = hand.config.joint_to_motor_map[joint]
            assert hand.motor_limits_dict[motor_id][0] == pytest.approx(MOCK_MOTOR_LO)
            assert hand.motor_limits_dict[motor_id][1] == pytest.approx(MOCK_MOTOR_HI)
    finally:
        hand.disconnect()
