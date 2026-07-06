# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Encoder-side sampling and map fitting for the joint-encoder calibration.

Anchor sweep: polls a client implementing ``get_latest`` for fresh
frames at the current pose and averages them with cosine-mean to handle
the 14-bit encoder wraparound.

Waypoint fit: after a joint's endpoint calibration completes, the hand
commands settled interior waypoints and measures them with the joint
encoder. A linear fit ``measured = a·commanded + b`` is folded back into
the motor limits and joint-to-motor ratio, correcting both the offset
and the scale that the endpoint-only calibration gets wrong (the motor
limits are read after a torque release, so the endpoint map inherits
the tendon settle-back bias). The runtime map keeps its exact form —
only the calibrated numbers improve.
"""
from __future__ import annotations

import math
import time
from collections import deque
from typing import Protocol, Sequence, Tuple

import numpy as np

from .hardware.sensing.constants import ENCODER_COUNTS_PER_REV
from .hardware.sensing.encoder_protocol import encoder_to_joint_angle
from .hardware.sensing.types import EncoderReading


# Waypoint-fit pass. Fractions are of the joint ROM; the sweep visits them
# moving away from the just-pressed hardstop, then re-samples any that lie
# on the way back to neutral, so the fit mixes both approach directions and
# the out-vs-back difference doubles as a hysteresis diagnostic. A pose
# counts as settled when the decoded angle's peak-to-peak over the trailing
# window is below the tolerance; the window mean is the measurement, so the
# wait is exactly as long as the mechanics need. The scale/offset bounds
# reject fits that cannot be a plausible map correction (wiring or polarity
# faults, a stalled motor); the endpoint calibration is kept in that case.
WAYPOINT_FIT_FRACTIONS = (0.25, 0.5, 0.75)
WAYPOINT_SETTLE_TOL_DEG = 0.15
WAYPOINT_SETTLE_WINDOW_S = 0.15
WAYPOINT_SETTLE_TIMEOUT_S = 2.5
WAYPOINT_SETTLE_POLL_S = 0.005
WAYPOINT_SETTLE_MIN_SAMPLES = 5
WAYPOINT_FIT_MIN_POINTS = 3
WAYPOINT_FIT_SCALE_BOUNDS = (0.75, 1.25)
WAYPOINT_FIT_OFFSET_MAX_DEG = 10.0
WAYPOINT_FIT_RESIDUAL_WARN_DEG = 0.75


class JointEncoderCalibrationError(RuntimeError):
    """Raised when the joint-encoder calibration sweep cannot complete a step."""


class _ReadsLatestEncoderFrame(Protocol):
    def get_latest(self) -> EncoderReading | None: ...


def average_anchor_count(samples: np.ndarray) -> int:
    """Cosine-mean of 14-bit encoder counts; correct across the 16383→0 wrap."""
    if samples.size == 0:
        raise JointEncoderCalibrationError("cannot average empty sample buffer")
    angles = samples.astype(np.float64) * (2.0 * math.pi / ENCODER_COUNTS_PER_REV)
    mean_angle = math.atan2(float(np.mean(np.sin(angles))), float(np.mean(np.cos(angles))))
    if mean_angle < 0:
        mean_angle += 2.0 * math.pi
    count = int(round(mean_angle * ENCODER_COUNTS_PER_REV / (2.0 * math.pi)))
    return count % ENCODER_COUNTS_PER_REV


def sample_anchor_count_from_client(
    client: _ReadsLatestEncoderFrame,
    slot: int,
    num_samples: int = 200,
    sample_period_s: float = 0.002,
    timeout_s: float = 5.0,
) -> int:
    """Cosine-average ``num_samples`` distinct frames for ``slot`` at the
    current pose.
    """
    if num_samples <= 0:
        raise ValueError("num_samples must be positive")

    counts = np.empty(num_samples, dtype=np.uint16)
    last_ts: float | None = None
    deadline = time.monotonic() + timeout_s
    collected = 0
    while collected < num_samples:
        reading = client.get_latest()
        if reading is not None and reading.timestamp != last_ts:
            counts[collected] = int(reading.raw_counts[slot]) & 0x3FFF
            last_ts = reading.timestamp
            collected += 1
            continue
        if time.monotonic() > deadline:
            raise JointEncoderCalibrationError(
                f"timed out waiting for encoder samples on slot {slot} "
                f"(got {collected}/{num_samples} in {timeout_s}s)"
            )
        time.sleep(sample_period_s)
    return average_anchor_count(counts)


def wait_for_settled_angles(
    client: _ReadsLatestEncoderFrame,
    slots: np.ndarray,
    anchors: np.ndarray,
    polarities: np.ndarray,
    anchor_angles: np.ndarray,
    tol_deg: float | None = None,
    window_s: float | None = None,
    timeout_s: float | None = None,
    poll_period_s: float | None = None,
    min_samples: int | None = None,
) -> np.ndarray:
    """Wait until every joint's decoded angle is stable, then return the
    per-joint mean over the stable window (degrees).

    A joint is settled when the peak-to-peak of its decoded angle over the
    trailing ``window_s`` is below ``tol_deg`` with at least ``min_samples``
    distinct frames. The same window doubles as the measurement, so the call
    blocks exactly as long as the mechanics need. Joints still moving at
    ``timeout_s`` return ``NaN`` so the caller can skip that sample.
    ``None`` parameters resolve to the module ``WAYPOINT_SETTLE_*`` constants
    at call time.

    Raises:
        JointEncoderCalibrationError: no encoder frames arrived at all.
    """
    tol = WAYPOINT_SETTLE_TOL_DEG if tol_deg is None else float(tol_deg)
    window = WAYPOINT_SETTLE_WINDOW_S if window_s is None else float(window_s)
    timeout = WAYPOINT_SETTLE_TIMEOUT_S if timeout_s is None else float(timeout_s)
    poll = WAYPOINT_SETTLE_POLL_S if poll_period_s is None else float(poll_period_s)
    min_n = WAYPOINT_SETTLE_MIN_SAMPLES if min_samples is None else int(min_samples)

    samples: deque[tuple[float, np.ndarray]] = deque()
    last_ts: float | None = None
    deadline = time.monotonic() + timeout

    def _window_is_full() -> bool:
        return (
            len(samples) >= min_n
            and samples[-1][0] - samples[0][0] >= window * 0.8
        )

    while True:
        reading = client.get_latest()
        now = time.monotonic()
        if reading is not None and reading.timestamp != last_ts:
            last_ts = reading.timestamp
            counts = np.asarray(reading.raw_counts)[slots]
            angles = np.asarray(
                encoder_to_joint_angle(counts, anchors, polarities, anchor_angles),
                dtype=np.float64,
            )
            samples.append((now, angles))
            while samples and samples[0][0] < now - window:
                samples.popleft()
            if _window_is_full():
                stack = np.stack([a for _, a in samples])
                if np.all(np.ptp(stack, axis=0) <= tol):
                    return stack.mean(axis=0)
        if now > deadline:
            if not samples:
                raise JointEncoderCalibrationError(
                    f"no encoder frames arrived within {timeout}s while "
                    "waiting for a settled pose"
                )
            stack = np.stack([a for _, a in samples])
            means = stack.mean(axis=0)
            if _window_is_full():
                means[np.ptp(stack, axis=0) > tol] = np.nan
            else:
                means[:] = np.nan
            return means
        time.sleep(poll)


def fit_linear_joint_map(
    points: Sequence[Tuple[float, float]],
) -> Tuple[float, float, np.ndarray]:
    """Least-squares fit of ``measured = a·commanded + b``.

    Returns ``(a, b, residuals)`` with residuals in measurement order.

    Raises:
        ValueError: fewer than two points, or the commanded angles span less
            than one degree (no slope information).
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 2 or pts.shape[0] < 2:
        raise ValueError("need at least two (commanded, measured) points")
    cmd, meas = pts[:, 0], pts[:, 1]
    if float(np.ptp(cmd)) < 1.0:
        raise ValueError("commanded angles span less than 1°; cannot fit a slope")
    design = np.vstack([cmd, np.ones_like(cmd)]).T
    (a, b), *_ = np.linalg.lstsq(design, meas, rcond=None)
    residuals = meas - (a * cmd + b)
    return float(a), float(b), residuals


def fold_linear_correction(
    a: float,
    b: float,
    lower: float,
    ratio: float,
    rom_lo: float,
    rom_up: float,
    inverted: bool,
) -> Tuple[float, float, float]:
    """Fold a measured linear response ``measured = a·commanded + b`` back
    into the joint→motor map parameters.

    The corrected map commands the motor position at which the joint
    encoder reads the requested angle: ``motor'(θ) = map((θ − b) / a)``,
    re-expressed in the map's own ``(lower, upper, ratio)`` parameters so
    the runtime mapping code is untouched. Returns
    ``(lower', upper', ratio')`` with ``upper'`` recomputed to keep the
    ``ratio = (upper − lower) / (rom_up − rom_lo)`` invariant.

    Raises:
        ValueError: non-positive scale ``a`` (a map correction cannot flip
            or collapse the direction of motion).
    """
    if a <= 0:
        raise ValueError(f"scale must be positive, got a={a}")
    new_ratio = ratio / a
    if inverted:
        new_lower = lower + ((a - 1.0) * rom_up + b) * ratio / a
    else:
        new_lower = lower + ((1.0 - a) * rom_lo - b) * ratio / a
    new_upper = new_lower + new_ratio * (rom_up - rom_lo)
    return float(new_lower), float(new_upper), float(new_ratio)
