# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Math + encoder-side sampling for the dual-capture joint-encoder sweep.

Pure functions plus a client-polling utility. Motor motion is driven
elsewhere; this module only reads from an encoder source.
"""
from __future__ import annotations

import math
import time
from typing import Literal, Protocol

import numpy as np

from .hardware.sensing.constants import ENCODER_COUNTS_PER_REV
from .hardware.sensing.types import EncoderReading


class JointEncoderCalibrationError(RuntimeError):
    """Raised when the joint-encoder calibration sweep cannot complete a step."""


class _ReadsLatestEncoderFrame(Protocol):
    def get_latest_encoder_reading(self) -> EncoderReading | None: ...


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


def _wrap_count_delta(delta: int) -> int:
    half_rev = ENCODER_COUNTS_PER_REV // 2
    if delta > half_rev:
        return delta - ENCODER_COUNTS_PER_REV
    if delta <= -half_rev:
        return delta + ENCODER_COUNTS_PER_REV
    return delta


def compute_polarity(
    enc_at_anchor: int,
    enc_after_tweak: int,
    anchor_end: Literal["min", "max"],
) -> int:
    """``±1`` such that ``polarity * Δenc_wrapped`` matches joint motion.
    Caller drives the joint a known direction off ``anchor_end`` and passes
    the resulting encoder count. Raises :class:`JointEncoderCalibrationError`
    if the encoder did not move.
    """
    delta_enc = _wrap_count_delta(int(enc_after_tweak) - int(enc_at_anchor))
    if delta_enc == 0:
        raise JointEncoderCalibrationError(
            "encoder count did not change during polarity tweak"
        )
    expected_joint_motion_sign = 1 if anchor_end == "min" else -1
    return int(np.sign(delta_enc)) * expected_joint_motion_sign


def sample_anchor_count_from_client(
    client: _ReadsLatestEncoderFrame,
    slot: int,
    num_samples: int = 200,
    sample_period_s: float = 0.002,
    timeout_s: float = 5.0,
) -> int:
    """Cosine-average ``num_samples`` distinct frames for ``slot`` at the
    current pose. A frame is "distinct" iff its ``timestamp`` differs from
    the previously consumed one. Raises
    :class:`JointEncoderCalibrationError` on timeout.
    """
    if num_samples <= 0:
        raise ValueError("num_samples must be positive")

    counts = np.empty(num_samples, dtype=np.uint16)
    last_ts: float | None = None
    deadline = time.monotonic() + timeout_s
    collected = 0
    while collected < num_samples:
        reading = client.get_latest_encoder_reading()
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


def sample_one_count_from_client(
    client: _ReadsLatestEncoderFrame,
    slot: int,
    timeout_s: float = 1.0,
    settle_s: float = 0.01,
    sample_period_s: float = 0.002,
) -> int:
    """Sleep ``settle_s``, then return ``slot``'s 14-bit count from the next
    frame whose timestamp differs from the one observed at call time.
    Raises :class:`JointEncoderCalibrationError` on timeout.
    """
    time.sleep(settle_s)
    deadline = time.monotonic() + timeout_s
    baseline = client.get_latest_encoder_reading()
    baseline_ts = baseline.timestamp if baseline is not None else None
    while True:
        reading = client.get_latest_encoder_reading()
        if reading is not None and reading.timestamp != baseline_ts:
            return int(reading.raw_counts[slot]) & 0x3FFF
        if time.monotonic() > deadline:
            raise JointEncoderCalibrationError(
                f"timed out waiting for fresh encoder frame on slot {slot}"
            )
        time.sleep(sample_period_s)
