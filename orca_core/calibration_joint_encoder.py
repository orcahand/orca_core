# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Encoder-side sampling for the joint-encoder anchor sweep.

Polls a client implementing ``get_latest_encoder_reading`` for fresh
frames at the current pose and averages them with cosine-mean to handle
the 14-bit encoder wraparound.
"""
from __future__ import annotations

import math
import time
from typing import Protocol

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
