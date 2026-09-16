"""Unit tests for the joint-encoder anchor-sampling helpers."""
from __future__ import annotations

import itertools
import threading
import time

import numpy as np
import pytest

from orca_core.hardware.joint_encoder_client import (
    JointEncoderCalibrationError,
    average_anchor_count,
    sample_anchor_count_from_client,
)
from orca_core.hardware.sensing.constants import ENCODER_COUNTS_PER_REV

from tests._encoder_helpers import feed_encoder_frame


# ---------------------------------------------------------------------------
# Cosine averaging
# ---------------------------------------------------------------------------


def test_average_anchor_count_simple_mean():
    samples = np.array([100, 110, 120, 90, 105], dtype=np.uint16)
    assert average_anchor_count(samples) == 105


def test_average_anchor_count_handles_wrap():
    """Samples straddling 0/16383 must average near the wrap, not near 8191."""
    near_zero = np.array(
        [16380, 16382, 0, 1, 2, 16383, 16381, 3, 4, 16383], dtype=np.uint16
    )
    avg = average_anchor_count(near_zero)
    # Expect close to 0; arithmetic mean would land at ~9831.
    assert avg < 10 or avg > ENCODER_COUNTS_PER_REV - 10


def test_average_anchor_count_empty_raises():
    with pytest.raises(JointEncoderCalibrationError):
        average_anchor_count(np.array([], dtype=np.uint16))


# ---------------------------------------------------------------------------
# Client polling helpers
# ---------------------------------------------------------------------------


def _sensor_word(count: int, *, angle_error: bool = False) -> int:
    """Assemble a u16 as the encoder chip would: angle in bits 0-13, bit 14
    the angle-error flag, bit 15 even parity over the whole word."""
    word = count | (0x4000 if angle_error else 0)
    if bin(word).count("1") % 2:
        word |= 0x8000
    return word


def _start_pump(link, raw_counts, period_s=0.005):
    """Background thread that feeds encoder frames at a steady rate,
    cycling when given a list of count arrays."""
    frames = raw_counts if isinstance(raw_counts, list) else [raw_counts]
    stop = threading.Event()

    def _run():
        for counts in itertools.cycle(frames):
            if stop.is_set():
                return
            feed_encoder_frame(link, counts)
            time.sleep(period_s)

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    return stop, thread


def test_sample_anchor_count_from_client_averages(encoder_link_and_client):
    link, client = encoder_link_and_client
    counts = np.zeros(17, dtype=np.uint16)
    counts[5] = _sensor_word(7777)
    stop, thread = _start_pump(link, counts)
    try:
        client.start_stream(timeout=1.0)
        avg = sample_anchor_count_from_client(client, slot=5, num_samples=20)
        assert avg == 7777
    finally:
        stop.set()
        thread.join(timeout=1.0)


def test_sample_anchor_count_from_client_times_out_with_no_frames(
    encoder_link_and_client,
):
    _, client = encoder_link_and_client
    with pytest.raises(JointEncoderCalibrationError, match="timed out"):
        sample_anchor_count_from_client(
            client, slot=0, num_samples=5, timeout_s=0.05
        )


def test_chip_flagged_samples_are_skipped(encoder_link_and_client):
    """Parity-fail and angle-error samples must not contaminate the anchor."""
    link, client = encoder_link_and_client
    clean = np.zeros(17, dtype=np.uint16)
    clean[5] = _sensor_word(7777)
    parity_bad = np.zeros(17, dtype=np.uint16)
    parity_bad[5] = 16  # popcount 1 with bit 15 clear: parity check fails
    angle_err = np.zeros(17, dtype=np.uint16)
    angle_err[5] = _sensor_word(16, angle_error=True)

    stop, thread = _start_pump(link, [clean, parity_bad, angle_err])
    try:
        client.start_stream(timeout=1.0)
        avg = sample_anchor_count_from_client(client, slot=5, num_samples=10)
        # Folding the flagged count (16) into the mean would drag it far off 7777.
        assert avg == 7777
    finally:
        stop.set()
        thread.join(timeout=1.0)


def test_all_flagged_samples_raise_calibration_error(encoder_link_and_client):
    link, client = encoder_link_and_client
    flagged = np.zeros(17, dtype=np.uint16)
    flagged[0] = 16  # parity check fails on slot 0

    stop, thread = _start_pump(link, flagged)
    try:
        client.start_stream(timeout=1.0)
        with pytest.raises(JointEncoderCalibrationError, match="chip-flagged"):
            sample_anchor_count_from_client(
                client, slot=0, num_samples=5, timeout_s=0.2
            )
    finally:
        stop.set()
        thread.join(timeout=1.0)




