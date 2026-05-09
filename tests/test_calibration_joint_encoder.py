"""Unit tests for the joint-encoder anchor-sampling helpers."""
from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from orca_core.calibration_joint_encoder import (
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


def _start_pump(link, raw_counts, period_s=0.005):
    """Background thread that feeds encoder frames at a steady rate."""
    stop = threading.Event()

    def _run():
        while not stop.is_set():
            feed_encoder_frame(link, raw_counts)
            time.sleep(period_s)

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    return stop, thread


def test_sample_anchor_count_from_client_averages(encoder_link_and_client):
    link, client = encoder_link_and_client
    counts = np.zeros(17, dtype=np.uint16)
    counts[5] = 7777
    stop, thread = _start_pump(link, counts)
    try:
        client.start_encoder_stream(timeout=1.0)
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




