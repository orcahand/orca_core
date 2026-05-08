"""Unit tests for the joint-encoder dual-capture sweep helpers."""
from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from orca_core.calibration_joint_encoder import (
    JointEncoderCalibrationError,
    average_anchor_count,
    compute_polarity,
    sample_anchor_count_from_client,
    sample_one_count_from_client,
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
# Polarity computation
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "anchor_end,delta,expected",
    [
        # min anchor → joint angle increases on tweak. Encoder going up means
        # polarity +1; encoder going down means polarity -1.
        ("min", +20, +1),
        ("min", -20, -1),
        # max anchor → joint angle decreases on tweak. Encoder going up means
        # polarity -1; encoder going down means polarity +1.
        ("max", +20, -1),
        ("max", -20, +1),
    ],
)
def test_compute_polarity_signs(anchor_end, delta, expected):
    assert compute_polarity(1000, 1000 + delta, anchor_end) == expected


def test_compute_polarity_handles_wrap():
    # Anchor near top, after-tweak near 0: raw delta -16380 → wraps to +4.
    assert compute_polarity(16382, 2, "min") == 1
    # Anchor near 0, after-tweak near top: raw delta +16380 → wraps to -4.
    assert compute_polarity(2, 16382, "min") == -1


def test_compute_polarity_zero_delta_raises():
    with pytest.raises(JointEncoderCalibrationError, match="did not change"):
        compute_polarity(1000, 1000, "min")


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


def test_sample_one_count_from_client_returns_fresh_count(encoder_link_and_client):
    link, client = encoder_link_and_client
    counts = np.zeros(17, dtype=np.uint16)
    counts[3] = 1234
    feed_encoder_frame(link, counts)
    client.start_encoder_stream(timeout=1.0)

    counts[3] = 4321
    stop, thread = _start_pump(link, counts)
    try:
        observed = sample_one_count_from_client(
            client, slot=3, timeout_s=1.0, settle_s=0.0
        )
        assert observed == 4321
    finally:
        stop.set()
        thread.join(timeout=1.0)


def test_sample_one_count_from_client_strips_flag_bits(encoder_link_and_client):
    link, client = encoder_link_and_client
    raw = np.zeros(17, dtype=np.uint16)
    raw[7] = 1500  # baseline frame
    feed_encoder_frame(link, raw)
    client.start_encoder_stream(timeout=1.0)

    raw[7] = 1500 | (1 << 14) | (1 << 15)  # set flag bits; sampler must mask
    stop, thread = _start_pump(link, raw)
    try:
        observed = sample_one_count_from_client(
            client, slot=7, timeout_s=1.0, settle_s=0.0
        )
        assert observed == 1500
    finally:
        stop.set()
        thread.join(timeout=1.0)


