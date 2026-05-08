"""Integration tests for JointEncoderClient on MockHandSerialLink.

Drives a real ``JointEncoderClient`` through a mock serial link with
fixture-built wire frames. Pure protocol codec tests live in
``test_encoder_protocol.py``.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np
import pytest

from orca_core.hardware.joint_encoder_client import (
    EncodersNotAvailableError,
    JointEncoderClient,
)
from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    PROTOCOL_BYTE_AUTO_ENC,
)

from tests._encoder_helpers import feed_encoder_frame


def _wait_until(predicate, timeout: float = 1.0) -> None:
    """Spin on ``predicate`` until true; let async link dispatch settle."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError(f"predicate not satisfied within {timeout}s")


# ---------------------------------------------------------------------------
# Stream lifecycle
# ---------------------------------------------------------------------------

def test_start_returns_after_first_valid_frame(encoder_link_and_client):
    link, client = encoder_link_and_client
    raw = np.arange(AUTO_ENC_NUM_JOINTS, dtype=np.uint16) + 100
    feed_encoder_frame(link, raw_counts=raw)

    client.start_encoder_stream(timeout=0.5)
    reading = client.get_latest_encoder_reading()

    assert reading is not None
    np.testing.assert_array_equal(reading.raw_counts, raw)
    assert reading.err_byte == 0


def test_start_raises_when_no_frames_arrive(encoder_link_and_client):
    _link, client = encoder_link_and_client
    with pytest.raises(EncodersNotAvailableError):
        client.start_encoder_stream(timeout=0.05)


def test_start_before_connect_raises():
    link = MockHandSerialLink()
    link.connect()
    try:
        client = JointEncoderClient(link)
        with pytest.raises(OSError):
            client.start_encoder_stream(timeout=0.05)
    finally:
        link.disconnect()


# ---------------------------------------------------------------------------
# Reading content
# ---------------------------------------------------------------------------

def test_latest_decodes_parity_angle_error_and_err_byte(encoder_link_and_client):
    link, client = encoder_link_and_client
    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    raw[3] = 0x8000  # popcount=1 → parity BAD; bit 14 clear
    raw[7] = 0x4002  # popcount=2 → parity OK; bit 14 set → angle_error
    feed_encoder_frame(link, raw_counts=raw, err_byte=0x42)

    client.start_encoder_stream(timeout=0.5)
    reading = client.get_latest_encoder_reading()

    assert reading is not None
    assert bool(reading.parity_ok[0]) is True   # raw 0x0000, popcount 0 → even
    assert bool(reading.parity_ok[3]) is False  # popcount 1 → odd → parity BAD
    assert bool(reading.parity_ok[7]) is True   # popcount 2 → even
    assert bool(reading.angle_error[7]) is True
    assert bool(reading.angle_error[0]) is False
    assert reading.err_byte == 0x42


def test_get_latest_returns_none_before_first_frame(encoder_link_and_client):
    _link, client = encoder_link_and_client
    assert client.get_latest_encoder_reading() is None


# ---------------------------------------------------------------------------
# Stats and freshness
# ---------------------------------------------------------------------------

def test_freshness_is_inf_before_first_frame(encoder_link_and_client):
    _link, client = encoder_link_and_client
    stats = client.get_encoder_stats()
    assert stats.last_frame_timestamp is None
    assert stats.last_freshness_ms == math.inf
    assert stats.frames_ok == 0


def test_freshness_finite_after_first_frame(encoder_link_and_client):
    link, client = encoder_link_and_client
    feed_encoder_frame(link)
    client.start_encoder_stream(timeout=0.5)

    stats = client.get_encoder_stats()
    assert stats.frames_ok == 1
    assert stats.last_frame_timestamp is not None
    assert 0.0 <= stats.last_freshness_ms < 200.0


# ---------------------------------------------------------------------------
# Always-parses contract
# ---------------------------------------------------------------------------

def test_handler_parses_even_before_start(encoder_link_and_client):
    """Handler runs at connect time; stats reflect link health pre-start.

    ``latest`` stays ``None`` until ``start_encoder_stream()`` flips the
    publish flag, but ``frames_ok`` and ``last_err_byte`` are updated so
    diagnostics work even when no consumer is reading yet.
    """
    link, client = encoder_link_and_client
    feed_encoder_frame(link, err_byte=0x07)

    _wait_until(lambda: client.get_encoder_stats().frames_ok >= 1)

    stats = client.get_encoder_stats()
    assert stats.frames_ok == 1
    assert stats.last_err_byte == 0x07
    assert client.get_latest_encoder_reading() is None


# ---------------------------------------------------------------------------
# Bad-frame rejection
# ---------------------------------------------------------------------------

def test_bad_lrc_does_not_update_latest(encoder_link_and_client):
    link, client = encoder_link_and_client
    good_raw = np.full(AUTO_ENC_NUM_JOINTS, 999, dtype=np.uint16)
    feed_encoder_frame(link, bad_lrc=True)
    feed_encoder_frame(link, raw_counts=good_raw)

    client.start_encoder_stream(timeout=0.5)
    reading = client.get_latest_encoder_reading()

    assert reading is not None
    np.testing.assert_array_equal(reading.raw_counts, good_raw)
    # Bad LRC is filtered by the link, not the client.
    link_stats = link.get_link_stats()
    assert link_stats.frames_bad_lrc[PROTOCOL_BYTE_AUTO_ENC] == 1


def test_wrong_eff_len_bumps_frames_bad_eff_len(encoder_link_and_client):
    link, client = encoder_link_and_client
    feed_encoder_frame(link, override_eff_len=20)
    feed_encoder_frame(link)

    client.start_encoder_stream(timeout=0.5)

    stats = client.get_encoder_stats()
    assert stats.frames_bad_eff_len == 1
    assert stats.frames_ok == 1
    assert client.get_latest_encoder_reading() is not None


# ---------------------------------------------------------------------------
# Stop / restart
# ---------------------------------------------------------------------------

def test_stop_clears_latest(encoder_link_and_client):
    link, client = encoder_link_and_client
    feed_encoder_frame(link)
    client.start_encoder_stream(timeout=0.5)
    assert client.get_latest_encoder_reading() is not None

    client.stop_encoder_stream()
    assert client.get_latest_encoder_reading() is None


def test_restart_after_stop_publishes_new_frames(encoder_link_and_client):
    link, client = encoder_link_and_client
    feed_encoder_frame(link, raw_counts=np.full(AUTO_ENC_NUM_JOINTS, 100, dtype=np.uint16))
    client.start_encoder_stream(timeout=0.5)
    client.stop_encoder_stream()

    # Frames fed while stopped must not leak into the next session.
    feed_encoder_frame(link, raw_counts=np.full(AUTO_ENC_NUM_JOINTS, 200, dtype=np.uint16))
    _wait_until(lambda: client.get_encoder_stats().frames_ok >= 2)
    assert client.get_latest_encoder_reading() is None

    # start_encoder_stream blocks until a frame arrives with publishing on,
    # so the test feeds from the main thread once start has reached its wait.
    def _start_in_thread():
        client.start_encoder_stream(timeout=1.0)

    t = threading.Thread(target=_start_in_thread, daemon=True)
    t.start()
    time.sleep(0.05)
    feed_encoder_frame(link, raw_counts=np.full(AUTO_ENC_NUM_JOINTS, 300, dtype=np.uint16))
    t.join(timeout=1.0)
    assert not t.is_alive()

    reading = client.get_latest_encoder_reading()
    assert reading is not None
    assert reading.raw_counts[0] == 300


def test_failed_start_clears_publish_flag(encoder_link_and_client):
    """A timed-out start must not leave the client in a half-publishing state."""
    link, client = encoder_link_and_client
    with pytest.raises(EncodersNotAvailableError):
        client.start_encoder_stream(timeout=0.05)

    # If publish were still active, this frame would set ``latest``.
    feed_encoder_frame(link)
    _wait_until(lambda: client.get_encoder_stats().frames_ok >= 1)
    assert client.get_latest_encoder_reading() is None
