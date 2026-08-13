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
    AUTO_ENC_ANGLE_ERROR_BIT,
    AUTO_ENC_ANGLE_MASK,
    AUTO_ENC_NUM_JOINTS,
    AUTO_ENC_PARITY_BIT,
    PROTOCOL_BYTE_AUTO_ENC,
)

from tests._encoder_helpers import feed_encoder_frame, full_counts
from tests._helpers import wait_until


# ---------------------------------------------------------------------------
# Stream lifecycle
# ---------------------------------------------------------------------------

def test_start_returns_after_first_valid_frame(encoder_link_and_client):
    link, client = encoder_link_and_client
    raw = np.arange(AUTO_ENC_NUM_JOINTS, dtype=np.uint16) + 100
    feed_encoder_frame(link, raw_counts=raw)

    client.start_stream(timeout=0.5)
    reading = client.get_latest()

    assert reading is not None
    np.testing.assert_array_equal(reading.raw_counts, raw)
    assert reading.error_byte == 0


def test_start_raises_when_no_frames_arrive(encoder_link_and_client):
    _link, client = encoder_link_and_client
    with pytest.raises(EncodersNotAvailableError):
        client.start_stream(timeout=0.05)


def test_start_before_connect_raises():
    link = MockHandSerialLink()
    link.connect()
    try:
        client = JointEncoderClient(link)
        with pytest.raises(OSError):
            client.start_stream(timeout=0.05)
    finally:
        link.disconnect()


# ---------------------------------------------------------------------------
# Reading content
# ---------------------------------------------------------------------------

def test_latest_decodes_parity_angle_error_and_error_byte(encoder_link_and_client):
    link, client = encoder_link_and_client
    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    raw[3] = 0x8000  # popcount=1 → parity BAD; bit 14 clear
    raw[7] = 0x4002  # popcount=2 → parity OK; bit 14 set → angle_error
    feed_encoder_frame(link, raw_counts=raw, error_byte=0x42)

    client.start_stream(timeout=0.5)
    reading = client.get_latest()

    assert reading is not None
    assert bool(reading.parity_ok[0]) is True   # raw 0x0000, popcount 0 → even
    assert bool(reading.parity_ok[3]) is False  # popcount 1 → odd → parity BAD
    assert bool(reading.parity_ok[7]) is True   # popcount 2 → even
    assert bool(reading.angle_error[7]) is True
    assert bool(reading.angle_error[0]) is False
    assert reading.error_byte == 0x42


def test_get_latest_returns_none_before_first_frame(encoder_link_and_client):
    _link, client = encoder_link_and_client
    assert client.get_latest() is None


# ---------------------------------------------------------------------------
# Stats and freshness
# ---------------------------------------------------------------------------

def test_freshness_is_inf_before_first_frame(encoder_link_and_client):
    _link, client = encoder_link_and_client
    stats = client.get_stats()
    assert stats.last_frame_timestamp is None
    assert stats.last_freshness_ms == math.inf
    assert stats.frames_ok == 0


def test_freshness_finite_after_first_frame(encoder_link_and_client):
    link, client = encoder_link_and_client
    feed_encoder_frame(link)
    client.start_stream(timeout=0.5)

    stats = client.get_stats()
    assert stats.frames_ok == 1
    assert stats.last_frame_timestamp is not None
    assert 0.0 <= stats.last_freshness_ms < 200.0


# ---------------------------------------------------------------------------
# Always-parses contract
# ---------------------------------------------------------------------------

def test_handler_parses_even_before_start(encoder_link_and_client):
    """Handler runs at connect time; stats reflect link health pre-start.

    ``latest`` stays ``None`` until ``start_stream()`` flips the
    publish flag, but ``frames_ok`` and ``last_error_byte`` are updated so
    diagnostics work even when no consumer is reading yet.
    """
    link, client = encoder_link_and_client
    feed_encoder_frame(link, error_byte=0x07)

    wait_until(lambda: client.get_stats().frames_ok >= 1)

    stats = client.get_stats()
    assert stats.frames_ok == 1
    assert stats.last_error_byte == 0x07
    assert client.get_latest() is None


# ---------------------------------------------------------------------------
# Bad-frame rejection
# ---------------------------------------------------------------------------

def test_bad_lrc_does_not_update_latest(encoder_link_and_client):
    link, client = encoder_link_and_client
    good_raw = np.full(AUTO_ENC_NUM_JOINTS, 999, dtype=np.uint16)
    feed_encoder_frame(link, bad_lrc=True)
    feed_encoder_frame(link, raw_counts=good_raw)

    client.start_stream(timeout=0.5)
    reading = client.get_latest()

    assert reading is not None
    np.testing.assert_array_equal(reading.raw_counts, good_raw)
    # Bad LRC is filtered by the link, not the client.
    link_stats = link.get_link_stats()
    assert link_stats.frames_bad_lrc[PROTOCOL_BYTE_AUTO_ENC] == 1


def test_wrong_effective_length_bumps_frames_bad_effective_length(encoder_link_and_client):
    link, client = encoder_link_and_client
    # One joint short: self-consistent on the wire (the link accepts it), but
    # the client's exact-size parser rejects it.
    feed_encoder_frame(link, raw_counts=np.zeros(AUTO_ENC_NUM_JOINTS - 1, dtype=np.uint16))
    feed_encoder_frame(link)

    client.start_stream(timeout=0.5)

    stats = client.get_stats()
    assert stats.frames_bad_effective_length == 1
    assert stats.frames_ok == 1
    assert client.get_latest() is not None


# ---------------------------------------------------------------------------
# Stop / restart
# ---------------------------------------------------------------------------

def test_stop_clears_latest(encoder_link_and_client):
    link, client = encoder_link_and_client
    feed_encoder_frame(link)
    client.start_stream(timeout=0.5)
    assert client.get_latest() is not None

    client.stop_stream()
    assert client.get_latest() is None


def test_restart_after_stop_publishes_new_frames(encoder_link_and_client):
    link, client = encoder_link_and_client
    feed_encoder_frame(link, raw_counts=np.full(AUTO_ENC_NUM_JOINTS, 100, dtype=np.uint16))
    client.start_stream(timeout=0.5)
    client.stop_stream()

    # Frames fed while stopped must not leak into the next session.
    feed_encoder_frame(link, raw_counts=np.full(AUTO_ENC_NUM_JOINTS, 200, dtype=np.uint16))
    wait_until(lambda: client.get_stats().frames_ok >= 2)
    assert client.get_latest() is None

    # start_stream blocks until a frame arrives with publishing on,
    # so the test feeds from the main thread once start has reached its wait.
    def _start_in_thread():
        client.start_stream(timeout=1.0)

    t = threading.Thread(target=_start_in_thread, daemon=True)
    t.start()
    time.sleep(0.05)
    feed_encoder_frame(link, raw_counts=np.full(AUTO_ENC_NUM_JOINTS, 300, dtype=np.uint16))
    t.join(timeout=1.0)
    assert not t.is_alive()

    reading = client.get_latest()
    assert reading is not None
    assert reading.raw_counts[0] == 300


def test_failed_start_clears_publish_flag(encoder_link_and_client):
    """A timed-out start must not leave the client in a half-publishing state."""
    link, client = encoder_link_and_client
    with pytest.raises(EncodersNotAvailableError):
        client.start_stream(timeout=0.05)

    # If publish were still active, this frame would set ``latest``.
    feed_encoder_frame(link)
    wait_until(lambda: client.get_stats().frames_ok >= 1)
    assert client.get_latest() is None


def test_frame_in_timeout_race_window_is_not_published(
    encoder_link_and_client, monkeypatch
):
    """A frame landing between the first-frame wait timing out and the
    cleanup must not stay visible via get_latest() after the failed start."""
    link, client = encoder_link_and_client

    def racy_wait(timeout):
        feed_encoder_frame(link)
        wait_until(lambda: client.get_stats().frames_ok >= 1)
        return False  # the wait had already timed out when the frame landed

    monkeypatch.setattr(client._first_frame_event, "wait", racy_wait)
    with pytest.raises(EncodersNotAvailableError):
        client.start_stream(timeout=0.05)

    assert client.get_latest() is None


# ---------------------------------------------------------------------------
# Smoothing
# ---------------------------------------------------------------------------

ANGLE = AUTO_ENC_ANGLE_MASK


def _feed_and_wait(link, client, value):
    """Feed one parity-clean frame with every slot at ``value``."""
    before = client.get_stats().frames_ok
    feed_encoder_frame(link, raw_counts=full_counts(value))
    wait_until(lambda: client.get_stats().frames_ok > before)


def _start_at(link, client, value):
    feed_encoder_frame(link, raw_counts=full_counts(value))
    client.start_stream(timeout=0.5)


def test_get_latest_smooths_while_unfiltered_stays_on_the_wire_value(
    encoder_link_and_client,
):
    link, client = encoder_link_and_client
    _start_at(link, client, 1000)

    _feed_and_wait(link, client, 5000)

    assert client.get_latest_unfiltered().raw_counts[0] & ANGLE == 5000
    assert 1000 < client.get_latest().raw_counts[0] & ANGLE < 5000


def test_repeated_frames_converge_on_the_wire_value(encoder_link_and_client):
    link, client = encoder_link_and_client
    _start_at(link, client, 1000)

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        _feed_and_wait(link, client, 5000)
        if client.get_latest().raw_counts[0] & ANGLE == 5000:
            break

    assert client.get_latest().raw_counts[0] & ANGLE == 5000


def test_parity_failure_holds_the_slot_instead_of_ingesting_it(
    encoder_link_and_client,
):
    link, client = encoder_link_and_client
    _start_at(link, client, 1000)

    corrupt = full_counts(5000) ^ np.uint16(AUTO_ENC_PARITY_BIT)
    for _ in range(20):
        before = client.get_stats().frames_ok
        feed_encoder_frame(link, raw_counts=corrupt)
        wait_until(lambda: client.get_stats().frames_ok > before)

    assert client.get_latest().raw_counts[0] & ANGLE == 1000


def test_smoothing_preserves_the_frames_flags(encoder_link_and_client):
    link, client = encoder_link_and_client
    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    raw[3] = 0x8000  # popcount 1 → parity BAD
    raw[7] = 0x4002  # popcount 2 → parity OK, angle-error set
    feed_encoder_frame(link, raw_counts=raw)
    client.start_stream(timeout=0.5)

    reading = client.get_latest()

    assert bool(reading.parity_ok[3]) is False
    assert bool(reading.angle_error[7]) is True
    assert reading.raw_counts[7] & AUTO_ENC_ANGLE_ERROR_BIT


def test_disabled_filter_publishes_the_raw_frame(encoder_link_and_client_unfiltered):
    link, client = encoder_link_and_client_unfiltered
    _start_at(link, client, 1000)

    _feed_and_wait(link, client, 5000)

    assert client.filter_cutoff_hz is None
    assert client.get_latest().raw_counts[0] & ANGLE == 5000


def test_restart_reseeds_smoothing_at_the_new_pose(encoder_link_and_client):
    """The hand may have been moved while stopped; the first frame of the
    next session must not be dragged toward the old pose."""
    link, client = encoder_link_and_client
    _start_at(link, client, 1000)
    client.stop_stream()

    def _start_in_thread():
        client.start_stream(timeout=1.0)

    t = threading.Thread(target=_start_in_thread, daemon=True)
    t.start()
    time.sleep(0.05)
    feed_encoder_frame(link, raw_counts=full_counts(9000))
    t.join(timeout=1.0)

    assert client.get_latest().raw_counts[0] & ANGLE == 9000


def test_cutoff_can_be_changed_and_disabled_while_streaming(encoder_link_and_client):
    link, client = encoder_link_and_client
    _start_at(link, client, 1000)

    client.filter_cutoff_hz = 50.0
    assert client.filter_cutoff_hz == 50.0

    client.filter_cutoff_hz = None
    _feed_and_wait(link, client, 5000)

    assert client.filter_cutoff_hz is None
    assert client.get_latest().raw_counts[0] & ANGLE == 5000
