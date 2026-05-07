"""Tests for HandSerialLink + MockHandSerialLink.

Threading model: the link's demuxer runs on a background thread, so feeding
bytes is asynchronous. Tests synchronise via ``Condition.wait_for``
"""
from __future__ import annotations

import threading

import numpy as np
import pytest

from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    FUNC_CODE_READ,
    PROTOCOL_BYTE_AUTO,
    PROTOCOL_BYTE_AUTO_ENC,
    PROTOCOL_HEADER_AUTO,
    PROTOCOL_HEADER_AUTO_ENC,
    PROTOCOL_HEADER_RESPONSE,
    PROTOCOL_RESERVED,
)
from orca_core.hardware.sensing.encoder_protocol import calculate_checksum
from orca_core.hardware.sensing.tactile_protocol import build_read_request


# ---------------------------------------------------------------------------
# Frame builders + sync helper
# ---------------------------------------------------------------------------

def _encoder_frame(*, bad_lrc: bool = False) -> bytes:
    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    eff_len = 1 + raw.nbytes  # err byte + payload
    body = (
        PROTOCOL_HEADER_AUTO_ENC
        + bytes([PROTOCOL_RESERVED])
        + eff_len.to_bytes(2, "little")
        + b"\x00"  # err byte
        + raw.astype("<u2").tobytes()
    )
    lrc = calculate_checksum(body) ^ (0xFF if bad_lrc else 0)
    return body + bytes([lrc])


def _tactile_frame() -> bytes:
    eff_len = 1  # err byte only, no force payload
    body = (
        PROTOCOL_HEADER_AUTO
        + bytes([PROTOCOL_RESERVED])
        + eff_len.to_bytes(2, "little")
        + b"\x00"
    )
    return body + bytes([calculate_checksum(body)])


def _read_response(address: int, data: bytes) -> bytes:
    body = (
        PROTOCOL_HEADER_RESPONSE
        + bytes([PROTOCOL_RESERVED, FUNC_CODE_READ])
        + address.to_bytes(2, "little")
        + len(data).to_bytes(2, "little")
        + data
    )
    return body + bytes([calculate_checksum(body)])


class _Capture:
    """Frame-recording handler with a Condition-based ``wait_for(count)``.
    """

    def __init__(self) -> None:
        self.frames: list[bytes] = []
        self._cv = threading.Condition()

    def __call__(self, frame: bytes) -> None:
        with self._cv:
            self.frames.append(frame)
            self._cv.notify_all()

    def wait_for(self, count: int, timeout: float = 1.0) -> None:
        with self._cv:
            if not self._cv.wait_for(lambda: len(self.frames) >= count, timeout=timeout):
                raise AssertionError(
                    f"got {len(self.frames)}/{count} captures within {timeout}s"
                )


@pytest.fixture
def link():
    link = MockHandSerialLink()
    link.connect()
    yield link
    link.disconnect()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_dispatch_routes_by_second_byte(link):
    enc, tac = _Capture(), _Capture()
    link.register_frame_handler(PROTOCOL_BYTE_AUTO_ENC, enc)
    link.register_frame_handler(PROTOCOL_BYTE_AUTO, tac)

    encoder, tactile = _encoder_frame(), _tactile_frame()
    link.feed_bytes(encoder + tactile + encoder)

    enc.wait_for(2)
    tac.wait_for(1)
    assert enc.frames == [encoder, encoder]
    assert tac.frames == [tactile]


def test_frame_with_no_handler_is_dropped(link):
    # The unhandled encoder frame is followed by a handled tactile marker;
    # by the time the marker fires, the encoder frame has been counted as dropped.
    marker = _Capture()
    link.register_frame_handler(PROTOCOL_BYTE_AUTO, marker)

    link.feed_bytes(_encoder_frame() + _tactile_frame())
    marker.wait_for(1)

    stats = link.get_link_stats()
    assert stats.frames_dropped_no_handler[PROTOCOL_BYTE_AUTO_ENC] == 1
    assert stats.frames_routed[PROTOCOL_BYTE_AUTO_ENC] == 0


def test_register_request_round_trip(link):
    expected = _read_response(0x0010, b"\x44\x44\x04\x00")
    link.set_response_provider(lambda _request: expected)

    request = build_read_request(0x0010, 4)
    assert link.send_register_request(request) == expected
    assert link.last_serial_write() == request


def test_drain_before_send_discards_stale_response(link):
    # Marker proves the stale response has been queued before we issue the
    # next request; drain-before-send must throw the stale away.
    marker = _Capture()
    link.register_frame_handler(PROTOCOL_BYTE_AUTO, marker)

    stale = _read_response(0x0010, b"\x01\x02\x03\x04")
    link.feed_bytes(stale + _tactile_frame())
    marker.wait_for(1)

    fresh = _read_response(0x0020, b"\x09\x09\x09\x09")
    link.set_response_provider(lambda _request: fresh)
    assert link.send_register_request(build_read_request(0x0020, 4)) == fresh


def test_demuxer_recovers_from_corruption(link):
    received = _Capture()
    link.register_frame_handler(PROTOCOL_BYTE_AUTO_ENC, received)

    bad_lrc, good = _encoder_frame(bad_lrc=True), _encoder_frame()
    link.feed_bytes(b"\x00\x01\x02" + bad_lrc + good)  # garbage prefix forces resync
    received.wait_for(1)

    assert received.frames == [good]
    stats = link.get_link_stats()
    assert stats.bad_header_resyncs >= 1
    assert stats.frames_bad_lrc[PROTOCOL_BYTE_AUTO_ENC] == 1


def test_disconnect_unblocks_pending_register_request():
    link = MockHandSerialLink()
    link.connect()

    errors: list[BaseException] = []
    done = threading.Event()

    def call() -> None:
        try:
            link.send_register_request(
                build_read_request(0x0010, 4), response_timeout_s=5.0,
            )
        except IOError as e:
            errors.append(e)
        finally:
            done.set()

    threading.Thread(target=call, daemon=True).start()
    # Once the worker has written its request, it's microseconds from blocking
    # on ``_response_queue.get`` — the exact spot the disconnect-sentinel must wake.
    link.wait_for_write()
    link.disconnect()

    assert done.wait(timeout=1.0)
    assert len(errors) == 1 and "closed" in str(errors[0]).lower()


def test_handler_exception_does_not_kill_demuxer(link):
    def boom(_frame: bytes) -> None:
        raise RuntimeError("kaboom")
    link.register_frame_handler(PROTOCOL_BYTE_AUTO_ENC, boom)

    # Demuxer survived if the marker on a different stream fires after boom raises.
    marker = _Capture()
    link.register_frame_handler(PROTOCOL_BYTE_AUTO, marker)
    link.feed_bytes(_encoder_frame() + _tactile_frame())
    marker.wait_for(1)

    assert link.get_link_stats().handler_errors[PROTOCOL_BYTE_AUTO_ENC] == 1


def test_send_before_connect_raises():
    with pytest.raises(RuntimeError):
        MockHandSerialLink().send_register_request(build_read_request(0x0010, 4))


def test_register_handler_after_disconnect_raises():
    link = MockHandSerialLink()
    link.connect()
    link.disconnect()
    with pytest.raises(RuntimeError):
        link.register_frame_handler(PROTOCOL_BYTE_AUTO_ENC, lambda _frame: None)
