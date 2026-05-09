# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Joint-encoder client that subscribes to AA A9 frames on a ``HandSerialLink``.

The encoder stream is always-on hardware-side, so this client never writes a
device register to enable it. ``connect()`` registers the handler and the
demuxer thread starts driving it immediately; ``start_encoder_stream()`` only
gates whether parsed frames are exposed via ``get_latest_encoder_reading()``.
This way diagnostic counters reflect link health even before any consumer
calls ``start_encoder_stream()``.
"""
from __future__ import annotations

import contextlib
import dataclasses
import logging
import math
import threading
import time
from dataclasses import dataclass

from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.sensing.constants import PROTOCOL_BYTE_AUTO_ENC
from orca_core.hardware.sensing.encoder_protocol import parse_auto_enc_frame
from orca_core.hardware.sensing.types import EncoderReading

logger = logging.getLogger(__name__)


_DEFAULT_FIRST_FRAME_TIMEOUT_S = 0.1


class EncodersNotAvailableError(Exception):
    pass


@dataclass
class EncoderStreamStats:
    """Diagnostic counters for the AA A9 handler.
    """
    frames_ok: int = 0
    frames_bad_eff_len: int = 0
    last_err_byte: int = 0
    last_frame_timestamp: float | None = None

    @property
    def last_freshness_ms(self) -> float:
        if self.last_frame_timestamp is None:
            return math.inf
        return (time.monotonic() - self.last_frame_timestamp) * 1000.0


class JointEncoderClient:
    """ORCA joint-encoder client over a :class:`HandSerialLink`.

    Subscribes to AA A9 auto-stream frames and exposes the latest reading.
    The handler always parses incoming frames so stats track the stream
    even before ``start_encoder_stream()``; the publish flag only gates
    public visibility of the latest reading.
    """

    def __init__(self, link: HandSerialLink):
        self._link = link
        self._connected = False

        self._lock = threading.Lock()
        self._publish_active = False
        self._latest: EncoderReading | None = None
        self._stats = EncoderStreamStats()
        self._first_frame_event = threading.Event()

    # ----- Lifecycle --------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            return
        self._link.register_frame_handler(PROTOCOL_BYTE_AUTO_ENC, self._on_encoder_frame)
        self._connected = True

    def disconnect(self) -> None:
        if not self._connected:
            return
        try:
            self.stop_encoder_stream()
        except Exception:
            logger.exception("Error stopping encoder stream during disconnect")
        try:
            self._link.unregister_frame_handler(PROTOCOL_BYTE_AUTO_ENC)
        except Exception:
            logger.exception("Error unregistering encoder handler during disconnect")
        self._connected = False

    def __enter__(self):
        if not self._connected:
            self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    # ----- Stream control ---------------------------------------------------

    def start_encoder_stream(self, timeout: float = _DEFAULT_FIRST_FRAME_TIMEOUT_S) -> None:
        """Begin publishing the latest reading; wait for the first valid frame.

        Raises :class:`EncodersNotAvailableError` if no valid frame arrives
        within ``timeout`` seconds. Does not write any device register —
        the AA A9 stream is always-on.
        """
        if not self._connected:
            raise OSError("Must call connect() first.")

        with self._lock:
            self._first_frame_event.clear()
            self._latest = None
            self._publish_active = True

        if not self._first_frame_event.wait(timeout):
            with self._lock:
                self._publish_active = False
            raise EncodersNotAvailableError(
                f"No encoder frame within {timeout}s"
            )

    def stop_encoder_stream(self) -> None:
        with self._lock:
            self._publish_active = False
            self._latest = None
            self._first_frame_event.clear()

    # ----- Link read pause / resume passthroughs ----------------------------

    def pause_link_reads(self) -> None:
        """Pause demuxer reads on the underlying link. Use around host
        operations whose USB CDC traffic is starved by sustained sibling-CDC
        reads (e.g. per-motor Dynamixel ``write_byte`` ack waits).
        """
        self._link.pause_reads()

    def resume_link_reads(self) -> None:
        """Flush stale link bytes and resume demuxer reads."""
        self._link.resume_reads()

    @contextlib.contextmanager
    def paused_link_reads(self):
        self.pause_link_reads()
        try:
            yield
        finally:
            self.resume_link_reads()

    # ----- Reads ------------------------------------------------------------

    def get_latest_encoder_reading(self) -> EncoderReading | None:
        with self._lock:
            return self._latest

    def get_encoder_stats(self) -> EncoderStreamStats:
        with self._lock:
            return dataclasses.replace(self._stats)

    # ----- Frame handler ----------------------------------------------------

    def _on_encoder_frame(self, frame_bytes: bytes) -> None:
        """Parse one AA A9 frame and (when publishing) update the latest cache.

        The link has already validated header alignment, ``eff_len`` bounds,
        and full-frame LRC, so the only payload-semantic check left is the
        exact frame size — handled by :func:`parse_auto_enc_frame`. Any
        ``IOError`` from the parser is therefore an eff_len-mismatch on the
        production path; non-IOError exceptions fall through to the link's
        handler-exception isolation.
        """
        timestamp = time.monotonic()
        try:
            reading = parse_auto_enc_frame(frame_bytes, timestamp=timestamp)
        except IOError:
            with self._lock:
                self._stats.frames_bad_eff_len += 1
            return

        with self._lock:
            self._stats.frames_ok += 1
            self._stats.last_err_byte = reading.err_byte
            self._stats.last_frame_timestamp = timestamp
            if not self._publish_active:
                return
            self._latest = reading
            self._first_frame_event.set()
