# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Client for reading joint encoder data from AA A9 frames.

The AA A9 stream is always-on hardware-side, so this client never writes
a device register to enable it. ``connect()`` registers the frame handler
(stats start updating immediately, useful for link-health checks);
``start_stream()`` only gates whether parsed readings become
visible via ``get_latest()``.

All public methods are thread-safe.
"""
import dataclasses
import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.sensing.constants import (
    ENCODER_COUNTS_PER_REV,
    ENCODER_FIRST_FRAME_TIMEOUT_S,
    PROTOCOL_BYTE_AUTO_ENC,
)
from orca_core.hardware.sensing.encoder_protocol import parse_encoder_frame
from orca_core.hardware.sensing.types import EncoderReading

logger = logging.getLogger(__name__)


class JointFeedbackConnectError(RuntimeError):
    """Raised when a joint-feedback connect precondition fails (unsupported
    hand side, no encoder port resolved, no encoder-backed joints, missing
    encoder calibration, or no encoder frames arriving).
    """


class EncodersNotAvailableError(JointFeedbackConnectError):
    """Raised when no valid encoder frame arrives within the start-stream timeout."""


class JointEncoderCalibrationError(RuntimeError):
    """Raised when the joint-encoder calibration sweep cannot complete a step."""


class _ReadsLatestEncoderFrame(Protocol):
    def get_latest(self) -> EncoderReading | None: ...


@dataclass
class EncoderStreamStats:
    """Diagnostic counters for the AA A9 handler."""
    frames_ok: int = 0
    frames_bad_effective_length: int = 0
    last_error_byte: int = 0
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
    even before ``start_stream()``; the publish flag only gates
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
        self.stop_stream()
        self._link.unregister_frame_handler(PROTOCOL_BYTE_AUTO_ENC)
        self._connected = False

    def __enter__(self):
        if not self._connected:
            self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    # ----- Stream control ---------------------------------------------------

    def start_stream(self, timeout: float = ENCODER_FIRST_FRAME_TIMEOUT_S) -> None:
        """Begin publishing encoder readings and wait for the first valid frame.

        Does not write any device register — the AA A9 stream is always-on.
        Raises :class:`EncodersNotAvailableError` if no valid frame arrives
        within ``timeout`` seconds.
        """
        if not self._connected:
            raise OSError("Must call connect() first.")

        with self._lock:
            self._first_frame_event.clear()
            self._latest = None
            self._publish_active = True

        if not self._first_frame_event.wait(timeout):
            # Mirror stop_stream: a frame landing between the wait timing out
            # and this cleanup must not stay visible after a failed start.
            with self._lock:
                self._publish_active = False
                self._latest = None
                self._first_frame_event.clear()
            raise EncodersNotAvailableError(
                f"No encoder frame within {timeout}s"
            )

    def stop_stream(self) -> None:
        """Hide further readings and drop the cached one so a later
        ``get_latest()`` returns ``None`` until the next
        ``start_stream()``."""
        with self._lock:
            self._publish_active = False
            self._latest = None
            self._first_frame_event.clear()

    # ----- Reads ------------------------------------------------------------

    def get_latest(self) -> EncoderReading | None:
        """Returns ``None`` before the first frame is published or while
        the stream is stopped."""
        with self._lock:
            return self._latest

    def get_stats(self) -> EncoderStreamStats:
        """Snapshot of the diagnostic counters — safe to call without
        ``start_stream``."""
        with self._lock:
            return dataclasses.replace(self._stats)

    # ----- Frame handler ----------------------------------------------------

    def _on_encoder_frame(self, frame_bytes: bytes) -> None:
        """Parse one AA A9 frame and update the latest reading cache.

        Ignores malformed frames and updates diagnostic statistics.
        """
        timestamp = time.monotonic()
        try:
            reading = parse_encoder_frame(frame_bytes, timestamp=timestamp)
        except IOError:
            with self._lock:
                self._stats.frames_bad_effective_length += 1
            return

        with self._lock:
            self._stats.frames_ok += 1
            self._stats.last_error_byte = reading.error_byte
            self._stats.last_frame_timestamp = timestamp
            if not self._publish_active:
                return
            self._latest = reading
            self._first_frame_event.set()


# ----- Anchor sampling for the joint-encoder calibration sweep --------------


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

    Chip-flagged samples (parity failure or angle-error bit) are rejected and
    do not count toward ``num_samples``; if the deadline expires — including
    when flagged samples starve the collection — a
    :class:`JointEncoderCalibrationError` is raised rather than folding
    corrupted counts into the anchor.
    """
    if num_samples <= 0:
        raise ValueError("num_samples must be positive")

    counts = np.empty(num_samples, dtype=np.uint16)
    last_ts: float | None = None
    deadline = time.monotonic() + timeout_s
    collected = 0
    rejected = 0
    while collected < num_samples:
        reading = client.get_latest()
        if reading is not None and reading.timestamp != last_ts:
            last_ts = reading.timestamp
            if bool(reading.parity_ok[slot]) and not bool(reading.angle_error[slot]):
                counts[collected] = int(reading.raw_counts[slot]) & 0x3FFF
                collected += 1
                continue
            rejected += 1
        if time.monotonic() > deadline:
            raise JointEncoderCalibrationError(
                f"timed out waiting for encoder samples on slot {slot} "
                f"(got {collected}/{num_samples} in {timeout_s}s, "
                f"{rejected} chip-flagged samples rejected)"
            )
        time.sleep(sample_period_s)
    return average_anchor_count(counts)
