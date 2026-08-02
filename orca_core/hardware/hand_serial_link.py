# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Serial-link transport with a background frame demultiplexer.

Carries two kinds of frames on one port: synchronous responses to host
requests, and asynchronous broadcasts the device emits on its own clock.
Both are AA-XX framed; the second byte tags the type so a single demuxer
thread can route responses to ``send_register_request`` and broadcasts to
registered handlers. Frame shape and checksum are validated; payload
meaning is left to the handler.
"""

import contextlib
import logging
import queue
import threading
import time
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, field

import serial

from orca_core.hardware.sensing.constants import (
    AUTO_FRAME_META_SIZE,
    LINK_DEFAULT_BAUDRATE,
    LINK_DEFAULT_RESPONSE_TIMEOUT_S,
    LINK_DEMUX_JOIN_TIMEOUT_S,
    LINK_DEMUX_READ_TIMEOUT_S,
    LINK_HANDLER_ERROR_LOG_INTERVAL_S,
    LINK_RESPONSE_QUEUE_MAXSIZE,
    MAX_AUTO_FRAME_EFFECTIVE_LENGTH,
    MAX_RESPONSE_DATA_LEN,
    PROTOCOL_BYTE_RESPONSE,
    PROTOCOL_HEADER_RESPONSE,
    RESPONSE_META_SIZE,
)
from orca_core.hardware.sensing.framing import calculate_checksum

logger = logging.getLogger(__name__)
FrameHandler = Callable[[bytes], None]
"""Handler signature: receives the full frame including header + LRC."""


@dataclass
class LinkStats:
    """Diagnostic counters for the demuxer.

    Frame counters are indexed by the second byte (XX) of AA-XX frames:
    - 'AA' is the fixed header byte (0xAA)
    - 'XX' identifies frame types and is used as the counter key
    """
    frames_routed: Counter = field(default_factory=Counter)
    frames_dropped_no_handler: Counter = field(default_factory=Counter)
    frames_bad_lrc: Counter = field(default_factory=Counter)
    handler_errors: Counter = field(default_factory=Counter)
    bad_header_resyncs: int = 0
    response_queue_dropped: int = 0
    responses_received: int = 0
    responses_implausible_length: int = 0

    def snapshot(self) -> "LinkStats":
        """Copy with independent Counters, so a reader sees a stable view while
        the demuxer keeps mutating the live stats."""
        return LinkStats(
            frames_routed=self.frames_routed.copy(),
            frames_dropped_no_handler=self.frames_dropped_no_handler.copy(),
            frames_bad_lrc=self.frames_bad_lrc.copy(),
            handler_errors=self.handler_errors.copy(),
            bad_header_resyncs=self.bad_header_resyncs,
            response_queue_dropped=self.response_queue_dropped,
            responses_received=self.responses_received,
            responses_implausible_length=self.responses_implausible_length,
        )


class HandSerialLink:
    """Serial-port owner and demultiplexer for AA-XX framed traffic.

    Frame layout: ``AA`` header, ``XX`` type byte, payload, LRC checksum
    on the last byte.

    Lifecycle: ``connect()`` opens the port and starts the demuxer thread;
    ``disconnect()`` stops it and closes the port. Handlers may be
    registered at any time before disconnect; after disconnect,
    registration raises ``RuntimeError``.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = LINK_DEFAULT_BAUDRATE,
        exclusive: bool = True,
    ):
        self._port = port
        self._baudrate = baudrate
        # Exclusive by default: two links reading one serial device steal bytes
        # from each other and corrupt every frame either of them sees.
        self._exclusive = exclusive

        self._serial: serial.Serial | None = None
        self._connected = False
        self._disconnected = False  # latches True once disconnect() runs
        self._port_dead = False  # latches True on a hard port failure (e.g. USB unplug)
        self._port_error: str | None = None
        self._demux_running = False
        self._demux_thread: threading.Thread | None = None

        self._handlers: dict[int, FrameHandler] = {}
        self._handlers_lock = threading.Lock()

        self._response_queue: queue.Queue[bytes | None] = queue.Queue(
            maxsize=LINK_RESPONSE_QUEUE_MAXSIZE
        )
        self._serial_write_lock = threading.Lock()

        self._stats = LinkStats()
        self._last_handler_error_log: dict[int, float] = {}

    # ----- Lifecycle --------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        """``True`` while the link is open and its port is healthy."""
        return self._connected and not self._port_dead

    @property
    def is_port_dead(self) -> bool:
        """``True`` once the serial port has failed hard (e.g. USB unplug).

        A dead link cannot recover: register requests fail fast and the
        demuxer has exited. Call ``disconnect()`` and build a new link to
        reconnect.
        """
        return self._port_dead

    @property
    def port_error(self) -> str | None:
        """Description of the port failure, or ``None`` while the port is healthy."""
        return self._port_error

    def connect(self) -> None:
        if self._port_dead:
            raise RuntimeError(
                f"hand serial link port failed ({self._port_error}); "
                "the link cannot recover — build a new link to reconnect"
            )
        if self._connected:
            return
        if self._disconnected:
            raise RuntimeError("hand serial link already disconnected; cannot reconnect")

        self._open_serial()
        self._demux_running = True
        self._connected = True
        self._demux_thread = threading.Thread(
            target=self._demux_loop,
            name=f"HandSerialLink({self._port})",
            daemon=True,
        )
        self._demux_thread.start()

    def disconnect(self) -> None:
        if not self._connected and not self._demux_running:
            self._disconnected = True
            return

        self._demux_running = False
        if self._demux_thread is not None:
            self._demux_thread.join(timeout=LINK_DEMUX_JOIN_TIMEOUT_S)
            self._demux_thread = None

        # Belt-and-braces: ensure a sentinel is on the queue so any blocked
        # caller wakes even if the demuxer exited before pushing one.
        try:
            self._response_queue.put_nowait(None)
        except queue.Full:
            pass

        self._close_serial()
        self._connected = False
        self._disconnected = True

    # ----- Handler registry -------------------------------------------------

    def register_frame_handler(self, second_byte: int, handler: FrameHandler) -> None:
        """Route AA-XX frames where XX == ``second_byte`` to ``handler``."""
        if self._disconnected:
            raise RuntimeError("hand serial link disconnected")
        if not 0 <= second_byte <= 0xFF:
            raise ValueError(f"second_byte must be a single byte, got {second_byte}")
        if second_byte == PROTOCOL_BYTE_RESPONSE:
            raise ValueError(
                f"0x{PROTOCOL_BYTE_RESPONSE:02X} is reserved for register responses"
            )
        with self._handlers_lock:
            self._handlers[second_byte] = handler

    def unregister_frame_handler(self, second_byte: int) -> None:
        with self._handlers_lock:
            self._handlers.pop(second_byte, None)

    # ----- Register transactions (AA 55 round-trip) -------------------------

    def send_register_request(
        self,
        request_bytes: bytes,
        response_timeout_s: float = LINK_DEFAULT_RESPONSE_TIMEOUT_S,
    ) -> bytes:
        """Write a register request and return the matching AA 55 response.

        The drain → write → wait is serialised under a single lock so a
        late response from a previously-timed-out caller can't be handed
        to the next caller.
        """
        if self._port_dead:
            raise IOError(f"hand serial link port failed: {self._port_error}")
        if not self._demux_running:
            raise RuntimeError(
                "hand serial link not running (not connected or demuxer crashed)"
            )

        with self._serial_write_lock:
            self._drain_response_queue()
            self._serial_write(request_bytes)
            try:
                response = self._response_queue.get(timeout=response_timeout_s)
            except queue.Empty:
                raise IOError(
                    f"Timed out waiting for register response after {response_timeout_s}s"
                )
            if response is None:
                # Re-post sentinel so any other blocked caller also wakes.
                try:
                    self._response_queue.put_nowait(None)
                except queue.Full:
                    pass
                if self._port_dead:
                    raise IOError(
                        f"hand serial link port failed: {self._port_error}"
                    )
                raise IOError("hand serial link closed")
            return response

    def _drain_response_queue(self) -> None:
        while True:
            try:
                stale = self._response_queue.get_nowait()
            except queue.Empty:
                return
            if stale is None:
                # Re-post the sentinel so the wait-loop below sees link closure.
                try:
                    self._response_queue.put_nowait(None)
                except queue.Full:
                    pass
                raise IOError("hand serial link closed")

    # ----- Stats ------------------------------------------------------------

    def get_link_stats(self) -> LinkStats:
        """Return a snapshot copy of the demuxer counters."""
        return self._stats.snapshot()

    # ----- I/O seam (overridden by MockHandSerialLink) ----------------------

    def _open_serial(self) -> None:
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=LINK_DEMUX_READ_TIMEOUT_S,
                exclusive=self._exclusive,
            )
        except (serial.SerialException, OSError) as e:
            raise ConnectionError(f"Failed to open serial port {self._port}: {e}") from e

    def _close_serial(self) -> None:
        if self._serial is not None and self._serial.is_open:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
        self._serial = None

    def _serial_write(self, data: bytes) -> None:
        if self._serial is None:
            raise IOError("serial port not open")
        self._serial.write(data)

    def _serial_read(self, n: int) -> bytes:
        """Read up to ``n`` bytes. Returns ``b""`` on read timeout. Returns at least one
        byte when bytes are available. A hard port failure (the read raises rather
        than timing out, e.g. after USB unplug) latches the port-dead state."""
        if self._serial is None:
            return b""
        try:
            return self._serial.read(n)
        except (serial.SerialException, OSError) as e:
            self._mark_port_dead(e)
            return b""

    def _mark_port_dead(self, error: Exception) -> None:
        """Latch the port-dead state so the demuxer exits and callers fail fast."""
        if self._port_dead:
            return
        self._port_error = str(error) or type(error).__name__
        self._port_dead = True
        logger.error(
            f"Serial port {self._port} failed; hand serial link is dead: "
            f"{self._port_error}"
        )

    # ----- Demuxer thread ---------------------------------------------------

    def _demux_loop(self) -> None:
        try:
            while self._demux_running:
                # Resync: slide one byte at a time until we land on 0xAA.
                first = self._serial_read(1)
                if self._port_dead:
                    break
                if not first:
                    continue
                if first[0] != 0xAA:
                    self._stats.bad_header_resyncs += 1
                    continue

                second = self._read_exact(1)
                if second is None:
                    break
                second_byte = second[0]

                if second_byte == PROTOCOL_BYTE_RESPONSE:
                    self._handle_response_frame()
                else:
                    self._handle_auto_frame(second_byte)
        except Exception:
            logger.exception("Demuxer thread crashed; exiting")
        finally:
            try:
                self._response_queue.put_nowait(None)
            except queue.Full:
                pass

    def _read_exact(self, n: int) -> bytes | None:
        """Read exactly ``n`` bytes. Returns ``None`` if the demuxer is
        asked to stop mid-read (shutdown or port death)."""
        out = bytearray()
        while len(out) < n:
            if not self._demux_running or self._port_dead:
                return None
            chunk = self._serial_read(n - len(out))
            if not chunk:
                continue
            out.extend(chunk)
        return bytes(out)

    def _handle_response_frame(self) -> None:
        meta = self._read_exact(RESPONSE_META_SIZE)
        if meta is None:
            return
        count = int.from_bytes(meta[4:6], "little")
        if count > MAX_RESPONSE_DATA_LEN:
            # Implausible payload size — treat as garbage rather than read it.
            self._stats.responses_implausible_length += 1
            return
        body = self._read_exact(count + 1)  # data + LRC
        if body is None:
            return
        full_frame = PROTOCOL_HEADER_RESPONSE + meta + body
        if calculate_checksum(full_frame[:-1]) != full_frame[-1]:
            self._stats.frames_bad_lrc[PROTOCOL_BYTE_RESPONSE] += 1
            return
        self._enqueue_response(full_frame)

    def _handle_auto_frame(self, second_byte: int) -> None:
        meta = self._read_exact(AUTO_FRAME_META_SIZE)
        if meta is None:
            return
        effective_length = int.from_bytes(meta[1:3], "little")
        if effective_length == 0 or effective_length > MAX_AUTO_FRAME_EFFECTIVE_LENGTH:
            # effective_length is well outside any legal frame — treat as a bad header
            # alignment and resync rather than read megabytes of garbage.
            self._stats.bad_header_resyncs += 1
            return
        body = self._read_exact(effective_length + 1)  # payload + LRC
        if body is None:
            return
        full_frame = bytes([0xAA, second_byte]) + meta + body
        if calculate_checksum(full_frame[:-1]) != full_frame[-1]:
            self._stats.frames_bad_lrc[second_byte] += 1
            return

        with self._handlers_lock:
            handler = self._handlers.get(second_byte)
        if handler is None:
            self._stats.frames_dropped_no_handler[second_byte] += 1
            return

        try:
            handler(full_frame)
        except Exception:
            self._stats.handler_errors[second_byte] += 1
            self._log_handler_error(second_byte)
            return
        self._stats.frames_routed[second_byte] += 1

    def _enqueue_response(self, frame: bytes) -> None:
        """Push ``frame`` onto the response queue, dropping the oldest entry
        on overflow so the newest response is always reachable."""
        try:
            self._response_queue.put_nowait(frame)
            self._stats.responses_received += 1
            return
        except queue.Full:
            pass
        # Drop oldest, then push.
        try:
            self._response_queue.get_nowait()
            self._stats.response_queue_dropped += 1
        except queue.Empty:
            pass
        try:
            self._response_queue.put_nowait(frame)
            self._stats.responses_received += 1
        except queue.Full:
            pass

    def _log_handler_error(self, second_byte: int) -> None:
        """Rate-limited per second-byte: at most one stack trace per
        ``LINK_HANDLER_ERROR_LOG_INTERVAL_S`` so a sick handler can't
        drown the log."""
        now = time.monotonic()
        last = self._last_handler_error_log.get(second_byte, 0.0)
        if now - last < LINK_HANDLER_ERROR_LOG_INTERVAL_S:
            return
        self._last_handler_error_log[second_byte] = now
        logger.exception(
            f"Handler for AA {second_byte:02X} raised; demuxer continues"
        )
