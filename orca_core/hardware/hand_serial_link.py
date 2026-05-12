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
    LINK_DEMUX_READ_TIMEOUT_S,
    LINK_HANDLER_ERROR_LOG_INTERVAL_S,
    LINK_RESPONSE_QUEUE_MAXSIZE,
    MAX_AUTO_FRAME_EFF_LEN,
    PROTOCOL_BYTE_RESPONSE,
    PROTOCOL_HEADER_RESPONSE,
    RESPONSE_META_SIZE,
)
from orca_core.hardware.sensing.tactile_protocol import calculate_checksum

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


class HandSerialLink:
    """Serial-port owner and demultiplexer for AA-XX framed traffic.

    Frame layout: ``AA`` header, ``XX`` type byte, payload, LRC checksum
    (longitudinal redundancy check) on the last byte.

    Lifecycle: ``connect()`` opens the port and starts the demuxer thread;
    ``disconnect()`` stops it and closes the port. Handlers may be
    registered at any time before disconnect; after disconnect,
    registration raises ``RuntimeError``.
    """

    def __init__(self, port: str, baudrate: int = LINK_DEFAULT_BAUDRATE):
        self._port = port
        self._baudrate = baudrate

        self._serial: serial.Serial | None = None
        self._connected = False
        self._disconnected = False  # latches True once disconnect() runs
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
        self._reads_paused = False

    # ----- Lifecycle --------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
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
            self._demux_thread.join(timeout=1.0)
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
        response_timeout_s: float = 0.5,
    ) -> bytes:
        """Write a register request and return the matching AA 55 response.

        The drain → write → wait is serialised under a single lock so a
        late response from a previously-timed-out caller can't be handed
        to the next caller.
        """
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

    # ----- Read pause / resume ---------------------------------------------

    def pause_reads(self) -> None:
        """Stop demuxer polling so a sibling USB-CDC endpoint (e.g. the
        Dynamixel bus) isn't starved during ack-blocking writes. The 50 ms
        sleep lets the demuxer ack the flag."""
        if self._reads_paused:
            return
        self._reads_paused = True
        time.sleep(0.05)

    def resume_reads(self) -> None:
        """Discard stale buffered bytes, then re-enable demuxer reads.

        The kernel CDC ring buffer can hold several hundred ms of frames
        while paused; flushing before resuming prevents downstream
        consumers from churning through stale frames.
        """
        if not self._reads_paused:
            return
        self._flush_input_buffer()
        self._reads_paused = False

    def _flush_input_buffer(self) -> None:
        if self._serial is not None:
            try:
                self._serial.reset_input_buffer()
            except (serial.SerialException, OSError):
                pass

    @contextlib.contextmanager
    def paused_reads(self):
        self.pause_reads()
        try:
            yield
        finally:
            self.resume_reads()

    # ----- Stats ------------------------------------------------------------

    def get_link_stats(self) -> LinkStats:
        return self._stats

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
        byte when bytes are available."""
        if self._serial is None:
            return b""
        try:
            return self._serial.read(n)
        except (serial.SerialException, OSError):
            return b""

    # ----- Demuxer thread ---------------------------------------------------

    def _demux_loop(self) -> None:
        try:
            while self._demux_running:
                if self._reads_paused:
                    time.sleep(0.01)
                    continue
                # Resync: slide one byte at a time until we land on 0xAA.
                first = self._serial_read(1)
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
        asked to stop mid-read (e.g. during shutdown)."""
        out = bytearray()
        while len(out) < n:
            if not self._demux_running:
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
        if count > MAX_AUTO_FRAME_EFF_LEN:
            # Implausible payload size — treat as garbage rather than read it.
            self._stats.frames_bad_lrc[PROTOCOL_BYTE_RESPONSE] += 1
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
        eff_len = int.from_bytes(meta[1:3], "little")
        if eff_len == 0 or eff_len > MAX_AUTO_FRAME_EFF_LEN:
            # eff_len is well outside any legal frame — treat as a bad header
            # alignment and resync rather than read megabytes of garbage.
            self._stats.bad_header_resyncs += 1
            return
        body = self._read_exact(eff_len + 1)  # payload + LRC
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
