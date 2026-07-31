# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""In-memory test seam for ``HandSerialLink``.

Subclasses the real link and overrides only the four serial I/O methods;
the demuxer, dispatcher, and transaction lock are unchanged production
code, exercised through this mock so there is no parallel link
implementation to drift.
"""
import threading
from collections.abc import Callable

from orca_core.hardware.hand_serial_link import HandSerialLink

ResponseProvider = Callable[[bytes], bytes | None]
"""Receives the request bytes; returns response bytes to inject (or ``None``)."""


class MockHandSerialLink(HandSerialLink):
    def __init__(self, port: str = "mock", baudrate: int = 921600):
        super().__init__(port=port, baudrate=baudrate)
        self._injected_buffer = bytearray()
        self._injected_cv = threading.Condition()
        self._mock_serial_open = False
        self._mock_port_error: Exception | None = None
        self._serial_writes: list[bytes] = []
        self._response_provider: ResponseProvider | None = None

    # ----- Test-facing API --------------------------------------------------

    def feed_bytes(self, data: bytes) -> None:
        """Inject raw bytes into the demuxer's read stream."""
        with self._injected_cv:
            self._injected_buffer.extend(data)
            self._injected_cv.notify_all()

    def simulate_port_death(self, error: Exception | None = None) -> None:
        """Make the port fail hard, as a USB unplug would: reads latch the
        link's port-dead state and writes raise."""
        with self._injected_cv:
            self._mock_port_error = error or IOError("mock device disconnected")
            self._injected_cv.notify_all()

    def set_response_provider(self, provider: ResponseProvider | None) -> None:
        """Install a callback invoked from ``_serial_write``. If it returns
        bytes, those bytes are injected into the read stream — letting a test
        round-trip ``send_register_request`` without a real serial peer."""
        self._response_provider = provider

    @property
    def response_provider(self) -> ResponseProvider | None:
        return self._response_provider

    def serial_writes(self) -> list[bytes]:
        """Snapshot of every payload passed to ``_serial_write``."""
        with self._injected_cv:
            return list(self._serial_writes)

    def last_serial_write(self) -> bytes | None:
        with self._injected_cv:
            return self._serial_writes[-1] if self._serial_writes else None

    def wait_for_write(self, count: int = 1, timeout: float = 1.0) -> None:
        """Block until at least ``count`` writes have happened. Lets a test
        synchronise on the worker thread reaching its serial-write step
        without polling — the timeout is a CI-safety upper bound, not the
        intended wait duration."""
        with self._injected_cv:
            if not self._injected_cv.wait_for(
                lambda: len(self._serial_writes) >= count, timeout=timeout
            ):
                raise AssertionError(
                    f"got {len(self._serial_writes)}/{count} writes within {timeout}s"
                )

    # ----- I/O seam overrides ----------------------------------------------

    def _open_serial(self) -> None:
        with self._injected_cv:
            self._mock_serial_open = True
            self._injected_cv.notify_all()

    def _close_serial(self) -> None:
        with self._injected_cv:
            self._mock_serial_open = False
            self._injected_cv.notify_all()

    def _serial_write(self, data: bytes) -> None:
        with self._injected_cv:
            if self._mock_port_error is not None:
                raise IOError(f"mock serial port failed: {self._mock_port_error}")
            self._serial_writes.append(bytes(data))
            self._injected_cv.notify_all()
        provider = self._response_provider
        if provider is not None:
            response = provider(bytes(data))
            if response:
                self.feed_bytes(response)

    def _serial_read(self, n: int) -> bytes:
        """Block briefly for bytes (matching ``serial.Serial.read`` timeout
        semantics); latch the port-dead state on a simulated port death,
        as the real override does when the read raises."""
        with self._injected_cv:
            if self._mock_port_error is None and not self._injected_buffer:
                if not self._mock_serial_open or not self._demux_running:
                    return b""
                self._injected_cv.wait(timeout=0.05)
            error = self._mock_port_error
            if error is None:
                if not self._injected_buffer:
                    return b""
                chunk = bytes(self._injected_buffer[:n])
                del self._injected_buffer[:n]
                return chunk
        self._mark_port_dead(error)
        return b""
