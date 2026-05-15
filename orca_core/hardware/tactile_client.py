# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
import serial
import threading
import time
import logging

from orca_core.hardware.sensing.constants import (
    FINGER_NAMES,
    DEFAULT_SENSOR_PORT,
    DEFAULT_SENSOR_BAUDRATE,
    DEFAULT_FINGER_TO_SENSOR_ID,
    PROTOCOL_HEADER_RESPONSE,
    PROTOCOL_HEADER_AUTO,
    RESPONSE_META_SIZE,
    AUTO_FRAME_META_SIZE,
    ADDR_CONNECTED_SENSORS_START,
    ADDR_CONNECTED_SENSORS_LENGTH,
    ADDR_NUM_TAXELS_START,
    ADDR_NUM_TAXELS_LENGTH,
    ADDR_RESULTANT_FORCE_START,
    RESULTANT_BLOCK_SIZE,
    ADDR_AUTO_DATA_TYPE,
    ADDR_AUTO_ENABLE,
    REGISTER_ENABLE,
    REGISTER_DISABLE,
)
from orca_core.hardware.sensing.protocol import (
    validate_auto_frame_lrc,
    build_read_request,
    build_write_request,
    parse_read_response,
    parse_write_response,
    extract_write_response_data_length,
    read_response_body_size,
    extract_auto_frame_eff_len,
    unpack_auto_payload,
    compute_expected_payload_size,
    compute_distal_module_index,
    decode_resultant_auto,
    decode_taxels_auto,
    decode_combined_auto,
    decode_resultant_register,
    decode_connected_sensors,
    decode_num_taxels,
    decode_auto_data_type,
    encode_auto_data_type,
)

logger = logging.getLogger(__name__)


class NoSensorsAvailableError(Exception):
    pass


class FrameError(Exception):
    """Recoverable frame-level error in auto-stream acquisition.

    Raised by _acquire_frame when a single frame is bad (LRC failure,
    parse error, size mismatch). The auto-reader loop increments error
    counters and continues to the next frame.
    """
    def __init__(self, message: str, bad_lrc: bool = False):
        super().__init__(message)
        self.bad_lrc = bad_lrc


@dataclass
class AutoStreamStats:
    """Diagnostic counters for the auto-stream reader loop."""
    frames_ok: int = 0
    frames_bad_checksum: int = 0
    parse_errors: int = 0
    resyncs: int = 0
    last_error_code: int = 0  # most recent sensor-reported error code (0 = no error)


@dataclass
class TactileSensorConfiguration:
    """Snapshot of connected sensors and their properties.

    Captured at stream start. The Paxini PX-6AX GEN3 firmware enumerates
    sensors at power-on and does not report mid-stream changes, so this
    snapshot is treated as immutable for the duration of a stream.
    """
    connected: dict[str, bool] = field(default_factory=dict)  # {finger: is_connected}
    num_taxels: dict[str, int] = field(default_factory=dict)  # {finger: taxel_count}
    module_indices: dict[str, int] = field(default_factory=dict)  # {finger: module_idx}
    finger_to_sensor_id: dict[str, int] = field(default_factory=lambda: dict(DEFAULT_FINGER_TO_SENSOR_ID))

    @property
    def active_sensors(self) -> list[str]:
        """List of currently connected sensors sorted by hardware slot order.

        Auto-stream data arrives in slot order, so this must match.
        Protocol decoders in protocol.py require this ordering.
        """
        active = [f for f in FINGER_NAMES if self.connected.get(f, False)]
        active.sort(key=lambda f: self.finger_to_sensor_id.get(f, FINGER_NAMES.index(f)))
        return active

    @property
    def num_active_sensors(self) -> int:
        return len(self.active_sensors)

    def __str__(self) -> str:
        active = ", ".join(self.active_sensors) if self.active_sensors else "none"
        return f"SensorConfig({self.num_active_sensors} active: {active})"


class TactileClient:
    """Client for communicating with ORCA Tactile Sensors"""

    def __init__(self,
                 port: str = DEFAULT_SENSOR_PORT,
                 baudrate: int = DEFAULT_SENSOR_BAUDRATE,
                 finger_to_sensor_id: dict[str, int] | None = None):

        self.port = port
        self.baudrate = baudrate
        self._connected = False
        self._serial_connection: serial.Serial | None = None

        # Finger-to-sensor-id mapping (configurable wiring)
        if finger_to_sensor_id is None:
            self._finger_to_sensor_id = dict(DEFAULT_FINGER_TO_SENSOR_ID)
        else:
            expected_fingers = set(FINGER_NAMES)
            if set(finger_to_sensor_id.keys()) != expected_fingers:
                raise ValueError(
                    f"finger_to_sensor_id must contain exactly {FINGER_NAMES}, "
                    f"got {sorted(finger_to_sensor_id.keys())}"
                )
            ids = sorted(finger_to_sensor_id.values())
            if ids != [0, 1, 2, 3, 4]:
                raise ValueError(
                    f"finger_to_sensor_id values must be 0-4 with no duplicates, "
                    f"got {sorted(finger_to_sensor_id.values())}"
                )
            self._finger_to_sensor_id = dict(finger_to_sensor_id)
        self._sensor_id_to_finger = {v: k for k, v in self._finger_to_sensor_id.items()}

        self._tactile_config: TactileSensorConfiguration | None = None

        self._auto_thread: threading.Thread | None = None
        self._auto_running = threading.Event()
        self._auto_lock = threading.Lock()
        self._auto_latest = None
        self._auto_latest_taxels = None
        self._auto_latest_ts = None
        self._auto_stats = AutoStreamStats()
        self._auto_mode_resultant = True
        self._auto_mode_taxels = False
        self._last_frame_debug_print: float = 0.0

        # {finger: [[fx, fy, fz], ...], ...} per-taxel zeroing offsets.
        self._taxel_offsets: dict | None = None
        # {finger: [fx, fy, fz], ...} sum of taxel offsets per finger.
        self._resultant_offsets: dict | None = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self):
        """Open the serial link and capture the initial sensor configuration."""
        if self.is_connected:
            return

        try:
            self._serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            self._connected = True
            logger.info(f"Connected to sensor at {self.port}")

            # Initial config is best-effort; start_auto_stream will read it again.
            try:
                self._tactile_config = self._get_configuration()
                logger.info(f"Initial configuration: {self._tactile_config}")
            except IOError as e:
                logger.warning(f"Failed to get initial configuration: {e}")

        except (serial.SerialException, OSError) as e:
            raise ConnectionError(f"Failed to connect to sensor at {self.port}: {e}") from e

    def disconnect(self):
        if not self.is_connected:
            return
        if self._serial_connection and self._serial_connection.is_open:
            self._serial_connection.close()
        self._connected = False

    def _read_register(self, address: int, count: int = 1, response_timeout_s: float = 0.5) -> bytes:
        """Send a read request and return the data bytes from the response.

        Tolerates auto-stream mode: AA56 frames arriving before the AA55
        response are skipped instead of treated as the response.
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        request = build_read_request(address, count)

        # Stale bytes in the input buffer would otherwise be treated as the response.
        if not self._is_streaming():
            self._serial_connection.reset_input_buffer()

        self._serial_connection.write(request)

        deadline = time.time() + response_timeout_s
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise TimeoutError("Timed out waiting for read response (AA55).")

            hdr = self._read_header_resync(timeout_s=remaining)
            if hdr == PROTOCOL_HEADER_AUTO:
                self._skip_auto_frame()
                continue
            break

        body = self._read_exact(read_response_body_size(count))
        return parse_read_response(hdr + body)

    def _skip_auto_frame(self) -> None:
        """Discard one auto-stream frame after the AA56 header has been consumed."""
        meta = self._read_exact(AUTO_FRAME_META_SIZE)
        eff_len = extract_auto_frame_eff_len(meta)
        _ = self._read_exact(eff_len + 1)  # payload + LRC

    def _read_header_resync(self, timeout_s: float) -> bytes:
        """Slide a 2-byte window until AA55 (response) or AA56 (auto) is found.

        Needed because the bus may begin mid-frame and AA56 frames can interleave
        with AA55 responses at any time. Returns the 2-byte header.
        """
        deadline = time.time() + timeout_s
        b1 = b""
        while time.time() < deadline:
            b = self._serial_connection.read(1)
            if not b:
                continue

            if not b1:
                b1 = b
                continue

            hdr = b1 + b
            if hdr == PROTOCOL_HEADER_RESPONSE or hdr == PROTOCOL_HEADER_AUTO:
                return hdr

            b1 = b

        raise TimeoutError("Timed out waiting for AA55/AA56 header.")

    def _is_streaming(self) -> bool:
        return self._auto_running.is_set()

    def _write_register(
        self,
        address: int,
        data: bytes,
        response_timeout_s: float = 0.5,
    ) -> None:
        """Send a write request and validate the status byte in the response.

        Tolerates auto-stream mode the same way as ``_read_register``.
        Per Paxini manual, ``data`` is capped at 10 bytes.
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        request = build_write_request(address, data)

        if not self._is_streaming():
            self._serial_connection.reset_input_buffer()

        self._serial_connection.write(request)

        deadline = time.time() + response_timeout_s
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise TimeoutError("Timed out waiting for write response (AA55).")

            hdr = self._read_header_resync(timeout_s=remaining)
            if hdr == PROTOCOL_HEADER_AUTO:
                self._skip_auto_frame()
                continue
            break

        meta = self._read_exact(RESPONSE_META_SIZE)
        data_len = extract_write_response_data_length(meta)
        rest = self._read_exact(data_len + 1)  # payload + LRC
        parse_write_response(hdr + meta + rest)



    def read_connected_sensors(self) -> dict[str, bool]:
        """Return ``{finger: is_connected}`` from the connected-sensors register."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        data = self._read_register(ADDR_CONNECTED_SENSORS_START, ADDR_CONNECTED_SENSORS_LENGTH)
        return decode_connected_sensors(data, self._sensor_id_to_finger)

    def read_num_taxels(self) -> dict[str, int]:
        """Return ``{finger: taxel_count}`` from the taxel-count register block."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        data = self._read_register(ADDR_NUM_TAXELS_START, ADDR_NUM_TAXELS_LENGTH)
        return decode_num_taxels(data, self._sensor_id_to_finger)

    def read_auto_data_type(self) -> dict:
        """Return the decoded auto-stream data-type register."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        data = self._read_register(ADDR_AUTO_DATA_TYPE, 1)
        return decode_auto_data_type(data)

    def _read_raw_resultant(self) -> dict[str, list[float]]:
        """Read raw resultant forces from hardware. Overridden by MockTactileClient."""
        if self._tactile_config is None:
            try:
                self._tactile_config = self._get_configuration()
            except IOError as e:
                logger.error(f"Failed to get configuration: {e}")
                # Fall back to assuming all 5 sensors are connected.
                data = self._read_register(ADDR_RESULTANT_FORCE_START, RESULTANT_BLOCK_SIZE)
                module_indices = {f: compute_distal_module_index(self._finger_to_sensor_id[f]) for f in FINGER_NAMES}
                return decode_resultant_register(data, list(FINGER_NAMES), module_indices)

        data = self._read_register(ADDR_RESULTANT_FORCE_START, RESULTANT_BLOCK_SIZE)
        return decode_resultant_register(
            data, self._tactile_config.active_sensors, self._tactile_config.module_indices,
        )

    def read_resultant_force(self) -> dict[str, list[float]]:
        """Read resultant force from all connected fingertip sensors, applying offsets."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        result = self._read_raw_resultant()
        if self._resultant_offsets:
            self._apply_resultant_offsets(result)
        return result

    def get_tactile_configuration(self) -> TactileSensorConfiguration | None:
        """Return the cached sensor configuration, or ``None`` if never read."""
        return self._tactile_config

    def _get_configuration(self) -> TactileSensorConfiguration:
        """Read the current connected-sensors and taxel-count registers from hardware."""
        try:
            connected = self.read_connected_sensors()
            num_taxels = self.read_num_taxels()

            module_indices = {}
            for finger in FINGER_NAMES:
                if connected.get(finger, False):
                    sensor_id = self._finger_to_sensor_id[finger]
                    module_indices[finger] = compute_distal_module_index(sensor_id)

            config = TactileSensorConfiguration(
                connected=connected,
                num_taxels=num_taxels,
                module_indices=module_indices,
                finger_to_sensor_id=dict(self._finger_to_sensor_id),
            )

            logger.info(f"Configuration captured: {config}")
            return config

        except (IOError, FrameError) as e:
            logger.error(f"Failed to get sensor configuration: {e}")
            raise IOError(f"Failed to read sensor configuration: {e}") from e

    def set_auto_data_type(self, resultant: bool = True, taxels: bool = False) -> None:
        """Configure which data types are included in auto-stream frames."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        self._write_register(ADDR_AUTO_DATA_TYPE, encode_auto_data_type(resultant, taxels))


    def enable_auto_data_transmission(self) -> None:
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        self._write_register(ADDR_AUTO_ENABLE, REGISTER_ENABLE)

    def disable_auto_data_transmission(self) -> None:
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        self._write_register(ADDR_AUTO_ENABLE, REGISTER_DISABLE)


    def get_auto_latest(self):
        """Return ``(forces, timestamp)`` for the most recent resultant frame, or ``(None, None)``."""
        with self._auto_lock:
            return self._auto_latest, self._auto_latest_ts

    def get_auto_latest_taxels(self):
        """Return ``(taxels, timestamp)`` for the most recent taxel frame, or ``(None, None)``."""
        with self._auto_lock:
            return self._auto_latest_taxels, self._auto_latest_ts

    def get_auto_latest_all(self):
        """Return ``(forces, taxels, timestamp)`` from a single locked read.

        Use this in combined mode so forces and taxels share one timestamp.
        """
        with self._auto_lock:
            return self._auto_latest, self._auto_latest_taxels, self._auto_latest_ts

    def get_auto_stats(self):
        """Return a snapshot copy of ``AutoStreamStats`` for the current loop."""
        with self._auto_lock:
            return dataclasses.replace(self._auto_stats)

    def set_taxel_offsets(self, offsets: dict) -> None:
        """Store per-taxel zeroing offsets and derive matching resultant offsets."""
        self._taxel_offsets = offsets
        self._resultant_offsets = {}
        for finger, taxel_list in offsets.items():
            sum_fx = sum(t[0] for t in taxel_list)
            sum_fy = sum(t[1] for t in taxel_list)
            sum_fz = sum(t[2] for t in taxel_list)
            self._resultant_offsets[finger] = [sum_fx, sum_fy, sum_fz]

    def clear_taxel_offsets(self) -> None:
        """Clear all zeroing offsets."""
        self._taxel_offsets = None
        self._resultant_offsets = None

    def capture_taxel_offsets(self, num_samples: int = 100) -> dict:
        """Average ``num_samples`` taxel frames and apply the result as zeroing offsets.

        Requires an active auto-stream with taxels enabled. Existing offsets
        are temporarily cleared so the average reflects raw readings.
        """
        if not self._is_streaming() or not self._auto_mode_taxels:
            raise RuntimeError("Auto-stream with taxels must be active to capture offsets")

        # Clear any current offsets so we average raw frames, not offset-corrected ones.
        prev_taxel = self._taxel_offsets
        prev_resultant = self._resultant_offsets
        self._taxel_offsets = None
        self._resultant_offsets = None

        # Let the reader thread emit at least one new frame past the offset clear.
        time.sleep(0.01)

        succeeded = False
        try:
            frames = []
            last_ts = None
            while len(frames) < num_samples:
                taxels, ts = self.get_auto_latest_taxels()
                if taxels is not None and ts != last_ts:
                    frames.append(taxels)
                    last_ts = ts
                time.sleep(0.002)

            fingers = list(frames[0].keys())
            offsets = {}
            for finger in fingers:
                num_taxels = len(frames[0][finger])
                avg = []
                for t_idx in range(num_taxels):
                    sum_fx = sum(f[finger][t_idx][0] for f in frames)
                    sum_fy = sum(f[finger][t_idx][1] for f in frames)
                    sum_fz = sum(f[finger][t_idx][2] for f in frames)
                    avg.append([
                        round(sum_fx / num_samples, 2),
                        round(sum_fy / num_samples, 2),
                        round(sum_fz / num_samples, 2),
                    ])
                offsets[finger] = avg

            self.set_taxel_offsets(offsets)
            succeeded = True
            return offsets
        finally:
            if not succeeded:
                self._taxel_offsets = prev_taxel
                self._resultant_offsets = prev_resultant

    def _apply_taxel_offsets(self, taxels: dict) -> None:
        """Subtract per-taxel offsets in-place. Clamps fz to >= 0."""
        for finger, taxel_list in taxels.items():
            finger_offsets = self._taxel_offsets.get(finger)
            if not finger_offsets:
                continue
            for i, taxel in enumerate(taxel_list):
                if i >= len(finger_offsets):
                    break
                off = finger_offsets[i]
                taxel[0] = round(taxel[0] - off[0], 1)
                taxel[1] = round(taxel[1] - off[1], 1)
                taxel[2] = round(max(0, taxel[2] - off[2]), 1)

    def _apply_resultant_offsets(self, forces: dict) -> None:
        """Subtract resultant offsets in-place. Clamps fz to >= 0."""
        for finger, fvec in forces.items():
            off = self._resultant_offsets.get(finger)
            if not off:
                continue
            fvec[0] = round(fvec[0] - off[0], 1)
            fvec[1] = round(fvec[1] - off[1], 1)
            fvec[2] = round(max(0, fvec[2] - off[2]), 1)

    def _on_frame_stored(self) -> None:
        """Hook for subclasses to signal frame availability (e.g. for tests)."""

    def _apply_stream_offsets(
        self,
        parsed_resultant: dict | None,
        parsed_taxels: dict | None,
    ) -> None:
        """Apply zeroing offsets to parsed auto-stream data in-place."""
        if self._taxel_offsets and parsed_taxels:
            self._apply_taxel_offsets(parsed_taxels)
        if self._resultant_offsets and parsed_resultant:
            self._apply_resultant_offsets(parsed_resultant)

    def _read_exact(self, n: int) -> bytes:
        """Read exactly n bytes, raising IOError if the serial 1.0s timeout fires."""
        out = bytearray()
        while len(out) < n:
            chunk = self._serial_connection.read(n - len(out))
            if chunk is None or len(chunk) == 0:
                raise IOError(f"Serial read timeout after reading {len(out)}/{n} bytes")
            out.extend(chunk)
        return bytes(out)

    def _resync_to_auto_header(self) -> None:
        """Slide a 2-byte window over the stream until 0xAA 0x56 is found.

        Exits and raises IOError when ``_auto_running`` is cleared, so a
        stop request unblocks this loop instead of hanging on the next read.
        """
        b1 = self._read_exact(1)
        while self._auto_running.is_set():
            b2 = self._read_exact(1)
            if b1 + b2 == PROTOCOL_HEADER_AUTO:
                return
            b1 = b2

        raise IOError("Auto stream stopped during resync")

    def _get_expected_payload_size(self, config: TactileSensorConfiguration) -> int:
        """Get expected payload size based on current streaming mode."""
        return compute_expected_payload_size(
            self._auto_mode_resultant,
            self._auto_mode_taxels,
            config.active_sensors,
            config.num_taxels,
        )

    def _acquire_frame(
        self,
        parse_resultant: bool,
        parse_taxels: bool,
    ) -> tuple[dict | None, dict | None]:
        """Read one frame from serial, validate LRC, decode according to mode.

        Overridden by ``MockTactileClient`` to swap the data source while
        keeping the surrounding stats, offset, and lifecycle logic.
        """
        self._resync_to_auto_header()

        meta = self._read_exact(AUTO_FRAME_META_SIZE)
        eff_len = extract_auto_frame_eff_len(meta)
        payload = self._read_exact(eff_len)
        lrc = self._read_exact(1)[0]

        now = time.time()
        if now - self._last_frame_debug_print > 1.0:
            config_str = str(self._tactile_config) if self._tactile_config else "no config"
            logger.debug(f"[auto] eff_len={eff_len}, config={config_str}")
            self._last_frame_debug_print = now

        if not validate_auto_frame_lrc(meta, payload, lrc):
            raise FrameError("LRC mismatch", bad_lrc=True)

        err_code, valid = unpack_auto_payload(payload)

        with self._auto_lock:
            self._auto_stats.last_error_code = err_code

        if not self._tactile_config:
            raise FrameError("No sensor configuration available")

        expected_size = self._get_expected_payload_size(self._tactile_config)
        if len(valid) != expected_size or expected_size == 0:
            raise FrameError(
                f"Unexpected payload: {len(valid)} bytes, expected {expected_size}"
            )

        cfg = self._tactile_config
        if parse_resultant and parse_taxels:
            parsed_resultant, parsed_taxels = decode_combined_auto(
                valid, cfg.active_sensors, cfg.num_taxels,
            )
        elif parse_resultant:
            parsed_resultant = decode_resultant_auto(valid, cfg.active_sensors)
            parsed_taxels = None
        elif parse_taxels:
            parsed_resultant = None
            parsed_taxels = decode_taxels_auto(
                valid, cfg.active_sensors, cfg.num_taxels,
            )
        else:
            parsed_resultant = None
            parsed_taxels = None

        return parsed_resultant, parsed_taxels

    def _auto_reader_loop(self, parse_resultant: bool, parse_taxels: bool):
        """Background thread: acquire → offset → store → repeat.
        """
        while self._auto_running.is_set():
            try:
                parsed_resultant, parsed_taxels = self._acquire_frame(
                    parse_resultant, parse_taxels
                )

                self._apply_stream_offsets(parsed_resultant, parsed_taxels)

                with self._auto_lock:
                    if parse_resultant and parsed_resultant is not None:
                        self._auto_latest = parsed_resultant
                    if parse_taxels and parsed_taxels is not None:
                        self._auto_latest_taxels = parsed_taxels
                    self._auto_latest_ts = time.time()
                    self._auto_stats.frames_ok += 1
                self._on_frame_stored()

            except FrameError as e:
                with self._auto_lock:
                    if e.bad_lrc:
                        self._auto_stats.frames_bad_checksum += 1
                    else:
                        self._auto_stats.parse_errors += 1

            except IOError as e:
                if "Auto stream stopped" in str(e):
                    logger.info("Auto stream stopped")
                    break
                logger.warning(f"IO error in auto reader: {e}")
                with self._auto_lock:
                    self._auto_stats.resyncs += 1
                time.sleep(0.01)

            except Exception:
                logger.exception("Unexpected error in auto reader, stopping stream")
                self._auto_running.clear()
                break

        logger.info("Auto reader loop exited")

    def start_auto_stream(self, resultant: bool = True, taxels: bool = False, min_sensors: int = 1):
        """Start the ~1 kHz auto-stream and the background reader thread.

        Read the latest frame via ``get_auto_latest`` / ``get_auto_latest_taxels`` /
        ``get_auto_latest_all``. ``min_sensors`` gates startup: if fewer sensors
        enumerate at power-on, ``NoSensorsAvailableError`` is raised.

        Sensor presence is fixed at stream start. The Paxini PX-6AX GEN3 board
        enumerates at power-on and does not report mid-stream disconnects —
        an unplugged slot keeps emitting its last bytes until the board is
        power-cycled.
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        if not resultant and not taxels:
            raise ValueError("At least one of resultant or taxels must be enabled")

        self.stop_auto_stream()

        self._auto_mode_resultant = resultant
        self._auto_mode_taxels = taxels

        try:
            self._tactile_config = self._get_configuration()
        except IOError as e:
            raise OSError(f"Failed to get sensor configuration: {e}") from e

        if self._tactile_config.num_active_sensors < min_sensors:
            raise NoSensorsAvailableError(
                f"Only {self._tactile_config.num_active_sensors} sensor(s) available, "
                f"need at least {min_sensors}"
            )

        expected_size = self._get_expected_payload_size(self._tactile_config)
        mode_str = []
        if resultant:
            mode_str.append("resultant")
        if taxels:
            mode_str.append("taxels")
        logger.info(
            f"Starting auto-stream with {self._tactile_config}, "
            f"mode={'+'.join(mode_str)}, expected_payload={expected_size} bytes"
        )

        # Disable in case the sensor was left in auto mode from a prior run;
        # _write_register tolerates incoming AA56 frames if it is still streaming.
        try:
            self.disable_auto_data_transmission()
        except IOError:
            pass

        self.set_auto_data_type(resultant=resultant, taxels=taxels)

        if self._serial_connection is not None:
            self._serial_connection.reset_input_buffer()

        self.enable_auto_data_transmission()

        self._auto_running.set()
        self._auto_thread = threading.Thread(
            target=self._auto_reader_loop,
            args=(resultant, taxels),
            daemon=True,
        )
        self._auto_thread.start()


    def stop_auto_stream(self):
        """Stop auto-stream mode and clean up the background thread.

        Safe to call multiple times and when the sensor is already disconnected.
        """
        self._auto_running.clear()

        if self._auto_thread is not None:
            self._auto_thread.join(timeout=1.0)
            self._auto_thread = None

        if self.is_connected:
            try:
                self.disable_auto_data_transmission()
            except IOError:
                pass

        with self._auto_lock:
            self._auto_latest = None
            self._auto_latest_taxels = None
            self._auto_latest_ts = None

    def __enter__(self):
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
