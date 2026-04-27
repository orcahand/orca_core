# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
from __future__ import annotations

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
    compute_resultant_payload_size,
    compute_taxel_payload_size,
    compute_combined_payload_size,
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
    """Raised when no sensors are available for communication."""
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
    """Diagnostic counters for the auto-stream reader loop.

    Attributes:
        frames_ok: Frames received and decoded successfully.
        frames_bad_checksum: Frames rejected due to checksum (LRC) mismatch.
        parse_errors: Frames received intact but whose payload failed to decode.
        resyncs: Times the reader had to resync after IO errors or bad framing.
        reconfiguration_count: Times the sensor configuration was re-queried
            after a payload-size mismatch.
        last_error_code: Most recent sensor-reported error code (0 = no error).
    """
    frames_ok: int = 0
    frames_bad_checksum: int = 0
    parse_errors: int = 0
    resyncs: int = 0
    reconfiguration_count: int = 0
    last_error_code: int = 0


@dataclass
class SensorConfiguration:
    """Snapshot of connected sensors and their properties.

    This configuration is captured when connecting or when errors trigger
    reconfiguration. It's used to build dynamic parsers that adapt to
    available sensors.
    """
    connected: dict[str, bool] = field(default_factory=dict)  # {finger: is_connected}
    num_taxels: dict[str, int] = field(default_factory=dict)  # {finger: taxel_count}
    module_indices: dict[str, int] = field(default_factory=dict)  # {finger: module_idx}
    expected_payload_size_resultant: int = 0  # Expected bytes for resultant force mode
    expected_payload_size_taxels: int = 0  # Expected bytes for taxel mode
    expected_payload_size_combined: int = 0  # Expected bytes for resultant + taxel mode
    timestamp: float = 0.0  # When this config was captured
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
        """Number of currently connected sensors."""
        return len(self.active_sensors)

    def __str__(self) -> str:
        """Human-readable representation."""
        active = ", ".join(self.active_sensors) if self.active_sensors else "none"
        return f"SensorConfig({self.num_active_sensors} active: {active})"


class SensorClient:
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

        # Sensor configuration (dynamic, adapts to connected sensors)
        self._sensor_config: SensorConfiguration | None = None

        self._auto_thread: threading.Thread | None = None
        self._auto_running = threading.Event()  # Thread-safe flag for auto stream
        self._auto_lock = threading.Lock()
        self._auto_latest = None            # parsed resultant forces dict
        self._auto_latest_taxels = None     # parsed taxels dict
        self._auto_latest_ts = None
        self._auto_stats = AutoStreamStats()
        self._auto_mode_resultant = True    # Whether to parse resultant forces
        self._auto_mode_taxels = False      # Whether to parse taxels
        self._last_frame_debug_print: float = 0.0

        # Per-taxel zeroing offsets
        self._taxel_offsets: dict | None = None      # {finger: [[fx, fy, fz], ...], ...}
        self._resultant_offsets: dict | None = None   # {finger: [fx, fy, fz], ...}

    @property
    def is_connected(self) -> bool:
        """Check if client is connected."""
        return self._connected

    def connect(self):
        """Connect to the sensor device and get initial configuration.

        This method establishes serial communication and reads the initial sensor
        configuration (which sensors are connected, taxel counts, etc.).

        Raises:
            ConnectionError: If serial connection fails
            IOError: If unable to read sensor configuration
        """
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

            # Get initial sensor configuration
            try:
                self._sensor_config = self._get_configuration()
                logger.info(f"Initial configuration: {self._sensor_config}")
            except IOError as e:
                logger.warning(f"Failed to get initial configuration: {e}")
                # Don't fail connection, config will be retrieved when starting auto-stream

        except (serial.SerialException, OSError) as e:
            raise ConnectionError(f"Failed to connect to sensor at {self.port}: {e}") from e

    def disconnect(self):
        """Disconnect from the sensor device."""
        if not self.is_connected:
            return
            
        if self._serial_connection and self._serial_connection.is_open:
            self._serial_connection.close()
        self._connected = False

    def _read_register(self, address: int, count: int = 1, response_timeout_s: float = 0.5) -> bytes:
        """Read one or more registers

        Protocol flow:
        1. Send request frame: 55 AA | reserved | 0x03 | addr(2) | count(2) | LRC
        2. Wait for response frame: AA 55 | reserved | 0x03 | addr(2) | count(2) | data(count) | LRC
        3. Handle auto frames (AA 56) that may arrive while waiting for response

        This method is robust against auto-stream mode: while waiting for the AA55
        response, any AA56 auto frames that arrive are skipped automatically.

        Args:
            address: Register address to read from
            count: Number of bytes to read
            response_timeout_s: Maximum time to wait for response (default: 0.5s)

        Returns:
            Raw bytes read from registers

        Raises:
            OSError: If not connected
            TimeoutError: If no response received within timeout
            IOError: If response checksum fails
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        request = build_read_request(address, count)

        # Clear stale data if not streaming (prevents reading old responses)
        if not self._is_streaming():
            self._serial_connection.reset_input_buffer()

        self._serial_connection.write(request)

        # Wait for AA55 response header, skipping any AA56 auto frames
        deadline = time.time() + response_timeout_s
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise TimeoutError("Timed out waiting for read response (AA55).")

            hdr = self._read_header_resync(timeout_s=remaining)
            if hdr == PROTOCOL_HEADER_AUTO:  # AA56 auto frame
                self._skip_auto_frame()
                continue
            break  # Found AA55 response

        body = self._read_exact(read_response_body_size(count))
        return parse_read_response(hdr + body)

    def _skip_auto_frame(self) -> None:
        """Skip one complete auto-stream frame after having consumed the AA56 header.

        This is called when waiting for a request-response (AA55) frame but an
        auto-stream (AA56) frame arrives first. We skip it to continue waiting
        for the AA55 response.

        Raises:
            IOError: If serial read fails
            ValueError: If eff_len is unreasonably large (>8KB)
        """
        meta = self._read_exact(AUTO_FRAME_META_SIZE)
        eff_len = extract_auto_frame_eff_len(meta)
        _ = self._read_exact(eff_len + 1)  # payload + LRC

    def _read_header_resync(self, timeout_s: float) -> bytes:
        """Read bytes until we find either AA55 (response) or AA56 (auto) header.

        Uses a sliding 2-byte window to locate frame headers even if the byte
        stream starts mid-frame or is misaligned. This is critical for robustness
        when auto-stream frames (AA56) can arrive at any time, even when waiting
        for request-response frames (AA55).

        Args:
            timeout_s: Maximum time to search for a header before giving up

        Returns:
            The 2-byte header (either AA55 or AA56)

        Raises:
            TimeoutError: If no valid header found within timeout_s
        """
        deadline = time.time() + timeout_s

        # Sliding 2-byte window: [b1, b]
        b1 = b""
        while time.time() < deadline:
            b = self._serial_connection.read(1)
            if not b:
                continue  # Serial timeout tick, keep trying until deadline

            # Build up 2-byte window
            if not b1:
                b1 = b
                continue

            hdr = b1 + b
            # Check if we found a valid header
            if hdr == PROTOCOL_HEADER_RESPONSE or hdr == PROTOCOL_HEADER_AUTO:
                return hdr

            # Slide window: b becomes new b1
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
        """Write bytes to registers

        Protocol flow:
        1. Send request: 55 AA | reserved | 0x10 | addr(2) | len(2) | data | LRC
        2. Wait for response: AA 55 | reserved | 0x10 | addr(2) | len(2) | status | LRC
        3. Handle auto frames (AA 56) that may arrive while waiting for response
        4. Check status byte (0x00 = success)

        This method is robust against auto-stream mode: while waiting for the AA55
        response, any AA56 auto frames that arrive are skipped automatically.

        Args:
            address: Register address to write to
            data: Bytes to write (max 10 bytes according to manual)
            response_timeout_s: Maximum time to wait for response (default: 0.5s)

        Raises:
            OSError: If not connected
            TimeoutError: If no response received within timeout
            IOError: If response checksum fails or status byte indicates failure
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        request = build_write_request(address, data)

        # Clear stale data if not streaming (prevents reading old responses)
        if not self._is_streaming():
            self._serial_connection.reset_input_buffer()

        self._serial_connection.write(request)

        # Wait for AA55 response header, skipping any AA56 auto frames
        deadline = time.time() + response_timeout_s
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise TimeoutError("Timed out waiting for write response (AA55).")

            hdr = self._read_header_resync(timeout_s=remaining)

            if hdr == PROTOCOL_HEADER_AUTO:  # AA56 auto frame
                self._skip_auto_frame()
                continue

            # Found AA55 response
            break

        # Read and parse response: header(2) + meta(6) + payload(nbytes) + LRC(1)
        meta = self._read_exact(RESPONSE_META_SIZE)
        data_len = extract_write_response_data_length(meta)
        rest = self._read_exact(data_len + 1)  # payload + LRC
        parse_write_response(hdr + meta + rest)



    def read_connected_sensors(self) -> dict[str, bool]:
        """Read the connected sensors.

        Returns:
            Dictionary of sensor names and their status

        Raises:
            OSError: If not connected to sensor
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        data = self._read_register(ADDR_CONNECTED_SENSORS_START, ADDR_CONNECTED_SENSORS_LENGTH)
        return decode_connected_sensors(data, self._sensor_id_to_finger)

    def read_num_taxels(self) -> dict[str, int]:
        """Read the number of taxels for each fingertip sensor.

        Returns:
            Dictionary mapping finger names to taxel counts

        Raises:
            OSError: If not connected to sensor
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        data = self._read_register(ADDR_NUM_TAXELS_START, ADDR_NUM_TAXELS_LENGTH)
        return decode_num_taxels(data, self._sensor_id_to_finger)

    def read_auto_data_type(self) -> dict:
        """Read the auto-stream data-type register.

        Returns the configured payload format for auto-stream frames (which of
        resultant force / individual taxels are included).

        Returns:
            Dict with the raw register byte and decoded resultant/taxels flags.

        Raises:
            OSError: If not connected to sensor.
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        data = self._read_register(ADDR_AUTO_DATA_TYPE, 1)
        return decode_auto_data_type(data)

    def _read_raw_resultant(self) -> dict[str, list[float]]:
        """Read raw resultant forces from hardware (no offset application).

        MockSensorClient overrides this to return simulated
        data. The public read_resultant_force() method calls this, then applies
        zeroing offsets.

        Returns:
            Dictionary mapping finger names to [fx, fy, fz] force vectors in Newtons
        """
        # Ensure we have current configuration
        if self._sensor_config is None:
            try:
                self._sensor_config = self._get_configuration()
            except IOError as e:
                logger.error(f"Failed to get configuration: {e}")
                # Fall back to static parsing using default module indices
                data = self._read_register(ADDR_RESULTANT_FORCE_START, RESULTANT_BLOCK_SIZE)
                module_indices = {f: compute_distal_module_index(self._finger_to_sensor_id[f]) for f in FINGER_NAMES}
                return decode_resultant_register(data, list(FINGER_NAMES), module_indices)

        data = self._read_register(ADDR_RESULTANT_FORCE_START, RESULTANT_BLOCK_SIZE)
        return decode_resultant_register(
            data, self._sensor_config.active_sensors, self._sensor_config.module_indices,
        )

    def read_resultant_force(self) -> dict[str, list[float]]:
        """Read resultant force from all connected fingertip sensors.

        Calls _read_raw_resultant() for data, then applies zeroing offsets.

        Returns:
            Dictionary mapping finger names to [fx, fy, fz] force vectors in Newtons
            Only includes sensors that are currently connected

        Raises:
            OSError: If not connected to sensor
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        result = self._read_raw_resultant()
        if self._resultant_offsets:
            self._apply_resultant_offsets(result)
        return result

    def get_sensor_configuration(self) -> SensorConfiguration | None:
        """Get the current sensor configuration snapshot.

        Returns:
            SensorConfiguration object with current sensor state, or None if not yet configured
        """
        return self._sensor_config

    def _get_configuration(self) -> SensorConfiguration:
        """Snapshot the current sensor configuration.

        Reads connected sensors and their properties from the hardware.
        This is called on connect and when errors trigger reconfiguration.

        Returns:
            SensorConfiguration with current hardware state

        Raises:
            IOError: If unable to read configuration from sensor
        """
        try:
            connected = self.read_connected_sensors()
            num_taxels = self.read_num_taxels()

            # Build module indices for active sensors (fingertip only)
            module_indices = {}
            for finger in FINGER_NAMES:
                if connected.get(finger, False):
                    sensor_id = self._finger_to_sensor_id[finger]
                    module_indices[finger] = compute_distal_module_index(sensor_id)

            # Calculate expected payload sizes
            active = [f for f in FINGER_NAMES if connected.get(f, False)]
            expected_resultant = compute_resultant_payload_size(len(active))
            expected_taxels = compute_taxel_payload_size(active, num_taxels)
            expected_combined = compute_combined_payload_size(active, num_taxels)

            config = SensorConfiguration(
                connected=connected,
                num_taxels=num_taxels,
                module_indices=module_indices,
                expected_payload_size_resultant=expected_resultant,
                expected_payload_size_taxels=expected_taxels,
                expected_payload_size_combined=expected_combined,
                timestamp=time.time(),
                finger_to_sensor_id=dict(self._finger_to_sensor_id),
            )

            logger.info(f"Configuration captured: {config}")
            return config

        except (IOError, FrameError) as e:
            logger.error(f"Failed to get sensor configuration: {e}")
            raise IOError(f"Failed to read sensor configuration: {e}") from e

    def _reconfigure(self) -> bool:
        """Re-query the sensor board and update the cached configuration.

        Called by `_acquire_frame` when a payload-size mismatch indicates the
        hardware reconfigured itself (sensor connected or disconnected).

        Returns:
            True if the active-sensor set changed, False if unchanged.

        Raises:
            NoSensorsAvailableError: If no sensors are connected after reconfiguration.
        """
        logger.info("Attempting reconfiguration...")
        new_config = self._get_configuration()

        if self._sensor_config is not None:
            old_active = set(self._sensor_config.active_sensors)
            new_active = set(new_config.active_sensors)

            if old_active == new_active:
                logger.debug("Configuration unchanged, no reconfiguration needed")
                return False

            added = new_active - old_active
            removed = old_active - new_active
            if added:
                logger.info(f"Sensors added: {', '.join(added)}")
            if removed:
                logger.warning(f"Sensors removed: {', '.join(removed)}")

        self._sensor_config = new_config

        with self._auto_lock:
            self._auto_stats.reconfiguration_count += 1

        if new_config.num_active_sensors == 0:
            logger.error("No sensors available after reconfiguration")
            raise NoSensorsAvailableError("All sensors disconnected")

        logger.info(f"Reconfiguration successful: {new_config}")
        return True


    def set_auto_data_type(self, resultant: bool = True, taxels: bool = False) -> None:
        """Configure which data types to include in auto stream.

        Args:
            resultant: Include resultant force data
            taxels: Include individual taxel force data

        Raises:
            OSError: If not connected to sensor
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        self._write_register(ADDR_AUTO_DATA_TYPE, encode_auto_data_type(resultant, taxels))


    def enable_auto_data_transmission(self) -> None:
        """Enable automatic data transmission mode.

        Raises:
            OSError: If not connected to sensor
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        self._write_register(ADDR_AUTO_ENABLE, REGISTER_ENABLE)

    def disable_auto_data_transmission(self) -> None:
        """Disable automatic data transmission mode.

        Raises:
            OSError: If not connected to sensor
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        self._write_register(ADDR_AUTO_ENABLE, REGISTER_DISABLE)


    def get_auto_latest(self):
        """Thread-safe snapshot of the most recent resultant-force frame.

        Updated by the background reader thread while auto-stream is active.

        Returns:
            (forces, timestamp): `forces` is a {finger: [fx, fy, fz]} dict in
            Newtons, or None if no frame has arrived or resultant mode is off.
            `timestamp` is the wall-clock time of receipt, or None.
        """
        with self._auto_lock:
            return self._auto_latest, self._auto_latest_ts

    def get_auto_latest_taxels(self):
        """Thread-safe snapshot of the most recent per-taxel frame.

        Only populated when auto-stream was started with `taxels=True`.

        Returns:
            (taxels, timestamp): `taxels` is a {finger: [[fx, fy, fz], ...]}
            dict (one [fx, fy, fz] per taxel, in Newtons), or None if no frame
            has arrived or taxel mode is off. `timestamp` is the wall-clock
            time of receipt, or None.
        """
        with self._auto_lock:
            return self._auto_latest_taxels, self._auto_latest_ts

    def get_auto_latest_all(self):
        """Thread-safe snapshot of both resultant and taxel data in one call.

        Useful in combined mode (`resultant=True, taxels=True`) to get a
        consistent view without two separate locked reads.

        Returns:
            (forces, taxels, timestamp). Any field may be None depending on
            the active stream mode and whether a frame has arrived yet.
        """
        with self._auto_lock:
            return self._auto_latest, self._auto_latest_taxels, self._auto_latest_ts

    def get_auto_stats(self):
        """Thread-safe snapshot of auto-stream diagnostics.

        See `AutoStreamStats` for the meaning of each field. Useful for health
        monitoring (frame rate, checksum errors, reconfigurations).
        """
        with self._auto_lock:
            return self._auto_stats
    
    def set_taxel_offsets(self, offsets: dict) -> None:
        """Set per-taxel zeroing offsets and compute resultant offsets.

        Args:
            offsets: {finger: [[fx, fy, fz], ...], ...} per-taxel offsets
        """
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
        """Capture live baseline offsets by averaging current sensor readings.

        Requires active auto-stream with taxels enabled. Temporarily clears
        any existing offsets so raw sensor data is captured.

        Args:
            num_samples: Number of unique frames to average

        Returns:
            Per-taxel offsets dict: {finger: [[fx, fy, fz], ...], ...}
        """
        if not self._is_streaming() or not self._auto_mode_taxels:
            raise RuntimeError("Auto-stream with taxels must be active to capture offsets")

        # Temporarily clear offsets to capture raw data
        prev_taxel = self._taxel_offsets
        prev_resultant = self._resultant_offsets
        self._taxel_offsets = None
        self._resultant_offsets = None

        # Wait for at least one raw frame to flush old offset-applied data
        time.sleep(0.01)

        succeeded = False
        try:
            # Collect unique frames by checking timestamps
            frames = []
            last_ts = None
            while len(frames) < num_samples:
                taxels, ts = self.get_auto_latest_taxels()
                if taxels is not None and ts != last_ts:
                    frames.append(taxels)
                    last_ts = ts
                time.sleep(0.002)

            # Average per-taxel [fx, fy, fz] across all frames
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
                # Restore previous offsets on failure
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
        """Hook called after a frame is stored in _auto_latest. Subclasses
        may override to signal frame availability (e.g. for tests)."""

    def _apply_stream_offsets(
        self,
        parsed_resultant: dict | None,
        parsed_taxels: dict | None,
    ) -> None:
        """Apply zeroing offsets to parsed auto-stream data in-place.

        Called by _auto_reader_loop implementations after parsing raw data.
        """
        if self._taxel_offsets and parsed_taxels:
            self._apply_taxel_offsets(parsed_taxels)
        if self._resultant_offsets and parsed_resultant:
            self._apply_resultant_offsets(parsed_resultant)

    def _read_exact(self, n: int) -> bytes:
        """Read exactly n bytes from serial connection, blocking until complete.

        This is a fundamental building block for the protocol implementation.
        Unlike serial.read(n) which may return fewer bytes, this guarantees
        exactly n bytes are read or an error is raised.

        The serial connection has a 1.0s timeout (set in connect()). If no data
        arrives within that window, this raises IOError. This prevents infinite
        blocking on sensor disconnect or communication failure.

        Args:
            n: Number of bytes to read

        Returns:
            Exactly n bytes read from serial connection

        Raises:
            IOError: If serial read times out (1.0s) or connection is closed
        """
        out = bytearray()
        while len(out) < n:
            chunk = self._serial_connection.read(n - len(out))
            if chunk is None or len(chunk) == 0:
                # Serial timeout or disconnect
                raise IOError(f"Serial read timeout after reading {len(out)}/{n} bytes")
            out.extend(chunk)
        return bytes(out)

    def _resync_to_auto_header(self) -> None:
        """Scan byte stream until we see 0xAA 0x56 auto-stream header.

        Uses a sliding 2-byte window to find the header even if the stream
        starts mid-frame or becomes misaligned. Exits cleanly when auto stream
        is stopped via _auto_running.clear().

        Performance note: Checking is_set() adds <0.01% overhead since serial I/O
        (milliseconds) dominates over the flag check (microseconds).

        Raises:
            IOError: If auto stream is stopped while resyncing
        """
        # Sliding 2-byte window to find AA 56 header
        b1 = self._read_exact(1)
        while self._auto_running.is_set():
            b2 = self._read_exact(1)
            if b1 + b2 == PROTOCOL_HEADER_AUTO:
                return  # Found the header
            # Slide window: b2 becomes new b1
            b1 = b2

        # If we exit the loop, auto stream was stopped
        raise IOError("Auto stream stopped during resync")

    def _get_expected_payload_size(self, config: SensorConfiguration) -> int:
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
        min_sensors: int,
    ) -> tuple[dict | None, dict | None]:
        """Acquire and return the next parsed (resultant, taxels) frame.

        Reads one auto-stream frame from serial, validates LRC, handles payload
        size mismatches with reconfiguration, and parses the data.

        Subclasses (e.g. MockSensorClient) override this to provide data from
        other sources while inheriting the loop's stats, offset, and lifecycle logic.

        Returns:
            Tuple of (parsed_resultant, parsed_taxels). Either may be None
            if not requested.

        Raises:
            FrameError: Recoverable frame-level error (bad LRC, parse failure)
            NoSensorsAvailableError: No sensors available after reconfiguration
            IOError: Serial communication failure or auto stream stopped
        """
        # Find and consume AA 56 header
        self._resync_to_auto_header()

        # Read frame metadata, payload, and checksum
        meta = self._read_exact(AUTO_FRAME_META_SIZE)
        eff_len = extract_auto_frame_eff_len(meta)
        payload = self._read_exact(eff_len)
        lrc = self._read_exact(1)[0]

        # Debug print (throttled to once per second)
        now = time.time()
        if now - self._last_frame_debug_print > 1.0:
            config_str = str(self._sensor_config) if self._sensor_config else "no config"
            logger.debug(f"[auto] eff_len={eff_len}, config={config_str}")
            self._last_frame_debug_print = now

        # Validate frame integrity
        if not validate_auto_frame_lrc(meta, payload, lrc):
            raise FrameError("LRC mismatch", bad_lrc=True)

        # Split error code and force data
        err_code, valid = unpack_auto_payload(payload)

        # Update serial-specific stats
        with self._auto_lock:
            self._auto_stats.last_error_code = err_code

        # Check for payload size mismatch (indicates config change)
        if self._sensor_config:
            expected_size = self._get_expected_payload_size(self._sensor_config)
            if len(valid) != expected_size and expected_size > 0:
                logger.warning(
                    f"Payload size mismatch: expected {expected_size}, got {len(valid)}. "
                    "Triggering reconfiguration..."
                )
                if self._reconfigure():
                    logger.info("Reconfiguration successful, continuing stream")
                raise FrameError("Payload size mismatch, skipping frame")

        # Parse payload based on mode
        if not self._sensor_config:
            raise FrameError("No sensor configuration available")

        expected_size = self._get_expected_payload_size(self._sensor_config)
        if len(valid) != expected_size or expected_size == 0:
            raise FrameError(
                f"Unexpected payload: {len(valid)} bytes, expected {expected_size}"
            )

        cfg = self._sensor_config
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

    def _auto_reader_loop(self, parse_resultant: bool, parse_taxels: bool, min_sensors: int):
        """Background thread that continuously reads and parses auto-stream frames.

        Calls _acquire_frame() to get parsed data, then applies offsets and updates
        stats. Subclasses override _acquire_frame() to change the data source while
        inheriting the shared loop logic.

        Error handling:
        - FrameError: recoverable (bad LRC / parse error / size mismatch). Count
          and continue. Size-mismatch frames trigger reconfiguration inside
          _acquire_frame.
        - NoSensorsAvailableError: terminal. Stop the stream cleanly.
        - IOError: serial-level hiccup. Count, back off briefly, continue.
        - Any other Exception: unexpected (likely a programming bug). Log the
          traceback and stop the stream loudly rather than rotting silently.

        Args:
            parse_resultant: Whether to parse resultant force data
            parse_taxels: Whether to parse individual taxel data
            min_sensors: Minimum number of sensors required to continue streaming
        """
        while self._auto_running.is_set():
            try:
                parsed_resultant, parsed_taxels = self._acquire_frame(
                    parse_resultant, parse_taxels, min_sensors
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

            except NoSensorsAvailableError:
                logger.error("No sensors available, stopping stream")
                self._auto_running.clear()
                break

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
        """Start continuous auto-stream mode for real-time force data.

        In auto-stream mode, the sensor continuously broadcasts force data at ~1kHz
        without requiring request-response polling. This provides much lower latency
        and higher throughput than repeatedly calling read_resultant_force().

        The data is read by a background thread and made available via:
        - get_auto_latest(): for resultant force data
        - get_auto_latest_taxels(): for taxel data
        - get_auto_latest_all(): for both

        The system automatically adapts to sensor configuration changes (connects/disconnects).

        Setup sequence:
        1. Stop any existing stream
        2. Get sensor configuration (which sensors are connected)
        3. Configure data type (0x0016: resultant and/or taxels)
        4. Clear serial buffer to remove stale data
        5. Enable auto transmission (0x0017 = 1)
        6. Start background reader thread

        Args:
            resultant: Include resultant force data (fx, fy, fz per sensor)
            taxels: Include individual taxel force data
            min_sensors: Minimum number of sensors required (default: 1)
                        If fewer sensors available, raises NoSensorsAvailableError

        Raises:
            OSError: If not connected to sensor
            NoSensorsAvailableError: If fewer than min_sensors are available
            ValueError: If neither resultant nor taxels is enabled
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        if not resultant and not taxels:
            raise ValueError("At least one of resultant or taxels must be enabled")

        # Stop any existing stream to ensure clean state
        self.stop_auto_stream()

        # Store mode settings for the reader loop
        self._auto_mode_resultant = resultant
        self._auto_mode_taxels = taxels

        # Get initial sensor configuration
        try:
            self._sensor_config = self._get_configuration()
        except IOError as e:
            raise OSError(f"Failed to get sensor configuration: {e}") from e

        # Check minimum sensor requirement
        if self._sensor_config.num_active_sensors < min_sensors:
            raise NoSensorsAvailableError(
                f"Only {self._sensor_config.num_active_sensors} sensor(s) available, "
                f"need at least {min_sensors}"
            )

        # Log expected payload size for debugging
        expected_size = self._get_expected_payload_size(self._sensor_config)
        mode_str = []
        if resultant:
            mode_str.append("resultant")
        if taxels:
            mode_str.append("taxels")
        logger.info(
            f"Starting auto-stream with {self._sensor_config}, "
            f"mode={'+'.join(mode_str)}, expected_payload={expected_size} bytes"
        )

        # Try to disable auto mode first (in case it was left enabled)
        try:
            self.disable_auto_data_transmission()
        except IOError:
            pass  # If this fails, robust _write_register will handle AA56 frames

        # Configure which data types to include in auto frames
        self.set_auto_data_type(resultant=resultant, taxels=taxels)

        # Clear any stale data from serial buffer before starting
        if self._serial_connection is not None:
            self._serial_connection.reset_input_buffer()

        # Enable auto transmission mode (sensor starts broadcasting)
        self.enable_auto_data_transmission()

        # Start background reader thread
        self._auto_running.set()  # Signal thread to run
        self._auto_thread = threading.Thread(
            target=self._auto_reader_loop,
            args=(resultant, taxels, min_sensors),
            daemon=True  # Thread exits when main program exits
        )
        self._auto_thread.start()


    def stop_auto_stream(self):
        """Stop auto-stream mode and clean up background thread.

        Sequence:
        1. Signal thread to stop (_auto_running.clear())
        2. Wait up to 1.0s for thread to exit gracefully
        3. Disable auto transmission on sensor (0x0017 = 0)
        4. Reset cached data

        This method is safe to call multiple times and handles cases where
        the sensor is disconnected or the thread is already stopped.
        """
        # Signal background thread to stop
        self._auto_running.clear()

        # Wait for thread to exit (up to 1 second)
        if self._auto_thread is not None:
            self._auto_thread.join(timeout=1.0)
            self._auto_thread = None

        # Disable auto mode on sensor (if still connected)
        if self.is_connected:
            try:
                self.disable_auto_data_transmission()
            except IOError:
                pass  # Ignore errors (e.g., if sensor disconnected)

        # Reset cached data
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
