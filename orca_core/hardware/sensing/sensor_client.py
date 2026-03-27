# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
from dataclasses import dataclass, field
from typing import Optional
import serial
import threading
import time
import logging

# Configure logging
logger = logging.getLogger(__name__)

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]

# Exceptions
class NoSensorsAvailableError(Exception):
    """Raised when no sensors are available for communication."""
    pass


# Protocol constants
PROTOCOL_HEADER_REQUEST = bytes([0x55, 0xAA])
PROTOCOL_HEADER_RESPONSE = bytes([0xAA, 0x55])
PROTOCOL_HEADER_AUTO = bytes([0xAA, 0x56])
PROTOCOL_RESERVED = 0x00
FUNC_CODE_READ = 0x03
FUNC_CODE_WRITE = 0x10

# Register addresses
ADDR_HARDWARE_VERSION_START = 0x0000
ADDR_HARDWARE_VERSION_LENGTH = 16

ADDR_RESET = 0x0022

ADDR_CONNECTED_SENSORS_START = 0x0010  
ADDR_CONNECTED_SENSORS_LENGTH = 4   

ADDR_NUM_TAXELS_START = 0x0030
ADDR_NUM_TAXELS_LENGTH = 56

ADDR_RESULTING_FORCE_START = 0x0500
ADDR_RESULTING_FORCE_LENGTH = 168

ADDR_AUTO_DATA_TYPE = 0x0016
ADDR_AUTO_ENABLE = 0x0017 


def calculate_checksum(frame: bytes) -> int:
    """Calculate checksum for the protocol frame.
    
    Algorithm: LRC
    
    Args:
        frame: The frame bytes to calculate checksum for (excluding checksum byte)
        
    Returns:
        Checksum value (single byte)
    """
    total_sum = sum(frame)
    lower_8_bits = total_sum & 0xFF
    checksum = (0x100 - lower_8_bits) & 0xFF
    
    return checksum

def int_to_little_endian(value: int, num_bytes: int = 2) -> bytes:
    """Convert integer to little-endian bytes.
    
    Args:
        value: Integer value to convert
        num_bytes: Number of bytes to use (default: 2)
        
    Returns:
        Little-endian byte representation
    """
    return value.to_bytes(num_bytes, byteorder='little')

@dataclass
class AutoStreamStats:
    frames_ok: int = 0
    frames_bad_lrc: int = 0
    resyncs: int = 0
    last_error_code: int = 0
    parse_ok: int = 0
    parse_errors: int = 0
    last_eff_len: int = 0
    last_payload_len: int = 0
    consecutive_errors: int = 0  # For error-triggered reconfiguration
    reconfiguration_count: int = 0  # Number of times config was updated


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
    finger_to_sensor_id: dict[str, int] = field(default_factory=lambda: {
        "thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4
    })

    @property
    def active_sensors(self) -> list[str]:
        """List of currently connected sensors sorted by hardware slot order.

        Auto-stream data arrives in slot order, so this must match.
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
                 port: str = '/dev/ttyUSB0',
                 baudrate: int = 921600,
                 finger_to_sensor_id: Optional[dict[str, int]] = None):

        self.port = port
        self.baudrate = baudrate
        self._connected = False
        self._serial_connection: Optional[serial.Serial] = None

        # Finger-to-sensor-id mapping (configurable wiring)
        if finger_to_sensor_id is None:
            self._finger_to_sensor_id = {
                "thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4
            }
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
        self._sensor_config: Optional[SensorConfiguration] = None
        self._last_reconfigure_time: float = 0.0  # Rate limiting for reconfiguration

        self._auto_thread: Optional[threading.Thread] = None
        self._auto_running = threading.Event()  # Thread-safe flag for auto stream
        self._auto_lock = threading.Lock()
        self._auto_latest = None            # parsed resultant forces dict
        self._auto_latest_taxels = None     # parsed taxels dict
        self._auto_latest_ts = None
        self._auto_stats = AutoStreamStats()
        self._auto_mode_resultant = True    # Whether to parse resultant forces
        self._auto_mode_taxels = False      # Whether to parse taxels

        # Per-taxel zeroing offsets
        self._taxel_offsets: Optional[dict] = None      # {finger: [[fx, fy, fz], ...], ...}
        self._resultant_offsets: Optional[dict] = None   # {finger: [fx, fy, fz], ...}

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
            except Exception as e:
                logger.warning(f"Failed to get initial configuration: {e}")
                # Don't fail connection, config will be retrieved when starting auto-stream

        except Exception as e:
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

        # Build request frame: 55 AA | reserved | func(0x03=READ) | addr | count | LRC
        request = (
            PROTOCOL_HEADER_REQUEST  # 55 AA
            + int_to_little_endian(PROTOCOL_RESERVED, 1)  # 0x00
            + int_to_little_endian(FUNC_CODE_READ, 1)  # 0x03
            + int_to_little_endian(address, 2)  # Address (little-endian)
            + int_to_little_endian(count, 2)  # Byte count (little-endian)
        )
        request += bytes([calculate_checksum(request)])

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

        # Parse response: AA 55 | meta(6) | data(count) | LRC(1)
        meta = self._read_exact(6)  # reserved(1) + func(1) + addr(2) + count(2)
        data = self._read_exact(count)
        lrc = self._read_exact(1)

        # Validate checksum
        full = hdr + meta + data + lrc
        if full[-1] != calculate_checksum(full[:-1]):
            raise IOError("Read response LRC mismatch")

        return data

    def _skip_auto_frame(self) -> None:
        """Skip one complete auto-stream frame after having consumed the AA56 header.

        Frame format (after AA56 header):
        - reserved (1 byte): typically 0x00
        - eff_len (2 bytes, little-endian): length of error_code + payload
        - payload (eff_len bytes): error_code(1) + valid_data(eff_len-1)
        - LRC (1 byte): checksum

        This is called when waiting for a request-response (AA55) frame but an
        auto-stream (AA56) frame arrives first. We skip it to continue waiting
        for the AA55 response.

        Raises:
            IOError: If serial read fails
            ValueError: If eff_len is unreasonably large (>8KB)
        """
        _reserved = self._read_exact(1)
        eff_len = int.from_bytes(self._read_exact(2), "little")

        # Sanity check: typical payload is 6-200 bytes, max reasonable is ~8KB
        if eff_len > 8192:
            raise ValueError(f"Invalid eff_len in auto frame: {eff_len} (possible corruption)")

        # Read and discard payload + LRC
        _ = self._read_exact(eff_len + 1)

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

        # Build request frame: 55 AA | reserved | func(0x10=WRITE) | addr | len | data | LRC
        request = (
            PROTOCOL_HEADER_REQUEST  # 55 AA
            + int_to_little_endian(PROTOCOL_RESERVED, 1)  # 0x00
            + int_to_little_endian(FUNC_CODE_WRITE, 1)  # 0x10
            + int_to_little_endian(address, 2)  # Address (little-endian)
            + int_to_little_endian(len(data), 2)  # Data length (little-endian)
            + data  # Payload
        )
        request += bytes([calculate_checksum(request)])

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

        # Parse response: AA 55 | meta(6) | status(1) | LRC(1)
        # Meta fields: reserved(1) + func(1) + addr(2) + nbytes(2)
        fixed_rest = self._read_exact(6)
        fixed = hdr + fixed_rest  # Total 8 bytes

        returned_nbytes = int.from_bytes(fixed[6:8], "little")

        # Read status + LRC
        rest = self._read_exact(returned_nbytes + 1)
        full = fixed + rest

        # Validate checksum
        expected = calculate_checksum(full[:-1])
        if full[-1] != expected:
            raise IOError("Write response LRC mismatch")

        # Check status byte (first byte of response payload)
        if returned_nbytes >= 1:
            status = rest[0]
            if status != 0:
                raise IOError(f"Write failed, status=0x{status:02X}")



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
        if len(data) < 4:
            raise ValueError(f'Expected 4 bytes, got {len(data)}')
        status = [
            bool(data[0] & (1 << 2)),
            bool(data[0] & (1 << 6)),
            bool(data[1] & (1 << 2)),
            bool(data[1] & (1 << 6)),
            bool(data[2] & (1 << 2)),
        ]
        return {self._sensor_id_to_finger[i]: status[i] for i in range(5)}

    def read_hardware_version(self) -> str:
        """Read the hardware version.

        Returns:
            Hardware version string

        Raises:
            OSError: If not connected to sensor
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        version_bytes = self._read_register(ADDR_HARDWARE_VERSION_START, ADDR_HARDWARE_VERSION_LENGTH)
        version_string = version_bytes.decode('ascii', errors='ignore').rstrip('\x00').rstrip()
        
        return version_string

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
        taxel_counts = [int.from_bytes(data[i:i+2], byteorder='little') for i in range(0, len(data), 2)]
        SLOT_REGISTER_OFFSETS = [0x0034, 0x003C, 0x0044, 0x004C, 0x0054]
        distal_indices = {
            self._sensor_id_to_finger[slot]: (addr - ADDR_NUM_TAXELS_START) // 2
            for slot, addr in enumerate(SLOT_REGISTER_OFFSETS)
        }
        return {finger: taxel_counts[idx] for finger, idx in distal_indices.items()}

    def read_auto_data_type(self) -> dict:
        """Read the auto data type register.

        Returns:
            Dictionary with raw binary value and parsed flags

        Raises:
            OSError: If not connected to sensor
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        data = self._read_register(ADDR_AUTO_DATA_TYPE, 1)
        byte_val = data[0]
        return {
            "raw": f"{byte_val:08b}",
            "resulting_force": bool(byte_val & 0x01),
            "individual_taxels_force": bool(byte_val & 0x02),
        }

    def read_resulting_force(self) -> dict[str, list[float]]:
        """Read resulting force from all connected fingertip sensors.

        Uses dynamic parsing based on current sensor configuration. Returns data
        only for sensors that are currently connected (sparse dict format).

        Note: This implementation assumes we only have fingertip (distal phalanx) sensors
        for each finger, not proximal/middle phalanx or palm sensors.

        Returns:
            Dictionary mapping finger names to [fx, fy, fz] force vectors in Newtons
            Only includes sensors that are currently connected

        Raises:
            OSError: If not connected to sensor
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        # Ensure we have current configuration
        if self._sensor_config is None:
            try:
                self._sensor_config = self._get_configuration()
            except Exception as e:
                logger.error(f"Failed to get configuration: {e}")
                # Fall back to static parsing
                data = self._read_register(ADDR_RESULTING_FORCE_START, ADDR_RESULTING_FORCE_LENGTH)
                return self._parse_resultant_force_block(data)

        # Use dynamic parsing based on configuration
        data = self._read_register(ADDR_RESULTING_FORCE_START, ADDR_RESULTING_FORCE_LENGTH)
        result = self._parse_resultant_force_dynamic(data, self._sensor_config)
        if self._resultant_offsets:
            self._apply_resultant_offsets(result)
        return result

    def get_sensor_configuration(self) -> Optional[SensorConfiguration]:
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
                    module_indices[finger] = sensor_id * 4 + 2

            # Calculate expected payload sizes
            num_active = sum(1 for c in connected.values() if c)
            expected_resultant = num_active * 6  # Each sensor: fx(2) + fy(2) + fz(2) = 6 bytes

            # Calculate taxel payload size: sum of taxels for active sensors * 3 bytes per taxel
            # Each taxel sends 3 bytes: fx(1) + fy(1) + fz(1) as int8 values
            expected_taxels = sum(
                num_taxels.get(finger, 0) * 3
                for finger, is_connected in connected.items()
                if is_connected
            )

            # Combined mode: resultant forces followed by taxels
            expected_combined = expected_resultant + expected_taxels

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

        except Exception as e:
            logger.error(f"Failed to get sensor configuration: {e}")
            raise IOError(f"Failed to read sensor configuration: {e}") from e

    def _reconfigure(self, force: bool = False) -> bool:
        """Attempt to reconfigure sensor client with current hardware state.

        This is called when errors indicate the sensor configuration may have changed
        (e.g., sensor disconnected or reconnected). It rate-limits reconfiguration
        to avoid thrashing on flaky connections.

        Args:
            force: If True, bypass rate limiting and reconfigure immediately

        Returns:
            True if reconfiguration succeeded and configuration changed, False otherwise

        Raises:
            NoSensorsAvailableError: If no sensors are connected after reconfiguration
        """
        # Rate limiting: don't reconfigure more than once per 2 seconds (unless forced)
        now = time.time()
        if not force and (now - self._last_reconfigure_time) < 2.0:
            logger.debug("Reconfiguration rate-limited, skipping")
            return False

        try:
            logger.info("Attempting reconfiguration...")
            new_config = self._get_configuration()

            # Check if configuration actually changed
            if self._sensor_config is not None:
                old_active = set(self._sensor_config.active_sensors)
                new_active = set(new_config.active_sensors)

                if old_active == new_active:
                    logger.debug("Configuration unchanged, no reconfiguration needed")
                    return False

                # Log configuration changes
                added = new_active - old_active
                removed = old_active - new_active
                if added:
                    logger.info(f"Sensors added: {', '.join(added)}")
                if removed:
                    logger.warning(f"Sensors removed: {', '.join(removed)}")

            # Update configuration
            self._sensor_config = new_config
            self._last_reconfigure_time = now

            with self._auto_lock:
                self._auto_stats.reconfiguration_count += 1
                self._auto_stats.consecutive_errors = 0  # Reset error counter

            # Check if we have any sensors left
            if new_config.num_active_sensors == 0:
                logger.error("No sensors available after reconfiguration")
                raise NoSensorsAvailableError("All sensors disconnected")

            logger.info(f"Reconfiguration successful: {new_config}")
            return True

        except Exception as e:
            logger.error(f"Reconfiguration failed: {e}")
            raise

    def _parse_auto_stream_compact(self, data: bytes, config: SensorConfiguration) -> dict[str, list[float]]:
        """Parse auto-stream compact format (active sensors only, sequential).

        Auto-stream mode sends only data for connected sensors in sequential order.
        Each sensor is 6 bytes: fx(2) + fy(2) + fz(2).

        For example, if only thumb and middle are connected:
        - Bytes 0-5: thumb (fx, fy, fz)
        - Bytes 6-11: middle (fx, fy, fz)

        Args:
            data: Raw byte data from auto-stream (6 * num_active_sensors bytes)
            config: Current sensor configuration

        Returns:
            Dictionary mapping active finger names to [fx, fy, fz] force vectors
            Uses sparse dict format (only active sensors included)

        Raises:
            ValueError: If data size doesn't match expected size
        """
        RESOLUTION_N_PER_LSB = 0.1
        BYTES_PER_SENSOR = 6

        expected_size = config.num_active_sensors * BYTES_PER_SENSOR
        if len(data) != expected_size:
            raise ValueError(
                f"Auto-stream compact data size mismatch: "
                f"expected {expected_size} bytes ({config.num_active_sensors} sensors), "
                f"got {len(data)} bytes"
            )

        result = {}
        for i, finger in enumerate(config.active_sensors):
            offset = i * BYTES_PER_SENSOR

            # Parse force data sequentially
            fx = int.from_bytes(data[offset:offset+2], byteorder='little', signed=True) * RESOLUTION_N_PER_LSB
            fy = int.from_bytes(data[offset+2:offset+4], byteorder='little', signed=True) * RESOLUTION_N_PER_LSB
            fz = int.from_bytes(data[offset+4:offset+6], byteorder='little', signed=False) * RESOLUTION_N_PER_LSB

            result[finger] = [round(float(fx), 1), round(float(fy), 1), round(float(fz), 1)]

        return result

    def _parse_taxels_compact(self, data: bytes, config: SensorConfiguration) -> dict[str, list[list[float]]]:
        """Parse auto-stream taxels-only format (active sensors only, sequential).

        Auto-stream taxels mode sends only taxel data for connected sensors in sequential order.
        Each taxel is 3 bytes: fx(int8) + fy(int8) + fz(int8).

        For example, if thumb (127 taxels) and index (52 taxels) are connected:
        - Bytes 0-380: thumb taxels (127 * 3 = 381 bytes)
        - Bytes 381-536: index taxels (52 * 3 = 156 bytes)

        Args:
            data: Raw byte data from auto-stream
            config: Current sensor configuration

        Returns:
            Dictionary mapping active finger names to list of taxel force vectors [fx, fy, fz]

        Raises:
            ValueError: If data size doesn't match expected size
        """
        RESOLUTION_N_PER_LSB = 0.1
        BYTES_PER_TAXEL = 3

        expected_size = config.expected_payload_size_taxels
        if len(data) != expected_size:
            raise ValueError(
                f"Auto-stream taxels data size mismatch: "
                f"expected {expected_size} bytes, got {len(data)} bytes"
            )

        result = {}
        offset = 0
        for finger in config.active_sensors:
            taxel_count = config.num_taxels.get(finger, 0)
            taxels = []
            for _ in range(taxel_count):
                # Each taxel: fx(int8), fy(int8), fz(uint8)
                fx = int.from_bytes(data[offset:offset+1], byteorder='little', signed=True) * RESOLUTION_N_PER_LSB
                fy = int.from_bytes(data[offset+1:offset+2], byteorder='little', signed=True) * RESOLUTION_N_PER_LSB
                fz = int.from_bytes(data[offset+2:offset+3], byteorder='little', signed=False) * RESOLUTION_N_PER_LSB
                taxels.append([round(fx, 2), round(fy, 2), round(fz, 2)])
                offset += BYTES_PER_TAXEL
            result[finger] = taxels

        return result

    def _parse_combined_compact(
        self, data: bytes, config: SensorConfiguration
    ) -> tuple[dict[str, list[float]], dict[str, list[list[float]]]]:
        """Parse auto-stream combined format (resultant + taxels for active sensors).

        Combined mode sends data interleaved per sensor:
        [sensor1_resultant][sensor1_taxels][sensor2_resultant][sensor2_taxels]...

        Args:
            data: Raw byte data from auto-stream
            config: Current sensor configuration

        Returns:
            Tuple of (resultant_forces, taxels):
            - resultant_forces: Dict mapping finger names to [fx, fy, fz]
            - taxels: Dict mapping finger names to list of taxel values

        Raises:
            ValueError: If data size doesn't match expected size
        """
        BYTES_PER_RESULTANT = 6
        BYTES_PER_TAXEL = 3
        RESOLUTION_RESULTANT = 0.1
        RESOLUTION_TAXEL = 0.1

        expected_size = config.expected_payload_size_combined
        if len(data) != expected_size:
            raise ValueError(
                f"Auto-stream combined data size mismatch: "
                f"expected {expected_size} bytes, got {len(data)} bytes"
            )

        offset = 0
        resultant_forces = {}
        taxels = {}

        for finger in config.active_sensors:
            # Parse resultant (6 bytes: fx:int16, fy:int16, fz:uint16)
            fx = int.from_bytes(data[offset:offset+2], byteorder='little', signed=True) * RESOLUTION_RESULTANT
            fy = int.from_bytes(data[offset+2:offset+4], byteorder='little', signed=True) * RESOLUTION_RESULTANT
            fz = int.from_bytes(data[offset+4:offset+6], byteorder='little', signed=False) * RESOLUTION_RESULTANT
            resultant_forces[finger] = [round(fx, 1), round(fy, 1), round(fz, 1)]
            offset += BYTES_PER_RESULTANT

            # Parse taxels (taxel_count × 3 bytes: fx:int8, fy:int8, fz:uint8)
            taxel_count = config.num_taxels.get(finger, 0)
            finger_taxels = []
            for _ in range(taxel_count):
                tfx = int.from_bytes(data[offset:offset+1], byteorder='little', signed=True) * RESOLUTION_TAXEL
                tfy = int.from_bytes(data[offset+1:offset+2], byteorder='little', signed=True) * RESOLUTION_TAXEL
                tfz = int.from_bytes(data[offset+2:offset+3], byteorder='little', signed=False) * RESOLUTION_TAXEL
                finger_taxels.append([round(tfx, 2), round(tfy, 2), round(tfz, 2)])
                offset += BYTES_PER_TAXEL
            taxels[finger] = finger_taxels

        return resultant_forces, taxels

    def _parse_resultant_force_dynamic(self, data: bytes, config: SensorConfiguration) -> dict[str, list[float]]:
        """Parse resultant force data using dynamic configuration (offset-based).

        This parser is used for request-response mode where the full 168-byte block
        is returned with all 28 modules. It adapts to the actual connected sensors,
        returning data only for available sensors. Uses sparse dict format.

        Args:
            data: Raw byte data from sensor (168 bytes for full block)
            config: Current sensor configuration

        Returns:
            Dictionary mapping active finger names to [fx, fy, fz] force vectors
            Only includes sensors that are currently connected

        Raises:
            ValueError: If data is too short for expected configuration
        """
        RESOLUTION_N_PER_LSB = 0.1

        if len(data) < ADDR_RESULTING_FORCE_LENGTH:
            raise ValueError(f"Resultant force block too short: {len(data)} bytes")

        result = {}
        for finger in config.active_sensors:
            # Get module index for this sensor from configuration
            module_idx = config.module_indices[finger]
            offset = module_idx * 6  # Each module force is 6 bytes (fx, fy, fz)

            # Parse force data from fixed offsets in full block
            fx = int.from_bytes(data[offset:offset+2], byteorder='little', signed=True) * RESOLUTION_N_PER_LSB
            fy = int.from_bytes(data[offset+2:offset+4], byteorder='little', signed=True) * RESOLUTION_N_PER_LSB
            fz = int.from_bytes(data[offset+4:offset+6], byteorder='little', signed=False) * RESOLUTION_N_PER_LSB

            result[finger] = [round(float(fx), 1), round(float(fy), 1), round(float(fz), 1)]

        return result


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

        val = (0x01 if resultant else 0) | (0x02 if taxels else 0)
        self._write_register(ADDR_AUTO_DATA_TYPE, bytes([val]))


    def enable_auto_data_transmission(self) -> None:
        """Enable automatic data transmission mode.

        Raises:
            OSError: If not connected to sensor
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        self._write_register(ADDR_AUTO_ENABLE, bytes([0x01]))

    def disable_auto_data_transmission(self) -> None:
        """Disable automatic data transmission mode.

        Raises:
            OSError: If not connected to sensor
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        self._write_register(ADDR_AUTO_ENABLE, bytes([0x00]))


    def reboot(self) -> None:
        """Reboot the sensor.

        Raises:
            OSError: If not connected to sensor
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        self._write_register(ADDR_RESET, bytes([0x01]))

    def get_auto_latest(self):
        """Get the most recently parsed auto-stream resultant force data (thread-safe).

        Returns a snapshot of the latest force data received from the auto-stream.
        This is updated by the background thread at ~1kHz when auto streaming is active.

        Returns:
            Tuple of (parsed_data, timestamp):
            - parsed_data: Dictionary mapping finger names to [fx, fy, fz] forces (Newtons)
                          None if no data received yet or resultant mode not enabled
            - timestamp: Unix timestamp (time.time()) when data was received
                        None if no data received yet

        Example:
            >>> client.start_auto_stream(resultant=True)
            >>> time.sleep(0.1)  # Let some data arrive
            >>> forces, ts = client.get_auto_latest()
            >>> print(forces)
            {'index': [0.1, -0.2, 1.5]}
        """
        with self._auto_lock:
            return self._auto_latest, self._auto_latest_ts

    def get_auto_latest_taxels(self):
        """Get the most recently parsed auto-stream taxel data (thread-safe).

        Returns a snapshot of the latest taxel data received from the auto-stream.
        Only available when auto-stream was started with taxels=True.

        Returns:
            Tuple of (parsed_data, timestamp):
            - parsed_data: Dictionary mapping finger names to list of taxel force vectors
                          Each taxel is [fx, fy, fz] in Newtons
                          None if no data received yet or taxel mode not enabled
            - timestamp: Unix timestamp (time.time()) when data was received
                        None if no data received yet

        Example:
            >>> client.start_auto_stream(resultant=False, taxels=True)
            >>> time.sleep(0.1)
            >>> taxels, ts = client.get_auto_latest_taxels()
            >>> print(taxels)
            {'index': [[0.1, -0.2, 0.5], [0.0, 0.1, 0.3], ...]}  # list of [fx, fy, fz] per taxel
        """
        with self._auto_lock:
            return self._auto_latest_taxels, self._auto_latest_ts

    def get_auto_latest_all(self):
        """Get both resultant forces and taxels from latest auto-stream data (thread-safe).

        Returns all available data from the auto-stream. Useful when running in
        combined mode (resultant=True, taxels=True).

        Returns:
            Tuple of (resultant_forces, taxels, timestamp):
            - resultant_forces: Dict mapping finger names to [fx, fy, fz], or None
            - taxels: Dict mapping finger names to taxel value lists, or None
            - timestamp: Unix timestamp when data was received, or None

        Example:
            >>> client.start_auto_stream(resultant=True, taxels=True)
            >>> time.sleep(0.1)
            >>> forces, taxels, ts = client.get_auto_latest_all()
        """
        with self._auto_lock:
            return self._auto_latest, self._auto_latest_taxels, self._auto_latest_ts

    def get_auto_stats(self):
        """Get auto-stream statistics (thread-safe).

        Returns diagnostic information about the auto-stream performance:
        - frames_ok: Number of successfully received frames
        - frames_bad_lrc: Number of frames with checksum errors
        - resyncs: Number of times the reader had to resync after errors
        - parse_ok: Number of successfully parsed frames
        - parse_errors: Number of frames that couldn't be parsed
        - last_error_code: Most recent error code from sensor (0 = no error)

        Returns:
            AutoStreamStats dataclass with statistics

        Example:
            >>> stats = client.get_auto_stats()
            >>> print(f"Success rate: {stats.frames_ok}/{stats.frames_ok + stats.frames_bad_lrc}")
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

    def get_taxel_offsets(self) -> Optional[dict]:
        """Return current per-taxel offsets (for saving to YAML)."""
        return self._taxel_offsets

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
            return offsets
        except Exception:
            # Restore previous offsets on failure
            self._taxel_offsets = prev_taxel
            self._resultant_offsets = prev_resultant
            raise

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
                taxel[0] = round(taxel[0] - off[0], 2)
                taxel[1] = round(taxel[1] - off[1], 2)
                taxel[2] = round(max(0, taxel[2] - off[2]), 2)

    def _apply_resultant_offsets(self, forces: dict) -> None:
        """Subtract resultant offsets in-place. Clamps fz to >= 0."""
        for finger, fvec in forces.items():
            off = self._resultant_offsets.get(finger)
            if not off:
                continue
            fvec[0] = round(fvec[0] - off[0], 1)
            fvec[1] = round(fvec[1] - off[1], 1)
            fvec[2] = round(max(0, fvec[2] - off[2]), 1)

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
            if b1 == bytes([0xAA]) and b2 == bytes([0x56]):
                return  # Found the header
            # Slide window: b2 becomes new b1
            b1 = b2

        # If we exit the loop, auto stream was stopped
        raise IOError("Auto stream stopped during resync")

    def _check_lrc(self, frame_wo_lrc: bytes, lrc_byte: int) -> bool:
        expected = calculate_checksum(frame_wo_lrc)
        return expected == lrc_byte


    def _parse_resultant_force_block(self, data: bytes) -> dict[str, list[float]]:
        """Parse a resultant force data block from the sensor.

        Module index calculation (module_idx = i * 4 + 2):
        - Each finger has 4 potential modules: proximal(0), middle(1), distal(2), nail(3)
        - We only use fingertip sensors, which are the distal phalanx (index 2)
        - For thumb: module_idx = 0*4+2 = 2 (byte offset = 2*6 = 12)
        - For index: module_idx = 1*4+2 = 6 (byte offset = 6*6 = 36)
        - etc.

        Args:
            data: Raw byte data containing resultant forces (168 bytes for 28 modules)

        Returns:
            Dictionary mapping finger names to [fx, fy, fz] force vectors in Newtons
        """
        RESOLUTION_N_PER_LSB = 0.1
        if len(data) < ADDR_RESULTING_FORCE_LENGTH:
            raise ValueError(f"Resultant force block too short: {len(data)} bytes")

        result = {}
        for finger in FINGER_NAMES:
            sensor_id = self._finger_to_sensor_id[finger]
            module_idx = sensor_id * 4 + 2
            offset = module_idx * 6  # Each module force is 6 bytes (fx, fy, fz)

            fx = int.from_bytes(data[offset:offset+2], byteorder='little', signed=True) * RESOLUTION_N_PER_LSB
            fy = int.from_bytes(data[offset+2:offset+4], byteorder='little', signed=True) * RESOLUTION_N_PER_LSB
            fz = int.from_bytes(data[offset+4:offset+6], byteorder='little', signed=False) * RESOLUTION_N_PER_LSB
            result[finger] = [round(float(fx), 1), round(float(fy), 1), round(float(fz), 1)]
        return result

    def _get_expected_payload_size(self, config: SensorConfiguration) -> int:
        """Get expected payload size based on current streaming mode."""
        if self._auto_mode_resultant and self._auto_mode_taxels:
            return config.expected_payload_size_combined
        elif self._auto_mode_resultant:
            return config.expected_payload_size_resultant
        elif self._auto_mode_taxels:
            return config.expected_payload_size_taxels
        return 0

    def _auto_reader_loop(self, parse_resultant: bool, parse_taxels: bool, min_sensors: int):
        """Background thread that continuously reads and parses auto-stream frames.

        Auto-stream frame format (when enabled via 0x0017 = 1):
        - Header: AA 56 (2 bytes)
        - Reserved: 0x00 (1 byte)
        - Effective length: eff_len (2 bytes, little-endian)
        - Error code: (1 byte) - part of eff_len
        - Valid data: (eff_len-1 bytes) - sensor force data
        - LRC: (1 byte) - checksum

        The sensor continuously sends these frames at ~1kHz when auto mode is enabled.
        This thread parses them and updates _auto_latest for the user to read via
        get_auto_latest().

        Supports three modes:
        - Resultant only: 6 bytes per sensor (fx, fy, fz)
        - Taxels only: 2 bytes per taxel for each sensor
        - Combined: Resultant forces followed by taxels

        Features error-triggered reconfiguration: if consecutive errors exceed threshold,
        attempts to reconfigure to adapt to changed sensor configuration.

        Args:
            parse_resultant: Whether to parse resultant force data
            parse_taxels: Whether to parse individual taxel data
            min_sensors: Minimum number of sensors required to continue streaming
        """
        last_print = 0.0
        ERROR_THRESHOLD = 5  # Trigger reconfiguration after 5 consecutive errors

        while self._auto_running.is_set():
            try:
                # ===== Step 1: Find and consume AA 56 header =====
                self._resync_to_auto_header()

                # ===== Step 2: Read frame metadata =====
                reserved = self._read_exact(1)  # Typically 0x00
                eff_len = int.from_bytes(self._read_exact(2), "little")

                # ===== Step 3: Read payload and checksum =====
                # Payload includes: error_code(1) + valid_data(eff_len-1)
                payload = self._read_exact(eff_len)
                lrc = self._read_exact(1)[0]

                # Debug print (throttled to once per second)
                now = time.time()
                if now - last_print > 1.0:
                    config_str = str(self._sensor_config) if self._sensor_config else "no config"
                    logger.debug(f"[auto] eff_len={eff_len}, config={config_str}")
                    last_print = now

                # ===== Step 4: Validate frame integrity =====
                frame_wo_lrc = bytes([0xAA, 0x56]) + reserved + int_to_little_endian(eff_len, 2) + payload
                if not self._check_lrc(frame_wo_lrc, lrc):
                    with self._auto_lock:
                        self._auto_stats.frames_bad_lrc += 1
                        self._auto_stats.consecutive_errors += 1
                    continue  # Skip corrupted frame, resync

                # ===== Step 5: Split error code and valid data =====
                err_code = payload[0]
                valid = payload[1:]  # The actual sensor data

                parsed_resultant = None
                parsed_taxels = None
                parse_success = False

                # ===== Step 6: Check for payload size mismatch (indicates config change) =====
                if self._sensor_config:
                    expected_size = self._get_expected_payload_size(self._sensor_config)
                    if len(valid) != expected_size and expected_size > 0:
                        # Payload size changed! Trigger immediate reconfiguration
                        logger.warning(
                            f"Payload size mismatch: expected {expected_size}, got {len(valid)}. "
                            "Triggering reconfiguration..."
                        )
                        try:
                            if self._reconfigure(force=False):
                                logger.info("Reconfiguration successful, continuing stream")
                            continue  # Skip this frame, let next iteration use new config
                        except NoSensorsAvailableError:
                            logger.error("No sensors available after reconfiguration, stopping stream")
                            self._auto_running.clear()
                            break
                        except Exception as e:
                            logger.error(f"Reconfiguration failed: {e}")
                            # Continue with old config

                # ===== Step 7: Parse payload based on mode =====
                if self._sensor_config:
                    try:
                        expected_size = self._get_expected_payload_size(self._sensor_config)

                        if len(valid) == expected_size and expected_size > 0:
                            # Combined mode: resultant + taxels
                            if parse_resultant and parse_taxels:
                                parsed_resultant, parsed_taxels = self._parse_combined_compact(
                                    valid, self._sensor_config
                                )
                                parse_success = True

                            # Resultant only mode
                            elif parse_resultant:
                                parsed_resultant = self._parse_auto_stream_compact(
                                    valid, self._sensor_config
                                )
                                parse_success = True

                            # Taxels only mode
                            elif parse_taxels:
                                parsed_taxels = self._parse_taxels_compact(
                                    valid, self._sensor_config
                                )
                                parse_success = True

                        else:
                            # Unexpected size
                            logger.warning(
                                f"Unexpected payload: {len(valid)} bytes, expected {expected_size}"
                            )
                            parse_success = False

                    except Exception as e:
                        logger.error(f"Parse error: {e}", exc_info=True)
                        parse_success = False

                # ===== Step 8: Apply zeroing offsets =====
                if parse_success:
                    if self._taxel_offsets and parsed_taxels:
                        self._apply_taxel_offsets(parsed_taxels)
                    if self._resultant_offsets and parsed_resultant:
                        self._apply_resultant_offsets(parsed_resultant)

                # ===== Step 9: Publish latest data and update stats =====
                with self._auto_lock:
                    if parse_success:
                        if parse_resultant:
                            self._auto_latest = parsed_resultant
                        if parse_taxels:
                            self._auto_latest_taxels = parsed_taxels
                        self._auto_latest_ts = time.time()
                        self._auto_stats.consecutive_errors = 0  # Reset on success

                    self._auto_stats.frames_ok += 1
                    self._auto_stats.last_error_code = err_code
                    self._auto_stats.last_eff_len = eff_len
                    self._auto_stats.last_payload_len = len(valid)

                    if parse_success:
                        self._auto_stats.parse_ok += 1
                    else:
                        self._auto_stats.parse_errors += 1
                        self._auto_stats.consecutive_errors += 1

                # ===== Step 10: Check if we should trigger reconfiguration =====
                if self._auto_stats.consecutive_errors >= ERROR_THRESHOLD:
                    logger.warning(
                        f"Consecutive errors ({self._auto_stats.consecutive_errors}) exceeded threshold. "
                        "Attempting reconfiguration..."
                    )
                    try:
                        if self._reconfigure(force=False):
                            logger.info("Reconfiguration successful")

                            # Check if we still meet minimum sensor requirement
                            if self._sensor_config.num_active_sensors < min_sensors:
                                logger.error(
                                    f"Only {self._sensor_config.num_active_sensors} sensor(s) available, "
                                    f"need {min_sensors}. Stopping stream."
                                )
                                self._auto_running.clear()
                                break
                    except NoSensorsAvailableError:
                        logger.error("No sensors available, stopping stream")
                        self._auto_running.clear()
                        break
                    except Exception as e:
                        logger.error(f"Reconfiguration failed: {e}")
                        # Reset error counter to avoid infinite reconfiguration attempts
                        with self._auto_lock:
                            self._auto_stats.consecutive_errors = 0

            except IOError as e:
                if "Auto stream stopped" in str(e):
                    # Normal shutdown, exit gracefully
                    logger.info("Auto stream stopped")
                    break
                # Other IO errors
                logger.warning(f"IO error in auto reader: {e}")
                with self._auto_lock:
                    self._auto_stats.resyncs += 1
                    self._auto_stats.consecutive_errors += 1
                time.sleep(0.01)  # Brief pause before retry

            except Exception as e:
                # Unexpected errors
                logger.error(f"Unexpected error in auto reader: {e}", exc_info=True)
                with self._auto_lock:
                    self._auto_stats.resyncs += 1
                    self._auto_stats.consecutive_errors += 1
                time.sleep(0.01)  # Brief pause before retry

        logger.info("Auto reader loop exited")

    def start_auto_stream(self, resultant: bool = True, taxels: bool = False, min_sensors: int = 1):
        """Start continuous auto-stream mode for real-time force data.

        In auto-stream mode, the sensor continuously broadcasts force data at ~1kHz
        without requiring request-response polling. This provides much lower latency
        and higher throughput than repeatedly calling read_resulting_force().

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
        except Exception as e:
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
        except Exception:
            pass  # If this fails, robust _write_register will handle AA56 frames

        # Configure which data types to include in auto frames
        self.set_auto_data_type(resultant=resultant, taxels=taxels)

        # Clear any stale data from serial buffer before starting
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
            except Exception:
                pass  # Ignore errors (e.g., if sensor disconnected)

        # Reset cached data
        with self._auto_lock:
            self._auto_latest = None
            self._auto_latest_taxels = None
            self._auto_latest_ts = None



if __name__ == "__main__":
    import sys

    sensor_client = SensorClient(port="/dev/ttyACM0", baudrate=921600)
   
    sensor_client.connect()
    print(sensor_client._sensor_config)
    exit()
    try:
        print("version:", sensor_client.read_hardware_version())
        print("connected sensors:", sensor_client.read_connected_sensors())
        print("num taxels:", sensor_client.read_num_taxels())
        print("config:", sensor_client.get_sensor_configuration())

        # Parse command line for mode selection
        mode = sys.argv[1] if len(sys.argv) > 1 else "resultant"

        if mode == "resultant":
            print("\n=== Resultant Force Only Mode ===")
            sensor_client.start_auto_stream(resultant=True, taxels=False)
            for _ in range(100):
                forces, ts = sensor_client.get_auto_latest()
                if forces is not None:
                    print(f"[{ts:.3f}] forces: {forces}")
                time.sleep(0.05)

        elif mode == "taxels":
            print("\n=== Taxels Only Mode ===")
            sensor_client.start_auto_stream(resultant=False, taxels=True)
            for _ in range(100):
                taxels, ts = sensor_client.get_auto_latest_taxels()
                if taxels is not None:
                    # Print summary: count and first taxel [fx, fy, fz] per finger
                    summary = {f: (len(vals), vals[0] if vals else []) for f, vals in taxels.items()}
                    print(f"[{ts:.3f}] taxels (count, first): {summary}")
                time.sleep(0.05)

        elif mode == "combined":
            print("\n=== Combined Mode (Resultant + Taxels) ===")
            sensor_client.start_auto_stream(resultant=True, taxels=True)
            for _ in range(100):
                forces, taxels, ts = sensor_client.get_auto_latest_all()
                if forces is not None:
                    taxel_summary = {f: len(vals) for f, vals in taxels.items()} if taxels else {}
                    print(f"[{ts:.3f}] forces: {forces}, taxel_counts: {taxel_summary}")
                time.sleep(0.05)

        else:
            print(f"Unknown mode: {mode}. Use 'resultant', 'taxels', or 'combined'")

        print("\nstats:", sensor_client.get_auto_stats())

    finally:
        try:
            sensor_client.stop_auto_stream()
        except Exception:
            pass
        sensor_client.disconnect()