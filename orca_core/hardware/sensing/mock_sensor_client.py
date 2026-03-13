# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Mock client for simulating tactile sensors in automated tests."""

from dataclasses import dataclass, field
from typing import Optional
import threading
import time
import random
import logging

logger = logging.getLogger(__name__)

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]

# Default taxel counts loaded from sensor model config
def _get_default_taxel_counts() -> dict[str, int]:
    try:
        from orca_core.hardware.sensing.taxel_coordinates import get_taxel_counts
        return get_taxel_counts()
    except Exception:
        return {"thumb": 51, "index": 87, "middle": 87, "ring": 87, "pinky": 51}

DEFAULT_TAXEL_COUNTS = _get_default_taxel_counts()


class NoSensorsAvailableError(Exception):
    """Raised when no sensors are available for communication."""
    pass


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
    consecutive_errors: int = 0
    reconfiguration_count: int = 0


@dataclass
class SensorConfiguration:
    """Snapshot of connected sensors and their properties."""
    connected: dict[str, bool] = field(default_factory=dict)
    num_taxels: dict[str, int] = field(default_factory=dict)
    module_indices: dict[str, int] = field(default_factory=dict)
    expected_payload_size_resultant: int = 0
    expected_payload_size_taxels: int = 0
    expected_payload_size_combined: int = 0
    timestamp: float = 0.0
    finger_to_sensor_id: dict[str, int] = field(default_factory=lambda: {
        "thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4
    })

    @property
    def active_sensors(self) -> list[str]:
        """List of currently connected sensors sorted by hardware slot order."""
        active = [f for f in FINGER_NAMES if self.connected.get(f, False)]
        active.sort(key=lambda f: self.finger_to_sensor_id.get(f, FINGER_NAMES.index(f)))
        return active

    @property
    def num_active_sensors(self) -> int:
        """Number of currently connected sensors."""
        return len(self.active_sensors)

    def __str__(self) -> str:
        active = ", ".join(self.active_sensors) if self.active_sensors else "none"
        return f"SensorConfig({self.num_active_sensors} active: {active})"


class MockSensorClient:
    """Mock client for simulating tactile sensor communication in tests.

    This class provides the same interface as SensorClient but returns
    simulated data instead of communicating with real hardware. Useful for:
    - Automated testing without hardware
    - Development and debugging
    - CI/CD pipelines

    The simulated data can be controlled via:
    - set_mock_forces(): Set specific force values to return
    - set_mock_taxels(): Set specific taxel values to return
    - set_connected_sensors(): Configure which sensors appear connected
    - set_noise_level(): Add random noise to simulated data
    """

    def __init__(self,
                 port: str = '/dev/ttyUSB0',
                 baudrate: int = 921600,
                 connected_sensors: Optional[list[str]] = None,
                 finger_to_sensor_id: Optional[dict[str, int]] = None):
        """Initialize mock sensor client.

        Args:
            port: Serial port (ignored, for API compatibility)
            baudrate: Baudrate (ignored, for API compatibility)
            connected_sensors: List of sensor names to simulate as connected.
                             Defaults to ["thumb", "index", "middle"]
            finger_to_sensor_id: Finger-to-sensor-id mapping (ignored, for API compatibility)
        """
        self.port = port
        self.baudrate = baudrate
        self._connected = False

        # Configure which sensors appear connected
        if connected_sensors is None:
            connected_sensors = ["thumb", "index", "middle"]
        self._simulated_connected = {f: f in connected_sensors for f in FINGER_NAMES}
        self._simulated_taxel_counts = {
            f: DEFAULT_TAXEL_COUNTS[f] if self._simulated_connected[f] else 0
            for f in FINGER_NAMES
        }

        # Mock data storage
        self._mock_forces: dict[str, list[float]] = {}
        self._mock_taxels: dict[str, list[list[float]]] = {}
        self._noise_level = 0.0
        self._hardware_version = "MOCK_V1.0.0"

        # Auto-stream state
        self._sensor_config: Optional[SensorConfiguration] = None
        self._auto_thread: Optional[threading.Thread] = None
        self._auto_running = threading.Event()
        self._auto_lock = threading.Lock()
        self._auto_latest = None
        self._auto_latest_taxels = None
        self._auto_latest_ts = None
        self._auto_stats = AutoStreamStats()
        self._auto_mode_resultant = True
        self._auto_mode_taxels = False
        self._auto_rate_hz = 100  # Simulated update rate

        # Initialize default mock data
        self._initialize_mock_data()

    def _initialize_mock_data(self):
        """Initialize default mock force and taxel data."""
        for finger in FINGER_NAMES:
            if self._simulated_connected[finger]:
                self._mock_forces[finger] = [0.0, 0.0, 0.0]
                taxel_count = self._simulated_taxel_counts[finger]
                self._mock_taxels[finger] = [[0.0, 0.0, 0.0] for _ in range(taxel_count)]

    # =========================================================================
    # Mock Control Methods (for test setup)
    # =========================================================================

    def set_connected_sensors(self, sensors: list[str]):
        """Configure which sensors appear as connected.

        Args:
            sensors: List of finger names to simulate as connected
        """
        self._simulated_connected = {f: f in sensors for f in FINGER_NAMES}
        self._simulated_taxel_counts = {
            f: DEFAULT_TAXEL_COUNTS[f] if self._simulated_connected[f] else 0
            for f in FINGER_NAMES
        }
        self._initialize_mock_data()

        # Update configuration if already connected
        if self._connected:
            self._sensor_config = self._get_configuration()

    def set_mock_forces(self, forces: dict[str, list[float]]):
        """Set the force values to return for each sensor.

        Args:
            forces: Dict mapping finger names to [fx, fy, fz] values
        """
        for finger, force in forces.items():
            if finger in FINGER_NAMES and len(force) == 3:
                self._mock_forces[finger] = list(force)

    def set_mock_taxels(self, taxels: dict[str, list[list[float]]]):
        """Set the taxel values to return for each sensor.

        Args:
            taxels: Dict mapping finger names to list of [fx, fy, fz] per taxel
        """
        for finger, data in taxels.items():
            if finger in FINGER_NAMES:
                self._mock_taxels[finger] = [list(t) for t in data]

    def set_noise_level(self, level: float):
        """Set random noise level to add to returned data.

        Args:
            level: Standard deviation of Gaussian noise to add (in Newtons)
        """
        self._noise_level = level

    def set_hardware_version(self, version: str):
        """Set the hardware version string to return.

        Args:
            version: Version string to return from read_hardware_version()
        """
        self._hardware_version = version

    def set_auto_rate(self, rate_hz: float):
        """Set the simulated auto-stream update rate.

        Args:
            rate_hz: Updates per second for auto-stream simulation
        """
        self._auto_rate_hz = rate_hz

    # =========================================================================
    # Connection Methods
    # =========================================================================

    @property
    def is_connected(self) -> bool:
        """Check if client is connected."""
        return self._connected

    def connect(self):
        """Simulate connecting to sensor device."""
        if self.is_connected:
            return

        self._connected = True
        logger.info(f"[MOCK] Connected to sensor at {self.port}")

        # Build initial configuration
        self._sensor_config = self._get_configuration()
        logger.info(f"[MOCK] Initial configuration: {self._sensor_config}")

    def disconnect(self):
        """Simulate disconnecting from sensor device."""
        if not self.is_connected:
            return

        self.stop_auto_stream()
        self._connected = False
        logger.info("[MOCK] Disconnected from sensor")

    # =========================================================================
    # Sensor Information Methods
    # =========================================================================

    def read_hardware_version(self) -> str:
        """Return mock hardware version."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        return self._hardware_version

    def read_connected_sensors(self) -> dict[str, bool]:
        """Return simulated connected sensor status."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        return dict(self._simulated_connected)

    def read_num_taxels(self) -> dict[str, int]:
        """Return simulated taxel counts."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        return dict(self._simulated_taxel_counts)

    def read_auto_data_type(self) -> dict:
        """Return simulated auto data type configuration."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        val = (0x01 if self._auto_mode_resultant else 0) | (0x02 if self._auto_mode_taxels else 0)
        return {
            "raw": f"{val:08b}",
            "resulting_force": self._auto_mode_resultant,
            "individual_taxels_force": self._auto_mode_taxels,
        }

    def get_sensor_configuration(self) -> Optional[SensorConfiguration]:
        """Get the current sensor configuration snapshot."""
        return self._sensor_config

    def _get_configuration(self) -> SensorConfiguration:
        """Build configuration from current simulated state."""
        connected = dict(self._simulated_connected)
        num_taxels = dict(self._simulated_taxel_counts)

        module_indices = {}
        for i, finger in enumerate(FINGER_NAMES):
            if connected.get(finger, False):
                module_indices[finger] = i * 4 + 2

        num_active = sum(1 for c in connected.values() if c)
        expected_resultant = num_active * 6
        expected_taxels = sum(
            num_taxels.get(finger, 0) * 3
            for finger, is_connected in connected.items()
            if is_connected
        )
        expected_combined = expected_resultant + expected_taxels

        return SensorConfiguration(
            connected=connected,
            num_taxels=num_taxels,
            module_indices=module_indices,
            expected_payload_size_resultant=expected_resultant,
            expected_payload_size_taxels=expected_taxels,
            expected_payload_size_combined=expected_combined,
            timestamp=time.time()
        )

    # =========================================================================
    # Force Reading Methods
    # =========================================================================

    def _add_noise(self, value: float) -> float:
        """Add Gaussian noise to a value."""
        if self._noise_level > 0:
            return value + random.gauss(0, self._noise_level)
        return value

    def _get_mock_forces(self) -> dict[str, list[float]]:
        """Get mock forces with optional noise."""
        result = {}
        for finger in FINGER_NAMES:
            if self._simulated_connected.get(finger, False):
                base = self._mock_forces.get(finger, [0.0, 0.0, 0.0])
                result[finger] = [
                    round(self._add_noise(base[0]), 1),
                    round(self._add_noise(base[1]), 1),
                    round(self._add_noise(base[2]), 1),
                ]
        return result

    def _get_mock_taxels(self) -> dict[str, list[list[float]]]:
        """Get mock taxels with optional noise."""
        result = {}
        for finger in FINGER_NAMES:
            if self._simulated_connected.get(finger, False):
                base_taxels = self._mock_taxels.get(finger, [])
                result[finger] = [
                    [
                        round(self._add_noise(t[0]), 2),
                        round(self._add_noise(t[1]), 2),
                        round(self._add_noise(t[2]), 2),
                    ]
                    for t in base_taxels
                ]
        return result

    def read_resulting_force(self) -> dict[str, list[float]]:
        """Return simulated resultant forces."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        return self._get_mock_forces()

    # =========================================================================
    # Auto-Stream Control Methods
    # =========================================================================

    def set_auto_data_type(self, resultant: bool = True, taxels: bool = False) -> None:
        """Configure data types for auto stream."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        self._auto_mode_resultant = resultant
        self._auto_mode_taxels = taxels

    def enable_auto_data_transmission(self) -> None:
        """Enable auto data transmission (no-op in mock)."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")

    def disable_auto_data_transmission(self) -> None:
        """Disable auto data transmission (no-op in mock)."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")

    def reboot(self) -> None:
        """Simulate sensor reboot."""
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        logger.info("[MOCK] Sensor reboot simulated")

    # =========================================================================
    # Auto-Stream Methods
    # =========================================================================

    def _auto_reader_loop(self, parse_resultant: bool, parse_taxels: bool):
        """Background thread that simulates auto-stream data generation."""
        interval = 1.0 / self._auto_rate_hz

        while self._auto_running.is_set():
            try:
                parsed_resultant = None
                parsed_taxels = None

                if parse_resultant:
                    parsed_resultant = self._get_mock_forces()
                if parse_taxels:
                    parsed_taxels = self._get_mock_taxels()

                with self._auto_lock:
                    if parse_resultant:
                        self._auto_latest = parsed_resultant
                    if parse_taxels:
                        self._auto_latest_taxels = parsed_taxels
                    self._auto_latest_ts = time.time()
                    self._auto_stats.frames_ok += 1
                    self._auto_stats.parse_ok += 1

                time.sleep(interval)

            except Exception as e:
                logger.error(f"[MOCK] Error in auto reader: {e}")
                with self._auto_lock:
                    self._auto_stats.parse_errors += 1
                time.sleep(interval)

        logger.info("[MOCK] Auto reader loop exited")

    def start_auto_stream(self, resultant: bool = True, taxels: bool = False, min_sensors: int = 1):
        """Start simulated auto-stream mode.

        Args:
            resultant: Include resultant force data
            taxels: Include taxel data
            min_sensors: Minimum sensors required

        Raises:
            OSError: If not connected
            NoSensorsAvailableError: If fewer than min_sensors available
            ValueError: If neither resultant nor taxels enabled
        """
        if not self.is_connected:
            raise OSError("Must call connect() first.")

        if not resultant and not taxels:
            raise ValueError("At least one of resultant or taxels must be enabled")

        self.stop_auto_stream()

        self._auto_mode_resultant = resultant
        self._auto_mode_taxels = taxels

        self._sensor_config = self._get_configuration()

        if self._sensor_config.num_active_sensors < min_sensors:
            raise NoSensorsAvailableError(
                f"Only {self._sensor_config.num_active_sensors} sensor(s) available, "
                f"need at least {min_sensors}"
            )

        mode_str = []
        if resultant:
            mode_str.append("resultant")
        if taxels:
            mode_str.append("taxels")
        logger.info(
            f"[MOCK] Starting auto-stream with {self._sensor_config}, "
            f"mode={'+'.join(mode_str)}"
        )

        self._auto_running.set()
        self._auto_thread = threading.Thread(
            target=self._auto_reader_loop,
            args=(resultant, taxels),
            daemon=True
        )
        self._auto_thread.start()

    def stop_auto_stream(self):
        """Stop simulated auto-stream mode."""
        self._auto_running.clear()

        if self._auto_thread is not None:
            self._auto_thread.join(timeout=1.0)
            self._auto_thread = None

        with self._auto_lock:
            self._auto_latest = None
            self._auto_latest_taxels = None
            self._auto_latest_ts = None

    def get_auto_latest(self):
        """Get latest simulated resultant force data."""
        with self._auto_lock:
            return self._auto_latest, self._auto_latest_ts

    def get_auto_latest_taxels(self):
        """Get latest simulated taxel data."""
        with self._auto_lock:
            return self._auto_latest_taxels, self._auto_latest_ts

    def get_auto_latest_all(self):
        """Get all latest simulated data."""
        with self._auto_lock:
            return self._auto_latest, self._auto_latest_taxels, self._auto_latest_ts

    def get_auto_stats(self):
        """Get auto-stream statistics."""
        with self._auto_lock:
            return self._auto_stats

    # =========================================================================
    # Context Manager Support
    # =========================================================================

    def __enter__(self):
        """Enable use as context manager."""
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, *args):
        """Enable use as context manager."""
        self.disconnect()

    def __del__(self):
        """Cleanup on destruction."""
        try:
            self.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    # Simple test of mock client
    logging.basicConfig(level=logging.INFO)

    print("=== Mock Sensor Client Test ===\n")

    with MockSensorClient(connected_sensors=["thumb", "index", "middle"]) as client:
        print(f"Hardware version: {client.read_hardware_version()}")
        print(f"Connected sensors: {client.read_connected_sensors()}")
        print(f"Taxel counts: {client.read_num_taxels()}")
        print(f"Configuration: {client.get_sensor_configuration()}")

        # Test request-response mode
        print("\n--- Request-Response Mode ---")
        forces = client.read_resulting_force()
        print(f"Forces: {forces}")

        # Set specific mock values
        client.set_mock_forces({
            "thumb": [1.0, 0.5, 2.0],
            "index": [0.0, 0.0, 1.5],
            "middle": [-0.5, 0.2, 0.8],
        })
        forces = client.read_resulting_force()
        print(f"Forces (with mock values): {forces}")

        # Test with noise
        client.set_noise_level(0.1)
        forces = client.read_resulting_force()
        print(f"Forces (with noise): {forces}")
        client.set_noise_level(0.0)

        # Test auto-stream mode
        print("\n--- Auto-Stream Mode (resultant only) ---")
        client.start_auto_stream(resultant=True, taxels=False)
        time.sleep(0.1)
        for _ in range(5):
            forces, ts = client.get_auto_latest()
            if forces:
                print(f"[{ts:.3f}] {forces}")
            time.sleep(0.05)
        client.stop_auto_stream()

        # Test combined mode
        print("\n--- Auto-Stream Mode (combined) ---")
        client.start_auto_stream(resultant=True, taxels=True)
        time.sleep(0.1)
        forces, taxels, ts = client.get_auto_latest_all()
        if forces:
            print(f"Forces: {forces}")
            print(f"Taxel counts: {({f: len(t) for f, t in taxels.items()}) if taxels else 'None'}")
        client.stop_auto_stream()

        # Show stats
        stats = client.get_auto_stats()
        print(f"\nStats: frames_ok={stats.frames_ok}, parse_ok={stats.parse_ok}")

    print("\n=== Test Complete ===")
