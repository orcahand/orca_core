# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Mock client for simulating tactile sensors in automated tests."""

from __future__ import annotations

from collections.abc import Callable
import threading
import time
import logging

from orca_core.hardware.sensor_client import SensorClient, NoSensorsAvailableError
from orca_core.hardware.sensing.constants import (
    FINGER_NAMES,
    DEFAULT_TAXEL_COUNTS,
    DEFAULT_SENSOR_BAUDRATE,
    AUTO_DATA_RESULTANT,
    AUTO_DATA_TAXELS,
)
from orca_core.hardware.sensing.protocol import (
    ForceVector,
    ResultantForces,
    TaxelForces,
)

logger = logging.getLogger(__name__)

ResultantProvider = Callable[[], ResultantForces]
TaxelProvider = Callable[[], TaxelForces]


class MockSensorClient(SensorClient):
    """Mock client for simulating tactile sensor communication in tests.

    Subclasses SensorClient, replacing hardware I/O with deterministic data
    sources. Inherits start_auto_stream, stop_auto_stream, offset logic, and
    context manager support from the base class.

    Control simulated data via:
    - set_mock_forces(): Set specific force values to return
    - set_mock_taxels(): Set specific taxel values to return
    - set_connected_sensors(): Configure which sensors appear connected
    - set_resultant_provider()/set_taxel_provider(): Inject deterministic generators

    Default behavior (if no providers or mock values are set):
    - All force components return 1.0 for every connected sensor/taxel.
    """

    def __init__(
        self,
        port: str = "mock",
        baudrate: int = DEFAULT_SENSOR_BAUDRATE,
        connected_sensors: list[str] | None = None,
        taxel_counts: dict[str, int] | None = None,
        finger_to_sensor_id: dict[str, int] | None = None,
        resultant_provider: ResultantProvider | None = None,
        taxel_provider: TaxelProvider | None = None,
        auto_rate_hz: float | None = None,
    ):
        super().__init__(port=port, baudrate=baudrate, finger_to_sensor_id=finger_to_sensor_id)

        if connected_sensors is None:
            connected_sensors = list(FINGER_NAMES)

        self._taxel_counts_per_finger: dict[str, int] = (
            dict(taxel_counts) if taxel_counts is not None else dict(DEFAULT_TAXEL_COUNTS)
        )

        self._sim_connected: dict[str, bool] = {
            f: f in connected_sensors for f in FINGER_NAMES
        }
        self._sim_taxel_counts: dict[str, int] = {
            f: self._taxel_counts_per_finger[f] if f in connected_sensors else 0
            for f in FINGER_NAMES
        }

        self._mock_forces: ResultantForces = {}
        self._mock_taxels: TaxelForces = {}

        self._resultant_provider: ResultantProvider = (
            resultant_provider if resultant_provider is not None else self._default_resultant_provider
        )
        self._taxel_provider: TaxelProvider = (
            taxel_provider if taxel_provider is not None else self._default_taxel_provider
        )

        # None = no sleep between frames (ideal for tests). Pass e.g. 1000
        # to throttle to ~1kHz for demos or UI prototyping.
        self._auto_rate_hz = auto_rate_hz

        # Set by _acquire_frame after the first frame is produced. Lets tests
        # synchronize on stream start without polling/sleeping.
        self._first_frame_event = threading.Event()

    # =========================================================================
    # Mock Control Methods
    # =========================================================================

    def set_connected_sensors(self, sensors: list[str]) -> None:
        """Configure which sensors appear as connected.

        Only clears mock data for sensors that were removed.
        """
        removed = {f for f, on in self._sim_connected.items() if on and f not in sensors}
        self._update_connectivity(sensors)
        for f in removed:
            self._mock_forces.pop(f, None)
            self._mock_taxels.pop(f, None)

    def simulate_dropout(self, dropped: list[str]) -> None:
        """Simulate one or more sensors dropping out."""
        remaining = [f for f in self._sim_connected if self._sim_connected[f] and f not in dropped]
        self.set_connected_sensors(remaining)

    def set_mock_forces(self, forces: ResultantForces) -> None:
        """Set the force values to return for each sensor.

        Replaces all previously set mock forces.

        Raises:
            ValueError: If any finger name is not valid or force vector is wrong length
        """
        self._validate_finger_vectors(forces, expected_len=3, label="Force")
        self._mock_forces = {f: list(v) for f, v in forces.items()}

    def set_mock_taxels(self, taxels: TaxelForces) -> None:
        """Set the taxel values to return for each sensor.

        Replaces all previously set mock taxels.

        Raises:
            ValueError: If any finger name is not valid or any taxel vector is wrong length
        """
        for finger, taxel_list in taxels.items():
            for taxel in taxel_list:
                self._validate_finger_vectors({finger: taxel}, expected_len=3, label="Taxel")
        self._mock_taxels = {f: [list(t) for t in data] for f, data in taxels.items()}

    def set_resultant_provider(self, provider: ResultantProvider) -> None:
        """Inject a deterministic resultant provider (called per read)."""
        self._resultant_provider = provider

    def set_taxel_provider(self, provider: TaxelProvider) -> None:
        """Inject a deterministic taxel provider (called per read)."""
        self._taxel_provider = provider

    # =========================================================================
    # Connection (no-op hardware I/O)
    # =========================================================================

    def connect(self) -> None:
        if self.is_connected:
            return
        self._connected = True
        self._sensor_config = self._get_configuration()
        logger.info(f"[MOCK] Connected, config: {self._sensor_config}")

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        # stop_auto_stream must precede _connected = False: the base class
        # stop_auto_stream calls disable_auto_data_transmission which checks
        # is_connected via _write_register.
        self.stop_auto_stream()
        self._connected = False
        logger.info("[MOCK] Disconnected")

    # =========================================================================
    # Sensor Information (return simulated state)
    # =========================================================================

    def read_connected_sensors(self) -> dict[str, bool]:
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        return dict(self._sim_connected)

    def read_num_taxels(self) -> dict[str, int]:
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        return dict(self._sim_taxel_counts)

    def read_auto_data_type(self) -> dict:
        if not self.is_connected:
            raise OSError("Must call connect() first.")
        val = (AUTO_DATA_RESULTANT if self._auto_mode_resultant else 0) | (
            AUTO_DATA_TAXELS if self._auto_mode_taxels else 0
        )
        return {
            "raw": f"{val:08b}",
            "resultant": self._auto_mode_resultant,
            "taxels": self._auto_mode_taxels,
        }

    # =========================================================================
    # Hardware I/O overrides (no-ops)
    # =========================================================================

    def _write_register(self, address: int, data: bytes, response_timeout_s: float = 0.5) -> None:
        pass

# =========================================================================
    # Force Reading
    # =========================================================================

    def _read_raw_resultant(self) -> ResultantForces:
        return self._resultant_provider()

    # =========================================================================
    # Frame Acquisition (overrides base class serial reader)
    # =========================================================================

    def start_auto_stream(self, *args, **kwargs):
        self._first_frame_event.clear()
        super().start_auto_stream(*args, **kwargs)

    def wait_for_first_frame(self, timeout: float = 2.0) -> None:
        """Block until the auto-stream loop has stored its first frame.

        Lets tests synchronize on stream startup without polling or sleeps.
        Raises TimeoutError if no frame arrives in `timeout` seconds.
        """
        if not self._first_frame_event.wait(timeout):
            raise TimeoutError(f"No auto-stream frame within {timeout}s")

    def _on_frame_stored(self) -> None:
        self._first_frame_event.set()

    def _acquire_frame(
        self,
        parse_resultant: bool,
        parse_taxels: bool,
        min_sensors: int,
    ) -> tuple[dict | None, dict | None]:
        """Acquire simulated frame data from mock providers."""
        if self._sensor_config is None or self._sensor_config.num_active_sensors < min_sensors:
            raise NoSensorsAvailableError("Insufficient sensors for auto-stream")

        parsed_resultant = self._resultant_provider() if parse_resultant else None
        parsed_taxels = self._taxel_provider() if parse_taxels else None

        # Rate limiting (None = no sleep, ideal for tests)
        if self._auto_rate_hz:
            time.sleep(1.0 / self._auto_rate_hz)

        return parsed_resultant, parsed_taxels

    # =========================================================================
    # Internal Helpers
    # =========================================================================

    def _update_connectivity(self, sensors: list[str]) -> None:
        """Update simulated connectivity and reconfigure if connected."""
        self._sim_connected = {f: f in sensors for f in FINGER_NAMES}
        self._sim_taxel_counts = {
            f: self._taxel_counts_per_finger[f] if self._sim_connected[f] else 0
            for f in FINGER_NAMES
        }
        if self._connected:
            self._sensor_config = self._get_configuration()

    def _default_resultant_provider(self) -> ResultantForces:
        forces = self._mock_forces
        return {
            f: list(forces[f]) if f in forces else [1.0, 1.0, 1.0]
            for f in FINGER_NAMES
            if self._sim_connected.get(f, False)
        }

    def _default_taxel_provider(self) -> TaxelForces:
        result = {}
        for finger in FINGER_NAMES:
            if not self._sim_connected.get(finger, False):
                continue
            expected = self._sim_taxel_counts.get(finger, 0)
            if finger in self._mock_taxels:
                provided = self._mock_taxels[finger]
                if len(provided) != expected:
                    raise ValueError(
                        f"Mock taxel count for '{finger}' is {len(provided)}, "
                        f"but sensor expects {expected}"
                    )
                result[finger] = [list(t) for t in provided]
            else:
                result[finger] = [[1.0, 1.0, 1.0] for _ in range(expected)]
        return result

    @staticmethod
    def _validate_finger_vectors(
        data: ResultantForces, expected_len: int, label: str
    ) -> None:
        for finger, vec in data.items():
            if finger not in FINGER_NAMES:
                raise ValueError(f"Unknown finger '{finger}'. Valid names: {FINGER_NAMES}")
            if len(vec) != expected_len:
                raise ValueError(
                    f"{label} vector for '{finger}' must have {expected_len} components, "
                    f"got {len(vec)}"
                )
