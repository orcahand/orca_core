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
import logging

from orca_core.hardware.tactile_client import TactileClient
from orca_core.hardware.sensing.constants import (
    FINGER_NAMES,
    DEFAULT_TAXEL_COUNTS,
    DEFAULT_SENSOR_BAUDRATE,
    AUTO_DATA_RESULTANT,
    AUTO_DATA_TAXELS,
)
from orca_core.hardware.sensing.protocol import (
    ResultantForces,
    TaxelForces,
    decode_combined_auto,
    decode_resultant_auto,
    decode_taxels_auto,
    encode_combined_auto_for_mock,
    encode_resultant_auto_for_mock,
    encode_taxels_auto_for_mock,
)

logger = logging.getLogger(__name__)

ResultantProvider = Callable[[], ResultantForces]
TaxelProvider = Callable[[], TaxelForces]


class MockTactileClient(TactileClient):
    """Mock client for simulating tactile sensor communication in tests.

    Subclasses TactileClient, replacing hardware I/O with deterministic data
    sources. Inherits start_auto_stream, stop_auto_stream, offset logic, and
    context manager support from the base class.

    Control simulated data via:
    - set_mock_forces(): Set specific force values to return
    - set_mock_taxels(): Set specific taxel values to return
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
    ):
        super().__init__(port=port, baudrate=baudrate, finger_to_sensor_id=finger_to_sensor_id)

        if connected_sensors is None:
            connected_sensors = list(FINGER_NAMES)

        full_taxel_counts = dict(taxel_counts) if taxel_counts is not None else dict(DEFAULT_TAXEL_COUNTS)

        self._sim_connected: dict[str, bool] = {
            f: f in connected_sensors for f in FINGER_NAMES
        }
        self._sim_taxel_counts: dict[str, int] = {
            f: full_taxel_counts[f] if f in connected_sensors else 0
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

        # Lets tests synchronize on stream start without polling.
        self._first_frame_event = threading.Event()

    # =========================================================================
    # Mock Control Methods
    # =========================================================================

    def set_mock_forces(self, forces: ResultantForces) -> None:
        """Replace the resultant-force values returned by ``_default_resultant_provider``."""
        self._validate_finger_vectors(forces, expected_len=3, label="Force")
        self._mock_forces = {f: list(v) for f, v in forces.items()}

    def set_mock_taxels(self, taxels: TaxelForces) -> None:
        """Replace the taxel values returned by ``_default_taxel_provider``."""
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
        self._tactile_config = self._get_configuration()
        logger.info(f"[MOCK] Connected, config: {self._tactile_config}")

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
        forces = self._resultant_provider()
        active = self._active_in_slot_order(forces)
        return decode_resultant_auto(
            encode_resultant_auto_for_mock(forces, active), active,
        )

    def _active_in_slot_order(self, forces: dict) -> list[str]:
        """Sort the provider's fingers by hardware slot ID, matching TactileSensorConfiguration."""
        return sorted(
            forces.keys(),
            key=lambda f: self._finger_to_sensor_id.get(f, FINGER_NAMES.index(f)),
        )

    # =========================================================================
    # Frame Acquisition (overrides base class serial reader)
    # =========================================================================

    def start_auto_stream(self, *args, **kwargs):
        self._first_frame_event.clear()
        super().start_auto_stream(*args, **kwargs)

    def wait_for_first_frame(self, timeout: float = 2.0) -> None:
        """Block until the auto-stream loop has stored its first frame, or raise ``TimeoutError``."""
        if not self._first_frame_event.wait(timeout):
            raise TimeoutError(f"No auto-stream frame within {timeout}s")

    def _on_frame_stored(self) -> None:
        self._first_frame_event.set()

    def _acquire_frame(
        self,
        parse_resultant: bool,
        parse_taxels: bool,
    ) -> tuple[dict | None, dict | None]:
        """Acquire simulated frame data, round-tripped through the real wire codec.

        Provider output is encoded into wire bytes and decoded back. This
        ensures mock-driven tests exercise the actual protocol decoders
        instead of bypassing them.
        """
        active = self._tactile_config.active_sensors
        num_taxels = self._tactile_config.num_taxels

        if parse_resultant and parse_taxels:
            forces = self._resultant_provider()
            taxels = self._taxel_provider()
            wire = encode_combined_auto_for_mock(forces, taxels, active)
            parsed_resultant, parsed_taxels = decode_combined_auto(wire, active, num_taxels)
        elif parse_resultant:
            forces = self._resultant_provider()
            wire = encode_resultant_auto_for_mock(forces, active)
            parsed_resultant = decode_resultant_auto(wire, active)
            parsed_taxels = None
        elif parse_taxels:
            taxels = self._taxel_provider()
            wire = encode_taxels_auto_for_mock(taxels, active)
            parsed_resultant = None
            parsed_taxels = decode_taxels_auto(wire, active, num_taxels)
        else:
            parsed_resultant = None
            parsed_taxels = None

        return parsed_resultant, parsed_taxels

    # =========================================================================
    # Internal Helpers
    # =========================================================================

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
