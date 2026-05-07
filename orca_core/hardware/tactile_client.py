# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
import dataclasses
from dataclasses import dataclass, field
import threading
import time
import logging

from orca_core.constants import FINGER_NAMES
from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.sensing.constants import (
    DEFAULT_FINGER_TO_SENSOR_ID,
    PROTOCOL_BYTE_AUTO,
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
    FORCE_ROUND_DECIMALS,
    OFFSET_CLEAR_SETTLE_S,
)
from orca_core.hardware.sensing.types import FingerForces
from orca_core.hardware.sensing.tactile_protocol import (
    build_read_request,
    build_write_request,
    parse_read_response,
    parse_write_response,
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


@dataclass
class AutoStreamStats:
    """Diagnostic counters for the AA 56 handler."""
    frames_ok: int = 0
    frames_bad_payload_size: int = 0
    frames_bad_payload: int = 0
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
        """Connected sensors sorted by hardware slot order, matching wire order."""
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
    """ORCA tactile sensor client over a :class:`HandSerialLink`.

    Subscribes to AA 56 auto-stream frames and exposes the latest reading;
    AA 55 register reads/writes share the same link.
    """

    def __init__(
        self,
        link: HandSerialLink,
        finger_to_sensor_id: dict[str, int] | None = None,
    ):
        self._link = link
        self._connected = False

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

        self._auto_lock = threading.Lock()
        self._auto_running = False
        self._auto_mode_resultant = True
        self._auto_mode_taxels = False
        self._auto_latest = None
        self._auto_latest_taxels = None
        self._auto_latest_ts = None
        self._auto_stats = AutoStreamStats()
        self._first_frame_event = threading.Event()

        # {finger: [[fx, fy, fz], ...], ...} per-taxel zeroing offsets.
        self._taxel_offsets: dict | None = None
        # {finger: [fx, fy, fz], ...} sum of taxel offsets per finger.
        self._resultant_offsets: dict | None = None

    # ----- Lifecycle --------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            return
        self._link.register_frame_handler(PROTOCOL_BYTE_AUTO, self._on_tactile_frame)
        self._connected = True
        try:
            self._tactile_config = self._get_configuration()
            logger.info(f"Tactile client connected, initial configuration: {self._tactile_config}")
        except IOError as e:
            logger.warning(f"Tactile client connected but failed to get initial configuration: {e}")

    def disconnect(self) -> None:
        if not self._connected:
            return
        try:
            self.stop_auto_stream()
        except Exception:
            logger.exception("Error stopping auto-stream during disconnect")
        try:
            self._link.unregister_frame_handler(PROTOCOL_BYTE_AUTO)
        except Exception:
            logger.exception("Error unregistering tactile handler during disconnect")
        self._connected = False

    def __enter__(self):
        if not self._connected:
            self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    # ----- Register I/O -----------------------------------------------------

    def _read_register(self, address: int, count: int, response_timeout_s: float = 0.5) -> bytes:
        if not self._connected:
            raise OSError("Must call connect() first.")
        request = build_read_request(address, count)
        response = self._link.send_register_request(request, response_timeout_s=response_timeout_s)
        return parse_read_response(response)

    def _write_register(self, address: int, data: bytes, response_timeout_s: float = 0.5) -> None:
        if not self._connected:
            raise OSError("Must call connect() first.")
        request = build_write_request(address, data)
        response = self._link.send_register_request(request, response_timeout_s=response_timeout_s)
        parse_write_response(response)

    def read_connected_sensors(self) -> dict[str, bool]:
        """Return ``{finger: is_connected}`` from the connected-sensors register."""
        data = self._read_register(ADDR_CONNECTED_SENSORS_START, ADDR_CONNECTED_SENSORS_LENGTH)
        return decode_connected_sensors(data, self._sensor_id_to_finger)

    def read_num_taxels(self) -> dict[str, int]:
        """Return ``{finger: taxel_count}`` from the taxel-count register block."""
        data = self._read_register(ADDR_NUM_TAXELS_START, ADDR_NUM_TAXELS_LENGTH)
        return decode_num_taxels(data, self._sensor_id_to_finger)

    def read_auto_data_type(self) -> dict:
        """Return the decoded auto-stream data-type register."""
        data = self._read_register(ADDR_AUTO_DATA_TYPE, 1)
        return decode_auto_data_type(data)

    def read_resultant_force(self) -> dict[str, FingerForces]:
        """Read resultant force from all connected fingertip sensors, applying offsets."""
        result = self._read_raw_resultant()
        offsets = self._resultant_offsets
        if offsets:
            self._apply_resultant_offsets(result, offsets)
        return result

    def _read_raw_resultant(self) -> dict[str, FingerForces]:
        """Raise ``IOError`` if no cached config is available and the read fails."""
        if self._tactile_config is None:
            self._tactile_config = self._get_configuration()

        data = self._read_register(ADDR_RESULTANT_FORCE_START, RESULTANT_BLOCK_SIZE)
        return decode_resultant_register(
            data, self._tactile_config.active_sensors, self._tactile_config.module_indices,
        )

    def get_tactile_configuration(self) -> TactileSensorConfiguration | None:
        """Return the cached sensor configuration, or ``None`` if never read."""
        return self._tactile_config

    def _get_configuration(self) -> TactileSensorConfiguration:
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

        except IOError as e:
            logger.error(f"Failed to get sensor configuration: {e}")
            raise IOError(f"Failed to read sensor configuration: {e}") from e

    def set_auto_data_type(self, resultant: bool = True, taxels: bool = False) -> None:
        """Configure which data types are included in auto-stream frames."""
        self._write_register(ADDR_AUTO_DATA_TYPE, encode_auto_data_type(resultant, taxels))

    def enable_auto_data_transmission(self) -> None:
        self._write_register(ADDR_AUTO_ENABLE, REGISTER_ENABLE)

    def disable_auto_data_transmission(self) -> None:
        self._write_register(ADDR_AUTO_ENABLE, REGISTER_DISABLE)

    # ----- Auto-stream ------------------------------------------------------

    def start_auto_stream(self, resultant: bool = True, taxels: bool = False, min_sensors: int = 1) -> None:
        """Enable the AA 56 stream on the device and start storing frames.

        Raises :class:`NoSensorsAvailableError` if fewer than ``min_sensors``
        sensors are connected.
        """
        if not self._connected:
            raise OSError("Must call connect() first.")
        if not resultant and not taxels:
            raise ValueError("At least one of resultant or taxels must be enabled")

        self._auto_running = False
        self._first_frame_event.clear()
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

        # Idempotent: clear any auto-stream left running from a prior session.
        # Best-effort — if the device or link is unreachable here, the next
        # register write will surface the error.
        try:
            self.disable_auto_data_transmission()
        except Exception:
            pass

        self.set_auto_data_type(resultant=resultant, taxels=taxels)
        self.enable_auto_data_transmission()

        with self._auto_lock:
            self._auto_latest = None
            self._auto_latest_taxels = None
            self._auto_latest_ts = None
            self._auto_stats = AutoStreamStats()
        self._auto_running = True

    def stop_auto_stream(self) -> None:
        """Disable the AA 56 stream on the device and clear the latest cache."""
        was_running = self._auto_running
        self._auto_running = False

        if self._connected and was_running:
            # Best-effort: if the device is unreachable we still tear down
            # local state.
            try:
                self.disable_auto_data_transmission()
            except Exception:
                pass

        with self._auto_lock:
            self._auto_latest = None
            self._auto_latest_taxels = None
            self._auto_latest_ts = None
            self._first_frame_event.clear()

    def wait_for_first_tactile_frame(self, timeout: float = 2.0) -> None:
        """Block until the first frame has been stored, or raise ``TimeoutError``."""
        if not self._first_frame_event.wait(timeout):
            raise TimeoutError(f"No tactile frame within {timeout}s")

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

    # ----- Offsets ----------------------------------------------------------

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
        self._taxel_offsets = None
        self._resultant_offsets = None

    def capture_taxel_offsets(self, num_samples: int = 100) -> dict:
        """Average ``num_samples`` taxel frames and apply the result as zeroing offsets.

        Requires an active auto-stream with taxels enabled. Existing offsets
        are temporarily cleared so the average reflects raw readings.
        """
        if not self._auto_running or not self._auto_mode_taxels:
            raise RuntimeError("Auto-stream with taxels must be active to capture offsets")

        prev_taxel = self._taxel_offsets
        prev_resultant = self._resultant_offsets
        self._taxel_offsets = None
        self._resultant_offsets = None

        time.sleep(OFFSET_CLEAR_SETTLE_S)

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

    def _apply_taxel_offsets(self, taxels: dict, offsets: dict) -> None:
        for finger, taxel_list in taxels.items():
            finger_offsets = offsets.get(finger)
            if not finger_offsets:
                continue
            for i, taxel in enumerate(taxel_list):
                if i >= len(finger_offsets):
                    break
                off = finger_offsets[i]
                taxel[0] = round(taxel[0] - off[0], FORCE_ROUND_DECIMALS)
                taxel[1] = round(taxel[1] - off[1], FORCE_ROUND_DECIMALS)
                taxel[2] = round(max(0, taxel[2] - off[2]), FORCE_ROUND_DECIMALS)

    def _apply_resultant_offsets(self, forces: dict, offsets: dict) -> None:
        for finger, fvec in forces.items():
            off = offsets.get(finger)
            if not off:
                continue
            fvec[0] = round(fvec[0] - off[0], FORCE_ROUND_DECIMALS)
            fvec[1] = round(fvec[1] - off[1], FORCE_ROUND_DECIMALS)
            fvec[2] = round(max(0, fvec[2] - off[2]), FORCE_ROUND_DECIMALS)

    def _apply_stream_offsets(self, parsed_resultant: dict | None, parsed_taxels: dict | None) -> None:
        # Snapshot the references once so a concurrent clear/set on the main
        # thread can't turn the dicts into None mid-iteration.
        taxel_offsets = self._taxel_offsets
        resultant_offsets = self._resultant_offsets
        if taxel_offsets and parsed_taxels:
            self._apply_taxel_offsets(parsed_taxels, taxel_offsets)
        if resultant_offsets and parsed_resultant:
            self._apply_resultant_offsets(parsed_resultant, resultant_offsets)

    def _get_expected_payload_size(self, config: TactileSensorConfiguration) -> int:
        return compute_expected_payload_size(
            self._auto_mode_resultant,
            self._auto_mode_taxels,
            config.active_sensors,
            config.num_taxels,
        )

    # ----- Frame handler ----------------------------------------------------

    def _on_tactile_frame(self, frame_bytes: bytes) -> None:
        """Parse one AA 56 frame and update the latest cache.

        Invoked by the link's demuxer thread; the link has already validated
        LRC, header alignment, and ``eff_len`` bounds, so the payload slice
        is well-formed. This method only checks that the payload size matches
        the active stream mode before decoding.
        """
        if not self._auto_running:
            return

        # AA 56 (2) + reserved (1) + eff_len (2) + payload + LRC (1).
        payload = frame_bytes[2 + AUTO_FRAME_META_SIZE:-1]
        err_code, valid = unpack_auto_payload(payload)

        with self._auto_lock:
            self._auto_stats.last_error_code = err_code

        if self._tactile_config is None:
            with self._auto_lock:
                self._auto_stats.frames_bad_payload += 1
            return

        expected_size = self._get_expected_payload_size(self._tactile_config)
        if expected_size == 0 or len(valid) != expected_size:
            with self._auto_lock:
                self._auto_stats.frames_bad_payload_size += 1
            return

        cfg = self._tactile_config
        if self._auto_mode_resultant and self._auto_mode_taxels:
            parsed_resultant, parsed_taxels = decode_combined_auto(
                valid, cfg.active_sensors, cfg.num_taxels,
            )
        elif self._auto_mode_resultant:
            parsed_resultant = decode_resultant_auto(valid, cfg.active_sensors)
            parsed_taxels = None
        elif self._auto_mode_taxels:
            parsed_resultant = None
            parsed_taxels = decode_taxels_auto(valid, cfg.active_sensors, cfg.num_taxels)
        else:
            return

        self._apply_stream_offsets(parsed_resultant, parsed_taxels)

        with self._auto_lock:
            # Re-check under the lock: stop_auto_stream may have flipped
            # the flag and cleared the cache while we were decoding.
            if not self._auto_running:
                return
            if self._auto_mode_resultant and parsed_resultant is not None:
                self._auto_latest = parsed_resultant
            if self._auto_mode_taxels and parsed_taxels is not None:
                self._auto_latest_taxels = parsed_taxels
            self._auto_latest_ts = time.time()
            self._auto_stats.frames_ok += 1
            self._first_frame_event.set()
