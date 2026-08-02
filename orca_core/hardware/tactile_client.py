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
    LINK_DEFAULT_RESPONSE_TIMEOUT_S,
    TACTILE_REGISTER_ATTEMPTS,
    OFFSET_CAPTURE_DECIMALS,
    OFFSET_CAPTURE_FRAME_BUDGET_S,
    OFFSET_CAPTURE_POLL_S,
    OFFSET_CLEAR_SETTLE_S,
    TACTILE_FIRST_FRAME_TIMEOUT_S,
    TACTILE_STREAM_REARM_MIN_INTERVAL_S,
    TACTILE_STREAM_STALE_REARM_S,
)
from orca_core.hardware.sensing.taxel_geometry import TaxelGeometry, load_taxel_geometry
from orca_core.hardware.sensing.types import (
    ResultantForces,
    ResultantReading,
    TactileReading,
    TaxelReading,
)
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
class TactileStreamStats:
    """Diagnostic counters for the AA 56 handler."""
    frames_ok: int = 0
    frames_bad_payload_size: int = 0
    frames_bad_payload: int = 0
    last_error_code: int = 0  # most recent sensor-reported error code (0 = no error)
    stream_rearms: int = 0  # re-arm attempts after mid-stream silence (device reset)


@dataclass
class TactileSensorConfiguration:
    """Snapshot of connected sensors and their properties.

    Captured at stream start. Sensors are enumerated at power-on and
    mid-stream changes are not reported, so this snapshot is treated as
    immutable for the duration of a stream.
    """
    connected: dict[str, bool] = field(default_factory=dict)  # {finger: is_connected}
    num_taxels: dict[str, int] = field(default_factory=dict)  # {finger: taxel_count}
    module_indices: dict[str, int] = field(default_factory=dict)  # {finger: module_idx}
    finger_to_sensor_id: dict[str, int] = field(default_factory=lambda: dict(DEFAULT_FINGER_TO_SENSOR_ID))
    taxel_geometry: dict[str, TaxelGeometry] = field(default_factory=dict)  # {finger: static positions}

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
        return f"TactileSensorConfiguration({self.num_active_sensors} active: {active})"


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
        self._auto_started_ts: float | None = None
        self._auto_stats = TactileStreamStats()
        self._first_frame_event = threading.Event()

        # Re-arm state: the control lock orders re-arm writes against
        # stop_stream's disable; the generation voids stale re-arm threads.
        self._last_rearm_ts = 0.0
        self._rearm_thread: threading.Thread | None = None
        self._stream_ctrl_lock = threading.Lock()
        self._stream_generation = 0

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
            self.stop_stream()
        except (OSError, RuntimeError):
            logger.exception("Error stopping stream during disconnect")
        try:
            self._link.unregister_frame_handler(PROTOCOL_BYTE_AUTO)
        except (OSError, RuntimeError):
            logger.exception("Error unregistering tactile handler during disconnect")
        self._connected = False

    def __enter__(self):
        if not self._connected:
            self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    # ----- Register I/O -----------------------------------------------------

    def _send_register_request(
        self,
        request: bytes,
        response_timeout_s: float,
        attempts: int = TACTILE_REGISTER_ATTEMPTS,
    ) -> bytes:
        """Send a register request, retrying on timeout up to ``attempts`` times.

        A single round-trip is occasionally lost on a busy link; the retries
        recover it. A link that is not running raises ``RuntimeError``, which
        is not retried; a link that closes or fails mid-wait raises ``IOError``,
        and its retries fail fast on the closed link.
        """
        last_err: IOError | None = None
        for _ in range(attempts):
            try:
                return self._link.send_register_request(
                    request, response_timeout_s=response_timeout_s
                )
            except IOError as e:
                last_err = e
        raise last_err

    def _read_register(self, address: int, count: int, response_timeout_s: float = LINK_DEFAULT_RESPONSE_TIMEOUT_S) -> bytes:
        if not self._connected:
            raise OSError("Must call connect() first.")
        request = build_read_request(address, count)
        response = self._send_register_request(request, response_timeout_s)
        return parse_read_response(response, expected_address=address)

    def _write_register(
        self,
        address: int,
        data: bytes,
        response_timeout_s: float = LINK_DEFAULT_RESPONSE_TIMEOUT_S,
        attempts: int = TACTILE_REGISTER_ATTEMPTS,
    ) -> None:
        if not self._connected:
            raise OSError("Must call connect() first.")
        request = build_write_request(address, data)
        response = self._send_register_request(request, response_timeout_s, attempts)
        parse_write_response(response, expected_address=address)

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

    def read_resultant_force(self) -> ResultantForces:
        """Read resultant force from all connected fingertip sensors, applying offsets."""
        result = self._read_raw_resultant()
        offsets = self._resultant_offsets
        if offsets:
            self._apply_resultant_offsets(result, offsets)
        return result

    def _read_raw_resultant(self) -> ResultantForces:
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

    def get_taxel_geometry(self) -> dict[str, TaxelGeometry]:
        """Return static per-taxel positions ``{finger: TaxelGeometry}`` for connected fingers.

        Positions are fixed sensor geometry in the sensor frame (meters), read
        once at connect. Row ``i`` of ``geometry[finger].positions`` aligns
        with taxel ``i`` in the force stream. Empty until the configuration is
        read.
        """
        if self._tactile_config is None:
            return {}
        return dict(self._tactile_config.taxel_geometry)

    @staticmethod
    def _load_finger_geometry(finger: str, reported_taxels: int | None) -> TaxelGeometry:
        """Load a finger's static geometry, warning if it disagrees with the sensor."""
        geometry = load_taxel_geometry(finger)
        if reported_taxels is not None and geometry.num_taxels != reported_taxels:
            logger.warning(
                f"Taxel geometry for {finger} has {geometry.num_taxels} positions but "
                f"sensor reports {reported_taxels} taxels; positions may be misaligned "
                f"(wrong sensor model in FINGER_MODELS?)"
            )
        return geometry

    def _get_configuration(self) -> TactileSensorConfiguration:
        try:
            connected = self.read_connected_sensors()
            num_taxels = self.read_num_taxels()

            module_indices = {}
            taxel_geometry = {}
            for finger in FINGER_NAMES:
                if connected.get(finger, False):
                    sensor_id = self._finger_to_sensor_id[finger]
                    module_indices[finger] = compute_distal_module_index(sensor_id)
                    try:
                        taxel_geometry[finger] = self._load_finger_geometry(
                            finger, num_taxels.get(finger))
                    except Exception as e:
                        logger.warning(
                            f"Taxel geometry unavailable for {finger}: {e}")

            config = TactileSensorConfiguration(
                connected=connected,
                num_taxels=num_taxels,
                module_indices=module_indices,
                finger_to_sensor_id=dict(self._finger_to_sensor_id),
                taxel_geometry=taxel_geometry,
            )

            logger.info(f"Configuration captured: {config}")
            return config

        except IOError as e:
            logger.error(f"Failed to get sensor configuration: {e}")
            raise IOError(f"Failed to read sensor configuration: {e}") from e

    def set_auto_data_type(
        self,
        resultant: bool = True,
        taxels: bool = False,
        attempts: int = TACTILE_REGISTER_ATTEMPTS,
    ) -> None:
        """Configure which data types are included in auto-stream frames."""
        self._write_register(
            ADDR_AUTO_DATA_TYPE, encode_auto_data_type(resultant, taxels),
            attempts=attempts,
        )

    def enable_auto_data_transmission(
        self, attempts: int = TACTILE_REGISTER_ATTEMPTS
    ) -> None:
        self._write_register(ADDR_AUTO_ENABLE, REGISTER_ENABLE, attempts=attempts)

    def disable_auto_data_transmission(self) -> None:
        self._write_register(ADDR_AUTO_ENABLE, REGISTER_DISABLE)

    # ----- Auto-stream ------------------------------------------------------

    def start_stream(self, resultant: bool = True, taxels: bool = False, min_sensors: int = 1) -> None:
        """Enable the AA 56 stream on the device and start storing frames.

        Raises :class:`NoSensorsAvailableError` if fewer than ``min_sensors``
        sensors are connected.
        """
        if not self._connected:
            raise OSError("Must call connect() first.")
        if not resultant and not taxels:
            raise ValueError("At least one of resultant or taxels must be enabled")

        with self._auto_lock:
            self._auto_running = False
            self._stream_generation += 1
            self._auto_mode_resultant = resultant
            self._auto_mode_taxels = taxels
            self._first_frame_event.clear()

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
        # Best-effort — a failure here surfaces on the next register write.
        try:
            self.disable_auto_data_transmission()
        except (OSError, RuntimeError) as e:
            logger.debug(f"Idempotent stream clear failed, continuing: {e}")

        self.set_auto_data_type(resultant=resultant, taxels=taxels)
        self.enable_auto_data_transmission()

        with self._auto_lock:
            self._auto_latest = None
            self._auto_latest_taxels = None
            self._auto_latest_ts = None
            self._auto_started_ts = time.monotonic()
            self._auto_stats = TactileStreamStats()
            self._auto_running = True
            self._last_rearm_ts = 0.0

    def stop_stream(self) -> None:
        """Disable the AA 56 stream on the device and clear the latest cache."""
        with self._auto_lock:
            was_running = self._auto_running
            self._auto_running = False
            self._stream_generation += 1

        if self._connected and was_running:
            # Best-effort disable; the control lock orders it after any
            # in-flight re-arm writes, so the device ends up disabled.
            with self._stream_ctrl_lock:
                try:
                    self.disable_auto_data_transmission()
                except (OSError, RuntimeError) as e:
                    logger.debug(f"Disabling stream during stop failed, continuing: {e}")

        with self._auto_lock:
            self._auto_latest = None
            self._auto_latest_taxels = None
            self._auto_latest_ts = None
            self._first_frame_event.clear()

    def wait_for_first_frame(self, timeout: float = TACTILE_FIRST_FRAME_TIMEOUT_S) -> None:
        """Block until the first frame has been stored, or raise ``TimeoutError``."""
        if not self._first_frame_event.wait(timeout):
            raise TimeoutError(f"No tactile frame within {timeout}s")

    def _maybe_rearm_stream(self) -> None:
        """Re-enable the stream if it went silent mid-run (a device reset
        clears its volatile enable register)."""
        now = time.monotonic()
        with self._auto_lock:
            if not self._auto_running:
                return
            last = self._auto_latest_ts or self._auto_started_ts
            if last is None or now - last < TACTILE_STREAM_STALE_REARM_S:
                return
            if now - self._last_rearm_ts < TACTILE_STREAM_REARM_MIN_INTERVAL_S:
                return
            if self._rearm_thread is not None and self._rearm_thread.is_alive():
                return
            self._last_rearm_ts = now
            self._auto_stats.stream_rearms += 1
            stale_s = now - last
            self._rearm_thread = threading.Thread(
                target=self._rearm_stream,
                args=(
                    stale_s,
                    self._auto_mode_resultant,
                    self._auto_mode_taxels,
                    self._stream_generation,
                ),
                name="TactileStreamRearm",
                daemon=True,
            )
        self._rearm_thread.start()

    def _rearm_stream(
        self, stale_s: float, resultant: bool, taxels: bool, generation: int
    ) -> None:
        """Best-effort; on failure the next stale read past the rate limit retries."""
        with self._stream_ctrl_lock:
            with self._auto_lock:
                # A stream stopped or restarted since this thread was spawned
                # must not be re-enabled behind the caller's back.
                if not self._auto_running or generation != self._stream_generation:
                    return
            logger.warning(
                "tactile stream silent for %.1f s while running — re-arming "
                "(a device reset clears its volatile enable register)",
                stale_s,
            )
            # Single-attempt writes keep the control-lock hold short, so a concurrent
            # stop_stream/disconnect never waits behind register retries.
            try:
                self.set_auto_data_type(resultant=resultant, taxels=taxels, attempts=1)
                with self._auto_lock:
                    # Stream stopped or restarted mid-re-arm: leave it disabled.
                    if not self._auto_running or generation != self._stream_generation:
                        return
                self.enable_auto_data_transmission(attempts=1)
            except Exception as e:
                logger.warning("tactile stream re-arm failed: %s", e)

    def get_latest_forces(self) -> ResultantReading | None:
        """Return the most recent resultant reading, or ``None`` if none yet."""
        self._maybe_rearm_stream()
        with self._auto_lock:
            if self._auto_latest is None:
                return None
            return ResultantReading(forces=self._auto_latest, timestamp=self._auto_latest_ts)

    def get_latest_taxels(self) -> TaxelReading | None:
        """Return the most recent per-taxel reading, or ``None`` if none yet."""
        self._maybe_rearm_stream()
        with self._auto_lock:
            if self._auto_latest_taxels is None:
                return None
            return TaxelReading(taxels=self._auto_latest_taxels, timestamp=self._auto_latest_ts)

    def get_latest(self) -> TactileReading | None:
        """Return resultant and per-taxel forces from the same frame, or ``None`` if none yet.

        Single locked snapshot, so ``forces`` and ``taxels`` share one
        timestamp. Either field is ``None`` if its stream mode is disabled.
        """
        with self._auto_lock:
            forces = self._auto_latest
            taxels = self._auto_latest_taxels
            timestamp = self._auto_latest_ts
        if forces is None and taxels is None:
            return None
        return TactileReading(
            forces=ResultantReading(forces=forces, timestamp=timestamp) if forces is not None else None,
            taxels=TaxelReading(taxels=taxels, timestamp=timestamp) if taxels is not None else None,
            timestamp=timestamp,
        )

    def get_stats(self):
        """Return a snapshot copy of ``TactileStreamStats`` for the current loop."""
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

    def capture_taxel_offsets(
        self, num_samples: int = 100, timeout_s: float | None = None
    ) -> dict:
        """Average ``num_samples`` taxel frames and apply the result as zeroing offsets.

        Requires an active auto-stream with taxels enabled. Existing offsets
        are temporarily cleared so the average reflects raw readings, and are
        restored if the capture fails. ``timeout_s`` bounds the wait for
        frames (derived from ``num_samples`` when ``None``); a stream that is
        dead at entry or stalls mid-capture raises ``TimeoutError`` instead
        of blocking forever.
        """
        if not self._auto_running or not self._auto_mode_taxels:
            raise RuntimeError("Auto-stream with taxels must be active to capture offsets")
        if timeout_s is None:
            timeout_s = (
                TACTILE_STREAM_STALE_REARM_S
                + num_samples * OFFSET_CAPTURE_FRAME_BUDGET_S
            )

        prev_taxel = self._taxel_offsets
        prev_resultant = self._resultant_offsets
        self._taxel_offsets = None
        self._resultant_offsets = None

        time.sleep(OFFSET_CLEAR_SETTLE_S)

        succeeded = False
        try:
            frames = []
            last_ts = None
            deadline = time.monotonic() + timeout_s
            while len(frames) < num_samples:
                reading = self.get_latest_taxels()
                if reading is not None and reading.timestamp != last_ts:
                    frames.append(reading.taxels)
                    last_ts = reading.timestamp
                    continue
                if time.monotonic() > deadline:
                    raise TimeoutError(
                        f"tactile stream stalled during offset capture: got "
                        f"{len(frames)}/{num_samples} frames in {timeout_s:.1f}s"
                    )
                time.sleep(OFFSET_CAPTURE_POLL_S)

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
                        round(sum_fx / num_samples, OFFSET_CAPTURE_DECIMALS),
                        round(sum_fy / num_samples, OFFSET_CAPTURE_DECIMALS),
                        round(sum_fz / num_samples, OFFSET_CAPTURE_DECIMALS),
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

    # ----- Frame handler ----------------------------------------------------

    def _on_tactile_frame(self, frame_bytes: bytes) -> None:
        """Parse one AA 56 frame and update the latest cache.

        Invoked by the link's demuxer thread; the link has already validated
        LRC, header alignment, and ``effective_length`` bounds, so the payload slice
        is well-formed. This method only checks that the payload size matches
        the active stream mode before decoding.
        """
        with self._auto_lock:
            if not self._auto_running:
                return
            mode_resultant = self._auto_mode_resultant
            mode_taxels = self._auto_mode_taxels
            config = self._tactile_config

        # AA 56 (2) + reserved (1) + effective_length (2) + payload + LRC (1).
        payload = frame_bytes[2 + AUTO_FRAME_META_SIZE:-1]
        err_code, valid = unpack_auto_payload(payload)

        with self._auto_lock:
            self._auto_stats.last_error_code = err_code

        if config is None:
            with self._auto_lock:
                self._auto_stats.frames_bad_payload += 1
            return

        expected_size = compute_expected_payload_size(
            mode_resultant, mode_taxels, config.active_sensors, config.num_taxels,
        )
        if expected_size == 0 or len(valid) != expected_size:
            with self._auto_lock:
                self._auto_stats.frames_bad_payload_size += 1
            return

        if mode_resultant and mode_taxels:
            parsed_resultant, parsed_taxels = decode_combined_auto(
                valid, config.active_sensors, config.num_taxels,
            )
        elif mode_resultant:
            parsed_resultant = decode_resultant_auto(valid, config.active_sensors)
            parsed_taxels = None
        elif mode_taxels:
            parsed_resultant = None
            parsed_taxels = decode_taxels_auto(valid, config.active_sensors, config.num_taxels)
        else:
            return

        self._apply_stream_offsets(parsed_resultant, parsed_taxels)

        with self._auto_lock:
            # Re-check under the lock: stop_stream may have flipped
            # the flag and cleared the cache while we were decoding.
            if not self._auto_running:
                return
            if mode_resultant and parsed_resultant is not None:
                self._auto_latest = parsed_resultant
            if mode_taxels and parsed_taxels is not None:
                self._auto_latest_taxels = parsed_taxels
            # Monotonic, matching EncoderReading.timestamp, so staleness
            # logic and cross-stream correlation survive wall-clock steps.
            self._auto_latest_ts = time.monotonic()
            self._auto_stats.frames_ok += 1
            self._first_frame_event.set()
