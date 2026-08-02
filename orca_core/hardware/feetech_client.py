# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Communication using the Feetech SCServo SDK."""

import atexit
import logging
import threading
import time
from typing import Optional, Sequence, Tuple
import numpy as np

from ..constants import FEETECH
from .motor_client import MotionTimeoutError, MotorClient, MotorRead
from .feetech import (
    PortHandler,
    sms_sts,
    GroupSyncWrite,
    GroupSyncRead,
    COMM_SUCCESS,
    SMS_STS_TORQUE_ENABLE,
    SMS_STS_MODE,
    SMS_STS_PRESENT_POSITION_L,
    SMS_STS_PRESENT_SPEED_L,
    SMS_STS_PRESENT_CURRENT_L,
    SMS_STS_PRESENT_TEMPERATURE,
    SMS_STS_MOVING,
    SMS_STS_ACC,
    SMS_STS_GOAL_POSITION_L,
    SMS_STS_GOAL_TIME_L,
    SMS_STS_GOAL_SPEED_L,
    SMS_STS_ID,
    SMS_STS_BAUD_RATE,
)

# Map host-facing baud rates to the firmware's register code.
FEETECH_BAUD_RATE_MAP: dict[int, int] = {
    1_000_000: 0,
    500_000: 1,
    250_000: 2,
    128_000: 3,
    115_200: 4,
    76_800: 5,
    57_600: 6,
    38_400: 7,
}

# Model-number → human name. Multiple raw values can map to the same label.
FEETECH_MODELS: dict[int, str] = {
    4106: 'HLS3930',
    6922: 'HLS3915',
    5130: 'HLS3915',
}

# SCServo position scale: 0-4095 raw units = 0-360 degrees = 0-2*pi radians
DEFAULT_POS_SCALE = 2.0 * np.pi / 4096  # 4096 steps for 360°
DEFAULT_VEL_SCALE = 0.732 * 2.0 * np.pi / 60.0  # Convert 0.732 RPM/unit to rad/s
DEFAULT_CUR_SCALE = 6.5  # mA per unit

# Position limits for STS servo mode (0-4095, one full rotation)
POS_MIN = 0
POS_MAX = 4095

# Feetech servos rotate in the opposite direction from Dynamixel for the same
# raw command. We invert here so OrcaHand sees a single, motor-type-agnostic
# convention; per-joint sign tuning (joint_to_motor_map) can then be shared
# across motor types.
POSITION_DIRECTION = -1


def feetech_cleanup_handler():
    """Disconnect every open Feetech client at interpreter exit."""
    FeetechClient.cleanup_open_clients()


class FeetechClient(MotorClient):
    """Client for communicating with Feetech SCServo motors.

    This implements the MotorClient interface for Feetech motors,
    providing compatibility with the OrcaHand control system.

    Thread safety / lock contract:
        All bus I/O is serialized by a single reentrant lock
        (``self._bus_lock``). Every public method that touches the port holds
        the lock for the whole transaction (request + status reply), so
        transactions from different threads can never interleave on the wire.
        Callers therefore do not need their own locking around individual
        calls; external locks remain harmless but redundant.

        ``set_torque_enabled`` keeps the lock across its retry loop
        (including the ``retry_interval`` sleeps between failed attempts) and
        ``set_operating_mode`` across its torque-off/mode-write/torque-on
        sequence; other threads block on the bus for that duration.
        ``wait_for_motion_complete`` locks per poll so it never wedges the
        bus for its whole timeout.

        On any failed transaction (comm error/timeout), the OS receive buffer
        is flushed before the lock is released, so a late status reply can
        never be consumed as the response to a later transaction.
    """

    waits_for_motion = True

    motor_type = FEETECH
    factory_default_id = 1
    factory_default_baudrate = 1_000_000
    baud_rate_map = FEETECH_BAUD_RATE_MAP
    # Feetech motors latch their ID at power-up, so the bus must be de-powered
    # before a motor is plugged in.
    requires_unpowered_hotplug = True

    # Clients with an open port; registered on successful connect() so the
    # atexit cleanup only ever touches live connections.
    OPEN_CLIENTS = set()

    def __init__(
        self,
        motor_ids: Sequence[int],
        port: str = '/dev/ttyUSB0',
        baudrate: int = 1000000,
        lazy_connect: bool = False,
        pos_scale: Optional[float] = None,
        vel_scale: Optional[float] = None,
        cur_scale: Optional[float] = None,
    ):
        """Initializes a new Feetech client.

        Args:
            motor_ids: All motor IDs being used by the client.
            port: The serial port to connect to.
            baudrate: The baudrate to communicate with.
            lazy_connect: If True, automatically connects when calling a method
                that requires a connection, if not already connected.
            pos_scale: The scaling factor for positions (raw to radians).
            vel_scale: The scaling factor for velocities.
            cur_scale: The scaling factor for currents.
        """
        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate
        self.lazy_connect = lazy_connect

        self.pos_scale = pos_scale if pos_scale is not None else DEFAULT_POS_SCALE
        self.vel_scale = vel_scale if vel_scale is not None else DEFAULT_VEL_SCALE
        self.cur_scale = cur_scale if cur_scale is not None else DEFAULT_CUR_SCALE

        self.port_handler = PortHandler(port)
        self.packet_handler: Optional[sms_sts] = None

        # RLock: mode/EEPROM sequences re-enter via set_torque_enabled.
        self._bus_lock = threading.RLock()

        self._connected = False

        # Last known-good state per motor: a motor missing from a read keeps
        # its cached value and the read is reported via ``last_read_ok``.
        num_motors = len(self.motor_ids)
        self._cached_positions = np.zeros(num_motors, dtype=np.float32)
        self._cached_velocities = np.zeros(num_motors, dtype=np.float32)
        self._cached_currents = np.zeros(num_motors, dtype=np.float32)
        self._last_read_ok = True

        # Default motion parameters
        # Speed unit is 0.732 RPM per value; the motor's firmware caps speed
        # to whatever its hardware can sustain, so passing a large value just
        # means "go as fast as you can".
        self._default_speed = 1500  # Effectively "max speed" for STS-class
        self._default_acc = 150  # Acceleration (0-254): faster ramp-up
        self._default_torque = 500  # Torque limit (0-1000), required for motion

    @property
    def is_connected(self) -> bool:
        return self._connected and self.port_handler.is_open

    def connect(self) -> None:
        """Connects to the Feetech motors."""
        if self._connected:
            raise RuntimeError('Client is already connected.')

        with self._bus_lock:
            self.port_handler.baudrate = self.baudrate

            if self.port_handler.openPort():
                logging.info('Succeeded to open port: %s', self.port_name)
            else:
                raise OSError(
                    f'Failed to open port at {self.port_name} (Check that the device is '
                    'powered on and connected to your computer).'
                )

            # A failure past this point must not leave the port open (and
            # advisory-locked) with no registered owner to close it.
            try:
                # Advisory-lock the port so exclusive-mode openers elsewhere are rejected.
                try:
                    import fcntl
                    fcntl.flock(self.port_handler.ser.fileno(),
                                fcntl.LOCK_EX | fcntl.LOCK_NB)
                except (ImportError, AttributeError, OSError):
                    pass  # Windows (no fcntl), mocked ports, or lock unavailable — best-effort

                # Enable low latency mode for faster communication
                if hasattr(self.port_handler, 'ser') and hasattr(self.port_handler.ser, 'set_low_latency_mode'):
                    try:
                        self.port_handler.ser.set_low_latency_mode(True)
                        logging.info('Enabled low latency mode for USB serial')
                    except Exception:
                        pass  # Not critical if it fails

                self.packet_handler = sms_sts(self.port_handler)
                self._connected = True

                # Ensure motors are in servo mode (not wheel mode)
                # This prevents issues if motors were left in wheel mode from a previous session
                for motor_id in self.motor_ids:
                    result, error = self.packet_handler.write1ByteTxRx(
                        motor_id, SMS_STS_MODE, 0
                    )
                    if result != COMM_SUCCESS or error != 0:
                        self._flush_input_buffer()

                # Torque is left as-is: connecting must never make the hand
                # stiffen or move. Callers opt in via enable_torque()/init_joints().

                self.OPEN_CLIENTS.add(self)
            except Exception:
                self._connected = False
                try:
                    self.port_handler.closePort()
                except Exception:
                    pass
                raise

    def disconnect(self) -> None:
        """Disconnects from the Feetech motors.

        The port is always closed and the client deregistered, even when the
        final torque-disable raises (e.g. the serial link is already gone);
        that exception propagates after cleanup.
        """
        if not self._connected:
            return

        if self.port_handler.is_using:
            logging.error('Port handler in use; cannot disconnect.')
            return

        with self._bus_lock:
            try:
                # Disable torque before disconnecting
                self.set_torque_enabled(self.motor_ids, False, retries=0)
            finally:
                self.port_handler.closePort()
                self._connected = False
                self.OPEN_CLIENTS.discard(self)

    @staticmethod
    def scan_for_motors(
        port: str,
        id_range: tuple,
        baud_rates: Optional[list] = None,
    ) -> list:
        """Scan ``port`` for any responding Feetech motors.

        Tries each baud in ``baud_rates`` (defaults to all known rates), pings
        every ID in ``id_range``, and returns a list of dicts with keys
        ``id``, ``baud_rate``, ``model_name``. Doesn't change motor state.
        """
        if baud_rates is None:
            baud_rates = list(FEETECH_BAUD_RATE_MAP.keys())
        detected_motors = []
        for baud_rate in baud_rates:
            port_handler = PortHandler(port)
            port_handler.baudrate = baud_rate
            try:
                if not port_handler.openPort():
                    logging.warning(
                        'Failed to open port %s at %d baud', port, baud_rate
                    )
                    continue
                packet_handler = sms_sts(port_handler)
                for motor_id in range(id_range[0], id_range[1] + 1):
                    model_number, result, _ = packet_handler.ping(motor_id)
                    if result == COMM_SUCCESS:
                        detected_motors.append({
                            'id': motor_id,
                            'baud_rate': baud_rate,
                            'model_name': FEETECH_MODELS.get(model_number, 'Feetech'),
                        })
                port_handler.closePort()
            except Exception as e:
                logging.warning(
                    'Error scanning port %s at %d baud: %s', port, baud_rate, e
                )
                try:
                    port_handler.closePort()
                except Exception:
                    pass
        return detected_motors

    def change_motor_id(self, current_id: int, new_id: int) -> bool:
        """Re-program a motor's bus ID. Persists in EEPROM."""
        if not (0 <= new_id <= 253):
            logging.error("Invalid ID %d. Valid range is 0-253.", new_id)
            return False
        try:
            self._check_connected()
            with self._bus_lock:
                self.set_torque_enabled([current_id], False, retries=0)
                self.packet_handler.unLockEprom(current_id)
                result, error = self.packet_handler.write1ByteTxRx(
                    current_id, SMS_STS_ID, new_id
                )
                self.packet_handler.LockEprom(new_id)
                if result == COMM_SUCCESS and error == 0:
                    logging.info("Changed motor ID: %d -> %d", current_id, new_id)
                    return True
                self._flush_input_buffer()
                logging.error(
                    "Failed to change motor ID: result=%d, error=%d", result, error
                )
                return False
        except Exception as e:
            logging.error("Failed to change motor ID: %s", e)
            return False

    def change_motor_baudrate(self, motor_id: int, new_baud_rate: int) -> bool:
        """Re-program a motor's UART baud. Caller must reconnect at the new rate."""
        if new_baud_rate not in FEETECH_BAUD_RATE_MAP:
            logging.error(
                "Invalid baud rate %d. Valid: %s",
                new_baud_rate,
                list(FEETECH_BAUD_RATE_MAP.keys()),
            )
            return False
        try:
            self._check_connected()
            with self._bus_lock:
                self.set_torque_enabled([motor_id], False, retries=0)
                self.packet_handler.unLockEprom(motor_id)
                result, error = self.packet_handler.write1ByteTxRx(
                    motor_id, SMS_STS_BAUD_RATE, FEETECH_BAUD_RATE_MAP[new_baud_rate]
                )
                self.packet_handler.LockEprom(motor_id)
                if result == COMM_SUCCESS and error == 0:
                    logging.info("Changed motor %d baud rate to %d", motor_id, new_baud_rate)
                    return True
                self._flush_input_buffer()
                logging.error(
                    "Failed to change baud rate: result=%d, error=%d", result, error
                )
                return False
        except Exception as e:
            logging.error("Failed to change baud rate: %s", e)
            return False

    def set_torque_enabled(
        self,
        motor_ids: Sequence[int],
        enabled: bool,
        retries: int = 3,
        retry_interval: float = 0.25,
    ) -> "list[int]":
        """Sets whether torque is enabled for the motors.

        Holds the bus lock for the entire call, including any retry sleeps.

        Args:
            motor_ids: The motor IDs to configure.
            enabled: Whether to engage or disengage the motors.
            retries: The number of times to retry after the first attempt.
                0 means a single attempt; <0 retries forever.
            retry_interval: The number of seconds to wait between retries.

        Returns:
            A list of motor IDs that could not be set.
        """
        self._check_connected()

        with self._bus_lock:
            remaining_ids = list(motor_ids)
            while remaining_ids:
                failed_ids = []
                for motor_id in remaining_ids:
                    result, error = self.packet_handler.write1ByteTxRx(
                        motor_id, SMS_STS_TORQUE_ENABLE, int(enabled)
                    )
                    if result != COMM_SUCCESS or error != 0:
                        failed_ids.append(motor_id)
                        self._flush_input_buffer()

                remaining_ids = failed_ids
                if not remaining_ids:
                    break
                logging.error(
                    'Could not set torque %s for IDs: %s',
                    'enabled' if enabled else 'disabled',
                    str(remaining_ids),
                )
                if retries == 0:
                    break
                time.sleep(retry_interval)
                retries -= 1
            return remaining_ids

    def set_operating_mode(self, motor_ids: Sequence[int], mode: int) -> None:
        """Sets the operating mode for the specified motors.

        Feetech only supports:
        - Mode 0: Servo mode (position control)
        - Mode 1: Wheel mode (velocity control)

        Unsupported Dynamixel modes are mapped to servo mode with a warning.
        Motors that stay unreachable are logged and skipped after the finite
        default retries of :meth:`set_torque_enabled`.
        """
        self._check_connected()

        # Warn about unsupported modes (mode 5 is supported via torque parameter)
        unsupported_modes = {
            0: "current control (mode 0) - using servo mode instead",
            4: "multi-turn (mode 4) - using servo mode (limited to 360°)",
        }
        if mode in unsupported_modes:
            logging.warning(
                "Feetech does not support %s",
                unsupported_modes[mode]
            )
        if mode == 5:
            logging.info(
                "Feetech: current-based position mode uses torque limiting"
            )

        with self._bus_lock:
            # Mode changes require torque off; motors that never acked the
            # torque-disable are skipped so their mode is not changed blind.
            failed_ids = self.set_torque_enabled(motor_ids, False)
            if failed_ids:
                logging.error(
                    'Skipping mode change for motors that did not ack torque '
                    'disable: %s', str(failed_ids)
                )
            acked_ids = [mid for mid in motor_ids if mid not in failed_ids]

            for motor_id in acked_ids:
                # Only velocity mode (1) maps to wheel mode; all others use servo mode
                feetech_mode = 1 if mode == 1 else 0

                result, error = self.packet_handler.write1ByteTxRx(
                    motor_id, SMS_STS_MODE, feetech_mode
                )
                if result != COMM_SUCCESS or error != 0:
                    self._flush_input_buffer()
                    logging.error(
                        'Failed to set operating mode for motor %d: result=%d, error=%d',
                        motor_id, result, error
                    )

            # Re-enable torque
            if acked_ids:
                self.set_torque_enabled(acked_ids, True)

    def _read_state_per_motor_fallback(self) -> MotorRead:
        """Per-motor read of position/velocity/current; used only when sync
        read fails. Fields that fail keep their cached value and the read is
        reported as not ok via ``last_read_ok``."""
        self._check_connected()

        with self._bus_lock:
            positions = self._cached_positions.copy()
            velocities = self._cached_velocities.copy()
            currents = self._cached_currents.copy()
            read_ok = True

            for i, motor_id in enumerate(self.motor_ids):
                # Read position
                pos_raw, result, error = self.packet_handler.read2ByteTxRx(
                    motor_id, SMS_STS_PRESENT_POSITION_L
                )
                if result == COMM_SUCCESS and error == 0:
                    pos_signed = self.packet_handler.scs_tohost(pos_raw, 15)
                    pos_normalized = self._normalize_position(pos_signed)
                    positions[i] = self._raw_to_rad(pos_normalized, self.pos_scale)
                else:
                    read_ok = False
                    self._flush_input_buffer()
                    logging.warning(
                        'Failed to read position for motor %d: result=%d, error=%d',
                        motor_id, result, error
                    )

                # Read velocity
                vel_raw, result, error = self.packet_handler.read2ByteTxRx(
                    motor_id, SMS_STS_PRESENT_SPEED_L
                )
                if result == COMM_SUCCESS and error == 0:
                    vel_signed = self.packet_handler.scs_tohost(vel_raw, 15)
                    velocities[i] = self._raw_to_rad(vel_signed, self.vel_scale)
                else:
                    read_ok = False
                    self._flush_input_buffer()
                    logging.warning(
                        'Failed to read velocity for motor %d: result=%d, error=%d',
                        motor_id, result, error
                    )

                # Read current
                cur_raw, result, error = self.packet_handler.read2ByteTxRx(
                    motor_id, SMS_STS_PRESENT_CURRENT_L
                )
                if result == COMM_SUCCESS and error == 0:
                    cur_signed = self.packet_handler.scs_tohost(cur_raw, 15)
                    currents[i] = cur_signed * self.cur_scale
                else:
                    read_ok = False
                    self._flush_input_buffer()
                    logging.warning(
                        'Failed to read current for motor %d: result=%d, error=%d',
                        motor_id, result, error
                    )

            # Cache copies: callers receive the returned arrays and may mutate them.
            self._cached_positions = positions.copy()
            self._cached_velocities = velocities.copy()
            self._cached_currents = currents.copy()
            self._last_read_ok = read_ok
            return MotorRead(position=positions, velocity=velocities, current=currents)

    def read_temperature(self) -> np.ndarray:
        """Reads the temperature for all motors via a single sync-read packet."""
        self._check_connected()

        with self._bus_lock:
            temperatures = np.zeros(len(self.motor_ids), dtype=np.float32)

            sync_read = GroupSyncRead(self.packet_handler, SMS_STS_PRESENT_TEMPERATURE, 1)
            for motor_id in self.motor_ids:
                sync_read.addParam(motor_id)

            if sync_read.txRxPacket() != COMM_SUCCESS:
                self._flush_input_buffer()
                logging.warning('Sync temp read failed, falling back to individual reads')
                return self._read_temperature_per_motor_fallback()

            for i, motor_id in enumerate(self.motor_ids):
                available, _ = sync_read.isAvailable(motor_id, SMS_STS_PRESENT_TEMPERATURE, 1)
                if available:
                    temperatures[i] = float(sync_read.getData(motor_id, SMS_STS_PRESENT_TEMPERATURE, 1))
                else:
                    self._flush_input_buffer()
                    logging.warning('Motor %d not available in sync temp read', motor_id)

            return temperatures

    def _read_temperature_per_motor_fallback(self) -> np.ndarray:
        """Per-motor read of temperature; used only when sync read fails."""
        temperatures = np.zeros(len(self.motor_ids), dtype=np.float32)
        with self._bus_lock:
            for i, motor_id in enumerate(self.motor_ids):
                temp, result, error = self.packet_handler.read1ByteTxRx(
                    motor_id, SMS_STS_PRESENT_TEMPERATURE
                )
                if result == COMM_SUCCESS and error == 0:
                    temperatures[i] = float(temp)
                else:
                    self._flush_input_buffer()
                    logging.warning(
                        'Failed to read temperature for motor %d: result=%d, error=%d',
                        motor_id, result, error
                    )

        return temperatures

    def wait_for_motion_complete(
        self,
        timeout: float = 5.0,
        poll_interval: float = 0.02,
    ) -> None:
        """Block until every motor reports it has stopped.

        Reads each motor's MOVING flag (register 66) via sync read; the bit
        is 1 while the servo is travelling toward its goal position and 0
        once it has settled.

        Args:
            timeout: Max seconds to wait before giving up.
            poll_interval: Seconds between polls.

        Raises:
            MotionTimeoutError: If any motor is still moving when the
                timeout elapses.
        """
        self._check_connected()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            # Lock per poll (not across the sleeps) so waiting for motion
            # never starves other bus traffic for the whole timeout.
            with self._bus_lock:
                sync_read = GroupSyncRead(self.packet_handler, SMS_STS_MOVING, 1)
                for motor_id in self.motor_ids:
                    sync_read.addParam(motor_id)

                if sync_read.txRxPacket() != COMM_SUCCESS:
                    self._flush_input_buffer()
                    all_stopped = False
                else:
                    all_stopped = True
                    for motor_id in self.motor_ids:
                        available, _ = sync_read.isAvailable(motor_id, SMS_STS_MOVING, 1)
                        if not available:
                            self._flush_input_buffer()
                            all_stopped = False
                            break
                        if sync_read.getData(motor_id, SMS_STS_MOVING, 1) != 0:
                            all_stopped = False
                            break

            if all_stopped:
                return
            time.sleep(poll_interval)

        raise MotionTimeoutError(
            f'Motors did not settle within {timeout:.1f}s'
        )

    def write_desired_pos(
        self,
        motor_ids: Sequence[int],
        positions: np.ndarray,
        speed: Optional[int] = None,
        torque: Optional[int] = None,
    ) -> None:
        """Writes desired positions to the motors.

        Routes through ``write_positions_sync`` so all motors receive their
        targets in a single broadcast packet (one round-trip instead of N).

        Args:
            motor_ids: Motor IDs to write to.
            positions: Target positions in radians.
            speed: Movement speed (0.732 RPM per unit). Default ~60 = 44 RPM.
            torque: Torque limit (0-1000). Default 500. Required for motion.
        """
        self.write_positions_sync(motor_ids, positions, speed=speed, torque=torque)

    def write_desired_current(
        self,
        motor_ids: Sequence[int],
        currents: np.ndarray,
    ) -> None:
        """Writes desired currents (torque limits) to the motors.

        Feetech uses torque limiting (0-1000) instead of direct current control.
        This method maps current values to torque: 1 mA ≈ 1 torque unit.
        The torque limit takes effect on the next position command.

        Args:
            motor_ids: Motor IDs to configure.
            currents: Desired current limits in mA. Mapped to torque (0-1000).
        """
        self._check_connected()

        if len(motor_ids) != len(currents):
            raise ValueError('motor_ids and currents must have the same length')

        with self._bus_lock:
            # Map current (mA) directly to torque (0-1000)
            # Typical values: 200-400 mA for calibration, 500-1000 mA for normal operation
            for current in currents:
                torque_raw = int(np.clip(abs(current), 0, 1000))
                self._default_torque = torque_raw

        logging.debug(
            'Updated default torque to %d (from current %.1f mA)',
            self._default_torque, currents[0] if len(currents) > 0 else 0
        )

    def _check_connected(self) -> None:
        """Ensures the client is connected."""
        if self.lazy_connect and not self._connected:
            self.connect()
        if not self._connected:
            raise OSError('Must call connect() first.')

    def _normalize_position(self, pos_raw: int) -> int:
        """Return signed position without modulo wrapping.

        The scs_tohost() call already converts to signed values.
        We return as-is to preserve the actual position for calibration.
        This matches INFI_hand's approach.
        """
        return pos_raw

    def _clamp_position(self, pos_raw: int) -> int:
        """Clamp position to valid servo range (0-4095)."""
        return max(POS_MIN, min(POS_MAX, pos_raw))

    @staticmethod
    def _raw_to_rad(raw: float, scale: float) -> float:
        """Convert raw motor units to radians (applies direction inversion)."""
        return raw * scale * POSITION_DIRECTION

    @staticmethod
    def _rad_to_raw(rad: float, scale: float) -> int:
        """Convert radians to raw motor units (applies direction inversion)."""
        return int((rad * POSITION_DIRECTION) / scale)

    @property
    def requires_offset_calibration(self) -> bool:
        return True

    def calibrate_offset(self, motor_id: int, upper: bool = True) -> bool:
        """Set current physical position to read as upper or lower bound.

        Uses Feetech's INST_OFSCAL command to shift the coordinate system.
        The offset is stored in EEPROM and persists across power cycles.
        Holds the bus lock for the whole transaction.

        Args:
            motor_id: Motor to calibrate.
            upper: If True, set to upper bound (3595). If False, set to lower bound (500).

        Returns:
            True if the motor acknowledged the offset command. ``False``
            means the position frame was NOT shifted; callers must check
            and must not persist limits derived from an unshifted frame.
        """
        self._check_connected()

        # When POSITION_DIRECTION inverts the read frame, "upper" maps to low raw.
        if POSITION_DIRECTION < 0:
            upper = not upper

        target_position = 3595 if upper else 500

        with self._bus_lock:
            result, error = self.packet_handler.reOfsCal(motor_id, target_position)
            if result != COMM_SUCCESS or error != 0:
                self._flush_input_buffer()
                logging.error(
                    'Failed to calibrate offset for motor %d: result=%d, error=%d',
                    motor_id, result, error
                )
                return False

        logging.info(
            'Motor %d offset calibrated: position now reads as %d',
            motor_id, target_position
        )
        return True

    @staticmethod
    def probe(
        port: str,
        baudrate: int,
        motor_ids: Sequence[int],
    ) -> bool:
        """Open ``port`` at ``baudrate`` and ping the first and last motor IDs.

        Returns True if either motor responds — i.e. the bus is speaking the
        Feetech protocol at this baudrate. Used at connect time to
        auto-detect the driver family without enabling torque.
        """
        ids = list(motor_ids)
        if not ids:
            return False
        sample = [ids[0]] if len(ids) == 1 else [ids[0], ids[-1]]

        handler = PortHandler(port)
        handler.baudrate = baudrate
        try:
            if not handler.openPort():
                return False
            packet = sms_sts(handler)
            for motor_id in sample:
                _, comm, _ = packet.ping(motor_id)
                if comm == COMM_SUCCESS:
                    return True
            return False
        finally:
            try:
                handler.closePort()
            except Exception:
                pass

    def __enter__(self):
        """Enables use as a context manager."""
        if not self._connected:
            self.connect()
        return self

    def __exit__(self, *args):
        """Enables use as a context manager."""
        self.disconnect()

    def __del__(self):
        """Automatically disconnect on destruction."""
        try:
            self.disconnect()
        except Exception:
            pass

    def write_positions_sync(
        self,
        motor_ids: Sequence[int],
        positions: np.ndarray,
        speed: Optional[int] = None,
        acc: Optional[int] = None,
        torque: Optional[int] = None,
    ) -> None:
        """Writes positions to multiple motors using sync write.

        More efficient than individual writes for multiple motors.

        Args:
            motor_ids: Motor IDs to write to.
            positions: Target positions in radians.
            speed: Movement speed (0.732 RPM per unit). Default ~60 = 44 RPM.
            acc: Acceleration (0-254). Default 50.
            torque: Torque limit (0-1000). Default 500.
        """
        self._check_connected()

        if len(motor_ids) != len(positions):
            raise ValueError('motor_ids and positions must have the same length')

        with self._bus_lock:
            speed = speed if speed is not None else self._default_speed
            acc = acc if acc is not None else self._default_acc
            torque = torque if torque is not None else self._default_torque

            # Clear any existing sync write params
            self.packet_handler.groupSyncWrite.clearParam()

            for motor_id, pos_rad in zip(motor_ids, positions):
                # STS servo mode uses raw 0–4095 (single rotation); clamp before sending.
                pos_raw = self._clamp_position(self._rad_to_raw(pos_rad, self.pos_scale))

                logging.debug(
                    'SyncWritePosEx: motor=%d, pos=%d, speed=%d, acc=%d, torque=%d',
                    motor_id, pos_raw, speed, acc, torque
                )

                self.packet_handler.SyncWritePosEx(motor_id, pos_raw, speed, acc, torque)

            # Send the sync write packet
            result = self.packet_handler.groupSyncWrite.txPacket()
            if result != COMM_SUCCESS:
                self._flush_input_buffer()
                logging.error('Sync write failed: result=%d', result)

            # Clear params for next use
            self.packet_handler.groupSyncWrite.clearParam()

    def read_position_velocity_current(self) -> MotorRead:
        """Read position, velocity, and current for all motors in one sync packet.

        Falls back to per-motor reads only if the sync transaction fails.
        Motors missing from a partial sync read keep their cached values and
        the read is reported as not ok via ``last_read_ok``.
        """
        self._check_connected()

        with self._bus_lock:
            positions = self._cached_positions.copy()
            velocities = self._cached_velocities.copy()
            currents = self._cached_currents.copy()

            # Create sync read for position, speed, load, voltage, temp, moving, current
            # From addr 56 (position) to 70 (current_h) = 15 bytes
            sync_read = GroupSyncRead(self.packet_handler, SMS_STS_PRESENT_POSITION_L, 15)

            for motor_id in self.motor_ids:
                sync_read.addParam(motor_id)

            result = sync_read.txRxPacket()
            if result != COMM_SUCCESS:
                # Late replies from the failed sync read must never be consumed
                # as answers to the per-motor fallback's transactions.
                self._flush_input_buffer()
                logging.warning('Sync read failed, falling back to individual reads')
                return self._read_state_per_motor_fallback()

            read_ok = True
            for i, motor_id in enumerate(self.motor_ids):
                available, error = sync_read.isAvailable(
                    motor_id, SMS_STS_PRESENT_POSITION_L, 2
                )
                if available:
                    pos_raw = sync_read.getData(motor_id, SMS_STS_PRESENT_POSITION_L, 2)
                    pos_signed = self.packet_handler.scs_tohost(pos_raw, 15)
                    pos_normalized = self._normalize_position(pos_signed)
                    positions[i] = self._raw_to_rad(pos_normalized, self.pos_scale)

                    vel_raw = sync_read.getData(motor_id, SMS_STS_PRESENT_SPEED_L, 2)
                    vel_signed = self.packet_handler.scs_tohost(vel_raw, 15)
                    velocities[i] = self._raw_to_rad(vel_signed, self.vel_scale)

                    cur_raw = sync_read.getData(motor_id, SMS_STS_PRESENT_CURRENT_L, 2)
                    cur_signed = self.packet_handler.scs_tohost(cur_raw, 15)
                    currents[i] = cur_signed * self.cur_scale
                else:
                    read_ok = False
                    self._flush_input_buffer()
                    logging.warning('Motor %d not available in sync read', motor_id)

            # Cache copies: callers receive the returned arrays and may mutate them.
            self._cached_positions = positions.copy()
            self._cached_velocities = velocities.copy()
            self._cached_currents = currents.copy()
            self._last_read_ok = read_ok
            return MotorRead(position=positions, velocity=velocities, current=currents)

    @property
    def last_read_ok(self) -> bool:
        return self._last_read_ok


# Register global cleanup function
atexit.register(feetech_cleanup_handler)
