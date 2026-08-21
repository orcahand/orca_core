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
from typing import Optional, Sequence
import numpy as np

from ..constants import (
    CURRENT_BASED_POSITION,
    FEETECH,
    POSITION,
    VELOCITY,
)
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

# Rates the firmware accepts but the host serial layer cannot open. A motor
# programmed to one of these would be unreachable, so never offer them.
FEETECH_HOST_UNREACHABLE_BAUD = frozenset({76_800})

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

# Speed unit is 0.732 RPM per value; the firmware caps speed to whatever the
# hardware can sustain, so a large value just means "go as fast as you can".
DEFAULT_SPEED = 1500
DEFAULT_ACC = 150  # Acceleration (0-254): faster ramp-up
DEFAULT_TORQUE_LIMIT = 500  # Torque limit (0-1000), required for motion

# Seconds between repeated out-of-range warnings for the same motor.
CLAMP_WARN_INTERVAL_S = 5.0

# Seconds between repeated warnings about the same motor's status flags. They
# latch until the condition clears, so an unthrottled log would repeat them at
# the read rate.
STATUS_WARN_INTERVAL_S = 5.0


def _raw_units_to_rad(raw: float, scale: float) -> float:
    """Convert raw motor units to radians (applies direction inversion)."""
    return raw * scale * POSITION_DIRECTION


def _rad_to_raw_units(rad: float, scale: float) -> int:
    """Convert radians to raw motor units (applies direction inversion)."""
    return int((rad * POSITION_DIRECTION) / scale)


# Radian image of the raw POS_MIN..POS_MAX span, ascending. POSITION_DIRECTION
# inverts the frame, so the whole span sits at or below zero.
POSITION_RANGE_RAD: "tuple[float, float]" = tuple(
    sorted(_raw_units_to_rad(raw, DEFAULT_POS_SCALE) + 0.0 for raw in (POS_MIN, POS_MAX))
)


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

    # STS servo mode covers one turn only, so there is no multi-turn mode and
    # current control is emulated through the torque limit.
    supports_multi_turn = False
    supported_modes = frozenset({POSITION, VELOCITY, CURRENT_BASED_POSITION})
    position_range_rad = POSITION_RANGE_RAD

    # Clients with an open port; registered on successful connect() so the
    # atexit cleanup only ever touches live connections.
    OPEN_CLIENTS = set()

    @classmethod
    def supported_baudrates(cls) -> list[int]:
        """Baud rates both the firmware and the host serial layer can use."""
        return [
            baud for baud in super().supported_baudrates()
            if baud not in FEETECH_HOST_UNREACHABLE_BAUD
        ]

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

        # Per-motor motion parameters. Torque and speed are registers the motor
        # keeps until overwritten, so they are commanded on their own rather
        # than re-sent with every position.
        self._default_acc = DEFAULT_ACC
        self._motor_torque = {mid: DEFAULT_TORQUE_LIMIT for mid in self.motor_ids}
        self._motor_speed = {mid: DEFAULT_SPEED for mid in self.motor_ids}

        # Sync read/write groups, built once per packet handler and reused: the
        # 100 Hz read path must not rebuild a param table per call.
        self._group_owner = None
        self._sync_reads: dict = {}
        self._sync_writes: dict = {}

        # Out-of-range command bookkeeping, keyed by motor id.
        self._clamp_counts: dict = {}
        self._clamp_warned_at: dict = {}

        # Last-warned time per (motor, status flags, context), and each motor's
        # known operating mode, so an unchanged mode costs no EEPROM write.
        self._status_warned_at: dict = {}
        self._motor_modes: dict = {}

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
                self._apply_port_options()

                self.packet_handler = sms_sts(self.port_handler)
                self._connected = True

                # Motor state is left as-is: connecting must never make the hand
                # stiffen, move, or burn an EEPROM write. Servo mode is enforced
                # by set_operating_mode, which disables torque first.

                self.OPEN_CLIENTS.add(self)
            except Exception:
                self._connected = False
                try:
                    self.port_handler.closePort()
                except Exception:
                    pass
                raise

    def _apply_port_options(self) -> None:
        """Advisory-lock the open port and enable low latency mode, best-effort."""
        # Exclusive-mode openers elsewhere are rejected while we hold the lock.
        try:
            import fcntl
            fcntl.flock(self.port_handler.ser.fileno(),
                        fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (ImportError, AttributeError, OSError):
            pass  # Windows (no fcntl), mocked ports, or lock unavailable — best-effort

        if hasattr(self.port_handler, 'ser') and hasattr(self.port_handler.ser, 'set_low_latency_mode'):
            try:
                self.port_handler.ser.set_low_latency_mode(True)
                logging.info('Enabled low latency mode for USB serial')
            except Exception:
                pass  # Not critical if it fails

    def _sync_read(self, address: int, length: int) -> GroupSyncRead:
        """Return the reusable sync-read group for ``address``, built once.

        The param table is filled on first use; a new packet handler (a
        reconnect, or a re-open at another baud) discards the cached groups.
        """
        self._invalidate_groups_on_new_handler()
        key = (address, length)
        reader = self._sync_reads.get(key)
        if reader is None:
            reader = GroupSyncRead(self.packet_handler, address, length)
            for motor_id in self.motor_ids:
                reader.addParam(motor_id)
            self._sync_reads[key] = reader
        return reader

    def _sync_write(self, address: int, length: int) -> GroupSyncWrite:
        """Return the reusable sync-write group for ``address``, built once."""
        self._invalidate_groups_on_new_handler()
        key = (address, length)
        writer = self._sync_writes.get(key)
        if writer is None:
            writer = GroupSyncWrite(self.packet_handler, address, length)
            self._sync_writes[key] = writer
        return writer

    def _invalidate_groups_on_new_handler(self) -> None:
        """Drop cached sync groups that belong to a superseded packet handler."""
        if self._group_owner is not self.packet_handler:
            self._sync_reads.clear()
            self._sync_writes.clear()
            self._group_owner = self.packet_handler

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
                # The next connect may find different motors on this port.
                self._motor_modes.clear()
                self.OPEN_CLIENTS.discard(self)

    def scan_for_motors(
        self,
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
            baud_rates = self.supported_baudrates()
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
        """Re-program a motor's UART baud and follow it to the new rate.

        The servo switches baud the moment the register write lands, so the
        EEPROM re-lock and the confirming ping have to go out on a port
        re-opened at ``new_baud_rate``.
        """
        if new_baud_rate not in self.supported_baudrates():
            logging.error(
                "Invalid baud rate %d. Valid: %s",
                new_baud_rate,
                self.supported_baudrates(),
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
                if result != COMM_SUCCESS:
                    self._flush_input_buffer()
                    logging.error(
                        "Failed to change baud rate: result=%d, error=%d", result, error
                    )
                    return False
                if error != 0:
                    logging.warning(
                        "Motor %d reports 0x%02X after the baud-rate write: %s",
                        motor_id, error, self.packet_handler.getRxPacketError(error)
                    )
                return self._relock_at_baudrate(motor_id, new_baud_rate)
        except Exception as e:
            logging.error("Failed to change baud rate: %s", e)
            return False

    def _relock_at_baudrate(self, motor_id: int, baudrate: int) -> bool:
        """Re-open the port at ``baudrate``, lock the EEPROM and confirm the motor."""
        if not self.port_handler.setBaudRate(baudrate):
            logging.error("Could not re-open %s at %d baud", self.port_name, baudrate)
            return False
        self.baudrate = baudrate
        self._apply_port_options()

        self.packet_handler.LockEprom(motor_id)
        _, result, _ = self.packet_handler.ping(motor_id)
        if result != COMM_SUCCESS:
            self._flush_input_buffer()
            logging.error(
                "Motor %d did not answer at %d baud after the change", motor_id, baudrate
            )
            return False
        logging.info("Changed motor %d baud rate to %d", motor_id, baudrate)
        return True

    def set_torque_enabled(
        self,
        motor_ids: Sequence[int],
        enabled: bool,
        retries: int = 3,
        retry_interval: float = 0.25,
    ) -> "list[int]":
        """Sets whether torque is enabled for the motors.

        Holds the bus lock for the entire call, including any retry sleeps.

        Only a failed transaction (no/garbled reply) is retried. A motor that
        answers with a status error — a latched overload or overheat flag —
        acknowledged the write: its condition is logged, not treated as a
        failure, so one flagged motor cannot report itself permanently dead.

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
                    if result != COMM_SUCCESS:
                        failed_ids.append(motor_id)
                        self._flush_input_buffer()
                    elif error != 0:
                        self._log_status_error(motor_id, error, 'torque write')

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

    def _log_status_error(self, motor_id: int, error: int, context: str) -> None:
        """Log a motor's decoded status byte; the transaction itself succeeded.

        Repeats of the same flags on the same motor are throttled, so a latched
        condition costs one line per interval instead of one per read.
        """
        key = (motor_id, error, context)
        now = time.monotonic()
        if now - self._status_warned_at.get(key, -STATUS_WARN_INTERVAL_S) < \
                STATUS_WARN_INTERVAL_S:
            return
        self._status_warned_at[key] = now
        logging.warning(
            'Motor %d reports 0x%02X after %s: %s',
            motor_id, error, context, self.packet_handler.getRxPacketError(error)
        )

    def read_hardware_error(self, motor_id: int) -> Optional[int]:
        """Reads a motor's status byte via ping.

        Returns:
            The raw error byte (see the ``ERRBIT_*`` flags), or None if the
            motor did not answer.
        """
        self._check_connected()
        with self._bus_lock:
            _, result, error = self.packet_handler.ping(motor_id)
            if result != COMM_SUCCESS:
                self._flush_input_buffer()
                return None
            return error

    def set_operating_mode(self, motor_ids: Sequence[int], mode: int) -> None:
        """Sets the operating mode for the specified motors.

        Feetech only supports:
        - Mode 0: Servo mode (position control)
        - Mode 1: Wheel mode (velocity control)

        Unsupported Dynamixel modes are mapped to servo mode with a warning.
        The mode register lives in EEPROM, which wears out: a motor already in
        the requested mode is left alone, and only a real change is written,
        bracketed by the EEPROM unlock/lock pair. Motors that stay unreachable,
        or whose mode write is not acked, are logged and skipped.
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

            # Only velocity mode (1) maps to wheel mode; all others use servo mode
            feetech_mode = 1 if mode == 1 else 0

            mode_set_ids = []
            for motor_id in acked_ids:
                if self._current_mode(motor_id) == feetech_mode:
                    mode_set_ids.append(motor_id)
                    continue
                self._motor_modes.pop(motor_id, None)
                self.packet_handler.unLockEprom(motor_id)
                result, error = self.packet_handler.write1ByteTxRx(
                    motor_id, SMS_STS_MODE, feetech_mode
                )
                self.packet_handler.LockEprom(motor_id)
                if result != COMM_SUCCESS:
                    self._flush_input_buffer()
                    logging.error(
                        'Mode write not acked by motor %d (result=%d); skipping it.',
                        motor_id, result
                    )
                    continue
                if error != 0:
                    self._log_status_error(motor_id, error, 'mode write')
                self._motor_modes[motor_id] = feetech_mode
                mode_set_ids.append(motor_id)

            if mode_set_ids:
                # Speed and acceleration are set here, once, so position
                # commands do not re-arm the servo's own motion profile.
                self._write_profile_params(mode_set_ids)
                self.set_torque_enabled(mode_set_ids, True)

    def _current_mode(self, motor_id: int) -> "int | None":
        """The motor's operating mode, cached from what this client last wrote
        or read back from the servo. None when the motor did not answer, so the
        caller writes rather than assumes."""
        cached = self._motor_modes.get(motor_id)
        if cached is not None:
            return cached
        mode, result, error = self.packet_handler.read1ByteTxRx(motor_id, SMS_STS_MODE)
        if result != COMM_SUCCESS:
            self._flush_input_buffer()
            return None
        if error != 0:
            self._log_status_error(motor_id, error, 'mode read')
        self._motor_modes[motor_id] = mode
        return mode

    def _write_profile_params(self, motor_ids: Sequence[int]) -> None:
        """Sync-write acceleration, torque limit and goal speed for ``motor_ids``.

        The torque limit is part of this: nothing moves while it reads zero,
        and position commands no longer carry it.
        """
        acc_write = self._sync_write(SMS_STS_ACC, 1)
        acc_write.clearParam()
        # Registers 44-47 are the torque limit followed by the goal speed.
        motion_write = self._sync_write(SMS_STS_GOAL_TIME_L, 4)
        motion_write.clearParam()
        for motor_id in motor_ids:
            acc_write.addParam(motor_id, [self._default_acc])
            motion_write.addParam(
                motor_id, self._torque_bytes(motor_id) + self._speed_bytes(motor_id))
        for writer, name in ((acc_write, 'acceleration'), (motion_write, 'torque/speed')):
            if writer.txPacket() != COMM_SUCCESS:
                self._flush_input_buffer()
                logging.error('Sync write of the motion-profile %s failed', name)
            writer.clearParam()

    def _speed_bytes(self, motor_id: int) -> "list[int]":
        """Little-endian goal-speed bytes for ``motor_id``."""
        speed = self.packet_handler.scs_toscs(
            self._motor_speed.get(motor_id, DEFAULT_SPEED), 15)
        return [self.packet_handler.scs_lobyte(speed),
                self.packet_handler.scs_hibyte(speed)]

    def _torque_bytes(self, motor_id: int) -> "list[int]":
        """Little-endian torque-limit bytes for ``motor_id``."""
        torque = self._motor_torque.get(motor_id, DEFAULT_TORQUE_LIMIT)
        return [self.packet_handler.scs_lobyte(torque),
                self.packet_handler.scs_hibyte(torque)]

    def _read_word(self, motor_id: int, address: int, field: str) -> "int | None":
        """Read one 2-byte register, or None when the motor did not answer.

        A status flag in an answered read qualifies the motor, not the data:
        it is logged and the value returned.
        """
        raw, result, error = self.packet_handler.read2ByteTxRx(motor_id, address)
        if result != COMM_SUCCESS:
            self._flush_input_buffer()
            logging.warning(
                'Failed to read %s for motor %d: result=%d', field, motor_id, result
            )
            return None
        if error != 0:
            self._log_status_error(motor_id, error, f'{field} read')
        return raw

    def _read_state_per_motor_fallback(self) -> MotorRead:
        """Per-motor read of position/velocity/current; used only when sync
        read fails. Fields the motor never answered keep their cached value and
        the read is reported as not ok via ``last_read_ok``."""
        self._check_connected()

        with self._bus_lock:
            positions = self._cached_positions.copy()
            velocities = self._cached_velocities.copy()
            currents = self._cached_currents.copy()
            read_ok = True

            for i, motor_id in enumerate(self.motor_ids):
                pos_raw = self._read_word(
                    motor_id, SMS_STS_PRESENT_POSITION_L, 'position')
                if pos_raw is not None:
                    pos_signed = self.packet_handler.scs_tohost(pos_raw, 15)
                    pos_normalized = self._normalize_position(pos_signed)
                    positions[i] = self._raw_to_rad(pos_normalized, self.pos_scale)
                else:
                    read_ok = False

                vel_raw = self._read_word(
                    motor_id, SMS_STS_PRESENT_SPEED_L, 'velocity')
                if vel_raw is not None:
                    vel_signed = self.packet_handler.scs_tohost(vel_raw, 15)
                    velocities[i] = self._raw_to_rad(vel_signed, self.vel_scale)
                else:
                    read_ok = False

                cur_raw = self._read_word(
                    motor_id, SMS_STS_PRESENT_CURRENT_L, 'current')
                if cur_raw is not None:
                    cur_signed = self.packet_handler.scs_tohost(cur_raw, 15)
                    currents[i] = cur_signed * self.cur_scale
                else:
                    read_ok = False

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

            sync_read = self._sync_read(SMS_STS_PRESENT_TEMPERATURE, 1)
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
                sync_read = self._sync_read(SMS_STS_MOVING, 1)
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
        acc: Optional[int] = None,
        torque: Optional[int] = None,
    ) -> None:
        """Writes desired positions to the motors in a single broadcast packet.

        With no explicit motion parameters this takes the position-only hot
        path: speed, acceleration and torque already live in the motor's
        registers, so re-sending them per command would re-arm the servo's own
        trapezoidal profile underneath the outer control loop. Passing any of
        them routes through :meth:`write_positions_sync` instead.

        Args:
            motor_ids: Motor IDs to write to.
            positions: Target positions in radians.
            speed: Movement speed (0.732 RPM per unit).
            acc: Acceleration (0-254).
            torque: Torque limit (0-1000).
        """
        if speed is None and acc is None and torque is None:
            self._write_positions_only(motor_ids, positions)
            return
        self.write_positions_sync(
            motor_ids, positions, speed=speed, acc=acc, torque=torque)

    def _write_positions_only(
        self,
        motor_ids: Sequence[int],
        positions: np.ndarray,
    ) -> None:
        """Sync-write goal positions alone, leaving the profile registers untouched."""
        self._check_connected()

        if len(motor_ids) != len(positions):
            raise ValueError('motor_ids and positions must have the same length')

        with self._bus_lock:
            sync_write = self._sync_write(SMS_STS_GOAL_POSITION_L, 2)
            sync_write.clearParam()
            for motor_id, pos_rad in zip(motor_ids, positions):
                pos_raw = self._clamp_position(
                    self._rad_to_raw(pos_rad, self.pos_scale), motor_id)
                pos_scs = self.packet_handler.scs_toscs(pos_raw, 15)
                sync_write.addParam(
                    motor_id,
                    [self.packet_handler.scs_lobyte(pos_scs),
                     self.packet_handler.scs_hibyte(pos_scs)],
                )
            if sync_write.txPacket() != COMM_SUCCESS:
                self._flush_input_buffer()
                logging.error('Sync write of goal positions failed')
            sync_write.clearParam()

    def write_desired_current(
        self,
        motor_ids: Sequence[int],
        currents: np.ndarray,
    ) -> None:
        """Writes desired currents (torque limits) to the motors.

        Feetech uses torque limiting (0-1000) instead of direct current
        control, so each current maps to one motor's torque limit (1 mA ≈ 1
        torque unit) and is written to the bus immediately — the limit must
        not wait for a following position command.

        Args:
            motor_ids: Motor IDs to configure.
            currents: Desired current limits in mA. Mapped to torque (0-1000).
        """
        self._check_connected()

        if len(motor_ids) != len(currents):
            raise ValueError('motor_ids and currents must have the same length')

        with self._bus_lock:
            sync_write = self._sync_write(SMS_STS_GOAL_TIME_L, 2)
            sync_write.clearParam()
            failed_ids = []
            for motor_id, current in zip(motor_ids, currents):
                self._motor_torque[motor_id] = int(np.clip(abs(current), 0, 1000))
                if not sync_write.addParam(motor_id, self._torque_bytes(motor_id)):
                    failed_ids.append(motor_id)
            if sync_write.txPacket() != COMM_SUCCESS:
                self._flush_input_buffer()
                failed_ids = list(motor_ids)
            sync_write.clearParam()

        if failed_ids:
            logging.error(
                'Torque limit not written for %d motor(s): %s',
                len(failed_ids), str(failed_ids)
            )

    def write_profile_velocity(
        self,
        motor_ids: Sequence[int],
        profile_velocity: np.ndarray,
    ) -> None:
        """Writes the per-motor goal speed, in raw units of 0.732 RPM.

        Args:
            motor_ids: Motor IDs to write to.
            profile_velocity: Speed limits in raw motor units.
        """
        self._check_connected()

        if len(motor_ids) != len(profile_velocity):
            raise ValueError(
                'motor_ids and profile_velocity must have the same length')

        with self._bus_lock:
            sync_write = self._sync_write(SMS_STS_GOAL_SPEED_L, 2)
            sync_write.clearParam()
            for motor_id, speed in zip(motor_ids, profile_velocity):
                self._motor_speed[motor_id] = int(np.clip(abs(speed), 0, 32766))
                sync_write.addParam(motor_id, self._speed_bytes(motor_id))
            if sync_write.txPacket() != COMM_SUCCESS:
                self._flush_input_buffer()
                logging.error('Sync write of the motion-profile speed failed')
            sync_write.clearParam()

    def read_status_is_done_moving(self) -> bool:
        """Returns True when no motor reports the MOVING flag set."""
        self._check_connected()

        with self._bus_lock:
            sync_read = self._sync_read(SMS_STS_MOVING, 1)
            if sync_read.txRxPacket() != COMM_SUCCESS:
                self._flush_input_buffer()
                return False
            for motor_id in self.motor_ids:
                available, _ = sync_read.isAvailable(motor_id, SMS_STS_MOVING, 1)
                if not available:
                    self._flush_input_buffer()
                    return False
                if sync_read.getData(motor_id, SMS_STS_MOVING, 1) != 0:
                    return False
        return True

    def check_connected(self) -> None:
        """Ensures the client is connected."""
        self._check_connected()

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

    def _clamp_position(self, pos_raw: int, motor_id: Optional[int] = None) -> int:
        """Clamp a commanded position to the servo range (0-4095).

        Saturating silently would hide sign/frame errors and wrap offsets, so
        every clamp is counted per motor and warned about at a bounded rate.
        """
        clamped = max(POS_MIN, min(POS_MAX, pos_raw))
        if clamped == pos_raw:
            return clamped

        count = self._clamp_counts.get(motor_id, 0) + 1
        self._clamp_counts[motor_id] = count
        now = time.monotonic()
        if now - self._clamp_warned_at.get(motor_id, -CLAMP_WARN_INTERVAL_S) >= \
                CLAMP_WARN_INTERVAL_S:
            self._clamp_warned_at[motor_id] = now
            logging.warning(
                'Motor %s commanded to raw position %d, outside %d-%d; clamped to '
                '%d (%d out-of-range commands so far)',
                motor_id, pos_raw, POS_MIN, POS_MAX, clamped, count
            )
        return clamped

    def clamped_command_counts(self) -> dict:
        """Out-of-range command count per motor id since the client was created."""
        return dict(self._clamp_counts)

    @staticmethod
    def _raw_to_rad(raw: float, scale: float) -> float:
        """Convert raw motor units to radians (applies direction inversion)."""
        return _raw_units_to_rad(raw, scale)

    @staticmethod
    def _rad_to_raw(rad: float, scale: float) -> int:
        """Convert radians to raw motor units (applies direction inversion)."""
        return _rad_to_raw_units(rad, scale)

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
        """Writes position plus motion profile to multiple motors using sync write.

        Each motor's packet carries its own torque limit (as set by
        :meth:`write_desired_current`) unless ``torque`` overrides it for the
        whole call.

        Args:
            motor_ids: Motor IDs to write to.
            positions: Target positions in radians.
            speed: Movement speed (0.732 RPM per unit).
            acc: Acceleration (0-254).
            torque: Torque limit (0-1000), applied to every listed motor.
        """
        self._check_connected()

        if len(motor_ids) != len(positions):
            raise ValueError('motor_ids and positions must have the same length')

        with self._bus_lock:
            acc = acc if acc is not None else self._default_acc

            # Clear any existing sync write params
            self.packet_handler.groupSyncWrite.clearParam()

            for motor_id, pos_rad in zip(motor_ids, positions):
                # STS servo mode uses raw 0–4095 (single rotation); clamp before sending.
                pos_raw = self._clamp_position(
                    self._rad_to_raw(pos_rad, self.pos_scale), motor_id)
                motor_speed = (
                    speed if speed is not None
                    else self._motor_speed.get(motor_id, DEFAULT_SPEED)
                )
                motor_torque = (
                    torque if torque is not None
                    else self._motor_torque.get(motor_id, DEFAULT_TORQUE_LIMIT)
                )

                logging.debug(
                    'SyncWritePosEx: motor=%d, pos=%d, speed=%d, acc=%d, torque=%d',
                    motor_id, pos_raw, motor_speed, acc, motor_torque
                )

                self.packet_handler.SyncWritePosEx(
                    motor_id, pos_raw, motor_speed, acc, motor_torque)

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

            # Sync read of position, speed, load, voltage, temp, moving, current:
            # from addr 56 (position) to 70 (current_h) = 15 bytes.
            sync_read = self._sync_read(SMS_STS_PRESENT_POSITION_L, 15)
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
                if not available:
                    read_ok = False
                    self._flush_input_buffer()
                    logging.warning('Motor %d not available in sync read', motor_id)
                    continue

                # A latched status flag (overload, overheat) describes the
                # motor's condition, not the packet: the data below came back
                # in this transaction and is fresh. Only missing data makes a
                # read stale, so a stalled motor -- what tensioning is for --
                # still reports its position.
                if error != 0:
                    self._log_status_error(motor_id, error, 'sync read')

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
