# Copyright 2019 The ROBEL Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Communication using the DynamixelSDK."""

import atexit
import logging
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple, Union
import numpy as np

from ..constants import DYNAMIXEL
from .motor_client import MotorClient, MotorRead

PROTOCOL_VERSION = 2.0

# Per-motor fallback bounds: skip a motor whose individual read failed for
# this long, and allow at most one full-bus fallback sweep per interval.
FALLBACK_MOTOR_COOLDOWN_S = 2.0
FALLBACK_FULL_SWEEP_MIN_INTERVAL_S = 1.0

# The following addresses assume XC motors.
# see https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/ for control table
ADDR_ID = 7
ADDR_BAUD_RATE = 8
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_GOAL_PWM = 100
ADDR_GOAL_CURRENT = 102
ADDR_PROFILE_VELOCITY = 112
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POS_VEL_CUR = 126
ADDR_MOVING_STATUS = 123
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PRESENT_TEMPERATURE = 146

# Data Byte Length
LEN_OPERATING_MODE = 1
LEN_HARDWARE_ERROR_STATUS = 1
LEN_PRESENT_POSITION = 4
LEN_PRESENT_VELOCITY = 4
LEN_PRESENT_CURRENT = 2
LEN_PRESENT_POS_VEL_CUR = 10
LEN_GOAL_POSITION = 4
LEN_GOAL_PWM = 2
LEN_GOAL_CURRENT = 2
LEN_PROFILE_VELOCITY = 4
LEN_MOVING_STATUS = 1
LEN_PRESENT_TEMPERATURE = 1

DEFAULT_POS_SCALE = 2.0 * np.pi / 4096  # 0.088 degrees
# See http://emanual.robotis.com/docs/en/dxl/x/xh430-v210/#goal-velocity
DEFAULT_VEL_SCALE = 0.229 * 2.0 * np.pi / 60.0  # 0.229 rpm
# The current registers are already in mA, the unit the control table and
# the Dynamixel Wizard both use, so readings need no conversion.
DEFAULT_CUR_SCALE = 1.0

# A rebooting motor is off the bus while its firmware restarts; writes sent
# before it answers again are lost.
MOTOR_REBOOT_SETTLE_S = 0.3

# Baud rate mapping for Dynamixel motors, see https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#baud-rate
BAUD_RATE_MAP = {
    9600: 0,
    57600: 1,
    115200: 2,
    1000000: 3,
    2000000: 4,
    3000000: 5,
    4000000: 6,
    4500000: 7,
    10500000: 8,
}

# Dynamixel model number to name mapping (see table 2.2. @ https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/ as reference)
DYNAMIXEL_MODELS = {
    1220: 'XC330-T288-T',
    1080: 'XC430-T240BB-T',
}

def dynamixel_cleanup_handler():
    """Disconnect every open Dynamixel client at interpreter exit."""
    DynamixelClient.cleanup_open_clients()


def signed_to_unsigned(value: int, size: int) -> int:
    """Converts the given value to its unsigned representation."""
    if value < 0:
        bit_size = 8 * size
        max_value = (1 << bit_size) - 1
        value = max_value + value
    return value


def unsigned_to_signed(value: int, size: int) -> int:
    """Converts the given value from its unsigned representation."""
    bit_size = 8 * size
    if (value & (1 << (bit_size - 1))) != 0:
        value = -((1 << bit_size) - value)
    return value


class DynamixelClient(MotorClient):
    """Client for communicating with Dynamixel motors.

    NOTE: This only supports Protocol 2.

    Thread safety / lock contract:
        All bus I/O is serialized by a single reentrant lock
        (``self._bus_lock``). Every public method that touches the port holds
        the lock for the whole transaction (request + status reply), so
        transactions from different threads can never interleave on the wire.
        Callers therefore do not need their own locking around individual
        calls; external locks remain harmless but redundant.

        Some methods hold the lock for longer than a single transaction:
        ``set_torque_enabled`` keeps it across its retry loop (including the
        ``retry_interval`` sleeps between failed attempts), and
        ``check_overload_and_reboot`` holds it across the motor reboot
        sequence. Other threads block on the bus for that duration, which is
        why reboots are driven by explicit calls rather than by reads.

        On any failed transaction (comm error/timeout), the OS receive buffer
        is flushed before the lock is released, so a late status reply can
        never be consumed as the response to a later transaction.
    """

    motor_type = DYNAMIXEL
    factory_default_id = 1
    factory_default_baudrate = 57600
    baud_rate_map = BAUD_RATE_MAP

    # Clients with an open port; registered on successful connect() so the
    # atexit cleanup only ever touches live connections.
    OPEN_CLIENTS = set()

    def __init__(self,
                 motor_ids: Sequence[int],
                 port: str = '/dev/ttyUSB0',
                 baudrate: int = 1000000,
                 lazy_connect: bool = False,
                 pos_scale: Optional[float] = None,
                 vel_scale: Optional[float] = None,
                 cur_scale: Optional[float] = None):
        """Initializes a new client.

        Args:
            motor_ids: All motor IDs being used by the client.
            port: The Dynamixel device to talk to. e.g.
                - Linux: /dev/ttyUSB0
                - Mac: /dev/tty.usbserial-*
                - Windows: COM1
            baudrate: The Dynamixel baudrate to communicate with.
            lazy_connect: If True, automatically connects when calling a method
                that requires a connection, if not already connected.
            pos_scale: The scaling factor for the positions. This is
                motor-dependent. If not provided, uses the default scale.
            vel_scale: The scaling factor for the velocities. This is
                motor-dependent. If not provided uses the default scale.
            cur_scale: The scaling factor for the currents. This is
                motor-dependent. If not provided uses the default scale.
        """
        import dynamixel_sdk
        self.dxl = dynamixel_sdk

        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate
        self.lazy_connect = lazy_connect

        self.port_handler = self.dxl.PortHandler(port)
        self.packet_handler = self.dxl.PacketHandler(PROTOCOL_VERSION)

        # RLock: alert handling re-enters from within a locked read/write path.
        self._bus_lock = threading.RLock()

        self._pos_vel_cur_reader = DynamixelPosVelCurReader(
            self,
            self.motor_ids,
            pos_scale=pos_scale if pos_scale is not None else DEFAULT_POS_SCALE,
            vel_scale=vel_scale if vel_scale is not None else DEFAULT_VEL_SCALE,
            cur_scale=cur_scale if cur_scale is not None else DEFAULT_CUR_SCALE,
        )
        
        self._temp_reader = DynamixelTempReader(
            self,
            self.motor_ids,
            address=ADDR_PRESENT_TEMPERATURE,
            size=LEN_PRESENT_TEMPERATURE,
        )
        
        self._moving_status_reader = DynamixelReader(self, self.motor_ids, ADDR_MOVING_STATUS, LEN_MOVING_STATUS)
        self._sync_writers = {}
        self._operating_modes = {}
        # Last Goal Current written per motor, re-applied after a reboot.
        self._goal_currents: Dict[int, int] = {}
        # Motors seen with the Alert bit set, drained by take_hardware_alerts().
        self._hardware_alerts: Dict[int, int] = {}
        self._alerts_lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self.port_handler.is_open

    def connect(self):
        """Connects to the Dynamixel motors."""
        assert not self.is_connected, 'Client is already connected.'

        with self._bus_lock:
            if self.port_handler.openPort():
                logging.info('Succeeded to open port: %s', self.port_name)
            else:
                raise OSError(
                    ('Failed to open port at {} (Check that the device is powered '
                     'on and connected to your computer).').format(self.port_name))

            # A failure past this point must not leave the port open (and
            # advisory-locked) with no registered owner to close it.
            try:
                if self.port_handler.setBaudRate(self.baudrate):
                    logging.info('Succeeded to set baudrate to %d', self.baudrate)
                else:
                    raise OSError(
                        ('Failed to set the baudrate to {} (Ensure that the device was '
                         'configured for this baudrate).').format(self.baudrate))

                # Advisory-lock the port so exclusive-mode openers elsewhere are rejected.
                try:
                    import fcntl
                    fcntl.flock(self.port_handler.ser.fileno(),
                                fcntl.LOCK_EX | fcntl.LOCK_NB)
                except (ImportError, AttributeError, OSError):
                    pass  # Windows (no fcntl), mocked ports, or lock unavailable — best-effort

                # Enable low latency mode for faster communication (~500 Hz vs ~30 Hz)
                if hasattr(self.port_handler, 'ser') and hasattr(self.port_handler.ser, 'set_low_latency_mode'):
                    try:
                        self.port_handler.ser.set_low_latency_mode(True)
                        logging.info('Enabled low latency mode for USB serial')
                    except Exception:
                        pass  # Not critical if it fails

                # Probed before any reboot below, so a motor still coming
                # back up cannot be mistaken for firmware without support.
                self._probe_fast_sync_read()

                # Clear any pre-existing hardware errors.
                self.check_overload_and_reboot(self.motor_ids)

                # Torque is left as-is: connecting must never make the hand
                # stiffen or move. Callers opt in via enable_torque()/init_joints().

                self.OPEN_CLIENTS.add(self)
            except Exception:
                try:
                    self.port_handler.closePort()
                except Exception:
                    pass
                raise

    def _probe_fast_sync_read(self) -> None:
        """Adopt fast sync read on every reader whose motors answer it."""
        readers = (self._pos_vel_cur_reader, self._temp_reader,
                   self._moving_status_reader)
        adopted = 0
        for reader in readers:
            try:
                adopted += bool(reader.probe_fast_sync_read())
            except Exception:
                logging.debug('fast sync read probe failed', exc_info=True)
        if adopted == len(readers):
            logging.info('Fast sync read supported: reads cost one bus '
                         'turnaround instead of one per motor')
        else:
            logging.info('Fast sync read unavailable on %d of %d readers; '
                         'using per-motor sync read',
                         len(readers) - adopted, len(readers))

    @staticmethod
    def probe(port: str, baudrate: int, motor_ids: Sequence[int]) -> bool:
        """Open ``port`` at ``baudrate`` and ping the first and last motor IDs.

        Returns True if either motor responds — i.e. the bus is speaking the
        Dynamixel Protocol 2.0 at this baudrate. Used at connect time to
        auto-detect the driver family without enabling torque.
        """
        import dynamixel_sdk

        ids = list(motor_ids)
        if not ids:
            return False
        sample = [ids[0]] if len(ids) == 1 else [ids[0], ids[-1]]

        handler = dynamixel_sdk.PortHandler(port)
        try:
            if not handler.openPort():
                return False
            if not handler.setBaudRate(baudrate):
                return False
            packet = dynamixel_sdk.PacketHandler(PROTOCOL_VERSION)
            for motor_id in sample:
                _, comm, _ = packet.ping(handler, motor_id)
                if comm == dynamixel_sdk.COMM_SUCCESS:
                    return True
            return False
        finally:
            try:
                handler.closePort()
            except Exception:
                pass

    def disconnect(self):
        """Disconnects from the Dynamixel device.

        The port is always closed and the client deregistered, even when the
        final torque-disable raises (e.g. the serial link is already gone);
        that exception propagates after cleanup.
        """
        if not self.is_connected:
            return
        if self.port_handler.is_using:
            logging.error('Port handler in use; cannot disconnect.')
            return
        with self._bus_lock:
            try:
                # Ensure motors are disabled at the end.
                self.set_torque_enabled(self.motor_ids, False, retries=0)
            finally:
                self.port_handler.closePort()
                self.OPEN_CLIENTS.discard(self)

    def set_torque_enabled(self,
                           motor_ids: Sequence[int],
                           enabled: bool,
                           retries: int = 3,
                           retry_interval: float = 0.25) -> List[int]:
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
        with self._bus_lock:
            remaining_ids = list(motor_ids)
            while remaining_ids:
                remaining_ids = self.write_byte(
                    remaining_ids,
                    int(enabled),
                    ADDR_TORQUE_ENABLE,
                )
                if not remaining_ids:
                    break
                logging.error('Could not set torque %s for IDs: %s',
                              'enabled' if enabled else 'disabled',
                              str(remaining_ids))
                if retries == 0:
                    break
                self._flush_input_buffer()
                time.sleep(retry_interval)
                retries -= 1
            return remaining_ids

    def set_operating_mode(self, motor_ids: Sequence[int], mode_value: int):
        """
        see https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#operating-mode11
        0: current control mode
        1: velocity control mode
        3: position control mode
        4: multi-turn position control mode
        5: current-based position control mode
        """
        with self._bus_lock:
            # EEPROM data can only be written when torque is disabled; motors
            # that never acked the disable are skipped, not written blind.
            failed_ids = self.set_torque_enabled(motor_ids, False)
            if failed_ids:
                logging.error(
                    'Skipping mode change for motors that did not ack torque '
                    'disable: %s', str(failed_ids))
            acked_ids = [mid for mid in motor_ids if mid not in failed_ids]
            if not acked_ids:
                return
            self.sync_write(acked_ids, [mode_value]*len(acked_ids), ADDR_OPERATING_MODE, LEN_OPERATING_MODE)
            self.set_torque_enabled(acked_ids, True)
            for mid in acked_ids:
                self._operating_modes[mid] = mode_value

    def read_position_velocity_current(self) -> MotorRead:
        """Return positions, velocities, and currents as a ``MotorRead`` snapshot.

        Overload detection is handled reactively via the Alert bit in
        handle_packet_result, so no extra bulk read is needed here.
        """
        pos, vel, cur = self._pos_vel_cur_reader.read()
        return MotorRead(position=pos, velocity=vel, current=cur)

    @property
    def last_read_ok(self) -> bool:
        return self._pos_vel_cur_reader.last_read_ok

    def read_status_is_done_moving(self) -> bool:
        """Returns the last bit of moving status"""
        moving_status = self._moving_status_reader.read().astype(np.int8)
        return np.bitwise_and(moving_status, np.array([0x01] * len(moving_status)).astype(np.int8))

    def read_temperature(self) -> np.ndarray:
        """Reads and returns the present temperature for each motor (in deg C)."""
        return self._temp_reader.read()

    def write_desired_pos(self, motor_ids: Sequence[int],
                          positions: np.ndarray):
        """Writes the given desired positions.

        Args:
            motor_ids: The motor IDs to write to.
            positions: The joint angles in radians to write.
        """
        assert len(motor_ids) == len(positions)

        # Convert to Dynamixel position space.
        positions = positions / self._pos_vel_cur_reader.pos_scale
        times = self.sync_write(motor_ids, positions, ADDR_GOAL_POSITION,
                        LEN_GOAL_POSITION)
        return times

    def write_desired_current(self, motor_ids: Sequence[int], current: np.ndarray):
        """Write Goal Current (102) in mA — the bound the position controller
        clips its output to, not the hardware ceiling (Current Limit, 38)."""
        assert len(motor_ids) == len(current)
        self.sync_write(motor_ids, current, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT)
        # Goal Current lives in RAM, which a reboot wipes. Remembering it is
        # what lets reboot_motor put the ceiling back.
        for motor_id, value in zip(motor_ids, current):
            self._goal_currents[int(motor_id)] = int(value)

    def write_profile_velocity(self, motor_ids: Sequence[int], profile_velocity: np.ndarray):
            assert len(motor_ids) == len(profile_velocity)

            self.sync_write(motor_ids, profile_velocity, ADDR_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY)

    def write_byte(
            self,
            motor_ids: Sequence[int],
            value: int,
            address: int,
    ) -> Sequence[int]:
        """Writes a value to the motors.

        Args:
            motor_ids: The motor IDs to write to.
            value: The value to write to the control table.
            address: The control table address to write to.

        Returns:
            A list of IDs that were unsuccessful.
        """
        self.check_connected()
        errored_ids = []
        with self._bus_lock:
            for motor_id in motor_ids:
                comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, motor_id, address, value)
                success = self.handle_packet_result(
                    comm_result, dxl_error, motor_id, context='write_byte')
                if not success:
                    errored_ids.append(motor_id)
                    self._flush_input_buffer()
        return errored_ids

    def sync_write(self, motor_ids: Sequence[int],
                   values: Sequence[Union[int, float]], address: int,
                   size: int):
        """Writes values to a group of motors.

        Args:
            motor_ids: The motor IDs to write to.
            values: The values to write.
            address: The control table address to write to.
            size: The size of the control table value being written to.
        """
        times = [time.monotonic()]
        self.check_connected()
        with self._bus_lock:
            key = (address, size)
            if key not in self._sync_writers:
                self._sync_writers[key] = self.dxl.GroupSyncWrite(
                    self.port_handler, self.packet_handler, address, size)
            sync_writer = self._sync_writers[key]
            times.append(time.monotonic())
            errored_ids = []
            for motor_id, desired_pos in zip(motor_ids, values):
                value = signed_to_unsigned(int(desired_pos), size=size)
                value = value.to_bytes(size, byteorder='little')
                success = sync_writer.addParam(motor_id, value)
                if not success:
                    errored_ids.append(motor_id)

            if errored_ids:
                logging.error('Sync write failed for: %s', str(errored_ids))
            times.append(time.monotonic())

            comm_result = sync_writer.txPacket()
            self.handle_packet_result(comm_result, context='sync_write')
            times.append(time.monotonic())

            sync_writer.clearParam()
        times.append(time.monotonic())
        return times

    def reboot_motor(self, motor_id: int):
        """Reboot one motor and put its RAM settings back.

        A reboot clears RAM to defaults, and the current ceiling lives there
        (Goal Current, unlike Operating Mode, which is EEPROM and survives).
        Without this a recovered motor runs uncapped until something calls
        set_max_current again, drawing far more than its configured limit.
        Torque is deliberately left off: re-energizing is the caller's call.
        """
        with self._bus_lock:
            comm_result, dxl_error = self.packet_handler.reboot(self.port_handler, motor_id)
            success = self.handle_packet_result(
                comm_result, dxl_error, motor_id, context='reboot')
            if not success:
                self._flush_input_buffer()
                return
            self._restore_ram_after_reboot(motor_id)

    def _restore_ram_after_reboot(self, motor_id: int) -> None:
        """Re-apply the RAM settings a reboot cleared, once the motor answers."""
        goal_current = self._goal_currents.get(int(motor_id))
        if goal_current is None:
            return
        time.sleep(MOTOR_REBOOT_SETTLE_S)
        self.sync_write([motor_id], [goal_current], ADDR_GOAL_CURRENT,
                        LEN_GOAL_CURRENT)

    def read_hardware_error(self, motor_id: int) -> Optional[int]:
        """Reads the Hardware Error Status register (address 70).

        Returns:
            The raw error byte, or None if the read failed (no/garbled reply).
        """
        with self._bus_lock:
            value, comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor_id, ADDR_HARDWARE_ERROR_STATUS)
            if comm_result != self.dxl.COMM_SUCCESS:
                self._flush_input_buffer()
                return None
            return value

    def read_hardware_errors(
        self, motor_ids: Sequence[int]
    ) -> "dict[int, Optional[int]]":
        """Hardware Error Status for every motor in one GroupSyncRead.

        Same address and length for all of them, which is exactly what sync
        read is for: seventeen round trips collapse into one transaction. A
        motor that does not answer comes back ``None`` rather than failing the
        whole sweep, and a failed transaction falls back to reading each motor
        individually so a single bad reply cannot blind the caller.
        """
        motor_ids = [int(mid) for mid in motor_ids]
        if not motor_ids:
            return {}
        with self._bus_lock:
            reader = self.dxl.GroupSyncRead(
                self.port_handler, self.packet_handler,
                ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS)
            try:
                for mid in motor_ids:
                    if not reader.addParam(mid):
                        return super().read_hardware_errors(motor_ids)
                comm_result = reader.txRxPacket()
                if comm_result != self.dxl.COMM_SUCCESS:
                    self._flush_input_buffer()
                    return super().read_hardware_errors(motor_ids)
                out: "dict[int, Optional[int]]" = {}
                for mid in motor_ids:
                    if reader.isAvailable(mid, ADDR_HARDWARE_ERROR_STATUS,
                                          LEN_HARDWARE_ERROR_STATUS):
                        out[mid] = reader.getData(
                            mid, ADDR_HARDWARE_ERROR_STATUS,
                            LEN_HARDWARE_ERROR_STATUS)
                    else:
                        out[mid] = None
                return out
            finally:
                reader.clearParam()

    def check_overload_and_reboot(self, motor_ids: Sequence[int]) -> list:
        """Checks for overload errors and reboots affected motors.

        Returns list of motor IDs that were rebooted.
        """
        OVERLOAD_BIT = 0x20
        rebooted = []
        with self._bus_lock:
            for mid in motor_ids:
                error_status = self.read_hardware_error(mid)
                if error_status is None:
                    error_status = self.read_hardware_error(mid)
                if error_status is None:
                    logging.warning(
                        'Could not read hardware error status for motor %d; '
                        'skipping overload check.', mid)
                    continue
                if error_status & OVERLOAD_BIT:
                    logging.warning(f'Motor {mid} overload detected (error=0x{error_status:02X}), rebooting...')
                    self.reboot_motor(mid)
                    rebooted.append(mid)
            if rebooted:
                time.sleep(0.3)
                for mid in rebooted:
                    mode = self._operating_modes.get(mid)
                    if mode is not None:
                        # Reboot clears RAM — restore operating mode and torque.
                        # Use retries=0 to avoid hanging if motor isn't ready yet.
                        self.set_torque_enabled([mid], False, retries=0)
                        self.sync_write([mid], [mode], ADDR_OPERATING_MODE, LEN_OPERATING_MODE)
                        self.set_torque_enabled([mid], True, retries=0)
                        self._operating_modes[mid] = mode
                    else:
                        self.set_torque_enabled([mid], True, retries=0)
        return rebooted

    def check_connected(self):
        """Ensures the robot is connected."""
        if self.lazy_connect and not self.is_connected:
            self.connect()
        if not self.is_connected:
            raise OSError('Must call connect() first.')

    def handle_packet_result(self,
                             comm_result: int,
                             dxl_error: Optional[int] = None,
                             dxl_id: Optional[int] = None,
                             context: Optional[str] = None):
        """Handles the result from a communication request.

        Records the Alert bit (0x80) in dxl_error, which the motor sets on
        every status packet while a hardware error is latched. Recovery is not
        run from here: see :meth:`take_hardware_alerts`.
        """
        error_message = None
        if comm_result != self.dxl.COMM_SUCCESS:
            error_message = self.packet_handler.getTxRxResult(comm_result)
        elif dxl_error is not None:
            # Alert bit (bit 7) means a hardware error is latched
            if dxl_error & 0x80 and dxl_id is not None:
                self._note_hardware_alert(dxl_id, dxl_error)
            if dxl_error & 0x7F:
                error_message = self.packet_handler.getRxPacketError(dxl_error)
        if error_message:
            if dxl_id is not None:
                error_message = '[Motor ID: {}] {}'.format(
                    dxl_id, error_message)
            if context is not None:
                error_message = '> {}: {}'.format(context, error_message)
            logging.error(error_message)
            return False
        return True

    def _note_hardware_alert(self, motor_id: int, dxl_error: int) -> None:
        """Record that a motor is carrying a latched fault.

        Costs nothing: the byte arrived in a status packet the caller already
        paid for. Recovery is deliberately not run from here — a reboot takes
        the bus for a third of a second, and the instant a motor faults is
        the worst time to restart it. Callers drain this with
        :meth:`take_hardware_alerts` and decide.
        """
        with self._alerts_lock:
            self._hardware_alerts[int(motor_id)] = int(dxl_error)

    def take_hardware_alerts(self) -> "dict[int, int]":
        """Motors seen carrying the Alert bit since the last call, and clear."""
        with self._alerts_lock:
            alerts = dict(self._hardware_alerts)
            self._hardware_alerts.clear()
        return alerts

    def convert_to_unsigned(self, value: int, size: int) -> int:
        """Converts the given value to its unsigned representation."""
        if value < 0:
            max_value = (1 << (8 * size)) - 1
            value = max_value + value
        return value

    def change_motor_id(self, current_id: int, new_id: int) -> bool:
        """Changes the ID of a Dynamixel motor (1-252)."""
        if not (1 <= new_id <= 252):
            logging.error(f"Invalid ID {new_id}. Valid range is 1-252.")
            return False   
        try:
            self.set_torque_enabled([current_id], False)
            success = not self.write_byte([current_id], new_id, ADDR_ID)
            if success:
                logging.info(f"Changed motor ID: {current_id} → {new_id}")
            return success
        except Exception as e:
            logging.error(f"Failed to change motor ID: {e}")
            return False
    
    def change_motor_baudrate(self, motor_id: int, new_baud_rate: int) -> bool:
        """Changes the baud rate of a Dynamixel motor. Requires reconnect after change."""
        if new_baud_rate not in BAUD_RATE_MAP:
            logging.error(f"Invalid baud rate {new_baud_rate}. Valid: {list(BAUD_RATE_MAP.keys())}")
            return False   
        try:
            self.set_torque_enabled([motor_id], False)
            success = not self.write_byte([motor_id], BAUD_RATE_MAP[new_baud_rate], ADDR_BAUD_RATE)
            if success:
                logging.info(f"Changed motor {motor_id} baud rate: {new_baud_rate}")
            return success
        except Exception as e:
            logging.error(f"Failed to change baud rate: {e}")
            return False
    
    def scan_for_motors(self, port: str, id_range: tuple,
                             baud_rates: Optional[list] = None) -> list:
        """Scans for Dynamixel motors. Returns list of {'id', 'baud_rate', 'model_name'}."""
        if baud_rates is None:
            baud_rates = list(BAUD_RATE_MAP.keys())
        detected_motors = []
        for baud_rate in baud_rates:
            port_handler = self.dxl.PortHandler(port)
            packet_handler = self.dxl.PacketHandler(PROTOCOL_VERSION)
            try:
                if not port_handler.openPort() or not port_handler.setBaudRate(baud_rate):
                    continue
                for motor_id in range(id_range[0], id_range[1] + 1):
                    model_number, comm_result, _ = packet_handler.ping(port_handler, motor_id)
                    if comm_result == self.dxl.COMM_SUCCESS:
                        detected_motors.append({
                            'id': motor_id, 'baud_rate': baud_rate, 
                            'model_name': DYNAMIXEL_MODELS.get(model_number, f'Unknown({model_number})')
                        })
                port_handler.closePort()
            except Exception:
                try:
                    port_handler.closePort()
                except Exception:
                    pass
        return detected_motors

    def __enter__(self):
        """Enables use as a context manager."""
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, *args):
        """Enables use as a context manager."""
        self.disconnect()

    def __del__(self):
        """Automatically disconnect on destruction."""
        self.disconnect()


class _AlertCaptureSyncRead:
    """Wraps GroupSyncRead to capture per-motor error bytes from status packets.

    Every reader here fetches the same register span from every motor, which is
    exactly what sync read is for: the request names the address and length once
    and costs one byte per motor, where a bulk read repeats all three per motor.

    The stock GroupSyncRead discards the error byte returned by each motor's
    status packet. This wrapper stores them in ``motor_errors`` so callers can
    detect the Alert bit (0x80) without any extra bus traffic.
    """

    ALERT_BIT = 0x80

    def __init__(self, port_handler, packet_handler, dxl, address, size):
        self._inner = dxl.GroupSyncRead(port_handler, packet_handler,
                                        address, size)
        self._dxl = dxl
        self.motor_errors = {}

    def __getattr__(self, name):
        return getattr(self._inner, name)

    def txRxPacket(self):
        result = self._inner.txPacket()
        if result != self._dxl.COMM_SUCCESS:
            return result
        return self._rxPacket()

    def txRxPacketFast(self):
        """Fast sync read: one request answered by a single packet for all motors.

        A bus turnaround per motor dominates read time, and this spends one for
        the whole group. It needs servo firmware that implements the
        instruction, so callers probe once and keep the per-motor form when it
        goes unanswered.
        """
        result = self._inner.fastSyncReadTxPacket()
        if result != self._dxl.COMM_SUCCESS:
            return result
        return self._fastRxPacket()

    def _fastRxPacket(self):
        inner = self._inner
        inner.last_result = False
        self.motor_errors = {}

        if not inner.data_dict:
            return self._dxl.COMM_NOT_AVAILABLE

        size = inner.data_length
        count = len(inner.data_dict)
        # Each motor contributes error(1) + id(1) + data + crc(2) to the packet.
        stride = size + 4
        raw, result, _ = inner.ph.fastSyncReadRx(
            inner.port, self._dxl.BROADCAST_ID, stride * count)
        if result != self._dxl.COMM_SUCCESS:
            return result

        raw = bytearray(raw)
        if len(raw) < stride * count:
            return self._dxl.COMM_RX_CORRUPT

        expected = set(inner.data_dict)
        at = 0
        for _ in range(count):
            motor_id = raw[at + 1]
            if motor_id not in expected:
                return self._dxl.COMM_RX_CORRUPT
            self.motor_errors[motor_id] = raw[at]
            inner.data_dict[motor_id] = bytearray(raw[at + 2:at + 2 + size])
            at += stride

        inner.last_result = True
        return self._dxl.COMM_SUCCESS

    def _rxPacket(self):
        inner = self._inner
        inner.last_result = False
        self.motor_errors = {}
        result = self._dxl.COMM_RX_FAIL

        if not inner.data_dict:
            return self._dxl.COMM_NOT_AVAILABLE

        for dxl_id in inner.data_dict:
            data, result, error = inner.ph.readRx(
                inner.port, dxl_id, inner.data_length)
            inner.data_dict[dxl_id] = data
            self.motor_errors[dxl_id] = error or 0
            if result != self._dxl.COMM_SUCCESS:
                return result

        if result == self._dxl.COMM_SUCCESS:
            inner.last_result = True
        return result


class DynamixelReader:
    """Reads data from Dynamixel motors.

    All motors are read at the same address and length, so this wraps a
    GroupSyncRead from the DynamixelSDK.
    """

    def __init__(self, client: DynamixelClient, motor_ids: Sequence[int],
                 address: int, size: int):
        """Initializes a new reader."""
        self.client = client
        self.motor_ids = motor_ids
        self.address = address
        self.size = size
        self.last_read_ok = True
        # Fallback bounds: a failed motor is skipped for a cooldown and full-bus sweeps
        # are rate limited, so a dead motor cannot stall reads while the bus lock is held.
        self._fallback_skip_until: Dict[int, float] = {}
        self._last_full_fallback = 0.0
        # Set by probe_fast_sync_read() at connect time; older servo firmware
        # ignores the instruction, so it is never assumed.
        self._fast_sync_read = False
        self._initialize_data()

        self.operation = _AlertCaptureSyncRead(client.port_handler,
                                               client.packet_handler,
                                               client.dxl, address, size)

        for motor_id in motor_ids:
            success = self.operation.addParam(motor_id)
            if not success:
                raise OSError(
                    '[Motor ID: {}] Could not add parameter to sync read.'
                    .format(motor_id))

    def probe_fast_sync_read(self) -> bool:
        """Try one fast sync read and adopt it if every motor answered.

        Alerts are deliberately not acted on here: a motor may still be
        carrying a latched error from before this client existed, and connect
        time is not the moment to reboot it.
        """
        with self.client._bus_lock:
            self._fast_sync_read = False
            if self.operation.txRxPacketFast() != self.client.dxl.COMM_SUCCESS:
                self.client._flush_input_buffer()
                return False
            if any(not self.operation.isAvailable(m, self.address, self.size)
                   for m in self.motor_ids):
                return False
            self._fast_sync_read = True
            return True

    def read(self, retries: int = 1):
        """Reads data from the motors, holding the bus lock for the whole transaction."""
        self.client.check_connected()
        with self.client._bus_lock:
            success = False
            attempt = 0
            while not success and retries >= 0:
                # Only the first attempt goes fast: if the grouped reply is
                # ever malformed, retrying per motor is what recovers it.
                if self._fast_sync_read and attempt == 0:
                    comm_result = self.operation.txRxPacketFast()
                else:
                    comm_result = self.operation.txRxPacket()
                success = self.client.handle_packet_result(
                    comm_result, context='read')
                if not success:
                    self.client._flush_input_buffer()
                attempt += 1
                retries -= 1

            if not success:
                # Bulk transaction failed entirely: try each motor individually
                # (still under the bus lock), at most once per rate-limit window.
                now = time.monotonic()
                if now - self._last_full_fallback < FALLBACK_FULL_SWEEP_MIN_INTERVAL_S:
                    self.last_read_ok = False
                    return self._get_data()
                self._last_full_fallback = now
                logging.warning(
                    'Sync read failed; falling back to per-motor reads for %d motor(s)',
                    len(self.motor_ids))
                still_failed = self._run_bounded_fallback(list(self.motor_ids))
                self.last_read_ok = not still_failed
                return self._get_data()

            # Alert bits ride along in the status packets we already have.
            # They are recorded, never acted on here: rebooting from a read
            # would stall the bus mid-motion and reboot a motor at the moment
            # it faulted, which is the worst moment to do it.
            for motor_id, error in self.operation.motor_errors.items():
                if error & _AlertCaptureSyncRead.ALERT_BIT:
                    self.client._note_hardware_alert(motor_id, error)

            errored_ids = []
            for i, motor_id in enumerate(self.motor_ids):
                available = self.operation.isAvailable(motor_id, self.address,
                                                       self.size)
                if not available:
                    errored_ids.append(motor_id)
                    continue

                try:
                    self._update_data(i, motor_id)
                except Exception as e:
                    logging.error(f'Error updating data for motor {motor_id}: {e}')
                    errored_ids.append(motor_id)
                    continue

            if errored_ids:
                logging.warning('Sync read missing data for %s; per-motor fallback',
                                str(errored_ids))
                errored_ids = self._run_bounded_fallback(errored_ids)

            # Expose whether every motor produced fresh data, so callers can tell
            # a real reading apart from the stale cache kept on failed reads.
            self.last_read_ok = not errored_ids

            return self._get_data()

    def _run_bounded_fallback(self, motor_ids: Sequence[int]) -> List[int]:
        """Run the per-motor fallback, skipping motors in their failure cooldown.

        Returns the IDs still lacking fresh data (failed now or cooling down).
        """
        now = time.monotonic()
        eligible = [m for m in motor_ids
                    if self._fallback_skip_until.get(m, 0.0) <= now]
        cooling = [m for m in motor_ids if m not in eligible]
        still_failed = self._read_per_motor_fallback(eligible) if eligible else []
        for motor_id in still_failed:
            self._fallback_skip_until[motor_id] = now + FALLBACK_MOTOR_COOLDOWN_S
        return still_failed + cooling

    def _read_per_motor_fallback(self, motor_ids: Sequence[int]) -> List[int]:
        """Read each motor individually after a failed sync read.

        Returns the IDs whose data could still not be refreshed (their cached
        values are kept). Base implementation cannot read individual motors,
        so everything stays stale.
        """
        return list(motor_ids)

    def _initialize_data(self):
        """Initializes the cached data."""
        self._data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        """Updates the data index for the given motor ID."""
        self._data[index] = self.operation.getData(motor_id, self.address,
                                                   self.size)

    def _get_data(self):
        """Returns a copy of the data."""
        return self._data.copy()


class DynamixelPosVelCurReader(DynamixelReader):
    """Reads positions and velocities."""

    def __init__(self,
                 client: DynamixelClient,
                 motor_ids: Sequence[int],
                 pos_scale: float = 1.0,
                 vel_scale: float = 1.0,
                 cur_scale: float = 1.0):
        super().__init__(
            client,
            motor_ids,
            address=ADDR_PRESENT_POS_VEL_CUR,
            size=LEN_PRESENT_POS_VEL_CUR,
        )
        self.pos_scale = pos_scale
        self.vel_scale = vel_scale
        self.cur_scale = cur_scale

    def _initialize_data(self):
        """Initializes the cached data."""
        self._pos_data = np.zeros(len(self.motor_ids), dtype=np.float32)
        self._vel_data = np.zeros(len(self.motor_ids), dtype=np.float32)
        self._cur_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        """Updates the data index for the given motor ID."""
        cur = self.operation.getData(motor_id, ADDR_PRESENT_CURRENT,
                                     LEN_PRESENT_CURRENT)
        vel = self.operation.getData(motor_id, ADDR_PRESENT_VELOCITY,
                                     LEN_PRESENT_VELOCITY)
        pos = self.operation.getData(motor_id, ADDR_PRESENT_POSITION,
                                     LEN_PRESENT_POSITION)
        cur = unsigned_to_signed(cur, size=2)
        vel = unsigned_to_signed(vel, size=4)
        pos = unsigned_to_signed(pos, size=4)
        self._pos_data[index] = float(pos) * self.pos_scale
        self._vel_data[index] = float(vel) * self.vel_scale
        self._cur_data[index] = float(cur) * self.cur_scale

    def _read_per_motor_fallback(self, motor_ids: Sequence[int]) -> List[int]:
        """Per-motor reads for position / velocity / current.

        Cached values are kept for motors whose individual read also fails;
        those IDs are returned.
        """
        port = self.client.port_handler
        packet = self.client.packet_handler
        comm_success = self.client.dxl.COMM_SUCCESS
        still_failed = []
        for motor_id in motor_ids:
            try:
                idx = self.motor_ids.index(motor_id)
            except ValueError:
                continue
            motor_ok = True
            try:
                pos_raw, comm, _ = packet.read4ByteTxRx(
                    port, motor_id, ADDR_PRESENT_POSITION)
                if comm == comm_success:
                    self._pos_data[idx] = (
                        float(unsigned_to_signed(pos_raw, size=4)) * self.pos_scale)
                else:
                    motor_ok = False
                vel_raw, comm, _ = packet.read4ByteTxRx(
                    port, motor_id, ADDR_PRESENT_VELOCITY)
                if comm == comm_success:
                    self._vel_data[idx] = (
                        float(unsigned_to_signed(vel_raw, size=4)) * self.vel_scale)
                else:
                    motor_ok = False
                cur_raw, comm, _ = packet.read2ByteTxRx(
                    port, motor_id, ADDR_PRESENT_CURRENT)
                if comm == comm_success:
                    self._cur_data[idx] = (
                        float(unsigned_to_signed(cur_raw, size=2)) * self.cur_scale)
                else:
                    motor_ok = False
            except Exception as e:
                logging.warning(
                    'Per-motor pos/vel/cur read failed for motor %d: %s',
                    motor_id, e)
                motor_ok = False
            if not motor_ok:
                still_failed.append(motor_id)
        return still_failed

    def _get_data(self):
        """Returns a copy of the data."""
        return (self._pos_data.copy(), self._vel_data.copy(),
                self._cur_data.copy())

class DynamixelTempReader(DynamixelReader):
    """Reads present temperature (1 byte) for each Dynamixel motor."""
    
    def _initialize_data(self):
        self._temp_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        # The raw value from the control table is 1 byte = 1 degree Celsius.
        raw_val = self.operation.getData(motor_id, self.address, self.size)
        self._temp_data[index] = float(raw_val)

    def _read_per_motor_fallback(self, motor_ids: Sequence[int]) -> List[int]:
        """Per-motor temperature reads; returns IDs that still failed."""
        port = self.client.port_handler
        packet = self.client.packet_handler
        comm_success = self.client.dxl.COMM_SUCCESS
        still_failed = []
        for motor_id in motor_ids:
            try:
                idx = self.motor_ids.index(motor_id)
            except ValueError:
                continue
            try:
                raw, comm, _ = packet.read1ByteTxRx(
                    port, motor_id, ADDR_PRESENT_TEMPERATURE)
                if comm == comm_success:
                    self._temp_data[idx] = float(raw)
                else:
                    still_failed.append(motor_id)
            except Exception as e:
                logging.warning('Per-motor temperature read failed for motor %d: %s',
                                motor_id, e)
                still_failed.append(motor_id)
        return still_failed

    def _get_data(self):
        return self._temp_data.copy()

atexit.register(dynamixel_cleanup_handler)

if __name__ == '__main__':
    import argparse
    import itertools

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-m',
        '--motors',
        required=True,
        help='Comma-separated list of motor IDs.')
    parser.add_argument(
        '-d',
        '--device',
        default=None,
        help='The Dynamixel device. Default: auto-detect the motor adapter.')
    parser.add_argument(
        '-b', '--baud', default=1000000, help='The baudrate to connect with.')
    from ..utils.utils import auto_detect_port

    parsed_args = parser.parse_args()
    motors = [int(motor) for motor in parsed_args.motors.split(',')]
    
    way_points = [np.zeros(len(motors)), np.full(len(motors), np.pi)]

    device = parsed_args.device or auto_detect_port('dynamixel')

    with DynamixelClient(motors, device, parsed_args.baud) as dxl_client:
        for step in itertools.count():
            if step > 0 and step % 50 == 0:
                way_point = way_points[(step // 100) % len(way_points)]
                print('Writing: {}'.format(way_point.tolist()))
                dxl_client.write_desired_pos(motors, way_point)
            read_start = time.time()
            pos_now, vel_now, cur_now = dxl_client.read_position_velocity_current()
            if step % 5 == 0:
                print('[{}] Frequency: {:.2f} Hz'.format(
                    step, 1.0 / (time.time() - read_start)))
                print('> Pos: {}'.format(pos_now.tolist()))
                print('> Vel: {}'.format(vel_now.tolist()))
                print('> Cur: {}'.format(cur_now.tolist()))