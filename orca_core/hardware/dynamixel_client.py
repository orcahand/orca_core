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
import time
from typing import Optional, Sequence, Union, Tuple
import numpy as np

PROTOCOL_VERSION = 2.0

# The following addresses assume XH motors.
# see https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/ for control table
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
ADDR_PRESENT_TEMPERATURE = 146

# Data Byte Length
LEN_OPERATING_MODE = 1
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
DEFAULT_CUR_SCALE = 1.34


def dynamixel_cleanup_handler():
    """Cleanup function to ensure Dynamixels are disconnected properly."""
    open_clients = list(DynamixelClient.OPEN_CLIENTS)
    for open_client in open_clients:
        if open_client.port_handler.is_using:
            logging.warning('Forcing client to close.')
        open_client.port_handler.is_using = False
        open_client.disconnect()


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


class DynamixelClient:
    """Client for communicating with Dynamixel motors.

    NOTE: This only supports Protocol 2.
    """

    # The currently open clients.
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

        self.OPEN_CLIENTS.add(self)

    @property
    def is_connected(self) -> bool:
        return self.port_handler.is_open

    def connect(self):
        """Connects to the Dynamixel motors.

        NOTE: This should be called after all DynamixelClients on the same
            process are created.
        """
        assert not self.is_connected, 'Client is already connected.'

        if self.port_handler.openPort():
            logging.info('Succeeded to open port: %s', self.port_name)
        else:
            raise OSError(
                ('Failed to open port at {} (Check that the device is powered '
                 'on and connected to your computer).').format(self.port_name))

        if self.port_handler.setBaudRate(self.baudrate):
            logging.info('Succeeded to set baudrate to %d', self.baudrate)
        else:
            raise OSError(
                ('Failed to set the baudrate to {} (Ensure that the device was '
                 'configured for this baudrate).').format(self.baudrate))

        # Start with all motors enabled.
        self.set_torque_enabled(self.motor_ids, True)

    def disconnect(self):
        """Disconnects from the Dynamixel device."""
        if not self.is_connected:
            return
        if self.port_handler.is_using:
            logging.error('Port handler in use; cannot disconnect.')
            return
        # Ensure motors are disabled at the end.
        self.set_torque_enabled(self.motor_ids, False, retries=0)
        self.port_handler.closePort()
        if self in self.OPEN_CLIENTS:
            self.OPEN_CLIENTS.remove(self)

    def set_torque_enabled(self,
                           motor_ids: Sequence[int],
                           enabled: bool,
                           retries: int = -1,
                           retry_interval: float = 0.25):
        """Sets whether torque is enabled for the motors.

        Args:
            motor_ids: The motor IDs to configure.
            enabled: Whether to engage or disengage the motors.
            retries: The number of times to retry. If this is <0, will retry
                forever.
            retry_interval: The number of seconds to wait between retries.
        """
        remaining_ids = list(motor_ids)
        while remaining_ids:
            remaining_ids = self.write_byte(
                remaining_ids,
                int(enabled),
                ADDR_TORQUE_ENABLE,
            )
            if remaining_ids:
                logging.error('Could not set torque %s for IDs: %s',
                              'enabled' if enabled else 'disabled',
                              str(remaining_ids))
            if retries == 0:
                break
            time.sleep(retry_interval)
            retries -= 1

    def set_operating_mode(self, motor_ids: Sequence[int], mode_value: int):
        """
        see https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#operating-mode11
        0: current control mode
        1: velocity control mode
        3: position control mode
        4: multi-turn position control mode
        5: current-based position control mode
        """
        # data in EEPROM area can only be written when torque is disabled
        self.set_torque_enabled(motor_ids, False)
        self.sync_write(motor_ids, [mode_value]*len(motor_ids), ADDR_OPERATING_MODE, LEN_OPERATING_MODE)
        self.set_torque_enabled(motor_ids, True)

    def read_pos_vel_cur(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns the positions, velocities, and currents."""
        return self._pos_vel_cur_reader.read()

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
        assert len(motor_ids) == len(current)
        self.sync_write(motor_ids, current, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT)

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
        for motor_id in motor_ids:
            comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, address, value)
            success = self.handle_packet_result(
                comm_result, dxl_error, motor_id, context='write_byte')
            if not success:
                errored_ids.append(motor_id)
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
        """Handles the result from a communication request."""
        error_message = None
        if comm_result != self.dxl.COMM_SUCCESS:
            error_message = self.packet_handler.getTxRxResult(comm_result)
        elif dxl_error is not None:
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

    def convert_to_unsigned(self, value: int, size: int) -> int:
        """Converts the given value to its unsigned representation."""
        if value < 0:
            max_value = (1 << (8 * size)) - 1
            value = max_value + value
        return value

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


class DynamixelReader:
    """Reads data from Dynamixel motors.

    This wraps a GroupBulkRead from the DynamixelSDK.
    """

    def __init__(self, client: DynamixelClient, motor_ids: Sequence[int],
                 address: int, size: int):
        """Initializes a new reader."""
        self.client = client
        self.motor_ids = motor_ids
        self.address = address
        self.size = size
        self._initialize_data()

        self.operation = self.client.dxl.GroupBulkRead(client.port_handler,
                                                       client.packet_handler)

        for motor_id in motor_ids:
            success = self.operation.addParam(motor_id, address, size)
            if not success:
                raise OSError(
                    '[Motor ID: {}] Could not add parameter to bulk read.'
                    .format(motor_id))

    def read(self, retries: int = 1):
        """Reads data from the motors."""
        self.client.check_connected()
        success = False
        while not success and retries >= 0:
            comm_result = self.operation.txRxPacket()
            success = self.client.handle_packet_result(
                comm_result, context='read')
            retries -= 1

        # If we failed, send a copy of the previous data.
        if not success:
            return self._get_data()

        errored_ids = []
        for i, motor_id in enumerate(self.motor_ids):
            # Check if the data is available.
            available = self.operation.isAvailable(motor_id, self.address,
                                                   self.size)
            if not available:
                errored_ids.append(motor_id)
                continue

            self._update_data(i, motor_id)

        if errored_ids:
            logging.error('Bulk read data is unavailable for: %s',
                          str(errored_ids))

        return self._get_data()

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

    def _get_data(self):
        """Returns a copy of the data."""
        return (self._pos_data.copy(), self._vel_data.copy(),
                self._cur_data.copy())

class DynamixelTempReader(DynamixelReader):
    """Reads present temperature (1 byte) for each Dynamixel motor."""
    
    def _initialize_data(self):
        # We'll store one float per motor for the temperature values.
        self._temp_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        # The raw value from the control table is 1 byte = 1 degree Celsius.
        raw_val = self.operation.getData(motor_id, self.address, self.size)
        self._temp_data[index] = float(raw_val)

    def _get_data(self):
        return self._temp_data.copy()

# Register global cleanup function.
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
        default='/dev/cu.usbserial-FT62AFSR',
        help='The Dynamixel device to connect to.')
    parser.add_argument(
        '-b', '--baud', default=1000000, help='The baudrate to connect with.')
    parsed_args = parser.parse_args()
    motors = [int(motor) for motor in parsed_args.motors.split(',')]
    
    way_points = [np.zeros(len(motors)), np.full(len(motors), np.pi)]

    with DynamixelClient(motors, parsed_args.device,
                         parsed_args.baud) as dxl_client:
        for step in itertools.count():
            if step > 0 and step % 50 == 0:
                way_point = way_points[(step // 100) % len(way_points)]
                print('Writing: {}'.format(way_point.tolist()))
                dxl_client.write_desired_pos(motors, way_point)
            read_start = time.time()
            pos_now, vel_now, cur_now = dxl_client.read_pos_vel_cur()
            if step % 5 == 0:
                print('[{}] Frequency: {:.2f} Hz'.format(
                    step, 1.0 / (time.time() - read_start)))
                print('> Pos: {}'.format(pos_now.tolist()))
                print('> Vel: {}'.format(vel_now.tolist()))
                print('> Cur: {}'.format(cur_now.tolist()))