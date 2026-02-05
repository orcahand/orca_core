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
import time
from typing import Optional, Sequence, Tuple
import numpy as np

from .motor_client import MotorClient
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
    SMS_STS_ACC,
    SMS_STS_GOAL_POSITION_L,
    SMS_STS_GOAL_TIME_L,
    SMS_STS_GOAL_SPEED_L,
)

# SCServo position scale: 0-4095 raw units = 0-360 degrees = 0-2*pi radians
DEFAULT_POS_SCALE = 2.0 * np.pi / 4096  # 4096 steps for 360°
DEFAULT_VEL_SCALE = 0.732 * 2.0 * np.pi / 60.0  # Convert 0.732 RPM/unit to rad/s
DEFAULT_CUR_SCALE = 6.5  # mA per unit

# Position limits for STS servo mode (0-4095, one full rotation)
POS_MIN = 0
POS_MAX = 4095


def feetech_cleanup_handler():
    """Cleanup function to ensure Feetech servos are disconnected properly."""
    open_clients = list(FeetechClient.OPEN_CLIENTS)
    for client in open_clients:
        if client.port_handler.is_using:
            logging.warning('Forcing Feetech client to close.')
        client.port_handler.is_using = False
        client.disconnect()


class FeetechClient(MotorClient):
    """Client for communicating with Feetech SCServo motors.

    This implements the MotorClient interface for Feetech motors,
    providing compatibility with the OrcaHand control system.
    """

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

        self._connected = False

        # Default motion parameters
        # Speed unit is 0.732 RPM per value. In practice, motor tops out around
        # 1500 steps/sec (~22 RPM) due to hardware limits.
        self._default_speed = 60  # Good balance of speed and control
        self._default_acc = 50  # Acceleration (0-254)
        self._default_torque = 500  # Torque limit (0-1000), required for motion

        self.OPEN_CLIENTS.add(self)

    @property
    def is_connected(self) -> bool:
        return self._connected and self.port_handler.is_open

    def connect(self) -> None:
        """Connects to the Feetech motors."""
        if self._connected:
            raise RuntimeError('Client is already connected.')

        self.port_handler.baudrate = self.baudrate

        if self.port_handler.openPort():
            logging.info('Succeeded to open port: %s', self.port_name)
        else:
            raise OSError(
                f'Failed to open port at {self.port_name} (Check that the device is '
                'powered on and connected to your computer).'
            )

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
            self.packet_handler.write1ByteTxRx(motor_id, SMS_STS_MODE, 0)

        # Enable torque for all motors
        self.set_torque_enabled(self.motor_ids, True)

    def disconnect(self) -> None:
        """Disconnects from the Feetech motors."""
        if not self._connected:
            return

        if self.port_handler.is_using:
            logging.error('Port handler in use; cannot disconnect.')
            return

        # Disable torque before disconnecting
        self.set_torque_enabled(self.motor_ids, False, retries=0)

        self.port_handler.closePort()
        self._connected = False

        if self in self.OPEN_CLIENTS:
            self.OPEN_CLIENTS.remove(self)

    def set_torque_enabled(
        self,
        motor_ids: Sequence[int],
        enabled: bool,
        retries: int = -1,
        retry_interval: float = 0.25,
    ) -> None:
        """Sets whether torque is enabled for the motors."""
        self._check_connected()

        remaining_ids = list(motor_ids)
        while remaining_ids:
            failed_ids = []
            for motor_id in remaining_ids:
                result, error = self.packet_handler.write1ByteTxRx(
                    motor_id, SMS_STS_TORQUE_ENABLE, int(enabled)
                )
                if result != COMM_SUCCESS or error != 0:
                    failed_ids.append(motor_id)

            remaining_ids = failed_ids
            if remaining_ids:
                logging.error(
                    'Could not set torque %s for IDs: %s',
                    'enabled' if enabled else 'disabled',
                    str(remaining_ids),
                )
            if retries == 0:
                break
            if remaining_ids:
                time.sleep(retry_interval)
            retries -= 1

    def set_operating_mode(self, motor_ids: Sequence[int], mode: int) -> None:
        """Sets the operating mode for the specified motors.

        Feetech only supports:
        - Mode 0: Servo mode (position control)
        - Mode 1: Wheel mode (velocity control)

        Unsupported Dynamixel modes are mapped to servo mode with a warning.
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

        # Disable torque to change mode
        self.set_torque_enabled(motor_ids, False)

        for motor_id in motor_ids:
            # Only velocity mode (1) maps to wheel mode; all others use servo mode
            feetech_mode = 1 if mode == 1 else 0

            result, error = self.packet_handler.write1ByteTxRx(
                motor_id, SMS_STS_MODE, feetech_mode
            )
            if result != COMM_SUCCESS or error != 0:
                logging.error(
                    'Failed to set operating mode for motor %d: result=%d, error=%d',
                    motor_id, result, error
                )

        # Re-enable torque
        self.set_torque_enabled(motor_ids, True)

    def read_pos_vel_cur(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Reads position, velocity, and current for all motors."""
        self._check_connected()

        positions = np.zeros(len(self.motor_ids), dtype=np.float32)
        velocities = np.zeros(len(self.motor_ids), dtype=np.float32)
        currents = np.zeros(len(self.motor_ids), dtype=np.float32)

        for i, motor_id in enumerate(self.motor_ids):
            # Read position
            pos_raw, result, error = self.packet_handler.read2ByteTxRx(
                motor_id, SMS_STS_PRESENT_POSITION_L
            )
            if result == COMM_SUCCESS and error == 0:
                pos_signed = self.packet_handler.scs_tohost(pos_raw, 15)
                pos_normalized = self._normalize_position(pos_signed)
                positions[i] = pos_normalized * self.pos_scale
            else:
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
                velocities[i] = vel_signed * self.vel_scale
            else:
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
                logging.warning(
                    'Failed to read current for motor %d: result=%d, error=%d',
                    motor_id, result, error
                )

        return positions, velocities, currents

    def read_temperature(self) -> np.ndarray:
        """Reads the temperature for all motors."""
        self._check_connected()

        temperatures = np.zeros(len(self.motor_ids), dtype=np.float32)

        for i, motor_id in enumerate(self.motor_ids):
            temp, result, error = self.packet_handler.read1ByteTxRx(
                motor_id, SMS_STS_PRESENT_TEMPERATURE
            )
            if result == COMM_SUCCESS and error == 0:
                temperatures[i] = float(temp)
            else:
                logging.warning(
                    'Failed to read temperature for motor %d: result=%d, error=%d',
                    motor_id, result, error
                )

        return temperatures

    def write_desired_pos(
        self,
        motor_ids: Sequence[int],
        positions: np.ndarray,
        speed: Optional[int] = None,
        torque: Optional[int] = None,
    ) -> None:
        """Writes desired positions to the motors.

        Args:
            motor_ids: Motor IDs to write to.
            positions: Target positions in radians.
            speed: Movement speed (0.732 RPM per unit). Default ~60 = 44 RPM.
            torque: Torque limit (0-1000). Default 500. Required for motion.
        """
        self._check_connected()

        if len(motor_ids) != len(positions):
            raise ValueError('motor_ids and positions must have the same length')

        speed = speed if speed is not None else self._default_speed
        torque = torque if torque is not None else self._default_torque

        for motor_id, pos_rad in zip(motor_ids, positions):
            # Convert radians to raw position and clamp to valid range
            pos_raw = int(pos_rad / self.pos_scale)
            pos_raw = self._clamp_position(pos_raw)

            logging.debug(
                'WritePosEx: motor=%d, pos=%d, speed=%d, acc=%d, torque=%d',
                motor_id, pos_raw, speed, self._default_acc, torque
            )

            result, error = self.packet_handler.WritePosEx(
                motor_id,
                pos_raw,
                speed,
                self._default_acc,
                torque,
            )
            if result != COMM_SUCCESS or error != 0:
                logging.error(
                    'Failed to write position to motor %d: result=%d, error=%d',
                    motor_id, result, error
                )

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

    @property
    def requires_offset_calibration(self) -> bool:
        return True

    def calibrate_offset(self, motor_id: int, upper: bool = True) -> bool:
        """Set current physical position to read as upper or lower bound.

        Uses Feetech's INST_OFSCAL command to shift the coordinate system.
        The offset is stored in EEPROM and persists across power cycles.

        Args:
            motor_id: Motor to calibrate.
            upper: If True, set to upper bound (3595). If False, set to lower bound (500).

        Returns:
            True on success, False otherwise.
        """
        self._check_connected()

        # Buffer of 500 (~44°) ensures room for tendon loosening over time
        target_position = 3595 if upper else 500

        result, error = self.packet_handler.reOfsCal(motor_id, target_position)
        if result != COMM_SUCCESS or error != 0:
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

        speed = speed if speed is not None else self._default_speed
        acc = acc if acc is not None else self._default_acc
        torque = torque if torque is not None else self._default_torque

        # Clear any existing sync write params
        self.packet_handler.groupSyncWrite.clearParam()

        for motor_id, pos_rad in zip(motor_ids, positions):
            pos_raw = int(pos_rad / self.pos_scale)
            pos_raw = self._clamp_position(pos_raw)

            logging.debug(
                'SyncWritePosEx: motor=%d, pos=%d, speed=%d, acc=%d, torque=%d',
                motor_id, pos_raw, speed, acc, torque
            )

            self.packet_handler.SyncWritePosEx(motor_id, pos_raw, speed, acc, torque)

        # Send the sync write packet
        result = self.packet_handler.groupSyncWrite.txPacket()
        if result != COMM_SUCCESS:
            logging.error('Sync write failed: result=%d', result)

        # Clear params for next use
        self.packet_handler.groupSyncWrite.clearParam()

    def read_pos_vel_cur_sync(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Reads position, velocity, and current for all motors using sync read.

        More efficient than individual reads for multiple motors.
        """
        self._check_connected()

        positions = np.zeros(len(self.motor_ids), dtype=np.float32)
        velocities = np.zeros(len(self.motor_ids), dtype=np.float32)
        currents = np.zeros(len(self.motor_ids), dtype=np.float32)

        # Create sync read for position, speed, load, voltage, temp, moving, current
        # From addr 56 (position) to 70 (current_h) = 15 bytes
        sync_read = GroupSyncRead(self.packet_handler, SMS_STS_PRESENT_POSITION_L, 15)

        for motor_id in self.motor_ids:
            sync_read.addParam(motor_id)

        result = sync_read.txRxPacket()
        if result != COMM_SUCCESS:
            logging.warning('Sync read failed, falling back to individual reads')
            return self.read_pos_vel_cur()

        for i, motor_id in enumerate(self.motor_ids):
            available, error = sync_read.isAvailable(
                motor_id, SMS_STS_PRESENT_POSITION_L, 2
            )
            if available:
                pos_raw = sync_read.getData(motor_id, SMS_STS_PRESENT_POSITION_L, 2)
                pos_signed = self.packet_handler.scs_tohost(pos_raw, 15)
                pos_normalized = self._normalize_position(pos_signed)
                positions[i] = pos_normalized * self.pos_scale

                vel_raw = sync_read.getData(motor_id, SMS_STS_PRESENT_SPEED_L, 2)
                vel_signed = self.packet_handler.scs_tohost(vel_raw, 15)
                velocities[i] = vel_signed * self.vel_scale

                cur_raw = sync_read.getData(motor_id, SMS_STS_PRESENT_CURRENT_L, 2)
                cur_signed = self.packet_handler.scs_tohost(cur_raw, 15)
                currents[i] = cur_signed * self.cur_scale
            else:
                logging.warning('Motor %d not available in sync read', motor_id)

        return positions, velocities, currents


# Register global cleanup function
atexit.register(feetech_cleanup_handler)
