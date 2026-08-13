# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Communication using a simulated Feetech client."""

import atexit
import logging
import random
from typing import Optional, Sequence

import numpy as np

from .feetech_client import DEFAULT_TORQUE_LIMIT, FeetechClient
from .motor_client import MotorClient, MotorRead


def feetech_cleanup_handler():
    """Disconnect every open mock client at interpreter exit."""
    MockFeetechClient.cleanup_open_clients()


class _MockPortHandler:
    """Stands in for the SDK port handler the exit cleanup inspects."""

    def __init__(self, port: str):
        self.port_name = port
        self.is_open = False
        self.is_using = False


class MockFeetechClient(MotorClient):
    """Mock client for simulating communication with Feetech SCServo motors.

    Carries FeetechClient's capability attributes, so code that branches on
    them (mode support, position bounds, offset calibration) behaves the same
    against the mock as against the hardware.
    """

    motor_type = FeetechClient.motor_type
    factory_default_id = FeetechClient.factory_default_id
    factory_default_baudrate = FeetechClient.factory_default_baudrate
    baud_rate_map = FeetechClient.baud_rate_map
    requires_unpowered_hotplug = FeetechClient.requires_unpowered_hotplug
    waits_for_motion = FeetechClient.waits_for_motion
    supports_multi_turn = FeetechClient.supports_multi_turn
    supported_modes = FeetechClient.supported_modes
    position_range_rad = FeetechClient.position_range_rad

    # Clients with an open (simulated) port; registered on successful
    # connect() so the atexit cleanup only ever touches live connections.
    OPEN_CLIENTS = set()

    def __init__(self,
                 motor_ids: Sequence[int],
                 port: str = '/dev/ttyUSB0',
                 baudrate: int = 1000000,
                 lazy_connect: bool = False,
                 pos_scale: Optional[float] = None,
                 vel_scale: Optional[float] = None,
                 cur_scale: Optional[float] = None):
        """Initializes a new mock client.

        Args:
            motor_ids: All motor IDs being used by the client.
            port: The serial port the real client would talk to.
            baudrate: The baudrate the real client would communicate at.
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

        self.pos_scale = pos_scale
        self.vel_scale = vel_scale
        self.cur_scale = cur_scale

        self.port_handler = _MockPortHandler(port)

        # States for simulation.
        self._connected = False
        self._torque_enabled = {mid: False for mid in self.motor_ids}
        # Servo mode, the family's power-on default.
        self._operating_mode = {mid: 0 for mid in self.motor_ids}
        self._pos = {mid: 0.0 for mid in self.motor_ids}
        self._vel = {mid: 0.0 for mid in self.motor_ids}
        self._cur = {mid: 0.0 for mid in self.motor_ids}
        self._torque = {mid: DEFAULT_TORQUE_LIMIT for mid in self.motor_ids}
        self._profile_velocity = {mid: 0.0 for mid in self.motor_ids}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        """Connects to the simulated Feetech motors.

        Mirrors the real client's registry contract: the client joins
        ``OPEN_CLIENTS`` only once the connect completed, so a failed connect
        never leaves a dead entry for the exit cleanup.
        """
        if self._connected:
            raise RuntimeError('Client is already connected.')

        logging.info('Succeeded to open port: %s', self.port_name)

        self._connected = True
        self.port_handler.is_open = True

        # Motor state is left as-is, mirroring the real client: connecting must
        # never make the hand stiffen, move, or change a mode register.

        self.OPEN_CLIENTS.add(self)

    def disconnect(self) -> None:
        """Disconnects from the simulated Feetech device.

        The client is always marked disconnected and deregistered, even when
        the final torque-disable raises; that exception propagates after
        cleanup, mirroring the real client.
        """
        if not self._connected:
            return

        try:
            self.set_torque_enabled(self.motor_ids, False, retries=0)
        finally:
            self._connected = False
            self.port_handler.is_open = False
            self.OPEN_CLIENTS.discard(self)

    def set_torque_enabled(self,
                           motor_ids: Sequence[int],
                           enabled: bool,
                           retries: int = 3,
                           retry_interval: float = 0.25) -> "list[int]":
        """Sets whether torque is enabled for the motors.

        Returns:
            A list of motor IDs that could not be set (unknown IDs, matching
            a real client's silent motors).
        """
        self.check_connected()
        failed_ids = []
        for mid in motor_ids:
            if mid not in self._torque_enabled:
                failed_ids.append(mid)
                continue
            self._torque_enabled[mid] = enabled
        if failed_ids:
            logging.error('Could not set torque %s for IDs: %s',
                          'enabled' if enabled else 'disabled', str(failed_ids))
        return failed_ids

    def set_operating_mode(self, motor_ids: Sequence[int], mode: int) -> None:
        """Sets the operating mode, mapping unsupported modes to servo mode.

        Mode changes require torque off, so unknown IDs are logged and skipped
        rather than raising.
        """
        self.check_connected()
        feetech_mode = 1 if mode == 1 else 0
        for mid in motor_ids:
            if mid not in self._operating_mode:
                logging.error('Failed to set operating mode for motor %d: unknown ID', mid)
                continue
            self._torque_enabled[mid] = False
            self._operating_mode[mid] = feetech_mode
            self._torque_enabled[mid] = True

    def wait_for_motion_complete(self, timeout: float = 5.0) -> None:
        """Returns immediately; simulated motion lands the instant it is commanded."""

    def read_position_velocity_current(self) -> MotorRead:
        """Returns the simulated positions, velocities, and currents."""
        self.check_connected()

        return MotorRead(
            position=np.array([self._pos[mid] for mid in self.motor_ids]),
            velocity=np.array([self._vel[mid] for mid in self.motor_ids]),
            current=np.array([self._cur[mid] for mid in self.motor_ids]),
        )

    def read_temperature(self) -> np.ndarray:
        """Reads and returns the simulated temperatures."""
        self.check_connected()
        return np.array([random.uniform(40, 60) for _ in self.motor_ids])

    def read_status_is_done_moving(self) -> bool:
        """The simulated motors always reach their target immediately."""
        self.check_connected()
        return True

    def write_desired_pos(self, motor_ids: Sequence[int],
                          positions: np.ndarray) -> None:
        """Writes the given desired positions, clamped to the one-turn range.

        Args:
            motor_ids: The motor IDs to write to.
            positions: The joint angles in radians to write.
        """
        assert len(motor_ids) == len(positions)
        self.check_connected()

        low, high = self.position_range_rad
        for mid, position in zip(motor_ids, positions):
            if mid not in self._pos:
                logging.error('Write ignored for unknown motor ID %d', mid)
                continue
            self._pos[mid] = float(np.clip(position, low, high))

    def write_desired_current(self, motor_ids: Sequence[int],
                              currents: np.ndarray) -> None:
        """Stores the per-motor torque limit the currents map to."""
        assert len(motor_ids) == len(currents)
        self.check_connected()

        for mid, current in zip(motor_ids, currents):
            if mid not in self._cur:
                logging.error('Write ignored for unknown motor ID %d', mid)
                continue
            self._cur[mid] = current
            self._torque[mid] = int(np.clip(abs(current), 0, 1000))

    def write_profile_velocity(self, motor_ids: Sequence[int],
                               profile_velocity: np.ndarray) -> None:
        """Stores the per-motor goal speed, in raw units of 0.732 RPM."""
        assert len(motor_ids) == len(profile_velocity)
        self.check_connected()

        for mid, speed in zip(motor_ids, profile_velocity):
            if mid not in self._profile_velocity:
                logging.error('Write ignored for unknown motor ID %d', mid)
                continue
            self._profile_velocity[mid] = speed

    @property
    def requires_offset_calibration(self) -> bool:
        return True

    def calibrate_offset(self, motor_id: int, upper: bool = True) -> bool:
        """Simulates shifting a motor's position frame; always acknowledges."""
        self.check_connected()
        return True

    def check_connected(self) -> None:
        """Ensures the client is connected."""
        if self.lazy_connect and not self._connected:
            self.connect()
        if not self._connected:
            raise OSError('Must call connect() first.')

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


# Register global cleanup function.
atexit.register(feetech_cleanup_handler)
