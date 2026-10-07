# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Shared base for the in-memory motor clients.

A mock describes its family by naming the real client class in ``real_client``;
the family's capability attributes are copied from it, so a mock hand behaves
like the family it stands in for.
"""

import logging
import random
from typing import ClassVar, Sequence

import numpy as np

from .motor_client import MotorClient, MotorRead, ServoGains, ServoProfile

# Capability attributes a mock takes from its real client, so none is retyped.
FAMILY_ATTRIBUTES = (
    "motor_type",
    "factory_default_id",
    "factory_default_baudrate",
    "baud_rate_map",
    "requires_unpowered_hotplug",
    "supports_multi_turn",
    "supported_modes",
    "position_range_rad",
    "waits_for_motion",
    "arrival_tolerance_rad",
    "servo_gain_max",
    "profile_velocity_max_rad_s",
    "profile_acceleration_max_rad_s2",
    "profile_ceiling_source",
    "no_load_speed_rad_s",
    "default_profile_velocity_fraction",
    "default_profile_acceleration_rad_s2",
    "max_operating_temp_c",
    "hardware_error_bits",
    "current_scale_ma",
    "max_current_ma",
    "default_max_current_ma",
    "default_calibration_current_ma",
)


class MockPortHandler:
    """Stands in for the SDK port handler the exit cleanup inspects."""

    def __init__(self, port: str):
        self.port_name = port
        self.is_open = False
        self.is_using = False


class MockMotorClient(MotorClient):
    """State and behaviour common to every in-memory motor client."""

    real_client: ClassVar[type]
    """The real client class this mock stands in for."""

    default_operating_mode: ClassVar[int]
    """Operating-mode value a motor of this family powers up in."""

    default_servo_gains: ClassVar[ServoGains]
    """Gains a motor of this family powers up with."""

    default_servo_profile: ClassVar[ServoProfile]
    """Trajectory limits a motor of this family powers up with."""

    # A plain attribute, so tests can force a stale read by assigning to it.
    last_read_ok = True

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        real = cls.__dict__.get("real_client")
        if real is None:
            return
        for name in FAMILY_ATTRIBUTES:
            if name not in cls.__dict__:
                setattr(cls, name, getattr(real, name))

    def __init__(self,
                 motor_ids: Sequence[int],
                 port: str = '/dev/ttyUSB0',
                 baudrate: int = 1000000):
        """Initializes a new mock client.

        Args:
            motor_ids: All motor IDs being used by the client.
            port: The serial port the real client would talk to.
            baudrate: The baudrate the real client would communicate at.
        """
        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate
        self.port_handler = MockPortHandler(port)

        self._connected = False
        self._torque_enabled = {mid: False for mid in self.motor_ids}
        self._operating_mode = {mid: self.default_operating_mode for mid in self.motor_ids}
        self._pos = {mid: 0.0 for mid in self.motor_ids}
        self._vel = {mid: 0.0 for mid in self.motor_ids}
        self._cur = {mid: 0.0 for mid in self.motor_ids}
        # Per-motor goal-current ceilings a test may script, in mA; None = no current register.
        self.current_ceilings_ma: dict = {}
        self._servo_gains = {int(mid): self.default_servo_gains for mid in self.motor_ids}
        self._servo_profiles = {int(mid): self.default_servo_profile for mid in self.motor_ids}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        """Connects to the simulated motors and registers for exit cleanup, leaving motor state as-is."""
        if self._connected:
            raise RuntimeError('Client is already connected.')

        logging.info('Succeeded to open port: %s', self.port_name)

        self._connected = True
        self.port_handler.is_open = True

        self._register_open()

    def disconnect(self) -> None:
        """Disconnects and deregisters, then re-raises any torque-disable failure, as the real clients do."""
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

    def read_position_velocity_current(self) -> MotorRead:
        """Returns the simulated positions, velocities, and currents."""
        self.check_connected()

        return MotorRead(
            position=np.array([self._pos[mid] for mid in self.motor_ids]),
            velocity=np.array([self._vel[mid] for mid in self.motor_ids]),
            current=np.array([self._cur[mid] for mid in self.motor_ids]),
        )

    def read_hardware_error(self, motor_id: int) -> "int | None":
        """A configured motor answers fault-free; any other ID is silent."""
        self.check_connected()
        return 0 if motor_id in self.motor_ids else None

    def read_temperature(self) -> np.ndarray:
        """Reads and returns the simulated temperatures."""
        self.check_connected()
        return np.array([random.uniform(40, 60) for _ in self.motor_ids])

    def wait_for_motion_complete(self, timeout: float = 5.0,
                                 poll_interval: float = 0.02) -> None:
        """Returns at once, since a mock write lands immediately."""
        self.check_connected()

    def _write_clamped_positions(self, motor_ids: Sequence[int],
                                 positions: np.ndarray,
                                 low: float, high: float) -> None:
        """Store goal positions clamped to ``low``..``high``, skipping unknown IDs."""
        assert len(motor_ids) == len(positions)
        self.check_connected()

        for mid, position in zip(motor_ids, positions):
            if mid not in self._pos:
                logging.error('Write ignored for unknown motor ID %d', mid)
                continue
            self._pos[mid] = float(np.clip(position, low, high))

    def write_desired_current(self, motor_ids: Sequence[int],
                              currents: np.ndarray) -> None:
        """Stores each motor's goal-current limit."""
        self.check_connected()

        for mid, raw in self._goal_current_plan(motor_ids, currents).items():
            if mid not in self._cur:
                logging.error('Write ignored for unknown motor ID %d', mid)
                continue
            self._cur[mid] = raw * self.current_scale_ma

    def _current_ceiling_ma(self, motor_id: int) -> "float | None":
        return self.current_ceilings_ma.get(motor_id, self.max_current_ma)

    def read_servo_gains(self, motor_ids: Sequence[int]):
        self.check_connected()
        return {int(mid): self._servo_gains.get(int(mid)) for mid in motor_ids}

    def read_servo_profile(self, motor_ids: Sequence[int]):
        self.check_connected()
        return {int(mid): self._servo_profiles.get(int(mid)) for mid in motor_ids}
