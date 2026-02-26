# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Abstract base class for motor communication clients."""

from abc import ABC, abstractmethod
from typing import Sequence, Tuple
import numpy as np


class MotorClient(ABC):
    """Abstract base class for motor communication clients.

    This defines the interface that all motor clients (Dynamixel, Feetech, etc.)
    must implement to work with OrcaHand.
    """

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Returns True if the client is connected to the motors."""
        ...

    @abstractmethod
    def connect(self) -> None:
        """Connects to the motors."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnects from the motors."""
        ...

    @abstractmethod
    def set_torque_enabled(
        self,
        motor_ids: Sequence[int],
        enabled: bool,
        retries: int = -1,
        retry_interval: float = 0.25
    ) -> None:
        """Sets whether torque is enabled for the specified motors.

        Args:
            motor_ids: The motor IDs to configure.
            enabled: Whether to engage or disengage the motors.
            retries: The number of times to retry. If <0, will retry forever.
            retry_interval: The number of seconds to wait between retries.
        """
        ...

    @abstractmethod
    def set_operating_mode(self, motor_ids: Sequence[int], mode: int) -> None:
        """Sets the operating mode for the specified motors.

        Args:
            motor_ids: The motor IDs to configure.
            mode: The operating mode value:
                0: current control mode
                1: velocity control mode
                3: position control mode
                4: multi-turn position control mode
                5: current-based position control mode
        """
        ...

    @abstractmethod
    def read_pos_vel_cur(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Reads the current position, velocity, and current for all motors.

        Returns:
            A tuple of (positions, velocities, currents) as numpy arrays.
            Positions are in radians, velocities in rad/s, currents in mA.
        """
        ...

    @abstractmethod
    def read_temperature(self) -> np.ndarray:
        """Reads the current temperature for all motors.

        Returns:
            An array of temperatures in degrees Celsius.
        """
        ...

    @abstractmethod
    def write_desired_pos(
        self,
        motor_ids: Sequence[int],
        positions: np.ndarray
    ) -> None:
        """Writes desired positions to the specified motors.

        Args:
            motor_ids: The motor IDs to write to.
            positions: The desired positions in radians.
        """
        ...

    @abstractmethod
    def write_desired_current(
        self,
        motor_ids: Sequence[int],
        currents: np.ndarray
    ) -> None:
        """Writes desired currents (torque limits) to the specified motors.

        Args:
            motor_ids: The motor IDs to write to.
            currents: The desired currents in mA.
        """
        ...

    @property
    def requires_offset_calibration(self) -> bool:
        """Returns True if this motor type needs offset calibration during joint calibration."""
        return False

    def calibrate_offset(self, motor_id: int, upper: bool = True) -> bool:
        """Set current physical position to read as upper or lower bound.

        Used during calibration to shift the position coordinate system,
        ensuring the motor's full range fits within valid bounds.

        Args:
            motor_id: Motor to calibrate.
            upper: If True, set to upper bound. If False, set to lower bound.

        Returns:
            True on success, False otherwise.
        """
        # Base implementation: no-op for motors that don't need this
        return True
