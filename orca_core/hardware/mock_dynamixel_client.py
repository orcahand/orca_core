# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Communication using a simulated Dynamixel client."""

import logging
from dataclasses import asdict, replace
from typing import Sequence, Union

import numpy as np

from .dynamixel_client import DynamixelClient
from .mock_motor_client import MockMotorClient
from .motor_client import ServoGains, ServoProfile


class MockDynamixelClient(MockMotorClient):
    """Mock client for simulating communication with Dynamixel motors.

    NOTE: This only supports Protocol 2.
    """

    real_client = DynamixelClient
    default_operating_mode = 3
    # Factory-default position P gain; the rest start at zero, matching a
    # servo straight out of the box.
    default_servo_gains = ServoGains(kp=800, ki=0, kd=0, ff_1st=0, ff_2nd=0)
    # Factory state: no profile at all, so Goal Position is a step.
    default_servo_profile = ServoProfile(velocity_rad_s=0.0, acceleration_rad_s2=0.0)

    # Simulated hardstops of the ORCA hand, in radians.
    _max_motor_pos = 1.0
    _min_pos = -1.0

    def set_operating_mode(self, motor_ids: Sequence[int], mode_value: int):
        """
        see https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#operating-mode11
        0: current control mode
        1: velocity control mode
        3: position control mode
        4: multi-turn position control mode
        5: current-based position control mode
        """
        self.check_connected()
        for mid in motor_ids:
            if mid not in self._operating_mode:
                logging.error('Failed to set operating mode for motor %d: unknown ID', mid)
                continue
            self._operating_mode[mid] = mode_value
            logging.info('Set operating mode for motor %d to %d', mid, mode_value)

    def write_servo_gains(self, gains) -> None:
        """Merge the named fields, leaving ``None`` fields as they were."""
        self.check_connected()
        for motor_id, entry in gains.items():
            motor_id = int(motor_id)
            current = self._servo_gains.get(motor_id)
            if current is None:
                continue
            self._servo_gains[motor_id] = replace(current, **{
                field: value
                for field, value in asdict(entry).items() if value is not None
            })

    def write_servo_profile(self, profiles) -> None:
        self.check_connected()
        for motor_id, entry in profiles.items():
            motor_id = int(motor_id)
            current = self._servo_profiles.get(motor_id)
            if current is None:
                continue
            self._servo_profiles[motor_id] = replace(current, **{
                field: value
                for field, value in asdict(entry).items() if value is not None
            })

    def write_desired_pos(self, motor_ids: Sequence[int],
                          positions: np.ndarray) -> None:
        """Writes the given desired positions, clamped to the simulated hardstops.

        Args:
            motor_ids: The motor IDs to write to.
            positions: The joint angles in radians to write.
        """
        self._write_clamped_positions(
            motor_ids, positions, self._min_pos, self._max_motor_pos)

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
        return []

    def sync_write(self, motor_ids: Sequence[int],
                   values: Sequence[Union[int, float]], address: int,
                   size: int) -> None:
        """Writes values to a group of motors.

        Args:
            motor_ids: The motor IDs to write to.
            values: The values to write.
            address: The control table address to write to.
            size: The size of the control table value being written to.
        """
        self.check_connected()
