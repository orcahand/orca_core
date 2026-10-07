# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Communication using a simulated Feetech client."""

import logging
from typing import Sequence

import numpy as np

from .feetech_client import DEFAULT_ACC, DEFAULT_SPEED, FeetechClient
from .feetech_registers import HLS
from .mock_motor_client import MockMotorClient
from .motor_client import ServoGains, ServoProfile


class MockFeetechClient(MockMotorClient):
    """Mock client for simulating communication with Feetech SCServo motors.

    Carries FeetechClient's capability attributes, so code that branches on
    them (mode support, position bounds, offset calibration) behaves the same
    against the mock as against the hardware.
    """

    real_client = FeetechClient
    # Servo mode, the family's power-on default.
    default_operating_mode = 0
    # The gains a Feetech servo powers up with, from its EEPROM
    # counterparts. Read off a real chain: Kp 32, Kd 32, Ki 0.
    default_servo_gains = ServoGains(kp=32, ki=0, kd=32)
    # What orca_core writes at mode-set time, in SI units.
    default_servo_profile = ServoProfile(
        velocity_rad_s=DEFAULT_SPEED * HLS.SPEED_SCALE_RAD_S,
        acceleration_rad_s2=DEFAULT_ACC * HLS.ACC_SCALE_RAD_S2,
    )

    def __init__(self,
                 motor_ids: Sequence[int],
                 port: str = '/dev/ttyUSB0',
                 baudrate: int = 1000000):
        super().__init__(motor_ids, port, baudrate)

    def write_servo_profile(self, profiles) -> None:
        """Merge the named fields, translating 0 the way the hardware does.

        Goal speed 0 stops this family rather than uncapping it, so the
        client turns a requested 0 into the register maximum; the mock
        reports back what the hardware would then hold.
        """
        self.check_connected()
        for motor_id, wanted in profiles.items():
            motor_id = int(motor_id)
            current = self._servo_profiles.get(motor_id) or ServoProfile()
            velocity = current.velocity_rad_s
            if wanted.velocity_rad_s is not None:
                value = float(wanted.velocity_rad_s)
                if value < 0:
                    raise ValueError("velocity must be non-negative")
                velocity = 0.0 if value == 0.0 else value
            acceleration = current.acceleration_rad_s2
            if wanted.acceleration_rad_s2 is not None:
                value = float(wanted.acceleration_rad_s2)
                if value < 0:
                    raise ValueError("acceleration must be non-negative")
                acceleration = min(
                    value, HLS.ACC_MAX_RAW * HLS.ACC_SCALE_RAD_S2)
            self._servo_profiles[motor_id] = ServoProfile(
                velocity_rad_s=velocity, acceleration_rad_s2=acceleration)

    def write_servo_gains(self, gains) -> None:
        """Merge the named fields; ``None`` leaves one as it was.

        Refuses feedforward for the same reason the hardware client does:
        this family's position loop is PID only.
        """
        self.check_connected()
        for motor_id, wanted in gains.items():
            for name in ("ff_1st", "ff_2nd"):
                if getattr(wanted, name) is not None:
                    raise ValueError(
                        f"{type(self).__name__} has no {name}: this family's "
                        "position loop is PID only")
            motor_id = int(motor_id)
            current = self._servo_gains.get(motor_id) or ServoGains()
            self._servo_gains[motor_id] = ServoGains(
                kp=current.kp if wanted.kp is None else int(wanted.kp),
                ki=current.ki if wanted.ki is None else int(wanted.ki),
                kd=current.kd if wanted.kd is None else int(wanted.kd),
            )

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

    def write_desired_pos(self, motor_ids: Sequence[int],
                          positions: np.ndarray) -> None:
        """Writes the given desired positions, clamped to the one-turn range.

        Args:
            motor_ids: The motor IDs to write to.
            positions: The joint angles in radians to write.
        """
        self._write_clamped_positions(motor_ids, positions, *self.position_range_rad)

    @property
    def requires_offset_calibration(self) -> bool:
        return True

    def calibrate_offset(self, motor_id: int, upper: bool = True) -> bool:
        """Simulates shifting a motor's position frame; always acknowledges."""
        self.check_connected()
        return True
