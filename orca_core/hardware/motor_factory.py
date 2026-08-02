# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Resolve a motor family name to its :class:`MotorClient` implementation.

The single place that maps ``motor_type`` to a concrete client. Everything else
works through :class:`MotorClient` and its class attributes, so supporting a new
motor family means adding a client and one entry here.

Clients are imported lazily: a hand running Dynamixel motors never imports the
Feetech SDK.
"""
from __future__ import annotations

from typing import Sequence

from ..constants import DYNAMIXEL, FEETECH, SUPPORTED_MOTOR_TYPES
from .motor_client import MotorClient


def motor_client_class(motor_type: str) -> type[MotorClient]:
    """Return the :class:`MotorClient` subclass driving ``motor_type``."""
    if motor_type == DYNAMIXEL:
        from .dynamixel_client import DynamixelClient

        return DynamixelClient

    if motor_type == FEETECH:
        from .feetech_client import FeetechClient

        return FeetechClient

    raise ValueError(
        f"Unknown motor_type: {motor_type}. "
        f"Expected one of [{', '.join(SUPPORTED_MOTOR_TYPES)}]."
    )


def create_motor_client(
    motor_type: str,
    motor_ids: Sequence[int],
    port: str,
    baudrate: int,
) -> MotorClient:
    """Construct (but do not connect) the client for ``motor_type``."""
    return motor_client_class(motor_type)(motor_ids, port, baudrate)
