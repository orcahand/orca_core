# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Connect-time motor-driver resolution.

Discovers which (motor_type, baudrate) combination a hand's motors answer on
when ``config.yaml`` doesn't pin them, and persists what a probe found so the
next connect skips it.
"""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from ..constants import MOTOR_BAUD_RATES, SUPPORTED_MOTOR_TYPES
from ..utils.utils import update_yaml

if TYPE_CHECKING:
    from ..hand_config import OrcaHandConfig


def trial_probe(config: "OrcaHandConfig", port: str) -> "tuple[str | None, int | None]":
    """Probe ``port`` until a (motor_type, baudrate) combination responds.

    Iterates each motor family x the baudrates listed in
    :data:`~orca_core.constants.MOTOR_BAUD_RATES`. If ``motor_type`` or
    ``baudrate`` is pinned in ``config``, that dimension is fixed and the
    probe only iterates the other.
    """
    from .dynamixel_client import DynamixelClient
    from .feetech_client import FeetechClient

    candidates = {
        "dynamixel": DynamixelClient,
        "feetech": FeetechClient,
    }
    motor_types = (
        [config.motor_type] if config.motor_type else list(SUPPORTED_MOTOR_TYPES)
    )
    for motor_type in motor_types:
        client_cls = candidates.get(motor_type)
        if client_cls is None:
            continue
        baudrates = (
            [config.baudrate]
            if config.baudrate is not None
            else list(MOTOR_BAUD_RATES.get(motor_type, []))
        )
        for baudrate in baudrates:
            print(f"Probing {motor_type} on {port} @ {baudrate} baud...")
            try:
                if client_cls.probe(port, baudrate, config.motor_ids):
                    print(f"  -> {motor_type} responded at {baudrate} baud.")
                    return motor_type, baudrate
            except Exception as e:
                print(f"  -> probe errored: {e}")
    return None, None


def persist_resolved_driver(
    existing: "OrcaHandConfig", resolved: "OrcaHandConfig"
) -> None:
    """Persist auto-detected driver fields to config.yaml.

    Each field is only written when it was missing (or, for the port,
    different) in yaml before this connect. Once written, the next
    connect short-circuits the probe and uses the yaml values directly.
    Clear the yaml fields to trigger a fresh probe.
    """
    updates = {}
    if existing.port != resolved.port and existing.port != "auto":
        updates["port"] = resolved.port
    if existing.motor_type is None and resolved.motor_type is not None:
        updates["motor_type"] = resolved.motor_type
    if existing.baudrate is None and resolved.baudrate is not None:
        updates["baudrate"] = resolved.baudrate
    for key, value in updates.items():
        update_yaml(resolved.config_path, key, value)
    if updates:
        print(
            f"Wrote auto-detected {', '.join(updates.keys())} to "
            f"{os.path.basename(resolved.config_path)}."
        )
