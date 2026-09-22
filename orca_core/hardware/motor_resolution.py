# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Connect-time motor-driver resolution.

Discovers which (motor_type, baudrate) combination a hand's motors answer on
when ``config.yaml`` doesn't pin them. Nothing is written back: every connect
probes afresh unless a human pinned the driver in the yaml.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from ..constants import MOTOR_BAUD_RATES, SUPPORTED_MOTOR_TYPES

if TYPE_CHECKING:
    from ..hand_config import OrcaHandConfig

logger = logging.getLogger(__name__)


def trial_probe(config: "OrcaHandConfig", port: str) -> "tuple[str | None, int | None]":
    """Probe ``port`` until a (motor_type, baudrate) combination responds.

    Iterates each motor family x that family's baud rates, fastest path first:
    the rates listed in :data:`~orca_core.constants.MOTOR_BAUD_RATES`, then any
    remaining rate the family's client accepts.

    A ``motor_type`` or ``baudrate`` pinned in ``config`` is tried first and
    alone. Only when the pin stays silent does the probe warn and widen to the
    full sweep, so a hand whose motors were swapped for another family still
    comes up.
    """
    pinned = config.motor_type is not None or config.baudrate is not None
    if pinned:
        motor_type, baudrate = _sweep(
            port,
            config.motor_ids,
            [config.motor_type] if config.motor_type else list(SUPPORTED_MOTOR_TYPES),
            config.baudrate,
        )
        if motor_type is not None:
            return motor_type, baudrate
        logger.warning(
            "config.yaml pins %s @ %s baud, which did not respond on %s; "
            "ignoring the pin and sweeping every supported family and baud "
            "rate. Clear those fields to autodetect without this attempt.",
            config.motor_type or "any", config.baudrate or "any", port,
        )
    return _sweep(port, config.motor_ids, list(SUPPORTED_MOTOR_TYPES), None)


def _sweep(
    port: str,
    motor_ids: "list[int]",
    motor_types: "list[str]",
    baudrate: "int | None",
) -> "tuple[str | None, int | None]":
    """Probe ``motor_types`` on ``port``, at ``baudrate`` alone when it is given."""
    from .motor_factory import motor_client_class

    for motor_type in motor_types:
        try:
            client_cls = motor_client_class(motor_type)
        except ValueError:
            continue
        baudrates = [baudrate] if baudrate is not None else _baudrates_for(client_cls, motor_type)
        for rate in baudrates:
            logger.info("Probing %s on %s @ %d baud...", motor_type, port, rate)
            try:
                if client_cls.probe(port, rate, motor_ids):
                    logger.info("%s responded on %s at %d baud.", motor_type, port, rate)
                    return motor_type, rate
            except Exception as e:
                logger.warning("Probe of %s @ %d baud errored: %s", motor_type, rate, e)
    return None, None


def _baudrates_for(client_cls, motor_type: str) -> "list[int]":
    """The family's priority rates first, then every other rate it accepts."""
    rates = list(MOTOR_BAUD_RATES.get(motor_type, []))
    rates.extend(r for r in client_cls.supported_baudrates() if r not in rates)
    return rates
