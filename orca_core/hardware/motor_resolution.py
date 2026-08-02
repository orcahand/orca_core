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

import logging
import os
import tempfile
from typing import TYPE_CHECKING

import yaml

from ..constants import MOTOR_BAUD_RATES, SUPPORTED_MOTOR_TYPES

if TYPE_CHECKING:
    from ..hand_config import OrcaHandConfig

logger = logging.getLogger(__name__)


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
            logger.info("Probing %s on %s @ %d baud...", motor_type, port, baudrate)
            try:
                if client_cls.probe(port, baudrate, config.motor_ids):
                    logger.info("%s responded on %s at %d baud.", motor_type, port, baudrate)
                    return motor_type, baudrate
            except Exception as e:
                logger.warning("Probe of %s @ %d baud errored: %s", motor_type, baudrate, e)
    return None, None


def persist_resolved_driver(
    existing: "OrcaHandConfig", resolved: "OrcaHandConfig"
) -> None:
    """Best-effort persistence of auto-detected driver fields to config.yaml.

    Each field is only written when it was missing (or, for the port,
    different) in yaml before this connect. Once written, the next
    connect short-circuits the probe and uses the yaml values directly.
    Clear the yaml fields to trigger a fresh probe.

    All resolved fields are written in a single atomic rewrite. A write
    failure (e.g. a read-only install) is logged and never raised: the
    connect stays valid and the probe simply runs again next time.
    """
    updates = {}
    if existing.port != resolved.port and existing.port != "auto":
        updates["port"] = resolved.port
    if existing.motor_type is None and resolved.motor_type is not None:
        updates["motor_type"] = resolved.motor_type
    if existing.baudrate is None and resolved.baudrate is not None:
        updates["baudrate"] = resolved.baudrate
    if not updates:
        return
    try:
        _atomic_yaml_update(resolved.config_path, updates)
    except (OSError, yaml.YAMLError) as e:
        logger.warning(
            "Could not persist auto-detected %s to %s (%s); the next connect will re-probe.",
            ", ".join(updates),
            resolved.config_path,
            e,
        )
        return
    logger.info(
        "Wrote auto-detected %s to %s.",
        ", ".join(updates),
        os.path.basename(resolved.config_path),
    )


def _atomic_yaml_update(path: str, updates: dict) -> None:
    """Apply ``updates`` to the yaml mapping at ``path`` in one atomic replace.

    Unrelated keys are preserved. The rewrite goes through a temp file in the
    same directory plus ``os.replace``, so a crash mid-write can never leave a
    half-written config behind.
    """
    if os.path.exists(path) and not os.access(path, os.W_OK):
        raise PermissionError(f"{path} is not writable")
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    data.update(updates)
    directory = os.path.dirname(os.path.abspath(path))
    fd, tmp_path = tempfile.mkstemp(
        dir=directory, prefix=".config-", suffix=".yaml"
    )
    try:
        try:
            os.chmod(tmp_path, os.stat(path).st_mode & 0o777)
        except OSError:
            pass
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        os.replace(tmp_path, path)
    except BaseException:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise
