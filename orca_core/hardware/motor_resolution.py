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
from ..utils.utils import read_yaml

if TYPE_CHECKING:
    from ..hand_config import OrcaHandConfig

logger = logging.getLogger(__name__)


def trial_probe(config: "OrcaHandConfig", port: str) -> "tuple[str | None, int | None]":
    """Probe ``port`` until a (motor_type, baudrate) combination responds.

    Iterates each motor family x that family's baud rates, fastest path first:
    the rates listed in :data:`~orca_core.constants.MOTOR_BAUD_RATES`, then any
    remaining rate the family's client accepts.

    A ``motor_type`` or ``baudrate`` pinned in ``config`` is tried first and
    alone. When the pinned combination stays silent the probe falls back to the
    full unpinned sweep, so a hand whose motors were swapped for another family
    still comes up on its bundled config.

    Raises:
        PortHeldError: another client already holds ``port``. Probing it
            anyway would put traffic on a bus someone else is driving, and
            "no motor answered" would be the wrong diagnosis — load-bearing
            for a second hand's detection never poking the first hand's bus.
    """
    from .motor_client import PortHeldError
    from .sensing.serial_discovery import port_in_use

    if port_in_use(port):
        raise PortHeldError(
            f"{port} is held by another client, so the driver probe was "
            f"skipped. Close the other hand, script or serial monitor "
            f"holding it."
        )

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
            "Pinned motor driver (%s @ %s baud) did not respond on %s; "
            "sweeping every supported family and baud rate.",
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


def persist_resolved_driver(resolved: "OrcaHandConfig") -> None:
    """Best-effort persistence of auto-detected driver fields to config.yaml.

    Only what the yaml itself leaves unset is filled in, so the next connect
    short-circuits the probe; clear those fields to trigger a fresh probe. A
    value the yaml pins is left alone even when another one answered — the file
    states the operator's intent, and :func:`trial_probe` already warns and
    sweeps when a pin stays silent. The comparison is against the file rather
    than the in-memory config, which hand detection may have patched for this
    connect. Packaged models are never rewritten: one file backs every hand of
    that model, and this hand's driver is not a property of the model.

    All fields are written in a single atomic rewrite. A write failure (e.g. a
    read-only install) is logged and never raised: the connect stays valid and
    the probe simply runs again next time.
    """
    if _is_packaged_model(resolved.config_path):
        return
    try:
        updates = _driver_updates(read_yaml(resolved.config_path) or {}, resolved)
        if not updates:
            return
        _atomic_yaml_update(resolved.config_path, updates)
    except (OSError, yaml.YAMLError) as e:
        logger.warning(
            "Could not persist auto-detected driver fields to %s (%s); "
            "the next connect will re-probe.",
            resolved.config_path,
            e,
        )
        return
    logger.info(
        "Wrote auto-detected %s to %s.",
        ", ".join(updates),
        os.path.basename(resolved.config_path),
    )


def _driver_updates(pinned: dict, resolved: "OrcaHandConfig") -> dict:
    """Driver fields to write back: only what the yaml itself leaves unset.

    The port is deliberately never written, even when a resolved port
    disagrees with an explicit pin — a config's ``port:`` is a multi-hand
    identity boundary as much as a device path: silently repointing it to
    whatever answered would risk pointing one hand's config at another
    hand's port. See tests/test_multi_hand_safety.py.
    """
    updates = {}
    if pinned.get("motor_type") is None and resolved.motor_type is not None:
        updates["motor_type"] = resolved.motor_type
    if pinned.get("baudrate") is None and resolved.baudrate is not None:
        updates["baudrate"] = resolved.baudrate
    return updates


def _is_packaged_model(config_path: str) -> bool:
    """Whether ``config_path`` is one of the models shipped inside the package."""
    models_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "models")
    return os.path.abspath(config_path).startswith(os.path.abspath(models_dir) + os.sep)


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
