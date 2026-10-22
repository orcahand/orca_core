# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""``setup.yaml``: everything the setup routines record about a hand, in one
file beside its ``calibration.yaml``.

One top-level section per kind of record (``slack_tests``, ``sweeps``,
``travel_baselines``, ``tension_progress``, ``last_tension``); records inside
are keyed by the hand they were made on where that matters. Each section is
read and replaced whole under a process-wide lock, so two writers never lose
each other's sections.

A section missing from ``setup.yaml`` is read from the separate file it used
to live in (``legacy``), so existing data keeps working; the next write of
that section moves it into ``setup.yaml``. The old file is left in place.
"""

from __future__ import annotations

import copy
import os
import threading
from typing import Any, Callable, Optional

from ..utils.utils import read_yaml, write_yaml_atomic

SETUP_FILE = "setup.yaml"

_lock = threading.RLock()


def setup_path(config) -> str:
    return os.path.join(os.path.dirname(config.calibration_path), SETUP_FILE)


def _read(path: str) -> dict:
    data = read_yaml(path) if os.path.exists(path) else None
    return data if isinstance(data, dict) else {}


def read_section(config, name: str, legacy: Optional[str] = None) -> Any:
    """The ``name`` section, or the content of the ``legacy`` file beside it
    while the section has never been written; None when neither exists."""
    with _lock:
        data = _read(setup_path(config))
        if name in data:
            return copy.deepcopy(data[name])
        if legacy:
            old = os.path.join(os.path.dirname(config.calibration_path), legacy)
            if os.path.exists(old):
                return read_yaml(old)
        return None


def write_section(config, name: str, value: Any) -> None:
    """Replace the ``name`` section; ``None`` removes it."""
    with _lock:
        path = setup_path(config)
        data = _read(path)
        if value is None:
            data.pop(name, None)
        else:
            data[name] = value
        write_yaml_atomic(path, data)


def update_section(
    config, name: str, fn: Callable[[Any], Any], legacy: Optional[str] = None
) -> Any:
    """Read the section, pass it to ``fn`` and store what ``fn`` returns, as
    one step under the lock; returns the stored value."""
    with _lock:
        value = fn(read_section(config, name, legacy))
        write_section(config, name, value)
        return value
