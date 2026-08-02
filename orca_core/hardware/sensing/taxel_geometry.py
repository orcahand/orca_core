# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Static taxel geometry for ORCA tactile sensors.

Each taxel sits at a fixed position on its sensor — a property of the sensor's
mechanical design, not something that changes while the hand runs. Those
positions live in the per-model config YAMLs under ``models/`` and are loaded
once here.

Two facts make this geometry trivial to use alongside the force stream:

* **Index alignment.** Position row ``i`` is intended to correspond to taxel
  ``i`` in the decoded force stream (see ``decode_taxels_auto``), so positions
  ``(n, 3)`` and a frame's forces ``(n, 3)`` line up by row with no bookkeeping.
* **Frame.** Positions are in the *sensor* frame: origin on the mounting plane
  at the connector tail, X across the sensor width, Y along the sensor toward
  the fingertip dome, Z outward through the sensing surface (right-handed).
  The streamed per-taxel forces use the same axes. Positions are converted to
  meters on load; the YAMLs store millimeters as supplied by the vendor.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import yaml

from orca_core.constants import FingerName
from orca_core.hardware.sensing.constants import FINGER_MODELS

MODELS_DIR = os.path.join(os.path.dirname(__file__), "models")

SENSOR_FRAME = "sensor"
"""Coordinate frame of the loaded positions: the sensor's own frame (see module docstring)."""

_YAML_UNITS_TO_METERS = {"mm": 1e-3, "m": 1.0}


@dataclass(frozen=True)
class TaxelGeometry:
    """Fixed taxel positions for one finger's sensor.

    ``positions`` is a read-only ``(n_taxels, 3)`` array of ``[x, y, z]`` in
    meters; row ``i`` is taxel ``i`` in the force stream. ``frame`` records
    which coordinate frame the positions are expressed in.
    """

    finger: str
    model: str
    positions: np.ndarray
    frame: str = SENSOR_FRAME

    @property
    def num_taxels(self) -> int:
        return len(self.positions)


_model_cache: dict[str, np.ndarray] = {}


def _load_model_positions(model_name: str) -> np.ndarray:
    """Load and cache the ``(n_taxels, 3)`` position array for a sensor model."""
    if model_name not in _model_cache:
        config_path = os.path.join(MODELS_DIR, model_name, "config.yaml")
        with open(config_path) as f:
            config = yaml.safe_load(f)
        coords = config["coordinates"]
        declared = config.get("num_taxels")
        if declared is not None and declared != len(coords):
            raise ValueError(
                f"{model_name}: num_taxels={declared} disagrees with "
                f"{len(coords)} coordinate rows"
            )
        frame = config.get("frame", SENSOR_FRAME)
        if frame != SENSOR_FRAME:
            raise ValueError(f"{model_name}: unsupported frame {frame!r}")
        units = config.get("units", "mm")
        if units not in _YAML_UNITS_TO_METERS:
            raise ValueError(f"{model_name}: unsupported units {units!r}")
        positions = _YAML_UNITS_TO_METERS[units] * np.array(
            [[c["x"], c["y"], c["z"]] for c in coords], dtype=float
        )
        # Cached and shared across callers, so freeze it: an accidental in-place
        # write would otherwise poison the geometry for every later caller.
        positions.flags.writeable = False
        _model_cache[model_name] = positions
    return _model_cache[model_name]


def load_taxel_geometry(finger: FingerName) -> TaxelGeometry:
    """Return the static taxel geometry for ``finger`` from its sensor model."""
    model = FINGER_MODELS[finger]
    return TaxelGeometry(finger=finger, model=model, positions=_load_model_positions(model))


def load_all_taxel_geometry(
    fingers: list[FingerName] | None = None,
) -> dict[str, TaxelGeometry]:
    """Return ``{finger: TaxelGeometry}`` for ``fingers`` (all known fingers if ``None``)."""
    if fingers is None:
        fingers = list(FINGER_MODELS)
    return {finger: load_taxel_geometry(finger) for finger in fingers}
