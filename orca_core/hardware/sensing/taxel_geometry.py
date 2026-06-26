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

* **Index alignment.** Position row ``i`` corresponds to taxel ``i`` in the
  decoded force stream (see ``decode_taxels_auto``). Positions ``(n, 3)`` and a
  frame's forces ``(n, 3)`` therefore line up by row with no bookkeeping.
* **Frame.** Positions are in the sensor-local *fingertip* frame, in
  millimetres. A future kinematics layer can transform them into the hand/world
  frame using the joint angles; the ``frame`` label is the hook for that.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import yaml

from orca_core.hardware.sensing.constants import FINGER_MODELS, FingerName

MODELS_DIR = os.path.join(os.path.dirname(__file__), "models")

FINGERTIP_LOCAL_FRAME = "fingertip_local"
"""Coordinate frame of the loaded positions: relative to the fingertip itself."""


@dataclass(frozen=True)
class TaxelGeometry:
    """Fixed taxel positions for one finger's sensor.

    ``positions`` is an ``(n_taxels, 3)`` array of ``[x, y, z]`` in millimetres;
    row ``i`` is taxel ``i`` in the force stream. ``frame`` records which
    coordinate frame the positions are expressed in (currently always
    ``"fingertip_local"``).
    """

    finger: str
    model: str
    positions: np.ndarray
    frame: str = FINGERTIP_LOCAL_FRAME

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
        _model_cache[model_name] = np.array(
            [[c["x"], c["y"], c["z"]] for c in coords], dtype=float
        )
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
