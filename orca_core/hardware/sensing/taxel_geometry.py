# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Static taxel geometry for ORCA tactile sensors.

Each taxel sits at a fixed position on its sensor, facing a fixed direction —
properties of the sensor's mechanical design, not something that changes while
the hand runs. Positions and per-taxel force axes live in the per-model config
YAMLs under ``models/`` and are loaded once here.

Three facts make this geometry trivial to use alongside the force stream:

* **Index alignment.** Row ``i`` of the geometry is intended to correspond to
  taxel ``i`` in the decoded force stream (see ``decode_taxels_auto``), so
  positions ``(n, 3)`` and a frame's forces ``(n, 3)`` line up by row with no
  bookkeeping.
* **Sensor frame.** Positions are in the *sensor* frame: origin on the mounting
  plane at the connector tail, X across the sensor width, Y along the sensor
  toward the fingertip dome, Z outward through the sensing surface
  (right-handed). Positions are converted to meters on load; the YAMLs store
  millimeters as supplied by the vendor.
* **Taxel frames.** Each taxel measures force in its own local axes: z along
  its outward surface normal (the unsigned pressure), x and y tangent to the
  surface (signed shear). On a curved fingertip those axes point differently
  for every taxel, so streamed per-taxel forces are *not* in sensor-frame axes.
  ``TaxelGeometry.orientations`` holds each taxel's rotation into the sensor
  frame and ``TaxelGeometry.rotate_forces`` applies them.
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
"""Coordinate frame of the loaded positions and orientations: the sensor's own frame (see module docstring)."""

_YAML_UNITS_TO_METERS = {"mm": 1e-3, "m": 1.0}


_ORTHONORMAL_TOLERANCE = 1e-4
"""Largest per-element deviation from a rotation accepted for a taxel's YAML axes."""


@dataclass(frozen=True)
class TaxelGeometry:
    """Fixed taxel positions and force axes for one finger's sensor.

    ``positions`` is a read-only ``(n_taxels, 3)`` array of ``[x, y, z]`` in
    meters; row ``i`` is taxel ``i`` in the force stream. ``orientations`` is
    a read-only ``(n_taxels, 3, 3)`` array of rotations taking a vector from
    taxel ``i``'s local force axes into ``frame``: its columns are the taxel's
    local x, y and z axes, z being the outward surface normal. ``frame``
    records which coordinate frame both are expressed in.
    """

    finger: str
    model: str
    positions: np.ndarray
    orientations: np.ndarray
    frame: str = SENSOR_FRAME

    @property
    def num_taxels(self) -> int:
        return len(self.positions)

    @property
    def normals(self) -> np.ndarray:
        """``(n_taxels, 3)`` outward unit surface normals (each taxel's local z axis)."""
        return self.orientations[:, :, 2]

    def rotate_forces(self, forces) -> np.ndarray:
        """Rotate per-taxel forces from each taxel's local axes into ``frame``.

        ``forces`` is ``(n_taxels, 3)`` as streamed (``[fx, fy, fz]`` per taxel,
        fz along the taxel's normal). Returns a new ``(n_taxels, 3)`` array;
        magnitudes are preserved.
        """
        forces = np.asarray(forces, dtype=float)
        if forces.shape != self.positions.shape:
            raise ValueError(
                f"{self.finger}: expected forces of shape {self.positions.shape}, "
                f"got {forces.shape}"
            )
        return np.einsum("nij,nj->ni", self.orientations, forces)


_model_cache: dict[str, tuple[np.ndarray, np.ndarray]] = {}


def _load_orientations(model_name: str, coords: list[dict]) -> np.ndarray:
    """Return ``(n, 3, 3)`` local-to-sensor rotations from the YAML's per-taxel axes."""
    try:
        axes = np.array(
            [[c["x_axis"], c["y_axis"], c["z_axis"]] for c in coords], dtype=float
        )
    except KeyError as e:
        raise ValueError(
            f"{model_name}: every coordinate row needs x_axis, y_axis and z_axis "
            f"(missing {e})"
        ) from e
    if axes.shape != (len(coords), 3, 3):
        raise ValueError(
            f"{model_name}: taxel axes must be three 3-vectors per coordinate row"
        )
    rotations = np.transpose(axes, (0, 2, 1))
    # Snap to the nearest exact rotation; the YAML values are rounded.
    u, _, vt = np.linalg.svd(rotations)
    nearest = u @ vt
    bad = np.flatnonzero(
        (np.abs(nearest - rotations).max(axis=(1, 2)) > _ORTHONORMAL_TOLERANCE)
        | (np.linalg.det(nearest) < 0)
    )
    if bad.size:
        raise ValueError(
            f"{model_name}: axes of taxel(s) {bad.tolist()} are not a "
            f"right-handed orthonormal set"
        )
    return nearest


def _load_model(model_name: str) -> tuple[np.ndarray, np.ndarray]:
    """Load and cache ``(positions (n, 3), orientations (n, 3, 3))`` for a sensor model."""
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
        orientations = _load_orientations(model_name, coords)
        # Cached and shared across callers, so freeze them: an accidental in-place
        # write would otherwise poison the geometry for every later caller.
        positions.flags.writeable = False
        orientations.flags.writeable = False
        _model_cache[model_name] = (positions, orientations)
    return _model_cache[model_name]


def load_taxel_geometry(finger: FingerName) -> TaxelGeometry:
    """Return the static taxel geometry for ``finger`` from its sensor model."""
    model = FINGER_MODELS[finger]
    positions, orientations = _load_model(model)
    return TaxelGeometry(
        finger=finger, model=model, positions=positions, orientations=orientations
    )


def load_all_taxel_geometry(
    fingers: list[FingerName] | None = None,
) -> dict[str, TaxelGeometry]:
    """Return ``{finger: TaxelGeometry}`` for ``fingers`` (all known fingers if ``None``)."""
    if fingers is None:
        fingers = list(FINGER_MODELS)
    return {finger: load_taxel_geometry(finger) for finger in fingers}
