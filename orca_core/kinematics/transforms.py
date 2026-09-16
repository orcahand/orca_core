# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Minimal rigid-body transforms (numpy only).

A :class:`Transform` is an immutable SE(3) rigid transform stored as a 4x4
homogeneous matrix. Rotations follow the URDF ``rpy`` convention: fixed-axis
(extrinsic) rolls about X, then Y, then Z, i.e. ``R = Rz @ Ry @ Rx``.
Translations are in meters, angles in radians.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


def _rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def rotation_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rotation matrix for ``angle`` radians about the (unit) ``axis`` (Rodrigues)."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    k = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


@dataclass(frozen=True, eq=False)
class Transform:
    """Immutable rigid transform; maps points from the child frame to the parent frame.

    Value semantics: two transforms are equal iff their matrices are element-wise
    identical (exact, no tolerance), and equal transforms hash alike, so
    transforms can be used as set members and dict keys.
    """

    matrix: np.ndarray = field(default_factory=lambda: np.eye(4))

    def __post_init__(self):
        matrix = np.asarray(self.matrix, dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError(f"expected a 4x4 matrix, got shape {matrix.shape}")
        matrix = matrix.copy()
        matrix.flags.writeable = False
        object.__setattr__(self, "matrix", matrix)

    @classmethod
    def _from_trusted(cls, matrix: np.ndarray) -> Transform:
        """Wrap a freshly built, correctly shaped matrix without re-validating it."""
        obj = object.__new__(cls)
        matrix.flags.writeable = False
        object.__setattr__(obj, "matrix", matrix)
        return obj

    @classmethod
    def identity(cls) -> Transform:
        return cls._from_trusted(np.eye(4))

    @classmethod
    def from_xyz_rpy(cls, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> Transform:
        """Build from a URDF-style origin: translation (m) and fixed-axis rpy (rad)."""
        matrix = np.eye(4)
        matrix[:3, :3] = _rotation_from_rpy(*rpy)
        matrix[:3, 3] = xyz
        return cls._from_trusted(matrix)

    @classmethod
    def from_rotation_translation(cls, rotation: np.ndarray, translation) -> Transform:
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = translation
        return cls._from_trusted(matrix)

    @property
    def rotation(self) -> np.ndarray:
        """The (3, 3) rotation block."""
        return self.matrix[:3, :3]

    @property
    def translation(self) -> np.ndarray:
        """The (3,) translation, meters."""
        return self.matrix[:3, 3]

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Transform):
            return NotImplemented
        return bool(np.array_equal(self.matrix, other.matrix))

    def __hash__(self) -> int:
        # Tuple-of-floats, not raw bytes: keeps -0.0 and 0.0 hashing alike, as they compare equal.
        return hash(tuple(self.matrix.ravel()))

    def __matmul__(self, other: Transform) -> Transform:
        return Transform._from_trusted(self.matrix @ other.matrix)

    def inverse(self) -> Transform:
        rotation_t = self.rotation.T
        matrix = np.eye(4)
        matrix[:3, :3] = rotation_t
        matrix[:3, 3] = -rotation_t @ self.translation
        return Transform._from_trusted(matrix)

    def apply_to_points(self, points: np.ndarray) -> np.ndarray:
        """Transform ``(n, 3)`` (or ``(3,)``) points: rotation + translation."""
        points = np.asarray(points, dtype=float)
        return points @ self.rotation.T + self.translation

    def apply_to_vectors(self, vectors: np.ndarray) -> np.ndarray:
        """Rotate ``(n, 3)`` (or ``(3,)``) free vectors (e.g. forces): no translation."""
        vectors = np.asarray(vectors, dtype=float)
        return vectors @ self.rotation.T
