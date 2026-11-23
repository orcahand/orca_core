"""Pinhole camera with Brown-Conrady distortion, OpenCV conventions.

Camera frame: x right, y down, z forward. Pixel (0, 0) is the centre of the
top-left pixel. Distortion (k1, k2, p1, p2, k3) as in cv2.projectPoints.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from orca_core.kinematics.transforms import Transform


@dataclass(frozen=True)
class Lens:
    fx: float
    fy: float
    cx: float
    cy: float
    size: tuple[int, int]                       # (width, height) px
    distortion: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)

    def project(self, points_cam: np.ndarray) -> np.ndarray:
        """(n, 3) points in the camera frame -> (n, 2) pixels."""
        p = np.atleast_2d(points_cam)
        x, y = p[:, 0] / p[:, 2], p[:, 1] / p[:, 2]
        k1, k2, p1, p2, k3 = self.distortion
        r2 = x * x + y * y
        radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
        xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        return np.column_stack([self.fx * xd + self.cx, self.fy * yd + self.cy])


def nominal_lens(size: tuple[int, int], diagonal_fov_deg: float = 65.0) -> Lens:
    """A distortion-free start for the lens fit, from the field of view."""
    w, h = size
    f = np.hypot(w, h) / 2 / np.tan(np.deg2rad(diagonal_fov_deg) / 2)
    return Lens(f, f, (w - 1) / 2, (h - 1) / 2, (w, h))


def brio_portrait() -> Lens:
    """Logitech BRIO at 4K on its side (2160 x 3840), narrowest field of view
    (65° diagonal): nominal values, until the lens is calibrated."""
    return nominal_lens((2160, 3840))


@dataclass(frozen=True)
class Camera:
    lens: Lens
    pose: Transform = field(default_factory=Transform.identity)   # camera in the world

    def project(self, points_world: np.ndarray) -> np.ndarray:
        return self.lens.project(self.pose.inverse().apply_to_points(points_world))

    def depth(self, points_world: np.ndarray) -> np.ndarray:
        return np.atleast_2d(self.pose.inverse().apply_to_points(points_world))[:, 2]


def look_at(eye, target, up=(0.0, 0.0, 1.0)) -> Transform:
    """Camera pose at ``eye`` looking at ``target``, image up along ``up``."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return Transform.from_rotation_translation(np.column_stack([x, y, z]), eye)
