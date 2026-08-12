# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Calibrated multi-camera rig: persistence, projection, triangulation.

The world frame is the ChArUco board frame (the board that calibrated the
rig), so every camera pose and every triangulated point live in one metric
frame with the board's caliper-verified scale. Extrinsics are stored
world->camera in the OpenCV convention: ``x_cam = R @ x_world + t``.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np
import yaml

from board import BoardSpec

RIG_FORMAT = 1


@dataclass
class Camera:
    name: str
    image_size: tuple[int, int]          # (w, h)
    K: np.ndarray                        # (3, 3)
    dist: np.ndarray                     # OpenCV distortion coefficients
    R: np.ndarray                        # (3, 3) world -> camera
    t: np.ndarray                        # (3,)
    meta: dict = field(default_factory=dict)

    @property
    def center(self) -> np.ndarray:
        """Camera center in world coordinates."""
        return -self.R.T @ self.t

    def project(self, pts_world: np.ndarray) -> np.ndarray:
        """World points (N, 3) -> distorted pixel coordinates (N, 2)."""
        rvec, _ = cv2.Rodrigues(self.R)
        px, _ = cv2.projectPoints(
            np.asarray(pts_world, float).reshape(-1, 3), rvec, self.t,
            self.K, self.dist,
        )
        return px.reshape(-1, 2)

    def undistort(self, px: np.ndarray) -> np.ndarray:
        """Distorted pixels (N, 2) -> ideal normalized coordinates (N, 2).

        NaN rows pass through as NaN so masked tracks keep their shape.
        """
        px = np.asarray(px, float).reshape(-1, 2)
        out = np.full_like(px, np.nan)
        ok = np.isfinite(px).all(axis=1)
        if ok.any():
            und = cv2.undistortPoints(px[ok].reshape(-1, 1, 2), self.K, self.dist)
            out[ok] = und.reshape(-1, 2)
        return out

    def undistort_px(self, px: np.ndarray) -> np.ndarray:
        """Distorted pixels -> undistorted pixel coordinates (P = K)."""
        n = self.undistort(px)
        out = np.full_like(n, np.nan)
        ok = np.isfinite(n).all(axis=1)
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        out[ok, 0] = n[ok, 0] * fx + cx
        out[ok, 1] = n[ok, 1] * fy + cy
        return out


@dataclass
class Rig:
    cameras: dict[str, Camera]
    board: BoardSpec

    def save(self, path: str) -> None:
        doc = {
            "format": RIG_FORMAT,
            "board": self.board.to_dict(),
            "cameras": {
                c.name: {
                    "image_size": [int(v) for v in c.image_size],
                    "K": [[float(v) for v in row] for row in c.K],
                    "dist": [float(v) for v in np.ravel(c.dist)],
                    "R": [[float(v) for v in row] for row in c.R],
                    "t": [float(v) for v in c.t],
                    "meta": dict(c.meta),
                }
                for c in self.cameras.values()
            },
        }
        with open(path, "w") as f:
            yaml.safe_dump(doc, f, sort_keys=False)

    @classmethod
    def load(cls, path: str) -> "Rig":
        with open(path) as f:
            doc = yaml.safe_load(f)
        if doc.get("format") != RIG_FORMAT:
            raise ValueError(f"rig format {doc.get('format')!r} != {RIG_FORMAT}")
        cams = {
            name: Camera(
                name=name,
                image_size=tuple(c["image_size"]),
                K=np.asarray(c["K"], float),
                dist=np.asarray(c["dist"], float),
                R=np.asarray(c["R"], float),
                t=np.asarray(c["t"], float),
                meta=dict(c.get("meta", {})),
            )
            for name, c in doc["cameras"].items()
        }
        return cls(cameras=cams, board=BoardSpec.from_dict(doc["board"]))


def fundamental_matrix(cam_a: Camera, cam_b: Camera) -> np.ndarray:
    """F mapping undistorted pixel coords of ``cam_a`` to epilines in ``cam_b``."""
    R_ab = cam_b.R @ cam_a.R.T
    t_ab = cam_b.t - R_ab @ cam_a.t
    tx = np.array([
        [0, -t_ab[2], t_ab[1]],
        [t_ab[2], 0, -t_ab[0]],
        [-t_ab[1], t_ab[0], 0],
    ])
    E = tx @ R_ab
    return np.linalg.inv(cam_b.K).T @ E @ np.linalg.inv(cam_a.K)


def sampson_distance(F: np.ndarray, px_a: np.ndarray, px_b: np.ndarray) -> np.ndarray:
    """First-order geometric epipolar distance (px) for point pairs (N, 2)."""
    xa = np.column_stack([px_a, np.ones(len(px_a))])
    xb = np.column_stack([px_b, np.ones(len(px_b))])
    Fxa = xa @ F.T
    Ftxb = xb @ F
    num = np.einsum("ni,ni->n", xb, Fxa)
    den = Fxa[:, 0] ** 2 + Fxa[:, 1] ** 2 + Ftxb[:, 0] ** 2 + Ftxb[:, 1] ** 2
    return np.abs(num) / np.sqrt(np.maximum(den, 1e-12))


def triangulate(obs: dict[str, np.ndarray], cameras: dict[str, Camera],
                refine_iters: int = 2) -> tuple[np.ndarray, float]:
    """Triangulate one point from ``{cam_name: (2,) distorted px}``.

    Linear DLT on normalized rays, then a few Gauss-Newton steps on the
    normalized reprojection residual. Returns ``(xyz_world, rms_px)`` where
    the rms is the reprojection error converted to pixels via each camera's
    focal length; ``(nan, nan)`` with fewer than two finite views.
    """
    names = [n for n, p in obs.items() if np.isfinite(p).all()]
    if len(names) < 2:
        return np.full(3, np.nan), float("nan")

    rows, rhs, cams_n = [], [], []
    for n in names:
        cam = cameras[n]
        xy = cam.undistort(obs[n])[0]
        R, t = cam.R, cam.t
        rows.append(xy[0] * R[2] - R[0])
        rhs.append(t[0] - xy[0] * t[2])
        rows.append(xy[1] * R[2] - R[1])
        rhs.append(t[1] - xy[1] * t[2])
        cams_n.append((cam, xy))
    A = np.asarray(rows)
    b = np.asarray(rhs)
    X, *_ = np.linalg.lstsq(A, b, rcond=None)

    for _ in range(refine_iters):
        J, r = [], []
        for cam, xy in cams_n:
            pc = cam.R @ X + cam.t
            if pc[2] <= 1e-6:
                return np.full(3, np.nan), float("nan")
            u, v = pc[0] / pc[2], pc[1] / pc[2]
            r.extend([u - xy[0], v - xy[1]])
            J.append((cam.R[0] - u * cam.R[2]) / pc[2])
            J.append((cam.R[1] - v * cam.R[2]) / pc[2])
        J = np.asarray(J)
        r = np.asarray(r)
        try:
            dX = np.linalg.lstsq(J, -r, rcond=None)[0]
        except np.linalg.LinAlgError:
            break
        X = X + dX

    sq_px = 0.0
    for cam, xy in cams_n:
        pc = cam.R @ X + cam.t
        f = 0.5 * (cam.K[0, 0] + cam.K[1, 1])
        sq_px += ((pc[0] / pc[2] - xy[0]) ** 2 + (pc[1] / pc[2] - xy[1]) ** 2) * f ** 2
    return X, float(np.sqrt(sq_px / (2 * len(cams_n))))
