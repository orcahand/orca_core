# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""ChArUco board: spec, detection, intrinsic calibration, pose estimation.

The board defines the rig's world frame and its metric scale. A printed board
carries the printer's scale error (0.2-0.5% is common and ~0.5 mm over the
palm span), so the spec records a caliper-verified ``measured_square_m``;
when present it rescales the whole board (print scale error is uniform).
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class BoardSpec:
    squares_x: int = 7
    squares_y: int = 5
    square_m: float = 0.030              # designed square edge
    marker_m: float = 0.022              # designed ArUco marker edge
    dictionary: str = "DICT_5X5_100"
    measured_square_m: float | None = None  # caliper-verified printed edge

    @property
    def effective_square_m(self) -> float:
        return self.measured_square_m or self.square_m

    @property
    def effective_marker_m(self) -> float:
        return self.marker_m * self.effective_square_m / self.square_m

    def make(self) -> cv2.aruco.CharucoBoard:
        dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary))
        return cv2.aruco.CharucoBoard(
            (self.squares_x, self.squares_y),
            self.effective_square_m, self.effective_marker_m, dic,
        )

    def to_dict(self) -> dict:
        return {
            "squares_x": self.squares_x, "squares_y": self.squares_y,
            "square_m": self.square_m, "marker_m": self.marker_m,
            "dictionary": self.dictionary,
            "measured_square_m": self.measured_square_m,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "BoardSpec":
        return cls(**{k: d[k] for k in cls.__dataclass_fields__ if k in d})


def detect_board(gray: np.ndarray, spec: BoardSpec,
                 min_corners: int = 8) -> tuple[np.ndarray, np.ndarray] | None:
    """ChArUco corners in one image: ``(obj (N, 3), img (N, 2))`` or None."""
    board = spec.make()
    corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(gray)
    if corners is None or ids is None or len(corners) < min_corners:
        return None
    obj, img = board.matchImagePoints(corners, ids)
    return obj.reshape(-1, 3), img.reshape(-1, 2)


def calibrate_intrinsics(images: list[np.ndarray], spec: BoardSpec,
                         min_corners: int = 8) -> dict:
    """Per-camera intrinsics from ChArUco views of one camera.

    Returns ``{K, dist, image_size, rms_px, n_views}``; raises if fewer than
    four usable views (calibration would be ill-conditioned, not just noisy).
    """
    obj_all, img_all = [], []
    size = None
    for im in images:
        gray = im if im.ndim == 2 else cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        det = detect_board(gray, spec, min_corners)
        if det is None:
            continue
        obj_all.append(det[0].astype(np.float32).reshape(-1, 1, 3))
        img_all.append(det[1].astype(np.float32).reshape(-1, 1, 2))
    if len(obj_all) < 4:
        raise ValueError(
            f"only {len(obj_all)} usable ChArUco views (need >= 4); "
            "check board spec, lighting, and focus"
        )
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_all, img_all, size, None, None)
    return {
        "K": np.asarray(K), "dist": np.ravel(dist), "image_size": size,
        "rms_px": float(rms), "n_views": len(obj_all),
    }


def board_pose(gray: np.ndarray, spec: BoardSpec, K: np.ndarray,
               dist: np.ndarray, min_corners: int = 8):
    """Board (= world) pose in the camera: ``(R, t, n_corners, rms_px)`` or None."""
    det = detect_board(gray, spec, min_corners)
    if det is None:
        return None
    obj, img = det
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist)
    if not ok:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.ravel(), len(obj), rms


def average_poses(poses: list[tuple[np.ndarray, np.ndarray]]):
    """Average of (R, t) samples of one static pose, with spread diagnostics.

    Rotation: eigenvector quaternion mean; translation: component median.
    Returns ``(R, t, {"rot_spread_deg", "t_spread_mm", "n_views"})``.
    """
    quats = []
    for R, _ in poses:
        w = np.sqrt(max(1.0 + np.trace(R), 1e-12)) / 2.0
        q = np.array([w, (R[2, 1] - R[1, 2]) / (4 * w),
                      (R[0, 2] - R[2, 0]) / (4 * w),
                      (R[1, 0] - R[0, 1]) / (4 * w)])
        quats.append(q / np.linalg.norm(q))
    Q = np.asarray(quats)
    Q[Q @ Q[0] < 0] *= -1.0
    _, vecs = np.linalg.eigh(Q.T @ Q)
    q = vecs[:, -1]
    w, x, y, z = q / np.linalg.norm(q)
    R_mean = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    ts = np.asarray([t for _, t in poses])
    t_mean = np.median(ts, axis=0)

    rot_devs = [
        np.degrees(np.arccos(np.clip((np.trace(R_mean.T @ R) - 1) / 2, -1, 1)))
        for R, _ in poses
    ]
    stats = {
        "rot_spread_deg": float(np.max(rot_devs)),
        "t_spread_mm": float(np.max(np.linalg.norm(ts - t_mean, axis=1)) * 1000),
        "n_views": len(poses),
    }
    return R_mean, t_mean, stats
