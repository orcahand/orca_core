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


def write_printable_svg(spec: BoardSpec, path: str, margin_mm: float = 15.0,
                        px_per_square: int = 60) -> None:
    """Print-exact board as SVG: the raster is embedded with physical mm
    dimensions, so any 'actual size' print reproduces the metric scale.
    Verify with calipers anyway and record ``measured_square_m``."""
    import base64
    import cv2
    w_mm = spec.squares_x * spec.square_m * 1000
    h_mm = spec.squares_y * spec.square_m * 1000
    img = spec.make().generateImage(
        (spec.squares_x * px_per_square, spec.squares_y * px_per_square),
        marginSize=0)
    ok, png = cv2.imencode(".png", img)
    b64 = base64.b64encode(png.tobytes()).decode()
    total_w, total_h = w_mm + 2 * margin_mm, h_mm + 2 * margin_mm
    ruler_y = margin_mm + h_mm + 6
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg"
     width="{total_w}mm" height="{total_h}mm"
     viewBox="0 0 {total_w} {total_h}">
  <rect width="{total_w}" height="{total_h}" fill="white"/>
  <image x="{margin_mm}" y="{margin_mm}" width="{w_mm}" height="{h_mm}"
         preserveAspectRatio="none" image-rendering="pixelated"
         href="data:image/png;base64,{b64}"/>
  <line x1="{margin_mm}" y1="{ruler_y}" x2="{margin_mm + 100}" y2="{ruler_y}"
        stroke="black" stroke-width="0.3"/>
  <line x1="{margin_mm}" y1="{ruler_y - 2}" x2="{margin_mm}" y2="{ruler_y + 2}"
        stroke="black" stroke-width="0.3"/>
  <line x1="{margin_mm + 100}" y1="{ruler_y - 2}" x2="{margin_mm + 100}"
        y2="{ruler_y + 2}" stroke="black" stroke-width="0.3"/>
  <text x="{margin_mm + 50}" y="{ruler_y + 5}" font-size="3"
        text-anchor="middle" font-family="sans-serif">
    exactly 100 mm — measure after printing; squares {spec.square_m * 1000:.1f} mm
  </text>
</svg>
"""
    with open(path, "w") as f:
        f.write(svg)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(
        description="Write a print-exact ChArUco board SVG")
    ap.add_argument("--out", required=True)
    ap.add_argument("--square-mm", type=float, default=30.0)
    args = ap.parse_args()
    s = BoardSpec(square_m=args.square_mm / 1000,
                  marker_m=args.square_mm / 1000 * 22 / 30)
    write_printable_svg(s, args.out)
    print(f"wrote {args.out} — print at 100% scale (no fit-to-page), "
          "then caliper the 100 mm ruler and a row of squares")
