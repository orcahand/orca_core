# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Dimple contact plate: spec, printable geometry, pose detection, dimple
coordinates.

The plate is the absolute anchor that crosses the wrist (design doc §4c
residual 4): a printed slab with a grid of conical dimples at a
caliper-verified pitch and two ArUco tags on its face. A fingertip seated in
a dimple is a repeatable physical point; the tags give the plate's pose in
the rig's world (board) frame through any calibrated camera, so every dimple
has known world coordinates.

Frames: plate origin at dimple (0, 0), x along columns, y along rows, z out
of the dimpled face. Tags sit left of the grid with their marker "top"
toward +y. Print the plate (``--scad``), glue printed paper tags into the
recesses, then caliper the dimple pitch across the grid and the tag edges
and record ``measured_pitch_m`` / ``measured_tag_m`` in the plate YAML —
print scale error enters every contact coordinate directly.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
import yaml

from board import average_poses


@dataclass
class PlateSpec:
    rows: int = 4
    cols: int = 5
    pitch_m: float = 0.016               # designed dimple pitch
    measured_pitch_m: float | None = None  # caliper-verified
    dimple_diam_m: float = 0.007
    dimple_depth_m: float = 0.003
    tag_ids: tuple = (96, 97, 98, 99)    # high ids: the board uses the low ones
    tag_m: float = 0.028                 # designed tag edge
    measured_tag_m: float | None = None
    dictionary: str = "DICT_5X5_100"     # same family as the board
    # Four tags flanking the grid: a WIDE layout is what pins the planar
    # pose — a narrow pair leaves millimetre-class depth/tilt ambiguity.
    tag_centers: tuple = ((-0.036, 0.004), (-0.036, 0.044),
                          (0.100, 0.004), (0.100, 0.044))

    @property
    def effective_pitch_m(self) -> float:
        return self.measured_pitch_m or self.pitch_m

    @property
    def effective_tag_m(self) -> float:
        return self.measured_tag_m or self.tag_m

    def to_dict(self) -> dict:
        return {
            "rows": self.rows, "cols": self.cols, "pitch_m": self.pitch_m,
            "measured_pitch_m": self.measured_pitch_m,
            "dimple_diam_m": self.dimple_diam_m,
            "dimple_depth_m": self.dimple_depth_m,
            "tag_ids": list(self.tag_ids), "tag_m": self.tag_m,
            "measured_tag_m": self.measured_tag_m,
            "dictionary": self.dictionary,
            "tag_centers": [list(c) for c in self.tag_centers],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "PlateSpec":
        kw = {k: d[k] for k in cls.__dataclass_fields__ if k in d}
        if "tag_ids" in kw:
            kw["tag_ids"] = tuple(kw["tag_ids"])
        if "tag_centers" in kw:
            kw["tag_centers"] = tuple(tuple(c) for c in kw["tag_centers"])
        return cls(**kw)

    @classmethod
    def load(cls, path: str) -> "PlateSpec":
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f))

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False)


def tag_corners_plate(spec: PlateSpec) -> dict[int, np.ndarray]:
    """ArUco corner coordinates in the plate frame, in OpenCV detection
    order (TL, TR, BR, BL of the marker as printed, top toward +y)."""
    s = spec.effective_tag_m / 2.0
    offsets = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
    return {
        tid: np.array([cx, cy, 0.0]) + offsets
        for tid, (cx, cy) in zip(spec.tag_ids, spec.tag_centers)
    }


def dimple_plate(spec: PlateSpec, i: int, j: int) -> np.ndarray:
    """Dimple (i=col, j=row) apex in the plate frame."""
    if not (0 <= i < spec.cols and 0 <= j < spec.rows):
        raise ValueError(f"dimple ({i},{j}) outside {spec.cols}x{spec.rows}")
    p = spec.effective_pitch_m
    return np.array([i * p, j * p, 0.0])


def detect_plate(gray: np.ndarray, spec: PlateSpec
                 ) -> tuple[np.ndarray, np.ndarray, int] | None:
    """Tag corners in one image: ``(obj (N,3) plate frame, img (N,2), n_tags)``."""
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.dictionary))
    corners, ids, _ = cv2.aruco.ArucoDetector(dic).detectMarkers(gray)
    if ids is None:
        return None
    known = tag_corners_plate(spec)
    obj, img = [], []
    n_tags = 0
    for c, tid in zip(corners, ids.ravel()):
        if int(tid) in known:
            obj.append(known[int(tid)])
            img.append(c.reshape(4, 2))
            n_tags += 1
    if not n_tags:
        return None
    return np.vstack(obj), np.vstack(img), n_tags


def plate_pose(gray: np.ndarray, spec: PlateSpec, K: np.ndarray,
               dist: np.ndarray):
    """Plate pose in the camera: ``(R, t, n_tags, rms_px)`` or None."""
    det = detect_plate(gray, spec)
    if det is None:
        return None
    obj, img, n_tags = det
    ok, rvec, tvec = cv2.solvePnP(obj, img.astype(np.float64), K, dist)
    if not ok:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(obj, img.astype(np.float64), K, dist,
                                      rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.ravel(), n_tags, rms


def plate_pose_world(images: dict[str, np.ndarray], rig, spec: PlateSpec):
    """Plate pose in the world (board) frame, averaged over every calibrated
    camera that sees it: ``(R, t, stats)`` or None. Cameras without a rig
    pose (assist) are skipped."""
    poses = []
    per_cam = {}
    for name, gray in images.items():
        cam = rig.cameras.get(name)
        if cam is None or cam.meta.get("assist"):
            continue
        got = plate_pose(gray, spec, cam.K, cam.dist)
        if got is None:
            per_cam[name] = "no tags"
            continue
        R_pc, t_pc, n_tags, rms = got
        R_w = cam.R.T @ R_pc
        t_w = cam.R.T @ (t_pc - cam.t)
        poses.append((R_w, t_w))
        per_cam[name] = {"n_tags": n_tags, "rms_px": round(rms, 3)}
    if not poses:
        return None
    R, t, stats = average_poses(poses)
    stats["per_camera"] = per_cam
    return R, t, stats


def dimple_world(R: np.ndarray, t: np.ndarray, spec: PlateSpec,
                 i: int, j: int) -> np.ndarray:
    return R @ dimple_plate(spec, i, j) + t


def write_scad(spec: PlateSpec, path: str, margin_m: float = 0.012,
               thickness_m: float = 0.006) -> None:
    """Parametric OpenSCAD source for the printable plate: slab, conical
    dimples, shallow recesses for the paper tags."""
    p = spec.pitch_m * 1000
    gw = (spec.cols - 1) * p
    gh = (spec.rows - 1) * p
    m = margin_m * 1000
    tag = spec.tag_m * 1000
    xs = [c[0] * 1000 for c in spec.tag_centers]
    ys = [c[1] * 1000 for c in spec.tag_centers]
    x0 = min(-m, min(xs) - tag / 2 - m / 2)
    x1 = max(gw + m, max(xs) + tag / 2 + m / 2)
    y0 = min(-m, min(ys) - tag / 2 - m / 2)
    y1 = max(gh + m, max(ys) + tag / 2 + m / 2)
    dd = spec.dimple_diam_m * 1000
    dh = spec.dimple_depth_m * 1000
    th = thickness_m * 1000
    # Dimple cutters: cones opening at the top face, apex dh below it.
    dimples = "\n".join(
        f"    translate([{i * p:.2f}, {j * p:.2f}, {th - dh:.2f}]) "
        f"cylinder(h={dh + 0.01:.2f}, d1=0.4, d2={dd:.2f}, $fn=48);"
        for i in range(spec.cols) for j in range(spec.rows))
    recesses = "\n".join(
        f"    translate([{cx * 1000 - tag / 2 - 1:.2f}, "
        f"{cy * 1000 - tag / 2 - 1:.2f}, {th - 0.6:.2f}]) "
        f"cube([{tag + 2:.2f}, {tag + 2:.2f}, 1]);"
        for cx, cy in spec.tag_centers)
    scad = f"""// ORCA calibration dimple plate — print flat, 100% scale.
// Glue printed {tag:.0f} mm ArUco tags (ids {list(spec.tag_ids)}) into the
// recesses, marker top toward +y. Caliper the dimple pitch across the grid
// and the tag edges; record measured_pitch_m / measured_tag_m in the YAML.
difference() {{
  translate([{x0:.2f}, {y0:.2f}, 0])
    cube([{x1 - x0:.2f}, {y1 - y0:.2f}, {th:.2f}]);
  union() {{
{dimples}
{recesses}
  }}
}}
"""
    with open(path, "w") as f:
        f.write(scad)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(
        description="Write the printable dimple plate and/or its spec YAML")
    ap.add_argument("--scad", default=None, help="OpenSCAD output path")
    ap.add_argument("--yaml", default=None, help="Plate spec YAML output path")
    args = ap.parse_args()
    spec = PlateSpec()
    if args.scad:
        write_scad(spec, args.scad)
        print(f"wrote {args.scad} — print at 100% scale, then caliper "
              "pitch and tag edges")
    if args.yaml:
        spec.save(args.yaml)
        print(f"wrote {args.yaml} — fill measured_pitch_m / measured_tag_m "
              "after calipering")
