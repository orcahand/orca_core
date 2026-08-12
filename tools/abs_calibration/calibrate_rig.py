#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Calibrate the camera rig from ChArUco images: intrinsics + extrinsics.

Expects one directory per camera:

    CALIB_DIR/<cam>/intrinsic/*.png   # 10+ varied views of the board
    CALIB_DIR/<cam>/extrinsic/*.png   # stills of the board FIXED in its
                                      # session pose — this defines the world
                                      # frame all cameras share

The board must not move between any extrinsic still of any camera; the rig
must not move between calibration and the session. If the printed board's
scale was caliper-verified, record it in a board YAML (``measured_square_m``).

Usage:
    uv run python tools/abs_calibration/calibrate_rig.py CALIB_DIR \
        --out rig.yaml [--board board.yaml] [--min-corners 8]
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import cv2
import yaml

from board import BoardSpec, average_poses, board_pose, calibrate_intrinsics
from cameras import Camera, Rig

EXTRINSIC_ROT_SPREAD_WARN_DEG = 0.2
EXTRINSIC_T_SPREAD_WARN_MM = 1.0


def _load_gray(paths: list[str]):
    for p in paths:
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"  WARNING: unreadable image {p}")
            continue
        yield p, img


def calibrate_camera_dir(cam_dir: str, spec: BoardSpec, min_corners: int) -> Camera:
    name = os.path.basename(cam_dir.rstrip("/"))
    intr_paths = sorted(glob.glob(os.path.join(cam_dir, "intrinsic", "*")))
    extr_paths = sorted(glob.glob(os.path.join(cam_dir, "extrinsic", "*")))
    if not intr_paths or not extr_paths:
        raise SystemExit(f"{name}: need intrinsic/ and extrinsic/ image dirs")

    intr = calibrate_intrinsics(
        [im for _, im in _load_gray(intr_paths)], spec, min_corners)
    print(f"{name}: intrinsics rms {intr['rms_px']:.3f} px "
          f"({intr['n_views']}/{len(intr_paths)} views)")

    poses = []
    for p, im in _load_gray(extr_paths):
        got = board_pose(im, spec, intr["K"], intr["dist"], min_corners)
        if got is None:
            print(f"  WARNING: no board pose in {p}")
            continue
        R, t, n, rms = got
        poses.append((R, t))
        print(f"  extrinsic {os.path.basename(p)}: {n} corners, rms {rms:.3f} px")
    if not poses:
        raise SystemExit(f"{name}: board not found in any extrinsic still")
    R, t, spread = average_poses(poses)
    if (spread["rot_spread_deg"] > EXTRINSIC_ROT_SPREAD_WARN_DEG
            or spread["t_spread_mm"] > EXTRINSIC_T_SPREAD_WARN_MM):
        print(f"  WARNING: extrinsic spread {spread['rot_spread_deg']:.3f} deg / "
              f"{spread['t_spread_mm']:.2f} mm — did the board or camera move?")

    return Camera(
        name=name, image_size=intr["image_size"], K=intr["K"],
        dist=intr["dist"], R=R, t=t,
        meta={"intrinsic_rms_px": round(intr["rms_px"], 4),
              "intrinsic_views": intr["n_views"], "extrinsic": spread},
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("calib_dir")
    ap.add_argument("--out", required=True, help="Output rig YAML")
    ap.add_argument("--board", default=None, help="Board spec YAML (defaults used otherwise)")
    ap.add_argument("--min-corners", type=int, default=8)
    args = ap.parse_args()

    spec = BoardSpec()
    if args.board:
        with open(args.board) as f:
            spec = BoardSpec.from_dict(yaml.safe_load(f))
    if spec.measured_square_m is None:
        print("NOTE: board scale not caliper-verified (measured_square_m unset); "
              "print scale error enters every length directly.")

    cam_dirs = sorted(
        d for d in glob.glob(os.path.join(args.calib_dir, "*")) if os.path.isdir(d))
    if not cam_dirs:
        raise SystemExit(f"no camera directories in {args.calib_dir}")

    cams = {}
    for d in cam_dirs:
        cam = calibrate_camera_dir(d, spec, args.min_corners)
        cams[cam.name] = cam

    rig = Rig(cameras=cams, board=spec)
    rig.save(args.out)
    print(f"wrote {args.out} ({len(cams)} cameras)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
