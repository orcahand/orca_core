#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Build a solver session from a raw capture: detect, track, triangulate, assign.

Capture directory layout (written by record_session.py, or synthetically by
render_synthetic.py):

    capture.yaml            # format, joints, sweep_frames, anchor_frames, cameras
    encoders.npy            # (F, J) measured joint angles, deg
    frames/<cam>/frame_%05d.png
    q_true.npy              # optional, synthetic captures only

Output is a dataset.py session directory holding the dot observations; the
direction / frame-anchor / plate observations are Phase-2 detection work and
are written empty (a dots-only session still yields curves and the axis
diagnostic; absolute offsets need the anchors).

Usage:
    uv run python tools/abs_calibration/build_session.py CAPTURE_DIR \
        --rig rig.yaml --out SESSION_DIR [--dark-dots] [--side right]
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np
import yaml

from assign import assign_links, link_signatures, STATIC_LINK
from cameras import Rig
from dataset import Dataset, save_session
from dots import (DetectorConfig, associate_tracks, detect_dots,
                  rigidity_filter, track_2d, triangulate_components)
from model import KinematicModel

CAPTURE_FORMAT = 1


def load_capture(capture_dir: str) -> dict:
    with open(os.path.join(capture_dir, "capture.yaml")) as f:
        cap = yaml.safe_load(f)
    if cap.get("format") != CAPTURE_FORMAT:
        raise ValueError(f"capture format {cap.get('format')!r} != {CAPTURE_FORMAT}")
    cap["encoders"] = np.load(os.path.join(capture_dir, "encoders.npy"))
    q_true = os.path.join(capture_dir, "q_true.npy")
    cap["q_true"] = np.load(q_true) if os.path.exists(q_true) else None
    return cap


def detect_all(capture_dir: str, cameras: list[str], n_frames: int,
               cfg: DetectorConfig) -> dict[str, list[np.ndarray]]:
    """Per-camera, per-frame dot centroids; a missing image detects nothing."""
    detections = {}
    for cam in cameras:
        cam_dir = os.path.join(capture_dir, "frames", cam)
        per_frame = []
        n_imgs = 0
        for f in range(n_frames):
            img = cv2.imread(os.path.join(cam_dir, f"frame_{f:05d}.png"),
                             cv2.IMREAD_GRAYSCALE)
            if img is None:
                per_frame.append(np.empty((0, 2)))
                continue
            per_frame.append(detect_dots(img, cfg))
            n_imgs += 1
        detections[cam] = per_frame
        n_dots = sum(len(d) for d in per_frame)
        print(f"  {cam}: {n_imgs}/{n_frames} frames, "
              f"{n_dots / max(n_imgs, 1):.1f} dots/frame")
    return detections


def build_session(capture_dir: str, rig: Rig, out_dir: str, *,
                  detector: DetectorConfig | None = None, side: str = "right",
                  max_step_px: float = 12.0, gate_px: float = 3.0,
                  reproj_gate_px: float = 3.0) -> Dataset:
    cap = load_capture(capture_dir)
    m = cap["encoders"]
    n_frames = len(m)
    joints = list(cap["joints"])
    sweep_frames = {j: np.asarray(v, int) for j, v in cap["sweep_frames"].items()}
    cameras = [c for c in cap["cameras"] if c in rig.cameras]
    missing = set(cap["cameras"]) - set(cameras)
    if missing:
        print(f"WARNING: capture cameras not in rig, skipped: {sorted(missing)}")

    print(f"Detecting dots ({len(cameras)} cameras, {n_frames} frames)...")
    detections = detect_all(capture_dir, cameras, n_frames, detector or DetectorConfig())

    print("Tracking...")
    tracks = {cam: track_2d(dets, max_step_px=max_step_px)
              for cam, dets in detections.items()}
    for cam, trs in tracks.items():
        print(f"  {cam}: {len(trs)} tracks")

    print("Associating across cameras...")
    components = associate_tracks(tracks, rig, gate_px=gate_px)
    pts, tri_stats = triangulate_components(
        components, tracks, rig, n_frames, reproj_gate_px=reproj_gate_px)
    print(f"  {tri_stats['n_components']} dot tracks, "
          f"{tri_stats['n_points']} 3D points "
          f"(mean reproj {tri_stats['mean_reproj_px']:.2f} px, "
          f"{tri_stats['n_rejected']} rejected)")

    print("Assigning tracks to links...")
    model = KinematicModel.load(side)
    assignment, report = assign_links(pts, sweep_frames, model, m, joints)
    unassigned = [r for r in report if r["link"] is None]
    for r in unassigned:
        print(f"  track {r['track']}: UNASSIGNED ({r['reason']})")
    dot_obs = {link: pts[:, idxs, :].copy() for link, idxs in assignment.items()}
    n_rigid = rigidity_filter(dot_obs, sweep_frames, link_signatures(model))
    if n_rigid:
        print(f"  rigidity filter dropped {n_rigid} points")
    for link in sorted(dot_obs):
        n_obs = int(np.isfinite(dot_obs[link][:, :, 0]).sum())
        tag = " (static: forearm/background)" if link == STATIC_LINK else ""
        print(f"  {link}: {dot_obs[link].shape[1]} dots, {n_obs} observations{tag}")

    ds = Dataset(
        m=m, dot_obs=dot_obs, dir_obs={}, frame_axes={},
        dir_frames=np.asarray(cap.get("anchor_frames", []), int),
        plate_obs=[], sweep_frames=sweep_frames, joints=joints,
        q_true=cap["q_true"],
        meta={
            **dict(cap.get("meta", {})),
            "producer": "build_session",
            "capture_dir": os.path.abspath(capture_dir),
            "side": side,
            "triangulation": tri_stats,
            "n_tracks_2d": {cam: int(len(trs)) for cam, trs in tracks.items()},
            "n_unassigned": len(unassigned),
        },
    )
    save_session(ds, out_dir)
    print(f"wrote session to {out_dir}")
    return ds


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("capture_dir")
    ap.add_argument("--rig", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--side", default="right", choices=("right", "left"))
    ap.add_argument("--dark-dots", action="store_true",
                    help="Dots darker than the surface (default: bright dots)")
    ap.add_argument("--max-step-px", type=float, default=12.0)
    ap.add_argument("--gate-px", type=float, default=3.0)
    args = ap.parse_args()

    detector = DetectorConfig(blob_color=0 if args.dark_dots else 255)
    build_session(args.capture_dir, Rig.load(args.rig), args.out,
                  detector=detector, side=args.side,
                  max_step_px=args.max_step_px, gate_px=args.gate_px)
    return 0


if __name__ == "__main__":
    sys.exit(main())
