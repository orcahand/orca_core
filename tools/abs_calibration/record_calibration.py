#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Interactively capture ChArUco calibration images for calibrate_rig.py.

NOT YET RUN ON HARDWARE. Two phases, writing the directory layout
calibrate_rig.py expects:

  intrinsic  hold the board in varied poses (tilted, rolled, covering the
             field); one Enter per shot, 12+ shots per camera recommended.
             Per-camera detected corner counts print as live feedback.
  extrinsic  fix the board in its session pose (it must not move again
             until the session is captured); a few Enters grab stills from
             every camera at once.

Use the SAME resolution here and in record_session.py — intrinsics are
resolution-specific.

Usage:
    uv run python tools/abs_calibration/record_calibration.py OUT_DIR \
        --camera cam0=0 --camera cam1=1 [--size 1280x960] [--board board.yaml]
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import yaml

from board import BoardSpec, detect_board


def open_cameras(specs: list[str], size: tuple[int, int]):
    caps = {}
    for spec in specs:
        name, _, index = spec.partition("=")
        cap = cv2.VideoCapture(int(index))
        if not cap.isOpened():
            raise SystemExit(f"cannot open camera {name} (device {index})")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(10):
            cap.grab()
        caps[name] = cap
    return caps


def grab_gray(cap):
    cap.grab()
    ok, img = cap.read()
    if not ok:
        return None
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img


def capture_round(caps, spec, out_dir, phase, shot):
    for name, cap in caps.items():
        img = grab_gray(cap)
        if img is None:
            print(f"  {name}: FRAME DROPPED")
            continue
        d = os.path.join(out_dir, name, phase)
        os.makedirs(d, exist_ok=True)
        cv2.imwrite(os.path.join(d, f"{phase}_{shot:03d}.png"), img)
        det = detect_board(img, spec, min_corners=4)
        n = 0 if det is None else len(det[0])
        print(f"  {name}: {n} corners")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out_dir")
    ap.add_argument("--camera", action="append", default=[],
                    metavar="NAME=INDEX", required=True)
    ap.add_argument("--assist-camera", default="", metavar="NAME=INDEX",
                    help="Hand-held assist camera: shoots the intrinsic "
                         "phase only (it has no fixed pose; each session "
                         "shot self-localises off the board)")
    ap.add_argument("--size", default="1280x960")
    ap.add_argument("--board", default=None)
    args = ap.parse_args()

    spec = BoardSpec()
    if args.board:
        with open(args.board) as f:
            spec = BoardSpec.from_dict(yaml.safe_load(f))
    if spec.measured_square_m is None:
        print("NOTE: board scale not caliper-verified (measured_square_m); "
              "measure a row of squares and record it in a board YAML.")

    w, h = (int(v) for v in args.size.split("x"))
    caps = open_cameras(args.camera, (w, h))
    assist_caps = (open_cameras([args.assist_camera], (w, h))
                   if args.assist_camera else {})

    print("\n== intrinsic phase: move the board between shots ==")
    print("Enter = shot, q+Enter = next phase")
    shot = 0
    while input(f"[intrinsic {shot}] > ").strip().lower() != "q":
        capture_round({**caps, **assist_caps}, spec, args.out_dir,
                      "intrinsic", shot)
        shot += 1
    if shot < 12:
        print(f"WARNING: only {shot} intrinsic shots; 12+ recommended")

    print("\n== extrinsic phase: FIX the board in its session pose ==")
    print("It must not move between these stills and the session.")
    print("Enter = still (3 recommended), q+Enter = done")
    shot = 0
    while input(f"[extrinsic {shot}] > ").strip().lower() != "q":
        capture_round(caps, spec, args.out_dir, "extrinsic", shot)
        shot += 1

    print(f"\nwrote {args.out_dir}; next:\n"
          f"  uv run python tools/abs_calibration/calibrate_rig.py "
          f"{args.out_dir} --out rig.yaml"
          + (f" --board {args.board}" if args.board else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
