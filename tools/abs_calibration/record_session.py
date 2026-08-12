#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Capture a calibration session on hardware: sweeps, frames, encoders.

NOT YET RUN ON HARDWARE — written against the public hand API and the
capture format that build_session.py consumes; validate on a real rig
before trusting a session from it.

Static-step capture: command a pose, wait for it to settle, read the
encoders, then grab one frame from every camera. Consumer webcams have no
hardware trigger and free-running rolling shutters — capturing at rest is
what makes their frames mutually consistent. The pose sequence is the
shared sweep protocol (sweep_plan.py): eased triangles from the held pose,
abduction at several flexion postures.

Requires a joint-feedback hand (the encoder loop supplies the measured
joint angles) whose rig was already calibrated (rig.yaml fixes each
camera's resolution). The board must stay where it was during extrinsic
calibration and the cameras must not move.

Usage:
    uv run python tools/abs_calibration/record_session.py CONFIG_PATH \
        --rig rig.yaml --out CAPTURE_DIR \
        --camera cam0=0 --camera cam1=1 [--camera cam2=2 ...] \
        [--settle 0.4] [--samples 5] [--abd-postures 0.15,0.5,0.8] [--dry-run]
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np
import yaml

from cameras import Rig
from sweep_plan import build_plan

CAPTURE_FORMAT = 1
ROM_MARGIN_DEG = 2.0


def open_cameras(specs: list[str], rig: Rig) -> dict[str, cv2.VideoCapture]:
    caps = {}
    for spec in specs:
        name, _, index = spec.partition("=")
        if name not in rig.cameras:
            raise SystemExit(f"camera {name!r} not in the rig")
        w, h = rig.cameras[name].image_size
        cap = cv2.VideoCapture(int(index))
        if not cap.isOpened():
            raise SystemExit(f"cannot open camera {name} (device {index})")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(10):  # warm up exposure and flush stale buffers
            cap.grab()
        ok, img = cap.read()
        if not ok or img.shape[1] != w or img.shape[0] != h:
            got = None if not ok else (img.shape[1], img.shape[0])
            raise SystemExit(
                f"camera {name}: resolution {got} != rig calibration {(w, h)} — "
                "recalibrate or fix the capture size")
        caps[name] = cap
    return caps


def grab_frame(cap: cv2.VideoCapture) -> np.ndarray | None:
    cap.grab()          # drop the possibly-stale buffered frame
    ok, img = cap.read()
    if not ok:
        return None
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img


def read_encoders(hand, joints: list[str], samples: int,
                  interval_s: float = 0.03) -> np.ndarray:
    rows = []
    for _ in range(samples):
        meas = hand.get_measured_joints()
        rows.append([meas.get(j, np.nan) for j in joints])
        time.sleep(interval_s)
    return np.nanmean(np.asarray(rows, float), axis=0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("config_path")
    ap.add_argument("--rig", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--camera", action="append", default=[],
                    metavar="NAME=INDEX", help="Rig camera to device index")
    ap.add_argument("--settle", type=float, default=0.4,
                    help="Seconds to settle before reading a pose")
    ap.add_argument("--samples", type=int, default=5,
                    help="Encoder samples averaged per pose")
    ap.add_argument("--step-scale", type=float, default=1.0)
    ap.add_argument("--abd-postures", default="0.15,0.5,0.8",
                    help="Flexion fractions for abd sweeps ('' = held only)")
    ap.add_argument("--anchor-poses", type=int, default=8)
    ap.add_argument("--dry-run", action="store_true",
                    help="Plan and report only; no hardware, no cameras")
    ap.add_argument("--yes", action="store_true", help="Skip the confirmation")
    args = ap.parse_args()

    from orca_core import load_hand

    hand = load_hand(args.config_path)
    if not hasattr(hand, "get_measured_joints"):
        raise SystemExit("this hand has no joint feedback; vision calibration "
                         "needs the encoder loop")

    fracs = tuple(float(v) for v in args.abd_postures.split(",") if v) or None

    if not args.dry_run:
        ok, msg = hand.connect()
        if not ok:
            raise SystemExit(f"connect failed: {msg}")
    try:
        joints = hand.loop_joint_names if not args.dry_run else None
        if joints is None:
            joints = list(hand.config.joint_roms_dict)
            if not args.dry_run:
                raise SystemExit("joint loop not running after connect()")
        roms = {j: tuple(hand.config.joint_roms_dict[j]) for j in joints}
        held = {
            j: float(np.clip(hand.config.neutral_position.get(j, 0.0),
                             roms[j][0] + ROM_MARGIN_DEG,
                             roms[j][1] - ROM_MARGIN_DEG))
            for j in joints
        }

        q, sweep_frames = build_plan(
            joints, roms, held, step_scale=args.step_scale,
            margin_deg=ROM_MARGIN_DEG, abd_flexion_fracs=fracs)
        rng = np.random.default_rng(0)
        anchors = np.array([
            [rng.uniform(lo + 0.3 * (hi - lo), lo + 0.7 * (hi - lo))
             for lo, hi in (roms[j] for j in joints)]
            for _ in range(args.anchor_poses)
        ])
        anchor_frames = list(range(len(q), len(q) + len(anchors)))
        q = np.vstack([q, anchors])

        per_pose = args.settle + args.samples * 0.03 + 0.2
        print(f"{len(q)} poses ({len(joints)} joints), "
              f"~{len(q) * per_pose / 60:.0f} min at {per_pose:.2f}s per pose")
        if args.dry_run:
            for j in joints:
                print(f"  {j}: {len(sweep_frames[j])} sweep frames")
            return 0
        if not args.yes and input("proceed? [y/N] ").strip().lower() != "y":
            return 1

        rig = Rig.load(args.rig)
        caps = open_cameras(args.camera, rig)
        if not caps:
            raise SystemExit("no cameras given (--camera NAME=INDEX)")
        for name in caps:
            os.makedirs(os.path.join(args.out, "frames", name), exist_ok=True)

        hand.init_joints()
        m = np.full((len(q), len(joints)), np.nan)
        t0 = time.monotonic()
        for f, row in enumerate(q):
            hand.set_joint_positions({j: float(v) for j, v in zip(joints, row)})
            time.sleep(args.settle)
            m[f] = read_encoders(hand, joints, args.samples)
            for name, cap in caps.items():
                img = grab_frame(cap)
                if img is None:
                    print(f"  WARNING: {name} dropped frame {f}")
                    continue
                cv2.imwrite(os.path.join(args.out, "frames", name,
                                         f"frame_{f:05d}.png"), img)
            if f % 25 == 0:
                el = time.monotonic() - t0
                eta = el / (f + 1) * (len(q) - f - 1)
                print(f"  {f}/{len(q)}  elapsed {el/60:.1f} min, eta {eta/60:.1f} min")

        np.save(os.path.join(args.out, "encoders.npy"), m)
        manifest = {
            "format": CAPTURE_FORMAT,
            "joints": list(joints),
            "sweep_frames": {j: [int(i) for i in v]
                             for j, v in sweep_frames.items()},
            "anchor_frames": [int(i) for i in anchor_frames],
            "cameras": sorted(caps),
            "meta": {
                "producer": "record_session",
                "config_path": os.path.abspath(args.config_path),
                "rig": os.path.abspath(args.rig),
                "settle_s": args.settle,
                "encoder_samples": args.samples,
                "abd_flexion_fracs": list(fracs) if fracs else [],
                "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            },
        }
        with open(os.path.join(args.out, "capture.yaml"), "w") as f:
            yaml.safe_dump(manifest, f, sort_keys=False)
        print(f"wrote capture to {args.out}")
        return 0
    finally:
        if not args.dry_run:
            try:
                hand.set_joint_positions(dict(hand.config.neutral_position))
                time.sleep(1.0)
            except Exception as e:
                print(f"WARNING: could not return to neutral: {e}")
            hand.disconnect()


if __name__ == "__main__":
    sys.exit(main())
