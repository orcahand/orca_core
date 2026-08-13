#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Record plate contacts: fingertips seated in dimples at several wrist angles.

NOT YET RUN ON HARDWARE. The plate contact is the absolute anchor that
crosses the wrist (design doc §4c): the plate's pose in the rig's world
frame comes from its ArUco tags through the calibrated cameras, so every
dimple has known world coordinates, and a fingertip seated in one equates
the model's fingertip point with a known point — at several wrist angles,
which is what anchors the wrist against the forearm frame.

Protocol per contact: the wrist holds a commanded angle (torque on), the
prompted finger's motors go torque-off (limp), and the operator seats that
fingertip into the prompted dimple by hand; ENTER captures once the
encoders are still. No force is commanded anywhere — the dimple's conical
seat registers the tip, the encoders record the configuration.

The solver treats plate contacts as calibration AND validation — never the
same ones for both: record generously and split at solve time
(``solve_session.py --plate-holdout``).

Usage:
    uv run python tools/abs_calibration/record_plate.py CONFIG_PATH \
        --rig rig.yaml --plate plate.yaml --out SESSION_DIR \
        --camera cam0=0 [--camera cam1=1 ...] \
        [--wrist-angles -40,-15,10] [--fingers index,middle,ring,pinky,thumb] \
        [--dimples 0,0 2,1 4,3] [--dry-run]
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

from cameras import Rig
from dataset import Dataset, save_session
from plate import PlateSpec, dimple_world, plate_pose_world
from record_session import grab_frame, open_cameras, read_encoders

STILL_TOL_DEG = 0.15
FINGER_STAGES = {"thumb": ("cmc", "abd", "mcp", "dip"),
                 "index": ("abd", "mcp", "pip"), "middle": ("abd", "mcp", "pip"),
                 "ring": ("abd", "mcp", "pip"), "pinky": ("abd", "mcp", "pip")}


def finger_motor_ids(config, finger: str) -> list[int]:
    ids = []
    for stage in FINGER_STAGES[finger]:
        j = f"{finger}_{stage}"
        if j in config.joint_ids:
            ids.append(config.joint_ids[j])
    return ids


def capture_contact(hand, joints: list[str], samples: int = 5):
    """Encoder row once still; None if the pose keeps moving."""
    for _ in range(6):
        a = read_encoders(hand, joints, samples)
        time.sleep(0.15)
        b = read_encoders(hand, joints, 2)
        if np.nanmax(np.abs(a - b)) < STILL_TOL_DEG:
            return a
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("config_path")
    ap.add_argument("--rig", required=True)
    ap.add_argument("--plate", required=True, help="Plate spec YAML")
    ap.add_argument("--out", required=True)
    ap.add_argument("--camera", action="append", default=[],
                    metavar="NAME=INDEX")
    ap.add_argument("--wrist-angles", default="-40,-15,10",
                    help="Commanded wrist angles, deg (several: the plate "
                         "anchors the wrist ACROSS angles)")
    ap.add_argument("--fingers", default="index,middle,ring,pinky,thumb")
    ap.add_argument("--dimples", nargs="*", default=["0,0", "2,1", "4,3"],
                    metavar="I,J", help="Dimple grid coordinates to visit")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    spec = PlateSpec.load(args.plate)
    if spec.measured_pitch_m is None:
        print("NOTE: plate pitch not caliper-verified (measured_pitch_m "
              "unset); print scale error enters every contact directly.")
    wrist_angles = [float(v) for v in args.wrist_angles.split(",") if v]
    fingers = [f.strip() for f in args.fingers.split(",") if f.strip()]
    dimples = [tuple(int(v) for v in d.split(",")) for d in args.dimples]

    n = len(wrist_angles) * len(fingers) * len(dimples)
    print(f"{n} contacts planned: {len(wrist_angles)} wrist angles x "
          f"{len(fingers)} fingers x {len(dimples)} dimples")
    if args.dry_run:
        for i, j in dimples:
            from plate import dimple_plate
            print(f"  dimple ({i},{j}) at plate {dimple_plate(spec, i, j)}")
        return 0

    rig = Rig.load(args.rig)
    caps = open_cameras(args.camera, rig)
    if not caps:
        raise SystemExit("no cameras given (--camera NAME=INDEX); the plate "
                         "pose comes from them")
    images = {name: grab_frame(cap) for name, cap in caps.items()}
    got = plate_pose_world({k: v for k, v in images.items() if v is not None},
                           rig, spec)
    if got is None:
        raise SystemExit("plate tags not visible in any calibrated camera")
    R_p, t_p, stats = got
    print(f"plate pose: {stats['n_views']} camera(s), "
          f"spread {stats['rot_spread_deg']:.3f} deg / "
          f"{stats['t_spread_mm']:.2f} mm")
    if stats["rot_spread_deg"] > 0.5 or stats["t_spread_mm"] > 2.0:
        print("WARNING: plate pose spread is high — check tag flatness "
              "and camera calibration before trusting these contacts.")

    from orca_core import load_hand
    hand = load_hand(args.config_path)
    if not hasattr(hand, "get_measured_joints"):
        raise SystemExit("plate contacts need a joint-feedback hand")
    ok, msg = hand.connect()
    if not ok:
        raise SystemExit(f"connect failed: {msg}")
    rows, plate_obs = [], []
    try:
        joints = hand.loop_joint_names
        if joints is None:
            raise SystemExit("joint loop not running after connect()")
        hand.init_joints()
        neutral = {j: float(hand.config.neutral_position.get(j, 0.0))
                   for j in joints}
        for w in wrist_angles:
            hand.set_joint_positions({**neutral, "wrist": float(w)})
            time.sleep(1.0)
            for finger in fingers:
                ids = finger_motor_ids(hand.config, finger)
                hand.disable_torque(ids)
                print(f"\n== wrist {w:+.0f} deg, {finger} limp ==")
                for i, j in dimples:
                    prompt = (f"seat the {finger} tip in dimple ({i},{j}), "
                              "then ENTER (s=skip, q=quit): ")
                    ans = input(prompt).strip().lower()
                    if ans == "q":
                        raise KeyboardInterrupt
                    if ans == "s":
                        continue
                    m_row = capture_contact(hand, joints)
                    if m_row is None:
                        print("  not still; skipped")
                        continue
                    frame = len(rows)
                    rows.append(m_row)
                    plate_obs.append((frame, finger,
                                      dimple_world(R_p, t_p, spec, i, j)))
                    print(f"  captured contact {len(plate_obs)}")
                hand.enable_torque(ids)
                hand.set_joint_positions({**neutral, "wrist": float(w)})
                time.sleep(0.6)
    except KeyboardInterrupt:
        print("\nstopping; saving what was captured")
    finally:
        try:
            hand.enable_torque()
            hand.set_neutral_position()
            time.sleep(1.0)
        except Exception as e:
            print(f"WARNING: could not return to neutral: {e}")
        hand.disconnect()
        for cap in caps.values():
            cap.release()

    if not plate_obs:
        print("no contacts captured; nothing written")
        return 1
    ds = Dataset(
        m=np.asarray(rows).reshape(-1, len(joints)),
        dot_obs={}, dir_obs={}, frame_axes={},
        dir_frames=np.array([], int), plate_obs=plate_obs,
        sweep_frames={}, joints=list(joints),
        meta={
            "producer": "record_plate",
            "config_path": os.path.abspath(args.config_path),
            "plate_spec": spec.to_dict(),
            "plate_pose_spread": {k: v for k, v in stats.items()
                                  if k != "per_camera"},
            "wrist_angles": wrist_angles,
            "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        },
    )
    save_session(ds, args.out)
    print(f"wrote {len(plate_obs)} plate contacts to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
