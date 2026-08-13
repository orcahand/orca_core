#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Merge calibration sessions into one solvable dataset.

Camera sessions (dots + anchors), contact sessions (abd-block / tip-press)
and plate sessions record different observation classes of the same hand;
the estimator solves them jointly when they share one dataset. Merging is
frame concatenation with the invariants kept explicit:

- Dot columns stay per-session (block-diagonal with NaN elsewhere): dots
  are reapplied between sessions, so each session's columns carry their own
  nuisance layout — VarPro handles that per column with no extra cost.
- Direction/frame-anchor rows concatenate along the anchor-frame axis, with
  NaN rows where a session lacks a link (the estimator's masks absorb them).
- Sweeps of the same joint concatenate: the axis diagnostic fits one axis
  over all of them, which is exactly right for one physical joint.
- ONE base pose serves the merged dataset, so every session carrying
  world-frame observations (dots, anchors, plate) must have been captured
  with the hand mounted rigidly in the SAME rig pose. Contact sessions are
  base-invariant and always safe to merge. Merging more than one
  world-frame session therefore requires the explicit same-mount flag.

Usage:
    uv run python tools/abs_calibration/merge_sessions.py OUT_DIR \
        SESSION_DIR [SESSION_DIR ...] [--same-mount]
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import replace

import numpy as np

from dataset import Dataset, load_session, save_session


def _has_world_frame_obs(ds: Dataset) -> bool:
    return bool(ds.dot_obs or ds.dir_obs or ds.frame_axes or ds.plate_obs)


def merge_datasets(sessions: list[Dataset], *,
                   same_mount: bool = False) -> Dataset:
    if not sessions:
        raise ValueError("nothing to merge")
    joints = sessions[0].joints
    for ds in sessions[1:]:
        if list(ds.joints) != list(joints):
            raise ValueError(
                f"joint lists differ: {joints} vs {ds.joints} — sessions of "
                "different hands or configs cannot merge")
    n_world = sum(_has_world_frame_obs(ds) for ds in sessions)
    if n_world > 1 and not same_mount:
        raise ValueError(
            f"{n_world} sessions carry world-frame observations (dots/"
            "anchors/plate) but share one base pose — merge them only if "
            "the hand stayed rigidly mounted and the rig did not move "
            "between them (pass same_mount=True / --same-mount)")

    offsets = np.cumsum([0] + [len(ds.m) for ds in sessions])
    F = int(offsets[-1])
    m = np.vstack([ds.m for ds in sessions])

    dot_obs: dict[str, np.ndarray] = {}
    for si, ds in enumerate(sessions):
        for link, arr in ds.dot_obs.items():
            block = np.full((F, arr.shape[1], 3), np.nan)
            block[offsets[si]:offsets[si + 1]] = arr
            if link in dot_obs:
                dot_obs[link] = np.concatenate([dot_obs[link], block], axis=1)
            else:
                dot_obs[link] = block

    dir_frames = np.concatenate(
        [np.asarray(ds.dir_frames, int) + offsets[si]
         for si, ds in enumerate(sessions)]) if any(
        len(ds.dir_frames) for ds in sessions) else np.array([], int)
    a_off = np.cumsum([0] + [len(ds.dir_frames) for ds in sessions])
    A = int(a_off[-1])

    dir_obs = {}
    dir_sigma = {}
    frame_axes = {}
    frame_sigma = {}
    for si, ds in enumerate(sessions):
        lo, hi = a_off[si], a_off[si + 1]
        for link, arr in ds.dir_obs.items():
            dir_obs.setdefault(link, np.full((A, 3), np.nan))[lo:hi] = arr
        for link, arr in ds.dir_sigma.items():
            dir_sigma.setdefault(link, np.full(A, np.nan))[lo:hi] = arr
        for link, arr in ds.frame_axes.items():
            frame_axes.setdefault(
                link, np.full((2, A, 3), np.nan))[:, lo:hi] = arr
        for link, arr in ds.frame_axes_sigma.items():
            frame_sigma.setdefault(
                link, np.full((2, A), np.nan))[:, lo:hi] = arr

    sweep_frames: dict[str, np.ndarray] = {}
    for si, ds in enumerate(sessions):
        for j, v in ds.sweep_frames.items():
            v = np.asarray(v, int) + offsets[si]
            sweep_frames[j] = (np.concatenate([sweep_frames[j], v])
                               if j in sweep_frames else v)

    plate_obs = [(int(fr) + offsets[si], fi, xyz)
                 for si, ds in enumerate(sessions)
                 for fr, fi, xyz in ds.plate_obs]
    contact_obs = [replace(e, frame=e.frame + int(offsets[si]))
                   for si, ds in enumerate(sessions)
                   for e in ds.contact_obs]

    q_true = (np.vstack([ds.q_true for ds in sessions])
              if all(ds.q_true is not None for ds in sessions) else None)

    return Dataset(
        m=m, dot_obs=dot_obs, dir_obs=dir_obs, frame_axes=frame_axes,
        dir_frames=dir_frames, plate_obs=plate_obs,
        sweep_frames=sweep_frames, joints=list(joints),
        contact_obs=contact_obs, dir_sigma=dir_sigma,
        frame_axes_sigma=frame_sigma, q_true=q_true,
        meta={
            "producer": "merge_sessions",
            "n_frames_per_session": [int(len(ds.m)) for ds in sessions],
            "sources": [dict(ds.meta) for ds in sessions],
        },
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out_dir")
    ap.add_argument("session_dirs", nargs="+")
    ap.add_argument("--same-mount", action="store_true",
                    help="Assert the hand stayed rigidly mounted (and the "
                         "rig fixed) across every world-frame session")
    args = ap.parse_args()

    sessions = [load_session(d) for d in args.session_dirs]
    try:
        merged = merge_datasets(sessions, same_mount=args.same_mount)
    except ValueError as e:
        sys.exit(str(e))
    save_session(merged, args.out_dir)
    classes = (f"dots {sum(a.shape[1] for a in merged.dot_obs.values())} cols, "
               f"dirs {len(merged.dir_obs)} links, "
               f"frame anchors {len(merged.frame_axes)}, "
               f"plate {len(merged.plate_obs)}, "
               f"contacts {len(merged.contact_obs)}")
    print(f"merged {len(sessions)} sessions -> {args.out_dir} "
          f"({len(merged.m)} frames; {classes})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
