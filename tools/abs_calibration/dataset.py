# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Observation dataset for the absolute calibration: in-memory type + on-disk
session format.

This is the seam between data producers and the solver: the synthetic
simulation writes it today, the camera/detection layer writes it at Phase 1,
and the estimator only ever reads it. Dot observations are (F, K, 3) world
points with NaN rows where a dot was not observed in a frame — occlusion is
the norm on a real rig, so masking is part of the format, not an afterthought.

Session directory layout (format 1):

    session.yaml        # format, side, joints, frame groups, provenance
    encoders.npy        # (F, J) measured encoder angle, deg, joint order = joints
    dots_<link>.npy     # (F, K, 3) world meters, NaN = unobserved
    dirs_<link>.npy     # (A, 3) unit link-axis directions at the anchor frames
    frame_axes_<link>.npy  # (2, A, 3) two-axis (z, x) mesh-referenced frame
                           # anchors — the forearm (mandatory for the wrist)
                           # and the palm dorsal shell
    plate.yaml          # [{frame, finger, xyz}] plate-contact observations
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

import numpy as np
import yaml

FORMAT_VERSION = 1


@dataclass
class Dataset:
    m: np.ndarray                        # (F, J) encoder readings, deg
    dot_obs: dict[str, np.ndarray]       # {link: (F, K, 3) world, NaN = unseen}
    dir_obs: dict[str, np.ndarray]       # {link: (A, 3)} unit dirs (NaN rows ok)
    frame_axes: dict[str, np.ndarray]    # {link: (2, A, 3)} z/x frame anchors
    dir_frames: np.ndarray               # (A,) frame indices of dir/frame obs
    plate_obs: list                      # [(frame_idx, finger, (3,) world)]
    sweep_frames: dict[str, np.ndarray]  # {joint: frame indices}
    joints: list
    q_true: np.ndarray | None = None     # (F, J) simulation truth, absent on hardware
    meta: dict = field(default_factory=dict)


def save_session(ds: Dataset, session_dir: str) -> None:
    os.makedirs(session_dir, exist_ok=True)
    manifest = {
        "format": FORMAT_VERSION,
        "joints": list(ds.joints),
        "links": sorted(ds.dot_obs),
        "dir_links": sorted(ds.dir_obs),
        "frame_axis_links": sorted(ds.frame_axes),
        "dir_frames": [int(i) for i in ds.dir_frames],
        "sweep_frames": {j: [int(i) for i in v] for j, v in ds.sweep_frames.items()},
        "meta": dict(ds.meta),
    }
    with open(os.path.join(session_dir, "session.yaml"), "w") as f:
        yaml.safe_dump(manifest, f, sort_keys=False)
    np.save(os.path.join(session_dir, "encoders.npy"), ds.m)
    for link, arr in ds.dot_obs.items():
        np.save(os.path.join(session_dir, f"dots_{link}.npy"), arr)
    for link, arr in ds.dir_obs.items():
        np.save(os.path.join(session_dir, f"dirs_{link}.npy"), arr)
    for link, arr in ds.frame_axes.items():
        np.save(os.path.join(session_dir, f"frame_axes_{link}.npy"), arr)
    with open(os.path.join(session_dir, "plate.yaml"), "w") as f:
        yaml.safe_dump(
            [{"frame": int(fr), "finger": fi, "xyz": [float(v) for v in xyz]}
             for fr, fi, xyz in ds.plate_obs],
            f, sort_keys=False,
        )
    if ds.q_true is not None:
        np.save(os.path.join(session_dir, "q_true.npy"), ds.q_true)


def load_session(session_dir: str) -> Dataset:
    with open(os.path.join(session_dir, "session.yaml")) as f:
        manifest = yaml.safe_load(f)
    if manifest.get("format") != FORMAT_VERSION:
        raise ValueError(
            f"session format {manifest.get('format')!r} != supported {FORMAT_VERSION}"
        )
    with open(os.path.join(session_dir, "plate.yaml")) as f:
        plate_raw = yaml.safe_load(f) or []
    q_true_path = os.path.join(session_dir, "q_true.npy")
    return Dataset(
        m=np.load(os.path.join(session_dir, "encoders.npy")),
        dot_obs={l: np.load(os.path.join(session_dir, f"dots_{l}.npy"))
                 for l in manifest["links"]},
        dir_obs={l: np.load(os.path.join(session_dir, f"dirs_{l}.npy"))
                 for l in manifest["dir_links"]},
        frame_axes={l: np.load(os.path.join(session_dir, f"frame_axes_{l}.npy"))
                    for l in manifest["frame_axis_links"]},
        dir_frames=np.asarray(manifest["dir_frames"], int),
        plate_obs=[(e["frame"], e["finger"], np.asarray(e["xyz"], float))
                   for e in plate_raw],
        sweep_frames={j: np.asarray(v, int)
                      for j, v in manifest["sweep_frames"].items()},
        joints=list(manifest["joints"]),
        q_true=np.load(q_true_path) if os.path.exists(q_true_path) else None,
        meta=dict(manifest.get("meta", {})),
    )
