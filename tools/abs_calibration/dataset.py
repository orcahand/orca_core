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

Every observation class is optional: a camera session carries dots (and at
Phase 2 directions/frame anchors), a camera-free session carries contact
events only, a full session carries both. The estimator consumes whatever
is present and its per-joint sigma reports what the data determined.

Session directory layout (format 2; format-1 sessions load, they simply
predate contact observations):

    session.yaml        # format, side, joints, frame groups, provenance
    encoders.npy        # (F, J) measured encoder angle, deg, joint order = joints
    dots_<link>.npy     # (F, K, 3) world meters, NaN = unobserved
    dirs_<link>.npy     # (A, 3) unit link-axis directions at the anchor frames
    frame_axes_<link>.npy  # (2, A, 3) two-axis (z, x) mesh-referenced frame
                           # anchors — the forearm (mandatory for the wrist)
                           # and the palm dorsal shell
    plate.yaml          # [{frame, finger, xyz}] plate-contact observations
    contacts.yaml       # recorded contact events (abd-block, tip-press)
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

import numpy as np
import yaml

FORMAT_VERSION = 2
READABLE_FORMATS = (1, 2)  # v1 sessions simply predate contact observations


@dataclass
class ContactEvent:
    """One recorded contact between two hand bodies at frame ``frame``.

    ``abd_block``: adjacent-finger abduction drive until first contact —
    ``body_a``/``body_b`` are the two abd joint names; the constraint is the
    mesh contact manifold between the fingers at the recorded encoder row.
    ``tip_press``: fingertip pads pressed together (hand-guided) —
    ``body_a``/``body_b`` are finger names, ``p_a``/``p_b`` the tactile
    contact centroids in each pad's SENSOR frame (meters), ``force_*`` the
    resultant force at capture (drives the location sigma).
    """
    frame: int
    kind: str                            # "abd_block" | "tip_press"
    body_a: str
    body_b: str
    p_a: np.ndarray | None = None        # (3,) sensor frame of a
    p_b: np.ndarray | None = None        # (3,) sensor frame of b
    force_a: float | None = None         # N
    force_b: float | None = None
    push_current_ma: float | None = None  # abd_block: pusher current limit —
                                          # two-current events let a later
                                          # pass extrapolate skin compression
                                          # out of the stall depth


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
    contact_obs: list = field(default_factory=list)  # [ContactEvent]
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
    with open(os.path.join(session_dir, "contacts.yaml"), "w") as f:
        yaml.safe_dump([_contact_to_dict(e) for e in ds.contact_obs],
                       f, sort_keys=False)
    if ds.q_true is not None:
        np.save(os.path.join(session_dir, "q_true.npy"), ds.q_true)


def _contact_to_dict(e: ContactEvent) -> dict:
    d = {"frame": int(e.frame), "kind": e.kind,
         "body_a": e.body_a, "body_b": e.body_b}
    for name in ("p_a", "p_b"):
        v = getattr(e, name)
        if v is not None:
            d[name] = [float(x) for x in v]
    for name in ("force_a", "force_b", "push_current_ma"):
        v = getattr(e, name)
        if v is not None:
            d[name] = float(v)
    return d


def _contact_from_dict(d: dict) -> ContactEvent:
    return ContactEvent(
        frame=int(d["frame"]), kind=d["kind"],
        body_a=d["body_a"], body_b=d["body_b"],
        p_a=np.asarray(d["p_a"], float) if "p_a" in d else None,
        p_b=np.asarray(d["p_b"], float) if "p_b" in d else None,
        force_a=d.get("force_a"), force_b=d.get("force_b"),
        push_current_ma=d.get("push_current_ma"),
    )


def load_session(session_dir: str) -> Dataset:
    with open(os.path.join(session_dir, "session.yaml")) as f:
        manifest = yaml.safe_load(f)
    if manifest.get("format") not in READABLE_FORMATS:
        raise ValueError(
            f"session format {manifest.get('format')!r} not in {READABLE_FORMATS}"
        )
    with open(os.path.join(session_dir, "plate.yaml")) as f:
        plate_raw = yaml.safe_load(f) or []
    contacts_path = os.path.join(session_dir, "contacts.yaml")
    contacts_raw = []
    if os.path.exists(contacts_path):
        with open(contacts_path) as f:
            contacts_raw = yaml.safe_load(f) or []
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
        contact_obs=[_contact_from_dict(d) for d in contacts_raw],
        q_true=np.load(q_true_path) if os.path.exists(q_true_path) else None,
        meta=dict(manifest.get("meta", {})),
    )
