# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Synthetic data generation for the absolute-calibration study.

Generates the observation classes of the v1 pipeline (sweep dot tracks,
link-direction anchors, palm plane, plate contacts) from a perturbed
"as-printed" truth, with the systematic error classes injected explicitly:

- geometry: axis tilt, origin shift, per-link direction bias (print-vs-mesh),
  fingertip point bias, palm-plane warp bias
- measurement: global scale error and a smooth systematic 3D field
  (distortion/reconstruction bias proxy), plus iid noise
- encoder: offset + gain + first two INL harmonics, quantized

Observations live at the 3D level (triangulated dots), not 2D pixels: the
study targets bias propagation through the estimator, and the systematic
field + scale term stand in for what a camera layer contributes. See README.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from dataset import Dataset
from model import (
    FINGERS, HELD_POSE, JOINT_ROMS, TIP_POINT_LOCAL, DISTAL_LINK, PALM_LINK,
    EncoderTruth, KinematicModel,
)
from orca_core.kinematics.transforms import rotation_about_axis

# Links that carry dots (finger abd stubs and the thumb cmc stub are tiny and
# occluded on the real hand, so they get none; their joints are observed
# through the links distal of them).
DOT_LINKS = [
    "wrist",  # carpals/palm body
    "thumb_abd", "thumb_mcp", "thumb_dip",
    "index_mcp", "index_pip", "middle_mcp", "middle_pip",
    "ring_mcp", "ring_pip", "pinky_mcp", "pinky_pip",
]
DIR_LINKS = [l for l in DOT_LINKS if l != "wrist"]

# The forearm/base link. Its dots are static in the rig, so an unknown dot
# layout absorbs them completely — the ONLY thing that pins base pose against
# the wrist offset is a mesh-referenced frame anchor on this link (print bias
# applies, like every mesh-referenced anchor). Distal observations cannot
# split base from wrist: a wrist-offset error is a constant rotation about the
# world-fixed wrist axis, which base pose absorbs exactly.
TOWER_LINK = "fixed0"

# Links carrying a two-axis mesh-referenced frame anchor: the forearm
# (mandatory, splits base from wrist) and the palm's dorsal printed shell
# (the palmar face is silicone-skinned and curved — never referenced).
FRAME_ANCHOR_LINKS = (TOWER_LINK, PALM_LINK)

# Usable dot span along each link's local z, meters (rough link lengths).
LINK_SPAN = {
    "wrist": 0.055, "thumb_abd": 0.026, "thumb_mcp": 0.030, "thumb_dip": 0.028,
    "index_mcp": 0.035, "index_pip": 0.042, "middle_mcp": 0.038,
    "middle_pip": 0.042, "ring_mcp": 0.038, "ring_pip": 0.042,
    "pinky_mcp": 0.029, "pinky_pip": 0.036,
}

DOTS_PER_LINK = 5


@dataclass
class Scenario:
    name: str
    axis_tilt_deg: float = 0.0
    origin_shift_m: float = 0.0
    scale: float = 0.0
    field_amp_m: float = 0.0
    dir_bias_deg: float = 0.0
    tip_bias_m: float = 0.0
    inl_deg: float = 0.6
    gain_frac: float = 0.005
    dot_noise_m: float = 1.5e-4
    dir_noise_deg: float = 0.05
    hardstop_bias_deg: float = 1.0
    # Occlusion: probability a link is wholly unobserved in a frame, and an
    # additional per-dot dropout. Real rigs are occlusion-heavy; masking is
    # exercised by default.
    link_dropout: float = 0.15
    dot_dropout: float = 0.10
    # Per-joint gross hardstop-init error (prototype-hand quirks), e.g.
    # {"ring_abd": 20.0}; exercises the de-novo init path.
    gross_init_err: dict = field(default_factory=dict)
    offsets_only: bool = False
    sweep_steps: int = 30
    n_anchor: int = 8
    n_palm: int = 5
    n_plate_wrist: int = 3
    seeds: int = 5


@dataclass
class Truth:
    kin: KinematicModel
    enc: EncoderTruth
    base_R: np.ndarray
    base_t: np.ndarray
    dots: dict[str, np.ndarray]          # {link: (K,3) local, as-printed}
    dir_bias: dict[str, np.ndarray]      # {link: (3,3)} print-vs-mesh rotation
    tip_bias: dict[str, np.ndarray]      # {finger: (3,)} local
    scale: float
    field: object                        # callable (N,3)->(N,3)
    hardstop_b0: dict[str, float]        # hardstop-derived offset init (deg)


def _smooth_field(rng: np.random.Generator, amp: float):
    """Smooth systematic 3D error field over the ~0.3 m workspace."""
    k = rng.normal(0, 2 * np.pi / 0.5, size=(3, 3))
    phase = rng.uniform(0, 2 * np.pi, size=3)

    def field(x: np.ndarray) -> np.ndarray:
        return amp * np.sin(x @ k.T + phase)

    return field


def _small_rotation(rng: np.random.Generator, sigma_deg: float) -> np.ndarray:
    if sigma_deg <= 0:
        return np.eye(3)
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    return rotation_about_axis(axis, np.deg2rad(rng.normal(0, sigma_deg)))


def make_truth(nominal: KinematicModel, sc: Scenario, rng: np.random.Generator) -> Truth:
    joints = nominal.joint_names
    kin = nominal.perturbed(rng, axis_tilt_deg=sc.axis_tilt_deg,
                            origin_shift_m=sc.origin_shift_m, scale=0.0)
    enc = EncoderTruth.sample(rng, joints, inl_deg=sc.inl_deg, gain_frac=sc.gain_frac)

    dots = {}
    for link in DOT_LINKS:
        z = rng.uniform(0.005, max(LINK_SPAN[link], 0.012), size=DOTS_PER_LINK)
        ang = rng.uniform(0, 2 * np.pi, size=DOTS_PER_LINK)
        r = rng.uniform(0.007, 0.011, size=DOTS_PER_LINK)
        dots[link] = np.stack([r * np.cos(ang), r * np.sin(ang), z], axis=1)

    base_R = _small_rotation(rng, 8.0)
    base_t = np.array([0.0, 0.0, 0.05]) + rng.normal(0, 0.01, size=3)

    hardstop_b0 = {
        j: -enc.offset[j] + rng.normal(0, sc.hardstop_bias_deg)
        + sc.gross_init_err.get(j, 0.0)
        for j in joints
    }

    return Truth(
        kin=kin, enc=enc, base_R=base_R, base_t=base_t, dots=dots,
        dir_bias={l: _small_rotation(rng, sc.dir_bias_deg)
                  for l in DIR_LINKS + list(FRAME_ANCHOR_LINKS)},
        tip_bias={f: rng.normal(0, sc.tip_bias_m, size=3) for f in FINGERS},
        scale=rng.normal(0, sc.scale) if sc.scale > 0 else 0.0,
        field=_smooth_field(rng, sc.field_amp_m) if sc.field_amp_m > 0 else (lambda x: 0.0 * x),
        hardstop_b0=hardstop_b0,
    )


def _measure_points(x: np.ndarray, truth: Truth, rng: np.random.Generator,
                    noise: float) -> np.ndarray:
    return (1.0 + truth.scale) * x + truth.field(x) + rng.normal(0, noise, size=x.shape)


def generate(nominal: KinematicModel, sc: Scenario, truth: Truth,
             rng: np.random.Generator) -> Dataset:
    joints = nominal.joint_names
    jidx = {j: i for i, j in enumerate(joints)}

    # ---- assemble the pose list -------------------------------------------
    q_rows: list[np.ndarray] = []
    sweep_frames: dict[str, list[int]] = {j: [] for j in joints}
    held = np.array([HELD_POSE[j] for j in joints], float)

    for j in joints:
        lo, hi = JOINT_ROMS[j]
        steps = np.linspace(lo + 2, hi - 2, sc.sweep_steps)
        for s in steps:
            row = held + rng.normal(0, 0.03, size=len(joints))  # held-joint drift
            row[jidx[j]] = s
            sweep_frames[j].append(len(q_rows))
            q_rows.append(row)

    dir_frames = []
    for _ in range(sc.n_anchor):
        row = np.array([
            rng.uniform(lo + 0.3 * (hi - lo), lo + 0.7 * (hi - lo))
            for lo, hi in (JOINT_ROMS[j] for j in joints)
        ])
        dir_frames.append(len(q_rows))
        q_rows.append(row)

    # Wrist-variation poses: extra observation frames exercising the wrist
    # chain (the palm anchor itself rides dir_frames like every frame anchor).
    wrist_lo, wrist_hi = JOINT_ROMS["wrist"]
    for w in np.linspace(wrist_lo + 5, wrist_hi - 5, sc.n_palm):
        row = held.copy()
        row[jidx["wrist"]] = w
        q_rows.append(row)

    plate_specs = []  # (frame_idx, finger)
    for w in np.linspace(wrist_lo + 10, wrist_hi - 10, sc.n_plate_wrist):
        for f in FINGERS:
            row = held + rng.normal(0, 5.0, size=len(joints))
            row = np.clip(row, [JOINT_ROMS[j][0] + 1 for j in joints],
                          [JOINT_ROMS[j][1] - 1 for j in joints])
            row[jidx["wrist"]] = w
            plate_specs.append((len(q_rows), f))
            q_rows.append(row)

    q = np.array(q_rows)  # (F, J)
    F = len(q)

    # ---- true world poses --------------------------------------------------
    q_dict = {j: q[:, i] for i, j in enumerate(joints)}
    poses = truth.kin.link_poses(q_dict)

    def world(link):
        R, t = poses[link]
        return truth.base_R @ R, (truth.base_R @ t[..., None])[..., 0] + truth.base_t

    # ---- encoders ----------------------------------------------------------
    m = np.stack([truth.enc.measure(j, q[:, i], rng) for i, j in enumerate(joints)], axis=1)

    # ---- dots --------------------------------------------------------------
    dot_obs = {}
    for link in DOT_LINKS:
        R, t = world(link)
        pts = np.einsum("fij,kj->fki", R, truth.dots[link]) + t[:, None, :]
        pts = _measure_points(pts, truth, rng, sc.dot_noise_m)
        if sc.link_dropout > 0 or sc.dot_dropout > 0:
            link_gone = rng.random(F) < sc.link_dropout
            dot_gone = rng.random((F, pts.shape[1])) < sc.dot_dropout
            gone = link_gone[:, None] | dot_gone
            pts = np.where(gone[..., None], np.nan, pts)
        dot_obs[link] = pts

    # ---- direction anchors -------------------------------------------------
    dir_obs = {}
    z_hat = np.array([0.0, 0.0, 1.0])
    for link in DIR_LINKS:
        R, _ = world(link)
        # Print-vs-mesh bias rides the link frame, so it rotates with the link.
        d = np.einsum("fij,j->fi", R[dir_frames], truth.dir_bias[link] @ z_hat)
        noise_rot = np.stack([
            _small_rotation(rng, sc.dir_noise_deg) for _ in dir_frames
        ])
        d = np.einsum("fij,fj->fi", noise_rot, d)
        dir_obs[link] = d / np.linalg.norm(d, axis=1, keepdims=True)

    # ---- mesh-referenced frame anchors (forearm + palm dorsal shell) ------
    # Two axes = full orientation up to the link's shared print bias. The
    # forearm one is what makes the wrist offset observable at all; its bias
    # class is the wrist's accuracy floor.
    frame_axes = {}
    for link in FRAME_ANCHOR_LINKS:
        Rl, _ = world(link)
        axes_obs = []
        for axis in (np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0])):
            d = np.einsum("fij,j->fi", Rl[dir_frames], truth.dir_bias[link] @ axis)
            noise_rot = np.stack([_small_rotation(rng, sc.dir_noise_deg) for _ in dir_frames])
            d = np.einsum("fij,fj->fi", noise_rot, d)
            axes_obs.append(d / np.linalg.norm(d, axis=1, keepdims=True))
        frame_axes[link] = np.stack(axes_obs)

    # ---- plate contacts ----------------------------------------------------
    plate_obs = []
    for frame_idx, finger in plate_specs:
        link = DISTAL_LINK[finger]
        R, t = world(link)
        tip = R[frame_idx] @ (TIP_POINT_LOCAL[finger] + truth.tip_bias[finger]) + t[frame_idx]
        obs = _measure_points(tip[None, :], truth, rng, 0.5 * sc.dot_noise_m)[0]
        plate_obs.append((frame_idx, finger, obs))

    return Dataset(
        m=m, dot_obs=dot_obs, dir_obs=dir_obs, frame_axes=frame_axes,
        dir_frames=np.array(dir_frames), plate_obs=plate_obs,
        sweep_frames={j: np.array(v) for j, v in sweep_frames.items()},
        joints=joints, q_true=q, meta={"scenario": sc.name},
    )
