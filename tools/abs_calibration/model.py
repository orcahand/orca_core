# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Kinematic + encoder models for the absolute-calibration synthetic study.

Loads the packaged v2 chains into a flat, perturbable joint tree and provides
batched forward kinematics to *every* link frame (HandKinematics only exposes
fingertips). The study needs two copies of the model — a perturbed "as-printed"
truth and the nominal model the estimator is allowed to know — so the chain
lives here as plain arrays rather than behind the packaged class.

Angles are degrees in the orca_core joint convention throughout.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np
import yaml

from orca_core.kinematics.hand_kinematics import DATA_DIR
from orca_core.kinematics.transforms import Transform, rotation_about_axis

import os

FINGERS = ("thumb", "index", "middle", "ring", "pinky")

# Config ROMs (deg) for the v2 hand; sweep and test poses stay inside these.
JOINT_ROMS: dict[str, tuple[float, float]] = {
    "wrist": (-65, 35),
    "thumb_cmc": (-45, 33), "thumb_abd": (-18, 55),
    "thumb_mcp": (-25, 100), "thumb_dip": (-15, 107),
    "index_abd": (-30, 25), "index_mcp": (-25, 100), "index_pip": (-15, 107),
    "middle_abd": (-27, 27), "middle_mcp": (-25, 100), "middle_pip": (-15, 107),
    "ring_abd": (-27, 27), "ring_mcp": (-25, 100), "ring_pip": (-15, 107),
    "pinky_abd": (-30, 30), "pinky_mcp": (-25, 100), "pinky_pip": (-15, 107),
}

# Held pose during another joint's sweep (the config neutral: a natural spread).
HELD_POSE: dict[str, float] = {
    "thumb_cmc": 0, "thumb_abd": 50, "thumb_mcp": 33, "thumb_dip": 18,
    "index_abd": -14, "index_mcp": 2, "index_pip": 6,
    "middle_abd": -4, "middle_mcp": 2, "middle_pip": 4,
    "ring_abd": 10, "ring_mcp": -2, "ring_pip": 8,
    "pinky_abd": 22, "pinky_mcp": -4, "pinky_pip": -2,
    "wrist": -20,
}

# Fingertip point in the distal link frame (URDF mesh z-max, meters).
TIP_POINT_LOCAL: dict[str, np.ndarray] = {
    "thumb": np.array([0.0, 0.0, 0.0305]),
    "index": np.array([0.0, 0.0, 0.0453]),
    "middle": np.array([0.0, 0.0, 0.0453]),
    "ring": np.array([0.0, 0.0, 0.0453]),
    "pinky": np.array([0.0, 0.0, 0.0383]),
}

DISTAL_LINK = {
    "thumb": "thumb_dip", "index": "index_pip", "middle": "middle_pip",
    "ring": "ring_pip", "pinky": "pinky_pip",
}

PALM_LINK = "wrist"  # child of the wrist joint = carpals/palm body


@dataclass
class JointNode:
    name: str
    parent: int  # index into KinematicModel.nodes; -1 = world/base entry
    origin_R: np.ndarray  # (3,3)
    origin_t: np.ndarray  # (3,)
    axis: np.ndarray | None  # None for fixed entries
    sign: float = 1.0


@dataclass
class KinematicModel:
    """Flat joint tree with batched FK to every link frame."""

    nodes: list[JointNode]
    index: dict[str, int] = field(init=False)

    def __post_init__(self):
        self.index = {n.name: i for i, n in enumerate(self.nodes) if n.axis is not None}

    @classmethod
    def load(cls, side: str = "right") -> "KinematicModel":
        path = os.path.join(DATA_DIR, "v2_kinematics.yaml")
        with open(path) as f:
            chains = yaml.safe_load(f)[side]["chains"]

        nodes: list[JointNode] = []

        def add(entry: dict, parent: int) -> int:
            T = Transform.from_xyz_rpy(entry["xyz"], entry["rpy"])
            axis = None if entry.get("fixed") else np.asarray(entry["axis"], float)
            nodes.append(JointNode(
                name=entry.get("joint", f"fixed{len(nodes)}"),
                parent=parent,
                origin_R=T.rotation.copy(),
                origin_t=T.translation.copy(),
                axis=axis,
                sign=float(entry.get("sign", 1)),
            ))
            return len(nodes) - 1

        parent = -1
        for entry in chains["base_chain"]:
            parent = add(entry, parent)
        wrist_idx = parent
        for finger in FINGERS:
            parent = wrist_idx
            for entry in chains["fingers"][finger]:
                parent = add(entry, parent)
        return cls(nodes)

    @property
    def joint_names(self) -> list[str]:
        return [n.name for n in self.nodes if n.axis is not None]

    def perturbed(self, rng: np.random.Generator, *, axis_tilt_deg: float = 0.0,
                  origin_shift_m: float = 0.0, scale: float = 0.0) -> "KinematicModel":
        """As-printed variant: tilted axes, shifted origins, global scale error."""
        new_nodes = []
        for n in self.nodes:
            axis = n.axis
            if axis is not None and axis_tilt_deg > 0:
                tilt_axis = rng.normal(size=3)
                tilt_axis -= axis * (tilt_axis @ axis)
                tilt_axis /= np.linalg.norm(tilt_axis)
                ang = np.deg2rad(rng.normal(0, axis_tilt_deg))
                axis = rotation_about_axis(tilt_axis, ang) @ axis
            t = n.origin_t * (1.0 + scale)
            if origin_shift_m > 0 and n.axis is not None:
                t = t + rng.normal(0, origin_shift_m, size=3)
            new_nodes.append(replace(n, origin_t=t, axis=axis))
        return KinematicModel(new_nodes)

    def link_poses(self, q: dict[str, np.ndarray] | dict[str, float]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """Batched FK: ``{link_name: (R (F,3,3), t (F,3))}`` for F poses.

        ``q`` maps joint name -> scalar or (F,) array; missing joints are 0.
        Link name = the joint whose child it is; fixed entries use their
        generated name.
        """
        lengths = [np.shape(v) for v in q.values() if np.ndim(v) > 0]
        F = lengths[0][0] if lengths else 1
        eye = np.broadcast_to(np.eye(3), (F, 3, 3))
        zeros = np.zeros((F, 3))
        poses: list[tuple[np.ndarray, np.ndarray]] = []
        out: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for n in self.nodes:
            pR, pt = poses[n.parent] if n.parent >= 0 else (eye, zeros)
            R = pR @ n.origin_R
            t = pt + np.einsum("fij,j->fi", pR, n.origin_t)
            if n.axis is not None:
                ang = np.deg2rad(n.sign * np.broadcast_to(np.asarray(q.get(n.name, 0.0), float), (F,)))
                R = R @ _batch_axis_rotation(n.axis, ang)
            poses.append((R, t))
            out[n.name] = (R, t)
        return out


def _batch_axis_rotation(axis: np.ndarray, angles: np.ndarray) -> np.ndarray:
    """Rodrigues rotation about a fixed axis for a batch of angles: (F,3,3)."""
    x, y, z = axis / np.linalg.norm(axis)
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    c = np.cos(angles)[:, None, None]
    s = np.sin(angles)[:, None, None]
    return c * np.eye(3) + s * K + (1 - c) * np.outer([x, y, z], [x, y, z])


@dataclass
class EncoderTruth:
    """Forward encoder model per joint: measured = q + error(q).

    ``error(q) = offset + gain*(q-mid) + A1*sin(2π(q+φ1)/360) + A2*sin(4π(q+φ2)/360)``
    — offset plus gain plus the first two harmonics of the mechanical
    revolution, i.e. the INL shape a diametric-magnet encoder actually has.
    """

    offset: dict[str, float]
    gain: dict[str, float]
    a1: dict[str, float]
    p1: dict[str, float]
    a2: dict[str, float]
    p2: dict[str, float]

    @classmethod
    def sample(cls, rng: np.random.Generator, joints: list[str], *,
               offset_deg: float = 3.0, gain_frac: float = 0.005,
               inl_deg: float = 0.6) -> "EncoderTruth":
        return cls(
            offset={j: float(rng.normal(0, offset_deg)) for j in joints},
            gain={j: float(rng.normal(0, gain_frac)) for j in joints},
            a1={j: float(abs(rng.normal(0, inl_deg))) for j in joints},
            p1={j: float(rng.uniform(0, 360)) for j in joints},
            a2={j: float(abs(rng.normal(0, inl_deg / 3))) for j in joints},
            p2={j: float(rng.uniform(0, 360)) for j in joints},
        )

    def measure(self, joint: str, q, rng: np.random.Generator | None = None,
                noise_deg: float = 0.01, quant_deg: float = 0.022) -> np.ndarray:
        q = np.asarray(q, float)
        mid = np.mean(JOINT_ROMS[joint])
        m = (q + self.offset[joint] + self.gain[joint] * (q - mid)
             + self.a1[joint] * np.sin(np.deg2rad(q + self.p1[joint]))
             + self.a2[joint] * np.sin(2 * np.deg2rad(q + self.p2[joint])))
        if rng is not None:
            m = m + rng.normal(0, noise_deg, size=m.shape)
            m = np.round(m / quant_deg) * quant_deg
        return m


def palm_plane_local(model: KinematicModel) -> tuple[np.ndarray, np.ndarray]:
    """(normal, centroid) of the four finger-abd origins, in the palm frame.

    Defines what "the palm plane" means consistently for truth and estimator.
    """
    q0 = {j: 0.0 for j in model.joint_names}
    poses = model.link_poses(q0)
    pR, pt = poses[PALM_LINK]
    pts = []
    for f in ("index", "middle", "ring", "pinky"):
        R, t = poses[f + "_abd"]
        # abd origin = its parent chain point; take frame origin position
        pts.append(t[0])
    pts = np.array(pts)
    local = (pts - pt[0]) @ pR[0]  # rows: points in palm frame
    centroid = local.mean(axis=0)
    _, _, vh = np.linalg.svd(local - centroid)
    normal = vh[2]
    return normal, centroid
