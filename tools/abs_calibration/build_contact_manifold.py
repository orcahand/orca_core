#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Precompute adjacent-finger contact manifolds from the description meshes.

For each abd pair and flexion posture of the contact protocol
(record_contacts.py), sweeps the parked finger's abduction over a grid and
bisects the pusher's abduction to the first-contact angle between the two
fingers' outer surfaces (URDF visual meshes — the merged parts bake the
silicone skin into the surface, which is what physically touches). The
resulting curve ``q_out = poly(q_arg)`` plus its honesty band is what turns
a recorded abd-block event into an estimator residual.

FK runs on the URDF itself so mesh poses are exactly description-consistent,
and is cross-checked numerically against the estimator's kinematic model
(the packaged v2 YAML) before anything is written — a silent frame mismatch
here would poison every contact residual.

Requires a checkout of orcahand_description next to this repo (or --urdf).

Usage:
    uv run python tools/abs_calibration/build_contact_manifold.py \
        --out tools/abs_calibration/contact_manifolds_right.yaml
"""
from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from scipy.spatial import cKDTree

from model import HELD_POSE, JOINT_ROMS, KinematicModel
from record_contacts import ABD_PAIRS, ABD_PIP_DEG, ABD_POSTURES
from orca_core.kinematics.transforms import Transform

DEFAULT_URDF = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..",
    "orcahand_description", "v2", "models", "urdf", "orcahand_right.urdf")

SAMPLES_PER_BODY = 6000
CONTACT_EPS_M = 5e-5          # bisection convergence on signed clearance
PRINT_ALLOWANCE_DEG = 0.5     # as-printed-vs-mesh bias class on contact faces
# The merged meshes carry the NOMINAL (uncompressed) silicone skin, so the
# manifold is the zero-force first-touch surface; the stall detector fires
# only once the skin resists the weak pusher, i.e. somewhat deeper. Verified
# against the hard-part meshes: the skin accounts for ~3 mm at the knuckles.
# Until the two-current extrapolation characterises it, that compression
# rides in sigma.
COMPLIANCE_ALLOWANCE_DEG = 1.5
GRID_POINTS = 9


class UrdfHand:
    """Right-hand URDF chains + posed surface point clouds per finger."""

    def __init__(self, urdf_path: str, mesh_root: str):
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        self.links = {}
        for link in root.findall("link"):
            vis = link.find("visual")
            if vis is None:
                continue
            mesh = vis.find("geometry/mesh")
            origin = vis.find("origin")
            xyz = np.array([float(v) for v in
                            (origin.get("xyz") or "0 0 0").split()])
            scale = np.array([float(v) for v in
                              (mesh.get("scale") or "1 1 1").split()])
            fname = mesh.get("filename").split("orcahand_description/")[-1]
            self.links[link.get("name")] = {
                "mesh": os.path.join(mesh_root, fname),
                "xyz": xyz, "scale": scale,
            }
        self.joints = {}
        self.children = {}
        for j in root.findall("joint"):
            origin = j.find("origin")
            entry = {
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "T": Transform.from_xyz_rpy(
                    [float(v) for v in origin.get("xyz").split()],
                    [float(v) for v in origin.get("rpy").split()]),
                "axis": (np.array([float(v) for v in
                                   j.find("axis").get("xyz").split()])
                         if j.find("axis") is not None else None),
            }
            self.joints[j.get("name")] = entry
            self.children.setdefault(entry["parent"], []).append(
                j.get("name"))

        self.carpals = next(l for l in self.links if l.startswith("R-Carpals"))
        self.joint_names = self._map_semantic_names()
        self.q_sign = {j: 1.0 for j in self.joint_names}
        self._clouds = {}

    def calibrate_signs(self, nominal: KinematicModel) -> None:
        """Per-joint sign between orca's joint convention and the URDF axes
        (the packaged model carries explicit sign fields; the URDF encodes
        direction in the axis vector — abduction differs between the two)."""
        poses0 = nominal.link_poses({})
        Rw, tw = poses0["wrist"]
        for jn in self.joint_names:
            finger = jn.rsplit("_", 1)[0]
            q_model = {jn: np.array([10.0])}
            poses = nominal.link_poses(q_model)
            tp = poses[f"{finger}_pip"][1][0]
            target = Rw[0].T @ (tp - tw[0])
            pip_link = self.joints[self.joint_names[f"{finger}_pip"]]["child"]
            errs = {}
            for s in (1.0, -1.0):
                self.q_sign[jn] = s
                t = self.chain_pose(finger, {jn: 10.0})[pip_link][1]
                errs[s] = np.linalg.norm(t - target)
            self.q_sign[jn] = min(errs, key=errs.get)

    def _map_semantic_names(self) -> dict[str, str]:
        """orca joint name -> URDF joint name for the four finger chains."""
        out = {}
        abd_joints = [n for n in self.children.get(self.carpals, [])
                      if self.joints[n]["type"] == "revolute"]
        m_aps = []
        for name in abd_joints:
            child = self.joints[name]["child"]
            if child.startswith("P-AP"):
                out["pinky_abd"] = name
            elif child.startswith("I-AP"):
                out["index_abd"] = name
            elif child.startswith("M-AP"):
                m_aps.append(name)
            # thumb chain (T-TP) is not part of the abd-block protocol
        # Middle vs ring share the M-AP part; on the right hand +x is the
        # pinky side, so the M-AP joint with the larger origin x is the ring.
        m_aps.sort(key=lambda n: self.joints[n]["T"].translation[0])
        out["middle_abd"], out["ring_abd"] = m_aps[0], m_aps[1]

        for finger in ("pinky", "ring", "middle", "index"):
            abd = out[f"{finger}_abd"]
            mcp = next(n for n in self.children[self.joints[abd]["child"]]
                       if self.joints[n]["type"] == "revolute")
            pip = next(n for n in self.children[self.joints[mcp]["child"]]
                       if self.joints[n]["type"] == "revolute")
            out[f"{finger}_mcp"] = mcp
            out[f"{finger}_pip"] = pip
        return out

    def _cloud(self, link_name: str):
        """Sampled OUTER-surface points + normals in the link frame (meters).

        The merged assembly meshes contain internal geometry (bearing
        pockets, tendon channels) whose faces would corrupt the signed
        clearance; keep only samples whose normal points away from the
        body's interior."""
        if link_name not in self._clouds:
            import trimesh
            info = self.links[link_name]
            mesh = trimesh.load(info["mesh"], force="mesh")
            mesh.apply_scale(info["scale"][0])
            mesh.apply_translation(info["xyz"])
            pts, face_idx = trimesh.sample.sample_surface(
                mesh, 2 * SAMPLES_PER_BODY)
            pts = np.asarray(pts)
            nrms = mesh.face_normals[face_idx]
            outward = np.einsum("ij,ij->i", nrms, pts - mesh.centroid) > 0
            self._clouds[link_name] = (pts[outward], nrms[outward])
        return self._clouds[link_name]

    def chain_pose(self, finger: str, q_deg: dict[str, float]):
        """(R, t) of each subtree link in the carpals frame."""
        poses = {}
        R = np.eye(3)
        t = np.zeros(3)
        for joint in (f"{finger}_abd", f"{finger}_mcp", f"{finger}_pip"):
            entry = self.joints[self.joint_names[joint]]
            R_new = R @ entry["T"].rotation
            t = t + R @ entry["T"].translation
            ang = np.deg2rad(self.q_sign[joint] * q_deg.get(joint, 0.0))
            axis = entry["axis"] / np.linalg.norm(entry["axis"])
            R = R_new @ _rodrigues(axis, ang)
            poses[entry["child"]] = (R, t.copy())
        return poses

    def posed_cloud(self, finger: str, q_deg: dict[str, float]):
        """Phalanx surfaces (PP + fingertip) in the carpals frame — the
        bodies that physically meet a neighbouring finger; the AP links sit
        buried in the palm shell and their meshes interleave permanently."""
        pts_all, nrm_all = [], []
        poses = self.chain_pose(finger, q_deg)
        skip = self.joints[self.joint_names[f"{finger}_abd"]]["child"]
        for link, (R, t) in poses.items():
            if link == skip:
                continue
            pts, nrms = self._cloud(link)
            pts_all.append(pts @ R.T + t)
            nrm_all.append(nrms @ R.T)
        return np.vstack(pts_all), np.vstack(nrm_all)


def _rodrigues(axis: np.ndarray, ang: float) -> np.ndarray:
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def signed_clearance(pts_a, pts_b, nrm_b) -> float:
    """Approximate minimum signed distance from cloud A to surface B.

    Nearest B sample per A point, signed by B's outward normal — negative
    when A has crossed inside B. Accuracy ~ half the sample spacing."""
    tree = cKDTree(pts_b)
    d, idx = tree.query(pts_a, k=1)
    near = d < 0.02   # only the closest region matters
    if not near.any():
        return float(d.min())
    signed = np.einsum("ij,ij->i", pts_a[near] - pts_b[idx[near]],
                       nrm_b[idx[near]])
    return float(signed.min())


def cross_check_fk(hand: UrdfHand, nominal: KinematicModel,
                   tol_m: float = 2e-4) -> float:
    """URDF chain FK must agree with the packaged-model FK (carpals frame)."""
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(4):
        q = {j: float(rng.uniform(*JOINT_ROMS[j])) for j in JOINT_ROMS}
        poses = nominal.link_poses({j: np.array([v]) for j, v in q.items()})
        Rw, tw = poses["wrist"]
        for finger in ("pinky", "ring", "middle", "index"):
            urdf_poses = hand.chain_pose(finger, q)
            pip_link = hand.joints[hand.joint_names[f"{finger}_pip"]]["child"]
            t_urdf = urdf_poses[pip_link][1]
            Rp, tp = poses[f"{finger}_pip"]
            t_model = Rw[0].T @ (tp[0] - tw[0])
            worst = max(worst, float(np.linalg.norm(t_urdf - t_model)))
    if worst > tol_m:
        raise SystemExit(
            f"URDF-vs-model FK disagreement {worst*1000:.2f} mm > "
            f"{tol_m*1000:.1f} mm — frame conventions differ; refusing to "
            "write manifolds")
    return worst


def build_pair(hand: UrdfHand, parked: str, pusher: str,
               posture: float) -> dict | None:
    fp, fb = parked.rsplit("_", 1)[0], pusher.rsplit("_", 1)[0]
    q_common = {
        f"{fp}_mcp": posture, f"{fb}_mcp": posture,
        f"{fp}_pip": ABD_PIP_DEG, f"{fb}_pip": ABD_PIP_DEG,
    }
    lo_p, hi_p = JOINT_ROMS[parked]
    lo_b, hi_b = JOINT_ROMS[pusher]

    # Which pusher direction approaches the parked finger: clearance must
    # shrink. Probe at the parked mid angle.
    def clearance(qp, qb):
        q = dict(q_common)
        q[parked], q[pusher] = qp, qb
        pa, na = hand.posed_cloud(fp, q)
        pb, nb = hand.posed_cloud(fb, q)
        return signed_clearance(pa, pb, nb)

    qp_mid = 0.5 * (lo_p + hi_p)
    sign = 1.0 if clearance(qp_mid, hi_b - 1) < clearance(qp_mid, lo_b + 1) \
        else -1.0
    far = lo_b + 1 if sign > 0 else hi_b - 1
    deep = hi_b - 1 if sign > 0 else lo_b + 1

    grid, contact = [], []
    for qp in np.linspace(lo_p + 1, hi_p - 1, GRID_POINTS):
        c_far, c_deep = clearance(qp, far), clearance(qp, deep)
        if not (c_far > 0 > c_deep):
            continue  # no contact reachable at this parked angle
        a, b_ = far, deep
        for _ in range(24):
            mid = 0.5 * (a + b_)
            c = clearance(qp, mid)
            if abs(c) < CONTACT_EPS_M:
                b_ = mid
                break
            if c > 0:
                a = mid
            else:
                b_ = mid
        grid.append(qp)
        contact.append(0.5 * (a + b_))

    if len(grid) < 4:
        print(f"  {parked}<-{pusher} @ {posture:.0f}: contact reachable at "
              f"only {len(grid)} grid points — pair skipped")
        return None

    coeffs = np.polyfit(grid, contact, 2)
    fit_rms = float(np.sqrt(np.mean(
        (np.polyval(coeffs, grid) - np.asarray(contact)) ** 2)))
    sigma = float(np.sqrt(max(fit_rms, 0.15) ** 2 + PRINT_ALLOWANCE_DEG ** 2
                          + COMPLIANCE_ALLOWANCE_DEG ** 2))
    print(f"  {parked}<-{pusher} @ {posture:.0f}: {len(grid)} contact points, "
          f"fit rms {fit_rms:.2f} deg, sigma {sigma:.2f} deg")
    return {
        "pair": sorted((parked, pusher)),
        "arg": parked, "out": pusher,
        "poly": [float(c) for c in coeffs],
        "arg_range": [float(grid[0]), float(grid[-1])],
        "sigma_deg": sigma,
        "posture": {j: float(v) for j, v in q_common.items()},
        "posture_tol_deg": 3.0,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    urdf = os.path.abspath(args.urdf)
    if not os.path.exists(urdf):
        raise SystemExit(f"URDF not found: {urdf} (pass --urdf)")
    mesh_root = os.path.dirname(os.path.dirname(os.path.dirname(urdf)))
    mesh_root = os.path.dirname(mesh_root)  # repo root (paths are repo-relative)

    hand = UrdfHand(urdf, mesh_root)
    nominal = KinematicModel.load("right")
    hand.calibrate_signs(nominal)
    flipped = sorted(j for j, s in hand.q_sign.items() if s < 0)
    if flipped:
        print(f"URDF axis sign flipped vs orca convention: {flipped}")
    worst = cross_check_fk(hand, nominal)
    print(f"URDF vs packaged-model FK agreement: {worst*1e6:.0f} um")

    entries = []
    for a, b in ABD_PAIRS:
        for parked, pusher in ((a, b), (b, a)):
            for posture in ABD_POSTURES:
                entry = build_pair(hand, parked, pusher, posture)
                if entry:
                    entries.append(entry)

    with open(args.out, "w") as f:
        yaml.safe_dump(entries, f, sort_keys=False)
    print(f"wrote {len(entries)} manifolds to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
