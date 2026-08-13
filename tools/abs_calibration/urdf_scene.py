# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""URDF meshes of the hand as point clouds in the packaged model's frames.

Verified numerically: the URDF link frames and the packaged v2 kinematic
model's link frames agree EXACTLY (0 um / 0.000 deg over random
configurations, full tree including wrist and thumb) once the four finger
abduction signs are flipped — and since posing happens through the
packaged model here, even the sign difference is irrelevant: only the
static link-frame geometry is taken from the URDF. That lets the renderer
pose real mesh surfaces with the FK it already has.

Clouds are keyed by the model's link names ("wrist" is the palm/carpals
body; "fixed0" the top tower; "world" the forearm structure at the model
root). The ``*_Skin`` sub-meshes mark the silicone zones — dot placement
excludes them — and take the same per-link visual origin offset as the
merged part (verified against the palm, whose offset is ~5 cm).

(The contact-manifold builder keeps its own finger-only URDF parser with
runtime sign calibration; merging the two is deliberate future cleanup —
its behaviour is validated and pinned.)
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import numpy as np

DEFAULT_URDF = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..",
    "orcahand_description", "v2", "models", "urdf", "orcahand_right.urdf")

# URDF child-link prefix -> model link name carrying that body.
_BODY_OF_PREFIX = [
    ("ForeArmStructure", "world"),
    ("TopTower", "fixed0"),
    ("R-Carpals", "wrist"),
    ("T-TP", "thumb_cmc"),
    ("R-T-AP", "thumb_abd"),
    ("T-PP", "thumb_mcp"),
    ("T-DP", "thumb_dip"),
    ("P-AP", "pinky_abd"), ("P-PP", "pinky_mcp"),
    ("P-FingerTipAssembly", "pinky_pip"),
    ("I-AP", "index_abd"), ("I-PP", "index_mcp"),
    ("I-FingerTipAssembly", "index_pip"),
]

# Skin sub-meshes (silicone zones, dot-free) per model link. The M- parts
# are shared between middle and ring.
_SKIN_FILES = {
    "wrist": ["R-Carpals_Skin.stl"],
    "thumb_mcp": ["T-PP_Skin.stl"],
    "thumb_dip": ["T-DP_Skin.stl"],
    "pinky_mcp": ["P-PP_Skin.stl"],
    "pinky_pip": ["P-FingerTipAssembly_P-DP-Skin.stl"],
    "index_mcp": ["I-PP_Skin.stl"],
    "index_pip": ["I-FingerTipAssembly_I-DP-Skin.stl"],
    "middle_mcp": ["M-PP_Skin.stl"], "ring_mcp": ["M-PP_Skin.stl"],
    "middle_pip": ["M-FingerTipAssembly_M-DP-Skin.stl"],
    "ring_pip": ["M-FingerTipAssembly_M-DP-Skin.stl"],
}


def _parse(urdf_path: str):
    root = ET.parse(urdf_path).getroot()
    links, joints = {}, []
    for link in root.findall("link"):
        vis = link.find("visual")
        if vis is None:
            continue
        mesh = vis.find("geometry/mesh")
        origin = vis.find("origin")
        links[link.get("name")] = {
            "file": os.path.basename(mesh.get("filename")),
            "xyz": np.array([float(v) for v in
                             (origin.get("xyz") or "0 0 0").split()]),
            "scale": float(mesh.get("scale", "0.001 0.001 0.001").split()[0]),
        }
    for j in root.findall("joint"):
        joints.append({"parent": j.find("parent").get("link"),
                       "child": j.find("child").get("link"),
                       "T": j.find("origin")})
    return links, joints


def _model_link_of(urdf_link: str, joints) -> str | None:
    for prefix, name in _BODY_OF_PREFIX:
        if urdf_link.startswith(prefix):
            return name
    if urdf_link.startswith(("M-AP", "M-PP", "M-FingerTipAssembly")):
        # Middle and ring share the M- parts; disambiguate by the abd
        # joint's origin x (right hand: +x is the pinky side -> ring).
        chain_root = urdf_link
        parents = {j["child"]: j["parent"] for j in joints}
        while not parents.get(chain_root, "").startswith("R-Carpals"):
            chain_root = parents[chain_root]
        xs = sorted(
            (float(j["T"].get("xyz").split()[0]), j["child"])
            for j in joints
            if j["child"].startswith("M-AP")
            and j["parent"].startswith("R-Carpals")
        )
        finger = "middle" if chain_root == xs[0][1] else "ring"
        stage = ("abd" if "M-AP" in urdf_link
                 else "mcp" if "M-PP" in urdf_link else "pip")
        return f"{finger}_{stage}"
    return None


class MeshScene:
    """Point-cloud surfaces (with normals) of every hand body, plus skin
    exclusion clouds, all in the packaged model's link frames.

    Sampling is deterministic (seeded per mesh file): a scene rebuilt from
    the same config reproduces the exact clouds — reusing a rendered workdir
    depends on it."""

    def __init__(self, urdf_path: str = DEFAULT_URDF, n_per_body: int = 6000,
                 seed: int = 0):
        urdf_path = os.path.abspath(urdf_path)
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(
                f"orcahand_description URDF not found at {urdf_path} — the "
                "mesh-rendered synthetic scene needs the description checkout")
        self._seed = seed
        side = "left" if "left" in os.path.basename(urdf_path) else "right"
        self.asset_dir = os.path.join(os.path.dirname(urdf_path), "..",
                                      "assets", side)
        links, joints = _parse(urdf_path)
        self.bodies: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.visual_offset: dict[str, np.ndarray] = {}
        for urdf_link, info in links.items():
            model_link = _model_link_of(urdf_link, joints)
            if model_link is None:
                continue
            pts, nrm = self._sample(info["file"], info["scale"], info["xyz"],
                                    n_per_body)
            self.bodies[model_link] = (pts, nrm)
            self.visual_offset[model_link] = info["xyz"]
        self.skin: dict[str, np.ndarray] = {}
        for model_link, files in _SKIN_FILES.items():
            clouds = [self._sample(f, 0.001,
                                   self.visual_offset.get(model_link, 0.0),
                                   1500)[0] for f in files]
            self.skin[model_link] = np.vstack(clouds)

    def _sample(self, fname: str, scale: float, offset, n: int):
        import zlib

        import trimesh
        mesh = trimesh.load(os.path.join(self.asset_dir, fname), force="mesh")
        mesh.apply_scale(scale)
        mesh.apply_translation(np.broadcast_to(np.asarray(offset, float), 3))
        seed = (self._seed + zlib.crc32(f"{fname}:{n}".encode())) & 0x7FFFFFFF
        pts, face_idx = trimesh.sample.sample_surface(mesh, n, seed=seed)
        return np.asarray(pts), mesh.face_normals[face_idx]
