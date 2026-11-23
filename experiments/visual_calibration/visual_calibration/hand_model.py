"""The hand as the camera sees it: the orcahand_description URDF + link meshes.

The source of truth is the sibling repo ``orcahand_description`` (v2 URDF and
STLs); ``ORCAHAND_DESCRIPTION`` points elsewhere. Joint angles are degrees in
the URDF's own axis convention (not orca_core's motor signs), keyed by the
name without the side prefix (``index_mcp``). Lengths in metres.
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from functools import cached_property
from pathlib import Path

import numpy as np
from orca_core.kinematics.transforms import Transform, rotation_about_axis

DESCRIPTION_ENV = "ORCAHAND_DESCRIPTION"


def description_root() -> Path | None:
    """The orcahand_description checkout: $ORCAHAND_DESCRIPTION, else next to the orca_core checkout."""
    env = os.environ.get(DESCRIPTION_ENV)
    root = Path(env) if env else Path(__file__).resolve().parents[4] / "orcahand_description"
    return root if (root / "v2" / "models" / "urdf").is_dir() else None


def read_stl(path: Path) -> np.ndarray:
    """Unique vertices of a binary STL, (n, 3), in the file's units."""
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    tris = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                         count=count, offset=84)
    return np.unique(tris["v"].reshape(-1, 3).astype(float), axis=0)


def _origin(node) -> Transform:
    o = node.find("origin") if node is not None else None
    if o is None:
        return Transform.identity()
    xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    return Transform.from_xyz_rpy(xyz, rpy)


@dataclass(frozen=True)
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: Transform
    axis: np.ndarray
    limits_deg: tuple[float, float] | None


@dataclass(frozen=True)
class Visual:
    path: Path
    origin: Transform
    scale: np.ndarray


class HandModel:
    """Kinematic tree + visual meshes of one hand."""

    def __init__(self, urdf: Path, side: str):
        self.urdf, self.side = urdf, side
        root = ET.parse(urdf).getroot()
        package = urdf.parents[3]                    # .../orcahand_description
        prefix = f"{side}_"

        def short(n):
            return n.removeprefix(prefix)

        self.visuals: dict[str, Visual] = {}
        for link in root.findall("link"):
            mesh = link.find("visual/geometry/mesh")
            if mesh is not None:
                rel = mesh.get("filename").split("orcahand_description/", 1)[1]
                self.visuals[short(link.get("name"))] = Visual(
                    package / rel, _origin(link.find("visual")),
                    np.array([float(s) for s in mesh.get("scale", "1 1 1").split()]))

        self.joints: dict[str, Joint] = {}
        for j in root.findall("joint"):
            limit = j.find("limit")
            axis = j.find("axis")
            self.joints[short(j.get("name"))] = Joint(
                short(j.get("name")), j.get("type"), short(j.find("parent").get("link")),
                short(j.find("child").get("link")), _origin(j),
                np.array([float(v) for v in (axis.get("xyz") if axis is not None else "1 0 0").split()]),
                None if limit is None else (np.rad2deg(float(limit.get("lower"))), np.rad2deg(float(limit.get("upper")))))
        children = {j.child for j in self.joints.values()}
        self.base = next(j.parent for j in self.joints.values() if j.parent not in children)
        self._order = self._topological()

    @classmethod
    def load(cls, side: str = "right", root: Path | None = None) -> HandModel:
        root = root or description_root()
        if root is None:
            raise FileNotFoundError(f"orcahand_description not found; set {DESCRIPTION_ENV}")
        return cls(root / "v2" / "models" / "urdf" / f"orcahand_{side}.urdf", side)

    def _topological(self) -> list[Joint]:
        order, frontier = [], [self.base]
        while frontier:
            parent = frontier.pop()
            for j in self.joints.values():
                if j.parent == parent:
                    order.append(j)
                    frontier.append(j.child)
        return order

    @cached_property
    def movable(self) -> list[str]:
        return [j.name for j in self._order if j.type in ("revolute", "continuous")]

    def link_poses(self, q_deg: Mapping[str, float]) -> dict[str, Transform]:
        """Every link's pose in the base link (the forearm) frame."""
        poses = {self.base: Transform.identity()}
        for j in self._order:
            pose = poses[j.parent] @ j.origin
            angle = q_deg.get(j.name, 0.0) if j.type != "fixed" else 0.0
            if angle:
                pose = pose @ Transform.from_rotation_translation(
                    rotation_about_axis(j.axis, np.deg2rad(angle)), np.zeros(3))
            poses[j.child] = pose
        return poses

    def mesh(self, link: str) -> np.ndarray:
        """The link's visual mesh vertices in the link frame, metres."""
        if link not in self._meshes:
            v = self.visuals[link]
            self._meshes[link] = v.origin.apply_to_points(read_stl(v.path) * v.scale)
        return self._meshes[link]

    @cached_property
    def _meshes(self) -> dict[str, np.ndarray]:
        return {}

    def fingertip(self, finger: str) -> tuple[str, np.ndarray]:
        """(distal link, the tip point in it): the mesh vertex furthest along the
        last joint's distal direction (+z of the distal link)."""
        link = "thumb_dp" if finger == "thumb" else f"{finger}_dp"
        verts = self.mesh(link)
        return link, verts[np.argmax(verts[:, 2])]
