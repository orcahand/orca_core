"""The calibration object: a square tube on the hand's connector with ChArUco
tiles on its sides (rig config ``calibration_object``).

Plate frame: z up along the turntable axis, origin on the plate's top face at
the axis, x toward the camera at stop 0. Lengths in mm. Tile frame (OpenCV's
board frame): origin at the top-left corner of the tile's front face, x right,
y down, z into the tile. ChArUco corner ``k`` of an ``nx x ny`` board sits at
``(margin + (k % (nx-1) + 1) * square, margin + (k // (nx-1) + 1) * square)``.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
from orca_core.kinematics.transforms import Transform


@dataclass(frozen=True)
class TileSpec:
    dictionary: str
    squares: tuple[int, int]
    square_size: float
    marker_size: float
    size: tuple[float, float, float]
    margin: float

    @property
    def markers_per_tile(self) -> int:
        return self.squares[0] * self.squares[1] // 2

    @property
    def corners(self) -> np.ndarray:
        """Every ChArUco corner in the tile frame, (n, 3), row by row."""
        nx, ny = self.squares
        i, j = np.meshgrid(np.arange(1, nx), np.arange(1, ny))
        xy = np.column_stack([i.ravel(), j.ravel()]) * self.square_size + self.margin
        return np.column_stack([xy, np.zeros(len(xy))])


@dataclass(frozen=True)
class Tile:
    name: str
    row: int
    side: int
    first_marker: int
    pose: Transform                       # tile frame -> plate frame, mm

    def marker_ids(self, spec: TileSpec) -> np.ndarray:
        return np.arange(self.first_marker, self.first_marker + spec.markers_per_tile)


def _rotation(x, y, z) -> np.ndarray:
    return np.column_stack([x, y, z])


@dataclass(frozen=True)
class CalibrationObject:
    spec: TileSpec
    tiles: dict[str, Tile]
    reference: str

    @classmethod
    def from_config(cls, cfg: Mapping) -> CalibrationObject:
        t = cfg["tile"]
        spec = TileSpec(t["dictionary"], tuple(t["squares"]), float(t["square_size"]), float(t["marker_size"]),
                        tuple(float(v) for v in t["size"]), float(t["margin"]))
        sides, first = int(cfg["sides"]), int(cfg.get("first_id", 0))
        nominal = cls._nominal_poses(cfg, spec)
        explicit = cfg.get("tiles") or {}
        tiles = {}
        for row, r in enumerate(cfg["rows"]):
            for side in range(sides):
                name = f"tile_r{row}_s{side}"
                if name not in explicit and (explicit or side not in r.get("sides", range(sides))):
                    continue                       # an empty seat
                pose = nominal[name]
                if name in explicit:
                    e = explicit[name]
                    pose = Transform.from_xyz_rpy(e["corner"], np.deg2rad(e["rpy"]))
                tiles[name] = Tile(name, row, side, first + (row * sides + side) * spec.markers_per_tile, pose)
        reference = cfg.get("reference_tile") or next(iter(tiles))
        if reference not in tiles:
            raise ValueError(f"reference tile {reference} is not on the object")
        return cls(spec, tiles, reference)

    @staticmethod
    def _nominal_poses(cfg: Mapping, spec: TileSpec) -> dict[str, Transform]:
        """Seats from the design values: side s faces outward at plate angle
        -side0_stop + 90° s; a tile's bottom edge touches face_distance; lean
        tilts its top outward about that edge."""
        w, h, _ = spec.size
        z = np.array([0.0, 0.0, 1.0])
        poses = {}
        for row, r in enumerate(cfg["rows"]):
            lean = np.deg2rad(float(r.get("lean", 0.0)))
            for side in range(int(cfg["sides"])):
                phi = np.deg2rad(-float(cfg.get("side0_stop", 0.0)) + 90.0 * side)
                n = np.array([np.cos(phi), np.sin(phi), 0.0])
                right = np.array([-np.sin(phi), np.cos(phi), 0.0])
                up = np.cos(lean) * z + np.sin(lean) * n
                normal = np.cos(lean) * n - np.sin(lean) * z
                bottom = float(cfg["face_distance"]) * n + (float(r["height"]) - h / 2 * np.cos(lean)) * z
                top_left = bottom + h * up - w / 2 * right
                poses[f"tile_r{row}_s{side}"] = Transform.from_rotation_translation(
                    _rotation(right, -up, -normal), top_left)
        return poses

    def tile_for_marker(self, marker_id: int) -> str | None:
        for t in self.tiles.values():
            if t.first_marker <= marker_id < t.first_marker + self.spec.markers_per_tile:
                return t.name
        return None

    def corners_plate(self, tile: str, ids: np.ndarray | None = None) -> np.ndarray:
        """Tile corners (all, or the given corner ids) in the plate frame, mm."""
        c = self.spec.corners if ids is None else self.spec.corners[np.asarray(ids, int)]
        return self.tiles[tile].pose.apply_to_points(c)
