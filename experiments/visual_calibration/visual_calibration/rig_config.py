# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""The camera rig for visual calibration: ``rigs/<version>/<name>/config.yaml``
(design values and solver settings) and ``calibration.yaml`` (what the solver
found). Plan: ``README.md``."""

import math
import os
from dataclasses import dataclass, field

from orca_core.utils.utils import _version_sort_key, read_yaml, write_yaml_atomic

DEFAULT_RIG_NAME = "camera-rig"
ANGLE_TOLERANCE_DEG = 1e-6


class RigConfigError(ValueError):
    pass


def rigs_dir() -> str:
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "rigs")


def get_rig_path(name: str = DEFAULT_RIG_NAME, version: str | None = None) -> str:
    """The rig's directory; the newest version that has `name` when `version` is None."""
    root = rigs_dir()
    versions = [version] if version else sorted(
        (v for v in os.listdir(root) if os.path.isdir(os.path.join(root, v))), key=_version_sort_key, reverse=True)
    for v in versions:
        path = os.path.join(root, v, name)
        if os.path.isdir(path):
            return path
    raise FileNotFoundError(f"No rig {name!r}{f' in {version}' if version else ''} under {root}")


def _as_list(value) -> list[float]:
    return [] if value is None else [float(v) for v in (value if isinstance(value, list) else [value])]


def _search_entries(node: dict, where: str):
    """Every {bound, sigma} leaf of solver.search, with its dotted path."""
    if "bound" in node:
        yield where, node
        return
    for key, child in node.items():
        if not isinstance(child, dict):
            raise RigConfigError(f"{where}.{key}: expected {{bound, sigma}}")
        yield from _search_entries(child, f"{where}.{key}")


@dataclass(frozen=True)
class HardStop:
    stop_center_angle: float
    stop_arm_half_angle: float
    fin_angle: float
    fin_contact_angle: float

    @property
    def arm_sector(self) -> tuple:
        return (self.stop_center_angle - self.stop_arm_half_angle, self.stop_center_angle + self.stop_arm_half_angle)

    @property
    def travel(self) -> float:
        """0° is the fin against the arm; it turns until it meets the arm's other side."""
        return 360.0 - self.fin_angle - 2.0 * self.stop_arm_half_angle

    def validate(self) -> None:
        if self.stop_arm_half_angle <= 0 or self.fin_angle <= 0:
            raise RigConfigError("hard_stop: arm and fin widths must be positive")
        if self.travel <= 0:
            raise RigConfigError(f"hard_stop: fin {self.fin_angle}° + arm {2 * self.stop_arm_half_angle}° leave no travel")
        if abs(self.fin_contact_angle - self.arm_sector[1]) > ANGLE_TOLERANCE_DEG:
            raise RigConfigError(
                f"hard_stop: fin_contact_angle {self.fin_contact_angle}° must meet the arm's edge at {self.arm_sector[1]}°")


@dataclass(frozen=True)
class Stops:
    start: float
    step: float
    count: int
    settle_s: float = 0.5
    home_each_sweep: bool = True

    @property
    def angles(self) -> list[float]:
        return [self.start + i * self.step for i in range(self.count)]


@dataclass
class RigConfig:
    path: str
    hard_stop: HardStop
    travel: float
    stops: Stops
    raw: dict = field(repr=False)

    @property
    def version(self) -> str:
        return os.path.basename(os.path.dirname(self.path))

    @property
    def name(self) -> str:
        return os.path.basename(self.path)

    @property
    def camera_nominal(self) -> dict | None:
        """The CAD camera pose {position, look_at} (rig frame, mm) from camera.mount, or nominal_pose if set."""
        cam = self.raw.get("camera") or {}
        if cam.get("nominal_pose"):
            return cam["nominal_pose"]
        m = cam.get("mount")
        if not m:
            return None
        dx, dz = m.get("lens_offset", [0.0, 0.0])
        x, z, tilt = float(m["distance"]) + dx, float(m["height"]) + dz, math.radians(float(m["tilt"]))
        return {"position": [round(x, 3), 0.0, round(z, 3)], "look_at": [0.0, 0.0, round(z - x * math.tan(tilt), 3)]}

    @property
    def search(self) -> dict:
        return self.raw["solver"]["search"]

    @classmethod
    def load(cls, name: str = DEFAULT_RIG_NAME, version: str | None = None, path: str | None = None) -> "RigConfig":
        path = path or get_rig_path(name, version)
        raw = read_yaml(os.path.join(path, "config.yaml"))
        if not raw:
            raise RigConfigError(f"{path}/config.yaml is missing or empty")
        if "turntable" not in raw:
            raw["turntable"] = read_yaml(os.path.join(path, "turntable.yaml"))
            if not raw["turntable"]:
                raise RigConfigError(f"{path}/turntable.yaml is missing or empty")
        try:
            table = raw["turntable"]
            arm, fin = table["hard_stop"]["arm"], table["hard_stop"]["fin"]
            hard_stop = HardStop(float(arm["stop_center_angle"]), float(arm["stop_arm_half_angle"]),
                                 float(fin["fin_angle"]), float(fin["fin_contact_angle"]))
            stops = Stops(**table["stops"])
            config = cls(path, hard_stop, float(table["travel"]), stops, raw)
        except (KeyError, TypeError) as exc:
            raise RigConfigError(f"{path}/config.yaml: missing or malformed {exc}") from exc
        config.validate()
        return config

    def validate(self) -> None:
        self.hard_stop.validate()
        if abs(self.travel - self.hard_stop.travel) > ANGLE_TOLERANCE_DEG:
            raise RigConfigError(f"travel {self.travel}° does not match the hard stop's {self.hard_stop.travel}°")
        if self.stops.count < 1 or self.stops.step <= 0:
            raise RigConfigError("stops: count must be >= 1 and step > 0")
        if self.stops.start < 0 or self.stops.angles[-1] > self.travel:
            raise RigConfigError(
                f"stops {self.stops.angles[0]}..{self.stops.angles[-1]}° leave the travel 0..{self.travel}°")
        for where, entry in _search_entries(self.search, "solver.search"):
            bounds, sigmas = _as_list(entry.get("bound")), _as_list(entry.get("sigma"))
            if not bounds or len(bounds) != len(sigmas) or min(bounds + sigmas) <= 0:
                raise RigConfigError(f"{where}: bound and sigma must be positive, one per direction")
        mount = (self.raw.get("camera") or {}).get("mount") or {}
        for key, o in (mount.get("options") or {}).items():
            v = float(mount[key])
            k = (v - o["min"]) / o["step"]
            if not o["min"] <= v <= o["max"] or abs(k - round(k)) > 1e-6:
                raise RigConfigError(f"camera.mount.{key} {v:g} is not a hole: {o['min']:g}..{o['max']:g} in steps of {o['step']:g}")

    @property
    def calibration_path(self) -> str:
        return os.path.join(self.path, "calibration.yaml")

    @property
    def turntable_calibration_path(self) -> str:
        return os.path.join(self.path, "turntable_calibration.yaml")

    def load_calibration(self) -> dict:
        return read_yaml(self.calibration_path) or {}

    def save_calibration(self, calibration: dict, keep_history: bool = True) -> None:
        """Write the solver's result; the previous run goes to `history`."""
        calibration = dict(calibration)
        if keep_history:
            old = self.load_calibration()
            history = list(old.get("history") or [])
            if old.get("calibrated_at"):
                history.append({k: old.get(k) for k in ("calibrated_at", "lens", "camera", "axis", "tower", "mounting", "stop_angle_corrections", "tile_poses", "fit")})
            calibration["history"] = history
        write_yaml_atomic(self.calibration_path, calibration)
