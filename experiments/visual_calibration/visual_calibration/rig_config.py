# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""The camera rig for visual calibration: ``rigs/<version>/<name>/config.yaml``
(design values and solver settings) in the repo, and the rig's own files
(``calibration.yaml``: what the solver found; ``turntable_calibration.yaml``)
in its folder under ORCA_HOME (rig_home). Plan: ``README.md``."""

import logging
import math
import os
from dataclasses import dataclass, field

from orca_core.utils.utils import _version_sort_key, read_yaml, write_yaml_atomic

from visual_calibration import rig_home

logger = logging.getLogger(__name__)

DEFAULT_RIG_NAME = "camera-rig"
DEFAULT_CAMERA = "a"            # the old single `camera:` block; holds the gauge (its CAD azimuth)
DEFAULT_TOWER = "v1"            # the `calibration_object:` block as written
ANGLE_TOLERANCE_DEG = 1e-6
CALIBRATION_FILE = "calibration.yaml"
TURNTABLE_CALIBRATION_FILE = "turntable_calibration.yaml"
_warned_left: set = set()


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
    home: str | None = None         # the rig's own files (rig_home); None: the rig's id is not known

    @property
    def version(self) -> str:
        return os.path.basename(os.path.dirname(self.path))

    @property
    def name(self) -> str:
        return os.path.basename(self.path)

    @property
    def cameras(self) -> dict[str, dict]:
        """Every camera's block by name (`cameras:`); the old single `camera:` block is camera "a"."""
        return dict(self.raw.get("cameras") or {DEFAULT_CAMERA: self.raw.get("camera") or {}})

    @property
    def active_cameras(self) -> list[str]:
        """The cameras a sweep grabs (`enabled: false` keeps one configured but unplugged)."""
        return [k for k, c in self.cameras.items() if c.get("enabled", True)]

    def nominal_of(self, cam: str = DEFAULT_CAMERA) -> dict | None:
        """A camera's CAD pose {position, look_at} (rig frame, mm) from its mount, or its nominal_pose if set.
        mount.azimuth (deg, default 0: +x) turns the mount about the axis."""
        c = self.cameras.get(cam) or {}
        if c.get("nominal_pose"):
            return c["nominal_pose"]
        m = c.get("mount")
        if not m:
            return None
        dx, dz = m.get("lens_offset", [0.0, 0.0])
        x, z, tilt = float(m["distance"]) + dx, float(m["height"]) + dz, math.radians(float(m["tilt"]))
        az = float(m.get("azimuth", 0.0))
        if not az:
            return {"position": [round(x, 3), 0.0, round(z, 3)], "look_at": [0.0, 0.0, round(z - x * math.tan(tilt), 3)]}
        a = math.radians(az)
        return {"position": [round(x * math.cos(a), 3), round(x * math.sin(a), 3), round(z, 3)],
                "look_at": [0.0, 0.0, round(z - x * math.tan(tilt), 3)]}

    @property
    def camera_nominal(self) -> dict | None:
        """Camera "a"'s CAD pose (nominal_of)."""
        return self.nominal_of(DEFAULT_CAMERA)

    def camera_azimuth(self, cam: str = DEFAULT_CAMERA) -> float:
        """The camera's CAD azimuth about the axis, deg (0 without a mount)."""
        n = self.nominal_of(cam)
        return float(math.degrees(math.atan2(n["position"][1], n["position"][0]))) if n else 0.0

    @property
    def search(self) -> dict:
        return self.raw["solver"]["search"]

    @classmethod
    def load(cls, name: str = DEFAULT_RIG_NAME, version: str | None = None, path: str | None = None,
             rig_id: str | None = None) -> "RigConfig":
        """The rig's config from the repo, its own files from its folder under ORCA_HOME: that of
        `rig_id`, else of the board turntable.yaml pins. A rig loaded from a folder of its own
        (`path`: a copy for a test, or of a past calibration) keeps its files there."""
        # An explicit path is a self-contained rig folder: its calibration files are read and written
        # there, never in ORCA_HOME (tests' copies, estimate.rig_at's past calibrations).
        home = path
        path = path or get_rig_path(name, version)
        raw = read_yaml(os.path.join(path, "config.yaml"))
        if not raw:
            raise RigConfigError(f"{path}/config.yaml is missing or empty")
        if raw.get("cameras") and "camera" not in raw:
            raw["camera"] = raw["cameras"][DEFAULT_CAMERA]       # what single-camera readers use
        tower = raw.get("tower", DEFAULT_TOWER)
        if tower != DEFAULT_TOWER:
            seats = (raw.get("towers") or {}).get(tower)
            if seats is None:
                raise RigConfigError(f"{path}/config.yaml: tower {tower!r} is not under towers:")
            raw["calibration_object"] = {**raw["calibration_object"], **seats}
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
        config.home = home or config._home(rig_id)
        return config

    def _home(self, rig_id: str | None) -> str | None:
        try:
            return str(rig_home.rig_dir(rig_id or rig_home.rig_id((self.raw["turntable"].get("board") or {}).get("usb_serial"))))
        except rig_home.RigIdError:
            return None

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
        if DEFAULT_CAMERA not in self.cameras:
            raise RigConfigError(f"cameras: camera {DEFAULT_CAMERA!r} is missing (it holds the gauge)")
        for name, cam in self.cameras.items():
            mount = cam.get("mount") or {}
            where = "camera" if "cameras" not in self.raw else f"cameras.{name}"
            for key, o in (mount.get("options") or {}).items():
                v = float(mount[key])
                k = (v - o["min"]) / o["step"]
                if not o["min"] <= v <= o["max"] or abs(k - round(k)) > 1e-6:
                    raise RigConfigError(f"{where}.mount.{key} {v:g} is not a hole: {o['min']:g}..{o['max']:g} in steps of {o['step']:g}")

    def own_file(self, name: str) -> str:
        """Where the rig's file `name` is written: its folder under ORCA_HOME."""
        if self.home is None:
            raise RigConfigError("the rig has no id: pin its board's usb_serial in turntable.yaml, or connect the rig")
        return os.path.join(self.home, name)

    def _in_force(self, name: str) -> str:
        """The rig's file `name` to read: its own; until it has one, a file left in the checkout
        (read with a warning, never moved)."""
        left = os.path.join(self.path, name)
        own = os.path.join(self.home, name) if self.home else None
        if own and (os.path.exists(own) or not os.path.exists(left)):
            return own
        if not os.path.exists(left):
            return self.own_file(name)
        if left not in _warned_left:
            _warned_left.add(left)
            logger.warning("reading the rig's %s from the checkout (%s); it belongs in %s: move it there",
                           name, left, own or "the rig's folder under ORCA_HOME")
        return left

    @property
    def calibration_path(self) -> str:
        return self._in_force(CALIBRATION_FILE)

    @property
    def turntable_calibration_path(self) -> str:
        return self._in_force(TURNTABLE_CALIBRATION_FILE)

    def load_calibration(self) -> dict:
        return read_yaml(self.calibration_path) or {}

    def save_calibration(self, calibration: dict, keep_history: bool = True) -> None:
        """Write the solver's result; the previous run goes to `history`."""
        calibration = dict(calibration)
        if keep_history:
            old = self.load_calibration()
            history = list(old.get("history") or [])
            if old.get("calibrated_at"):
                history.append({k: old.get(k) for k in ("calibrated_at", "lens", "camera", "axis", "tower", "mounting", "stop_angle_corrections", "tile_poses", "fit")}
                               | ({"cameras": old["cameras"]} if "cameras" in old else {}))
            calibration["history"] = history
        path = self.own_file(CALIBRATION_FILE)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        write_yaml_atomic(path, calibration)
