"""Which way the calibration object sits on the plate, found from the photos.

Nothing is assumed about how the tower sits against the hard stop (the Franka
connector bolts to the plate four ways; other mounts may differ), so the plate
angle at which side 0 faces the camera is found. The tiles' poses
in config.yaml define the object frame; the mounting is a turn `yaw` about z,
object -> plate. At plate angle θ a face with outward azimuth `a` (object frame)
points at `d·θ + yaw + a` in the rig frame; it faces the camera when that is the
camera's azimuth. d is the plate's direction: -1 means the motor turns the
other way than gear_ratio says (the homing then ends on the arm's other side and
the motor alone cannot tell).

Two kinds of observation give (θ, a, weight): a detected tile (a = its side's
normal, weight = its corners), or a solved camera position in the object frame
(a = its azimuth). The yaw is their circular mean, snapped to the connector's
mountings when near one.
"""
from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass

import numpy as np

from visual_calibration.calibration_object import CalibrationObject

MOUNTINGS = (0.0, 90.0, 180.0, 270.0)   # the connector's bolt pattern, 4-fold
SNAP_DEG = 15.0
MAX_VIEW_DEG = 60.0          # a tile turned further from the camera is not detected reliably


def wrap(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def side_azimuths(obj: CalibrationObject) -> dict[int, float]:
    """Outward normal azimuth (deg, object frame) of every side with a tile."""
    out = {}
    for side in sorted({t.side for t in obj.tiles.values()}):
        n = sum(-t.pose.rotation[:, 2] for t in obj.tiles.values() if t.side == side)
        out[side] = float(np.degrees(np.arctan2(n[1], n[0])))
    return out


@dataclass(frozen=True)
class Mounting:
    yaw: float                   # object -> plate about z, deg: the value used
    measured: float              # the circular mean before snapping
    side0_stop: float            # plate angle at which side 0 faces the camera, [0, 360)
    direction: int               # +1, or -1: the plate turns the other way (gear_ratio sign)
    direction_known: bool        # the stops spread enough to tell, and one direction fits clearly better
    consistency: float           # 0..1, how well the observations agree (1: all the same yaw)
    snapped: bool
    source: str                  # "tiles" or "camera"

    def as_dict(self) -> dict:
        return {k: (round(float(v), 2) if isinstance(v, float) else v) for k, v in self.__dict__.items()}

    def with_yaw(self, yaw: float, source: str = "given") -> Mounting:
        side0 = float((self.side0_stop + (self.yaw - yaw) * self.direction) % 360.0)
        return Mounting(float(yaw % 360.0), self.measured, side0, self.direction, self.direction_known,
                        self.consistency, True, source)

    def warnings(self) -> list[str]:
        w = []
        if self.direction_known and self.direction < 0:
            w.append("the plate turns the other way than the motor config says: set motor.gear_ratio to "
                     "-1.0 in turntable.yaml and run the motor calibration again")
        if not self.snapped:
            w.append(f"the object sits at {self.measured:.0f}°, not at one of the connector's mountings")
        if self.consistency < 0.7:
            w.append(f"the photos disagree on the mounting (consistency {self.consistency:.2f})")
        return w


def fit_mounting(obs: Iterable[tuple[float, float, float]], side0_azimuth: float, camera_azimuth: float = 0.0,
                 mountings: Sequence[float] | None = MOUNTINGS, snap_deg: float = SNAP_DEG,
                 source: str = "tiles") -> Mounting:
    obs = np.array(list(obs), float).reshape(-1, 3)
    if not len(obs):
        raise ValueError("no tiles seen: nothing to tell the mounting from")
    th, a, w = obs.T

    def mean(d):
        z = (w * np.exp(1j * np.radians(camera_azimuth - d * th - a))).sum() / w.sum()
        return float(np.degrees(np.angle(z))), float(abs(z))

    (y_pos, c_pos), (y_neg, c_neg) = mean(1), mean(-1)
    spread = float(np.ptp(th)) >= 45.0
    direction = 1 if c_pos >= c_neg or not spread else -1
    measured, consistency = (y_pos, c_pos) if direction > 0 else (y_neg, c_neg)
    yaw, snapped = measured, mountings is None
    if mountings:
        near = min(mountings, key=lambda m: abs(wrap(measured - m)))
        if abs(wrap(measured - near)) <= snap_deg:
            yaw, snapped = float(near), True
    side0 = float(((camera_azimuth - yaw - side0_azimuth) * direction) % 360.0)
    return Mounting(float(yaw % 360.0), float(measured % 360.0), side0, direction,
                    spread and abs(c_pos - c_neg) > 0.2, consistency, snapped, source)


def tile_observations(obj: CalibrationObject, photos) -> list[tuple[float, float, float]]:
    az = side_azimuths(obj)
    return [(p.stop_deg, az[obj.tiles[d.tile].side], float(len(d.ids))) for p in photos for d in p.detections]


def mounting_from_tiles(obj: CalibrationObject, photos, camera_azimuth: float = 0.0,
                        mountings: Sequence[float] | None = MOUNTINGS) -> Mounting:
    """From which tiles are seen at which plate angle (no lens or poses needed)."""
    return fit_mounting(tile_observations(obj, photos), side_azimuths(obj)[0], camera_azimuth, mountings)


def mounting_from_cameras(obj: CalibrationObject, stops: Sequence[float], positions: Sequence[np.ndarray],
                          camera_azimuth: float = 0.0, mountings: Sequence[float] | None = MOUNTINGS) -> Mounting:
    """From the camera positions solved per photo in the object frame (PnP)."""
    obs = [(s, float(np.degrees(np.arctan2(p[1], p[0]))), 1.0) for s, p in zip(stops, positions)]
    return fit_mounting(obs, side_azimuths(obj)[0], camera_azimuth, mountings, snap_deg=SNAP_DEG, source="camera")


def facing(obj: CalibrationObject, stops: Sequence[float], yaw: float, camera_azimuth: float = 0.0,
           max_view_deg: float = MAX_VIEW_DEG, direction: int = 1) -> dict[float, list[int]]:
    """The sides facing the camera within max_view_deg at each stop."""
    az = side_azimuths(obj)
    return {float(s): [k for k, a in az.items() if abs(wrap(direction * s + yaw + a - camera_azimuth)) <= max_view_deg]
            for s in stops}


def coverage(obj: CalibrationObject, stops: Sequence[float], mountings: Sequence[float] = MOUNTINGS,
             camera_azimuth: float = 0.0, max_view_deg: float = MAX_VIEW_DEG) -> dict:
    """Over the stops, does every mounting show sides 0 and 2 and at least one of 1/3?
    Checked for the given mountings and, as `any`, the worst yaw in 1° steps."""
    def missing(yaw):
        seen = {k for ks in facing(obj, stops, yaw, camera_azimuth, max_view_deg).values() for k in ks}
        need = [k for k in (0, 2) if k not in seen] + ([] if seen & {1, 3} else ["1 or 3"])
        return need

    out = {"mountings": {float(m): missing(m) for m in mountings}}
    bad = [y for y in range(360) if missing(float(y))]
    out["any"] = {"ok": not bad, "bad_yaws": bad[:10]}
    out["warnings"] = [f"mounted at {m:g}° the stops never show side(s) {', '.join(map(str, v))}"
                       for m, v in out["mountings"].items() if v]
    if bad:
        out["warnings"].append(f"at some mountings (e.g. {bad[0]}°) a side is never seen: add stops")
    return out


def config_mountings(cfg: Mapping) -> list[float]:
    return [float(m) for m in cfg.get("mountings", MOUNTINGS)]
