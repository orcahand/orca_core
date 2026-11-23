"""Step 1 of visual calibration: the rig without the hand (README.md), in two parts.

1a camera (`calibrate_camera`): photos of the calibration object, each with its
   own free object pose (the plate angle is not used) -> the lens and each
   tile's pose on the object.
1b rotation (`calibrate_rig` with the camera result held): photos at the motor's
   stops -> the camera pose in the rig, the axis and every stop's angle.
`calibrate_rig` alone also fits everything at once:
- the lens (fx, fy, cx, cy + distortion),
- the camera pose in the rig frame (the axis frame at stop 0),
- the axis: how the object (the hand's connector) sits on the true rotation
  axis, tilt about x/y and offset in x/y,
- each stop's angle correction (the stop nearest 0°, the hard stop, pinned),
- each tile's pose on the object (the reference tile held at the CAD).

A point X on the object (object frame: the tiles' CAD seats) is at
Rz(stop + correction) @ M @ Rz(mount + yaw) @ X in the rig frame, M = offset @
Rx(tilt_x) @ Ry(tilt_y): the tower's pose on the turning axis. Tilt and xy
offset are solved (the turn makes them observable; their start comes from the
circle the camera draws around the tower). The mount (the tower's turn on the
plate) is found from the photos (visual_calibration.mounting) and snapped to the
connector's mountings. The small yaw around it is not observable: it trades
exactly with the camera's turn about the axis (nothing in the photos shows the
hard stop). It is held during the fit and set afterwards so the camera sits at
its CAD azimuth (a turn of the whole solution about the axis: pixels unchanged). The tower's height on the axis is held at the CAD: it trades
exactly with the camera's height (a tower sitting high shows up as the camera
that much lower; step 2's hand mount takes it up). Lengths mm, angles deg, pixels in the raw camera image.

Several cameras (Photo.camera, "a" by default): each has its own lens and pose; the object,
the axis, the tower on it, the stops and the tiles are shared. Camera "a" (else the first one
seen) holds the gauge: the solution is turned so it sits at its CAD azimuth. With photos of
one camera only, every number is as it was before cameras were added.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from datetime import datetime

import numpy as np
from orca_core.kinematics.transforms import Transform, rotation_about_axis

from visual_calibration.calibration_object import CalibrationObject
from visual_calibration.camera import Lens, nominal_lens
from visual_calibration.detect import TileDetection, _cv2
from visual_calibration.mounting import MOUNTINGS, Mounting, mounting_from_cameras, wrap
from visual_calibration.rig_config import DEFAULT_CAMERA
from visual_calibration.solve import least_squares

DISTORTION = ("k1", "k2", "p1", "p2", "k3")
DEFAULT_SEARCH = {
    "tower": {"tilt": {"bound": [5.0, 5.0], "sigma": [2.0, 2.0]},
              "offset": {"bound": [10.0, 10.0], "sigma": [5.0, 5.0]},
              "yaw": {"bound": 10.0, "sigma": 1.0}},
    "stop_angle": {"bound": 1.5, "sigma": 0.5},
    "tile": {"translation": {"bound": [1.0] * 3, "sigma": [0.3] * 3},
             "rotation": {"bound": [1.0] * 3, "sigma": [0.3] * 3}},
}
HAND_TURNED = {"bound": 30.0, "sigma": 10.0}   # stops set by hand: the angle is nearly free


@dataclass(frozen=True)
class Photo:
    stop_deg: float
    detections: Sequence[TileDetection]
    name: str = ""
    camera: str = DEFAULT_CAMERA


def camera_order(photos: Sequence[Photo]) -> list[str]:
    """The cameras in the photos, the gauge camera ("a", else the first) first."""
    return sorted({p.camera for p in photos}, key=lambda c: (c != DEFAULT_CAMERA, c))


def _photo_name(p: Photo) -> str:
    return p.name or f"{p.stop_deg:g}" + ("" if p.camera == DEFAULT_CAMERA else f"_{p.camera}")


def _cameras_dict(lenses, rms, points, poses=None) -> dict:
    """The `cameras:` block of calibration.yaml: lens (+ pose) and fit of every camera but "a"
    (camera "a" stays at the top level, the single-camera layout)."""
    r3 = lambda v: [round(float(x), 3) for x in v]
    return {c: {"lens": _lens_dict(L, rms[c]),
                **({"camera": {"translation": r3(poses[c].translation), "rotation": r3(rpy_deg(poses[c].rotation))}}
                   if poses else {}),
                "fit": {"reprojection_px": round(rms[c], 3), "points": points[c]}}
            for c, L in lenses.items() if c != DEFAULT_CAMERA}


@dataclass
class CameraCalibration:
    lenses: dict[str, Lens]
    tile_poses: dict[str, Transform]
    photo_poses: dict[str, Transform]    # camera in the object frame, per photo
    rms_px: float
    photo_rms: dict[str, float]
    tile_rms: dict[str, float]
    points: int
    std: dict[str, list[float]] = field(default_factory=dict)
    iterations: int = 0
    tile_scales: dict[str, float] = field(default_factory=dict)    # printed size / design, when solved
    camera_rms: dict[str, float] = field(default_factory=dict)
    camera_points: dict[str, int] = field(default_factory=dict)

    @property
    def lens(self) -> Lens:
        """The gauge camera's lens (camera "a" when it was in the photos)."""
        return next(iter(self.lenses.values()))

    def to_calibration(self) -> dict:
        r3 = lambda v: [round(float(x), 3) for x in v]
        return plain({
            "calibrated_at": datetime.now().isoformat(timespec="seconds"),
            **({"lens": _lens_dict(self.lenses[DEFAULT_CAMERA], self.rms_px)} if DEFAULT_CAMERA in self.lenses else {}),
            **({"cameras": _cameras_dict(self.lenses, self.camera_rms, self.camera_points)}
               if list(self.lenses) != [DEFAULT_CAMERA] else {}),
            "tile_poses": {k: {"corner": r3(t.translation), "rpy": r3(rpy_deg(t.rotation))}
                           for k, t in self.tile_poses.items()},
            **({"tile_scales": {k: round(float(v), 5) for k, v in self.tile_scales.items()}} if self.tile_scales else {}),
            "fit": {"reprojection_px": round(self.rms_px, 3), "points": self.points, "photos": len(self.photo_rms)},
        })


def _lens_dict(L: Lens, rms_px: float) -> dict:
    return {"image_size": list(L.size), "fx": round(L.fx, 3), "fy": round(L.fy, 3), "cx": round(L.cx, 3),
            "cy": round(L.cy, 3), "distortion": [round(float(d), 6) for d in L.distortion],
            "reprojection_px": round(rms_px, 3), "source": "calibration_object"}


def _robust(r: np.ndarray, scale: float) -> np.ndarray:
    """soft_l1 by reweighting: rows of r (pixels) scaled so their square is the soft_l1 cost."""
    e = np.linalg.norm(r, axis=1, keepdims=True) / scale
    return r * np.sqrt(2 * (np.sqrt(1 + e ** 2) - 1)) / np.maximum(e, 1e-12)


@dataclass
class RigCalibration:
    lenses: dict[str, Lens]              # per camera, the gauge camera first
    cameras: dict[str, Transform]        # each camera in the rig frame, mm
    axis_tilt: np.ndarray                # deg, the tower on the axis, about x and y
    axis_offset: np.ndarray              # mm, x and y
    tower_yaw: float                     # deg, object -> plate: the mounting + the solved yaw
    stop_corrections: dict[float, float]  # nominal stop -> deg
    tile_poses: dict[str, Transform]
    rms_px: float
    photo_rms: dict[str, float]
    tile_rms: dict[str, float]
    points: int
    std: dict[str, list[float]] = field(default_factory=dict)
    iterations: int = 0
    mounting: Mounting | None = None
    camera_rms: dict[str, float] = field(default_factory=dict)
    camera_points: dict[str, int] = field(default_factory=dict)

    @property
    def lens(self) -> Lens:
        """The gauge camera's lens (camera "a" when it was in the photos)."""
        return next(iter(self.lenses.values()))

    @property
    def camera(self) -> Transform:
        """The gauge camera's pose (camera "a" when it was in the photos)."""
        return next(iter(self.cameras.values()))

    def to_calibration(self) -> dict:
        """The calibration.yaml layout (rig_config.RigConfig.save_calibration). The top-level lens
        and camera are camera "a"'s; every other camera's are under `cameras:`."""
        r3 = lambda v: [round(float(x), 3) for x in v]
        a = self.cameras.get(DEFAULT_CAMERA)
        return plain({
            "calibrated_at": datetime.now().isoformat(timespec="seconds"),
            **({"lens": _lens_dict(self.lenses[DEFAULT_CAMERA], self.rms_px),
                "camera": {"translation": r3(a.translation), "rotation": r3(rpy_deg(a.rotation))}} if a is not None else {}),
            **({"cameras": _cameras_dict(self.lenses, self.camera_rms, self.camera_points, self.cameras)}
               if list(self.cameras) != [DEFAULT_CAMERA] else {}),
            "axis": {"tilt": r3(self.axis_tilt), "offset": r3(self.axis_offset)},
            "tower": {"tilt": r3(self.axis_tilt), "offset": r3(self.axis_offset), "yaw": round(float(self.tower_yaw), 3),
                      "yaw_from_mounting": round(float(wrap(self.tower_yaw - (self.mounting.yaw if self.mounting else 0.0))), 3),
                      "height": "held at the CAD (trades with the camera height)"},
            "stop_angle_corrections": {float(k): round(float(v), 4) for k, v in self.stop_corrections.items()},
            "tile_poses": {k: {"corner": r3(t.translation), "rpy": r3(rpy_deg(t.rotation))}
                           for k, t in self.tile_poses.items()},
            "fit": {"reprojection_px": round(self.rms_px, 3), "axis_wobble": None,
                    "stops_used": len(self.stop_corrections), "points": self.points},
            **({"mounting": self.mounting.as_dict()} if self.mounting else {}),
        })


def plain(x):
    """Numpy scalars and arrays inside dicts/lists -> plain Python, for YAML/JSON."""
    if isinstance(x, dict):
        return {plain(k): plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple, np.ndarray)):
        return [plain(v) for v in x]
    return x.item() if isinstance(x, np.generic) else x


def rpy_deg(R: np.ndarray) -> np.ndarray:
    """Fixed-axis roll, pitch, yaw (R = Rz Ry Rx), degrees."""
    return np.degrees([np.arctan2(R[2, 1], R[2, 2]), -np.arcsin(np.clip(R[2, 0], -1, 1)), np.arctan2(R[1, 0], R[0, 0])])


def _rot(v_deg: np.ndarray) -> np.ndarray:
    a = np.linalg.norm(v_deg)
    return np.eye(3) if a < 1e-12 else rotation_about_axis(v_deg / a, np.deg2rad(a))


def _rz(a_deg: np.ndarray) -> np.ndarray:
    a = np.deg2rad(a_deg)
    c, s = np.cos(a), np.sin(a)
    R = np.zeros((len(a), 3, 3))
    R[:, 0, 0], R[:, 0, 1], R[:, 1, 0], R[:, 1, 1], R[:, 2, 2] = c, -s, s, c, 1.0
    return R


def _project(lens: np.ndarray, model: Sequence[str], P: np.ndarray) -> np.ndarray:
    fx, fy, cx, cy = lens[:4]
    d = dict(zip(model, lens[4:]))
    k1, k2, p1, p2, k3 = (d.get(k, 0.0) for k in DISTORTION)
    x, y = P[:, 0] / P[:, 2], P[:, 1] / P[:, 2]
    r2 = x * x + y * y
    radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.column_stack([fx * xd + cx, fy * yd + cy])


def _pnp(lens: Lens, X: np.ndarray, uv: np.ndarray):
    """Camera pose in the object's frame from one photo, and its pixel RMS."""
    cv2 = _cv2()
    K = np.array([[lens.fx, 0, lens.cx], [0, lens.fy, lens.cy], [0, 0, 1.0]])
    dist = np.array(lens.distortion, float)
    _, rv, tv = cv2.solvePnP(X, uv, K, dist, flags=cv2.SOLVEPNP_SQPNP)
    _, rv, tv = cv2.solvePnP(X, uv, K, dist, rv, tv, True, flags=cv2.SOLVEPNP_ITERATIVE)
    R = cv2.Rodrigues(rv)[0]
    err = cv2.projectPoints(X, rv, tv, K, dist)[0].reshape(-1, 2) - uv
    return Transform.from_rotation_translation(R.T, (-R.T @ tv).ravel()), float(np.sqrt((err ** 2).sum(1).mean()))


def _points(obj: CalibrationObject, photo: Photo):
    X = np.vstack([obj.corners_plate(d.tile, d.ids) for d in photo.detections])
    uv = np.vstack([d.uv for d in photo.detections])
    return X, uv


def initial_lens(obj: CalibrationObject, photos: Sequence[Photo], size: tuple[int, int]) -> Lens:
    """Nominal lens with the focal length that best fits the photo with most corners."""
    best = max(photos, key=lambda p: sum(len(d.ids) for d in p.detections))
    X, uv = _points(obj, best)
    base = nominal_lens(size)
    trials = [Lens(base.fx * s, base.fy * s, base.cx, base.cy, size) for s in np.linspace(0.6, 1.6, 41)]
    return min(trials, key=lambda L: _pnp(L, X, uv)[1])


def calibrate_rig(obj: CalibrationObject, photos: Sequence[Photo], size: tuple[int, int],
                  search: Mapping | None = None, lens: Lens | None = None,
                  distortion: Sequence[str] = DISTORTION, hand_turned: bool = False,
                  loss_scale_px: float = 1.0, max_iter: int = 60, fixed_lens: bool = False,
                  tile_poses: Mapping[str, Transform] | None = None,
                  camera_prior: Mapping | None = None, mount_yaw: float | None = None,
                  mountings: Sequence[float] | None = MOUNTINGS,
                  lenses: Mapping[str, Lens] | None = None,
                  camera_priors: Mapping[str, Mapping] | None = None) -> RigCalibration:
    """fixed_lens: hold `lens` (step 1b); tile_poses: hold the tiles there (from step 1a);
    camera_prior: where the CAD puts the camera (`nominal_camera`): the start must be
    near it, and the fit is pulled toward it; mount_yaw: the object's turn on the plate
    (deg), else found from the photos and snapped to `mountings` when near one.
    lenses / camera_priors: the same per camera (Photo.camera); `lens` and `camera_prior`
    are camera "a"'s."""
    lenses, priors = dict(lenses or {}), dict(camera_priors or {})
    if lens is not None:
        lenses.setdefault(DEFAULT_CAMERA, lens)
    if camera_prior is not None:
        priors.setdefault(DEFAULT_CAMERA, camera_prior)
    search = {**DEFAULT_SEARCH, **(search or {})}
    photos = [p for p in photos if sum(len(d.ids) for d in p.detections) >= 6]
    cams = camera_order(photos)
    ref = cams[0] if cams else DEFAULT_CAMERA
    if sum(p.camera == ref for p in photos) < 3:
        raise ValueError("need at least 3 photos with 6+ tile corners each")
    if fixed_lens and any(c not in lenses for c in cams):
        raise ValueError("fixed_lens needs a lens")
    L = {c: lenses.get(c) or initial_lens(obj, [p for p in photos if p.camera == c], size) for c in cams}
    model = [d for d in DISTORTION if d in distortion]
    of_ref = [i for i, p in enumerate(photos) if p.camera == ref]

    # Start: the gauge camera from its photo nearest the hard stop, every stop's
    # correction from its own pose against it; each other camera from its best photo.
    poses = [_pnp(L[p.camera], *_points(obj, p))[0] for p in photos]
    eye = np.asarray(priors[ref]["position"], float) if ref in priors else np.array([1.0, 0.0, 0.0])
    mounting = mounting_from_cameras(obj, [photos[i].stop_deg for i in of_ref], [poses[i].translation for i in of_ref],
                                     float(np.degrees(np.arctan2(eye[1], eye[0]))), mountings)
    if mounting.direction_known and mounting.direction < 0 and not hand_turned:
        raise ValueError(mounting.warnings()[0])
    yaw = mounting.yaw if mount_yaw is None else float(mount_yaw)
    Rm = _rz(np.array([yaw]))[0]
    poses = [Transform.from_rotation_translation(Rm, np.zeros(3)) @ c for c in poses]    # object -> plate frame
    stops = sorted({p.stop_deg for p in photos})
    ref_stops = sorted({photos[i].stop_deg for i in of_ref})
    tw = search["tower"]
    tilt0, off0 = _tower_start(np.array([poses[i].translation for i in of_ref]), ref_stops, tw)
    T0 = Transform.from_rotation_translation(_tilt(tilt0), np.array([*off0, 0.0]))
    poses = [T0 @ c for c in poses]                                                      # -> on the axis
    pinned = min(ref_stops, key=abs)
    k0 = next(i for i in of_ref if photos[i].stop_deg == pinned)
    cam0 = {ref: Transform.from_rotation_translation(_rz(np.array([pinned]))[0], np.zeros(3)) @ poses[k0]}

    def turn(s, at):
        rel = [cam0[photos[i].camera].rotation @ (_rz(np.array([s]))[0] @ poses[i].rotation).T for i in at]
        return float(np.degrees(np.mean([np.arctan2(R[1, 0], R[0, 0]) for R in rel])))

    corr0 = {}
    for s in stops:
        at = [i for i in of_ref if photos[i].stop_deg == s]
        if at:
            corr0[s] = 0.0 if s == pinned else turn(s, at)
    for c in cams[1:]:
        best = max((i for i, p in enumerate(photos) if p.camera == c and p.stop_deg in corr0),
                   key=lambda i: sum(len(d.ids) for d in photos[i].detections), default=None)
        if best is None:
            raise ValueError(f"camera {c} has no photo at a stop camera {ref} saw")
        s = photos[best].stop_deg
        cam0[c] = Transform.from_rotation_translation(_rz(np.array([s + corr0[s]]))[0], np.zeros(3)) @ poses[best]
    for s in stops:
        if s not in corr0:
            corr0[s] = turn(s, [i for i, p in enumerate(photos) if p.stop_deg == s])
    prior = {c: nominal_camera_check(cam0[c], priors[c]) for c in cams if c in priors}
    free_stops = [s for s in stops if s != pinned]
    tiles = sorted({d.tile for p in photos for d in p.detections})
    free_tiles = [] if tile_poses else [t for t in tiles if t != obj.reference]

    # Observations, flat.
    P, ti, pi, uv = [], [], [], []
    for k, p in enumerate(photos):
        for d in p.detections:
            P.append(obj.spec.corners[d.ids])
            ti += [tiles.index(d.tile)] * len(d.ids)
            pi += [k] * len(d.ids)
            uv.append(d.uv)
    P, ti, pi, uv = np.vstack(P), np.array(ti), np.array(pi), np.vstack(uv)
    ci = np.array([cams.index(photos[k].camera) for k in pi])
    stop_of = np.array([p.stop_deg for p in photos])
    cad = [(tile_poses or {}).get(t, obj.tiles[t].pose) for t in tiles]

    # Parameters: lens | camera rot(3, deg) trans(3) | tower tilt(2) offset(2) yaw(1) | stops | tiles 6 each
    # | per other camera: lens, rot(3), trans(3).
    n_lens = 4 + len(model)
    i_cam, i_axis, i_stop = n_lens, n_lens + 6, n_lens + 11
    i_tile = i_stop + len(free_stops)
    i_more = i_tile + 6 * len(free_tiles)
    at_lens = {c: 0 if k == 0 else i_more + (k - 1) * (n_lens + 6) for k, c in enumerate(cams)}
    at_cam = {c: i + n_lens for c, i in at_lens.items()}
    x0 = np.concatenate([[L[ref].fx, L[ref].fy, L[ref].cx, L[ref].cy], _dist0(L[ref], model), np.zeros(6), tilt0, off0, [0.0],
                         [corr0[s] for s in free_stops], np.zeros(6 * len(free_tiles)),
                         *[np.r_[L[c].fx, L[c].fy, L[c].cx, L[c].cy, _dist0(L[c], model), np.zeros(6)] for c in cams[1:]]])
    n_more = x0.size - i_more
    st, tl = (HAND_TURNED if hand_turned else search["stop_angle"]), search["tile"]
    yb = tw.get("yaw", DEFAULT_SEARCH["tower"]["yaw"])
    bound = np.concatenate([np.full(n_lens + 6, np.inf), tw["tilt"]["bound"], tw["offset"]["bound"], _as1(yb["bound"]),
                            np.full(len(free_stops), st["bound"]),
                            np.tile(np.concatenate([tl["rotation"]["bound"], tl["translation"]["bound"]]), len(free_tiles)),
                            np.full(n_more, np.inf)])
    sigma = np.concatenate([np.full(n_lens + 6, np.inf), tw["tilt"]["sigma"], tw["offset"]["sigma"], _as1(yb["sigma"]),
                            np.full(len(free_stops), st["sigma"]),
                            np.tile(np.concatenate([tl["rotation"]["sigma"], tl["translation"]["sigma"]]), len(free_tiles)),
                            np.full(n_more, np.inf)])
    prior_at = x0.copy()
    prior_at[i_axis:i_axis + 5] = 0.0                 # the CAD: upright, centred, at the mounting
    prior_at[i_stop:i_tile] = 0.0 if not hand_turned else x0[i_stop:i_tile]
    lo = np.where(np.isinf(bound), -np.inf, prior_at - bound)
    hi = np.where(np.isinf(bound), np.inf, prior_at + bound)
    has_prior = np.isfinite(sigma)

    def unpack(x):
        Rt = np.empty((len(tiles), 3, 3))
        tt = np.empty((len(tiles), 3))
        for j, t in enumerate(tiles):
            R, tr = cad[j].rotation, cad[j].translation
            if t in free_tiles:
                q = x[i_tile + 6 * free_tiles.index(t):][:6]
                Rt[j], tt[j] = R @ _rot(q[:3]), tr + R @ q[3:]
            else:
                Rt[j], tt[j] = R, tr
        corr = {pinned: 0.0, **dict(zip(free_stops, x[i_stop:i_tile]))}
        Rc = {c: cam0[c].rotation @ _rot(x[i:i + 3]) for c, i in at_cam.items()}
        tc = {c: cam0[c].translation + x[i + 3:i + 6] for c, i in at_cam.items()}
        tilt, off = x[i_axis:i_axis + 2], x[i_axis + 2:i_axis + 4]
        M = _tilt(tilt) @ _rz(np.array([yaw + x[i_axis + 4]]))[0]
        return Rt, tt, corr, Rc, tc, M, np.array([off[0], off[1], 0.0])

    def pixels(x):
        Rt, tt, corr, Rc, tc, M, off = unpack(x)
        Xp = np.einsum("nij,nj->ni", Rt[ti], P) + tt[ti]
        Xa = Xp @ M.T + off
        Rz = _rz(stop_of + np.array([corr[s] for s in stop_of]))
        Xr = np.einsum("nij,nj->ni", Rz[pi], Xa)
        if len(cams) == 1:
            return _project(x[:n_lens], model, (Xr - tc[ref]) @ Rc[ref])
        out = np.empty((len(Xr), 2))
        for k, c in enumerate(cams):
            m = ci == k
            out[m] = _project(x[at_lens[c]:at_lens[c] + n_lens], model, (Xr[m] - tc[c]) @ Rc[c])
        return out

    def residual(x):
        rows = [_robust(pixels(x) - uv, loss_scale_px).ravel(), (x - prior_at)[has_prior] / sigma[has_prior]]
        for c, (pos, sig) in prior.items():
            rows.append((cam0[c].translation + x[at_cam[c] + 3:at_cam[c] + 6] - pos) / sig)
        return np.concatenate(rows)

    steps = np.full(x0.size, 1e-4)
    free = np.ones(x0.size, bool)
    for i in at_lens.values():
        steps[i:i + 4] = 1e-3
        free[i:i + n_lens] = not fixed_lens
    free[i_axis + 4] = False                          # the yaw gauge: set after the fit

    def full(z):
        x = x0.copy()
        x[free] = z
        return x
    fit = least_squares(lambda z: residual(full(z)), x0[free], lo[free], hi[free], step=steps[free], max_iter=max_iter)

    x = full(fit.x)
    fstd = np.zeros(x0.size)
    fstd[free] = fit.std
    Rt, tt, corr, Rc, tc, M, off = unpack(x)
    err = np.linalg.norm(pixels(x) - uv, axis=1)
    # The gauge: turn everything about the axis so the gauge camera is at its CAD azimuth.
    yb = float(_as1(tw.get("yaw", DEFAULT_SEARCH["tower"]["yaw"])["bound"])[0])
    alpha = float(np.clip(wrap(np.degrees(np.arctan2(eye[1], eye[0]) - np.arctan2(tc[ref][1], tc[ref][0]))), -yb, yb))
    G = _rz(np.array([alpha]))[0]
    Rc, tc, off = {c: G @ R for c, R in Rc.items()}, {c: G @ t for c, t in tc.items()}, G @ off
    Q = G @ M                                         # = Rx(a) Ry(b) Rz(yaw)
    tilt_out = np.degrees([np.arctan2(-Q[1, 2], Q[2, 2]), np.arcsin(np.clip(Q[0, 2], -1, 1))])
    yaw_out = float(np.degrees(np.arctan2(-Q[0, 1], Q[0, 0])) % 360.0)
    rms = lambda m: float(np.sqrt((err[m] ** 2).mean()))

    def lens_of(c):
        v = x[at_lens[c]:at_lens[c] + n_lens]
        d = dict(zip(model, v[4:]))
        return Lens(*v[:4], size, tuple(float(d.get(k, 0.0)) for k in DISTORTION))

    std = {"lens_fx_fy_cx_cy": fstd[:4].round(3).tolist(),
           "camera_translation": fstd[i_cam + 3:i_cam + 6].round(4).tolist(),
           "camera_rotation": fstd[i_cam:i_cam + 3].round(4).tolist(),
           "axis_tilt": fstd[i_axis:i_axis + 2].round(4).tolist(),
           "axis_offset": fstd[i_axis + 2:i_axis + 4].round(4).tolist()}
    for c in cams[1:]:
        std[f"camera_{c}"] = {"lens_fx_fy_cx_cy": fstd[at_lens[c]:at_lens[c] + 4].round(3).tolist(),
                              "translation": fstd[at_cam[c] + 3:at_cam[c] + 6].round(4).tolist(),
                              "rotation": fstd[at_cam[c]:at_cam[c] + 3].round(4).tolist()}
    return RigCalibration(
        lenses={c: lens_of(c) for c in cams},
        cameras={c: Transform.from_rotation_translation(Rc[c], tc[c]) for c in cams},
        axis_tilt=tilt_out, axis_offset=off[:2].copy(), tower_yaw=yaw_out,
        stop_corrections={float(s): float(c) for s, c in sorted(corr.items())},
        tile_poses={t: Transform.from_rotation_translation(Rt[j], tt[j]) for j, t in enumerate(tiles)},
        rms_px=rms(np.ones_like(err, bool)),
        photo_rms={_photo_name(p): rms(pi == k) for k, p in enumerate(photos)},
        tile_rms={t: rms(ti == j) for j, t in enumerate(tiles)},
        points=len(err),
        std=std,
        iterations=fit.iterations,
        mounting=mounting if mount_yaw is None else mounting.with_yaw(yaw),
        camera_rms={c: rms(ci == k) for k, c in enumerate(cams)},
        camera_points={c: int((ci == k).sum()) for k, c in enumerate(cams)},
    )


def calibrate_camera(obj: CalibrationObject, photos: Sequence[Photo], size: tuple[int, int],
                     search: Mapping | None = None, lens: Lens | None = None,
                     distortion: Sequence[str] = DISTORTION, loss_scale_px: float = 1.0,
                     max_iter: int = 60, lenses: Mapping[str, Lens] | None = None) -> CameraCalibration:
    """Step 1a: the lens and the tiles' poses on the object; every photo has its own
    free object pose, so neither the plate angles nor the axis enter. With
    search.tile.scale, each tile's print scale too (about its centre; the
    reference tile holds the overall scale). Several cameras (Photo.camera): a lens
    each (`lenses` to start from; `lens` is camera "a"'s), the tiles shared."""
    tl = {**DEFAULT_SEARCH, **(search or {})}["tile"]
    sc = tl.get("scale")
    photos = [p for p in photos if sum(len(d.ids) for d in p.detections) >= 6]
    cams = camera_order(photos)
    if any(sum(p.camera == c for p in photos) < 3 for c in cams) or not cams:
        raise ValueError("need at least 3 photos with 6+ tile corners each")
    lenses = dict(lenses or {})
    if lens is not None:
        lenses.setdefault(DEFAULT_CAMERA, lens)
    L = {c: lenses.get(c) or initial_lens(obj, [p for p in photos if p.camera == c], size) for c in cams}
    lens = L[cams[0]]
    model = [d for d in DISTORTION if d in distortion]
    start = [_pnp(L[p.camera], *_points(obj, p))[0] for p in photos]
    tiles = sorted({d.tile for p in photos for d in p.detections})
    free_tiles = [t for t in tiles if t != obj.reference]

    P, ti, pi, uv = [], [], [], []
    for k, p in enumerate(photos):
        for d in p.detections:
            P.append(obj.spec.corners[d.ids])
            ti += [tiles.index(d.tile)] * len(d.ids)
            pi += [k] * len(d.ids)
            uv.append(d.uv)
    P, ti, pi, uv = np.vstack(P), np.array(ti), np.array(pi), np.vstack(uv)
    ci = np.array([cams.index(photos[k].camera) for k in pi])
    cad = [obj.tiles[t].pose for t in tiles]
    R0 = np.array([c.rotation for c in start])
    t0 = np.array([c.translation for c in start])

    # Parameters: lens | per photo camera rot(3, deg) trans(3) | tiles 6 each | tile scales | other cameras' lenses.
    n_lens = 4 + len(model)
    i_tile = n_lens + 6 * len(photos)
    i_scale = i_tile + 6 * len(free_tiles)
    n_scale = len(free_tiles) if sc else 0
    at_lens = {c: 0 if k == 0 else i_scale + n_scale + (k - 1) * n_lens for k, c in enumerate(cams)}
    centre = _tile_centre(obj)
    x0 = np.concatenate([[lens.fx, lens.fy, lens.cx, lens.cy], _dist0(lens, model), np.zeros(6 * len(photos)),
                         np.zeros(6 * len(free_tiles) + n_scale),
                         *[np.r_[L[c].fx, L[c].fy, L[c].cx, L[c].cy, _dist0(L[c], model)] for c in cams[1:]]])
    n_more = x0.size - i_scale - n_scale
    tb = np.concatenate([tl["rotation"]["bound"], tl["translation"]["bound"]])
    ts = np.concatenate([tl["rotation"]["sigma"], tl["translation"]["sigma"]])
    bound = np.concatenate([np.full(i_tile, np.inf), np.tile(tb, len(free_tiles)), np.full(n_scale, sc["bound"] if sc else 0),
                            np.full(n_more, np.inf)])
    sigma = np.concatenate([np.full(i_tile, np.inf), np.tile(ts, len(free_tiles)), np.full(n_scale, sc["sigma"] if sc else 0),
                            np.full(n_more, np.inf)])
    lo, hi = np.where(np.isinf(bound), -np.inf, -bound), np.where(np.isinf(bound), np.inf, bound)
    lo[:n_lens], hi[:n_lens] = x0[:n_lens] + lo[:n_lens], x0[:n_lens] + hi[:n_lens]
    has_prior = np.isfinite(sigma)

    def unpack(x):
        Rt = np.array([c.rotation for c in cad])
        tt = np.array([c.translation for c in cad])
        for f, t in enumerate(free_tiles):
            j, q = tiles.index(t), x[i_tile + 6 * f:][:6]
            Rt[j], tt[j] = cad[j].rotation @ _rot(q[:3]), cad[j].translation + cad[j].rotation @ q[3:]
        q = x[n_lens:i_tile].reshape(-1, 6)
        Rc = np.einsum("kij,kjl->kil", R0, np.array([_rot(v) for v in q[:, :3]]))
        return Rt, tt, Rc, t0 + q[:, 3:]

    def scales(x):
        s = np.ones(len(tiles))
        for f, t in enumerate(free_tiles[:n_scale]):
            s[tiles.index(t)] += x[i_scale + f]
        return s

    def pixels(x):
        Rt, tt, Rc, tc = unpack(x)
        Ps = centre + (P - centre) * scales(x)[ti, None] if n_scale else P
        Xp = np.einsum("nij,nj->ni", Rt[ti], Ps) + tt[ti]
        Xc = np.einsum("nji,nj->ni", Rc[pi], Xp - tc[pi])
        if len(cams) == 1:
            return _project(x[:n_lens], model, Xc)
        out = np.empty((len(Xc), 2))
        for k, c in enumerate(cams):
            m = ci == k
            out[m] = _project(x[at_lens[c]:at_lens[c] + n_lens], model, Xc[m])
        return out

    def residual(x):
        return np.concatenate([_robust(pixels(x) - uv, loss_scale_px).ravel(), x[has_prior] / sigma[has_prior]])

    steps = np.full(x0.size, 1e-4)
    for i in at_lens.values():
        steps[i:i + 4] = 1e-3
    fit = least_squares(residual, x0, lo, hi, step=steps, max_iter=max_iter)
    x = fit.x
    Rt, tt, Rc, tc = unpack(x)
    err = np.linalg.norm(pixels(x) - uv, axis=1)
    rms = lambda m: float(np.sqrt((err[m] ** 2).mean()))

    def lens_of(c):
        v = x[at_lens[c]:at_lens[c] + n_lens]
        d = dict(zip(model, v[4:]))
        return Lens(*v[:4], size, tuple(float(d.get(k, 0.0)) for k in DISTORTION))
    names = [_photo_name(p) for p in photos]
    std = {"lens_fx_fy_cx_cy": fit.std[:4].round(3).tolist(), "distortion": fit.std[4:n_lens].round(5).tolist()}
    for c in cams[1:]:
        std[f"lens_{c}"] = {"fx_fy_cx_cy": fit.std[at_lens[c]:at_lens[c] + 4].round(3).tolist(),
                            "distortion": fit.std[at_lens[c] + 4:at_lens[c] + n_lens].round(5).tolist()}
    return CameraCalibration(
        lenses={c: lens_of(c) for c in cams},
        tile_poses={t: Transform.from_rotation_translation(Rt[j], tt[j]) for j, t in enumerate(tiles)},
        photo_poses={n: Transform.from_rotation_translation(Rc[k], tc[k]) for k, n in enumerate(names)},
        rms_px=rms(np.ones_like(err, bool)),
        photo_rms={n: rms(pi == k) for k, n in enumerate(names)},
        tile_rms={t: rms(ti == j) for j, t in enumerate(tiles)},
        points=len(err),
        std=std,
        iterations=fit.iterations,
        tile_scales=dict(zip(tiles, scales(x).tolist())) if n_scale else {},
        camera_rms={c: rms(ci == k) for k, c in enumerate(cams)},
        camera_points={c: int((ci == k).sum()) for k, c in enumerate(cams)},
    )


def _tile_centre(obj: CalibrationObject) -> np.ndarray:
    s = obj.spec
    return np.array([s.margin + s.squares[0] * s.square_size / 2, s.margin + s.squares[1] * s.square_size / 2, 0.0])


def held_out_rms(obj: CalibrationObject, lens: Lens, tile_poses: Mapping[str, Transform], photo: Photo,
                 tile_scales: Mapping[str, float] | None = None) -> float:
    """A photo left out of the fit: its own pose only, lens and tiles held -> pixel RMS."""
    c = _tile_centre(obj)
    X = np.vstack([tile_poses.get(d.tile, obj.tiles[d.tile].pose).apply_to_points(
        c + (obj.spec.corners[d.ids] - c) * (tile_scales or {}).get(d.tile, 1.0)) for d in photo.detections])
    return _pnp(lens, X, np.vstack([d.uv for d in photo.detections]))[1]


def camera_block(c: Mapping, cam: str = DEFAULT_CAMERA) -> Mapping:
    """A camera's part of a result or calibration.yaml ({lens, camera, fit}): the top level for
    camera "a" (the single-camera layout), else its `cameras:` entry. Empty: not calibrated."""
    return c if cam == DEFAULT_CAMERA else (c.get("cameras") or {}).get(cam) or {}


def lens_from_calibration(c: Mapping, cam: str = DEFAULT_CAMERA) -> Lens:
    """The `lens` block of a camera result back into a Lens (camera `cam`'s)."""
    L = camera_block(c, cam)["lens"]
    return Lens(L["fx"], L["fy"], L["cx"], L["cy"], tuple(L["image_size"]), tuple(L["distortion"]))


def camera_from_calibration(c: Mapping, cam: str = DEFAULT_CAMERA) -> Transform | None:
    """Camera `cam`'s pose in the rig frame from calibration.yaml, or None (not solved)."""
    p = camera_block(c, cam).get("camera") or {}
    if p.get("translation") is None or p.get("rotation") is None:
        return None
    return Transform.from_xyz_rpy(p["translation"], np.deg2rad(p["rotation"]))


def tile_poses_from_calibration(c: Mapping) -> dict[str, Transform]:
    return {k: Transform.from_xyz_rpy(v["corner"], np.deg2rad(v["rpy"])) for k, v in (c.get("tile_poses") or {}).items()}


def _as1(v) -> list[float]:
    return [float(v[0] if isinstance(v, (list, tuple)) else v)]


def _tilt(t_deg) -> np.ndarray:
    return rotation_about_axis(np.array([1.0, 0, 0]), np.deg2rad(t_deg[0])) @ \
        rotation_about_axis(np.array([0, 1.0, 0]), np.deg2rad(t_deg[1]))


def _tower_start(cams: np.ndarray, stops: Sequence[float], tower: Mapping) -> tuple[np.ndarray, np.ndarray]:
    """Tilt (deg) and xy offset (mm) of the tower on the axis from the circle the camera
    positions (in the plate frame, one per photo) draw around it; zero without a wide turn."""
    if len(cams) < 4 or np.ptp(stops) < 90.0:
        return np.zeros(2), np.zeros(2)
    c = cams.mean(0)
    n = np.linalg.svd(cams - c)[2][2]
    n = n if n[2] > 0 else -n
    A = np.column_stack([2 * cams[:, :2], np.ones(len(cams))])
    k = np.linalg.lstsq(A, (cams[:, :2] ** 2).sum(1), rcond=None)[0]
    centre = np.array([k[0], k[1], c[2]])
    tilt = np.degrees([np.arcsin(np.clip(n[1], -1, 1)), np.arctan2(-n[0], n[2])])
    tilt = np.clip(tilt, -np.asarray(tower["tilt"]["bound"]), tower["tilt"]["bound"])
    off = -(_tilt(tilt) @ centre)[:2]
    return tilt, np.clip(off, -np.asarray(tower["offset"]["bound"]), tower["offset"]["bound"])


def _dist0(lens: Lens, model: Sequence[str]) -> np.ndarray:
    d = dict(zip(DISTORTION, lens.distortion or ()))
    return np.array([float(d.get(k, 0.0)) for k in model])


def nominal_camera_check(cam0: Transform, prior: Mapping) -> tuple[np.ndarray, float]:
    """The camera found in the first photo against the CAD: the lens within
    `bound_mm` of `position`, looking within `bound_deg` of `look_at`. Returns
    (position, sigma_mm) for the fit's prior."""
    eye, target = np.asarray(prior["position"], float), np.asarray(prior["look_at"], float)
    off = float(np.linalg.norm(cam0.translation - eye))
    view = (target - eye) / np.linalg.norm(target - eye)
    angle = float(np.degrees(np.arccos(np.clip(cam0.rotation[:, 2] @ view, -1, 1))))
    if off > float(prior.get("bound_mm", 30.0)) or angle > float(prior.get("bound_deg", 10.0)):
        raise ValueError(f"the camera is {off:.0f} mm and {angle:.0f}° from where the rig config puts it "
                         f"({eye.round(1).tolist()}): check the camera mount setting in config.yaml")
    return eye, float(prior.get("sigma_mm", 10.0))
