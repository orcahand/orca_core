"""The hand's pose from the camera alone: its mount and, optionally, every joint.

Per frame, the hand is segmented near where the current estimate puts it
(GrabCut seeded by the model's silhouette) and its edges found (Canny). The hand
model (URDF meshes as dense surface samples, posed by the joint angles) is then
fitted to that, with the applied rig calibration at the plate angle:
- anchor points on every link must fall inside the segmented hand (distance to it);
- the segmented hand must be covered by the model's silhouette (distance to it);
- the image's edges inside the hand must lie on the model's outlines, which are
  rendered depth-ordered (links painted far to near, labelled by group: tower,
  palm, thumb, each finger), so a finger crossing the palm has an outline too.
Unknowns: a small 6-DoF correction of the mount (plate axes, about the hand's
base origin) on top of the found mounting x CAD hand mount, and the joints within
their URDF limits, warm-started from the last frame. Kept views (other plate
angles, same pose) add their residuals.

Observability, per joint: the share of its motion the view sees (across the line
of sight, not hidden), its 1-sigma from the fit's Jacobian at 1 px noise, and for
a finger's two bending joints the spread over a coarse grid of cells that fit
about as well (searched every few frames; a better cell is jumped to). Seen
palm-on, bent and over-stretched look alike, and a base joint trades against the
middle one; finger bending needs views from two plate angles. For joints the view
can't pin, the plate angle that shows them best.
"""
from __future__ import annotations

import base64
import time
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
from orca_core.kinematics.transforms import Transform

from visual_calibration.camera import Lens
from visual_calibration.hand_model import HandModel
from visual_calibration.overlay import camera_model, hand_in_plate, plate_in_rig
from visual_calibration.rig_calibration import _rot
from visual_calibration.solve import least_squares

BOUND = np.full(6, 10.0)                     # mount: rot deg x3, trans mm x3
MOUNT_STEP = np.array([0.3, 0.3, 0.3, 0.6, 0.6, 0.6])
JOINT_STEP = 1.5
FIT_WIDTH = 480                              # the fit's image width (px)
SPACING_MM = 1.2                             # surface sample spacing (rendering)
ANCHORS = 60                                 # anchor points per link (palm, tower: x4)
CAP_PX = 12.0                                # distances are capped (fit px): far misses don't drag
EDGE_CAP_PX = 4.0
W_EDGE = 0.7
N_FG, N_EDGE = 3000, 600                     # samples of the segmented hand and of its edges
ROI_PX = 40
MOUNT_PRIOR = 1.0
HYPER = 0.5                                  # prior against over-extension (px over every residual at -30°)
SIGMA_PX = 1.0
GOOD_DEG, FAIR_DEG = 2.0, 6.0
GOOD_OBS, FAIR_OBS = 0.6, 0.3                # share of a joint's motion the view sees (observability)
MULTI_VIEW_DEG = 40.0                        # kept views this far apart can say something about the bending
TIE = 0.05                                   # grid cells within 5 % of the finger's cost range of the best fit about as well
SPREAD_DEG = 15.0                            # a joint spread wider than this over those: the view can't tell
GRID = {"mcp": (-20.0, 0.0, 20.0, 40.0, 60.0, 80.0), "pip": (-10.0, 10.0, 30.0, 50.0, 70.0, 90.0)}
GRID["dip"] = GRID["pip"]
GROUPS = ("tower", "palm", "thumb", "index", "middle", "ring", "pinky")
FLEX = {"thumb": ("thumb_mcp", "thumb_dip"), "index": ("index_mcp", "index_pip"),
        "middle": ("middle_mcp", "middle_pip"), "ring": ("ring_mcp", "ring_pip"),
        "pinky": ("pinky_mcp", "pinky_pip")}
PLATE_RANGE = (0.0, 315.0)
SUGGEST_EVERY_S = 10.0


def _cv2():
    import cv2
    return cv2


def scaled(lens: Lens, s: float) -> Lens:
    return Lens(lens.fx * s, lens.fy * s, (lens.cx + 0.5) * s - 0.5, (lens.cy + 0.5) * s - 0.5,
                (round(lens.size[0] * s), round(lens.size[1] * s)), lens.distortion)


def delta(x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return _rot(x[:3]), x[3:6]


def group_of(link: str) -> int:
    if link in ("forearm", "tower"):
        return 0
    if link == "carpals":
        return 1
    return GROUPS.index(link.split("_", 1)[0])


def surface(path, origin: Transform, scale, spacing: float, seed: int = 0) -> np.ndarray:
    """Points over a binary STL's surface, about one per `spacing` cube (small triangles too), mm, in the link frame."""
    data = path.read_bytes()
    n = int.from_bytes(data[80:84], "little")
    tri = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                        count=n, offset=84)["v"].astype(float) * scale * 1000.0
    area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    rng = np.random.default_rng(seed)
    k = np.floor(area / spacing ** 2 + rng.random(len(tri))).astype(int)
    idx = np.repeat(np.arange(len(tri)), k)
    a, b = rng.random((2, len(idx)))
    flip = a + b > 1
    a[flip], b[flip] = 1 - a[flip], 1 - b[flip]
    t = tri[idx]
    pts = t[:, 0] + a[:, None] * (t[:, 1] - t[:, 0]) + b[:, None] * (t[:, 2] - t[:, 0])
    pts = np.vstack([pts, tri.mean(axis=1)])
    _, keep = np.unique(np.floor(pts / spacing).astype(np.int64), axis=0, return_index=True)
    o = Transform.from_rotation_translation(origin.rotation, origin.translation * 1000.0)
    return o.apply_to_points(pts[keep])


def _even(points: np.ndarray, n: int) -> np.ndarray:
    return points[np.linspace(0, len(points) - 1, n).astype(int)] if len(points) else np.zeros((0, 2), int)


def _bilinear(img: np.ndarray, uv: np.ndarray, outside: float) -> np.ndarray:
    h, w = img.shape
    u, v = uv[:, 0], uv[:, 1]
    ok = (u >= 0) & (u <= w - 1) & (v >= 0) & (v <= h - 1)
    out = np.full(len(uv), outside)
    u, v = u[ok], v[ok]
    u0, v0 = np.minimum(np.floor(u).astype(int), w - 2), np.minimum(np.floor(v).astype(int), h - 2)
    fu, fv = u - u0, v - v0
    out[ok] = (img[v0, u0] * (1 - fu) * (1 - fv) + img[v0, u0 + 1] * fu * (1 - fv) +
               img[v0 + 1, u0] * (1 - fu) * fv + img[v0 + 1, u0 + 1] * fu * fv)
    return out


@dataclass
class View:
    """A frame prepared for the fit, cropped around the hand, at the fit's scale."""
    angle: float
    offset: np.ndarray          # crop origin (u, v) in the fit image
    fg: np.ndarray              # the segmented hand (0/1)
    edges: np.ndarray           # the edges used (0/1)
    to_fg: np.ndarray           # distance to the segmented hand (px, capped)
    fg_px: np.ndarray           # samples of the segmented hand (u, v), crop px
    edge_px: np.ndarray         # samples of its edges


class LiveHandFit:
    def __init__(self, rig, model: HandModel | None = None, side: str = "right", spacing: float = SPACING_MM):
        self.rig, self.model = rig, model or HandModel.load(side)
        self.lens, self.cam, self.source = camera_model(rig, 270)
        self.base = hand_in_plate(rig)
        self.joints = list(self.model.movable)
        lim = [self.model.joints[j].limits_deg or (-180.0, 180.0) for j in self.joints]
        self.lo, self.hi = np.array([l for l, _ in lim]), np.array([h for _, h in lim])
        self.cloud = {l: surface(v.path, v.origin, v.scale, spacing) for l, v in self.model.visuals.items()}
        self.ids = {l: i + 1 for i, l in enumerate(self.cloud)}
        self.below = {j: self._subtree(self.model.joints[j].child) for j in self.joints}
        rng = np.random.default_rng(0)
        self.anchor = {l: c[rng.permutation(len(c))[:ANCHORS * (4 if group_of(l) < 2 else 1)]]
                       for l, c in self.cloud.items()}
        self.n_anchor = sum(len(a) for a in self.anchor.values())
        self.flexors = [self.joints.index(j) for js in FLEX.values() for j in js if j in self.joints]
        self.views: list[View] = []
        self.last: dict = {}
        self._view: View | None = None
        self._suggest: tuple[float, dict | None] = (0.0, None)
        self.reset()

    def _subtree(self, link: str) -> list[str]:
        out, todo = [], [link]
        while todo:
            l = todo.pop()
            out += [l] if l in self.cloud else []
            todo += [j.child for j in self.model.joints.values() if j.parent == l]
        return out

    def reset(self) -> None:
        self.x = np.zeros(6 + len(self.joints))
        self.std = np.full(self.x.size, np.nan)
        self.spread: dict[str, float] = {}
        self.obs: np.ndarray | None = None

    # -- rendering -----------------------------------------------------------
    def q_of(self, x: np.ndarray) -> dict[str, float]:
        return dict(zip(self.joints, x[6:]))

    def hand_in_plate(self, x: np.ndarray) -> Transform:
        """The URDF base frame (mm) in the plate frame, with the mount correction in x."""
        R, t = delta(x)
        return Transform.from_rotation_translation(R @ self.base.rotation, self.base.translation + t)

    def cam_poses(self, x: np.ndarray, angle: float) -> dict[str, Transform]:
        """Every visual link in the camera frame (mm)."""
        hand = self.hand_in_plate(x)
        to_cam = self.cam.inverse() @ plate_in_rig(self.rig, angle) @ hand
        return {l: to_cam @ Transform.from_rotation_translation(p.rotation, p.translation * 1000.0)
                for l, p in self.model.link_poses(self.q_of(x)).items() if l in self.cloud}

    def parts(self, poses: dict[str, Transform], lens: Lens, shape, offset=(0, 0)) -> list[tuple]:
        """Each link as a solid mask: (mean depth, group, u0, v0, mask), in the given image."""
        cv2 = _cv2()
        h, w = shape
        out = []
        k = np.ones((3, 3), np.uint8)
        for link, T in poses.items():
            c = T.apply_to_points(self.cloud[link])
            c = c[c[:, 2] > 1.0]
            if not len(c):
                continue
            u, v = np.round(lens.project(c) - offset).astype(int).T
            ok = (u >= -2) & (u < w + 2) & (v >= -2) & (v < h + 2)
            if not ok.any():
                continue
            u, v = u[ok], v[ok]
            u0, v0 = u.min() - 2, v.min() - 2
            m = np.zeros((v.max() - v0 + 3, u.max() - u0 + 3), np.uint8)
            m[v - v0, u - u0] = 1
            out.append((float(c[:, 2].mean()), self.ids[link], u0, v0, cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)))
        return out

    @staticmethod
    def composite(parts: list[tuple], shape) -> np.ndarray:
        """Link id per pixel (0: background), nearest link on top."""
        h, w = shape
        lab = np.zeros((h, w), np.uint8)
        for _, g, u0, v0, m in sorted(parts, key=lambda p: -p[0]):
            a0, b0 = max(u0, 0), max(v0, 0)
            a1, b1 = min(u0 + m.shape[1], w), min(v0 + m.shape[0], h)
            if a1 > a0 and b1 > b0:
                sub = lab[b0:b1, a0:a1]
                sub[m[b0 - v0:b1 - v0, a0 - u0:a1 - u0] > 0] = g
        return lab

    def paint(self, poses: dict[str, Transform], lens: Lens, shape, offset=(0, 0)) -> np.ndarray:
        return self.composite(self.parts(poses, lens, shape, offset), shape)

    def label(self, x: np.ndarray, angle: float, lens: Lens, shape, offset=(0, 0)) -> np.ndarray:
        return self.paint(self.cam_poses(x, angle), lens, shape, offset)

    @staticmethod
    def outlines(lab: np.ndarray) -> np.ndarray:
        """Where the label changes: the silhouette and the depth-ordered inner outlines (0/1)."""
        b = np.zeros(lab.shape, np.uint8)
        b[:, 1:] |= lab[:, 1:] != lab[:, :-1]
        b[1:, :] |= lab[1:, :] != lab[:-1, :]
        return b

    # -- the frame -----------------------------------------------------------
    def prepare(self, img: np.ndarray, angle: float, lens: Lens) -> View:
        """Crop around the estimate, segment the hand (GrabCut seeded by the model), find its edges."""
        cv2 = _cv2()
        h, w = img.shape[:2]
        est = self.label(self.x, angle, lens, (h, w))
        ys, xs = np.nonzero(est)
        if not len(xs):
            raise ValueError("the hand model is outside the image at this angle")
        u0, v0 = max(xs.min() - ROI_PX, 0), max(ys.min() - ROI_PX, 0)
        u1, v1 = min(xs.max() + ROI_PX + 1, w), min(ys.max() + ROI_PX + 1, h)
        crop = img[v0:v1, u0:u1]
        bgr = crop if crop.ndim == 3 else cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)
        gray = crop if crop.ndim == 2 else cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        model = (est[v0:v1, u0:u1] > 0).astype(np.uint8)
        seed = np.full(model.shape, cv2.GC_PR_BGD, np.uint8)
        seed[cv2.dilate(model, np.ones((5, 5), np.uint8)) > 0] = cv2.GC_PR_FGD
        seed[cv2.dilate(model, np.ones((2 * ROI_PX - 9, 2 * ROI_PX - 9), np.uint8)) == 0] = cv2.GC_BGD
        seed[:3, :] = seed[-3:, :] = cv2.GC_BGD
        seed[:, :3] = seed[:, -3:] = cv2.GC_BGD
        try:
            cv2.grabCut(bgr, seed, None, np.zeros((1, 65)), np.zeros((1, 65)), 3, cv2.GC_INIT_WITH_MASK)
            fg = ((seed == cv2.GC_FGD) | (seed == cv2.GC_PR_FGD)).astype(np.uint8)
        except cv2.error:
            fg = model
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        e = (cv2.Canny(cv2.GaussianBlur(gray, (3, 3), 0), 30, 90) > 0).astype(np.uint8)
        e &= cv2.dilate(fg, np.ones((5, 5), np.uint8))
        to_fg = np.minimum(cv2.distanceTransform(1 - fg, cv2.DIST_L2, 3), CAP_PX).astype(np.float32)
        fy, fx = np.nonzero(fg)
        ey, ex = np.nonzero(e)
        return View(angle, np.array([u0, v0]), fg, e, to_fg,
                    _even(np.c_[fx, fy], N_FG), _even(np.c_[ex, ey], N_EDGE))

    # -- the fit -------------------------------------------------------------
    def residual_view(self, x: np.ndarray, view: View, lens: Lens) -> np.ndarray:
        poses = self.cam_poses(x, view.angle)
        uv = np.vstack([lens.project(T.apply_to_points(self.anchor[l])) for l, T in poses.items()]) - view.offset
        return self._terms(uv, self.paint(poses, lens, view.fg.shape, view.offset), view)

    def _terms(self, uv: np.ndarray, lab: np.ndarray, view: View) -> np.ndarray:
        cv2 = _cv2()
        inside = _bilinear(view.to_fg, uv, CAP_PX)
        to_model = cv2.distanceTransform(1 - (lab > 0).astype(np.uint8), cv2.DIST_L2, 3)
        to_line = cv2.distanceTransform(1 - self.outlines(lab), cv2.DIST_L2, 3)
        cover = np.zeros(N_FG)
        if len(view.fg_px):
            cover = np.minimum(to_model[view.fg_px[:, 1], view.fg_px[:, 0]], CAP_PX)
        line = np.zeros(N_EDGE)
        if len(view.edge_px):
            line = W_EDGE * np.minimum(to_line[view.edge_px[:, 1], view.edge_px[:, 0]], EDGE_CAP_PX)
        return np.concatenate([inside, np.pad(cover, (0, N_FG - len(cover))),
                               np.pad(line, (0, N_EDGE - len(line)))])

    def _prior(self, x: np.ndarray, n: int) -> np.ndarray:
        """The mount stays near the CAD unless the image clearly says otherwise (a bound's worth
        costs MOUNT_PRIOR px over every residual); slightly bent beats over-stretched."""
        w = np.sqrt(n)
        hyper = np.zeros(len(self.joints))
        hyper[self.flexors] = HYPER * w * np.minimum(x[6:][self.flexors], 0.0) / 30.0
        return np.concatenate([MOUNT_PRIOR * w * x[:6] / BOUND, 0.02 * w * x[6:] / 30.0, hyper])

    def cost(self, x: np.ndarray, views: list[View], lens: Lens) -> float:
        r = [self.residual_view(x, v, lens) for v in views]
        r = np.concatenate([*r, self._prior(x, sum(len(q) for q in r))])
        return float(r @ r)

    def step(self, img: np.ndarray, angle: float, q: Mapping[str, float] | None = None, iters: int = 2,
             solve_joints: bool = False, search: bool = False) -> dict:
        """One frame (gray or BGR, any size): a few iterations from the last estimate."""
        t0 = time.perf_counter()
        cv2 = _cv2()
        if img.shape[1] != FIT_WIDTH:
            img = cv2.resize(img, (FIT_WIDTH, round(img.shape[0] * FIT_WIDTH / img.shape[1])),
                             interpolation=cv2.INTER_AREA)
        s = FIT_WIDTH / self.lens.size[0]
        lens = scaled(self.lens, s)
        if not solve_joints:
            self.x[6:] = np.clip([float((q or {}).get(j, 0.0)) for j in self.joints], self.lo, self.hi)
        view = self.prepare(img, angle, lens)
        views = [view, *self.views]
        free = np.arange(self.x.size if solve_joints else 6)
        lo, hi = np.r_[-BOUND, self.lo][free], np.r_[BOUND, self.hi][free]
        step = np.r_[MOUNT_STEP, np.full(len(self.joints), JOINT_STEP)][free]

        def full(y):
            x = self.x.copy()
            x[free] = y
            return x

        def residual(y):
            x = full(y)
            r = [self.residual_view(x, v, lens) for v in views]
            return np.concatenate([*r, self._prior(x, sum(len(q) for q in r))])

        x_cad = np.r_[np.zeros(6), np.zeros(len(self.joints)) if solve_joints else self.x[6:]]
        cad_cost = self._rms(x_cad, view, lens)
        if solve_joints and (search or not self.spread):
            self.search(views, lens)
        fit = least_squares(residual, self.x[free], lo, hi, step=step, max_iter=iters)
        self.x = full(fit.x)
        self.std = np.full(self.x.size, np.nan)
        self.std[free] = fit.std * SIGMA_PX / max(fit.rms, 1e-6)
        self._view = view
        if solve_joints:
            self.obs = self.observability([angle], lens, img.shape[:2])[0]
        fit_lab = self.label(self.x, angle, lens, view.fg.shape, view.offset)
        self.last = {
            "delta": {"rotation_deg": self.x[:3].round(2).tolist(), "translation_mm": self.x[3:6].round(2).tolist()},
            "mount_std": self.std[:6].round(2).tolist(),
            "cost_px": {"cad": round(cad_cost / s, 2), "fit": round(self._rms(self.x, view, lens) / s, 2)},
            "iterations": fit.iterations, "angle": float(angle), "scale": s, "source": self.source,
            "outlines": {"cad": self._polys(self.label(x_cad, angle, lens, view.fg.shape, view.offset) > 0,
                                            view, s, whole=True),
                         "fit": self._polys(fit_lab, view, s, whole=False)},
            "size": list(self.lens.size), "views": [v.angle for v in self.views],
            "solve_joints": solve_joints,
            "plate_from_hand": self.hand_in_plate(self.x).matrix.round(6).tolist(),
            "joints_deg": {j: round(float(v), 2) for j, v in zip(self.joints, self.x[6:])},
            "joint_table": self.joint_table() if solve_joints else [],
            "suggest": self.suggest(angle, lens, img.shape[:2]) if solve_joints else None,
            "segmentation": self.segmentation(view, s),
        }
        self.last["solve_s"] = round(time.perf_counter() - t0, 3)
        return self.last

    def search(self, views: list[View], lens: Lens) -> None:
        """Each finger's flexion pair over a coarse grid (the rest of the hand held): jump to a
        better cell than the local fit found, and record each joint's spread over the cells
        that fit about as well (palm-on, flexed and over-extended look alike)."""
        self.spread = {}
        rest_cache = []
        for v in views:
            poses = self.cam_poses(self.x, v.angle)
            rest_cache.append(poses)
        for finger, names in FLEX.items():
            idx = [6 + self.joints.index(n) for n in names]
            links = [l for l in self.cloud if group_of(l) == GROUPS.index(finger)]
            fixed = []
            for v, poses in zip(views, rest_cache):
                others = {l: T for l, T in poses.items() if l not in links}
                uv = np.vstack([lens.project(T.apply_to_points(self.anchor[l])) for l, T in others.items()])
                fixed.append((self.parts(others, lens, v.fg.shape, v.offset), uv - v.offset))
            grid = [(a, b) for a in GRID[names[0].split("_")[1]] for b in GRID[names[1].split("_")[1]]]
            grid.append(tuple(self.x[idx]))
            costs = []
            for a, b in grid:
                x = self.x.copy()
                x[idx] = np.clip([a, b], self.lo[np.array(idx) - 6], self.hi[np.array(idx) - 6])
                r = []
                for v, (rest, uv_rest) in zip(views, fixed):
                    mine = {l: T for l, T in self.cam_poses(x, v.angle).items() if l in links}
                    uv = np.vstack([uv_rest] + [lens.project(T.apply_to_points(self.anchor[l])) - v.offset
                                                for l, T in mine.items()])
                    lab = self.composite(rest + self.parts(mine, lens, v.fg.shape, v.offset), v.fg.shape)
                    r.append(self._terms(uv, lab, v))
                r.append(self._prior(x, sum(len(q) for q in r)))
                r = np.concatenate(r)
                costs.append(float(r @ r))
            costs = np.array(costs)
            best = int(np.argmin(costs))
            tie = costs.min() + TIE * (costs.max() - costs.min())
            if best != len(grid) - 1 and costs[-1] > tie:
                self.x[idx] = grid[best]
            near = np.array([g for g, c in zip(grid, costs) if c <= tie])
            for k, n in enumerate(names):
                self.spread[n] = float(np.ptp(near[:, k]))

    def _rms(self, x, view: View, lens: Lens) -> float:
        """The silhouette misfit (px): anchor points outside the hand and hand not covered."""
        r = self.residual_view(x, view, lens)[: self.n_anchor + N_FG]
        return float(np.sqrt((r ** 2).mean()))

    def observability(self, angles: list[float], lens: Lens, shape) -> np.ndarray:
        """Per plate angle and joint, 0..1: how much of the joint's motion the camera sees.
        First order: the share of its links' motion (a x r) across the line of sight (palm-on, a
        finger bending moves along it: unseen, and bent and over-stretched look alike), times
        the share of those links not hidden behind others."""
        out = np.zeros((len(angles), len(self.joints)))
        for k, angle in enumerate(angles):
            poses = self.cam_poses(self.x, angle)
            parts = self.parts(poses, lens, shape)
            lab = self.composite(parts, shape)
            seen = np.bincount(lab.ravel(), minlength=len(self.ids) + 1)
            alone = np.zeros(len(self.ids) + 1)
            for _, i, _, _, m in parts:
                alone[i] += m.sum()
            for j, name in enumerate(self.joints):
                links = self.below[name]
                J = self.model.joints[name]
                T = poses[J.child]
                a, o = T.rotation @ J.axis, T.translation
                P = np.vstack([poses[l].apply_to_points(self.anchor[l]) for l in links])
                d = np.cross(a, P - o)
                ray = P / np.linalg.norm(P, axis=1, keepdims=True)
                across = np.linalg.norm(d - (d * ray).sum(1, keepdims=True) * ray, axis=1).sum()
                ids = [self.ids[l] for l in links]
                visible = seen[ids].sum() / max(alone[ids].sum(), 1.0)
                out[k, j] = across / max(np.linalg.norm(d, axis=1).sum(), 1e-9) * min(visible, 1.0)
        return out

    def spans(self) -> float:
        """The plate angles the fit sees the hand from: their span (deg)."""
        a = [v.angle for v in self.views] + ([self._view.angle] if self._view else [])
        return max(a) - min(a) if a else 0.0

    def joint_table(self) -> list[dict]:
        """Each joint's estimate and how sure it is. A finger's bending is never called good from
        one view: on synthetic frames its silhouette fits nearly as well at quite other bends
        (base vs. middle joint trade, bent vs. over-stretched) and the measures here don't catch it."""
        out = []
        flex = {n for ns in FLEX.values() for n in ns}
        multi = self.spans() >= MULTI_VIEW_DEG
        for i, j in enumerate(self.joints):
            v, sd = float(self.x[6 + i]), float(self.std[6 + i])
            ob = float(self.obs[i]) if self.obs is not None else 1.0
            ambiguous = self.spread.get(j, 0.0) > SPREAD_DEG
            conf = ("poor" if ambiguous or ob < FAIR_OBS or not sd < FAIR_DEG
                    else "good" if ob >= GOOD_OBS and sd < GOOD_DEG else "fair")
            if j in flex:
                conf = "fair" if multi and conf != "poor" else "poor"
            out.append({"name": j, "deg": round(v, 1), "std_deg": None if np.isnan(sd) else round(min(sd, 999.0), 1),
                        "confidence": conf, "ambiguous": ambiguous, "seen": round(ob, 2),
                        "needs_views": j in flex and not multi,
                        "spread_deg": round(self.spread[j], 1) if j in self.spread else None,
                        "at_limit": bool(min(v - self.lo[i], self.hi[i] - v) < 0.5)})
        return out

    def suggest(self, angle: float, lens: Lens, shape) -> dict | None:
        """For joints this view can't pin: the plate angle that shows them best (throttled)."""
        poor = [i for i, r in enumerate(self.joint_table()) if r["confidence"] == "poor"]
        if not poor:
            self._suggest = (0.0, None)
            return None
        t, last = self._suggest
        names = [self.joints[i] for i in poor]
        if last is not None and time.monotonic() - t < SUGGEST_EVERY_S and last["joints"] == names:
            return last
        candidates = list(np.arange(PLATE_RANGE[0], PLATE_RANGE[1] + 1, 15.0))
        ob = self.observability(candidates, lens, shape)[:, poor]
        score = np.minimum(ob, GOOD_OBS).mean(axis=1)        # enough of each beats a lot of a few
        here = float(np.minimum(self.obs[poor], GOOD_OBS).mean()) if self.obs is not None else 0.0
        best = int(np.argmax(score))
        out = {"angle": float(candidates[best]), "joints": names, "better": bool(score[best] > here + 0.1)}
        self._suggest = (time.monotonic(), out)
        return out

    def keep_view(self) -> int:
        """Keep the last frame as an extra view (same pose, another plate angle)."""
        if self._view is None:
            raise ValueError("no frame yet")
        self.views = [v for v in self.views if abs(v.angle - self._view.angle) > 5.0] + [self._view]
        return len(self.views)

    def clear_views(self) -> None:
        self.views = []

    @staticmethod
    def segmentation(view: View, s: float) -> dict:
        """What the fit sees, for display: the segmented hand (1) and the edges it matches (2),
        over the crop, as a PNG (base64) and its full-resolution box (u, v, w, h)."""
        cv2 = _cv2()
        seg = view.fg.copy()
        seg[view.edges > 0] = 2
        h, w = seg.shape
        u0, v0 = view.offset
        return {"png": base64.b64encode(cv2.imencode(".png", seg)[1].tobytes()).decode(),
                "box": [round((u0 + 0.5) / s - 0.5, 1), round((v0 + 0.5) / s - 0.5, 1), round(w / s, 1), round(h / s, 1)]}

    def _polys(self, lab: np.ndarray, view: View, s: float, whole: bool) -> list[list[list[float]]]:
        """Outlines in full-resolution raw pixels: the whole silhouette, or each link's."""
        cv2 = _cv2()
        masks = [lab > 0] if whole else [lab == i for i in range(1, len(self.ids) + 1)]
        out = []
        for m in masks:
            cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            out += [((cv2.approxPolyDP(c, 1.0, True)[:, 0, :] + view.offset + 0.5) / s - 0.5).round(1).tolist()
                    for c in cs if len(c) > 8]
        return out
