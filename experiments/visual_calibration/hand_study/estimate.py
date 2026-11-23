"""Joint angles of one configuration from its multi-view photos (the mount held fixed, or
solved with it), with per-joint uncertainty and a view-split consistency check.

Frames are (plate angle, image) of camera "a", or (plate angle, image, camera) of any camera;
meta.json views carry `camera` (missing: "a"). Every camera's views join the one fit."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

import numpy as np

from hand_study import segment
from visual_calibration import live_fit as lf
from visual_calibration.live_fit import LiveHandFit, scaled
from visual_calibration import overlay as _overlay
from visual_calibration.rig_config import DEFAULT_CAMERA, RigConfig
from visual_calibration.solve import least_squares

WIDTH = 960
SPACING_MM = 0.9
ITERS = 12
FULL_ONLY = os.environ.get("STUDY_FULL_ONLY", "0") != "0"     # off by default: the rule cost accuracy on ser-4485
FULL_MARGIN_PX = 5.0        # fit px: a view counts only if every link projects this far inside the image
MIN_FULL_VIEWS = 2
EXEMPT = ("forearm",)       # the stand's forearm mesh runs out of every view by design
# Light parts (ser-7577's white tips and shells): GrabCut alone loses them against the cupboard
# when they are a small share of the crop; they are bluish (Lab b < 0) where the cupboard is
# not (b > 0), so near the model they are hand for sure. "L_MIN,B_MAX" or empty (off).
WHITE = tuple(map(float, os.environ["STUDY_WHITE_PARTS"].split(","))) if os.environ.get("STUDY_WHITE_PARTS") else None
# Only the hand and the tower's top are used: the fit ignores everything farther than KEEP_PAD_PX
# (fit px) from the model's geometry above Z_CUT_MM (plate frame), so floor, cables and the
# connector never enter the segmentation or the residual. 0 = off.
Z_CUT_MM = float(os.environ.get("STUDY_Z_CUT_MM", "90"))
KEEP_PAD_PX = int(os.environ.get("STUDY_KEEP_PAD_PX", str(lf.ROI_PX)))
# Weighting of the per-pose joint fits (mount fixed): "link" = every link counts the same, anchors
# (ANCHORS each) and mask samples (normalised per link, assigned to the nearest link of the model at
# the round's start pose); "tips2"/"tips4" = link with the distal links x2/x4; "area" = live_fit's
# (palm and tower anchors x4, mask samples even over the area). The tower and forearm carry no joint
# information and count 0 except in "area". Solves with the mount free (the baseline) always use "area".
WEIGHTS = os.environ.get("STUDY_WEIGHTS", "link").strip().lower()
DROP_LINKS = tuple(filter(None, os.environ.get("STUDY_DROP_LINKS", "").replace(" ", "").split(",")))
NO_JOINT_INFO = ("forearm", "tower")
TIP_MULT = {"tips2": 2, "tips4": 4}
W2_MAX = 10.0               # a barely visible link's samples weigh at most this (squared) over their area weight
MIN_LINK_SAMPLES = 10


_PLATE: dict = {}


def frame(f) -> tuple:
    """(angle, image, camera) of a frame (angle, image) or (angle, image, camera)."""
    return (f[0], f[1], f[2] if len(f) > 2 else DEFAULT_CAMERA)


def frame_key(f):
    """A frame's key: its angle for camera "a" (as single-camera results always had), else (camera, angle)."""
    a, _, c = frame(f)
    return a if c == DEFAULT_CAMERA else (c, a)


def key_name(k) -> str:
    return f"{k:g}" if not isinstance(k, tuple) else f"{k[0]}:{k[1]:g}"


def plate_in_rig(rig, angle: float):
    """overlay.plate_in_rig, cached: it re-reads calibration.yaml on every call (31 ms since the rig
    file keeps its history, ~7000 calls per solve = 3.5 min); refreshed when the file changes."""
    key = (os.stat(rig.calibration_path).st_mtime, round(float(angle), 4))
    if key not in _PLATE:
        if len(_PLATE) > 4096:
            _PLATE.clear()
        _PLATE[key] = _overlay.plate_in_rig(rig, angle)
    return _PLATE[key]


lf.plate_in_rig = plate_in_rig          # live_fit's projections use it too


RIG_AT = os.environ.get("STUDY_RIG_AT", "")         # refit old photos: the rig calibration in force then


def rig_at(calibrated_at: str = RIG_AT) -> RigConfig:
    """The rig, or with `calibrated_at` (a `history` entry of calibration.yaml) a copy of it with that
    calibration (in the rig's folder, calibrations/): photos are fitted with the rig calibration they were shot with."""
    from hand_study import paths
    rig = RigConfig.load(rig_id=paths.run_rig_id())
    if not calibrated_at:
        return rig
    import shutil
    import yaml
    d = Path(rig.own_file("calibrations")) / calibrated_at.replace(":", "")
    if not (d / "calibration.yaml").exists():
        cal = yaml.safe_load(Path(rig.calibration_path).read_text())
        old = [h for h in cal.get("history") or [] if h.get("calibrated_at") == calibrated_at]
        if not old:
            raise ValueError(f"no rig calibration {calibrated_at!r} in the history of {rig.calibration_path}")
        shutil.copytree(rig.path, d, dirs_exist_ok=True)
        cal = {**{k: v for k, v in cal.items() if k != "history"}, **{k: v for k, v in old[0].items() if v is not None}}
        (d / "calibration.yaml").write_text(yaml.safe_dump(cal, sort_keys=False))
    return RigConfig.load(path=str(d))


class StudyFit(LiveHandFit):
    def __init__(self, width: int = WIDTH, spacing: float = SPACING_MM):
        super().__init__(rig_at(RIG_AT), spacing=spacing)
        self.width = width
        self.s = width / self.lens.size[0]
        self.lens_s = scaled(self.lens, self.s)
        self._anchor_area = self.anchor
        self.mode = "area"
        self._unweighted = False

    def link_mult(self, link: str, mode: str) -> float:
        if link in DROP_LINKS:
            return 0.0
        if mode == "area":
            return 1.0
        if link in NO_JOINT_INFO:
            return 0.0
        return float(TIP_MULT.get(mode, 1)) if link.endswith("_dp") else 1.0

    def weigh(self, mode: str) -> None:
        """Anchors for the weighting `mode` (see WEIGHTS)."""
        if mode not in ("area", "link", *TIP_MULT):
            raise ValueError(f"STUDY_WEIGHTS={mode!r}: use link, area, tips2 or tips4")
        self.mode = mode
        if mode == "area":
            self.anchor = {l: a if l not in DROP_LINKS else a[:0] for l, a in self._anchor_area.items()}
        else:
            rng = np.random.default_rng(0)
            self._anchor_idx = {l: rng.permutation(len(c))[:int(lf.ANCHORS * self.link_mult(l, mode))]
                                for l, c in self.cloud.items()}
            self.anchor = {l: self.cloud[l][i] for l, i in self._anchor_idx.items()}
        self.n_anchor = sum(len(a) for a in self.anchor.values())

    def fg_weights(self, view: "lf.View") -> np.ndarray | None:
        """Per mask sample, the weight that makes every link count the same (mode != area), or that
        drops DROP_LINKS (area); None: all 1."""
        if self.mode == "area" and not DROP_LINKS or not len(view.fg_px):
            return None
        cv2 = lf._cv2()
        lab = self.label(self.x, view.angle, self.lens_s, view.fg.shape, view.offset, view.camera)
        k = np.ones((3, 3), np.uint8)
        for _ in range(int(lf.CAP_PX)):               # the nearest link within the distance cap
            empty = lab == 0
            if not empty.any():
                break
            grown = cv2.dilate(lab, k)
            lab[empty] = grown[empty]
        name = {i: l for l, i in self.ids.items()}
        links = [name.get(int(i)) for i in lab[view.fg_px[:, 1], view.fg_px[:, 0]]]
        w2 = np.ones(len(links))
        mult = {l: self.link_mult(l, self.mode) for l in set(links) if l}
        if self.mode == "area":
            for i, l in enumerate(links):
                w2[i] = mult[l] if l else 1.0
            return np.sqrt(w2)
        n = {l: links.count(l) for l in mult}
        use = {l: m for l, m in mult.items() if m > 0 and n[l] >= MIN_LINK_SAMPLES}
        n_use = sum(n[l] for l in use)
        tot = sum(use.values())
        for i, l in enumerate(links):
            if l is None:
                continue
            if mult[l] == 0:
                w2[i] = 0.0
            elif l in use:
                w2[i] = min(use[l] * n_use / (n[l] * tot), W2_MAX)
        return np.sqrt(w2)

    def _terms(self, uv: np.ndarray, lab: np.ndarray, view: "lf.View") -> np.ndarray:
        r = super()._terms(uv, lab, view)
        w = None if self._unweighted else getattr(view, "w_fg", None)
        if w is not None:
            r[len(uv):len(uv) + len(w)] *= w
        return r

    def _rms(self, x, view, lens) -> float:
        """The silhouette misfit (px) always unweighted, with live_fit's anchors: comparable across weightings."""
        mode = self.mode
        self.weigh("area")
        self._unweighted = True
        try:
            return super()._rms(x, view, lens)
        finally:
            self._unweighted = False
            self.weigh(mode)

    def views_of(self, frames: list[tuple[float, np.ndarray]]) -> list:
        cv2 = lf._cv2()
        seg = segment.masker()                   # None: GrabCut (the default)
        self._prob = {}
        out = []
        for f in self.usable(frames):
            angle, img, cam = frame(f)
            keep = self.keep(self.x, angle, self.lens, img.shape[:2], cam)  # crop -> segment -> _clip:
            if keep is not None:                                            # no segmenter sees floor or cables
                img = img.copy()
                img[~keep] = 0
            if seg is not None:                  # cached per (cropped) photo
                p = seg.prob(img, self.label(self.x, angle, self.lens, img.shape[:2], cam=cam), self.ids) if seg.prompted else seg.prob(img)
                self._prob[frame_key(f)] = cv2.resize(p, (self.width, round(p.shape[0] * self.width / p.shape[1])),
                                                      interpolation=cv2.INTER_AREA)
            if img.shape[1] != self.width:
                img = cv2.resize(img, (self.width, round(img.shape[0] * self.width / img.shape[1])),
                                 interpolation=cv2.INTER_AREA)
            v = self.prepare(img, angle, self.lens_s, cam)
            v.w_fg = self.fg_weights(v)
            out.append(v)
        return out

    def usable(self, frames) -> list:
        """The frames of cameras the rig calibration has (a camera not yet solved would only mislead)."""
        return [f for f in frames if frame(f)[2] in self.cameras]

    def keep(self, x: np.ndarray, angle: float, lens, shape, cam: str = DEFAULT_CAMERA) -> np.ndarray | None:
        """The pixels the fit may use: within KEEP_PAD_PX (at the fit scale) of the model above Z_CUT_MM."""
        if Z_CUT_MM <= 0:
            return None
        cv2 = lf._cv2()
        pose = self.cam if cam == DEFAULT_CAMERA else self.cameras[cam][1]
        lens = self.lens_of(cam, lens)
        hand, to_cam = self.hand_in_plate(x), pose.inverse() @ plate_in_rig(self.rig, angle)
        pts = []
        for l, p in self.model.link_poses(self.q_of(x)).items():
            if l in self.cloud:
                X = (hand @ lf.Transform.from_rotation_translation(p.rotation, p.translation * 1000.0)).apply_to_points(self.cloud[l])
                pts.append(X[X[:, 2] >= Z_CUT_MM])
        if not sum(map(len, pts)):
            return None
        P = to_cam.apply_to_points(np.vstack(pts))
        h, w = shape
        f = max(1, round(w / self.width))         # drawn at the fit scale, then scaled up (a 4K dilation costs ~1.4 s)
        hs, ws = -(-h // f), -(-w // f)
        uv = np.floor(lens.project(P[P[:, 2] > 1.0]) / f).astype(int)
        uv = uv[(uv[:, 0] >= 0) & (uv[:, 0] < ws) & (uv[:, 1] >= 0) & (uv[:, 1] < hs)]
        m = np.zeros((hs, ws), np.uint8)
        m[uv[:, 1], uv[:, 0]] = 1
        r = max(1, round(KEEP_PAD_PX * w / self.width / f))
        m = cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))
        if f > 1:
            m = cv2.resize(m, (ws * f, hs * f), interpolation=cv2.INTER_NEAREST)[:h, :w]
        return m > 0

    def prepare(self, img: np.ndarray, angle: float, lens, cam: str = DEFAULT_CAMERA) -> "lf.View":
        view = self._prepare(img, angle, lens, cam)
        keep = self.keep(self.x, angle, lens, img.shape[:2], cam)
        return view if keep is None else self._clip(view, keep)

    def _clip(self, view: "lf.View", keep: np.ndarray) -> "lf.View":
        """fg and edges only inside keep; outside it every model point is free (no pull either way)."""
        cv2 = lf._cv2()
        u0, v0 = view.offset
        k = keep[v0:v0 + view.fg.shape[0], u0:u0 + view.fg.shape[1]]
        fg, e = view.fg & k, view.edges & k
        to_fg = np.minimum(cv2.distanceTransform(1 - fg, cv2.DIST_L2, 3), lf.CAP_PX).astype(np.float32)
        to_fg[~k] = 0.0
        fy, fx = np.nonzero(fg)
        ey, ex = np.nonzero(e)
        return lf.View(view.angle, view.offset, fg, e, to_fg,
                       lf._even(np.c_[fx, fy], lf.N_FG), lf._even(np.c_[ex, ey], lf.N_EDGE), view.camera)

    def _prepare(self, img: np.ndarray, angle: float, lens, cam: str = DEFAULT_CAMERA) -> "lf.View":
        prob = getattr(self, "_prob", {}).get(frame_key((angle, None, cam)))
        if prob is not None:
            return self._prepare_learned(img, angle, lens, prob, cam)
        if WHITE is None:
            return super().prepare(img, angle, lens, cam)
        cv2 = lf._cv2()
        h, w = img.shape[:2]
        est = self.label(self.x, angle, lens, (h, w), cam=cam)
        ys, xs = np.nonzero(est)
        if not len(xs):
            raise ValueError("the hand model is outside the image at this angle")
        R = lf.ROI_PX
        u0, v0 = max(xs.min() - R, 0), max(ys.min() - R, 0)
        u1, v1 = min(xs.max() + R + 1, w), min(ys.max() + R + 1, h)
        crop = img[v0:v1, u0:u1]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB).astype(np.float32)
        white = (lab[..., 0] * (100 / 255) > WHITE[0]) & (lab[..., 2] - 128 < WHITE[1])
        model = (est[v0:v1, u0:u1] > 0).astype(np.uint8)
        near = cv2.dilate(model, np.ones((2 * R - 9, 2 * R - 9), np.uint8)) > 0
        white &= near
        seed = np.full(model.shape, cv2.GC_PR_BGD, np.uint8)
        seed[cv2.dilate(model, np.ones((5, 5), np.uint8)) > 0] = cv2.GC_PR_FGD
        seed[~near] = cv2.GC_BGD
        seed[:3, :] = seed[-3:, :] = cv2.GC_BGD
        seed[:, :3] = seed[:, -3:] = cv2.GC_BGD
        seed[white] = cv2.GC_FGD
        try:
            cv2.grabCut(crop, seed, None, np.zeros((1, 65)), np.zeros((1, 65)), 3, cv2.GC_INIT_WITH_MASK)
            fg = ((seed == cv2.GC_FGD) | (seed == cv2.GC_PR_FGD)).astype(np.uint8)
        except cv2.error:
            fg = model | white.astype(np.uint8)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        e = (cv2.Canny(cv2.GaussianBlur(gray, (3, 3), 0), 30, 90) > 0).astype(np.uint8)
        e &= cv2.dilate(fg, np.ones((5, 5), np.uint8))
        to_fg = np.minimum(cv2.distanceTransform(1 - fg, cv2.DIST_L2, 3), lf.CAP_PX).astype(np.float32)
        fy, fx = np.nonzero(fg)
        ey, ex = np.nonzero(e)
        return lf.View(angle, np.array([u0, v0]), fg, e, to_fg,
                       lf._even(np.c_[fx, fy], lf.N_FG), lf._even(np.c_[ex, ey], lf.N_EDGE), cam)

    def _prepare_learned(self, img: np.ndarray, angle: float, lens, prob: np.ndarray,
                         cam: str = DEFAULT_CAMERA) -> "lf.View":
        """prepare() with the learned mask (segment.py, STUDY_SEGMENTER) in GrabCut's place, kept within
        GrabCut's band around the model; crop, edges and samples as in live_fit.prepare."""
        cv2 = lf._cv2()
        h, w = img.shape[:2]
        est = self.label(self.x, angle, lens, (h, w), cam=cam)
        ys, xs = np.nonzero(est)
        if not len(xs):
            raise ValueError("the hand model is outside the image at this angle")
        R = lf.ROI_PX
        u0, v0 = max(xs.min() - R, 0), max(ys.min() - R, 0)
        u1, v1 = min(xs.max() + R + 1, w), min(ys.max() + R + 1, h)
        gray = cv2.cvtColor(img[v0:v1, u0:u1], cv2.COLOR_BGR2GRAY)
        model = (est[v0:v1, u0:u1] > 0).astype(np.uint8)
        near = cv2.dilate(model, np.ones((2 * R - 9, 2 * R - 9), np.uint8)) > 0
        fg = ((prob[v0:v1, u0:u1] > 0.5) & near).astype(np.uint8)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        e = (cv2.Canny(cv2.GaussianBlur(gray, (3, 3), 0), 30, 90) > 0).astype(np.uint8)
        e &= cv2.dilate(fg, np.ones((5, 5), np.uint8))
        to_fg = np.minimum(cv2.distanceTransform(1 - fg, cv2.DIST_L2, 3), lf.CAP_PX).astype(np.float32)
        fy, fx = np.nonzero(fg)
        ey, ex = np.nonzero(e)
        return lf.View(angle, np.array([u0, v0]), fg, e, to_fg,
                       lf._even(np.c_[fx, fy], lf.N_FG), lf._even(np.c_[ex, ey], lf.N_EDGE), cam)

    def fit(self, views, free_mount: bool, iters: int = ITERS):
        free = np.arange(self.x.size) if free_mount else np.arange(6, self.x.size)
        lo, hi = np.r_[-lf.BOUND, self.lo][free], np.r_[lf.BOUND, self.hi][free]
        step = np.r_[lf.MOUNT_STEP, np.full(len(self.joints), lf.JOINT_STEP)][free]
        base = self.x.copy()

        def full(y):
            x = base.copy()
            x[free] = y
            return x

        def residual(y):
            x = full(y)
            r = [self.residual_view(x, v, self.lens_s) for v in views]
            return np.concatenate([*r, self._prior(x, sum(len(q) for q in r))])

        f = least_squares(residual, self.x[free], lo, hi, step=step, max_iter=iters)
        self.x = full(f.x)
        std = np.full(self.x.size, np.nan)
        std[free] = f.std * lf.SIGMA_PX / max(f.rms, 1e-6)
        return f, std

    def margins(self, x: np.ndarray, angle: float, cam: str = DEFAULT_CAMERA) -> dict[str, float]:
        """Per link, how far (fit px) its projection stays inside the image (< 0: cut off)."""
        h, w = round(self.lens.size[1] * self.s), self.width
        lens = self.lens_of(cam, self.lens_s)
        out = {}
        for l, T in self.cam_poses(x, angle, cam).items():
            if l in EXEMPT:
                continue
            c = T.apply_to_points(self.cloud[l])
            c = c[c[:, 2] > 1.0]
            if not len(c):
                out[l] = -np.inf
                continue
            uv = lens.project(c)
            out[l] = float(min(uv[:, 0].min(), uv[:, 1].min(), w - 1 - uv[:, 0].max(), h - 1 - uv[:, 1].max()))
        return out

    def cut(self, x: np.ndarray, frames) -> dict:
        """The frames whose hand is not wholly inside the image: frame_key -> [worst link, margin px]."""
        out = {}
        for f in frames:
            a, _, c = frame(f)
            m = self.margins(x, a, c)
            worst = min(m, key=m.get)
            if m[worst] < FULL_MARGIN_PX:
                out[frame_key(f)] = [worst, round(m[worst], 1)]
        return out

    def per_view_rms(self, views) -> list[float]:
        return [round(self._rms(self.x, v, self.lens_s) / self.s, 2) for v in views]

    def solve(self, frames, q0: dict, mount: np.ndarray | None, rounds: int = 2, split: bool = True) -> dict:
        """mount None: solve it too (a baseline). Two rounds: the segmentation is re-seeded
        from the first round's pose. FULL_ONLY: only the views showing the whole hand (checked
        at the start pose, then again at the solution, re-solved once if that changes the set);
        fewer than MIN_FULL_VIEWS left: the pose is suspect (left out of every analysis)."""
        frames = self.usable(frames)
        if not FULL_ONLY:
            return self._solve(frames, q0, mount, rounds, split)
        x0 = np.r_[np.zeros(6) if mount is None else mount,
                   np.clip([float(q0.get(j, 0.0)) for j in self.joints], self.lo, self.hi)]
        cut = self.cut(x0, frames)
        use = [f for f in frames if frame_key(f) not in cut] if len(frames) - len(cut) >= MIN_FULL_VIEWS else frames
        out = self._solve(use, q0, mount, rounds, split)
        for _ in range(2):
            cut = self.cut(self.x, frames)
            now = [f for f in frames if frame_key(f) not in cut]
            if [frame_key(f) for f in now] == [frame_key(f) for f in use] or len(now) < MIN_FULL_VIEWS:
                break
            use = now
            out = self._solve(use, q0, mount, rounds, split)
        out["cut_views"] = {key_name(k): v for k, v in cut.items()}
        out["n_views_all"] = len(frames)
        full = [frame_key(f) for f in frames if frame_key(f) not in cut]
        if len(full) < MIN_FULL_VIEWS:
            out["suspect"] = [*out["suspect"], f"only {len(full)} views show the whole hand"]
        elif [frame_key(f) for f in use] != full:
            out["suspect"] = [*out["suspect"], "the full-view set did not settle"]
        return out

    def _solve(self, frames, q0: dict, mount: np.ndarray | None, rounds: int = 2, split: bool = True) -> dict:
        t0 = time.perf_counter()
        self.weigh("area" if mount is None else WEIGHTS)
        self.x = np.r_[np.zeros(6) if mount is None else mount,
                       np.clip([float(q0.get(j, 0.0)) for j in self.joints], self.lo, self.hi)]
        x_start = self.x.copy()
        for r in range(rounds):
            views = self.views_of(frames)
            self.search(views, self.lens_s)
            f, std = self.fit(views, free_mount=mount is None)
        rms = self.per_view_rms(views)
        self.last_views = views
        quality = self.quality(views)
        cams = [frame(f)[2] for f in frames]
        out = {"x": self.x.tolist(), "joints": dict(zip(self.joints, self.x[6:].round(3).tolist())),
               "std": dict(zip(self.joints, np.nan_to_num(std[6:], nan=-1).round(3).tolist())),
               "mount": self.x[:6].round(3).tolist(), "mount_std": np.nan_to_num(std[:6], nan=-1).round(3).tolist(),
               "spread": {k: round(v, 1) for k, v in self.spread.items()},
               "rms_px": float(np.mean(rms)), "view_rms_px": rms, "angles": [frame(f)[0] for f in frames],
               **({"cameras": cams} if set(cams) != {DEFAULT_CAMERA} else {}),
               "quality": quality, "suspect": suspect(quality),
               "obs": self.observability([frame(f)[0] for f in frames], self.lens_s,
                                         (round(self.lens.size[1] * self.s), self.width), cams).max(0).round(3).tolist()}
        if split and len(frames) >= 4:
            halves = {}
            x_full = self.x.copy()
            for name, sub in (("even", frames[0::2]), ("odd", frames[1::2])):
                self.x = np.r_[x_full[:6], x_start[6:]]
                vs = self.views_of(sub)
                self.search(vs, self.lens_s)
                self.fit(vs, free_mount=False)
                halves[name] = dict(zip(self.joints, self.x[6:].round(3).tolist()))
            self.x = x_full
            out["split"] = halves
        out["solve_s"] = round(time.perf_counter() - t0, 1)
        if segment.name() != "grabcut":
            out["segmenter"] = segment.name()
        out["weights"] = self.mode + (f" -{','.join(DROP_LINKS)}" if DROP_LINKS else "")
        return out


    def quality(self, views) -> list[dict]:
        """Per view, mask vs. model silhouette at the solution: the share of the mask the model
        does not explain (rods, cables, shadows) and the share of the model's fingers the mask misses."""
        out = []
        fingers = {self.ids[l] for l in self.cloud if lf.group_of(l) >= 2}
        for v in views:
            lab = self.label(self.x, v.angle, self.lens_s, v.fg.shape, v.offset, v.camera)
            model, fg = lab > 0, v.fg > 0
            fin = np.isin(lab, list(fingers))
            out.append({"angle": round(float(v.angle), 1), **({"camera": v.camera} if v.camera != DEFAULT_CAMERA else {}),
                        "mask_only": round(float((fg & ~model).sum() / max(fg.sum(), 1)), 3),
                        "model_only": round(float((model & ~fg).sum() / max(model.sum(), 1)), 3),
                        "finger_missed": round(float((fin & ~fg).sum() / max(fin.sum(), 1)), 3)})
        return out

    def save_views(self, path: Path) -> None:
        v = self.last_views
        cams = {"camera": [x.camera for x in v]} if any(x.camera != DEFAULT_CAMERA for x in v) else {}
        np.savez_compressed(path, angle=[x.angle for x in v], offset=np.array([x.offset for x in v]), **cams,
                            **{f"fg{k}": x.fg.astype(bool) for k, x in enumerate(v)},
                            **{f"edges{k}": x.edges.astype(bool) for k, x in enumerate(v)})


MASK_ONLY_MAX = 0.15
FINGER_MISSED_MAX = 0.25


def suspect(quality: list[dict]) -> list[str]:
    """Why the masks of a pose are not to be trusted (empty: fine)."""
    why = []
    for q in quality:
        at = f"{q['angle']:.0f}°" + (f" (camera {q['camera']})" if q.get("camera") else "")
        if q["mask_only"] > MASK_ONLY_MAX:
            why.append(f"{at}: {q['mask_only']:.0%} of the mask not hand (rods, cables, shadow?)")
        if q["finger_missed"] > FINGER_MISSED_MAX:
            why.append(f"{at}: mask misses {q['finger_missed']:.0%} of the fingers")
    return why


def load_frames(cfg_dir: Path, stops: list[float] | None = None, cameras: list[str] | None = None) -> list[tuple]:
    """(angle, image) per view of camera "a", (angle, image, camera) of any other (meta.json `camera`).
    cameras: only these (default: all; the fit skips cameras it has no calibration for)."""
    cv2 = lf._cv2()
    meta = json.loads((cfg_dir / "meta.json").read_text())
    views = sorted(meta["views"], key=lambda v: (v["stop"], v.get("camera", DEFAULT_CAMERA) != DEFAULT_CAMERA,
                                                 v.get("camera", DEFAULT_CAMERA)))
    if stops is not None:
        views = [v for v in views if v["stop"] in stops]
    if cameras is not None:
        views = [v for v in views if v.get("camera", DEFAULT_CAMERA) in cameras]
    return [(float(v["angle"]), cv2.imread(str(cfg_dir / v["file"])),
             *([v["camera"]] if v.get("camera", DEFAULT_CAMERA) != DEFAULT_CAMERA else [])) for v in views]
