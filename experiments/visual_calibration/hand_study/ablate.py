"""Fit and segmentation ablations (Clemens, 9 Nov), offline on the stored photos: every variant
re-solves the same poses (a training subset and the held-out set), a per-joint curve is trained on
its own training solves and scored on its own held-out solves (joint deg, fingertip mm by FK), next
to the noise and bias checks that a self-consistent score cannot see (repeatability, silhouette
residual, shift against the baseline, pull towards a perturbed seed).

The learned masks of the variants that do not change the crop or the segmenter are computed once
per photo on the crop at the primary solution ("ref": hand_study/estimate's keep at est.json's pose),
cached beside the photo (configs/<id>/abl/cache), so most variants are CPU-only; the seeded variants
recompute theirs from the current estimate in every round ("live", like the night's workers).

    python -m hand_study.ablate run STUDY [--procs 5] [--gpu 2] [--only k1,k2]
    python -m hand_study.ablate score STUDY          # -> protocol/<STUDY>/fit_ablation.json
    python -m hand_study.ablate one STUDY KEY        # one variant (the pool runs these)

STUDY: a protocol night's tag (train S1 + S2 + deep, held out V5), or ser-7577 / ser-4485 (the
weighting re-run on the old photos, with the rig calibration they were shot with).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np

from hand_study import analysis, colour, curve, estimate, hardstops, segment
from hand_study.configs import JOINTS
from hand_study.estimate import StudyFit, load_frames
from hand_study.mapping import LinearMap
from visual_calibration import live_fit as lf

STUDY = analysis.STUDY
SESS = STUDY / "sessions"
N_TRAIN = 24
FIVE = (45.0, 112.5, 180.0, 225.0, 292.5)
THREE = (45.0, 180.0, 292.5)
EIGHT = (0.0, 45.0, 112.5, 135.0, 180.0, 225.0, 270.0, 292.5)
PERTURB_DEG = 10.0
SKIN = tuple(filter(None, os.environ.get("STUDY_SKIN_LINKS", "thumb_dp,index_dp,middle_dp,ring_dp,pinky_dp").split(",")))
PALM = ("carpals",)
OLD_RIG = "2026-11-07T21:29:08"      # ser-4485 and ser-7577 were shot with it (the rig changed 9 Nov 01:37)

BASE = {"seg": "birefnet", "crop": "ref", "weights": "link", "drop_links": (), "zcut": 90.0, "pad": 40, "views": 5,
        "width": 960, "edge": True, "mount": "night", "seed": "assumed", "band": None, "disagree": False,
        "drop_iou": None, "drop_px": None, "offsets": False, "scope": "all", "color": 0.0}

VARIANTS = [   # (group, key, label, changes)
    ("0", "base", "baseline: BiRefNet, crop at the solution, link weights, z 90 mm, pad 40 px, 5 views, 960 px", {}),
    ("1", "band", "drop BiRefNet's uncertain band (0.2 < p < 0.8)", {"band": (0.2, 0.8)}),
    ("1", "disagree", "drop pixels the segmenters disagree on (BiRefNet, GrabCut, HQ-SAM)", {"disagree": True}),
    ("2", "drop_iou", "drop views with mask/model IoU < 0.8", {"drop_iou": 0.8}),
    ("2", "drop_px", "drop views with < 60 % of the pose's median hand pixels", {"drop_px": 0.6}),
    ("3", "grabcut", "GrabCut (seeded from the estimate, every round)", {"seg": "grabcut", "crop": "live"}),
    ("3", "hqsam_box", "HQ-SAM, one box (the seed-free crop's)", {"seg": "hqsam_box"}),
    ("3", "hqsam", "HQ-SAM prompted by the URDF (box + points per finger, every round)", {"seg": "hqsam", "crop": "live"}),
    ("4", "unseeded", "unseeded: BiRefNet on the seed-free crop (the hand's reach)", {"crop": "free"}),
    ("4", "pull_grabcut", f"seed +-{PERTURB_DEG:g}°: GrabCut, seeded", {"seg": "grabcut", "crop": "live", "seed": "perturb", "scope": "test"}),
    ("4", "pull_live", f"seed +-{PERTURB_DEG:g}°: BiRefNet, crop from the estimate (the night's pipeline)", {"crop": "live", "seed": "perturb", "scope": "test"}),
    ("4", "pull_free", f"seed +-{PERTURB_DEG:g}°: BiRefNet, seed-free crop", {"crop": "free", "seed": "perturb", "scope": "test"}),
    ("5", "area", "per area (live_fit: palm and tower anchors x4, mask samples even)", {"weights": "area"}),
    ("5", "tips2", "link weights, fingertips x2", {"weights": "tips2"}),
    ("5", "tips4", "link weights, fingertips x4", {"weights": "tips4"}),
    ("6", "no_palm", "without palm / carpals", {"drop_links": PALM}),
    ("6", "no_skin", f"without the skin-covered links ({', '.join(SKIN)})", {"drop_links": SKIN}),
    ("6", "offsets", "constant outline offset per link (fitted on 8 training poses, then fixed)", {"offsets": True}),
    ("7", "z60", "crop z-cut 60 mm", {"zcut": 60.0}),
    ("7", "z120", "crop z-cut 120 mm", {"zcut": 120.0}),
    ("7", "pad20", "crop pad 20 px", {"pad": 20}),
    ("7", "pad60", "crop pad 60 px", {"pad": 60}),
    ("8", "views3", "3 views per pose", {"views": 3}),
    ("8", "views8", "8 views per pose (held out only; training keeps 5)", {"views": 8}),
    ("8", "w1920", "fit width 1920 px", {"width": 1920}),
    ("9", "no_edge", "edge term off", {"edge": False}),
    ("11", "color", "skin/frame colour boundaries (orca-ui's coloured meshes), weight 0.7", {"color": 0.7}),
    ("11", "color2", "skin/frame colour boundaries, weight 1.5", {"color": 1.5}),
    ("10", "mount_session", "mount refit per session (median of free-mount solves of 6 poses)", {"mount": "session"}),
]
GROUPS = {"0": "baseline", "1": "uncertain pixels", "2": "poor views", "3": "segmenter", "4": "seeding (confirmation bias)",
          "5": "weighting", "6": "links and outline", "7": "crop", "8": "views and resolution", "9": "edge term", "10": "mount"}
OLD_VARIANTS = [("5", "live_area", "per area (as before 9 Nov)", {"crop": "live", "weights": "area"}),
                ("5", "live_link", "link weights, tower 0 (new default)", {"crop": "live", "weights": "link"})]
GPU_HEAVY = {"band", "disagree", "hqsam_box", "hqsam", "unseeded", "pull_live", "pull_free", "z60", "z120", "pad20",
             "pad60", "w1920", "live_area", "live_link", "pull_grabcut", "grabcut"}


def variants(study: str) -> dict[str, dict]:
    """The study's variants; a black-skin hand (touch, full) has no colour variants (no skin/frame contrast)."""
    vs = OLD_VARIANTS if study.startswith("ser-") else VARIANTS
    black = colour.tone_of_study(study) == "black"
    return {k: {**BASE, **ch, "group": g, "label": lab, "key": k} for g, k, lab, ch in vs
            if not (black and ch.get("color"))}


# -- the poses ------------------------------------------------------------------
def solved(sessions: list[Path]) -> list[Path]:
    return [d for s in sessions for d in sorted((s / "configs").glob("*")) if (d / "est.json").exists()]


def even(ds: list[Path], n: int) -> list[Path]:
    if len(ds) <= n:
        return ds
    return [ds[i] for i in np.linspace(0, len(ds) - 1, n).round().astype(int)]


def selection(study: str) -> dict:
    """train / test config dirs, the rig calibration, the map base (stops) for the study."""
    if study == "ser-7577":
        return {"train": solved([SESS / "20261108-s7577-N07-short"]), "test": solved([SESS / "20261108-s7577-N08-V"]),
                "rig_at": OLD_RIG, "rom": None}
    if study == "ser-4485":
        return {"train": solved([SESS / "20261108-night-N07-short"]),
                "test": solved([SESS / "20261108-night-N02-V0", SESS / "20261108-night-N08-V5"]), "rig_at": OLD_RIG, "rom": None}
    spec = json.loads((STUDY / "night" / f"{study}.json").read_text())
    tr = solved([Path(p) for k in ("S1", "S2", "deep") for p in spec["sets"].get(k, [])])
    te = solved([Path(p) for p in spec["sets"].get("V5", [])]) or solved([Path(p) for p in spec["sets"].get("V4", [])])   # V5: 8 stops
    own = Path(spec["protocol_dir"]) / "refit-stops.json"
    rom = {j: s["new"] for j, s in json.loads(own.read_text())["stops"].items()} if own.exists() else None
    return {"train": even(tr, N_TRAIN), "test": te, "rig_at": "", "rom": rom}


def out_of(d: Path, key: str) -> Path:
    return d / "abl" / f"{key}.json"


# -- the fit ----------------------------------------------------------------------
class AFit(StudyFit):
    def __init__(self, v: dict):
        super().__init__(width=v["width"])
        self.v = v
        self.ref_x = None
        self.cfg: Path | None = None
        self.models: dict = {}
        self.base_cloud = {l: c.copy() for l, c in self.cloud.items()}
        self.normal = {l: self._normals(c) for l, c in self.cloud.items()}
        self.d = {l: 0.0 for l in self.cloud}
        self.colour = colour.clouds(self.model) if v["color"] else {}    # variants() drops it for black skin
        self._colour_off = False

    @staticmethod
    def _normals(c: np.ndarray) -> np.ndarray:
        """Outward directions of a link's surface points: away from its long axis (PCA)."""
        m = c.mean(0)
        a = np.linalg.svd(c - m, full_matrices=False)[2][0]
        r = (c - m) - np.outer((c - m) @ a, a)
        return r / np.maximum(np.linalg.norm(r, axis=1, keepdims=True), 1e-6)

    def set_offsets(self, d: dict[str, float]) -> None:
        self.d = dict(d)
        for l in self.cloud:
            self.cloud[l] = self.base_cloud[l] + self.d.get(l, 0.0) * self.normal[l]
        if getattr(self, "_anchor_idx", None) and self.mode != "area":
            self.anchor = {l: self.cloud[l][i] for l, i in self._anchor_idx.items()}

    def weigh(self, mode: str) -> None:
        super().weigh(mode)
        if mode != "area" and any(self.d.values()):
            self.anchor = {l: self.cloud[l][i] for l, i in self._anchor_idx.items()}

    def segm(self, kind: str):
        if kind not in self.models:
            self.models[kind] = segment.BiRefNet() if kind == "birefnet" else segment.HQSam()
        return self.models[kind]

    # crops --------------------------------------------------------------------
    def keep_free(self, angle: float, shape) -> np.ndarray | None:
        """Seed-free: the model above the z-cut at the ROM's corners and middle, all together."""
        k = None
        for q in (self.lo, self.hi, np.zeros_like(self.lo), (self.lo + self.hi) / 2):
            x = np.r_[self.x[:6], np.clip(q, self.lo, self.hi)]
            m = self.keep(x, angle, self.lens, shape)
            if m is not None:
                k = m if k is None else k | m
        return k

    def crop_keep(self, angle: float, shape) -> np.ndarray | None:
        c = self.v["crop"]
        if c == "free":
            return self.keep_free(angle, shape)
        return self.keep(self.ref_x if c == "ref" else self.x, angle, self.lens, shape)

    def cache(self, name: str, angle: float) -> Path:
        v = self.v
        return self.cfg / "abl" / "cache" / f"{name}_{v['crop']}_z{v['zcut']:g}_p{v['pad']}_w{v['width']}_{angle:.1f}.npz"

    def learned(self, kind: str, angle: float, img: np.ndarray) -> np.ndarray:
        """The segmenter's probability at the fit width; cached on disk for the crops that do not follow the estimate."""
        cv2 = lf._cv2()
        f = self.cache(kind, angle) if self.v["crop"] != "live" else None
        if f is not None and f.exists():
            return np.load(f)["p"].astype(np.float32) / 255.0
        keep = self.crop_keep(angle, img.shape[:2])
        im = img.copy()
        if keep is not None:
            im[~keep] = 0
        if kind == "birefnet":
            p = self.segm("birefnet").prob(im)
        elif kind == "hqsam":
            x = self.x if self.v["crop"] == "live" else self.ref_x
            p = self.segm("hqsam").prob(im, self.label(x, angle, self.lens, im.shape[:2]), self.ids)
        elif kind == "hqsam_box":
            p = self.sam_box(im, self.keep_free(angle, im.shape[:2]))
        else:
            raise ValueError(kind)
        p = cv2.resize(p, (self.width, round(p.shape[0] * self.width / p.shape[1])), interpolation=cv2.INTER_AREA)
        if f is not None:
            f.parent.mkdir(parents=True, exist_ok=True)
            tmp = f.with_suffix(".tmp.npz")
            np.savez_compressed(tmp, p=np.round(p * 255).astype(np.uint8))
            tmp.replace(f)
        return p

    def sam_box(self, img: np.ndarray, region: np.ndarray | None) -> np.ndarray:
        """HQ-SAM with one box (the seed-free crop's bounding box) and no points: no URDF pose in the prompt."""
        import cv2
        sam = self.segm("hqsam")
        out = np.zeros(img.shape[:2], np.float32)
        if region is None or not region.any():
            return out
        ys, xs = np.nonzero(region)
        u0, v0, u1, v1 = xs.min(), ys.min(), xs.max(), ys.max()
        sam.pred.set_image(cv2.cvtColor(img[v0:v1 + 1, u0:u1 + 1], cv2.COLOR_BGR2RGB))
        m, sc, _ = sam.pred.predict(box=np.array([0, 0, u1 - u0, v1 - v0], float), multimask_output=True)
        out[v0:v1 + 1, u0:u1 + 1] = m[int(np.argmax(sc))].astype(np.float32)
        return out

    def grabcut_ref(self, angle: float, img: np.ndarray) -> np.ndarray:
        """GrabCut seeded at the primary solution, as a full-frame mask at the fit width (cached)."""
        cv2 = lf._cv2()
        f = self.cfg / "abl" / "cache" / f"grabcut_ref_z{self.v['zcut']:g}_p{self.v['pad']}_w{self.width}_{angle:.1f}.npz"
        if f.exists():
            return np.load(f)["p"].astype(np.float32)
        x, self.x = self.x, self.ref_x
        try:
            keep = self.keep(self.ref_x, angle, self.lens, img.shape[:2])
            im = img.copy()
            if keep is not None:
                im[~keep] = 0
            im = cv2.resize(im, (self.width, round(im.shape[0] * self.width / im.shape[1])), interpolation=cv2.INTER_AREA)
            view = lf.LiveHandFit.prepare(self, im, angle, self.lens_s)
        finally:
            self.x = x
        full = np.zeros(im.shape[:2], np.float32)
        (u0, v0), (h, w) = view.offset, view.fg.shape
        full[v0:v0 + h, u0:u0 + w] = view.fg
        f.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(f, p=full.astype(np.uint8))
        return full

    # views --------------------------------------------------------------------
    def views_of(self, frames):
        cv2 = lf._cv2()
        v = self.v
        self._prob = {}
        out = []
        for angle, img in frames:
            im = img
            if v["seg"] == "grabcut":
                keep = self.crop_keep(angle, img.shape[:2])
                if keep is not None:
                    im = img.copy()
                    im[~keep] = 0
            else:
                self._prob[angle] = self.learned(v["seg"], angle, img)
            if im.shape[1] != self.width:
                im = cv2.resize(im, (self.width, round(im.shape[0] * self.width / im.shape[1])), interpolation=cv2.INTER_AREA)
            view = self.neutral(self.prepare(im, angle, self.lens_s), angle, img)
            if v["color"]:
                (u0, v0), (h, w) = view.offset, view.fg.shape
                near = colour.near_mask(self, self.x, angle, self.lens_s, (h, w), view.offset, self.colour)
                view.color_px = colour.photo_lines(im[v0:v0 + h, u0:u0 + w], view.fg, near)
            view.w_fg = self.fg_weights(view)
            out.append(view)
        return out

    def residual_view(self, x, view, lens):
        """The study fit's residuals, plus (variant color) the colour-line term."""
        if not self.colour or self._colour_off or getattr(view, "color_px", None) is None:
            return super().residual_view(x, view, lens)
        cv2 = lf._cv2()
        poses = self.cam_poses(x, view.angle)
        uv = np.vstack([lens.project(T.apply_to_points(self.anchor[l])) for l, T in poses.items()]) - view.offset
        lab = self.paint(poses, lens, view.fg.shape, view.offset)
        r = self._terms(uv, lab, view)
        line = colour.model_lines(self, poses, lens, lab, view.offset, self.colour)
        d = cv2.distanceTransform(1 - line, cv2.DIST_L2, 3) if line.any() else np.full(line.shape, colour.CAP_PX, np.float32)
        px = view.color_px
        c = self.v["color"] * np.minimum(d[px[:, 1], px[:, 0]], colour.CAP_PX) if len(px) else np.zeros(0)
        return np.concatenate([r, np.pad(c, (0, colour.N_COLOR - len(c)))])

    def _rms(self, x, view, lens) -> float:
        self._colour_off = True                     # the silhouette misfit stays comparable across variants
        try:
            return super()._rms(x, view, lens)
        finally:
            self._colour_off = False

    def neutral(self, view, angle: float, img: np.ndarray):
        """Uncertain pixels neither pull nor push: out of the mask samples, edges and the inside term."""
        v = self.v
        if not v["band"] and not v["disagree"]:
            return view
        cv2 = lf._cv2()
        (u0, v0), (h, w) = view.offset, view.fg.shape
        sl = np.s_[v0:v0 + h, u0:u0 + w]
        bad = np.zeros((h, w), bool)
        if v["band"]:
            p = self._prob[angle][sl]
            bad |= (p > v["band"][0]) & (p < v["band"][1])
        if v["disagree"]:
            b = self.learned("birefnet", angle, img)[sl] > 0.5 if v["seg"] != "birefnet" else self._prob[angle][sl] > 0.5
            g = self.grabcut_ref(angle, img)[sl] > 0.5
            s = self.learned("hqsam", angle, img)[sl] > 0.5
            bad |= (b != g) | (b != s)
        if not bad.any():
            return view
        clipped = (view.to_fg == 0) & (view.fg == 0)
        fg = view.fg & ~bad
        to_fg = np.minimum(cv2.distanceTransform((1 - fg).astype(np.uint8), cv2.DIST_L2, 3), lf.CAP_PX).astype(np.float32)
        to_fg[clipped | bad] = 0.0
        e = view.edges & ~bad
        fy, fx = np.nonzero(fg)
        ey, ex = np.nonzero(e)
        out = lf.View(view.angle, view.offset, fg.astype(np.uint8), e.astype(np.uint8), to_fg,
                      lf._even(np.c_[fx, fy], lf.N_FG), lf._even(np.c_[ex, ey], lf.N_EDGE))
        out.bad_share = float(bad[view.fg > 0].mean()) if view.fg.any() else 0.0
        return out

    # outline offsets ------------------------------------------------------------
    def fit_offsets(self, views) -> dict[str, float]:
        """Joints and one outline offset per link (mm, bounded -1.5..3, prior sigma 1 mm) on these views."""
        from visual_calibration.solve import least_squares
        links = [l for l in self.cloud if l not in estimate.NO_JOINT_INFO]
        free = np.arange(6, self.x.size)
        base = self.x.copy()
        y0 = np.r_[self.x[free], [self.d.get(l, 0.0) for l in links]]
        lo = np.r_[self.lo, np.full(len(links), -1.5)]
        hi = np.r_[self.hi, np.full(len(links), 3.0)]
        step = np.r_[np.full(len(free), lf.JOINT_STEP), np.full(len(links), 0.3)]

        def residual(y):
            x = base.copy()
            x[free] = y[:len(free)]
            self.set_offsets(dict(zip(links, y[len(free):])))
            r = [self.residual_view(x, v, self.lens_s) for v in views]
            n = sum(len(q) for q in r)
            return np.concatenate([*r, self._prior(x, n), np.sqrt(n) * 0.05 * y[len(free):] / 1.0])

        f = least_squares(residual, y0, lo, hi, step=step, max_iter=estimate.ITERS)
        self.x[free] = f.x[:len(free)]
        d = dict(zip(links, f.x[len(free):].round(3).tolist()))
        self.set_offsets(d)
        return d


def apply_globals(v: dict) -> None:
    estimate.WEIGHTS = v["weights"]
    estimate.DROP_LINKS = tuple(v["drop_links"])
    estimate.Z_CUT_MM = float(v["zcut"])
    estimate.KEEP_PAD_PX = int(v["pad"])
    estimate.FULL_ONLY = False
    lf.W_EDGE = 0.7 if v["edge"] else 0.0


def frames_of(d: Path, n: int, test: bool):
    meta = json.loads((d / "meta.json").read_text())
    have = sorted(float(x["stop"]) for x in meta["views"])
    want = THREE if n == 3 else EIGHT if (n == 8 and test) else FIVE
    use = [s for s in have if any(abs(s - w) < 0.5 for w in want)]
    if len(use) < min(3, len(have)):
        use = have
    return load_frames(d, use)


def seed_of(d: Path, sel: dict, v: dict) -> tuple[dict, dict, np.ndarray | None]:
    plan = json.loads((d.parent.parent / "plan.json").read_text())
    meta = json.loads((d / "meta.json").read_text())
    L = LinearMap(STUDY / f"calib_{meta.get('calibration') or plan['calibration']}")
    q0 = L.joints_of(meta["before"]["motor_ids"], meta["before"]["motor_pos"])
    delta = {}
    if v["seed"] == "perturb":
        rng = np.random.default_rng(abs(hash(d.name + d.parent.parent.name)) % 2 ** 32)
        delta = {j: float(rng.uniform(-PERTURB_DEG, PERTURB_DEG)) for j in JOINTS}
        q0 = {j: q0[j] + delta[j] for j in JOINTS}
    return q0, delta, np.array(plan["mount"]) if plan.get("mount") else None


def iou_px(fit: AFit) -> tuple[list[float], list[int]]:
    ious, px = [], []
    for v in fit.last_views:
        m = fit.label(fit.x, v.angle, fit.lens_s, v.fg.shape, v.offset) > 0
        fg = v.fg > 0
        ious.append(float((m & fg).sum() / max((m | fg).sum(), 1)))
        px.append(int(fg.sum()))
    return ious, px


def one(study: str, key: str) -> None:
    v = variants(study)[key]
    sel = selection(study)
    estimate.RIG_AT = sel["rig_at"]
    apply_globals(v)
    fit = AFit(v)
    poses = ([*sel["train"]] if v["scope"] == "all" else []) + [*sel["test"]]
    if os.environ.get("STUDY_ABL_LIMIT"):                   # a quick check: the first N of train and of test
        n = int(os.environ["STUDY_ABL_LIMIT"])
        poses = [*(sel["train"][:n] if v["scope"] == "all" else []), *sel["test"][:n]]
    test = set(sel["test"])
    adir = STUDY / "protocol" / study / "ablate"
    adir.mkdir(parents=True, exist_ok=True)
    if v["offsets"]:
        f = adir / "offsets.json"
        if not f.exists():
            ds = []
            for d in even(sel["train"], 8):
                q0, _, mount = seed_of(d, sel, v)
                fit.ref_x = np.array(json.loads((d / "est.json").read_text())["x"])
                fit.cfg = d
                fit.set_offsets({l: 0.0 for l in fit.cloud})
                fit.solve(frames_of(d, 5, False), q0, mount, split=False)
                ds.append(fit.fit_offsets(fit.views_of(frames_of(d, 5, False))))
            f.write_text(json.dumps({l: round(float(np.median([x[l] for x in ds])), 3) for l in ds[0]}, indent=1))
        fit.set_offsets(json.loads(f.read_text()))
    mounts = {}
    if v["mount"] == "session":
        f = adir / "mounts.json"
        mounts = json.loads(f.read_text()) if f.exists() else {}
        for s in sorted({d.parent.parent for d in poses}):
            if s.name in mounts:
                continue
            ms = []
            for d in even([d for d in poses if d.parent.parent == s], 6):
                q0, _, _ = seed_of(d, sel, v)
                fit.ref_x, fit.cfg = np.array(json.loads((d / "est.json").read_text())["x"]), d
                ms.append(fit.solve(frames_of(d, 5, False), q0, None, split=False)["mount"])
            mounts[s.name] = np.median(ms, 0).round(3).tolist()
            f.write_text(json.dumps(mounts, indent=1))
    for d in poses:
        o = out_of(d, key)
        if o.exists():
            continue
        try:
            q0, delta, mount = seed_of(d, sel, v)
            if v["mount"] == "session":
                mount = np.array(mounts[d.parent.parent.name])
            fit.ref_x = np.array(json.loads((d / "est.json").read_text())["x"])
            if v["mount"] == "session":
                fit.ref_x[:6] = mount
            fit.cfg = d
            frames = frames_of(d, v["views"], d in test)
            res = fit.solve(frames, q0, mount, split=False)
            ious, px = iou_px(fit)
            dropped = []
            if v["drop_iou"] or v["drop_px"]:
                med = np.median(px)
                dropped = [a for (a, _), i, n in zip(frames, ious, px)
                           if (v["drop_iou"] and i < v["drop_iou"]) or (v["drop_px"] and n < v["drop_px"] * med)]
                if dropped and len(frames) - len(dropped) >= 3:
                    res = fit.solve([f for f in frames if f[0] not in dropped], q0, mount, split=False)
                    ious, px = iou_px(fit)
                elif dropped:
                    dropped = []
            res.update(assumed=seed_of(d, sel, {**v, "seed": "assumed"})[0], delta=delta, iou=[round(i, 3) for i in ious],
                       hand_px=px, dropped=dropped, variant=key,
                       bad_share=[round(getattr(x, "bad_share", 0.0), 3) for x in fit.last_views])
            if v["offsets"]:
                res["offsets"] = fit.d
            o.parent.mkdir(parents=True, exist_ok=True)
            tmp = o.with_suffix(".tmp")
            tmp.write_text(json.dumps(res, default=float))
            tmp.replace(o)
            print(time.strftime("%H:%M:%S"), key, d.parent.parent.name, d.name, f"{res['solve_s']} s", flush=True)
        except Exception:
            o.parent.mkdir(parents=True, exist_ok=True)
            (o.parent / f"{key}.error").write_text(traceback.format_exc())
            print(key, d.name, "error", traceback.format_exc().splitlines()[-1], flush=True)


# -- the pool ---------------------------------------------------------------------
def progress(study: str) -> dict:
    sel = selection(study)
    out = {}
    for k, v in variants(study).items():
        poses = ([*sel["train"]] if v["scope"] == "all" else []) + [*sel["test"]]
        out[k] = {"done": sum(out_of(d, k).exists() for d in poses), "n": len(poses)}
    return out


def run(study: str, procs: int, gpu: int, only: list[str] | None) -> None:
    keys = [k for k in variants(study) if not only or k in only]
    keys = [k for k in keys if progress(study)[k]["done"] < progress(study)[k]["n"]]
    log = (STUDY / "protocol" / study / "ablate")
    log.mkdir(parents=True, exist_ok=True)
    if not study.startswith("ser-") and "base" in keys:     # the shared caches first, then everything else
        subprocess.run([sys.executable, "-m", "hand_study.ablate", "one", study, "base"],
                       stdout=(log / "base.log").open("a"), stderr=subprocess.STDOUT)
        keys.remove("base")
    running: dict[str, subprocess.Popen] = {}
    env = {**os.environ, "PYTHONPATH": str(Path(__file__).resolve().parents[1])}
    while keys or running:
        for k, p in list(running.items()):
            if p.poll() is not None:
                running.pop(k)
        heavy = sum(k in GPU_HEAVY for k in running)
        for k in list(keys):
            if len(running) >= procs:
                break
            if k in GPU_HEAVY and heavy >= gpu:
                continue
            running[k] = subprocess.Popen(["nice", "-n", "10", sys.executable, "-m", "hand_study.ablate", "one", study, k],
                                          stdout=(log / f"{k}.log").open("a"), stderr=subprocess.STDOUT, env=env)
            heavy += k in GPU_HEAVY
            keys.remove(k)
        time.sleep(10)
    score(study)


# -- the score --------------------------------------------------------------------
def rows(ds: list[Path], key: str) -> list[dict]:
    R = []
    for s in sorted({d.parent.parent for d in ds}):
        mine = {d.name for d in ds if d.parent.parent == s}
        R += [r for r in analysis.rows([s], f"abl/{key}.json") if r["id"] in mine]
    return R


def score(study: str) -> dict:
    from hand_study.offline import Tips, acc, linear
    sel = selection(study)
    tips = Tips()
    base_map = linear(sel["rom"])
    vs = variants(study)
    out = {"study": study, "updated": time.strftime("%Y-%m-%d %H:%M:%S"), "n_train": len(sel["train"]),
           "n_test": len(sel["test"]), "groups": GROUPS, "progress": progress(study), "variants": []}
    ref_key = "live_area" if study.startswith("ser-") else "base"
    ref = {(r["session"], r["id"]): r for r in rows(sel["test"], ref_key)}
    for k, v in vs.items():
        te = rows(sel["test"], k)
        e = {"key": k, "group": v["group"], "label": v["label"], "n_test": len(te)}
        if not te:
            out["variants"].append(e)
            continue
        raw = [json.loads(out_of(d, k).read_text()) for d in sel["test"] if out_of(d, k).exists()]
        e["silhouette_px"] = round(float(np.median([r["rms_px"] for r in te])), 2)
        e["iou"] = round(float(np.median([i for x in raw for i in x.get("iou", [])] or [np.nan])), 3)
        e["dropped_views"] = int(sum(len(x.get("dropped") or []) for x in raw))
        e["solve_s"] = round(float(np.median([x["solve_s"] for x in raw])), 1)
        e["stops"] = {"deg": acc(te, base_map), "mm": tips.errors(te, base_map)}
        if v["scope"] == "all":
            tr = rows(sel["train"], k)
            e["n_train"] = len(tr)
            if len(tr) >= 10:
                X = {j: np.array([base_map(r)[j] for r in tr]) for j in JOINTS}
                Y = {j: np.array([r["measured"][j] for r in tr]) for j in JOINTS}
                ok = {j: np.ones(len(tr), bool) for j in JOINTS}
                cm = curve.Model(False)
                cm.fit(X, Y, ok)
                f = (lambda r: cm.forward(base_map(r)))
                e["curve"] = {"deg": acc(te, f), "mm": tips.errors(te, f)}
        by = {r["id"]: r for r in te}
        rep = [r["measured"][j] - by[r["id"][:-2]]["measured"][j] for r in te if r["id"].endswith("-r") and r["id"][:-2] in by
               for j in JOINTS]
        if rep:
            e["repeat_sd"] = round(float(np.sqrt(np.mean(np.square(rep)) / 2)), 2)
        sh = [r["measured"][j] - ref[(r["session"], r["id"])]["measured"][j] for r in te if (r["session"], r["id"]) in ref
              for j in JOINTS]
        if sh and k != ref_key:
            e["shift_vs_base"] = {"p50": round(float(np.median(np.abs(sh))), 2), "rms": round(float(np.sqrt(np.mean(np.square(sh)))), 2)}
        if v["seed"] == "perturb":
            prim = {("live" if v["crop"] == "live" and v["seg"] == "birefnet" else None)}
            unp = {"grabcut": "grabcut", "free": "unseeded"}.get(v["seg"] if v["seg"] == "grabcut" else v["crop"])
            num = den = 0.0
            per = {j: [0.0, 0.0] for j in JOINTS}
            for d in sel["test"]:
                if not out_of(d, k).exists():
                    continue
                p = json.loads(out_of(d, k).read_text())
                u = d / "est.json" if unp is None else out_of(d, unp)
                if not u.exists():
                    continue
                q = json.loads(u.read_text())["joints"]
                for j in JOINTS:
                    dd = p["delta"][j]
                    per[j][0] += (p["joints"][j] - q[j]) * dd
                    per[j][1] += dd * dd
                    num += (p["joints"][j] - q[j]) * dd
                    den += dd * dd
            del prim
            if den:
                e["pull"] = {"all": round(num / den, 3), **{j: round(a / b, 3) for j, (a, b) in per.items() if b}}
                e["pull_vs"] = unp or "est.json (the night's solve)"
        if v["offsets"] and raw:
            e["offsets_mm"] = raw[0].get("offsets")
        out["variants"].append(e)
    p = STUDY / "protocol" / study / "fit_ablation.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out, indent=1, default=float))
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=("run", "one", "score"))
    ap.add_argument("study")
    ap.add_argument("key", nargs="?")
    ap.add_argument("--procs", type=int, default=5)
    ap.add_argument("--gpu", type=int, default=2)
    ap.add_argument("--only", default="")
    a = ap.parse_args()
    if a.cmd == "one":
        one(a.study, a.key)
    elif a.cmd == "run":
        run(a.study, a.procs, a.gpu, [x for x in a.only.split(",") if x] or None)
    else:
        print(json.dumps({v["key"]: {k: v.get(k) for k in ("n_test", "silhouette_px")} for v in score(a.study)["variants"]}, indent=1))
