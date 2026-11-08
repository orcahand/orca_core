"""Joint angles of one configuration from its multi-view photos (the mount held fixed, or
solved with it), with per-joint uncertainty and a view-split consistency check."""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from visual_calibration import live_fit as lf
from visual_calibration.live_fit import LiveHandFit, scaled
from visual_calibration.rig_config import RigConfig
from visual_calibration.solve import least_squares

WIDTH = 960
SPACING_MM = 0.9
ITERS = 12


class StudyFit(LiveHandFit):
    def __init__(self, width: int = WIDTH, spacing: float = SPACING_MM):
        super().__init__(RigConfig.load(), spacing=spacing)
        self.width = width
        self.s = width / self.lens.size[0]
        self.lens_s = scaled(self.lens, self.s)

    def views_of(self, frames: list[tuple[float, np.ndarray]]) -> list:
        cv2 = lf._cv2()
        out = []
        for angle, img in frames:
            if img.shape[1] != self.width:
                img = cv2.resize(img, (self.width, round(img.shape[0] * self.width / img.shape[1])),
                                 interpolation=cv2.INTER_AREA)
            out.append(self.prepare(img, angle, self.lens_s))
        return out

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

    def per_view_rms(self, views) -> list[float]:
        return [round(self._rms(self.x, v, self.lens_s) / self.s, 2) for v in views]

    def solve(self, frames, q0: dict, mount: np.ndarray | None, rounds: int = 2, split: bool = True) -> dict:
        """mount None: solve it too (a baseline). Two rounds: the segmentation is re-seeded
        from the first round's pose."""
        t0 = time.perf_counter()
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
        out = {"x": self.x.tolist(), "joints": dict(zip(self.joints, self.x[6:].round(3).tolist())),
               "std": dict(zip(self.joints, np.nan_to_num(std[6:], nan=-1).round(3).tolist())),
               "mount": self.x[:6].round(3).tolist(), "mount_std": np.nan_to_num(std[:6], nan=-1).round(3).tolist(),
               "spread": {k: round(v, 1) for k, v in self.spread.items()},
               "rms_px": float(np.mean(rms)), "view_rms_px": rms, "angles": [a for a, _ in frames],
               "quality": quality, "suspect": suspect(quality),
               "obs": self.observability([a for a, _ in frames], self.lens_s,
                                         (round(self.lens.size[1] * self.s), self.width)).max(0).round(3).tolist()}
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
        return out


    def quality(self, views) -> list[dict]:
        """Per view, mask vs. model silhouette at the solution: the share of the mask the model
        does not explain (rods, cables, shadows) and the share of the model's fingers the mask misses."""
        out = []
        fingers = {self.ids[l] for l in self.cloud if lf.group_of(l) >= 2}
        for v in views:
            lab = self.label(self.x, v.angle, self.lens_s, v.fg.shape, v.offset)
            model, fg = lab > 0, v.fg > 0
            fin = np.isin(lab, list(fingers))
            out.append({"angle": round(float(v.angle), 1),
                        "mask_only": round(float((fg & ~model).sum() / max(fg.sum(), 1)), 3),
                        "model_only": round(float((model & ~fg).sum() / max(model.sum(), 1)), 3),
                        "finger_missed": round(float((fin & ~fg).sum() / max(fin.sum(), 1)), 3)})
        return out

    def save_views(self, path: Path) -> None:
        v = self.last_views
        np.savez_compressed(path, angle=[x.angle for x in v], offset=np.array([x.offset for x in v]),
                            **{f"fg{k}": x.fg.astype(bool) for k, x in enumerate(v)},
                            **{f"edges{k}": x.edges.astype(bool) for k, x in enumerate(v)})


MASK_ONLY_MAX = 0.15
FINGER_MISSED_MAX = 0.25


def suspect(quality: list[dict]) -> list[str]:
    """Why the masks of a pose are not to be trusted (empty: fine)."""
    why = []
    for q in quality:
        if q["mask_only"] > MASK_ONLY_MAX:
            why.append(f"{q['angle']:.0f}°: {q['mask_only']:.0%} of the mask not hand (rods, cables, shadow?)")
        if q["finger_missed"] > FINGER_MISSED_MAX:
            why.append(f"{q['angle']:.0f}°: mask misses {q['finger_missed']:.0%} of the fingers")
    return why


def load_frames(cfg_dir: Path, stops: list[float] | None = None) -> list[tuple[float, np.ndarray]]:
    cv2 = lf._cv2()
    meta = json.loads((cfg_dir / "meta.json").read_text())
    views = sorted(meta["views"], key=lambda v: v["stop"])
    if stops is not None:
        views = [v for v in views if v["stop"] in stops]
    return [(float(v["angle"]), cv2.imread(str(cfg_dir / v["file"]))) for v in views]
