"""The hand's mount on the plate, solved once over every pose together (each pose's joints
held at their current estimate), instead of from the baseline pose alone. Alternate with a
re-solve of every pose's joints (worker) until the mount stops moving. Every camera's views
(meta.json `camera`, estimate.load_frames) join the one mount fit.

    python -m hand_study.bundle            # writes mount_bundle.json, sets each plan's mount
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from hand_study import analysis
from hand_study.estimate import StudyFit, load_frames
from visual_calibration import live_fit as lf
from visual_calibration.solve import least_squares

STUDY = analysis.STUDY
BASELINE = STUDY / "sessions/20261107-baseline-run1/configs/zero"


def poses() -> list[tuple[Path, Path]]:
    """(config dir, its estimate) of every solved pose, the baseline included."""
    out = [(BASELINE, BASELINE / "est_w960.json")]
    for s in analysis.sessions():
        out += [(d, d / "est.json") for d in sorted((s / "configs").glob("*")) if (d / "est.json").exists()]
    return out


def main(iters: int = 8) -> dict:
    t0 = time.time()
    f = StudyFit()
    items = []
    for d, e in poses():
        est = json.loads(e.read_text())
        x = np.array(est["x"], float)
        f.x = x.copy()
        items.append((d, x[6:], f.views_of(load_frames(d))))
    per_cam = {}
    for *_, views in items:
        for v in views:
            per_cam[v.camera] = per_cam.get(v.camera, 0) + 1
    print(f"{len(items)} poses, {sum(len(v) for *_, v in items)} views prepared in {time.time() - t0:.0f} s"
          + (f" (per camera {per_cam})" if len(per_cam) > 1 else ""), flush=True)
    x0 = np.array(json.loads((BASELINE / "est_w960.json").read_text())["mount"], float)

    def per_pose(mount):
        return [np.concatenate([f.residual_view(np.r_[mount, q], v, f.lens_s) for v in views]) for _, q, views in items]

    def residual(mount):
        r = per_pose(mount)
        n = sum(len(x) for x in r)
        return np.concatenate([*r, lf.MOUNT_PRIOR * np.sqrt(n) * mount / lf.BOUND])

    def rms(mount):
        """Per pose, the silhouette misfit (inside + cover terms), px at 3840 width."""
        out = []
        for _, q, views in items:
            out.append(float(np.mean([f._rms(np.r_[mount, q], v, f.lens_s) for v in views]) / f.s))
        return out

    before = rms(x0)
    fit = least_squares(residual, x0, -lf.BOUND, lf.BOUND, step=lf.MOUNT_STEP, max_iter=iters)
    after = rms(fit.x)
    out = {"mount_before": x0.round(3).tolist(), "mount": fit.x.round(3).tolist(),
           "change": {"rotation_deg": (fit.x[:3] - x0[:3]).round(3).tolist(),
                      "translation_mm": (fit.x[3:6] - x0[3:6]).round(3).tolist()},
           "std": (fit.std * lf.SIGMA_PX / max(fit.rms, 1e-6)).round(4).tolist(),
           "rms_before": float(np.mean(before)), "rms_after": float(np.mean(after)),
           "poses": [{"id": d.name, "before": b, "after": a} for (d, *_), b, a in zip(items, before, after)],
           "iterations": fit.iterations, "seconds": round(time.time() - t0)}
    history = json.loads((STUDY / "mount_bundle.json").read_text()).get("history", []) \
        if (STUDY / "mount_bundle.json").exists() else []
    out["history"] = history + [{k: out[k] for k in ("mount_before", "mount", "rms_before", "rms_after")}]
    (STUDY / "mount_bundle.json").write_text(json.dumps(out, indent=1))
    for s in analysis.sessions():
        p = s / "plan.json"
        plan = json.loads(p.read_text())
        plan.setdefault("mount_baseline", plan.get("mount"))
        plan["mount"] = out["mount"]
        p.write_text(json.dumps(plan, indent=1))
    return out


if __name__ == "__main__":
    o = main()
    print(json.dumps({k: o[k] for k in ("mount_before", "mount", "change", "std", "rms_before", "rms_after",
                                          "iterations", "seconds")}, indent=1))
