"""Plate stops per pose that show the whole hand (estimate.FULL_ONLY): predicted from the
commanded pose and the fixed mount before collecting. A standard stop that cuts the hand off is
replaced by the nearest stop that shows it all; a pose that cannot get MIN_FULL_VIEWS is dropped.

    python -m hand_study.views SESSION... [--dry]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from hand_study.estimate import FULL_MARGIN_PX, MIN_FULL_VIEWS, StudyFit

CANDIDATES = np.arange(0.0, 320.1, 7.5)
KEEP_PX = -10.0         # a standard stop predicted this far inside is still shot (the fit decides) ...
SURE_PX = 25.0          # ... and below this it gets a second stop next to it that is surely full (the
                        # commanded pose is off the real one by up to ~20 px at the thumb tip)
MAX_VIEWS = 7
TRAVEL = (0.0, 320.0)   # stops the plate can reach now (--travel: ser-7577's plate blocks below ~48°)
MIN_SEP_DEG = 30.0


def plan_views(fit: StudyFit, x: np.ndarray, stops: list[float]) -> tuple[list[float], dict]:
    m = {}

    def margin(a):
        if a not in m:
            d = fit.margins(x, a)
            m[a] = min(d.values())
        return m[a]
    if TRAVEL != (0.0, 320.0):
        for a in stops:
            if not TRAVEL[0] <= a <= TRAVEL[1]:
                m[a] = -np.inf
    keep = [a for a in stops if margin(a) >= KEEP_PX]
    cut = [a for a in stops if a not in keep]
    for a in [a for a in stops if margin(a) < SURE_PX]:
        if len(keep) >= MAX_VIEWS:
            break
        for c in sorted((c for c in CANDIDATES if TRAVEL[0] <= c <= TRAVEL[1]), key=lambda c: abs(c - a)):
            if all(abs(c - k) >= MIN_SEP_DEG or (k == a and abs(c - k) >= 7.5) for k in keep) \
                    and margin(float(c)) >= SURE_PX:
                keep.append(float(c))
                break
    return sorted(keep), {"cut": {f"{a:g}": round(m[a], 1) for a in cut},
                          "full": sum(margin(a) >= FULL_MARGIN_PX for a in keep),
                          "margin": {f"{a:g}": round(m[a], 1) for a in sorted(keep)}}


def main(sessions: list[Path], mount: list[float] | None = None, dry: bool = False, log=print) -> dict:
    fit = StudyFit()
    stats = {}
    for s in sessions:
        plan = json.loads((s / "plan.json").read_text())
        mt = np.array(mount if mount is not None else plan["mount"], float)
        n_cut = n_drop = n_new = 0
        for c in plan["configs"]:
            q = np.clip([float(c["q"].get(j, 0.0)) for j in fit.joints], fit.lo, fit.hi)
            angles, info = plan_views(fit, np.r_[mt, q], plan["angles"])
            c["angles"], c["views_info"] = angles, info
            n_cut += len(info["cut"])
            n_new += len([a for a in angles if a not in plan["angles"]])
            if info["full"] < MIN_FULL_VIEWS:
                c["skip"] = f"only {info['full']} predicted views show the whole hand"
                n_drop += 1
            else:
                c.pop("skip", None)
        stats[s.name] = {"poses": len(plan["configs"]), "views_cut": n_cut, "views_added": n_new, "poses_dropped": n_drop}
        log(f"{s.name}: {stats[s.name]}")
        if not dry:
            plan["views_planned"] = True
            (s / "plan.json").write_text(json.dumps(plan, indent=1))
    return stats


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("sessions", type=Path, nargs="+")
    p.add_argument("--mount", type=float, nargs=6)
    p.add_argument("--dry", action="store_true")
    p.add_argument("--travel", type=float, nargs=2)
    a = p.parse_args()
    if a.travel:
        TRAVEL = tuple(a.travel)
    main(a.sessions, a.mount, a.dry)
