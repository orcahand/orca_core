"""Thumb-to-fingertip touch poses from the URDF: the thumb's and the finger's distal pads
press lightly (1 mm overlap on the model), every other link stays clear.

    python -m hand_study.touch        # writes touch_poses.json
"""

from __future__ import annotations

import json

import numpy as np

from hand_study.configs import REST, ROM, Collider
from hand_study.paths import STUDY
from visual_calibration.solve import least_squares

THUMB = ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"]
GAP_MM = -1.0              # the pads 1 mm into each other on the model: a light press when the hand is right


def min_dist(c: Collider, q: dict, a: str, b: str) -> float:
    poses = c.model.link_poses(q)
    A = c.cloud[a] @ poses[a].rotation.T + poses[a].translation * 1000
    B = c.cloud[b] @ poses[b].rotation.T + poses[b].translation * 1000
    return float(np.sqrt(((A[:, None] - B[None]) ** 2).sum(-1).min()))


def solve(c: Collider, finger: str, seed: int = 0) -> dict:
    names = THUMB + [f"{finger}_mcp", f"{finger}_pip"]
    lo = np.array([ROM[j][0] for j in names])
    hi = np.array([ROM[j][1] for j in names])
    rng = np.random.default_rng(seed)
    pairs = [p for p in c.pairs if set(p) != {"thumb_dp", f"{finger}_dp"}]

    def q_of(x):
        return {**REST, **dict(zip(names, x))}

    def residual(x):
        q = q_of(x)
        d = min_dist(c, q, "thumb_dp", f"{finger}_dp")
        dist = c.distances(q)
        clear = [min(0.0, dist[p] - c.margin[p]) if p in dist else 0.0 for p in pairs]
        return np.r_[d - GAP_MM, 3 * np.array(clear), 0.01 * (x - (lo + hi) / 2)]

    best = None
    for _ in range(300):
        x = rng.uniform(lo, hi)
        r = residual(x)
        cost = float(r @ r)
        if best is None or cost < best[0]:
            best = (cost, x)
    f = least_squares(residual, best[1], lo, hi, step=np.full(len(names), 1.0), max_iter=40)
    q = q_of(f.x)
    return {"finger": finger, "q": {j: round(float(v), 1) for j, v in q.items()},
            "gap_mm": round(min_dist(c, q, "thumb_dp", f"{finger}_dp"), 2), "clear": c.clearance(q)}


if __name__ == "__main__":
    c = Collider(spacing=1.5)
    out = {}
    for fng in ("index", "middle", "ring", "pinky"):
        r = solve(c, fng)
        out[fng] = r
        print(fng, r["gap_mm"], r["clear"], {k: v for k, v in r["q"].items() if v})
    (STUDY / "touch_poses.json").write_text(json.dumps(out, indent=1))
