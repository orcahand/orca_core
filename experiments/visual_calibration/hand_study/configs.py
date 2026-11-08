"""Study plans: per-joint sweeps (both directions), coupling grids, repeats approached from
both sides, and random space-filling poses, all checked against self-collision on the URDF
meshes with a margin (no finger contact)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from visual_calibration.hand_model import HandModel
from visual_calibration.live_fit import surface

JOINTS = ["wrist", "thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip", "index_abd", "index_mcp", "index_pip",
          "middle_abd", "middle_mcp", "middle_pip", "ring_abd", "ring_mcp", "ring_pip", "pinky_abd",
          "pinky_mcp", "pinky_pip"]
ROM = {"wrist": (-65, 35), "thumb_cmc": (-45, 33), "thumb_abd": (-18, 55), "thumb_mcp": (-25, 100),
       "thumb_dip": (-15, 107), "index_abd": (-25, 30), "middle_abd": (-27, 27), "ring_abd": (-27, 27),
       "pinky_abd": (-30, 30), **{f"{f}_mcp": (-25, 100) for f in ("index", "middle", "ring", "pinky")},
       **{f"{f}_pip": (-15, 107) for f in ("index", "middle", "ring", "pinky")}}
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
MARGIN_MM = 6.0
NEED = {"ap": 1.5, "tp": 1.5, "pp": 3.0, "dp": 5.0}
REST = {j: 0.0 for j in JOINTS}


def finger_of(link: str) -> str:
    return link.split("_", 1)[0] if link.split("_", 1)[0] in FINGERS else link


class Collider:
    def __init__(self, spacing: float = 2.5):
        self.model = HandModel.load("right")
        self.cloud = {l: surface(v.path, v.origin, v.scale, spacing) for l, v in self.model.visuals.items()}
        links = list(self.cloud)
        tips = [l for l in links if l.endswith("_dp") or l == "thumb_pp"]
        self.pairs = [(a, b) for i, a in enumerate(links) for b in links[i + 1:]
                      if finger_of(a) != finger_of(b) and (finger_of(a) in FINGERS or finger_of(b) in FINGERS)
                      and not (a == "carpals" and b not in tips) and not (b == "carpals" and a not in tips)
                      and not ({finger_of(a), finger_of(b)} & {"tower", "forearm"} and not {a, b} & set(tips))]

        self.margin = {}
        rest = self.distances(REST)
        self.margin = {p: min(self.need(p), 0.8 * rest.get(p, np.inf)) for p in self.pairs}

    @staticmethod
    def need(pair: tuple[str, str]) -> float:
        """Knuckle blocks pass close by design and barely move with an angle error; the
        error grows along the finger, so do the margins."""
        far = max(NEED.get(l.rsplit("_", 1)[-1], MARGIN_MM) for l in pair if "_" in l) if any("_" in l for l in pair) else MARGIN_MM
        return far

    def clearance(self, q: dict[str, float]) -> tuple[float, tuple[str, str]]:
        """The smallest gap less its pair's margin (mm; < 0: too close), and that pair."""
        d = self.distances(q)
        worst = min(d, key=lambda p: d[p] - self.margin[p], default=None)
        return (np.inf, ("", "")) if worst is None else (float(d[worst] - self.margin[worst]), worst)

    def distances(self, q: dict[str, float]) -> dict[tuple[str, str], float]:
        """Gaps (mm) of the pairs closer than MARGIN_MM + 1."""
        poses = self.model.link_poses(q)
        pts = {l: poses[l].rotation @ c.T * 1.0 for l, c in self.cloud.items()}
        pts = {l: (p.T + poses[l].translation * 1000.0) for l, p in pts.items()}
        box = {l: (p.min(0), p.max(0)) for l, p in pts.items()}
        out = {}
        for a, b in self.pairs:
            gap = np.maximum(box[a][0] - box[b][1], box[b][0] - box[a][1]).max()
            if gap > MARGIN_MM:
                continue
            A, B = pts[a], pts[b]
            lo, hi = np.maximum(box[a][0], box[b][0]) - MARGIN_MM, np.minimum(box[a][1], box[b][1]) + MARGIN_MM
            A = A[np.all((A >= lo) & (A <= hi), 1)]
            B = B[np.all((B >= lo) & (B <= hi), 1)]
            if not len(A) or not len(B):
                continue
            d = np.sqrt(((A[:, None, :] - B[None, :, :]) ** 2).sum(-1).min()) if len(A) * len(B) < 4e6 else \
                min(np.sqrt(((A[i:i + 500, None, :] - B[None, :, :]) ** 2).sum(-1).min()) for i in range(0, len(A), 500))
            if d < MARGIN_MM + 1:
                out[(a, b)] = float(d)
        return out

    def free(self, q: dict[str, float]) -> bool:
        return self.clearance({**REST, **q})[0] >= 0
