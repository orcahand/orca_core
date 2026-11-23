"""Build a study plan (plan.json) for one calibration run: per-joint sweeps up and down,
coupling grids, repeats approached from both sides, random poses; collision-free only.

    python -m hand_study.plan SESSION_DIR --calibration run1 --part A [--pilot]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from hand_study.configs import JOINTS, REST, ROM, Collider

ANGLES = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
PARTS = {"A": ["wrist", "thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip", "index_abd", "index_mcp", "index_pip",
               "middle_abd", "middle_mcp", "middle_pip"],
         "B": ["ring_abd", "ring_mcp", "ring_pip", "pinky_abd", "pinky_mcp", "pinky_pip"]}
ABDS = ("index_abd", "middle_abd", "ring_abd", "pinky_abd")
APPROACH_DEG = 15.0
STUDY = Path.home() / ".orca_ui/visual_calibration/hand_study"


def points(j: str) -> list[float]:
    lo, hi = ROM[j]
    n = 7 if j.endswith("_abd") and j != "thumb_abd" else 8 if j in ("thumb_cmc", "thumb_abd") else 9
    return np.linspace(lo, hi, n).round(1).tolist()


class Builder:
    def __init__(self):
        self.c = Collider()
        self.out: list[dict] = []
        self.skipped: list[dict] = []

    def add(self, cid: str, q: dict, **tags) -> bool:
        full = {**REST, **q}
        gap, pair = self.c.clearance(full)
        if gap < 0:
            self.skipped.append({"id": cid, "q": q, "pair": pair, "gap": round(gap, 1)})
            return False
        self.out.append({"id": cid, "q": full, **tags})
        return True

    def free_with_neighbour(self, j: str, v: float, base: dict) -> dict | None:
        q = {**base, j: v}
        if self.c.free(q):
            return q
        if j in ABDS:
            for f in (0.3, 0.6, 0.8, 1.0):
                fan = {**q, **{o: float(np.clip(round(f * v, 1), *ROM[o])) for o in ABDS if o != j}}
                if self.c.free(fan):
                    return fan
        if j.startswith("thumb") or j == "wrist":
            return None
        for tv in ({"thumb_abd": 20.0}, {"thumb_cmc": -30.0}, {"thumb_cmc": -30.0, "thumb_abd": 30.0}):
            if self.c.free({**q, **tv}):
                return {**q, **tv}
        return None

    def sweep(self, j: str, base: dict | None = None, tag: str = "sweep"):
        base = dict(base or {})
        for direction, pts in (("up", points(j)), ("down", points(j)[::-1])):
            for k, v in enumerate(pts):
                q = self.free_with_neighbour(j, v, base)
                cid = f"{tag}-{j}-{direction}-{k:02d}"
                if q is None:
                    self.skipped.append({"id": cid, "q": {j: v}, "pair": "no free neighbour", "gap": None})
                    continue
                self.add(cid, q, kind=tag, joint=j, dir=direction, value=v)

    def grid(self, name: str, a: str, b: str, na: int = 4, nb: int = 4, base: dict | None = None):
        va = np.linspace(*ROM[a], na + 2)[1:-1].round(1)
        vb = np.linspace(*ROM[b], nb + 2)[1:-1].round(1)
        for i, x in enumerate(va):
            for k, y in enumerate(vb if i % 2 == 0 else vb[::-1]):
                self.add(f"grid-{name}-{i}{k}", {**(base or {}), a: float(x), b: float(y)},
                         kind="grid", joints=[a, b], grid=name)

    def repeats(self, refs: dict[str, dict], visit: int):
        for name, q in refs.items():
            for side, sgn in (("below", -1.0), ("above", 1.0)):
                via = {j: float(np.clip(v + sgn * APPROACH_DEG, *ROM[j])) for j, v in q.items()}
                if not self.c.free(via):
                    via = {j: float(np.clip(v + sgn * APPROACH_DEG / 2, *ROM[j])) for j, v in q.items()}
                if self.add(f"rep-{name}-{side}-{visit}", q, kind="repeat", ref=name, side=side, visit=visit):
                    self.out[-1]["via"] = [{**REST, **via}]

    def near_touch(self, fingers: list[str], fractions=(0.4, 0.6, 0.75, 0.85, 0.92, 0.97)):
        """Between the open hand and each thumb-to-fingertip touch pose, as close as the margins allow."""
        poses = json.loads((STUDY / "touch_poses.json").read_text())
        for f in fingers:
            q = poses[f]["q"]
            for k, a in enumerate(fractions):
                self.add(f"touch-{f}-{k}", {j: round(a * v, 1) for j, v in q.items()}, kind="touch", finger=f, frac=a)

    def random(self, n: int, joints: list[str], seed: int, tag: str = "random"):
        rng = np.random.default_rng(seed)
        k = 0
        tries = 0
        while k < n and tries < 200 * n:
            tries += 1
            q = {j: float(np.round(rng.uniform(*ROM[j]), 1)) for j in joints}
            if self.add(f"{tag}-{k:03d}", q, kind=tag):
                k += 1
            else:
                self.skipped.pop()


REFS_A = {"open": {}, "pinch": {"thumb_cmc": -20.0, "thumb_abd": 30.0, "thumb_mcp": 30.0, "thumb_dip": 30.0,
                               "index_mcp": 40.0, "index_pip": 40.0},
          "mid": {"wrist": -15.0, "index_mcp": 50.0, "index_pip": 50.0, "middle_mcp": 50.0, "middle_pip": 50.0,
                  "index_abd": 10.0, "middle_abd": 0.0}}


SPARSE_ANGLES = [45.0, 112.5, 180.0, 225.0, 292.5]
LEVELS = (0.1, 0.5, 0.9)
FLEXORS = [f"{f}_{k}" for f in ("index", "middle", "ring", "pinky") for k in ("mcp", "pip")]


def level(j: str, i: int) -> float:
    lo, hi = ROM[j]
    return round(lo + LEVELS[i % 3] * (hi - lo), 1)


def build_sparse() -> Builder:
    """All 17 joints at 10 / 50 / 90 % of their range, a few joints at a time, ~26 poses."""
    b = Builder()

    def add_backoff(cid, q, **tags):
        for shrink in (1.0, 0.85, 0.7, 0.55):
            qq = {j: round(v * shrink, 1) if j not in ("wrist",) else v for j, v in q.items()}
            if b.c.free({**REST, **qq}):
                return b.add(cid, qq, **tags, shrink=shrink)
        b.skipped.append({"id": cid, "q": q, "pair": "no free shrink", "gap": None})
        return False

    b.add("open-0", {}, kind="repeat", ref="open", side="start", visit=0)
    for shift in (1, 2):
        for i in range(3):
            q = {j: level(j, i if j.endswith("mcp") else i + shift) for j in FLEXORS}
            add_backoff(f"flex-{shift}{i}", q, kind="sparse", group="fingers")
    for i in range(3):
        q = b.free_with_neighbour("index_abd", level("index_abd", i), {})
        if q is not None:
            fan = {o: level(o, i) for o in ABDS}
            ok = b.c.free({**REST, **fan})
            b.add(f"abd-{i}", fan if ok else q, kind="sparse", group="abduction")
    thumb = ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"]
    for shift in (0, 1):
        for i in range(3):
            q = {j: level(j, i + (k * (1 + shift)) % 3) for k, j in enumerate(thumb)}
            add_backoff(f"thumb-{shift}{i}", q, kind="sparse", group="thumb")
    for i in range(3):
        b.add(f"wrist-{i}", {"wrist": level("wrist", i)}, kind="sparse", group="wrist")
    b.random(6, JOINTS, seed=7, tag="mix")
    b.add("open-1", {}, kind="repeat", ref="open", side="end", visit=1)
    return b


def build(part: str, pilot: bool) -> Builder:
    b = Builder()
    if pilot:
        b.repeats(REFS_A, 0)
        b.sweep("index_mcp")
        return b
    js = PARTS[part]
    b.repeats(REFS_A, 0)
    for k, j in enumerate(js):
        b.sweep(j)
        if k % 4 == 3:
            b.repeats(REFS_A, k // 4 + 1)
    if part == "A":
        b.grid("index", "index_mcp", "index_pip")
        b.grid("middle", "middle_mcp", "middle_pip")
        b.grid("thumb_flex", "thumb_mcp", "thumb_dip")
        b.grid("thumb_base", "thumb_cmc", "thumb_abd")
        b.grid("wrist_index", "wrist", "index_mcp", 4, 3)
        b.grid("wrist_middle", "wrist", "middle_pip", 4, 3)
        b.grid("index_abd_mcp", "index_abd", "index_mcp", 3, 3)
    b.near_touch(["index", "middle"] if part == "A" else ["ring", "pinky"])
    b.repeats(REFS_A, 9)
    b.random(60, js, seed=1 if part == "A" else 2, tag=f"random{part}")
    b.repeats(REFS_A, 10)
    return b


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("session", type=Path)
    p.add_argument("--calibration", default="run1")
    p.add_argument("--part", default="A")
    p.add_argument("--pilot", action="store_true")
    p.add_argument("--sparse", action="store_true")
    p.add_argument("--mount", type=Path, help="baseline est json whose mount is held fixed")
    a = p.parse_args()
    b = build_sparse() if a.sparse else build(a.part, a.pilot)
    a.session.mkdir(parents=True, exist_ok=True)
    mount = json.loads(a.mount.read_text())["mount"] if a.mount else None
    (a.session / "plan.json").write_text(json.dumps(
        {"calibration": a.calibration, "part": "sparse" if a.sparse else a.part, "pilot": a.pilot,
         "angles": SPARSE_ANGLES if a.sparse else ANGLES, "mount": mount,
         "joints": JOINTS, "configs": b.out, "skipped": b.skipped}, indent=1))
    print(len(b.out), "configs,", len(b.skipped), "skipped")
