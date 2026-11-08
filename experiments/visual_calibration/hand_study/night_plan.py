"""The overnight study's plans (STUDY.md): one session folder per block, every pose approached
from the same side (from below, through a via pose), the validation set identical in every
V block, plus the wear programmes.

    python -m hand_study.night_plan NIGHT_TAG       # writes sessions/<tag>-N##-*/plan.json + night/<tag>.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from hand_study.configs import JOINTS, REST, ROM, Collider
from hand_study.plan import ABDS, REFS_A, SPARSE_ANGLES, STUDY, Builder

APPROACH_DEG = 8.0
FLEXORS = [f"{f}_{k}" for f in ("index", "middle", "ring", "pinky") for k in ("mcp", "pip")]
THUMB = ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"]
S_PER_POSE = 40.0


def approach(b: Builder, start: int = 0) -> None:
    """Every pose without its own via gets one APPROACH_DEG below it on every joint (half that
    if not collision-free), so each pose is reached from the same side."""
    for c in b.out[start:]:
        if c.get("via"):
            continue
        for d in (APPROACH_DEG, APPROACH_DEG / 2):
            via = {j: float(np.clip(v - d, *ROM[j])) for j, v in c["q"].items()}
            if b.c.free(via):
                c["via"], c["approach"] = [via], f"below {d:g}"
                break
        else:
            c["approach"] = "direct"


def points(j: str, n: int) -> list[float]:
    return np.linspace(*ROM[j], n).round(1).tolist()


def sweep_up(b: Builder, j: str, n: int, tag: str) -> None:
    for k, v in enumerate(points(j, n)):
        q = b.free_with_neighbour(j, v, {})
        if q is None:
            b.skipped.append({"id": f"{tag}-{j}-{k:02d}", "q": {j: v}, "pair": "no free neighbour", "gap": None})
            continue
        b.add(f"{tag}-{j}-{k:02d}", q, kind="sweep", joint=j, dir="up", value=v)


def n_points(j: str) -> int:
    return 7 if j in ABDS else 8 if j in ("thumb_cmc", "thumb_abd") else 9


def dense() -> Builder:
    """~3 h: every joint over its full range (ends = the configured stops), coupling grids, a
    hysteresis subset (repeats from both sides) and random poses."""
    b = Builder()
    b.repeats(REFS_A, 0)
    for j in JOINTS:
        sweep_up(b, j, n_points(j), "sweep")
    for f in ("index", "middle", "ring", "pinky"):
        b.grid(f, f"{f}_mcp", f"{f}_pip", 3, 3)
    b.grid("thumb_flex", "thumb_mcp", "thumb_dip", 3, 3)
    b.grid("thumb_base", "thumb_cmc", "thumb_abd", 3, 3)
    b.grid("wrist_index", "wrist", "index_mcp", 3, 3)
    b.grid("wrist_middle", "wrist", "middle_pip", 3, 3)
    b.grid("index_abd_mcp", "index_abd", "index_mcp", 3, 3)
    b.grid("ring_abd_mcp", "ring_abd", "ring_mcp", 3, 3)
    b.repeats(REFS_A, 1)
    b.random(22, JOINTS, seed=21, tag="random")
    b.repeats(REFS_A, 2)
    approach(b)
    return b


def validation() -> Builder:
    """The fixed validation set (identical in every V block): 2x open, 12 random, 10 at the
    stops, 3 references."""
    b = Builder()
    b.add("v-open-a", {}, kind="val", group="open")
    b.random(12, JOINTS, seed=99, tag="v-random")
    for j, side in (("index_abd", 1), ("pinky_abd", 0), ("middle_abd", 0), ("ring_abd", 1)):
        q = b.free_with_neighbour(j, float(ROM[j][side]), {})
        if q is not None:
            b.add(f"v-stop-{j}-{'hi' if side else 'lo'}", q, kind="val", group="stop")
    for j in ("index_mcp", "middle_pip", "ring_mcp", "pinky_pip"):
        b.add(f"v-stop-{j}-hi", {j: float(ROM[j][1])}, kind="val", group="stop")
    b.add("v-stop-flex-lo", {j: float(ROM[j][0]) for j in FLEXORS + ["wrist"]}, kind="val", group="stop")
    b.add("v-stop-thumb", {"thumb_cmc": float(ROM["thumb_cmc"][0]), "thumb_abd": float(ROM["thumb_abd"][1]),
                           "thumb_mcp": 50.0, "thumb_dip": float(ROM["thumb_dip"][1]) - 7}, kind="val", group="stop")
    for name, q in REFS_A.items():
        if name != "open":
            b.add(f"v-ref-{name}", q, kind="val", group="ref")
    b.add("v-mix", {"wrist": 20.0, "index_mcp": 30.0, "middle_mcp": 60.0, "ring_pip": 40.0,
                    "thumb_abd": 40.0, "thumb_mcp": 20.0}, kind="val", group="ref")
    b.add("v-open-b", {}, kind="val", group="open")
    approach(b)
    return b


def short() -> Builder:
    """The ~50 min visual calibration: 4 points per joint over the full range, 6 random."""
    b = Builder()
    b.add("open-0", {}, kind="repeat", ref="open", side="start", visit=0)
    for j in JOINTS:
        sweep_up(b, j, 4, "short")
    b.random(6, JOINTS, seed=33, tag="short-random")
    approach(b)
    return b


def squeeze(c: Collider, a: str, b_: str) -> dict | None:
    """Two neighbouring fingers' abductions toward each other until their proximal phalanges
    touch on the model (1 mm overlap): a light finger-on-finger press."""
    best = None
    for sa in (-1, 1):
        for sb in (-1, 1):
            q = {**REST, f"{a}_abd": sa * 10.0, f"{b_}_abd": sb * 10.0}
            d = c.distances(q)
            gap = min((v for p, v in d.items() if {p[0].split("_")[0], p[1].split("_")[0]} == {a, b_}), default=99)
            if best is None or gap < best[0]:
                best = (gap, sa, sb)
    _, sa, sb = best
    for k in np.arange(2.0, 28.0, 1.0):
        q = {**REST, f"{a}_abd": float(np.clip(sa * k, *ROM[f"{a}_abd"])), f"{b_}_abd": float(np.clip(sb * k, *ROM[f"{b_}_abd"]))}
        d = c.distances(q)
        gap = min((v for p, v in d.items() if {p[0].split("_")[0], p[1].split("_")[0]} == {a, b_}), default=99)
        if gap <= 0.5:
            return {j: v for j, v in q.items() if v}
    return None


def wear_programmes(c: Collider) -> dict:
    fist = {**{j: 0.75 * ROM[j][1] for j in FLEXORS}}
    if not c.free(fist):
        fist = {j: 0.6 * ROM[j][1] for j in FLEXORS}
    spread = {o: 0.8 * ROM[o][1] for o in ABDS}
    if not c.free(spread):
        spread = {o: 0.8 * ROM[o][0] for o in ABDS}
    thumb_in = {"thumb_cmc": 0.0, "thumb_abd": 30.0, "thumb_mcp": 40.0, "thumb_dip": 40.0}
    touch = json.loads((STUDY / "touch_poses.json").read_text())
    pinches = {f: {j: v for j, v in t["q"].items() if v} for f, t in touch.items()}
    presses = {f"{a}-{b}": squeeze(c, a, b) for a, b in (("index", "middle"), ("middle", "ring"), ("ring", "pinky"))}
    stops = {}
    for j in JOINTS:
        for side, v in (("lo", ROM[j][0]), ("hi", ROM[j][1])):
            q = {j: float(v - np.sign(v) * 1.0)}
            if c.free(q):
                stops[f"{j}-{side}"] = q
    return {"cycle": {"poses": {"open": {}, "fist": fist, "spread": spread, "thumb": thumb_in},
                      "sequence": ["fist", "open", "spread", "open", "thumb", "open"], "speed_deg_s": 60.0,
                      "hold_s": 0.3},
            "loaded": {"pinches": pinches, "presses": {k: v for k, v in presses.items() if v}, "stops": stops,
                       "press_extra_deg": 6.0, "hold_s": 4.0, "speed_deg_s": 30.0}}


BLOCKS = [  # (name, kind, plan, calibration run)
    ("N00-baseline", "collect", "baseline", "A"), ("N01-dense", "collect", "dense", "A"),
    ("N02-V0", "collect", "val", "A"), ("wear-cycle-1", "wear", "cycle", "A"), ("N03-V1", "collect", "val", "A"),
    ("wear-cycle-2", "wear", "cycle", "A"), ("N04-V2", "collect", "val", "A"),
    ("wear-loaded", "wear", "loaded", "A"), ("N05-V3", "collect", "val", "A"),
    ("recalibrate", "calibrate", None, "B"), ("N06-V4", "collect", "val", "B"),
    ("N07-short", "collect", "short", "B"), ("N08-V5", "collect", "val", "B"),
    ("N09-V6", "closedloop", "val", "B"), ("N11-kapandji", "kapandji", "kapandji", "B"),
    ("N12-sac", "sac", "sac", "B"),
    ("N10-extra", "collect", "extra", "B"),
]
MINUTES = {"cycle": 30, "loaded": 60, "calibrate": 2}


def main(tag: str, run_a: str, run_b: str, mount: list | None) -> dict:
    plans = {"dense": dense(), "val": validation(), "short": short()}
    base = Builder()
    base.add("zero", {}, kind="repeat", ref="open", side="start", visit=0)
    plans["baseline"] = base
    extra = Builder()
    extra.random(150, JOINTS, seed=77, tag="extra")
    approach(extra)
    plans["extra"] = extra
    plans["kapandji"] = Builder()
    plans["sac"] = Builder()
    runs = {"A": run_a, "B": run_b}
    night = {"tag": tag, "runs": runs, "angles": SPARSE_ANGLES, "mount": mount,
             "steps": [{"name": "calibrate-start", "kind": "calibrate", "run": run_a, "minutes": 2}], "sessions": []}
    for name, kind, which, r in BLOCKS:
        step = {"name": name, "kind": kind, "run": runs[r]}
        if kind in ("collect", "closedloop", "kapandji", "sac"):
            b = plans[which]
            sdir = STUDY / "sessions" / f"{tag}-{name}"
            sdir.mkdir(parents=True, exist_ok=True)
            (sdir / "plan.json").write_text(json.dumps(
                {"calibration": runs[r], "part": which, "pilot": False, "angles": SPARSE_ANGLES, "mount": mount,
                 "joints": JOINTS, "configs": b.out, "skipped": b.skipped, "night": tag, "block": name,
                 "closed_loop": kind == "closedloop"}, indent=1))
            n = {"kapandji": 12, "sac": 0}.get(kind, len(b.out))
            step.update(session=str(sdir), poses=n,
                        minutes=25 if kind == "sac" else round(n * (55 if kind == "kapandji" else S_PER_POSE) / 60))
            night["sessions"].append(str(sdir))
        else:
            step["minutes"] = MINUTES[which or "calibrate"]
            if kind == "wear":
                step["programme"] = which
        night["steps"].append(step)
    night["wear"] = wear_programmes(plans["dense"].c)
    (STUDY / "night").mkdir(exist_ok=True)
    (STUDY / "night" / f"{tag}.json").write_text(json.dumps(night, indent=1))
    return night


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("tag")
    p.add_argument("--run-a", default="run2")
    p.add_argument("--run-b", default="run3")
    a = p.parse_args()
    bundle = STUDY / "mount_bundle.json"
    n = main(a.tag, a.run_a, a.run_b, json.loads(bundle.read_text())["mount"] if bundle.exists() else None)
    t = 0
    for s in n["steps"]:
        print(f"{t // 60:2d}:{t % 60:02d}  {s['name']:16s} {s.get('poses', ''):>4} poses  {s['minutes']:4d} min")
        t += s["minutes"]
    print(f"total {t // 60} h {t % 60:02d} min (extra block runs until stopped)")
    print("wear presses:", n["wear"]["loaded"]["presses"])
