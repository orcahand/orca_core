"""The advanced calibration's collection (ADVANCED_CALIBRATION.md) for one hand: the data that
identifies advanced.py's terms. Blocks after the snapshot, baseline and mount:

    A01-sweeps   every joint up its range (from below), then down at the points between (from above)
    A02-wrist    wrist sweeps (up; down at mid flexion) with the fingers open / mid / flexed
    A03-pairs    mcp x pip grid per finger, thumb mcp x dip and cmc x abd
    A04-loaded   thumb-to-fingertip pinches and finger presses, each light and pressed (held)
    A05-V        the validation set of the quick run (held out; from below)
    A06-Vmix     random poses, every joint approached from a random side (held out; backlash)

The motor currents are logged at every pose and at every plate stop (current_each_view).
Plans span the hand's config ROM (STUDY_ROM_CONFIG, set before import).

    STUDY_ROM_CONFIG=<bench>/config.yaml python -m hand_study.night_advanced TAG --run RUN --bench DIR \\
        [--travel 55 300]
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np

from hand_study.configs import JOINTS, REST, ROM
from hand_study.night_plan import APPROACH_DEG, FLEXORS, S_PER_POSE, validation
from hand_study.plan import ABDS, SPARSE_ANGLES, STUDY, Builder

PRESS_DEG = 6.0
FLEX_LEVELS = {"open": 0.0, "mid": 0.4, "flexed": 0.7}


def approach(b: Builder, start: int = 0) -> None:
    """A via per pose: every joint APPROACH_DEG on the side given by c['dirs'] (default below),
    half that if the via collides; the pose is then reached from that side."""
    for c in b.out[start:]:
        if c.get("via"):
            continue
        dirs = c.get("dirs", {})
        for d in (APPROACH_DEG, APPROACH_DEG / 2):
            via = {j: float(np.clip(v - dirs.get(j, 1) * d, *ROM[j])) for j, v in c["q"].items()}
            if b.c.free(via):
                c["via"], c["approach"] = [via], f"{d:g}"
                break
        else:
            c["approach"] = "direct"


def sweeps(b: Builder) -> None:
    for j in JOINTS:
        lo, hi = ROM[j]
        n = 4 if j in ABDS else 5
        up = np.linspace(lo, hi, n)
        down = (lo + (np.arange(n - 1) + 0.5) / (n - 1) * (hi - lo))[::-1]
        for direction, pts in (("up", up), ("down", down)):
            for k, v in enumerate(pts.round(1)):
                q = b.free_with_neighbour(j, float(v), {})
                cid = f"sw-{j}-{direction}-{k}"
                if q is None:
                    b.skipped.append({"id": cid, "q": {j: float(v)}, "pair": "no free neighbour", "gap": None})
                    continue
                if b.add(cid, q, kind="sweep", joint=j, dir=direction, value=float(v)) and direction == "down":
                    b.out[-1]["dirs"] = {j: -1}


def flexion(level: float) -> dict:
    return {j: round(level * ROM[j][1], 1) for j in FLEXORS}


def wrist(b: Builder) -> None:
    lo, hi = ROM["wrist"]
    for name, f in FLEX_LEVELS.items():
        for k, v in enumerate(np.linspace(lo, hi, 5).round(1)):
            q = next((q for s in (1.0, 0.85, 0.7) if b.c.free(q := {**flexion(f * s), "wrist": float(v)})), None)
            if q is None:
                b.skipped.append({"id": f"wr-{name}-up-{k}", "q": {"wrist": float(v)}, "pair": "no free flexion", "gap": None})
                continue
            b.add(f"wr-{name}-up-{k}", q, kind="wrist", dir="up", flex=name)
    for k, v in enumerate((lo + (np.arange(4) + 0.5) / 4 * (hi - lo))[::-1].round(1)):
        if b.add(f"wr-mid-down-{k}", {**flexion(FLEX_LEVELS["mid"]), "wrist": float(v)}, kind="wrist", dir="down", flex="mid"):
            b.out[-1]["dirs"] = {"wrist": -1}


def pairs(b: Builder) -> None:
    for f in ("index", "middle", "ring", "pinky"):
        b.grid(f, f"{f}_mcp", f"{f}_pip", 3, 3)
    b.grid("thumb_flex", "thumb_mcp", "thumb_dip", 3, 3)
    b.grid("thumb_base", "thumb_cmc", "thumb_abd", 3, 3)


def clip(q: dict) -> dict:
    return {j: float(np.clip(v, *ROM[j])) for j, v in q.items()}


def loaded(b: Builder) -> None:
    """Pinches (touch_poses.json: the pads 1 mm into each other on the URDF) and finger presses
    (neighbouring proximal phalanges, night_plan.squeeze): light, then pressed PRESS_DEG further.
    Contact is the point, so no clearance check; the tracking check is off for these poses."""
    from hand_study.night_plan import squeeze
    touch = json.loads((STUDY / "touch_poses.json").read_text())
    for f, t in touch.items():
        q = clip({j: v for j, v in t["q"].items() if v})
        near = clip({j: 0.8 * v for j, v in q.items()})
        press = clip({**q, f"{f}_mcp": q.get(f"{f}_mcp", 0.0) + PRESS_DEG, "thumb_mcp": q.get("thumb_mcp", 0.0) + PRESS_DEG})
        b.out.append({"id": f"ld-pinch-{f}-light", "q": {**REST, **q}, "kind": "loaded", "loaded": True, "finger": f,
                      "via": [{**REST, **near}], "speed": 10.0})
        b.out.append({"id": f"ld-pinch-{f}-press", "q": {**REST, **press}, "kind": "loaded", "loaded": True, "finger": f,
                      "via": [{**REST, **near}], "speed": 10.0})
    for a, c in (("index", "middle"), ("middle", "ring"), ("ring", "pinky")):
        q = squeeze(b.c, a, c)
        if not q:
            continue
        q = clip(q)
        press = clip({j: v + PRESS_DEG * np.sign(v) / 2 for j, v in q.items()})
        for name, p in (("light", q), ("press", press)):
            b.out.append({"id": f"ld-press-{a}-{c}-{name}", "q": {**REST, **p}, "kind": "loaded", "loaded": True,
                          "via": [dict(REST)], "speed": 5.0})


def vmix(b: Builder, n: int = 12, seed: int = 55) -> None:
    rng = np.random.default_rng(seed)
    start = len(b.out)
    b.random(n, JOINTS, seed=seed, tag="vmix")
    for c in b.out[start:]:
        c["dirs"] = {j: int(rng.choice((-1, 1))) for j in JOINTS}
        c["kind"] = "val"


def blocks() -> list[tuple[str, Builder, str | None]]:
    base = Builder()
    base.add("zero", {}, kind="repeat", ref="open", side="start", visit=0)
    out = [("N00-baseline", base, None)]
    for name, fill in (("A01-sweeps", sweeps), ("A02-wrist", wrist), ("A03-pairs", pairs), ("A04-loaded", loaded)):
        b = Builder()
        fill(b)
        approach(b)
        out.append((name, b, "pending"))
    out.append(("A05-V", validation(wrist_lo=False), "pending"))
    b = Builder()
    vmix(b)
    approach(b)
    out.append(("A06-Vmix", b, "pending"))
    return out


def main(tag: str, run: str, bench: Path, motors: str, track_ma: float, travel: list[float] | None) -> dict:
    assert os.environ.get("STUDY_ROM_CONFIG"), "set STUDY_ROM_CONFIG to the hand's config.yaml"
    night = {"tag": tag, "mode": "advanced", "runs": {"A": run, "B": run}, "angles": SPARSE_ANGLES, "bench": str(bench),
             "motors": motors, "track_ma": track_ma, "rom_config": os.environ["STUDY_ROM_CONFIG"], "travel": travel,
             "train": [], "test": {},
             "steps": [{"name": "snapshot", "kind": "snapshot", "run": run, "minutes": 1}], "sessions": []}
    for name, b, mount in blocks():
        sdir = STUDY / "sessions" / f"{tag}-{name}"
        sdir.mkdir(parents=True, exist_ok=True)
        (sdir / "plan.json").write_text(json.dumps(
            {"calibration": run, "part": name.split("-", 1)[1], "pilot": False, "angles": SPARSE_ANGLES,
             "mount": mount, "joints": JOINTS, "configs": b.out, "skipped": b.skipped, "night": tag, "block": name,
             "closed_loop": False, "current_each_view": True, "mode": "advanced"}, indent=1))
        night["steps"].append({"name": name, "kind": "collect", "run": run, "session": str(sdir), "poses": len(b.out),
                               "minutes": round(len(b.out) * S_PER_POSE / 60)})
        night["sessions"].append(str(sdir))
        if name.startswith("A0") and name not in ("A05-V", "A06-Vmix"):
            night["train"].append(str(sdir))
        elif name.startswith("A0"):
            night["test"][name.split("-", 1)[1]] = [str(sdir)]
        if name == "N00-baseline":
            night["steps"].append({"name": "mount", "kind": "mount", "run": run, "from": str(sdir), "minutes": 3})
    (STUDY / "night").mkdir(exist_ok=True)
    (STUDY / "night" / f"{tag}.json").write_text(json.dumps(night, indent=1))
    return night


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("tag")
    p.add_argument("--run", required=True)
    p.add_argument("--bench", type=Path, required=True)
    p.add_argument("--motors", default="feetech")
    p.add_argument("--track-ma", type=float, default=500.0)
    p.add_argument("--travel", type=float, nargs=2)
    a = p.parse_args()
    n = main(a.tag, a.run, a.bench.expanduser(), a.motors, a.track_ma, a.travel)
    t = 0
    for s in n["steps"]:
        print(f"{t // 60:2d}:{t % 60:02d}  {s['name']:14s} {s.get('poses', ''):>4} poses {s['minutes']:4d} min")
        t += s["minutes"]
    print(f"total {t // 60} h {t % 60:02d} min")
