"""Super Advanced Calibration (SAC), the hardware part: thumb-to-fingertip contacts found by
touch. The tactile pads are zeroed and their noise measured with the hand open; then for
each thumb-finger pair the hand creeps (4 deg/s) from an open start toward the Kapandji touch
pose and a little beyond, and stops at the first clear rise of the thumb's or the finger's pad
above its noise. The motor positions and tactile readings at that onset are a closure
constraint (thumb tip = fingertip) for the fit. Torque off after every contact.

Writes <session>/contacts.jsonl (one line per contact) and <session>/noise.json.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from hand_study.configs import REST, ROM
from hand_study.hw import Abort, Hand, beat, call

K_SIGMA = 6.0
MIN_RISE = 0.12            # sensor units (N) above the zeroed mean, at least (readings come in 0.1 steps)
CREEP_DEG_S = 4.0
HZ = 20.0
OVERSHOOT_DEG = 12.0
CONTACTS_PER_PAIR = 5
VARIANTS = [  # (start fraction, finger pip offset, thumb dip offset, thumb abd offset)
    (0.6, 0.0, 0.0, 0.0), (0.5, 10.0, 0.0, 0.0), (0.5, -10.0, 10.0, 0.0), (0.7, 0.0, -10.0, 5.0),
    (0.6, 5.0, 5.0, -5.0), (0.55, -5.0, 0.0, 5.0)]


def norm(v) -> float:
    return float(np.linalg.norm(v)) if v is not None else 0.0


def forces(hand: Hand) -> dict[str, float]:
    f = hand.s.get("tactile.forces").get("forces") or {}
    return {k: norm(v) for k, v in f.items()}


def noise(hand: Hand, seconds: float = 4.0) -> dict[str, dict]:
    xs: dict[str, list] = {}
    t0 = time.time()
    while time.time() - t0 < seconds:
        for k, v in forces(hand).items():
            xs.setdefault(k, []).append(v)
        time.sleep(1 / 50)
    return {k: {"mean": float(np.mean(v)), "std": float(np.std(v)), "n": len(v),
                "threshold": float(np.mean(v) + max(K_SIGMA * np.std(v), MIN_RISE))} for k, v in xs.items()}


def clip(q: dict) -> dict:
    return {j: float(np.clip(v, *ROM[j])) for j, v in q.items()}


def creep(hand: Hand, start: dict, end: dict, watch: list[str], thr: dict) -> dict | None:
    """Targets from start to end at CREEP_DEG_S; returns the onset sample, or None."""
    span = max(abs(end[j] - start[j]) for j in end)
    n = max(1, int(np.ceil(span / CREEP_DEG_S * HZ)))
    hits = 0
    for k in range(1, n + 1):
        q = {j: start[j] + k / n * (end[j] - start[j]) for j in end}
        call("/joints/target", {"angles": q})
        hand.target = q
        time.sleep(1 / HZ)
        f = forces(hand)
        hits = hits + 1 if any(f.get(s, 0.0) > thr[s]["threshold"] for s in watch if s in thr) else 0
        if hits >= 2:
            time.sleep(0.3)
            s = hand.sample(3, 0.05)
            return {"fraction": k / n, "command": q, "forces": f, **s}
        if k % int(HZ) == 0:
            beat("sac creep")
    return None


def run(session: Path, hand: Hand, log=print, fingers: list[str] | None = None) -> int:
    session.mkdir(parents=True, exist_ok=True)
    touch = json.loads((Path.home() / ".orca_ui/visual_calibration/hand_study/touch_poses.json").read_text())
    out = session / "contacts.jsonl"
    hand.torque(True)
    try:
        hand.move(dict(REST))
        time.sleep(1.0)
        call("/tactile/zero", {})
        time.sleep(1.0)
        thr = noise(hand)
        (session / "noise.json").write_text(json.dumps(thr, indent=1))
    finally:
        hand.torque(False)
    log(f"SAC noise: { {k: round(v['threshold'], 3) for k, v in thr.items()} }")
    n = 0
    for f, t in touch.items():
        if fingers and f not in fingers:
            continue
        for i, (frac, dpip, ddip, dabd) in enumerate(VARIANTS[:CONTACTS_PER_PAIR]):
            target = clip({**t["q"], f"{f}_pip": t["q"][f"{f}_pip"] + dpip, "thumb_dip": t["q"]["thumb_dip"] + ddip,
                           "thumb_abd": t["q"]["thumb_abd"] + dabd})
            end = clip({**target, f"{f}_mcp": target[f"{f}_mcp"] + OVERSHOOT_DEG,
                        "thumb_mcp": target["thumb_mcp"] + OVERSHOOT_DEG / 2})
            start = {j: round(frac * v, 2) for j, v in target.items()}
            hand.torque(True)
            try:
                hand.guard()
                hand.move({**REST, **start})
                time.sleep(0.5)
                hit = creep(hand, {**REST, **start}, {**REST, **end}, ["thumb", f], thr)
                hand.move({**REST, **start}, 15.0)
                hand.move(dict(REST))
            finally:
                hand.torque(False)
            rec = {"t": time.time(), "finger": f, "variant": i, "target": target, "end": end, "start": start,
                   "contact": hit is not None, **({"onset": hit} if hit else {})}
            with out.open("a") as fh:
                fh.write(json.dumps(rec) + "\n")
            n += hit is not None
            log(f"SAC {f} #{i}: {'contact at ' + format(hit['fraction'], '.2f') if hit else 'no contact'}")
            hand.cool_if_hot(log)
    return n
