"""New hard-stop angles for the hand's config: orca_core's linear map unchanged, only the
joint ROM it spans (config joint_roms) refitted by least squares on the measured true angles.

orca_core: joint = lo + (m - m0) / ratio (inverted: hi - ...), with ratio = (m1 - m0) / (hi - lo)
set by its calibration from the config ROM. So after a standard calibration with the new config,
with t = (m - m0) / (m1 - m0):   joint = lo (1 - t) + hi t   (inverted: hi (1 - t) + lo t),
linear in (lo, hi). Note: the new numbers act fully only after a calibration with the new
config (it recomputes the ratio); without one only the offset moves.

    python -m hand_study.hardstops SESSION... [--run run2]    # proposed/ config + diff + accuracy
"""

from __future__ import annotations

import argparse
import json
import os
import re
from pathlib import Path

import numpy as np

from hand_study import analysis
from hand_study.configs import JOINTS
from hand_study.mapping import LinearMap

STUDY = analysis.STUDY
OUT = STUDY / "proposed"
RMS_MAX_PX = float(os.environ.get("STUDY_RMS_MAX_PX", "9.5"))  # worse fits (typ. 8-8.5 px; rig 330 + crop: 9.7) are vision blunders: left out
TRIM_DEG = 6.0              # per joint, residuals beyond max(this, 3 robust sigma) are dropped and refitted
SPLIT_MAX_DEG = 12.0        # a joint whose two half-view solves disagree this much is not seen: left out
MIN_SPAN = 0.4              # the motor must have covered this share of its range, else the old stops stay


def good(R: list[dict]) -> list[dict]:
    return [r for r in R if r["rms_px"] <= RMS_MAX_PX]


def t_of(L: LinearMap, j: str, m: float) -> float:
    lo, hi = L.limits[L.motor[j]]
    return (L.unwrap(j, m) - lo) / (hi - lo)


def fit(R: list[dict], run: str) -> dict[str, dict]:
    L = LinearMap(STUDY / f"calib_{run}")
    out = {}
    rr = [r for r in good(R) if r["run"] == run]
    for j in JOINTS:
        t = np.array([t_of(L, j, r["motor"][j]) for r in rr])
        y = np.array([r["measured"][j] for r in rr])
        A = np.c_[t, 1 - t] if L.inverted[j] else np.c_[1 - t, t]
        keep = np.ones(len(y), bool)
        for _ in range(3):
            (lo, hi), *_ = np.linalg.lstsq(A[keep], y[keep], rcond=None)
            res = y - A @ [lo, hi]
            mad = 1.4826 * np.median(np.abs(res[keep] - np.median(res[keep])))
            keep = np.abs(res) <= max(TRIM_DEG, 3 * mad)
        span = float(np.ptp(t[keep])) if keep.any() else 0.0
        identifiable = span >= MIN_SPAN
        new = [round(float(lo), 1), round(float(hi), 1)] if identifiable else list(L.rom[j])
        out[j] = {"old": list(L.rom[j]), "new": new, "n": int(keep.sum()), "dropped": int((~keep).sum()),
                  "t_span": [round(float(t.min()), 2), round(float(t.max()), 2)] if len(t) else None,
                  "identifiable": identifiable, "fit": [round(float(lo), 1), round(float(hi), 1)],
                  "rms": round(float(np.sqrt(np.mean(res[keep] ** 2))), 2)}
    return out


class Linear:
    """orca_core's map for one calibration run with a given ROM (old config or new stops)."""

    def __init__(self, run: str, rom: dict[str, list[float]] | None = None):
        self.L = LinearMap(STUDY / f"calib_{run}")
        self.rom = {j: tuple(rom[j]) for j in JOINTS} if rom else self.L.rom

    def joint(self, j: str, m: float) -> float:
        lo, hi = self.rom[j]
        t = t_of(self.L, j, m)
        return hi - t * (hi - lo) if self.L.inverted[j] else lo + t * (hi - lo)

    def motor(self, j: str, q: float) -> float:
        lo, hi = self.rom[j]
        m0, m1 = self.L.limits[self.L.motor[j]]
        t = (hi - q) / (hi - lo) if self.L.inverted[j] else (q - lo) / (hi - lo)
        return m0 + t * (m1 - m0)


def accuracy(R: list[dict], predict) -> dict:
    """Per joint and overall RMS and max |true - predicted| (deg)."""
    out, allerr = {}, []
    for j in JOINTS:
        e = np.array([r["measured"][j] - predict(r, j) for r in R if (r.get("split") or {}).get(j, 0.0) <= SPLIT_MAX_DEG])
        allerr.append(e)
        out[j] = {"rms": round(float(np.sqrt(np.mean(e ** 2))), 2), "max": round(float(np.abs(e).max()), 2),
                  "mean": round(float(e.mean()), 2)} if len(e) else None
    e = np.concatenate(allerr) if allerr else np.zeros(0)
    out["all"] = {"rms": round(float(np.sqrt(np.mean(e ** 2))), 2), "max": round(float(np.abs(e).max()), 2),
                  "p50": round(float(np.percentile(np.abs(e), 50)), 2),
                  "p95": round(float(np.percentile(np.abs(e), 95)), 2), "n": int(len(e))} if len(e) else None
    return out


def compare(R: list[dict], stops: dict[str, dict], lut=None) -> dict:
    """a) old config, b) new stops (linear), c) the full LUT, on the rows given (each in its own run)."""
    runs = {r["run"] for r in R}
    old = {run: Linear(run) for run in runs}
    new = {run: Linear(run, {j: stops[j]["new"] for j in JOINTS}) for run in runs}
    out = {"a_old": accuracy(R, lambda r, j: old[r["run"]].joint(j, r["motor"][j])),
           "b_new_stops": accuracy(R, lambda r, j: new[r["run"]].joint(j, r["motor"][j]))}
    if lut is not None:
        cache = {}

        def lut_pred(r, j):
            if r["id"] + r["session"] not in cache:
                cache[r["id"] + r["session"]] = lut.forward(r["commanded"])
            return cache[r["id"] + r["session"]][j]
        out["c_lut"] = accuracy(R, lut_pred)
    return out


def proposed_config(config: Path, stops: dict[str, dict]) -> tuple[str, list[str]]:
    """The config text with only the joint_roms numbers replaced, and the changed lines."""
    text = config.read_text()
    lines = text.splitlines(keepends=True)
    out, diff, in_roms, joint, idx = [], [], False, None, 0
    for line in lines:
        if re.match(r"^joint_roms:\s*$", line):
            in_roms = True
        elif in_roms and re.match(r"^\S", line):
            in_roms = False
        if in_roms:
            m = re.match(r"^  (\w+):\s*$", line)
            if m:
                joint, idx = m.group(1), 0
            m = re.match(r"^(\s*-\s*)(-?[\d.]+)(\s*)$", line)
            if m and joint in stops:
                new = stops[joint]["new"][idx]
                old = float(m.group(2))
                idx += 1
                if abs(new - old) >= 0.05:
                    nl = f"{m.group(1)}{new:g}{m.group(3)}"
                    diff.append(f"joint_roms.{joint}[{idx - 1}]: {old:g} -> {new:g}")
                    line = nl
        out.append(line)
    return "".join(out), diff


def main(sessions: list[Path], run: str, lut_path: Path | None = None) -> dict:
    R = analysis.rows(sessions)
    stops = fit(R, run)
    L = LinearMap(STUDY / f"calib_{run}")
    OUT.mkdir(exist_ok=True)
    text, diff = proposed_config(STUDY / f"calib_{run}" / "config.yaml", stops)
    (OUT / "config.yaml").write_text(text)
    (OUT / "config.diff.txt").write_text("\n".join(diff) + "\n")
    lut = None
    if lut_path and lut_path.exists():
        from hand_study.lut import Lut
        lut = Lut.load(lut_path)
    out = {"run": run, "sessions": [s.name for s in sessions], "n": len(R), "stops": stops, "diff": diff,
           "train": compare(R, stops, lut), "inverted": L.inverted}
    (OUT / "hardstops.json").write_text(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("sessions", type=Path, nargs="+")
    p.add_argument("--run", default="run2")
    p.add_argument("--lut", type=Path)
    a = p.parse_args()
    o = main(a.sessions, a.run, a.lut)
    for j, s in o["stops"].items():
        print(f"{j:12s} {s['old']} -> {s['new']}  rms {s['rms']}  n {s['n']}  t {s['t_span']}")
    print("old", o["train"]["a_old"]["all"], "new", o["train"]["b_new_stops"]["all"])
    print("\n".join(o["diff"]))
