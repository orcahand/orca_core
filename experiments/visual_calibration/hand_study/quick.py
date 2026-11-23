"""One hand's quick visual recalibration evaluated on its validation block: the maps fitted on the
short block (hard stops refitted, LUT) against the configs given (--rom NAME=config.yaml, each a
ROM for orca_core's linear map under the same calibration run). Run once per hand, each in its
own process (STUDY_ROM_CONFIG sets the LUT's knots), then compare the JSON files.

    python -m hand_study.quick --short S --val V --run RUN --rom packaged=... [--drop ID] --out OUT.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml

from hand_study import analysis, hardstops
from hand_study.configs import JOINTS
from hand_study.lut import fit_rows


def roms(path: Path) -> dict:
    return {j: list(map(float, r)) for j, r in yaml.safe_load(path.read_text())["joint_roms"].items()}


def vision(R: list[dict]) -> dict:
    split = {j: [r["split"][j] for r in R if r["split"].get(j) is not None] for j in JOINTS}
    return {"rms_px_median": round(float(np.median([r["rms_px"] for r in R])), 2),
            "split_median": {j: round(float(np.median(v)), 2) for j, v in split.items() if v},
            "seen_share": {j: round(float(np.mean(np.array(v) <= hardstops.SPLIT_MAX_DEG)), 2) for j, v in split.items() if v}}


def repeat(R: list[dict], a: str, b: str) -> dict | None:
    """The same pose at the start and the end of the block: measured difference per joint."""
    ra, rb = next((r for r in R if r["id"] == a), None), next((r for r in R if r["id"] == b), None)
    if not ra or not rb:
        return None
    d = {j: round(rb["measured"][j] - ra["measured"][j], 2) for j in JOINTS}
    return {"per_joint": d, "rms": round(float(np.sqrt(np.mean(np.square(list(d.values()))))), 2)}


def check(val: Path, run: str, stops: dict[str, dict], recorded: list[str] | None = None) -> dict:
    """The hand's accuracy on its validation block (true minus predicted, deg): orca_core's linear map
    under calibration `run` with the ranges in force before (the archived ones) and after: the fitted
    `stops` (hardstops.fit) of the joints `recorded` in joint_ranges.yaml (default: every joint the fit
    identified), the old range for the rest."""
    V_all = analysis.rows([val])
    V = hardstops.good(V_all)
    kept = set(recorded if recorded is not None else (j for j, s in stops.items() if s["identifiable"]))
    before = hardstops.Linear(run)
    after = hardstops.Linear(run, {j: stops[j]["new"] if j in kept else stops[j]["old"] for j in JOINTS})
    return {"run": run, "val": val.name, "n": len(V), "n_all": len(V_all), "recorded": sorted(kept),
            "before": hardstops.accuracy(V, lambda r, j: before.joint(j, r["motor"][j])),
            "after": hardstops.accuracy(V, lambda r, j: after.joint(j, r["motor"][j]))}


def main(short: Path, val: Path, run: str, extra_roms: dict[str, Path], drop: list[str]) -> dict:
    S_all = analysis.rows([short])
    s_ex = list(analysis.EXCLUDED)
    V_all = [r for r in analysis.rows([val]) if r["id"] not in drop]
    v_ex = list(analysis.EXCLUDED)
    S, V = hardstops.good(S_all), hardstops.good(V_all)
    stops = hardstops.fit(S, run)
    lut = fit_rows(S)
    P = {}
    for name, p in extra_roms.items():
        L = hardstops.Linear(run, roms(p))
        P[name] = lambda r, j, L=L: L.joint(j, r["motor"][j])
    L0 = hardstops.Linear(run)
    P["calibrated_config"] = lambda r, j: L0.joint(j, r["motor"][j])
    Ls = hardstops.Linear(run, {j: stops[j]["new"] for j in JOINTS})
    P["refit_short"] = lambda r, j: Ls.joint(j, r["motor"][j])
    cache: dict = {}

    def lut_p(r, j):
        if r["id"] not in cache:
            cache[r["id"]] = lut.forward(r["commanded"])
        return cache[r["id"]][j]
    P["lut_short"] = lut_p
    return {"run": run, "short": short.name, "val": val.name, "drop": drop,
            "n": {"short": len(S), "short_all": len(S_all), "val": len(V), "val_all": len(V_all)},
            "excluded": {"short": s_ex, "val": v_ex},
            "blunders": [r["id"] for r in S_all + V_all if r["rms_px"] > hardstops.RMS_MAX_PX],
            "val": {k: hardstops.accuracy(V, f) for k, f in P.items()},
            "train": {k: hardstops.accuracy(S, f) for k, f in P.items()},
            "stops": stops, "vision": {"short": vision(S), "val": vision(V)},
            "repeat_open": repeat(V_all, "v-open-a", "v-open-b"),
            "val_ids": [r["id"] for r in V]}


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--short", type=Path, required=True)
    p.add_argument("--val", type=Path, required=True)
    p.add_argument("--run", required=True)
    p.add_argument("--rom", action="append", default=[], help="NAME=config.yaml")
    p.add_argument("--drop", action="append", default=[])
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--est", default="est.json")
    a = p.parse_args()
    analysis.EST = a.est
    res = main(a.short, a.val, a.run, {k: Path(v).expanduser() for k, v in (x.split("=", 1) for x in a.rom)}, a.drop)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(res, indent=1, default=float))
    for k, v in res["val"].items():
        print(f"{k:18s} {v['all']}")
