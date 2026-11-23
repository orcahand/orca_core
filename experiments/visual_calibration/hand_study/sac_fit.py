"""Super Advanced Calibration, the fit: the new hard stops (hardstops.py) refitted with the
SAC contacts as extra data. Each contact is a closure constraint: at the touch onset the thumb's
and the finger's distal links are GAP0_MM apart on the URDF (pads just touching), with the
joint angles read from the motors through orca_core's linear map with the stops being fitted.

    python -m hand_study.sac_fit DENSE_SESSION SAC_SESSION [--loo]   # proposed/sac.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from hand_study import analysis, hardstops
from hand_study.configs import JOINTS, Collider
from hand_study.mapping import LinearMap
from hand_study.touch import min_dist
from visual_calibration.solve import least_squares

STUDY = analysis.STUDY
GAP0_MM = 0.0
SIGMA_DEG = 2.0            # vision noise per joint
SIGMA_MM = 1.5             # contact: pad compliance + model


def contacts(session: Path) -> list[dict]:
    p = session / "contacts.jsonl"
    return [c for c in (json.loads(l) for l in p.read_text().splitlines() if l.strip()) if c.get("contact")] \
        if p.exists() else []


class SacFit:
    def __init__(self, R: list[dict], C: list[dict], run: str, stops: dict, run_c: str | None = None):
        self.L = LinearMap(STUDY / f"calib_{run}")
        self.Lc = LinearMap(STUDY / f"calib_{run_c or run}")
        self.R, self.C = R, C
        self.col = Collider(spacing=1.5)
        self.x0 = np.array([v for j in JOINTS for v in stops[j]["new"]], float)
        self.T = np.array([[hardstops.t_of(self.L, j, r["motor"][j]) for j in JOINTS] for r in R])
        self.Y = np.array([[r["measured"][j] for j in JOINTS] for r in R])
        self.Tc = [{j: hardstops.t_of(self.Lc, j, m) for j, m in self._motors(c).items()} for c in C]

    @staticmethod
    def _motors(c: dict) -> dict[str, float]:
        o = c["onset"]
        return {j: o["motor_pos"][o["motor_joint"].index(j)] for j in JOINTS}

    def q_of(self, x: np.ndarray, t: dict[str, float]) -> dict[str, float]:
        out = {}
        for k, j in enumerate(JOINTS):
            lo, hi = x[2 * k], x[2 * k + 1]
            out[j] = hi - t[j] * (hi - lo) if self.L.inverted[j] else lo + t[j] * (hi - lo)
        return out

    def vision_res(self, x: np.ndarray) -> np.ndarray:
        lo, hi = x[0::2], x[1::2]
        inv = np.array([self.L.inverted[j] for j in JOINTS])
        P = np.where(inv, hi - self.T * (hi - lo), lo + self.T * (hi - lo))
        return ((self.Y - P) / SIGMA_DEG).ravel()

    def gap(self, x: np.ndarray, i: int) -> float:
        c = self.C[i]
        return min_dist(self.col, self.q_of(x, self.Tc[i]), "thumb_dp", f"{c['finger']}_dp")

    def contact_res(self, x: np.ndarray, skip: int | None = None) -> np.ndarray:
        return np.array([(self.gap(x, i) - GAP0_MM) / SIGMA_MM for i in range(len(self.C)) if i != skip])

    def fit(self, x0=None, skip: int | None = None, iters: int = 12):
        x0 = self.x0 if x0 is None else x0
        f = least_squares(lambda x: np.r_[self.vision_res(x), self.contact_res(x, skip)], x0, x0 - 15, x0 + 15,
                          step=np.full(len(x0), 0.5), max_iter=iters)
        return f.x


def main(dense: list[Path], sac: Path, run: str, run_c: str, loo: bool = False) -> dict:
    R = hardstops.good(analysis.rows(dense))
    stops = hardstops.fit(R, run)
    C = contacts(sac)
    if len(C) < 4:
        raise SystemExit(f"only {len(C)} contacts")
    S = SacFit([r for r in R if r["run"] == run], C, run, stops, run_c)
    gaps0 = [S.gap(S.x0, i) for i in range(len(C))]
    x = S.fit()
    out = {"run": run, "contacts": len(C), "gap_before_mm": np.round(gaps0, 2).tolist(),
           "gap_after_mm": [round(S.gap(x, i), 2) for i in range(len(C))],
           "stops": {j: {"old": stops[j]["old"], "new": stops[j]["new"], "sac": [round(float(x[2 * k]), 1),
                                                                                  round(float(x[2 * k + 1]), 1)]}
                     for k, j in enumerate(JOINTS)}}
    if loo:
        out["loo"] = []
        for i in range(len(C)):
            xi = S.fit(x, skip=i, iters=4)
            out["loo"].append({"finger": C[i]["finger"], "gap_no_sac": round(gaps0[i], 2),
                               "gap_loo": round(S.gap(xi, i), 2)})
    (STUDY / "proposed").mkdir(exist_ok=True)
    (STUDY / "proposed" / "sac.json").write_text(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("dense", type=Path)
    p.add_argument("sac", type=Path)
    p.add_argument("--run", default="run2")
    p.add_argument("--run-c", default="run3")
    p.add_argument("--loo", action="store_true")
    a = p.parse_args()
    o = main([a.dense], a.sac, a.run, a.run_c, a.loo)
    print(json.dumps({k: o[k] for k in ("contacts", "gap_before_mm", "gap_after_mm")}))
    for j, s in o["stops"].items():
        print(f"{j:12s} {s['old']} -> {s['new']} -> sac {s['sac']}")
