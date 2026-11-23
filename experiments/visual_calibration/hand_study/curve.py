"""Per-joint curve and a coupled table for one hand, on top of orca_core's standard map with the
hand's current config and calibration (x = what orca_core estimates now). The curve: per joint,
true = x + h(x), h piecewise linear over the ROM, smooth (second differences) and shrunk to 0
(the standard map) where the data is thin. The coupled table adds, per joint, 1-D terms over its
finger's joints and the wrist, shrunk harder. Fitted with trimming (vision blunders), checked on
held-out poses, exported in orca-ui's study-map format (maps/<hand>/<name>.json).

    STUDY_ROM_CONFIG=<bench>/config.yaml python -m hand_study.curve --hand ser-7577 --run s7577b \\
        --train SESSION... --test SESSION... [--packaged CONFIG]
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import yaml

from hand_study import analysis, hardstops
from hand_study.configs import JOINTS, ROM
from hand_study.lut import hat

STUDY = analysis.STUDY
KNOTS = 7
PARTNER_KNOTS = 4
SMOOTH = 4.0          # weight of the second differences (deg per knot)
RIDGE = 0.15          # shrink of the correction to the standard map
PARTNER_RIDGE = 1.0
TRIM_DEG = 6.0
FINGERS = ("thumb", "index", "middle", "ring", "pinky")


def finger(j: str) -> str:
    return j.split("_")[0]


def partners(j: str) -> list[str]:
    return [k for k in JOINTS if k != j and j != "wrist" and (finger(k) == finger(j) or k == "wrist")]


class Model:
    """true_j = x_j + own_j(x_j) [+ sum_p g_jp(x_p) when coupled]."""

    def __init__(self, coupled: bool):
        self.coupled = coupled
        self.own = {j: np.linspace(*ROM[j], KNOTS) for j in JOINTS}
        self.pk = {j: np.linspace(*ROM[j], PARTNER_KNOTS) for j in JOINTS}
        self.w: dict[str, np.ndarray] = {}
        self.parts = {j: partners(j) if coupled else [] for j in JOINTS}
        self.rms: dict[str, float] = {}
        self.n: dict[str, int] = {}

    def design(self, j: str, X: dict[str, np.ndarray]) -> np.ndarray:
        cols = [hat(X[j], self.own[j])]
        for p in self.parts[j]:
            B = hat(X[p], self.pk[p])
            cols.append(B - hat(np.zeros(1), self.pk[p]))        # 0 at the partner's 0
        return np.hstack(cols)

    def penalty(self, j: str, n: int) -> np.ndarray:
        rows = []
        for i in range(KNOTS - 2):
            r = np.zeros(n)
            r[i:i + 3] = SMOOTH * np.array([1, -2, 1])
            rows.append(r)
        rows += list(RIDGE * np.eye(n)[:KNOTS])
        rows += list(PARTNER_RIDGE * np.eye(n)[KNOTS:])
        return np.array(rows)

    def fit(self, X: dict[str, np.ndarray], Y: dict[str, np.ndarray], ok: dict[str, np.ndarray],
            wt: dict[str, np.ndarray] | None = None) -> None:
        """wt: per-row weights (squared-residual multipliers, default 1), e.g. collision contacts."""
        for j in JOINTS:
            A, y = self.design(j, X), Y[j] - X[j]
            keep = ok[j].copy()
            P = self.penalty(j, A.shape[1])
            sw = np.sqrt(wt[j]) if wt is not None else np.ones(len(y))
            for _ in range(3):
                w, *_ = np.linalg.lstsq(np.vstack([A[keep] * sw[keep, None], P]), np.r_[y[keep] * sw[keep], np.zeros(len(P))], rcond=None)
                res = y - A @ w
                mad = 1.4826 * np.median(np.abs(res[keep] - np.median(res[keep])))
                keep = ok[j] & (np.abs(res) <= max(TRIM_DEG, 3 * mad))
            self.w[j] = w
            self.rms[j] = float(np.sqrt(np.mean(res[keep] ** 2)))
            self.n[j] = int(keep.sum())

    def forward(self, x: dict[str, float]) -> dict[str, float]:
        X = {j: np.array([float(x.get(j, 0.0))]) for j in JOINTS}
        return {j: float(X[j][0] + (self.design(j, X) @ self.w[j])[0]) for j in JOINTS}

    def own_curve(self, j: str, xs: np.ndarray) -> np.ndarray:
        """The joint's own curve with every partner at 0."""
        return xs + hat(xs, self.own[j]) @ self.w[j][:KNOTS]

    def params(self) -> int:
        return int(sum(len(w) for w in self.w.values()))


def x_of(r: dict, L: hardstops.Linear) -> dict[str, float]:
    return {j: L.joint(j, r["motor"][j]) for j in JOINTS}


def arrays(R, L):
    X = {j: np.array([x_of(r, L)[j] for r in R]) for j in JOINTS}
    Y = {j: np.array([r["measured"][j] for r in R]) for j in JOINTS}
    ok = {j: np.array([(r.get("split") or {}).get(j, 0.0) <= hardstops.SPLIT_MAX_DEG for r in R]) for j in JOINTS}
    return X, Y, ok


def export(model: Model, name: str, R: list[dict], noise: dict, created: str, extra: dict) -> dict:
    joints = {}
    for j in JOINTS:
        xs = np.linspace(*ROM[j], 25)
        ys = np.maximum.accumulate(model.own_curve(j, xs))         # monotone for the inverse
        if ys[-1] - ys[0] < 1e-6:
            ys = xs.copy()
        g, c = np.polyfit(xs, ys, 1)
        rms = model.rms[j]
        joints[j] = {"applied": True, "confidence": "good" if rms < 2 else "fair" if rms < 4 else "low",
                     "noise_deg": noise.get(j), "n": model.n[j], "cmd_knots": xs.round(3).tolist(),
                     "cmd_true": ys.round(3).tolist(), "assumed_knots": xs.round(3).tolist(),
                     "assumed_true": ys.round(3).tolist(), "reach": [round(float(ys[0]), 1), round(float(ys[-1]), 1)],
                     "rom": list(ROM[j]), "fit_rms_deg": round(rms, 2), "gain": round(float(g), 3),
                     "offset_at_0": round(float(np.interp(0.0, xs, ys) - 0.0), 2)}
    out = {"created": created, "n_poses": len(R), "kind": name, "params": model.params(),
           "hysteresis": "not corrected", "base": "orca_core's standard map with the hand's current config and calibration",
           "joints": joints, **extra}
    if model.coupled:
        out["coupled"] = {"knots": KNOTS, "partner_knots": PARTNER_KNOTS, "own": {j: model.own[j].tolist() for j in JOINTS},
                          "pk": {j: model.pk[j].tolist() for j in JOINTS}, "parts": model.parts,
                          "w": {j: model.w[j].round(5).tolist() for j in JOINTS}}
    return out


def main(hand: str, run: str, train: list[Path], test: list[Path], packaged: Path | None, old_run: str | None) -> dict:
    Rtr = hardstops.good(analysis.rows(train))
    Rte = hardstops.good(analysis.rows(test))
    L = hardstops.Linear(run)
    created = time.strftime("%Y-%m-%d %H:%M")
    P = {"standard (current config)": lambda r, j: L.joint(j, r["motor"][j])}
    if packaged and old_run:
        rom = {j: list(map(float, v)) for j, v in yaml.safe_load(packaged.read_text())["joint_roms"].items()}
        Lp = hardstops.Linear(old_run, rom)
        P = {"packaged config": lambda r, j: Lp.joint(j, r["motor"][j]), **P}
    models = {}
    for name, coupled in (("curve", False), ("lut", True)):
        m = Model(coupled)
        m.fit(*arrays(Rtr, L))
        models[name] = m
        cache: dict = {}

        def pred(r, j, m=m, cache=cache):
            k = r["session"] + r["id"]
            if k not in cache:
                cache[k] = m.forward(x_of(r, L))
            return cache[k][j]
        P[name] = pred
    heldout = {k: hardstops.accuracy(Rte, f) for k, f in P.items()}
    train_acc = {k: hardstops.accuracy(Rtr, f) for k, f in P.items()}
    params = {"packaged config": 34, "standard (current config)": 34,
              "curve": models["curve"].params(), "lut": models["lut"].params()}
    R = Rtr + Rte
    X, Y, ok = arrays(R, L)
    noise = {j: round(float(np.median([r["split"][j] for r in R if r.get("split")])), 2) for j in JOINTS}
    out_dir = STUDY / "maps" / hand
    out_dir.mkdir(parents=True, exist_ok=True)
    final = {}
    for name, coupled in (("curve", False), ("lut", True)):
        m = Model(coupled)
        m.fit(X, Y, ok)
        final[name] = m
        d = export(m, name, R, noise, created, {"hand": hand, "run": run,
                                                "heldout": heldout.get(name, {}).get("all"),
                                                "heldout_standard": heldout["standard (current config)"]["all"]})
        (out_dir / f"{name}.json").write_text(json.dumps(d, indent=1))
    points = [{"session": r["session"], "id": r["id"], "x": x_of(r, L), "measured": r["measured"],
               "split": r.get("split") or {}, "test": r in Rte} for r in R]
    ev = {"hand": hand, "run": run, "created": created, "train": [p.name for p in train], "test": [p.name for p in test],
          "n_train": len(Rtr), "n_test": len(Rte), "heldout": heldout, "train_acc": train_acc, "params": params,
          "curves": {j: {"x": np.linspace(*ROM[j], 25).round(2).tolist(),
                         "curve": final["curve"].own_curve(j, np.linspace(*ROM[j], 25)).round(2).tolist()} for j in JOINTS},
          "rom": {j: list(ROM[j]) for j in JOINTS}, "points": points}
    (out_dir / "eval.json").write_text(json.dumps(ev, indent=1, default=float))
    return ev


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--hand", required=True)
    p.add_argument("--run", required=True)
    p.add_argument("--train", type=Path, nargs="+", required=True)
    p.add_argument("--test", type=Path, nargs="+", required=True)
    p.add_argument("--packaged", type=Path)
    p.add_argument("--old-run")
    a = p.parse_args()
    e = main(a.hand, a.run, a.train, a.test, a.packaged, a.old_run)
    for k, v in e["heldout"].items():
        print(f"{k:28s} {v['all']}  params {e['params'][k]}")
