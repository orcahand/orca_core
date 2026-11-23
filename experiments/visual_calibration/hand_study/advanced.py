"""Advanced calibration (ADVANCED_CALIBRATION.md): a small physical model per joint on top of the
hand's standard map x (orca_core's linear motor -> joint map):

    true_j = x_j + sum_k a_jk P_k(u_j)          smooth curve, Legendre P0..P3 over the ROM
                 + sum_p c_jp x_p               tendon coupling: the joints the tendon crosses
                 + kappa_j I_j                  compliance: tendon stretch with the motor current (A)
                 + beta_j d_j                   backlash: d = +1 approached from below, -1 from above
                                                (play width h_j = 2 |beta_j|)

about 6-9 parameters per joint, fitted by regularised least squares with trimming. The coupling
entries are shrunk toward a seed (advanced_seed.json): CAD moment-arm ratios when entered there,
otherwise a fleet prior fitted on ser-4485's dense block. The inverse (command) gives the
command that reaches a wanted angle for a direction of motion and a current.

Evaluation: standard / hard stops / curve / LUT / advanced on held-out poses, per joint RMS /
median / max and parameter count, to report-advanced/<hand>.json (the study page reads it).

    python -m hand_study.advanced ser4485 [--make-seed]
    STUDY_ROM_CONFIG=<bench>/config.yaml python -m hand_study.advanced ser7577 [--export]
    STUDY_ROM_CONFIG=... python -m hand_study.advanced ser7577 --train S... --test NAME=S... [--export]
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import yaml

from hand_study import analysis, curve, hardstops
from hand_study.configs import JOINTS, ROM

STUDY = analysis.STUDY
SEED = STUDY / "advanced_seed.json"
OUT = STUDY / "report-advanced"
SESS = STUDY / "sessions"
LAM_CURVE = (0.3, 0.3, 1.0, 2.0)      # P0..P3: offset, gain, bend, S-shape
LAM_COUPLE = 0.5                       # per deg/100deg, toward the seed
LAM_K = 0.5                            # per deg/A
LAM_BETA = 1.0                         # per deg
TRIM_DEG = 6.0
U_CLIP = 1.1


def chain(j: str) -> list[str]:
    """The joints the tendon of j crosses on its way from the forearm (proximal ones)."""
    if j == "wrist":
        return []
    f, k = j.split("_")
    if f == "thumb":
        order = ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"]
        return order[:order.index(j)] + ["wrist"]
    return {"abd": [], "mcp": [f"{f}_abd"], "pip": [f"{f}_abd", f"{f}_mcp"]}[k] + ["wrist"]


def legendre(u: np.ndarray) -> np.ndarray:
    u = np.clip(u, -U_CLIP, U_CLIP)
    return np.c_[np.ones_like(u), u, (3 * u ** 2 - 1) / 2, (5 * u ** 3 - 3 * u) / 2]


def directions(session: Path) -> dict[str, dict[str, int]]:
    """Per pose and joint, the direction of the last move that changed the joint (> 0.5 deg),
    followed through the session in execution order (vias included)."""
    metas = sorted((json.loads(p.read_text()) for p in session.glob("configs/*/meta.json")), key=lambda m: m["t0"])
    last, d, out = {j: 0.0 for j in JOINTS}, {j: 1 for j in JOINTS}, {}
    for m in metas:
        for p in (m.get("via") or []) + [m["q"]]:
            for j in JOINTS:
                v = float(p.get(j, last[j]))
                if abs(v - last[j]) > 0.5:
                    d[j] = 1 if v > last[j] else -1
                last[j] = v
        out[m["id"]] = dict(d)
    return out


TERMS = frozenset({"wrist", "finger", "backlash", "current"})
OFF = 1e6                              # the ridge that holds a switched-off term at 0


class Advanced:
    def __init__(self, seed: dict[str, dict[str, float]] | None = None, rom: dict | None = None,
                 terms: frozenset | set = TERMS, beta_prior: dict[str, float] | None = None):
        self.terms = frozenset(terms)
        self.beta_prior = dict(beta_prior or {})       # deg per joint, e.g. -deadzone/2 from the slack checks
        self.rom = {j: tuple(map(float, (rom or ROM)[j])) for j in JOINTS}
        self.parts = {j: chain(j) for j in JOINTS}
        self.seed = {j: {p: float((seed or {}).get(j, {}).get(p, 0.0)) for p in self.parts[j]} for j in JOINTS}
        self.w: dict[str, np.ndarray] = {}
        self.rms: dict[str, float] = {}
        self.n: dict[str, int] = {}

    def u(self, j: str, x):
        lo, hi = self.rom[j]
        return 2 * (np.asarray(x, float) - lo) / (hi - lo) - 1

    def design(self, j: str, X: dict, I: dict, D: dict) -> np.ndarray:
        cols = [legendre(self.u(j, X[j]))]
        cols += [np.asarray(X[p], float)[:, None] / 100 for p in self.parts[j]]
        cols += [np.asarray(I[j], float)[:, None] / 1000, np.asarray(D[j], float)[:, None]]
        return np.hstack(cols)

    def on(self, j: str) -> list[bool]:
        """Which of the joint's columns are in the model (the curve always)."""
        return [True] * 4 + [("wrist" if p == "wrist" else "finger") in self.terms for p in self.parts[j]] + \
            ["current" in self.terms, "backlash" in self.terms]

    def prior(self, j: str) -> tuple[np.ndarray, np.ndarray]:
        on = self.on(j)
        lam = [*LAM_CURVE, *[LAM_COUPLE] * len(self.parts[j]), LAM_K, LAM_BETA]
        target = [0.0] * 4 + [100 * self.seed[j][p] for p in self.parts[j]] + [0.0, float(self.beta_prior.get(j, 0.0))]
        lam = [l if o else OFF for l, o in zip(lam, on)]
        target = [x if o else 0.0 for x, o in zip(target, on)]
        return np.diag(lam), np.array(lam) * np.array(target)

    def fit(self, X: dict, Y: dict, I: dict, D: dict, ok: dict) -> Advanced:
        for j in JOINTS:
            A, y = self.design(j, X, I, D), Y[j] - X[j]
            P, t = self.prior(j)
            keep = ok[j].copy()
            for _ in range(3):
                w, *_ = np.linalg.lstsq(np.vstack([A[keep], P]), np.r_[y[keep], t], rcond=None)
                res = y - A @ w
                mad = 1.4826 * np.median(np.abs(res[keep] - np.median(res[keep])))
                keep = ok[j] & (np.abs(res) <= max(TRIM_DEG, 3 * mad))
            self.w[j] = w
            self.rms[j] = float(np.sqrt(np.mean(res[keep] ** 2)))
            self.n[j] = int(keep.sum())
        return self

    def forward(self, x: dict, cur: dict | None = None, d: dict | None = None) -> dict[str, float]:
        X = {j: np.array([float(x.get(j, 0.0))]) for j in JOINTS}
        I = {j: np.array([float((cur or {}).get(j, 0.0))]) for j in JOINTS}
        D = {j: np.array([float((d or {}).get(j, 0.0))]) for j in JOINTS}
        return {j: float(X[j][0] + (self.design(j, X, I, D) @ self.w[j])[0]) for j in JOINTS}

    def command(self, want: dict, d: dict | None = None, cur: dict | None = None) -> dict[str, float]:
        """The commands (standard-map space) that reach `want`, moving in direction d at current cur."""
        x = {j: float(want.get(j, 0.0)) for j in JOINTS}
        for _ in range(40):
            f = self.forward(x, cur, d)
            err = {j: float(want.get(j, 0.0)) - f[j] for j in JOINTS}
            for j in JOINTS:
                lo, hi = self.rom[j]
                x[j] = float(np.clip(x[j] + 0.8 * err[j], lo - 15, hi + 15))
            if max(abs(e) for e in err.values()) < 0.02:
                break
        return x

    def own_curve(self, j: str, xs: np.ndarray) -> np.ndarray:
        return xs + legendre(self.u(j, xs)) @ self.w[j][:4]

    def params(self) -> int:
        return int(sum(sum(self.on(j)) for j in self.w))

    def summary(self) -> dict:
        out = {}
        for j in JOINTS:
            w, k = self.w[j], 4 + len(self.parts[j])
            out[j] = {"curve": w[:4].round(3).tolist(), "coupling": {p: round(float(w[4 + i]) / 100, 4) for i, p in enumerate(self.parts[j])},
                      "k_deg_per_a": round(float(w[k]), 2), "beta": round(float(w[k + 1]), 2),
                      "play_deg": round(2 * abs(float(w[k + 1])), 2), "fit_rms": round(self.rms[j], 2), "n": self.n[j]}
        return out

    def to_json(self) -> dict:
        return {"rom": {j: list(self.rom[j]) for j in JOINTS}, "parts": self.parts,
                "a": {j: self.w[j][:4].round(5).tolist() for j in JOINTS},
                "c": {j: (self.w[j][4:4 + len(self.parts[j])] / 100).round(6).tolist() for j in JOINTS},
                "k_per_a": {j: round(float(self.w[j][4 + len(self.parts[j])]), 5) for j in JOINTS},
                "beta": {j: round(float(self.w[j][5 + len(self.parts[j])]), 4) for j in JOINTS}, "u_clip": U_CLIP}


class Stops:
    """Hard stops refitted on the data: per joint true = g x + o (the linear map with new stop angles)."""

    def __init__(self):
        self.w: dict[str, np.ndarray] = {}

    def fit(self, X: dict, Y: dict, ok: dict) -> Stops:
        for j in JOINTS:
            A, keep = np.c_[X[j], np.ones(len(X[j]))], ok[j].copy()
            for _ in range(3):
                w, *_ = np.linalg.lstsq(A[keep], Y[j][keep], rcond=None)
                res = Y[j] - A @ w
                mad = 1.4826 * np.median(np.abs(res[keep] - np.median(res[keep])))
                keep = ok[j] & (np.abs(res) <= max(TRIM_DEG, 3 * mad))
            self.w[j] = w
        return self

    def forward(self, x: dict) -> dict[str, float]:
        return {j: float(self.w[j][0] * x[j] + self.w[j][1]) for j in JOINTS}


# data

class Data:
    """Rows of solved poses with x (the base map), the motor currents and the approach directions."""

    def __init__(self, sessions: list[Path], base):
        self.R = hardstops.good(analysis.rows(sessions))
        dirs = {s.name: directions(s) for s in sessions}
        for r in self.R:
            r["x"] = {j: base(r, j) for j in JOINTS}
            r["d"] = dirs[r["session"]].get(r["id"], {j: 1 for j in JOINTS})
        self.X = {j: np.array([r["x"][j] for r in self.R]) for j in JOINTS}
        self.Y = {j: np.array([r["measured"][j] for r in self.R]) for j in JOINTS}
        self.I = {j: np.array([r["current"][j] for r in self.R]) for j in JOINTS}
        self.D = {j: np.array([r["d"][j] for r in self.R]) for j in JOINTS}
        self.ok = {j: np.array([(r.get("split") or {}).get(j, 0.0) <= hardstops.SPLIT_MAX_DEG for r in self.R])
                   for j in JOINTS}


def accuracy(R: list[dict], pred) -> dict:
    out, allerr = {}, []
    for j in JOINTS:
        e = np.array([r["measured"][j] - pred(r, j) for r in R if (r.get("split") or {}).get(j, 0.0) <= hardstops.SPLIT_MAX_DEG])
        allerr.append(e)
        out[j] = {"rms": round(float(np.sqrt(np.mean(e ** 2))), 2), "p50": round(float(np.median(np.abs(e))), 2),
                  "max": round(float(np.abs(e).max()), 2), "n": int(len(e))} if len(e) else None
    e = np.concatenate(allerr)
    out["all"] = {"rms": round(float(np.sqrt(np.mean(e ** 2))), 2), "p50": round(float(np.median(np.abs(e))), 2),
                  "p95": round(float(np.percentile(np.abs(e), 95)), 2), "max": round(float(np.abs(e).max()), 2),
                  "n": int(len(e))}
    return out


def cached(f):
    cache: dict = {}

    def pred(r, j):
        k = r["session"] + "/" + r["id"]
        if k not in cache:
            cache[k] = f(r)
        return cache[k][j]
    return pred


def load_seed() -> dict:
    return json.loads(SEED.read_text())["c"] if SEED.exists() else {}


def fit_all(tr: Data, seed: dict) -> dict:
    cm, cl = curve.Model(False), curve.Model(True)
    cm.fit(tr.X, tr.Y, tr.ok)
    cl.fit(tr.X, tr.Y, tr.ok)
    return {"stops": Stops().fit(tr.X, tr.Y, tr.ok), "curve": cm, "coupled": cl,
            "advanced": Advanced(seed).fit(tr.X, tr.Y, tr.I, tr.D, tr.ok)}


def predictors(m: dict) -> dict:
    return {"hard stops (refit)": cached(lambda r: m["stops"].forward(r["x"])),
            "curve": cached(lambda r: m["curve"].forward(r["x"])),
            "coupled table": cached(lambda r: m["coupled"].forward(r["x"])),
            "advanced": cached(lambda r: m["advanced"].forward(r["x"], r["current"], r["d"]))}


def params(m: dict) -> dict:
    return {"hard stops (refit)": 2 * len(JOINTS), "curve": m["curve"].params(), "coupled table": m["coupled"].params(),
            "advanced": m["advanced"].params()}


VARIANTS = {"curve (Legendre)": frozenset(), "+ wrist coupling": frozenset({"wrist"}),
            "+ finger couplings": frozenset({"wrist", "finger"}),
            "+ hysteresis/backlash": frozenset({"wrist", "finger", "backlash"}),
            "+ current/compliance (full)": TERMS}


def ablation(tr: Data, tests: dict[str, Data], seed: dict) -> dict:
    """Held-out RMS as the terms are added one by one, and with the coupling unseeded."""
    out = {}
    for name, terms in {**VARIANTS, "full, coupling unseeded": TERMS}.items():
        m = Advanced({} if "unseeded" in name else seed, terms=terms).fit(tr.X, tr.Y, tr.I, tr.D, tr.ok)
        out[name] = {t: accuracy(d.R, cached(lambda r, m=m: m.forward(r["x"], r["current"], r["d"])))["all"]
                     for t, d in tests.items()}
    return out


def export(hand: str, run: str, m: Advanced | Stops | curve.Model, kind: str, data: Data, extra: dict) -> None:
    """maps/<hand>/<kind>.json in orca-ui's study-map format (per-joint curves for display and
    the inverse's start; the advanced block holds the full model)."""
    joints = {}
    for j in JOINTS:
        xs = np.linspace(*ROM[j], 25)
        ys = m.own_curve(j, xs) if kind == "advanced" else np.array([m.forward({**{k: 0.0 for k in JOINTS}, j: x})[j] for x in xs])
        ys = np.maximum.accumulate(ys)
        if ys[-1] - ys[0] < 1e-6:
            ys = xs.copy()
        g, _ = np.polyfit(xs, ys, 1)
        rms = m.rms[j] if kind == "advanced" else None
        joints[j] = {"applied": True, "confidence": None if rms is None else "good" if rms < 2 else "fair" if rms < 4 else "low",
                     "n": int(data.ok[j].sum()), "cmd_knots": xs.round(3).tolist(), "cmd_true": ys.round(3).tolist(),
                     "assumed_knots": xs.round(3).tolist(), "assumed_true": ys.round(3).tolist(),
                     "reach": [round(float(ys[0]), 1), round(float(ys[-1]), 1)], "rom": list(ROM[j]),
                     "fit_rms_deg": None if rms is None else round(rms, 2), "gain": round(float(g), 3),
                     "offset_at_0": round(float(np.interp(0.0, xs, ys)), 2)}
    out = {"created": time.strftime("%Y-%m-%d %H:%M"), "n_poses": len(data.R), "kind": kind, "hand": hand, "run": run,
           "base": "orca_core's standard map with the hand's current config and calibration",
           "hysteresis": "corrected by the direction of motion" if kind == "advanced" else "not corrected",
           "joints": joints, **extra}
    if kind == "advanced":
        out["advanced"] = m.to_json()
    d = STUDY / "maps" / hand
    d.mkdir(parents=True, exist_ok=True)
    (d / f"{kind}.json").write_text(json.dumps(out, indent=1))


def evaluate(hand: str, tr: Data, tests: dict[str, Data], extra_maps: dict, extra_params: dict, seed: dict) -> dict:
    m = fit_all(tr, seed)
    P = {**extra_maps, **predictors(m)}
    res = {"hand": hand, "created": time.strftime("%Y-%m-%d %H:%M"), "n_train": len(tr.R),
           "train_sessions": sorted({r["session"] for r in tr.R}),
           "tests": {t: {"sessions": sorted({r["session"] for r in d.R}), "n": len(d.R),
                         "maps": {k: accuracy(d.R, f) for k, f in P.items()}} for t, d in tests.items()},
           "train_acc": {k: accuracy(tr.R, f)["all"] for k, f in P.items()},
           "params": {**extra_params, **params(m)}, "advanced": m["advanced"].summary(),
           "directions_seen": {j: sorted({int(v) for v in tr.D[j]}) for j in JOINTS},
           "current_range_ma": {j: [round(float(np.percentile(tr.I[j], 5))), round(float(np.percentile(tr.I[j], 95)))] for j in JOINTS},
           "ablation": ablation(tr, tests, seed),
           "inverse_check_deg": inverse_check(m["advanced"], tr)}
    OUT.mkdir(exist_ok=True)
    (OUT / f"{hand}.json").write_text(json.dumps(res, indent=1))
    return {"result": res, "models": m}


def inverse_check(m: Advanced, d: Data) -> float:
    """max |forward(command(want)) - want| over the training poses' measured angles."""
    worst = 0.0
    for r in d.R[:40]:
        want = {j: float(np.clip(r["measured"][j], *ROM[j])) for j in JOINTS}
        got = m.forward(m.command(want, r["d"], r["current"]), r["current"], r["d"])
        worst = max(worst, max(abs(got[j] - want[j]) for j in JOINTS))
    return round(worst, 3)


# the two hands

def ser4485(make_seed: bool) -> dict:
    """ser-4485's night: train on the dense block, held out V0 (same state) and V3-V5 (after wear,
    load and orca_core's recalibration). Base x = the new hard stops (b), per pose in its run."""
    from hand_study.lut import fit_rows
    tag = "20261108-night"
    dense = SESS / f"{tag}-N01-dense"
    stops = hardstops.fit(hardstops.good(analysis.rows([dense])), "run2")
    lin_b: dict = {}
    lin_a: dict = {}

    def base(r, j):
        if r["run"] not in lin_b:
            lin_b[r["run"]] = hardstops.Linear(r["run"], {k: stops[k]["new"] for k in JOINTS})
        return lin_b[r["run"]].joint(j, r["motor"][j])

    def old(r, j):
        if r["run"] not in lin_a:
            lin_a[r["run"]] = hardstops.Linear(r["run"])
        return lin_a[r["run"]].joint(j, r["motor"][j])
    tr = Data([dense], base)
    if make_seed:
        global LAM_COUPLE
        lc, LAM_COUPLE = LAM_COUPLE, 0.1
        m = Advanced({}).fit(tr.X, tr.Y, tr.I, tr.D, tr.ok)
        LAM_COUPLE = lc
        SEED.write_text(json.dumps({"source": "fleet prior: ser-4485 dense block (no CAD moment arms in the URDF, "
                                              "orca_core or orcahand_description); replace c by CAD moment-arm ratios "
                                              "r_partner / r_joint (signed) when available", "created": time.strftime("%Y-%m-%d %H:%M"),
                                    "c": {j: s["coupling"] for j, s in m.summary().items()}}, indent=1))
    tests = {n: Data([SESS / f"{tag}-{b}"], base) for n, b in (("V0", "N02-V0"), ("V3", "N05-V3"), ("V4", "N06-V4"), ("V5", "N08-V5"))}
    lut = fit_rows(tr.R)
    extra = {"standard (config then)": lambda r, j: old(r, j), "hard stops (b, dense)": lambda r, j: r["x"][j],
             "LUT (study, lut.py)": cached(lambda r: lut.forward(r["commanded"]))}
    return evaluate("ser-4485", tr, tests, extra, {"standard (config then)": 34, "hard stops (b, dense)": 34,
                                                  "LUT (study, lut.py)": sum(t.w.size for t in lut.tables.values())},
                    load_seed())


def ser7577(train: list[Path], tests: dict[str, list[Path]], run: str, export_maps: bool) -> dict:
    """ser-7577: base x = its current config and calibration (own refit stops); maps on top."""
    L = hardstops.Linear(run)
    bench = Path.home() / "orca_bench/ser-7577/orcahand-right"
    rom = {j: list(map(float, v)) for j, v in yaml.safe_load((bench / "config.orig.yaml").read_text())["joint_roms"].items()}
    Lp = hardstops.Linear("s7577a", rom)

    def base(r, j):
        return L.joint(j, r["motor"][j])
    tr = Data(train, base)
    T = {n: Data(s, base) for n, s in tests.items()}
    extra = {"packaged config": lambda r, j: Lp.joint(j, r["motor"][j]), "standard (own hard stops)": lambda r, j: r["x"][j]}
    out = evaluate("ser-7577", tr, T, extra, {"packaged config": 34, "standard (own hard stops)": 34}, load_seed())
    if export_maps:
        first = next(iter(T))
        allr = Data(train + [p for s in tests.values() for p in s], base)
        m = fit_all(allr, load_seed())
        acc = out["result"]["tests"][first]["maps"]
        std = acc["standard (own hard stops)"]["all"]
        for kind, key in (("advanced", "advanced"), ("stops", "hard stops (refit)")):
            export("ser-7577", run, m[kind], kind, allr, {"params": params(m)[key], "heldout": acc[key]["all"],
                                                          "heldout_standard": std, "heldout_set": first})
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("hand", choices=("ser4485", "ser7577"))
    p.add_argument("--make-seed", action="store_true")
    p.add_argument("--run", default="s7577b")
    p.add_argument("--train", type=Path, nargs="+")
    p.add_argument("--test", nargs="+", help="NAME=SESSION[,SESSION]")
    p.add_argument("--export", action="store_true")
    a = p.parse_args()
    if a.hand == "ser4485":
        o = ser4485(a.make_seed)
    else:
        train = a.train or [SESS / "20261108-s7577-N07-short"]
        tests = {t.split("=")[0]: [Path(s) if "/" in s else SESS / s for s in t.split("=")[1].split(",")] for t in a.test} \
            if a.test else {"V": [SESS / "20261108-s7577-N08-V"]}
        o = ser7577(train, tests, a.run, a.export)
    r = o["result"]
    for t, d in r["tests"].items():
        print(f"-- {t} (n={d['n']})")
        for k, v in d["maps"].items():
            a_ = v["all"]
            print(f"  {k:28s} rms {a_['rms']:5.2f}  p50 {a_['p50']:5.2f}  max {a_['max']:5.1f}  params {r['params'].get(k)}")
    print("ablation", json.dumps({k: {t: v['rms'] for t, v in d.items()} for k, d in r["ablation"].items()}))
    print("inverse check", r["inverse_check_deg"])
