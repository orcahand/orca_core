"""Adaptive trajectories (Clemens, 9 Nov): the night uses what it has learnt so far. From every
solved training pose: per joint and ROM bin the residual of a per-joint curve, the hysteresis
(approached from above vs below) and the coverage. Then

- prune: a block's poses for a joint that is already well determined are dropped (at most 40 %);
- densify: poses where the residuals or the hysteresis are large or the bins are empty, from both sides;
- re-aim: poses that went out of frame (cut views), were suspect, or collided/stuck, moved 10° inwards.

Every change is logged with its reason (night/<tag>-adaptive.jsonl) and summed up in the Study feed.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from hand_study import analysis, curve, hardstops
from hand_study.configs import JOINTS, ROM
from hand_study.night_advanced import approach
from hand_study.plan import Builder

NIGHT = analysis.STUDY / "night"
BINS = 6
WELL_N = 30
PRUNE_MAX = 0.4
REAIM_DEG = 10.0


def state(spec: dict) -> dict:
    """Per joint: bins (n, residual rms, hysteresis), overall rms, noise floor; the poses to re-aim."""
    train = [Path(p) for k in ("S1", "S2", "deep") for p in spec["sets"].get(k, [])]
    R = hardstops.good(analysis.rows([p for p in train if (p / "configs").exists()]))
    out = {"n": len(R), "joints": {}, "reaim": []}
    if len(R) < 10:
        return out
    L = {r["run"]: hardstops.Linear(r["run"]) for r in R}
    X = {j: np.array([L[r["run"]].joint(j, r["motor"][j]) for r in R]) for j in JOINTS}
    Y = {j: np.array([r["measured"][j] for r in R]) for j in JOINTS}
    ok = {j: np.ones(len(R), bool) for j in JOINTS}
    cm = curve.Model(False)
    cm.fit(X, Y, ok)
    for j in JOINTS:
        pred = np.array([cm.forward({k: X[k][i] for k in JOINTS})[j] for i in range(len(R))])
        res = Y[j] - pred
        edges = np.linspace(*ROM[j], BINS + 1)
        b = np.clip(np.digitize(Y[j], edges) - 1, 0, BINS - 1)
        dirs = np.array([(r.get("dir") == "down") for r in R])
        bins = []
        for k in range(BINS):
            m = b == k
            up, dn = res[m & ~dirs], res[m & dirs]
            bins.append({"lo": round(float(edges[k]), 1), "hi": round(float(edges[k + 1]), 1), "n": int(m.sum()),
                         "rms": round(float(np.sqrt(np.mean(res[m] ** 2))), 2) if m.any() else None,
                         "hyst": round(float(dn.mean() - up.mean()), 2) if len(up) >= 2 and len(dn) >= 2 else None})
        split = [r["split"][j] for r in R if r.get("split")]
        out["joints"][j] = {"rms": round(float(np.sqrt(np.mean(res ** 2))), 2), "noise": round(float(np.median(split)), 2) if split else 0.5,
                            "n": len(R), "bins": bins}
    for p in [Path(x) for x in spec["sessions"]]:
        for d in (p / "configs").glob("*") if (p / "configs").exists() else []:
            e, sk = d / "est.json", d / "skipped.json"
            why = None
            if e.exists():
                est = json.loads(e.read_text())
                if est.get("cut_views"):
                    why = f"cut off in {len(est['cut_views'])} views"
                elif est.get("suspect"):
                    why = "suspect masks"
            elif sk.exists() and json.loads(sk.read_text()).get("why") == "fault":
                why = "collided / stuck"
            if why:
                q = json.loads((d / "meta.json").read_text())["q"] if (d / "meta.json").exists() else None
                if q is None:
                    plan = json.loads((p / "plan.json").read_text())
                    q = next((c["q"] for c in plan["configs"] if c["id"] == d.name), None)
                if q:
                    out["reaim"].append({"id": f"{p.name}/{d.name}", "q": q, "why": why})
    return out


def well(st: dict, j: str) -> bool:
    J = st["joints"].get(j)
    return bool(J and J["n"] >= WELL_N and J["rms"] <= 1.2 * J["noise"] + 0.3 and all(b["n"] >= 3 for b in J["bins"]))


def log_change(tag: str, kind: str, cid: str, why: str) -> None:
    with (NIGHT / f"{tag}-adaptive.jsonl").open("a") as f:
        f.write(json.dumps({"t": time.time(), "kind": kind, "id": cid, "why": why}) + "\n")


def prune(tag: str, plan: dict, st: dict) -> list[str]:
    """Drop (at most PRUNE_MAX of) the block's poses whose joint is already well determined."""
    cs = plan["configs"]
    drop = [c for c in cs if c.get("joint") and well(st, c["joint"])][: int(PRUNE_MAX * len(cs))]
    for c in drop:
        J = st["joints"][c["joint"]]
        log_change(tag, "skip", c["id"], f"{c['joint']} well determined: rms {J['rms']}° vs noise {J['noise']}°, n {J['n']}")
    ids = {c["id"] for c in drop}
    plan["configs"] = [c for c in cs if c["id"] not in ids]
    return sorted(ids)


def densify(tag: str, st: dict, n_max: int = 30) -> tuple[Builder, list[dict]]:
    """New poses where the model is weakest: worst bins first (residual over noise, hysteresis, gaps)."""
    b = Builder()
    cand = []
    for j, J in st["joints"].items():
        for k, bn in enumerate(J["bins"]):
            score, why = 0.0, []
            if bn["n"] < 3:
                score += 3 - bn["n"]
                why.append(f"only {bn['n']} poses")
            if bn["rms"] is not None and bn["rms"] > 2 * J["noise"]:
                score += bn["rms"] / max(J["noise"], 0.2)
                why.append(f"residual {bn['rms']}° (noise {J['noise']}°)")
            if bn["hyst"] is not None and abs(bn["hyst"]) > 1.5:
                score += abs(bn["hyst"])
                why.append(f"hysteresis {bn['hyst']}°")
            if score:
                cand.append((score, j, k, bn, "; ".join(why)))
    made = []
    for score, j, k, bn, why in sorted(cand, reverse=True):
        if len(b.out) >= n_max:
            break
        v = round((bn["lo"] + bn["hi"]) / 2, 1)
        q = b.free_with_neighbour(j, v, {})
        if q is None:
            continue
        for d in (1, -1):
            cid = f"ad-{j}-{k}-{'up' if d > 0 else 'down'}"
            if b.add(cid, q, kind="sweep", joint=j, dir="up" if d > 0 else "down", value=v, adaptive=why):
                b.out[-1]["dirs"] = {j: d}
                made.append({"id": cid, "why": f"{j} {bn['lo']}..{bn['hi']}°: {why}"})
    for r in st["reaim"]:
        if len(b.out) >= n_max + 10:
            break
        q = {j: float(np.clip(v - np.sign(v - np.mean(ROM[j])) * REAIM_DEG, *ROM[j])) for j, v in r["q"].items()}
        cid = "re-" + r["id"].replace("/", "-")[-40:]
        if b.add(cid, q, kind="val", adaptive=f"re-aimed: {r['why']}"):
            made.append({"id": cid, "why": f"re-aimed {r['id']} ({r['why']}): extreme joints {REAIM_DEG:g}° inwards"})
    approach(b)
    for m in made:
        log_change(tag, "add", m["id"], m["why"])
    return b, made


def free_design(spec: dict, st: dict, results: dict | None, n: int = 40) -> tuple[Builder, str]:
    """The automatic free experiment (when no plan was written): the joints whose held-out error is
    largest relative to their noise; hypothesis: the error is direction-dependent (hysteresis/backlash).
    Test: 5 angles over the ROM, each approached from below and from above, twice."""
    err = {}
    ab = (results or {}).get("ablation") or {}
    for t in ab.get("tests", {}).values():
        m = t.get("per-joint curve") or {}
        for j, a in (m.get("deg") or {}).items():
            if j != "all":
                err.setdefault(j, []).append(a["rms"])
    if err:
        score = {j: np.mean(v) / max(st["joints"].get(j, {}).get("noise", 0.5), 0.2) for j, v in err.items()}
    else:
        score = {j: J["rms"] / max(J["noise"], 0.2) for j, J in st["joints"].items()}
    top = sorted(score, key=score.get, reverse=True)[:2]
    b = Builder()
    for j in top:
        for v in np.linspace(*ROM[j], 7)[1:-1].round(1):
            q = b.free_with_neighbour(j, float(v), {})
            if q is None:
                continue
            for rep in (0, 1):
                for d in (1, -1):
                    if b.add(f"fx-{j}-{v:g}-{'up' if d > 0 else 'down'}-{rep}", q, kind="sweep", joint=j,
                             dir="up" if d > 0 else "down", value=float(v)):
                        b.out[-1]["dirs"] = {j: d}
    approach(b)
    b.out = b.out[:n]
    why = (f"free experiment: {', '.join(top)} have the largest held-out error over their noise "
           f"({', '.join(f'{j} x{score[j]:.1f}' for j in top)}); hypothesis: it is direction-dependent (hysteresis/backlash). "
           f"Test: {len(b.out)} poses, 5 angles each from below and above, twice.")
    return b, why
