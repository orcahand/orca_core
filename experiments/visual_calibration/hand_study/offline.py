"""The protocol's analysis, recomputed as poses get solved (--follow), to protocol/<tag>/results.json
for the study page's Protocol tab:

- the hand as run in each phase (the map in force) on every V set;
- P1: V1 scored offline from the same photos with the packaged, ser-4485 and ser-7577 stops (and
  the own stops once fitted);
- the map variants on the held-out sets (V1-V5, X), trained on S1 + S2 + deep: packaged / foreign
  stops, own stops, stops refit, per-joint curve, the advanced model term by term, coupled table;
  joint degrees and fingertip millimetres (forward kinematics, measured vs predicted angles);
- the headline: positional accuracy after the recalibration (V4, V5, X) per joint and fingertip;
- quality: view-split noise floor, repeatability (V repeats), drift (V2 -> V3), silhouette
  residual, segmenter agreement (BiRefNet primary vs GrabCut and HQ-SAM: angles and mask IoU).

    python -m hand_study.offline TAG [--follow]
"""

from __future__ import annotations

import argparse
import json
import time
import traceback
from pathlib import Path

import numpy as np
import yaml

from hand_study import advanced as adv
from hand_study import analysis, curve, hardstops
from hand_study.configs import JOINTS

STUDY = analysis.STUDY
NIGHT = STUDY / "night"
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
SEGMENTERS = {"birefnet": "est.json", "grabcut": "est_grabcut.json", "hqsam": "est_hqsam.json"}


def roms(spec: dict) -> dict[str, dict[str, list[float]]]:
    out = {"packaged": yaml.safe_load(Path(spec.get("packaged_config") or spec["rom_config"]).read_text())["joint_roms"]}
    f = spec.get("foreign", {})
    if Path(f.get("ser-4485", "")).exists():
        out["ser-4485 stops"] = {j: s["new"] for j, s in json.loads(Path(f["ser-4485"]).read_text())["stops"].items()}
    if Path(f.get("ser-7577", "")).exists():
        out["ser-7577 stops"] = yaml.safe_load(Path(f["ser-7577"]).read_text())["joint_roms"]
    own = Path(spec["protocol_dir"]) / "refit-stops.json"
    if own.exists():
        out["own stops"] = {j: s["new"] for j, s in json.loads(own.read_text())["stops"].items()}
    return {k: {j: list(map(float, v[j])) for j in JOINTS} for k, v in out.items()}


def linear(rom: dict | None):
    made: dict = {}

    def f(r):
        if r["run"] not in made:
            made[r["run"]] = hardstops.Linear(r["run"], rom)
        L = made[r["run"]]
        return {j: L.joint(j, r["motor"][j]) for j in JOINTS}
    return f


def acc(R: list[dict], pred) -> dict:
    out, allerr = {}, []
    P = [pred(r) for r in R]
    for j in JOINTS:
        e = np.array([r["measured"][j] - p[j] for r, p in zip(R, P) if (r.get("split") or {}).get(j, 0.0) <= hardstops.SPLIT_MAX_DEG])
        allerr.append(e)
        if len(e):
            a = np.abs(e)
            out[j] = {"rms": round(float(np.sqrt(np.mean(e ** 2))), 2), "p50": round(float(np.median(a)), 2),
                      "p95": round(float(np.percentile(a, 95)), 2), "max": round(float(a.max()), 2),
                      "mean": round(float(e.mean()), 2), "n": int(len(e))}
    e = np.concatenate(allerr) if allerr else np.zeros(0)
    if len(e):
        a = np.abs(e)
        out["all"] = {"rms": round(float(np.sqrt(np.mean(e ** 2))), 2), "p50": round(float(np.median(a)), 2),
                      "p95": round(float(np.percentile(a, 95)), 2), "max": round(float(a.max()), 2), "n": int(len(e)), "mae": round(float(a.mean()), 2)}
    return out


class Tips:
    """Fingertip positions (mm, hand base frame) by forward kinematics on the URDF."""

    def __init__(self):
        from visual_calibration.hand_model import HandModel
        self.model = HandModel.load("right")
        self.tips = {f: self.model.fingertip(f) for f in FINGERS}
        self.cache: dict = {}

    def of(self, q: dict) -> dict[str, np.ndarray]:
        key = tuple(round(float(q[j]), 2) for j in JOINTS)
        if key not in self.cache:
            poses = self.model.link_poses({j: float(q[j]) for j in JOINTS})
            self.cache[key] = {f: poses[l].apply_to_points(t[None])[0] * 1000 for f, (l, t) in self.tips.items()}
        return self.cache[key]

    def errors(self, R: list[dict], pred) -> dict:
        per = {f: [] for f in FINGERS}
        for r in R:
            ok = {j for j in JOINTS if (r.get("split") or {}).get(j, 0.0) <= hardstops.SPLIT_MAX_DEG}
            a, b = self.of(r["measured"]), self.of(pred(r))
            for f in FINGERS:
                if "wrist" in ok and all(j in ok for j in JOINTS if j.startswith(f)):
                    per[f].append(float(np.linalg.norm(a[f] - b[f])))
        out = {}
        for f, v in [*per.items(), ("all", sum(per.values(), []))]:
            if v:
                v = np.array(v)
                out[f] = {"rms": round(float(np.sqrt(np.mean(v ** 2))), 2), "p50": round(float(np.median(v)), 2),
                          "p95": round(float(np.percentile(v, 95)), 2), "max": round(float(v.max()), 2), "n": int(len(v)), "mean": round(float(v.mean()), 2)}
        return out


def rows_of(paths: list[str], est: str = "est.json") -> list[dict]:
    ps = [Path(p) for p in paths if (Path(p) / "configs").exists()]
    return hardstops.good(analysis.rows(ps, est)) if ps else []


def counts(spec: dict) -> dict:
    out = {}
    for s in map(Path, spec["sessions"]):
        plan = json.loads((s / "plan.json").read_text())
        c = s / "configs"
        d = {"planned": len(plan["configs"]), "shot": len(list(c.glob("*/meta.json"))),
             "skipped": len(list(c.glob("*/skipped.json"))),
             "solved": {k: len(list(c.glob(f"*/{v}"))) for k, v in SEGMENTERS.items()},
             "errors": len(list(c.glob("*/est.json.error")))}
        sus = 0
        for e in c.glob("*/est.json"):
            try:
                sus += bool(json.loads(e.read_text()).get("suspect"))
            except Exception:
                pass
        d["suspect"] = sus
        out[s.name.split(f"{spec['tag']}-", 1)[-1]] = d
    return out


def quality(spec: dict, sets: dict[str, list[dict]]) -> dict:
    out: dict = {}
    allr = [r for R in sets.values() for r in R]
    if not allr:
        return out
    out["noise_floor"] = {j: round(float(np.median([r["split"][j] for r in allr if r.get("split")])), 2) for j in JOINTS}
    out["silhouette_px"] = {k: {"p50": round(float(np.median([r["rms_px"] for r in R])), 2),
                                "p95": round(float(np.percentile([r["rms_px"] for r in R], 95)), 2), "n": len(R)}
                            for k, R in sets.items() if R}
    rep = {j: [] for j in JOINTS}
    for k, R in sets.items():
        if not k.startswith("V"):
            continue
        by = {r["id"]: r for r in R}
        for r in R:
            if r["id"].endswith("-r") and r["id"][:-2] in by:
                for j in JOINTS:
                    rep[j].append(r["measured"][j] - by[r["id"][:-2]]["measured"][j])
    out["repeatability"] = {j: {"sd": round(float(np.sqrt(np.mean(np.square(v)) / 2)), 2), "max": round(float(np.abs(v).max()), 2),
                                "n": len(v)} for j, v in rep.items() if v}
    for a, b, name in (("V2", "V3", "exercise (V2 -> V3)"), ("V3", "V4", "recalibration (V3 -> V4)"),
                       ("V5", "V6", "collisions + recalibration (V5 -> V6)")):
        if sets.get(a) and sets.get(b):
            A = {r["id"]: r for r in sets[a]}
            d = {j: [r["measured"][j] - A[r["id"]]["measured"][j] for r in sets[b] if r["id"] in A] for j in JOINTS}
            out.setdefault("drift", {})[name] = {j: {"mean": round(float(np.mean(v)), 2), "rms": round(float(np.sqrt(np.mean(np.square(v)))), 2)}
                                                 for j, v in d.items() if v}
    out["segmenters"] = segmenters(spec)
    return out


def iou(a: Path, b: Path) -> list[float]:
    A, B = np.load(a), np.load(b)
    out = []
    for k, ang in enumerate(A["angle"]):
        kb = [i for i, x in enumerate(B["angle"]) if abs(x - ang) < 0.5]
        if not kb:
            continue
        H, W = 540 + 200, 960 + 200
        ca, cb = np.zeros((H, W), bool), np.zeros((H, W), bool)
        for c, D, i in ((ca, A, k), (cb, B, kb[0])):
            fg, (u, v) = D[f"fg{i}"], D["offset"][i]
            h, w = min(fg.shape[0], H - v), min(fg.shape[1], W - u)
            c[v:v + h, u:u + w] = fg[:h, :w]
        out.append(float((ca & cb).sum() / max((ca | cb).sum(), 1)))
    return out


def segmenters(spec: dict) -> dict:
    prim = {}
    for s in map(Path, spec["sessions"]):
        for d in (s / "configs").glob("*"):
            if (d / "est.json").exists():
                prim[d] = json.loads((d / "est.json").read_text())
    out = {"primary": spec.get("primary", "birefnet"), "n_primary": len(prim), "rms_px": {}, "joints": {}, "iou": {}}
    out["rms_px"][out["primary"]] = round(float(np.median([e["rms_px"] for e in prim.values()])), 2) if prim else None
    for seg, est in SEGMENTERS.items():
        if est == "est.json":
            continue
        dj, px, io = {j: [] for j in JOINTS}, [], []
        for d, e in prim.items():
            f = d / est
            if not f.exists():
                continue
            o = json.loads(f.read_text())
            px.append(o["rms_px"])
            for j in JOINTS:
                dj[j].append(o["joints"][j] - e["joints"][j])
            if (d / "views.npz").exists() and (d / f"views_{Path(est).stem}.npz").exists():
                io += iou(d / "views.npz", d / f"views_{Path(est).stem}.npz")
        if px:
            out["rms_px"][seg] = round(float(np.median(px)), 2)
            out["joints"][seg] = {j: {"p50": round(float(np.median(np.abs(v))), 2), "rms": round(float(np.sqrt(np.mean(np.square(v)))), 2)}
                                  for j, v in dj.items()}
            out["joints"][seg]["n"] = len(px)
            out["iou"][seg] = {"p50": round(float(np.median(io)), 3), "p05": round(float(np.percentile(io, 5)), 3), "n": len(io)} if io else None
    return out


def analyse(tag: str) -> dict:
    spec = json.loads((NIGHT / f"{tag}.json").read_text())
    hardstops.RMS_MAX_PX = float(spec.get("rms_max_px", hardstops.RMS_MAX_PX))
    R = roms(spec)
    sets = {k: rows_of(v) for k, v in spec["sets"].items()}
    tips = Tips()
    res: dict = {"tag": tag, "hand": spec["hand"], "updated": time.strftime("%Y-%m-%d %H:%M:%S"), "counts": counts(spec),
                 "roms": R, "n": {k: len(v) for k, v in sets.items()}}
    std = linear(None)
    res["as_run"] = {k: {"deg": acc(v, std), "mm": tips.errors(v, std)} for k, v in sets.items() if k.startswith("V") and v}
    if sets.get("V1"):
        res["p1"] = {"set": "V1", "maps": {name: {"deg": acc(sets["V1"], linear(rom)), "mm": tips.errors(sets["V1"], linear(rom))}
                                           for name, rom in R.items()}}
    train = [r for k in ("S1", "S2", "deep") for r in sets.get(k, [])]
    tests = {k: v for k, v in sets.items() if (k.startswith("V") or k == "X") and v}
    global spec_label
    spec_label = spec.get("label", "")
    res["label"] = spec_label
    from hand_study.colour import skin_tone
    res["skin"] = spec.get("skin") or skin_tone(spec.get("bench"))
    if "own stops" in R and len(train) >= 30 and tests:
        res["ablation"] = ablation(spec, R, train, tests, tips)
        late = [r for k in ("V4", "V5", "V6", "X") for r in sets.get(k, [])]
        if late:
            res["headline"] = headline(res["ablation"], late, tips)
    res["quality"] = quality(spec, sets)
    fa = STUDY / "protocol" / tag / "fit_ablation.json"
    if fa.exists():
        res["fit_ablation"] = json.loads(fa.read_text())
    res["weighting_rerun"] = {h: json.loads((STUDY / "protocol" / h / "fit_ablation.json").read_text())
                              for h in ("ser-4485", "ser-7577") if (STUDY / "protocol" / h / "fit_ablation.json").exists()}
    ad = NIGHT / f"{tag}-adaptive.jsonl"
    if ad.exists():
        res["adaptive"] = [json.loads(x) for x in ad.read_text().splitlines()[-200:] if x.strip()]
    res["collisions"] = collisions(spec)
    res["slack"] = slack_of(tag, spec)
    return res


def slack_of(tag: str, spec: dict) -> dict | None:
    from hand_study import slack
    n = sum(len(list(Path(s).glob("configs/*/slack.json"))) for s in spec["sets"].get("slack", []))
    p = STUDY / "protocol" / tag / "slack.json"
    old = json.loads(p.read_text()) if p.exists() else None
    if not n:
        return None
    if old and old.get("n_files") == n:
        return old
    try:
        out = slack.analyse(tag)
    except Exception:
        out = {"error": traceback.format_exc()[-400:], "checks": []}
    out["n_files"] = n
    p.write_text(json.dumps(out, indent=1, default=float))
    return out


def collisions(spec: dict) -> dict:
    out = {"pairs": 0, "contact": 0, "unexpected": 0, "none": 0, "by_pair": {}}
    for s in spec["sets"].get("collision", []):
        for f in sorted(Path(s).glob("configs/*/contact.json")):
            c = json.loads(f.read_text())
            p = out["by_pair"].setdefault(c["pair"]["id"], {"urdf": c["pair"]["angle"], "window": c["pair"]["window"], "reps": []})
            p["reps"].append({"result": c["result"], "est": (c.get("at") or {}).get("est"), "why": c.get("why")})
            out[c["result"]] = out.get(c["result"], 0) + 1
    out["pairs"] = len(out["by_pair"])
    return out


def ablation(spec: dict, R: dict, train: list[dict], tests: dict[str, list[dict]], tips: Tips) -> dict:
    own = R["own stops"]
    base = linear(own)
    dirs: dict = {}

    def prep(rows):
        for r in rows:
            r["x"] = base(r)
            s = r["session"]
            if s not in dirs:
                dirs[s] = adv.directions(STUDY / "sessions" / s)
            r["d"] = dirs[s].get(r["id"], {j: 1 for j in JOINTS})
        return rows
    prep(train)
    for v in tests.values():
        prep(v)
    X = {j: np.array([r["x"][j] for r in train]) for j in JOINTS}
    Y = {j: np.array([r["measured"][j] for r in train]) for j in JOINTS}
    I = {j: np.array([r["current"][j] for r in train]) for j in JOINTS}
    D = {j: np.array([r["d"][j] for r in train]) for j in JOINTS}
    ok = {j: np.array([(r.get("split") or {}).get(j, 0.0) <= hardstops.SPLIT_MAX_DEG for r in train]) for j in JOINTS}
    seed = adv.load_seed()
    maps, params = {}, {}
    for name, rom in R.items():
        if name != "own stops":
            maps[name], params[name] = linear(rom), 34
    maps["own stops"], params["own stops"] = (lambda r: r["x"]), 34
    st = adv.Stops().fit(X, Y, ok)
    maps["stops refit (all training)"], params["stops refit (all training)"] = (lambda r: st.forward(r["x"])), 34
    cm = curve.Model(False)
    cm.fit(X, Y, ok)
    maps["per-joint curve"], params["per-joint curve"] = (lambda r: cm.forward(r["x"])), cm.params()
    models = {}
    for name, terms in adv.VARIANTS.items():
        m = adv.Advanced(seed, terms=terms).fit(X, Y, I, D, ok)
        models[name] = m
        maps[f"advanced: {name}"] = (lambda r, m=m: m.forward(r["x"], r["current"], r["d"]))
        params[f"advanced: {name}"] = m.params()
    from hand_study import collision
    cons = collision.constraints([Path(p) for p in spec["sets"].get("collision", [])], base)
    if cons:
        for W in (1.0, 5.0):            # a contact counts W poses' worth (squared residual) for its moving joint
            X2 = {j: np.r_[X[j], [c["x"][j] for c in cons]] for j in JOINTS}
            Y2 = {j: np.r_[Y[j], [c["y"] if c["joint"] == j else c["x"][j] for c in cons]] for j in JOINTS}
            ok2 = {j: np.r_[ok[j], [c["joint"] == j for c in cons]] for j in JOINTS}
            wt = {j: np.r_[np.ones(len(train)), np.full(len(cons), W)] for j in JOINTS}
            cc = curve.Model(False)
            cc.fit(X2, Y2, ok2, wt)
            name = f"per-joint curve + {len(cons)} contacts (w {W:g})"
            maps[name], params[name] = (lambda r, cc=cc: cc.forward(r["x"])), cc.params()
    from hand_study import slack
    bp = slack.prior(spec["tag"])
    if bp:
        m = adv.Advanced(seed, terms=adv.TERMS, beta_prior=bp).fit(X, Y, I, D, ok)
        maps["advanced: full + slack prior"] = (lambda r, m=m: m.forward(r["x"], r["current"], r["d"]))
        params["advanced: full + slack prior"] = m.params()
    cl = curve.Model(True)
    cl.fit(X, Y, ok)
    maps["LUT (coupled table)"], params["LUT (coupled table)"] = (lambda r: cl.forward(r["x"])), cl.params()
    full = models["+ current/compliance (full)"]
    return {"train_n": len(train), "params": params, "maps": list(maps),
            "tests": {t: {k: {"deg": acc(v, f), "mm": tips.errors(v, f)} for k, f in maps.items()} for t, v in tests.items()},
            "advanced": full.summary(), "_maps": maps}


spec_label = ""


def headline(ab: dict, late: list[dict], tips: Tips) -> dict:
    maps = ab.pop("_maps")
    show = ["ser-4485 stops", "ser-7577 stops", "packaged", "own stops", "per-joint curve", "advanced: + current/compliance (full)", "LUT (coupled table)",
            *[k for k in maps if "contacts" in k]]
    out = {"set": "V4 + V5 + V6 + X (held out, after a recalibration)", "n": len(late), "maps": {}}
    for k in show:
        if k in maps:
            out["maps"][k] = {"deg": acc(late, maps[k]), "mm": tips.errors(late, maps[k])}
    best = min(out["maps"], key=lambda k: out["maps"][k]["deg"].get("all", {}).get("rms", 1e9))
    out["best"] = best
    out["label"] = spec_label
    out["methods"] = {"held_out_only": True, "sets": out["set"], "n": len(late), "train_n": ab["train_n"],
                      "segmenter": "BiRefNet (crop at the URDF estimate -> segment -> clip), mount fixed per night",
                      "weights": "every link equal (tower 0)", "map": best,
                      "error": "true (vision) - map; fingertip by URDF forward kinematics", "rig": "calibration_rig_330"}
    return out


def main(tag: str, follow: bool) -> None:
    out = STUDY / "protocol" / tag
    out.mkdir(parents=True, exist_ok=True)
    last = None
    while True:
        spec = json.loads((NIGHT / f"{tag}.json").read_text())
        sig = sum(len(list(Path(s).glob("configs/*/est*.json"))) for s in spec["sessions"])
        sig = (sig, (out / "refit-stops.json").exists(), (out / "fit_ablation.json").stat().st_mtime if (out / "fit_ablation.json").exists() else 0)
        if sig != last:
            try:
                res = analyse(tag)
                res.get("ablation", {}).pop("_maps", None)
                tmp = out / "results.tmp"
                tmp.write_text(json.dumps(res, indent=1, default=float))
                tmp.replace(out / "results.json")
                try:
                    from hand_study import registry
                    registry.update(tag)
                except Exception:
                    print(traceback.format_exc(), flush=True)
                print(time.strftime("%H:%M:%S"), "results updated", res["n"], flush=True)
                last = sig
            except Exception:
                (out / "results.error").write_text(traceback.format_exc())
                print(traceback.format_exc(), flush=True)
        if not follow:
            return
        time.sleep(120)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("tag")
    p.add_argument("--follow", action="store_true")
    a = p.parse_args()
    main(a.tag, a.follow)
