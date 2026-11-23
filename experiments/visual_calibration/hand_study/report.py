"""Export everything the morning report needs to hand_study/report/: results.json (every
number), CSV series for the charts, key photos, and findings.md (generated numbers; the
interpretation is added by hand).

    python -m hand_study.report TAG [--loo]
"""

from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path

import numpy as np

from hand_study import analysis, hardstops
from hand_study.configs import JOINTS
from hand_study.lut import fit_rows

STUDY = analysis.STUDY
OUT = STUDY / "report"
FINGERS = ("index", "middle", "ring", "pinky")


def session(tag: str, block: str) -> Path:
    return STUDY / "sessions" / f"{tag}-{block}"


def blocks(tag: str) -> dict[str, Path]:
    return {p.name[len(tag) + 1:]: p for p in sorted((STUDY / "sessions").glob(f"{tag}-N*"))}


def runs_of(R: list[dict]) -> set[str]:
    return {r["run"] for r in R}


def predictors(R_train: list[dict], stops: dict, sac_stops: dict | None, lut):
    cache: dict = {}

    def lin(rom):
        made: dict = {}

        def f(r, j):
            if r["run"] not in made:
                made[r["run"]] = hardstops.Linear(r["run"], rom)
            return made[r["run"]].joint(j, r["motor"][j])
        return f

    def lut_p(r, j):
        k = r["session"] + r["id"]
        if k not in cache:
            cache[k] = lut.forward(r["commanded"])
        return cache[k][j]

    out = {"a_old": lin(None), "b_new_stops": lin({j: stops[j]["new"] for j in JOINTS}), "c_lut": lut_p}
    if sac_stops:
        out["b_sac"] = lin({j: sac_stops[j]["sac"] for j in JOINTS})
    return out


def kapandji(path: Path) -> list[dict]:
    from hand_study.configs import Collider
    from hand_study.touch import min_dist
    col = Collider(spacing=1.5)
    out = []
    for d in sorted((path / "configs").glob("kap-*")):
        meta = json.loads((d / "meta.json").read_text()) if (d / "meta.json").exists() else None
        if not meta:
            continue
        est = json.loads((d / "est.json").read_text()) if (d / "est.json").exists() else None
        f = meta["finger"]
        tac = [s.get("tactile") or {} for s in (meta["before"], meta["after"])]
        force = {k: max(float(np.linalg.norm(t.get(k) or [0, 0, 0])) for t in tac) for k in ("thumb", f)}
        out.append({"id": d.name, "finger": f, "map": meta["map"],
                    "distance_mm": round(min_dist(col, est["joints"], "thumb_dp", f"{f}_dp"), 2) if est else None,
                    "fit_px": est and round(est["rms_px"], 2),
                    "force_thumb": round(force["thumb"], 3), "force_finger": round(force[f], 3),
                    "touch": bool(force["thumb"] >= 0.12 or force[f] >= 0.12), "photo": f"kapandji/{d.name}.jpg"})
    return out


def main(tag: str, loo: bool = False) -> dict:
    OUT.mkdir(exist_ok=True)
    B = blocks(tag)
    spec = json.loads((STUDY / "night" / f"{tag}.json").read_text())
    run_a = spec["runs"]["A"]
    dense_all = analysis.rows([B["N01-dense"]])
    dense = hardstops.good(dense_all)
    blunders = [{"session": r["session"], "id": r["id"], "rms_px": r["rms_px"]} for r in dense_all if r not in dense]
    stops = hardstops.fit(dense, run_a)
    lut = fit_rows(dense)
    sac_path = STUDY / "proposed" / "sac.json"
    if loo and (B.get("N12-sac") and (B["N12-sac"] / "contacts.jsonl").exists()):
        from hand_study import sac_fit
        sac_fit.main([B["N01-dense"]], B["N12-sac"], run_a, spec["runs"]["B"], loo=True)
    sac = json.loads(sac_path.read_text()) if sac_path.exists() else None
    P = predictors(dense, stops, sac and sac["stops"], lut)
    res = {"tag": tag, "runs": spec["runs"], "stops": stops, "accuracy": {}, "drift": {}, "excluded": []}
    res["accuracy"]["train_dense"] = {k: hardstops.accuracy(dense, f) for k, f in P.items()}
    vrows = {}
    for name in ("N02-V0", "N03-V1", "N04-V2", "N05-V3", "N06-V4", "N08-V5", "N09-V6"):
        if name in B:
            Rall = analysis.rows([B[name]])
            res["excluded"] += list(analysis.EXCLUDED)
            R = hardstops.good(Rall)
            blunders += [{"session": r["session"], "id": r["id"], "rms_px": r["rms_px"]} for r in Rall if r not in R]
            if name == "N09-V6":
                R = [{**r, "commanded": json.loads((B[name] / "configs" / r["id"] / "meta.json").read_text())
                      .get("q_wanted", r["commanded"])} for r in R]
            vrows[name] = R
            res["drift"][name] = {k: hardstops.accuracy(R, f) for k, f in P.items()} if R else None
            if name == "N09-V6" and R:
                res["drift"][name]["closed_loop_vs_wanted"] = hardstops.accuracy(R, lambda r, j: r["commanded"][j])
    if vrows.get("N05-V3"):
        # V3 was cut by the 10:30 wrap: before it under run A, after the recalibration under run B.
        for part, run in (("N05-V3-pre", spec["runs"]["A"]), ("N05-V3-post", spec["runs"]["B"])):
            R = [r for r in vrows["N05-V3"] if r["run"] == run]
            res["drift"][part] = {k: hardstops.accuracy(R, f) for k, f in P.items()} if R else None
    res["accuracy"]["heldout_V0"] = res["drift"].get("N02-V0")
    if "N10-extra" in B:
        Rall = analysis.rows([B["N10-extra"]])
        res["excluded"] += list(analysis.EXCLUDED)
        R = hardstops.good(Rall)
        blunders += [{"session": r["session"], "id": r["id"], "rms_px": r["rms_px"]} for r in Rall if r not in R]
        res["accuracy"]["heldout_extra"] = {k: hardstops.accuracy(R, f) for k, f in P.items()} if R else None
        res["accuracy"]["heldout_extra_n"] = len(R)
        xrows = R
    else:
        xrows = []
    if "N07-short" in B:
        short = hardstops.good(analysis.rows([B["N07-short"]]))
        if short and vrows.get("N08-V5"):
            s_stops = hardstops.fit(short, spec["runs"]["B"])
            s_lut = fit_rows(short) if len(short) >= 20 else None
            Ps = predictors(short, s_stops, None, s_lut or lut)
            res["short_vs_long_on_V5"] = {"long": res["drift"]["N08-V5"],
                                          "short": {k: hardstops.accuracy(vrows["N08-V5"], f) for k, f in Ps.items()},
                                          "short_stops": s_stops}
            if xrows:
                # N10-extra was taken under run B like the short block: the fair test of a LUT
                # (commanded -> true), which is tied to the calibration it was fitted under.
                res["short_on_extra"] = {k: hardstops.accuracy(xrows, f) for k, f in Ps.items()}
    res["vision_blunders"] = blunders
    res["couplings"] = analysis.dependencies(dense)
    res["hysteresis"] = analysis.repeatability(dense)
    res["visibility"] = analysis.vision(dense)
    if "N11-kapandji" in B:
        res["kapandji"] = kapandji(B["N11-kapandji"])
    res["sac"] = sac
    res["faults"] = faults(tag, B)
    if sac and vrows:
        res["sac_on_V"] = {n: {"b_new_stops": res["drift"][n]["b_new_stops"]["all"], "b_sac": res["drift"][n]["b_sac"]["all"]}
                           for n in vrows if res["drift"].get(n)}
    (OUT / "results.json").write_text(json.dumps(res, indent=1, default=float))

    with (OUT / "joint_series.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["session", "id", "kind", "joint", "run", "motor_rad", "commanded", "true", "old_linear", "new_linear", "lut"])
        for name, R in [("N01-dense", dense), *vrows.items(), ("N10-extra", xrows)]:
            for r in R:
                for j in JOINTS:
                    w.writerow([name, r["id"], r["kind"], j, r["run"], round(r["motor"][j], 5), round(r["commanded"][j], 2),
                                round(r["measured"][j], 2), round(P["a_old"](r, j), 2), round(P["b_new_stops"](r, j), 2),
                                round(P["c_lut"](r, j), 2)])
    with (OUT / "drift.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["block", "map", "joint", "rms", "max", "mean"])
        for n, d in res["drift"].items():
            for m, acc in (d or {}).items():
                for j, a in acc.items():
                    if a:
                        w.writerow([n, m, j, a["rms"], a["max"], a.get("mean")])
    if B.get("N12-sac") and (B["N12-sac"] / "contacts.jsonl").exists():
        with (OUT / "sac_onsets.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["finger", "variant", "contact", "fraction", "force_thumb", "force_finger"])
            for line in (B["N12-sac"] / "contacts.jsonl").read_text().splitlines():
                c = json.loads(line)
                o = c.get("onset") or {}
                w.writerow([c["finger"], c["variant"], c["contact"], o.get("fraction"),
                            o.get("forces", {}).get("thumb"), o.get("forces", {}).get(c["finger"])])
    photos(B, res)
    findings(res)
    return res


def photos(B: dict[str, Path], res: dict) -> None:
    import cv2
    (OUT / "kapandji").mkdir(exist_ok=True)
    for k in res.get("kapandji") or []:
        src = B["N11-kapandji"] / "configs" / k["id"] / "v180.jpg"
        if src.exists():
            im = cv2.imread(str(src))
            h, w = im.shape[:2]
            im = cv2.resize(im[h // 6: 5 * h // 6, w // 4: 3 * w // 4], None, fx=0.5, fy=0.5)
            cv2.imwrite(str(OUT / k["photo"]), im, [cv2.IMWRITE_JPEG_QUALITY, 88])
    import urllib.request
    for name in ("N01-dense", "N02-V0", "N09-V6", "N11-kapandji"):
        if name in B:
            try:
                urllib.request.urlretrieve(f"http://127.0.0.1:5010/contact/{B[name].name}.jpg", OUT / f"contact_{name}.jpg")
            except Exception as e:
                print(f"contact sheet {name}: needs the :5010 server", e)


def faults(tag: str, B: dict) -> dict:
    """Robustness data: every motor fault (time, motor, pose, temperature, load), and the poses
    skipped after a failed retry or by the back-off."""
    from collections import Counter
    path = STUDY / "night" / f"{tag}-faults.jsonl"
    events = [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []
    skipped = []
    for name, d in B.items():
        for f in sorted((d / "configs").glob("*/skipped.json")) if (d / "configs").is_dir() else []:
            skipped.append({"block": name, "id": f.parent.name, **json.loads(f.read_text())})
    return {"events": events, "per_joint": dict(Counter(e.get("joint") for e in events)),
            "skipped_poses": skipped}


def findings(res: dict) -> None:
    L = ["# Findings (generated numbers; interpretation in STUDY.md)", ""]
    tr = res["accuracy"]["train_dense"]
    L.append("Accuracy on the dense block (all joints, RMS / max deg): " +
             ", ".join(f"{k} {v['all']['rms']} / {v['all']['max']}" for k, v in tr.items() if v.get("all")))
    hx = res["accuracy"].get("heldout_extra")
    if hx:
        L.append(f"N10-extra held out ({res['accuracy']['heldout_extra_n']} poses): " +
                 ", ".join(f"{k} {v['all']['rms']} / {v['all']['max']}" for k, v in hx.items() if v and v.get("all")))
    sx = res.get("short_on_extra")
    if sx:
        L.append("N10-extra with the maps fitted on N07-short (same calibration run): " +
                 ", ".join(f"{k} {v['all']['rms']} / {v['all']['max']}" for k, v in sx.items() if v and v.get("all")))
    L.append("Robust (all joints, median / p95 |error| deg): " + "; ".join(
        f"{n}: " + ", ".join(f"{k} {v['all']['p50']}/{v['all']['p95']}" for k, v in d.items() if v and v.get("all"))
        for n, d in [("dense", tr), ("V0", res["drift"].get("N02-V0")), ("V5", res["drift"].get("N08-V5")),
                     ("extra", hx)] if d))
    for n, d in res["drift"].items():
        if d:
            L.append(f"{n}: " + ", ".join(f"{k} {v['all']['rms']}" for k, v in d.items() if v and v.get("all")))
    L.append("")
    L.append("Hard stops changed: " + ", ".join(f"{j} {s['old']}→{s['new']}" for j, s in res["stops"].items()
                                                if max(abs(a - b) for a, b in zip(s["old"], s["new"])) >= 1.0))
    if res.get("kapandji"):
        L.append("")
        L.append("| finger | map | distance mm | touch | force thumb | force finger |")
        L.append("|---|---|---|---|---|---|")
        for k in res["kapandji"]:
            L.append(f"| {k['finger']} | {k['map']} | {k['distance_mm']} | {'yes' if k['touch'] else 'no'} | "
                     f"{k['force_thumb']} | {k['force_finger']} |")
    fl = res.get("faults") or {}
    if fl.get("events") or fl.get("skipped_poses"):
        L.append("")
        L.append(f"Motor faults: {len(fl['events'])} ({fl['per_joint']}); poses skipped: {len(fl['skipped_poses'])}")
        for e in fl["events"]:
            L.append(f"- {e.get('joint')} id {e.get('id')} {e.get('flags')} at {e.get('where')}, "
                     f"{e.get('temp_c')} °C, {e.get('current_ma')} mA")
    (OUT / "findings.md").write_text("\n".join(L) + "\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("tag")
    p.add_argument("--loo", action="store_true")
    a = p.parse_args()
    r = main(a.tag, a.loo)
    print((OUT / "findings.md").read_text())
