"""The registry of every hand the study has measured: which hand (serial, model, skin), when, on which
rig and with which protocol, where its data is, and its held-out accuracy per mapping, in names
that compare across hands. One row per study (a night); rebuilt from the study files, so it is
never edited by hand:

    <ORCA_HOME>/ui/visual_calibration/hand_study/registry.json (hand_study.paths)   (+ registry.csv, one line per study)

Updated by offline.py whenever a night's results change; `python -m hand_study.registry` rebuilds
it from everything on disk. The Study tab shows it (server.py: protocol status -> registry).
"""
from __future__ import annotations

import csv
import json
import time
from pathlib import Path

from hand_study.analysis import STUDY
from hand_study.colour import skin_tone

REG = STUDY / "registry.json"
NIGHT = STUDY / "night"
MAPS = ("main stops", "own stops", "curve", "coupled table", "advanced")
NAMES = {"packaged": "main stops", "own stops": "own stops", "per-joint curve": "curve",
         "advanced: + current/compliance (full)": "advanced", "LUT (coupled table)": "coupled table"}
LEGACY = {   # studies before the night protocol (rig before 9 Nov 01:37), from report-advanced/<hand>.json
    "ser-4485": {"tag": "20261108-night", "date": "2026-11-08", "model": "orcahand-touch-right",
                 "names": {"standard (config then)": "main stops", "hard stops (refit)": "own stops", "curve": "curve",
                           "coupled table": "coupled table", "advanced": "advanced"}},
    "ser-7577": {"tag": "20261108-s7577", "date": "2026-11-08", "model": "orcahand-right",
                 "names": {"packaged config": "main stops", "standard (own hard stops)": "own stops", "curve": "curve",
                           "coupled table": "coupled table", "advanced": "advanced"}},
}


def _clock(t: float | None) -> str | None:
    return time.strftime("%Y-%m-%d %H:%M", time.localtime(t)) if t else None


def night_entry(tag: str) -> dict | None:
    spec_p = NIGHT / f"{tag}.json"
    if not spec_p.exists():
        return None
    spec = json.loads(spec_p.read_text())
    if spec.get("mode") != "protocol" or tag.startswith("dry"):
        return None
    st_p, res_p = NIGHT / f"{tag}.state.json", STUDY / "protocol" / tag / "results.json"
    st = json.loads(st_p.read_text()) if st_p.exists() else {}
    res = json.loads(res_p.read_text()) if res_p.exists() else {}
    ends = [d.get("t1") for d in st.get("done", {}).values() if d.get("t1")]
    bench = Path(spec.get("bench", ""))
    h = res.get("headline") or {}
    maps = {}
    for k, m in (h.get("maps") or {}).items():
        name = NAMES.get(k)
        if name:
            maps[name] = {"joint_p50": m["deg"]["all"]["p50"], "joint_rms": m["deg"]["all"]["rms"],
                          "tip_p50_mm": m["mm"]["all"]["p50"], "tip_mean_mm": m["mm"]["all"].get("mean"),
                          "tip_p95_mm": m["mm"]["all"]["p95"]}
    qf = STUDY / "protocol" / tag / "quickfit.json"
    q = json.loads(qf.read_text()) if qf.exists() else None
    if q:
        for name, m in q["heldout"].items():
            maps[name] = {"joint_p50": m["deg"]["all"]["p50"], "joint_rms": m["deg"]["all"]["rms"],
                          "tip_p50_mm": (m["mm"].get("all") or {}).get("p50"), "tip_mean_mm": (m["mm"].get("all") or {}).get("mean"),
                          "tip_p95_mm": (m["mm"].get("all") or {}).get("p95")}
        h = {"set": f"{q['n_test']} held-out poses (V set subset)", "n": q["n_test"]}
    archive = bench.parent / f"study-{tag}"
    meth = h.get("methods") or {}
    return {"tag": tag, "hand": spec.get("hand"), "model": bench.name, "skin": spec.get("skin") or skin_tone(bench),
            "date": time.strftime("%Y-%m-%d", time.localtime(st["started"])) if st.get("started") else tag[:8],
            "started": _clock(st.get("started")), "ended": _clock(max(ends)) if st.get("state") == "done" and ends else None,
            "state": st.get("state"), "protocol": "quick (~20 min)" if spec.get("protocol") == "quick" else f"night protocol ({len(spec.get('phases', []))} phases)",
            "rig": meth.get("rig", "calibration_rig_330"), "segmenter": meth.get("segmenter"),
            "rms_cutoff_px": spec.get("rms_max_px", 9.5), "test_set": h.get("set"), "n_test": h.get("n"),
            "n_poses": sum((res.get("n") or {}).values()) or None, "maps": maps,
            "best": min(maps, key=lambda k: maps[k]["tip_p50_mm"] if maps[k]["tip_p50_mm"] is not None else maps[k]["joint_p50"]) if maps else None,
            "archive": str(archive) if archive.exists() else None, "results": str(res_p) if res_p.exists() else None,
            "calibration": {"file": str(bench / "visual_calibration.json"), "created": q["created"], "run": q["calibration_run"],
                            "joint_roms": q["stops"]["joint_roms"],
                            "curve": {j: {"x": k["x"], "true": k["true"]} for j, k in q["curve"]["knots"].items()}} if q else None}


def legacy_entry(hand: str) -> dict | None:
    L = LEGACY[hand]
    p = STUDY / "report-advanced" / f"{hand}.json"
    if not p.exists():
        return None
    d = json.loads(p.read_text())
    tests = d["tests"]
    maps = {}
    for k, name in L["names"].items():
        v = [t["maps"][k]["all"] for t in tests.values() if k in t["maps"]]
        if v:
            maps[name] = {"joint_p50": round(sum(x["p50"] for x in v) / len(v), 2),
                          "joint_rms": round(sum(x["rms"] for x in v) / len(v), 2),
                          "tip_p50_mm": None, "tip_mean_mm": None, "tip_p95_mm": None}
    return {"tag": L["tag"], "hand": hand, "model": L["model"], "skin": skin_tone(L["model"]), "date": L["date"],
            "started": None, "ended": None, "state": "done", "protocol": "earlier study (before the night protocol)",
            "rig": "camera rig before 9 Nov (XC430 turntable)", "segmenter": None, "rms_cutoff_px": 9.5,
            "test_set": " + ".join(tests) + " (mean of the sets)", "n_test": sum(t["n"] for t in tests.values()),
            "n_poses": d.get("n_train"), "maps": maps,
            "best": min(maps, key=lambda k: maps[k]["joint_p50"]) if maps else None,
            "archive": None, "results": str(p), "note": "joint angles only (no fingertip numbers in that study)"}


def rebuild() -> list[dict]:
    rows = [e for e in (legacy_entry(h) for h in LEGACY) if e]
    rows += [e for e in (night_entry(p.stem) for p in sorted(NIGHT.glob("*.json"))
                         if not p.stem.endswith(".state") and "." not in p.stem) if e]
    rows.sort(key=lambda r: (r["date"], r["tag"]))
    _write(rows)
    return rows


def update(tag: str) -> None:
    rows = json.loads(REG.read_text())["studies"] if REG.exists() else rebuild()
    e = night_entry(tag)
    if e:
        rows = [r for r in rows if r["tag"] != tag] + [e]
        rows.sort(key=lambda r: (r["date"], r["tag"]))
        _write(rows)


def _write(rows: list[dict]) -> None:
    tmp = REG.with_suffix(".tmp")
    tmp.write_text(json.dumps({"updated": time.strftime("%Y-%m-%d %H:%M:%S"), "maps": MAPS, "studies": rows}, indent=1))
    tmp.replace(REG)
    with REG.with_suffix(".csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["date", "tag", "hand", "model", "skin", "rig", "protocol", "state", "n_test", "best"] +
                   [f"{m} {k}" for m in MAPS for k in ("joint_p50", "tip_p50_mm")] + ["archive"])
        for r in rows:
            w.writerow([r["date"], r["tag"], r["hand"], r["model"], r["skin"], r["rig"], r["protocol"], r["state"],
                        r["n_test"], r["best"]] +
                       [(r["maps"].get(m) or {}).get(k) for m in MAPS for k in ("joint_p50", "tip_p50_mm")] + [r["archive"]])


def load() -> dict:
    return json.loads(REG.read_text()) if REG.exists() else {"studies": rebuild()}


if __name__ == "__main__":
    for r in rebuild():
        m = r["maps"]
        print(r["date"], r["tag"], r["hand"], r["skin"], {k: (v["joint_p50"], v["tip_p50_mm"]) for k, v in m.items()})
