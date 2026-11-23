"""The night's report again with only the views that show the whole hand (est_full.json, from
`worker --out est_full.json` with estimate.FULL_ONLY), into report-full/, and a comparison with
the report from every view (report/results.json): views and poses dropped per block, the maps'
accuracy on V0, V5 and extra (overall and per joint), and how far the refitted hard stops move.

    python -m hand_study.fullview TAG
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from hand_study import analysis, report
from hand_study.configs import JOINTS

STUDY = analysis.STUDY
BLOCKS = ("N01-dense", "N02-V0", "N07-short", "N08-V5", "N10-extra")


def dropped(tag: str) -> dict:
    out = {}
    for name in BLOCKS:
        s = STUDY / "sessions" / f"{tag}-{name}"
        n = views = cut = lost = lost_before = refit = 0
        links: dict[str, int] = {}
        for d in sorted((s / "configs").glob("*")):
            if not (d / "est.json").exists():
                continue
            n += 1
            old = json.loads((d / "est.json").read_text())
            lost_before += bool(old.get("suspect"))
            if not (d / "est_full.json").exists():
                continue
            refit += 1
            e = json.loads((d / "est_full.json").read_text())
            views += e.get("n_views_all", len(e["angles"]))
            cut += len(e.get("cut_views", {}))
            for link, _ in e.get("cut_views", {}).values():
                links[link] = links.get(link, 0) + 1
            lost += bool(e.get("suspect"))
        out[name] = {"poses": n, "refit": refit, "views": views, "views_cut": cut,
                     "poses_suspect_before": lost_before, "poses_suspect_full": lost,
                     "cut_by_link": dict(sorted(links.items(), key=lambda t: -t[1]))}
    return out


def table(before: dict | None, after: dict | None) -> dict:
    """Per map: all-joint rms / p50 / n before and after, and per joint rms before -> after."""
    out = {}
    for k in (after or {}):
        b, a = (before or {}).get(k), after[k]
        if not a or not a.get("all"):
            continue
        out[k] = {"all": {"before": b and b["all"], "after": a["all"]},
                  "joints": {j: [b and b.get(j) and b[j]["rms"], a.get(j) and a[j]["rms"]] for j in JOINTS}}
    return out


def main(tag: str) -> dict:
    before = json.loads((STUDY / "report" / "results.json").read_text())
    analysis.EST = "est_full.json"
    report.OUT = STUDY / "report-full"
    after = report.main(tag) or json.loads((report.OUT / "results.json").read_text())
    cmp = {"dropped": dropped(tag),
           "V0": table(before["drift"].get("N02-V0"), after["drift"].get("N02-V0")),
           "V5": table(before["drift"].get("N08-V5"), after["drift"].get("N08-V5")),
           "extra": table(before["accuracy"].get("heldout_extra"), after["accuracy"].get("heldout_extra")),
           "train_dense": table(before["accuracy"].get("train_dense"), after["accuracy"].get("train_dense")),
           "short_on_V5": table(before.get("short_vs_long_on_V5", {}).get("short"),
                                after.get("short_vs_long_on_V5", {}).get("short")),
           "short_on_extra": table(before.get("short_on_extra"), after.get("short_on_extra")),
           "stops": {j: {"all_views": before["stops"][j]["new"], "full_views": after["stops"][j]["new"],
                         "moved": [round(a - b, 1) for a, b in zip(after["stops"][j]["new"], before["stops"][j]["new"])],
                         "n": [before["stops"][j]["n"], after["stops"][j]["n"]]} for j in JOINTS}}
    mv = np.abs([m for s in cmp["stops"].values() for m in s["moved"]])
    cmp["stops_moved"] = {"max": round(float(mv.max()), 1), "median": round(float(np.median(mv)), 1),
                          "over_2deg": int((mv > 2).sum())}
    (report.OUT / "compare.json").write_text(json.dumps(cmp, indent=1, default=float))
    return cmp


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("tag")
    a = p.parse_args()
    c = main(a.tag)
    print(json.dumps(c["dropped"], indent=1))
    for blk in ("V0", "V5", "extra", "short_on_V5", "short_on_extra"):
        for k, v in c[blk].items():
            print(f"{blk:15s} {k:12s} {v['all']['before'] and v['all']['before']['rms']} -> {v['all']['after']['rms']}  "
                  f"p50 {v['all']['before'] and v['all']['before']['p50']} -> {v['all']['after']['p50']}  n {v['all']['after']['n']}")
    print("stops moved", c["stops_moved"])
