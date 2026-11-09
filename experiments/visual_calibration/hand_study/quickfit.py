"""The quick protocol's result (protocol.py --quick): own hard stops and the per-joint curve, fitted on
the training poses, scored on the held-out ones against orca_core's packaged map, then refitted on
every pose and stored for the hand -- the numbers a customer's calibration would carry:

    protocol/<tag>/quickfit.json  and  <bench>/visual_calibration.json   (+ the registry entry)

Stops: per joint true = g x + o, i.e. new stop angles [g lo + o, g hi + o]; orca_core recalibrates the
motor ratio from them, so they hold after an orca_core calibration. Curve: true = x + h(x) at 7 knots
over the packaged ROM, x = orca_core's estimate with the packaged stops (a fraction of the motor's way
between its calibrated limits, so it too holds after a recalibration).

    python -m hand_study.quickfit TAG
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

from hand_study import advanced as adv
from hand_study import curve, hardstops, offline
from hand_study.configs import JOINTS

NIGHT = offline.NIGHT


def _arrays(R, base):
    X = {j: np.array([base(r)[j] for r in R]) for j in JOINTS}
    Y = {j: np.array([r["measured"][j] for r in R]) for j in JOINTS}
    ok = {j: np.array([(r.get("split") or {}).get(j, 0.0) <= hardstops.SPLIT_MAX_DEG for r in R]) for j in JOINTS}
    return X, Y, ok


def fit(R, base):
    X, Y, ok = _arrays(R, base)
    st = adv.Stops().fit(X, Y, ok)
    cm = curve.Model(False)
    cm.fit(X, Y, ok)
    return st, cm


def score(R, pred, tips):
    return {"deg": offline.acc(R, pred), "mm": tips.errors(R, pred)}


def run(tag: str) -> dict:
    spec = json.loads((NIGHT / f"{tag}.json").read_text())
    hardstops.RMS_MAX_PX = float(spec.get("rms_max_px", hardstops.RMS_MAX_PX))
    pkg = offline.roms(spec)["packaged"]
    base = offline.linear(pkg)
    train = offline.rows_of(spec["sets"]["S1"])
    test = offline.rows_of(spec["sets"]["V1"])
    tips = offline.Tips()
    st, cm = fit(train, base)
    held = {"main stops": score(test, base, tips),
            "own stops": score(test, lambda r: st.forward(base(r)), tips),
            "curve": score(test, lambda r: cm.forward(base(r)), tips)} if test and train else {}
    allR = train + test
    st, cm = fit(allR, base)
    stops = {j: [round(float(st.w[j][0] * pkg[j][0] + st.w[j][1]), 2), round(float(st.w[j][0] * pkg[j][1] + st.w[j][1]), 2)]
             for j in JOINTS}
    knots = {}
    for j in JOINTS:
        xs = cm.own[j]
        ys = np.maximum.accumulate(cm.own_curve(j, xs))
        knots[j] = {"x": xs.round(2).tolist(), "true": ys.round(2).tolist(), "n": cm.n[j], "fit_rms_deg": round(cm.rms[j], 2)}
    out = {"tag": tag, "hand": spec["hand"], "model": Path(spec["bench"]).name, "skin": spec.get("skin"),
           "created": time.strftime("%Y-%m-%d %H:%M"), "calibration_run": spec["runs"]["p"],
           "n_train": len(train), "n_test": len(test), "heldout": held,
           "stops": {"joint_roms": stops, "packaged": {j: pkg[j] for j in JOINTS},
                     "use": "config.yaml joint_roms, then an orca_core calibration"},
           "curve": {"knots": knots, "base": "orca_core's estimate with the packaged stops (x)",
                     "use": "true = interp(x, knots.x, knots.true); monotone, invertible for commands"},
           "fitted_on": "every pose of the night (training + held out)",
           "note": f"{len(allR)} poses: a quick calibration; the curve is shrunk to the standard map where data is thin"}
    p = Path(spec["protocol_dir"]) / "quickfit.json"
    p.write_text(json.dumps(out, indent=1, default=float))
    (Path(spec["bench"]) / "visual_calibration.json").write_text(json.dumps(out, indent=1, default=float))
    from hand_study import registry
    registry.update(tag)
    return out


if __name__ == "__main__":
    o = run(sys.argv[1])
    for k, v in o["heldout"].items():
        print(f"{k:12s} joint p50 {v['deg']['all']['p50']:.2f}°  tip p50 {v['mm']['all']['p50']:.2f} mm")
