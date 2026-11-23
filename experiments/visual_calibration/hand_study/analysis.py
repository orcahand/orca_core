"""The study's numbers: per-joint motor -> joint maps (measured vs. orca_core's linear map),
residuals, hard stops, cross-joint dependencies, repeatability and vision quality."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from hand_study.configs import JOINTS, ROM
from hand_study.mapping import LinearMap
from hand_study.paths import STUDY

FINGER = {j: j.split("_")[0] for j in JOINTS}
T_SIGNIFICANT = 3.0
MIN_ROWS = 8


def sessions() -> list[Path]:
    return sorted(p for p in (STUDY / "sessions").glob("*") if (p / "plan.json").exists()
                  and "baseline" not in p.name)


EXCLUDED: list[dict] = []
EST = "est.json"
EST_OLD = "est_mount0.json"     # the results with the baseline-only mount, kept for comparison


def rows(paths: list[Path] | None = None, est_name: str | None = None) -> list[dict]:
    """Every solved pose, except those whose masks are suspect (listed in EXCLUDED)."""
    out = []
    EXCLUDED.clear()
    for s in paths or sessions():
        plan = json.loads((s / "plan.json").read_text())
        for d in sorted((s / "configs").glob("*")):
            e = d / (est_name or EST)
            if not e.exists():
                continue
            meta = json.loads((d / "meta.json").read_text())
            est = json.loads(e.read_text())
            if est.get("suspect"):
                EXCLUDED.append({"session": s.name, "id": d.name, "why": est["suspect"]})
                continue
            b = meta["before"]
            motor = {j: b["motor_pos"][b["motor_joint"].index(j)] for j in JOINTS}
            current = {j: b["current_ma"][b["motor_joint"].index(j)] for j in JOINTS}
            split = est.get("split") or {}
            out.append({     # run per pose: V3 changed calibration mid-block
                "id": d.name, "session": s.name, "run": meta.get("calibration") or plan["calibration"], "kind": meta.get("kind"),
                "joint": meta.get("joint"), "dir": meta.get("dir"), "ref": meta.get("ref"), "side": meta.get("side"),
                "visit": meta.get("visit"), "t": meta.get("t0"),
                "commanded": meta["q"], "assumed": est["assumed"], "measured": est["joints"], "std": est["std"],
                "split": {j: abs(split["even"][j] - split["odd"][j]) for j in JOINTS} if split else {},
                "spread": est.get("spread", {}), "rms_px": est["rms_px"], "view_rms_px": est["view_rms_px"],
                "angles": est["angles"], "quality": est.get("quality", []), "motor": motor, "current": current,
                "via": meta.get("via"), "loaded": bool(meta.get("loaded")),
                "drift": max(abs(x - y) for x, y in zip(meta["after"]["motor_pos"], b["motor_pos"])),
            })
    return out


def joint_maps(R: list[dict]) -> dict:
    """Per joint and run: (motor, measured, assumed) points, the linear map's line, the measured
    stops at the commanded range ends, and a straight-line fit of measured vs. motor."""
    out = {}
    runs = sorted({r["run"] for r in R})
    for j in JOINTS:
        per = {}
        for run in runs:
            L = LinearMap(STUDY / f"calib_{run}")
            rr = [r for r in R if r["run"] == run]
            mid = L.motor[j]
            lo_m, hi_m = L.limits[mid]
            pts = [{"id": r["id"], "m": r["motor"][j], "meas": r["measured"][j], "assumed": r["assumed"][j],
                    "cmd": r["commanded"][j], "std": r["std"].get(j), "split": r["split"].get(j),
                    "kind": r["kind"], "dir": r["dir"], "cur": r["current"][j],
                    "swept": r["joint"] == j or (r["joint"] is None and abs(r["commanded"][j]) > 0.5)}
                   for r in rr]
            fit = None
            if len(pts) >= 3:
                m = np.array([L.unwrap(j, p["m"]) for p in pts])
                y = np.array([p["meas"] for p in pts])
                if np.ptp(m) > 1e-3:
                    A = np.c_[m, np.ones_like(m)]
                    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
                    slope_assumed = (-1 if L.inverted[j] else 1) / L.ratio[mid]
                    fit = {"slope": float(coef[0]), "intercept": float(coef[1]),
                           "gain": float(coef[0] / slope_assumed),
                           "rms": float(np.sqrt(np.mean((A @ coef - y) ** 2))),
                           "at_motor_limits": [float(coef[0] * lo_m + coef[1]), float(coef[0] * hi_m + coef[1])],
                           "offset_at_zero": float(coef[0] * L.to_motor(j, 0.0) + coef[1]),
                           "n": len(pts), "motor_span": float(np.ptp(m)), "coverage": float(np.ptp(m) / (hi_m - lo_m))}
            ends = {}
            for side, val in (("lo", ROM[j][0]), ("hi", ROM[j][1])):
                at = [p["meas"] for p in pts if p["swept"] and abs(p["cmd"] - val) < 0.5]
                ends[side] = {"configured": val, "measured": float(np.mean(at)) if at else None, "n": len(at)}
            per[run] = {"points": pts, "line": [[lo_m, L.to_joint(j, lo_m)], [hi_m, L.to_joint(j, hi_m)]],
                        "motor_limits": [lo_m, hi_m], "rom": list(ROM[j]), "fit": fit, "stops": ends}
        out[j] = per
    return out


def _poly(x: np.ndarray, deg: int = 2) -> np.ndarray:
    x = (x - x.mean()) / (x.std() + 1e-9)
    return np.vander(x, deg + 1)


def dependencies(R: list[dict]) -> dict:
    """Error of joint j (measured - assumed) against every other joint k's measured angle,
    each with the part explained by j's own angle removed (partial correlation), plus the
    sensitivity (deg of error per 10 deg of k) and a t value."""
    n = len(R)
    out = {"joints": JOINTS, "n": n, "r": [], "t": [], "slope10": [], "pairs": {}}
    if n < MIN_ROWS:
        return out
    E = np.array([[r["measured"][j] - r["assumed"][j] for j in JOINTS] for r in R])
    Q = np.array([[r["measured"][j] for j in JOINTS] for r in R])
    for a, j in enumerate(JOINTS):
        P = _poly(Q[:, a])
        e = E[:, a] - P @ np.linalg.lstsq(P, E[:, a], rcond=None)[0]
        rr, tt, ss = [], [], []
        for b, k in enumerate(JOINTS):
            if a == b or np.ptp(Q[:, b]) < 1.0:
                rr.append(None), tt.append(None), ss.append(None)
                continue
            q = Q[:, b] - P @ np.linalg.lstsq(P, Q[:, b], rcond=None)[0]
            den = np.sqrt((e @ e) * (q @ q))
            r = float(e @ q / den) if den > 0 else 0.0
            t = r * np.sqrt(max(n - 4, 1)) / np.sqrt(max(1 - r * r, 1e-9))
            rr.append(round(r, 3)), tt.append(round(float(t), 2)), ss.append(round(float(10 * (e @ q) / max(q @ q, 1e-9)), 3))
            if FINGER[j] == FINGER[k] or "wrist" in (j, k) or abs(t) >= T_SIGNIFICANT:
                out["pairs"][f"{j}|{k}"] = {"x": Q[:, b].round(2).tolist(), "y": E[:, a].round(2).tolist(),
                                           "y_partial": e.round(2).tolist(), "r": round(r, 3), "t": round(float(t), 2)}
        out["r"].append(rr), out["t"].append(tt), out["slope10"].append(ss)
    return out


def repeatability(R: list[dict]) -> dict:
    """Revisited configurations (repeats from below/above, sweep points up vs. down)."""
    reps = {}
    for r in R:
        if r["kind"] == "repeat":
            reps.setdefault(r["ref"], []).append(r)
    per_ref = {}
    for ref, rs in reps.items():
        per_joint = {}
        for j in JOINTS:
            vals = [x["measured"][j] for x in rs]
            motors = [x["motor"][j] for x in rs]
            below = [x["measured"][j] for x in rs if x["side"] == "below"]
            above = [x["measured"][j] for x in rs if x["side"] == "above"]
            per_joint[j] = {"sd": float(np.std(vals)) if len(vals) > 1 else None,
                            "hyst": float(np.mean(above) - np.mean(below)) if below and above else None,
                            "motor_sd_deg": float(np.std(motors) * 180 / np.pi) if len(motors) > 1 else None,
                            "points": [{"visit": x["visit"], "side": x["side"], "meas": x["measured"][j],
                                        "assumed": x["assumed"][j]} for x in rs]}
        per_ref[ref] = per_joint
    sweep = {}
    for j in JOINTS:
        up = {round(r["commanded"][j], 1): r for r in R if r["kind"] == "sweep" and r["joint"] == j and r["dir"] == "up"}
        down = {round(r["commanded"][j], 1): r for r in R if r["kind"] == "sweep" and r["joint"] == j and r["dir"] == "down"}
        both = sorted(set(up) & set(down))
        if both:
            sweep[j] = [{"cmd": c, "up": up[c]["measured"][j], "down": down[c]["measured"][j],
                         "up_assumed": up[c]["assumed"][j], "down_assumed": down[c]["assumed"][j]} for c in both]
    return {"refs": per_ref, "sweeps": sweep}


def vision(R: list[dict]) -> dict:
    out = {}
    for j in JOINTS:
        std = [r["std"][j] for r in R if r["std"].get(j, -1) >= 0]
        split = [r["split"][j] for r in R if j in r["split"]]
        out[j] = {"std_med": float(np.median(std)) if std else None,
                  "split_med": float(np.median(split)) if split else None,
                  "split_p90": float(np.percentile(split, 90)) if split else None,
                  "split": [round(s, 2) for s in split]}
    return {"joints": out, "rms": [r["rms_px"] for r in R],
            "view_rms": {str(a): float(np.median([v for r in R for a2, v in zip(r["angles"], r["view_rms_px"])
                                                    if abs(a2 - a) < 5])) for a in (0, 45, 90, 135, 180, 225, 270, 315)
                         if any(abs(a2 - a) < 5 for r in R for a2 in r["angles"])}}


def errors(R: list[dict]) -> dict:
    """Measured - assumed, and measured - commanded, per joint."""
    out = {}
    for j in JOINTS:
        e = np.array([r["measured"][j] - r["assumed"][j] for r in R])
        c = np.array([r["measured"][j] - r["commanded"][j] for r in R])
        out[j] = {"vs_assumed": {"mean": float(e.mean()), "rms": float(np.sqrt((e ** 2).mean())),
                                 "max": float(np.abs(e).max())} if len(e) else None,
                  "vs_commanded": {"mean": float(c.mean()), "rms": float(np.sqrt((c ** 2).mean())),
                                   "max": float(np.abs(c).max())} if len(c) else None}
    return out


def _mean_quality(R: list[dict]) -> dict:
    q = [v for r in R for v in r["quality"]]
    return {k: float(np.mean([v[k] for v in q])) if q else None for k in ("mask_only", "model_only", "finger_missed")}


def mount_compare(R: list[dict]) -> dict | None:
    """The pooled (bundle) mount against the baseline-only one: fit px, mask/model disagreement
    and per joint gain/offset, for the poses solved both ways."""
    old = rows(est_name=EST_OLD)
    excluded = list(EXCLUDED)
    rows()
    if not old:
        return None
    ids = {r["id"] for r in old} & {r["id"] for r in R}
    old = [r for r in old if r["id"] in ids]
    new = [r for r in R if r["id"] in ids]
    mo, mn = joint_maps(old), joint_maps(new)
    joints = {}
    for j in JOINTS:
        per = {}
        for run in mn[j]:
            a, b = (mo[j].get(run) or {}).get("fit"), mn[j][run]["fit"]
            if a and b:
                per[run] = {k: [round(a[k], 3), round(b[k], 3)] for k in ("gain", "offset_at_zero", "rms")}
        joints[j] = per
    shift = {j: float(np.median([abs(n["measured"][j] - o["measured"][j]) for o, n in
                                 zip(sorted(old, key=lambda r: r["id"]), sorted(new, key=lambda r: r["id"]))]))
             for j in JOINTS}
    p = STUDY / "mount_bundle.json"
    return {"n": len(ids), "bundle": json.loads(p.read_text()) if p.exists() else None,
            "rms_px": [float(np.mean([r["rms_px"] for r in old])), float(np.mean([r["rms_px"] for r in new]))],
            "quality": [_mean_quality(old), _mean_quality(new)], "excluded_old": excluded,
            "joints": joints, "median_change_deg": shift}


def report() -> dict:
    R = rows()
    return {"n": len(R), "excluded": list(EXCLUDED), "runs": sorted({r["run"] for r in R}), "joints": JOINTS, "rom": ROM,
            "maps": joint_maps(R), "deps": dependencies(R), "repeat": repeatability(R), "vision": vision(R),
            "errors": errors(R), "mount": mount_compare(R),
            "rows": [{k: r[k] for k in ("id", "session", "run", "kind", "joint", "dir", "commanded", "assumed",
                                        "measured", "std", "rms_px", "t", "current")} for r in R]}
