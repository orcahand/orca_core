#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Phase-0.7 synthetic study: can the laptop-tier pipeline recover encoder
calibration to budget under injected model error?

Each scenario perturbs a different error class (see sim.Scenario); the
estimator only ever sees the nominal model, so recovered-parameter bias is
the study's measurement. Scenario A (no perturbations) must recover to well
under 0.1 deg — it is the machinery self-test; a study that cannot fail
cannot inform a decision, which is why B..F inject the error classes the
real rig will face.

Usage:
    uv run python tools/abs_calibration/study_synthetic.py [--quick] [--scenario NAME]
"""
from __future__ import annotations

import argparse
import os
import time

import numpy as np
import yaml

from model import JOINT_ROMS, KinematicModel, TIP_POINT_LOCAL, DISTAL_LINK, EncoderTruth  # noqa: F401
from sim import Scenario, generate, make_truth
from estimator import Estimator, PriorConfig, _rodrigues

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "results")

JOINT_CLASS = {j: ("wrist" if j == "wrist" else
                   "thumb" if j.startswith("thumb") else
                   j.split("_")[1]) for j in JOINT_ROMS}
# Per-joint budgets from the design doc §1 (deg).
JOINT_BUDGET_DEG = {
    "wrist": 0.16,
    "thumb_cmc": 0.26, "thumb_abd": 0.28, "thumb_mcp": 0.40, "thumb_dip": 0.83,
    **{f"{f}_abd": 0.27 for f in ("index", "middle", "ring", "pinky")},
    **{f"{f}_mcp": 0.34 for f in ("index", "middle", "ring", "pinky")},
    **{f"{f}_pip": 0.63 for f in ("index", "middle", "ring", "pinky")},
}


def scenarios() -> list[Scenario]:
    base = dict(axis_tilt_deg=0.5, origin_shift_m=3e-4, dir_bias_deg=0.3,
                tip_bias_m=3e-4, palm_bias_deg=0.2)
    meas = dict(scale=0.003, field_amp_m=2.5e-4)
    return [
        Scenario(name="A-ideal", inl_deg=0.0, gain_frac=0.0, seeds=3),
        Scenario(name="B-print-geometry", **base),
        Scenario(name="C-measurement-sys", **meas),
        Scenario(name="D-offsets-only", **base, **meas, offsets_only=True),
        Scenario(name="E-webcam-combined", **base, **meas),
        Scenario(name="F-gross-init", **base, **meas,
                 gross_init_err={"ring_abd": 20.0, "wrist": 12.0}),
    ]


def run_scenario(nominal: KinematicModel, sc: Scenario, quick: bool) -> dict:
    joints = nominal.joint_names
    seeds = 2 if quick else sc.seeds
    if quick:
        sc.sweep_steps = 18

    per_joint_mean = {j: [] for j in joints}
    per_joint_max = {j: [] for j in joints}
    tip_errs, axis_diags, times = [], {j: [] for j in joints}, []

    for seed in range(seeds):
        rng = np.random.default_rng(1000 + seed)
        truth = make_truth(nominal, sc, rng)
        ds = generate(nominal, sc, truth, rng)

        prior = PriorConfig(mean=dict(truth.hardstop_b0))
        denovo = list(sc.gross_init_err)
        for j in denovo:
            prior.mode[j] = "off" if j != "wrist" else "wide"

        est = Estimator(nominal, offsets_only=sc.offsets_only, prior=prior)
        x0 = est.initial_x(truth.hardstop_b0)
        if denovo:
            x0 = est.denovo_init(x0, ds, denovo)

        t0 = time.monotonic()
        result = est.solve(ds, x0)
        times.append(time.monotonic() - t0)
        _, _, b = est.unpack(result.x)

        # Encoder-mapping recovery over the ROM (model-independent metric).
        for i, j in enumerate(joints):
            q = np.linspace(*JOINT_ROMS[j], 200)
            m = truth.enc.measure(j, q)
            q_hat = est.apply_cal(np.tile(m[:, None], (1, len(joints))), b)[:, i]
            err = q_hat - q
            per_joint_mean[j].append(float(np.mean(np.abs(err))))
            per_joint_max[j].append(float(np.max(np.abs(err))))

        # Absolute fingertip error at random test poses (includes the
        # uncorrected geometry error, i.e. the honest v1 number).
        rvec, t, _ = est.unpack(result.x)
        Rb_hat = _rodrigues(rvec)
        q_test = np.stack([
            rng.uniform(lo + 3, hi - 3, size=60)
            for lo, hi in (JOINT_ROMS[j] for j in joints)
        ], axis=1)
        m_test = np.stack([truth.enc.measure(j, q_test[:, i], rng)
                           for i, j in enumerate(joints)], axis=1)
        q_hat = est.apply_cal(m_test, b)
        pred = nominal.link_poses({j: q_hat[:, i] for i, j in enumerate(joints)})
        true = truth.kin.link_poses({j: q_test[:, i] for i, j in enumerate(joints)})
        for finger, link in DISTAL_LINK.items():
            tip = TIP_POINT_LOCAL[finger]
            Rp, tp = pred[link]
            x_pred = np.einsum("ij,fj->fi", Rb_hat, np.einsum("fij,j->fi", Rp, tip) + tp) + t
            Rt, tt = true[link]
            x_true = np.einsum("ij,fj->fi", truth.base_R,
                               np.einsum("fij,j->fi", Rt, tip + truth.tip_bias[finger]) + tt) + truth.base_t
            tip_errs.append(np.linalg.norm(x_pred - x_true, axis=1))

        for j, ang in est.axis_diagnostic(result.x, ds).items():
            axis_diags[j].append(ang)

    tips = np.concatenate(tip_errs) * 1000.0
    return {
        "per_joint_mean_deg": {j: float(np.mean(v)) for j, v in per_joint_mean.items()},
        "per_joint_max_deg": {j: float(np.max(v)) for j, v in per_joint_max.items()},
        "fingertip_mm": {"mean": float(np.mean(tips)), "p95": float(np.percentile(tips, 95))},
        "axis_diag_deg": {j: float(np.median(v)) for j, v in axis_diags.items() if v},
        "solve_s": float(np.mean(times)),
        "seeds": seeds,
    }


def class_table(res: dict) -> list[dict]:
    rows = []
    for cls in ("wrist", "abd", "mcp", "pip", "thumb"):
        js = [j for j in JOINT_ROMS if JOINT_CLASS[j] == cls]
        mean = float(np.mean([res["per_joint_mean_deg"][j] for j in js]))
        worst = float(np.max([res["per_joint_max_deg"][j] for j in js]))
        # Worst joint relative to its own budget decides the verdict.
        ratio = float(np.max([res["per_joint_max_deg"][j] / JOINT_BUDGET_DEG[j] for j in js]))
        rows.append({
            "class": cls, "mean_deg": round(mean, 3), "worst_deg": round(worst, 3),
            "worst_vs_budget": round(ratio, 2),
            "within": bool(ratio <= 1.0),
        })
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--quick", action="store_true", help="2 seeds, short sweeps")
    ap.add_argument("--scenario", help="run only this scenario name")
    args = ap.parse_args()

    nominal = KinematicModel.load("right")
    all_results = {}
    for sc in scenarios():
        if args.scenario and sc.name != args.scenario:
            continue
        t0 = time.monotonic()
        res = run_scenario(nominal, sc, args.quick)
        res["classes"] = class_table(res)
        all_results[sc.name] = res
        print(f"\n=== {sc.name}  ({res['seeds']} seeds, "
              f"{time.monotonic() - t0:.0f}s, solve {res['solve_s']:.1f}s) ===")
        print(f"{'class':<8}{'mean deg':>10}{'worst deg':>11}{'x budget':>10}   ok")
        for row in res["classes"]:
            print(f"{row['class']:<8}{row['mean_deg']:>10.3f}{row['worst_deg']:>11.3f}"
                  f"{row['worst_vs_budget']:>10.2f}   {'PASS' if row['within'] else 'over'}")
        print(f"fingertip: mean {res['fingertip_mm']['mean']:.2f} mm, "
              f"p95 {res['fingertip_mm']['p95']:.2f} mm")

    os.makedirs(RESULTS_DIR, exist_ok=True)
    out = os.path.join(RESULTS_DIR, "results.yaml")
    with open(out, "w") as f:
        yaml.safe_dump(all_results, f, sort_keys=False)
    print(f"\nFull results written to {out}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
