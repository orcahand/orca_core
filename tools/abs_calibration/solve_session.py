#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Solve a recorded calibration session: offsets + per-ROM curves + diagnostics.

Reads a session directory (see dataset.py for the format — written by the
synthetic simulation today, by the camera/detection layer at Phase 1) and
writes:

    <out>/vision_calibration.yaml       # per-joint offset/poly/sigma, source: vision
    <out>/calibration_diagnostics.yaml  # axis disagreements, residuals, solver info

The calibration file is estimator-native — wiring it into orca_core's
JointEncoderCal is the Phase-4 schema change and deliberately not done here.

Usage:
    uv run python tools/abs_calibration/solve_session.py SESSION_DIR \
        [--out DIR] [--offsets-only] [--priors priors.yaml]

priors.yaml (per-joint MAP lever):
    ring_abd: {mode: off}                 # de-novo init, no prior
    wrist:    {mode: wide, mean: -1.2}
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import yaml

from dataset import load_session
from estimator import Estimator, PriorConfig
from model import KinematicModel

SIGMA_GATE_DEG = 0.5


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("session_dir")
    ap.add_argument("--out", default=None, help="Output dir (default: session dir)")
    ap.add_argument("--offsets-only", action="store_true")
    ap.add_argument("--priors", default=None, help="Per-joint prior config YAML")
    ap.add_argument("--side", default="right", choices=("right", "left"))
    ap.add_argument("--manifolds", default=None,
                    help="Contact-manifold YAML for abd-block events "
                         "(build_contact_manifold.py)")
    args = ap.parse_args()

    ds = load_session(args.session_dir)
    out_dir = args.out or args.session_dir
    n_dots = sum(a.shape[1] for a in ds.dot_obs.values())
    print("Observation classes: "
          f"dots {n_dots} cols / {sum(int(np.isfinite(a[:, :, 0]).sum()) for a in ds.dot_obs.values())} pts, "
          f"dirs {len(ds.dir_obs)}, frame anchors {len(ds.frame_axes)}, "
          f"plate {len(ds.plate_obs)}, contacts {len(ds.contact_obs)}")

    manifolds = []
    if args.manifolds:
        with open(args.manifolds) as f:
            manifolds = yaml.safe_load(f) or []
    elif any(e.kind == "abd_block" for e in ds.contact_obs):
        print("NOTE: session has abd-block events but no --manifolds file; "
              "they will be skipped.")

    prior = PriorConfig()
    denovo = []
    if args.priors:
        with open(args.priors) as f:
            raw = yaml.safe_load(f) or {}
        for joint, cfg in raw.items():
            if joint not in ds.joints:
                sys.exit(f"priors: unknown joint {joint!r}")
            prior.mode[joint] = cfg.get("mode", "normal")
            if "mean" in cfg:
                prior.mean[joint] = float(cfg["mean"])
            if prior.mode[joint] == "off":
                denovo.append(joint)

    nominal = KinematicModel.load(args.side)
    est = Estimator(nominal, offsets_only=args.offsets_only, prior=prior,
                    manifolds=manifolds)

    x0 = est.initial_x(dict(prior.mean))
    if denovo:
        print(f"De-novo init for {denovo}...")
        est.prepare(ds)
        x0 = est.denovo_init(x0, ds, denovo)

    print(f"Solving {len(ds.joints)} joints over {len(ds.m)} frames...")
    result = est.solve(ds, x0)
    axis_diag = est.axis_diagnostic(result.x, ds)
    if est.skipped_contacts:
        from collections import Counter
        for (kind, a, bdy, why), n in Counter(est.skipped_contacts).items():
            print(f"WARNING: skipped {n} {kind} event(s) {a}/{bdy}: {why}")

    has_vision = bool(ds.dot_obs or ds.dir_obs or ds.frame_axes)
    has_contact = bool(ds.contact_obs)
    calibration = {
        "source": ("vision+contact" if has_vision and has_contact
                   else "contact" if has_contact else "vision"),
        "format": 1,
        "joints": {
            j: {
                "offset_deg": round(float(result.b[i, 0]), 4),
                "angle_correction_poly": [round(float(v), 5) for v in result.b[i, 1:]],
                "sigma_deg": round(result.sigma_b0_deg[j], 4),
            }
            for i, j in enumerate(ds.joints)
        },
    }
    diagnostics = {
        "axis_disagreement": {
            j: {"deg": round(v["deg"], 3), "floor_deg": round(v["floor_deg"], 3)}
            for j, v in axis_diag.items()
        },
        "solver": {
            "cost": result.cost,
            "status": int(result.scipy_result.status),
            "nfev": int(result.scipy_result.nfev),
            "offsets_only": bool(args.offsets_only),
        },
        "session_meta": dict(ds.meta),
    }

    os.makedirs(out_dir, exist_ok=True)
    for name, doc in (("vision_calibration.yaml", calibration),
                      ("calibration_diagnostics.yaml", diagnostics)):
        path = os.path.join(out_dir, name)
        with open(path, "w") as f:
            yaml.safe_dump(doc, f, sort_keys=False)
        print(f"wrote {path}")

    print(f"\n{'joint':<14}{'offset':>9}{'sigma':>8}   {'axis dev':>9}{'(floor)':>9}")
    gated = []
    for j in ds.joints:
        c = calibration["joints"][j]
        flag = ""
        if c["sigma_deg"] > SIGMA_GATE_DEG:
            gated.append(j)
            flag = "  SIGMA>GATE"
        ax = axis_diag.get(j, {"deg": float("nan"), "floor_deg": float("nan")})
        print(f"{j:<14}{c['offset_deg']:>9.3f}{c['sigma_deg']:>8.3f}   "
              f"{ax['deg']:>9.3f}{ax['floor_deg']:>9.3f}{flag}")
    if gated:
        print(f"\nWARNING: {len(gated)} joint(s) exceed the {SIGMA_GATE_DEG} deg "
              f"sigma gate: {gated} — do not persist without more data.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
