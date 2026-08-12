#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""End-to-end validation of the camera layer on rendered synthetic images.

Renders a synthetic scene through realistic lenses (render_synthetic), then
runs the *production* pipeline on the images — calibrate_rig on the ChArUco
views, build_session on the dot frames — and scores every stage against the
known truth:

    rig          recovered intrinsics/extrinsics vs the truth cameras
    dots         triangulated 3D tracks vs true dot positions; link assignment
    solve        estimator offset recovery on the built session, with
                 truth-derived direction/frame anchors standing in for the
                 Phase-2 anchor detection

Hand geometry stays nominal: this validates the camera layer, not model-error
bias (Phase 0.7 covered that). Gates are calibrated to the observed synthetic
performance with margin; a regression in any stage fails the run.

Usage:
    uv run python tools/abs_calibration/validate_camera_layer.py [--quick]
        [--seed N] [--workdir DIR] [--keep]
"""
from __future__ import annotations

import argparse
import glob
import os
import shutil
import sys
import tempfile
from dataclasses import replace

import numpy as np
import yaml

from assign import STATIC_LINK, link_signatures
from build_session import build_session
from calibrate_rig import calibrate_camera_dir
from cameras import Rig
from dataset import load_session, save_session
from dots import DetectorConfig
from estimator import Estimator, PriorConfig
from model import JOINT_ROMS, KinematicModel
from render_synthetic import SynthConfig, build_scene, quick_config, render_all
from sim import DIR_LINKS, FRAME_ANCHOR_LINKS

RESULTS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "results", "camera_validation.yaml")
ANCHOR_NOISE_DEG = 0.05     # truth-anchor noise, matching the study's dir class
MATCH_GATE_M = 0.002        # recovered column <-> truth dot pairing gate

# Gate philosophy: link-assignment integrity is the pipeline's core claim
# and stays a hard zero. The solve gates are the purpose-level sufficiency
# check (an oracle experiment showed ~0.5 coverage already supports a
# 0.11 deg mean solve; data-thin joints hover near 0.5 deg, runaway
# failures sat at 5-55 deg before the curve prior). Coverage is a
# regression tripwire, not a target — every deliberate safety trade
# (tight re-acquisition, association vetoes, rigidity drops, chain
# masking) spends coverage to protect identity. The extrinsic and
# triangulation gates are anchored to the PHYSICAL rig tier (28 cm board
# standoff -> cameras 0.55-0.65 m from the board): the farthest camera
# carries a ~0.25 deg / 3 mm systematic pose bias at that range which the
# end-to-end solve absorbs — the solve gates carry the requirement.
GATES = {
    "intrinsic_rms_px": 0.6,
    "focal_err_pct": 1.5,
    "extrinsic_rot_deg": 0.35,
    "extrinsic_t_mm": 4.5,
    "triang_rms_mm": 1.2,
    "triang_p95_mm": 1.5,
    "coverage": 0.45,
    "misassigned": 0,
    "unmatched_tracks": 3,
    "offset_mean_deg": 0.3,
    "offset_max_deg": 0.8,
}


def score_rig(rig: Rig, truth: Rig) -> dict:
    per_cam = {}
    for name, cam in rig.cameras.items():
        tc = truth.cameras[name]
        focal = 0.5 * (cam.K[0, 0] + cam.K[1, 1])
        focal_t = 0.5 * (tc.K[0, 0] + tc.K[1, 1])
        rot = np.degrees(np.arccos(np.clip((np.trace(tc.R.T @ cam.R) - 1) / 2, -1, 1)))
        per_cam[name] = {
            "intrinsic_rms_px": cam.meta["intrinsic_rms_px"],
            "focal_err_pct": float(abs(focal - focal_t) / focal_t * 100),
            "extrinsic_rot_deg": float(rot),
            "extrinsic_t_mm": float(np.linalg.norm(cam.center - tc.center) * 1000),
        }
    return per_cam


def score_dots(session_dir: str, scene) -> tuple[dict, "np.ndarray"]:
    """Match recovered dot columns to truth dots; error, coverage, assignment."""
    ds = load_session(session_dir)
    truth_pts = scene.dots_world                       # (F, D, 3)
    truth_links = ["fixed0" if l == "clutter" else l for l in scene.dot_links]

    chain_cols = {tuple(v) for v in ds.meta.get("chain_columns", [])}
    cols, col_links, col_chain = [], [], []
    for link in sorted(ds.dot_obs):
        arr = ds.dot_obs[link]
        for k in range(arr.shape[1]):
            cols.append(arr[:, k, :])
            col_links.append(link)
            col_chain.append((link, k) in chain_cols)

    errors, matched_of_truth = [], {}
    misassigned = []
    unmatched_moving = unmatched_static = proximal_parks = 0
    matched_pairs = 0
    sigs = link_signatures(scene.kin)
    for c, (col, link) in enumerate(zip(cols, col_links)):
        fin = np.isfinite(col).all(axis=1)
        d = np.linalg.norm(truth_pts - col[:, None, :], axis=2)   # (F, D)
        med = np.nanmedian(np.where(fin[:, None], d, np.nan), axis=0)
        if not np.isfinite(med).any() or med[int(np.nanargmin(med))] > MATCH_GATE_M:
            # Static columns matching no truth dot are board features and
            # mount texture — expected junk, harmlessly parked on fixed0.
            if link == STATIC_LINK:
                unmatched_static += 1
            else:
                unmatched_moving += 1
            continue
        best = int(np.nanargmin(med))
        matched_of_truth.setdefault(best, []).append(c)
        errors.append(d[fin, best])
        matched_pairs += int(fin.sum())
        # A proximal assignment is bounded by construction: the joints
        # separating it from the truth link demonstrably moved this track
        # less than the motion threshold (chain parks are masked outright;
        # near-axis dots wobble a millimetre). Distal or cross-chain
        # assignments have no such bound and are the real failures.
        if link != truth_links[best]:
            proximal = sigs[link] < sigs.get(truth_links[best], frozenset())
            if proximal and (link != STATIC_LINK or col_chain[c]):
                # fixed0 counts only as a flagged chain park (masked to its
                # provably-static frames); an unmasked static filing of a
                # moving dot stays an error.
                proximal_parks += 1
            else:
                misassigned.append((c, link, truth_links[best]))

    err = np.concatenate(errors) if errors else np.array([np.nan])
    denom = int((scene_n_views(scene) >= 2).sum())
    n_matched = sum(len(v) for v in matched_of_truth.values())
    stats = {
        "n_columns": len(cols),
        "n_matched": n_matched,
        "unmatched_tracks": unmatched_moving,
        "unmatched_static": unmatched_static,
        "proximal_parks": proximal_parks,
        "duplicates": sum(len(v) - 1 for v in matched_of_truth.values()),
        "triang_rms_mm": float(np.sqrt(np.nanmean(err ** 2)) * 1000),
        "triang_p95_mm": float(np.nanpercentile(err, 95) * 1000),
        "coverage": float(matched_pairs / max(denom, 1)),
        "misassigned": len(misassigned),
        "misassigned_detail": [
            {"column": c, "assigned": a, "truth": t} for c, a, t in misassigned],
    }
    return stats, ds


def scene_n_views(scene):
    if getattr(scene, "_n_views", None) is None:
        raise RuntimeError("scene missing view counts")
    return scene._n_views


def _noisy_dirs(dirs: np.ndarray, sigma_deg: float,
                rng: np.random.Generator) -> np.ndarray:
    out = np.empty_like(dirs)
    for i, d in enumerate(dirs):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        ang = np.deg2rad(rng.normal(0, sigma_deg))
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        R = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)
        out[i] = R @ d
    return out


def add_truth_anchors(ds, scene, rng: np.random.Generator):
    """Direction + frame anchors from truth poses at the anchor frames —
    the Phase-2 detection stand-in that makes absolute offsets observable."""
    frames = ds.dir_frames
    poses = scene.kin.link_poses(
        {j: scene.q[:, i] for i, j in enumerate(scene.joints)})

    def world_R(link):
        R, _ = poses[link]
        return np.einsum("ij,fjk->fik", scene.base_R, R)[frames]

    scene_links = set(scene.dot_links)
    dir_obs = {
        link: _noisy_dirs(world_R(link)[:, :, 2], ANCHOR_NOISE_DEG, rng)
        for link in DIR_LINKS if link in scene_links
    }
    frame_axes = {
        link: np.stack([
            _noisy_dirs(world_R(link)[:, :, 2], ANCHOR_NOISE_DEG, rng),
            _noisy_dirs(world_R(link)[:, :, 0], ANCHOR_NOISE_DEG, rng),
        ])
        for link in FRAME_ANCHOR_LINKS
    }
    return replace(ds, dir_obs=dir_obs, frame_axes=frame_axes)


def score_solve(ds, scene) -> dict:
    """Estimator recovery, scored on the swept joints only — a reduced smoke
    scene leaves the other joints anchored by nothing but their prior."""
    est = Estimator(scene.kin, prior=PriorConfig(mean=dict(scene.hardstop_b0)))
    x0 = est.initial_x(scene.hardstop_b0)
    result = est.solve(ds, x0)

    scored = set(scene.sweep_frames)
    per_joint = {}
    for i, j in enumerate(est.joints):
        if j not in scored:
            continue
        qg = np.linspace(*JOINT_ROMS[j], 100)
        m = scene.enc.measure(j, qg)
        q_hat = est.apply_cal(np.tile(m[:, None], (1, len(est.joints))),
                              result.b)[:, i]
        per_joint[j] = {
            "mapping_err_deg": float(np.mean(np.abs(q_hat - qg))),
            "b0_err_deg": float(abs(result.b[i, 0] + scene.enc.offset[j])),
            "sigma_deg": round(result.sigma_b0_deg[j], 4),
        }
    errs = [v["mapping_err_deg"] for v in per_joint.values()]
    return {
        "offset_mean_deg": float(np.mean(errs)),
        "offset_max_deg": float(np.max(errs)),
        "per_joint": per_joint,
        "solver_status": int(result.scipy_result.status),
    }


def run(cfg: SynthConfig, workdir: str, reuse: bool = False) -> dict:
    quick = cfg.sweep_joints is not None
    gates = dict(GATES)
    if quick:
        # Smoke run at coarser steps and lower resolution: relax the
        # continuous gates, keep the count gates (misassignment stays zero).
        for k, v in gates.items():
            if isinstance(v, float) and k != "coverage":
                gates[k] = v * 1.5
        gates["coverage"] = GATES["coverage"] * 0.85
    if reuse and os.path.exists(os.path.join(workdir, "truth", "truth.npz")):
        print("Reusing rendered scene (same seed/config).")
        scene = build_scene(cfg)
    else:
        scene = render_all(cfg, workdir)

    print("\n=== calibrate rig ===")
    cams = {}
    for d in sorted(glob.glob(os.path.join(workdir, "calib", "*"))):
        cam = calibrate_camera_dir(d, scene.board, min_corners=8)
        cams[cam.name] = cam
    rig = Rig(cameras=cams, board=scene.board)
    rig_path = os.path.join(workdir, "rig.yaml")
    rig.save(rig_path)
    rig_scores = score_rig(rig, scene.rig_truth)
    for name, s in rig_scores.items():
        print(f"  {name}: intr rms {s['intrinsic_rms_px']:.3f} px, "
              f"focal {s['focal_err_pct']:.2f}%, "
              f"extr {s['extrinsic_rot_deg']:.3f} deg / {s['extrinsic_t_mm']:.2f} mm")

    print("\n=== build session ===")
    session_dir = os.path.join(workdir, "session")
    build_session(os.path.join(workdir, "capture"), rig, session_dir,
                  detector=DetectorConfig(blob_color=255))

    # Truth view counts, needed for the coverage denominator.
    scene._n_views = np.load(os.path.join(workdir, "truth", "truth.npz"))["n_views"]

    print("\n=== score dots ===")
    dot_stats, ds = score_dots(session_dir, scene)
    print(f"  {dot_stats['n_matched']}/{dot_stats['n_columns']} columns matched, "
          f"rms {dot_stats['triang_rms_mm']:.3f} mm / "
          f"p95 {dot_stats['triang_p95_mm']:.3f} mm, "
          f"coverage {dot_stats['coverage']:.2f}, "
          f"misassigned {dot_stats['misassigned']}, "
          f"duplicates {dot_stats['duplicates']}")

    print("\n=== solve (with truth anchors) ===")
    rng = np.random.default_rng(cfg.seed + 2)
    ds_full = add_truth_anchors(ds, scene, rng)
    save_session(ds_full, os.path.join(workdir, "session_full"))
    solve_stats = score_solve(ds_full, scene)
    print(f"  mapping err mean {solve_stats['offset_mean_deg']:.3f} deg, "
          f"max {solve_stats['offset_max_deg']:.3f} deg")

    results = {
        "config": {"quick": quick, "seed": cfg.seed,
                   "n_cameras": cfg.n_cameras, "step_scale": cfg.step_scale,
                   "image_size": list(cfg.image_size)},
        "rig": rig_scores,
        "dots": {k: v for k, v in dot_stats.items() if k != "misassigned_detail"},
        "solve": {k: solve_stats[k] for k in
                  ("offset_mean_deg", "offset_max_deg", "solver_status")},
        "solve_per_joint": solve_stats["per_joint"],
    }

    failures = []
    for name, s in rig_scores.items():
        for key in ("intrinsic_rms_px", "focal_err_pct",
                    "extrinsic_rot_deg", "extrinsic_t_mm"):
            if s[key] > gates[key]:
                failures.append(f"{name}.{key}={s[key]:.3f} > {gates[key]}")
    for key in ("triang_rms_mm", "triang_p95_mm"):
        if dot_stats[key] > gates[key]:
            failures.append(f"{key}={dot_stats[key]:.3f} > {gates[key]}")
    if dot_stats["coverage"] < gates["coverage"]:
        failures.append(f"coverage={dot_stats['coverage']:.2f} < {gates['coverage']}")
    if dot_stats["misassigned"] > gates["misassigned"]:
        failures.append(f"misassigned={dot_stats['misassigned']}: "
                        f"{dot_stats['misassigned_detail']}")
    if dot_stats["unmatched_tracks"] > gates["unmatched_tracks"]:
        failures.append(f"unmatched_tracks={dot_stats['unmatched_tracks']}")
    if quick:
        # 1-2 surviving dots per link make the reduced scene's solve a noisy
        # statistic; the solve gates belong to the full-density run.
        print("  (quick: solve reported, not gated)")
    else:
        for key in ("offset_mean_deg", "offset_max_deg"):
            if solve_stats[key] > gates[key]:
                failures.append(f"{key}={solve_stats[key]:.3f} > {gates[key]}")

    results["gates"] = gates
    results["failures"] = failures
    results["pass"] = not failures
    return results


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workdir", default=None,
                    help="Where to render/process (default: temp dir)")
    ap.add_argument("--keep", action="store_true",
                    help="Keep the workdir even on success")
    ap.add_argument("--reuse", action="store_true",
                    help="Reuse an already-rendered workdir (same seed/config)")
    args = ap.parse_args()

    cfg = SynthConfig(seed=args.seed)
    if args.quick:
        cfg = quick_config(args.seed)

    workdir = args.workdir or tempfile.mkdtemp(prefix="orca_camval_")
    made_temp = args.workdir is None
    results = run(cfg, workdir, reuse=args.reuse)

    if not args.quick:
        os.makedirs(os.path.dirname(RESULTS_PATH), exist_ok=True)
        with open(RESULTS_PATH, "w") as f:
            yaml.safe_dump(results, f, sort_keys=False)
        print(f"\nwrote {RESULTS_PATH}")

    if results["pass"]:
        print("\nVALIDATION PASS")
        if made_temp and not args.keep:
            shutil.rmtree(workdir, ignore_errors=True)
        elif args.keep or not made_temp:
            print(f"workdir kept: {workdir}")
        return 0
    print("\nVALIDATION FAIL")
    for f in results["failures"]:
        print(f"  {f}")
    print(f"workdir kept for inspection: {workdir}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
