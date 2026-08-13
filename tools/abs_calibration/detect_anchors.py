#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Detect the Phase-2 anchors on a captured session and write them into it.

build_session writes a dots-only session: curves and relative geometry, but
every absolute offset is still gauge. This tool adds the anchors that pin
them: a preliminary dots-only solve provides predicted poses (offsets held
by their priors — that is all a prediction needs), then the mesh-guided
contour detector (anchors.py) measures at every anchor frame

    dir_obs      link z directions of the phalanx links (mid-flexion poses)
    frame_axes   two-axis frame anchors: the forearm tower group (mandatory —
                 the only observation splitting the wrist offset from base
                 pose; static, so measured once over pooled views) and the
                 palm dorsal shell (per anchor frame)

and writes them back into the session (in place by default). Failed anchors
are NaN rows with reasons in the report — never silent predictions.

Assist shots (a hand-held camera orbiting the hand at each anchor pose) are
consumed when the capture lists them: each shot self-localises off the
ChArUco board with the assist camera's calibrated intrinsics.

Joints listed with ``mode: off`` in the priors file get the detection-side
de-novo pass first: a wide 1-D scan of their offset against edge support
(a grossly wrong offset — e.g. a misplaced encoder sensor — puts the
predicted contour outside the search window, or onto a neighbour's edges).

Usage:
    uv run python tools/abs_calibration/detect_anchors.py SESSION_DIR \
        --rig rig.yaml [--capture DIR] [--out DIR] [--side right]
        [--priors priors.yaml] [--assist-camera NAME]
"""
from __future__ import annotations

import argparse
import os
import sys
from dataclasses import replace

import cv2
import numpy as np
import yaml

from anchors import (AnchorDetectConfig, AnchorScene, ViewContext, ViewGeom,
                     denovo_scan, edge_support, make_board_clutter,
                     refine_group)
from board import board_pose
from cameras import Camera, Rig
from dataset import Dataset, load_session, save_session
from estimator import Estimator, PriorConfig
from model import KinematicModel
from sim import DIR_LINKS, FRAME_ANCHOR_LINKS, PALM_LINK, TOWER_LINK

TOWER_POOL_FRAMES = 3      # static tower: one measurement over pooled views
ASSIST_POSE_RMS_PX = 1.5   # board self-localisation quality gate per shot
ASSIST_MIN_CORNERS = 10

# The link observed for a joint during de-novo scanning (the joint's own
# child link where it carries surface, else the first distal link that does).
DENOVO_OBS_LINK = {"wrist": PALM_LINK, "thumb_cmc": "thumb_abd",
                   "thumb_abd": "thumb_mcp", "thumb_mcp": "thumb_dip",
                   "thumb_dip": "thumb_dip"}
for _f in ("index", "middle", "ring", "pinky"):
    DENOVO_OBS_LINK[_f + "_abd"] = _f + "_mcp"
    DENOVO_OBS_LINK[_f + "_mcp"] = _f + "_mcp"
    DENOVO_OBS_LINK[_f + "_pip"] = _f + "_pip"


def _frame_image(capture_dir: str, cam: str, f: int) -> np.ndarray | None:
    return cv2.imread(os.path.join(capture_dir, "frames", cam,
                                   f"frame_{f:05d}.png"), cv2.IMREAD_GRAYSCALE)


def _body_poses(world: dict, base: tuple, scene: AnchorScene,
                f: int) -> dict:
    """World pose of every mesh body at frame ``f`` from the estimator's
    posed link frames (the forearm structure sits at the model root)."""
    poses = {}
    for body in scene.bodies:
        if body == "world":
            poses[body] = base
        elif body in world:
            R, t = world[body]
            poses[body] = (R[f], t[f])
    return poses


def _assist_views(capture_dir: str, cap: dict, rig: Rig, assist_name: str,
                  frame: int, report: list) -> list[tuple[Camera, np.ndarray]]:
    """Self-localised assist shots of one anchor frame: each shot's world
    pose comes from the board in that very image."""
    views = []
    for shot in cap.get("assist", {}).get("shots", []):
        if shot["frame"] != frame:
            continue
        img = cv2.imread(os.path.join(capture_dir, shot["file"]),
                         cv2.IMREAD_GRAYSCALE)
        if img is None:
            report.append({"shot": shot["file"], "reason": "unreadable"})
            continue
        base_cam = rig.cameras[assist_name]
        got = board_pose(img, rig.board, base_cam.K, base_cam.dist,
                         min_corners=ASSIST_MIN_CORNERS)
        if got is None:
            report.append({"shot": shot["file"], "reason": "board not found"})
            continue
        R, t, n, rms = got
        if rms > ASSIST_POSE_RMS_PX:
            report.append({"shot": shot["file"],
                           "reason": f"pose rms {rms:.2f} px"})
            continue
        cam = Camera(name=f"{assist_name}@{shot['file']}",
                     image_size=base_cam.image_size, K=base_cam.K,
                     dist=base_cam.dist, R=R, t=t)
        views.append((cam, img))
    return views


def detect_session_anchors(ds: Dataset, capture_dir: str, cap: dict, rig: Rig,
                           side: str = "right",
                           prior: PriorConfig | None = None,
                           denovo: list[str] | None = None,
                           assist_camera: str | None = None,
                           cfg: AnchorDetectConfig | None = None,
                           scene: AnchorScene | None = None,
                           urdf_path: str | None = None):
    """Run the full Phase-2 detection: returns ``(dir_obs, frame_axes,
    report, x_prelim)``. ``ds`` must hold the dot observations; any anchors
    already present are ignored for the preliminary solve."""
    cfg = cfg or AnchorDetectConfig()
    prior = prior or PriorConfig()
    nominal = KinematicModel.load(side)
    est = Estimator(nominal, prior=prior)
    ds_dots = replace(ds, dir_obs={}, frame_axes={})

    print("Preliminary dots-only solve (offsets held by priors)...")
    x = est.initial_x(dict(prior.mean))
    x = est.solve(ds_dots, x, max_nfev=60).x

    if scene is None:
        kwargs = {"urdf_path": urdf_path} if urdf_path else {}
        from urdf_scene import MeshScene
        scene = AnchorScene(mesh=MeshScene(**kwargs),
                            skin_margin_m=cfg.skin_margin_m)

    fixed_cams = [c for c in cap["cameras"] if c in rig.cameras]
    anchor_frames = [int(i) for i in ds.dir_frames]
    A = len(anchor_frames)
    report = {"per_anchor": [], "assist_rejected": [], "denovo": {}}
    # The board's checker/marker gradients behind the hand are predictable
    # (its pose IS the world frame) — masked out of every edge search.
    fixed_clutter = {c: make_board_clutter(rig.board, rig.cameras[c])
                     for c in fixed_cams}

    def contexts_for(frame: int) -> list[ViewContext]:
        world = est.world_link_poses(x, ds_dots)
        poses = _body_poses(world, est.base_pose(x), scene, frame)
        out = []
        for cname in fixed_cams:
            img = _frame_image(capture_dir, cname, frame)
            if img is None:
                continue
            out.append(ViewContext(cam=rig.cameras[cname], image=img,
                                   poses=poses, clutter=fixed_clutter[cname]))
        if assist_camera:
            for cam, img in _assist_views(capture_dir, cap, rig,
                                          assist_camera, frame,
                                          report["assist_rejected"]):
                # Loose offset prior: hand-held self-localisation carries
                # the planar-target tilt ambiguity (mm-class at the hand).
                out.append(ViewContext(cam=cam, image=img, poses=poses,
                                       clutter=make_board_clutter(rig.board,
                                                                  cam),
                                       offset_prior_px=8.0))
        return out

    # De-novo wide scan BEFORE detection: a grossly wrong offset would make
    # every prediction for its chain miss (or steal a neighbour's edges).
    for j in denovo or []:
        link = DENOVO_OBS_LINK.get(j)
        if link is None:
            continue
        ji = est.joints.index(j)
        scan_frames = anchor_frames[:2]

        def support(off_deg: float) -> float:
            x_try = x.copy()
            x_try[6 + ji * est.K] += off_deg
            world = est.world_link_poses(x_try, ds_dots)
            base = est.base_pose(x_try)
            s = 0.0
            for f in scan_frames:
                poses = _body_poses(world, base, scene, f)
                ctxs = []
                for cname in fixed_cams:
                    img = _frame_image(capture_dir, cname, f)
                    if img is not None:
                        ctxs.append(ViewContext(cam=rig.cameras[cname],
                                                image=img, poses=poses))
                s += edge_support(scene, ctxs, [link], cfg)
            return s

        best, scores = denovo_scan(support, cfg.denovo_span_deg,
                                   cfg.denovo_step_deg)
        report["denovo"][j] = {"offset_deg": best,
                               "support": round(scores[best], 1)}
        x[6 + ji * est.K] += best
        print(f"  de-novo {j}: offset {best:+.0f} deg "
              f"(support {scores[best]:.0f})")

    dir_links = [l for l in DIR_LINKS if l in scene.bodies]
    dir_obs = {l: np.full((A, 3), np.nan) for l in dir_links}
    dir_sigma = {l: np.full(A, np.nan) for l in dir_links}
    frame_axes = {l: np.full((2, A, 3), np.nan) for l in FRAME_ANCHOR_LINKS}
    frame_sigma = {l: np.full((2, A), np.nan) for l in FRAME_ANCHOR_LINKS}

    for ai, f in enumerate(anchor_frames):
        contexts = contexts_for(f)
        if not contexts:
            report["per_anchor"].append({"frame": f, "reason": "no images"})
            continue
        geoms = [ViewGeom(scene, ctx, cfg) for ctx in contexts]
        row = {"frame": f, "n_views": len(contexts), "links": {}}
        for link in dir_links:
            res = refine_group(scene, contexts, [link], link, "dir",
                               cfg, geoms=geoms)
            if res.ok:
                dir_obs[link][ai] = res.R[:, 2]
                dir_sigma[link][ai] = res.sigma_tilt_deg
            row["links"][link] = {
                "ok": res.ok, "n": res.n_points,
                "rms_px": round(res.rms_px, 2) if np.isfinite(res.rms_px) else None,
                "sigma_deg": round(res.sigma_tilt_deg, 3)
                if np.isfinite(res.sigma_tilt_deg) else None,
                **({"reason": res.reason} if not res.ok else {}),
            }
        res = refine_group(scene, contexts, [PALM_LINK], PALM_LINK, "frame",
                           cfg, geoms=geoms)
        if res.ok:
            frame_axes[PALM_LINK][0, ai] = res.R[:, 2]
            frame_axes[PALM_LINK][1, ai] = res.R[:, 0]
            frame_sigma[PALM_LINK][:, ai] = res.sigma_tilt_deg
        row["links"][PALM_LINK] = {"ok": res.ok, "n": res.n_points,
                                   **({"reason": res.reason} if not res.ok else {})}
        report["per_anchor"].append(row)

    # Tower group: static in the rig — one measurement. Fixed cameras see
    # the identical tower every frame (pooling them again only re-counts the
    # same edges); assist shots differ per frame and pool for real.
    tower_group = [b for b in (TOWER_LINK, "world") if b in scene.bodies]
    pooled = []
    for i, f in enumerate(anchor_frames[:TOWER_POOL_FRAMES]):
        for ctx in contexts_for(f):
            if i == 0 or ctx.offset_prior_px > 2.0:
                pooled.append(ctx)
    if pooled and tower_group:
        res = refine_group(scene, pooled, tower_group, TOWER_LINK, "frame", cfg)
        if res.ok:
            frame_axes[TOWER_LINK][0, 0] = res.R[:, 2]
            frame_axes[TOWER_LINK][1, 0] = res.R[:, 0]
            frame_sigma[TOWER_LINK][:, 0] = res.sigma_tilt_deg
        report["tower"] = {"ok": res.ok, "n": res.n_points,
                           "n_views": len(pooled),
                           **({"reason": res.reason} if not res.ok else {})}
    else:
        report["tower"] = {"ok": False, "reason": "no views or no tower mesh"}

    return (dir_obs, dir_sigma), (frame_axes, frame_sigma), report, x


def summarize(report: dict, dir_obs: dict, frame_axes: dict) -> None:
    A = len(next(iter(dir_obs.values()))) if dir_obs else 0
    print(f"\n{'link':<14}{'anchors ok':>11}   worst reason")
    for link in sorted(dir_obs):
        ok = int(np.isfinite(dir_obs[link][:, 0]).sum())
        reasons = [r["links"].get(link, {}).get("reason", "")
                   for r in report["per_anchor"] if "links" in r]
        worst = next((s for s in reasons if s), "")
        print(f"{link:<14}{ok:>7}/{A:<3}   {worst}")
    for link in sorted(frame_axes):
        ok = int(np.isfinite(frame_axes[link][0, :, 0]).sum())
        print(f"{link:<14}{ok:>7}/{A:<3}   (frame anchor)")
    t = report.get("tower", {})
    if not t.get("ok"):
        print(f"WARNING: forearm tower anchor FAILED ({t.get('reason')}) — "
              "the wrist offset cannot be split from base pose without it.")
    if report["assist_rejected"]:
        print(f"assist shots rejected: {len(report['assist_rejected'])}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("session_dir")
    ap.add_argument("--rig", required=True)
    ap.add_argument("--capture", default=None,
                    help="Capture dir (default: the session's recorded one)")
    ap.add_argument("--out", default=None,
                    help="Output session dir (default: in place)")
    ap.add_argument("--side", default="right", choices=("right", "left"))
    ap.add_argument("--priors", default=None,
                    help="Per-joint prior YAML (solve_session format); "
                         "'off' joints get the de-novo scan")
    ap.add_argument("--assist-camera", default=None,
                    help="Rig camera holding the assist intrinsics")
    ap.add_argument("--urdf", default=None,
                    help="orcahand_description URDF (defaults per side)")
    args = ap.parse_args()

    ds = load_session(args.session_dir)
    capture_dir = args.capture or ds.meta.get("capture_dir")
    if not capture_dir or not os.path.isdir(capture_dir):
        sys.exit(f"capture dir not found: {capture_dir!r} (use --capture)")
    with open(os.path.join(capture_dir, "capture.yaml")) as f:
        cap = yaml.safe_load(f)
    rig = Rig.load(args.rig)

    prior = PriorConfig()
    denovo = []
    if args.priors:
        with open(args.priors) as f:
            raw = yaml.safe_load(f) or {}
        for joint, jcfg in raw.items():
            prior.mode[joint] = jcfg.get("mode", "normal")
            if "mean" in jcfg:
                prior.mean[joint] = float(jcfg["mean"])
            if prior.mode[joint] == "off":
                denovo.append(joint)

    if not len(ds.dir_frames):
        sys.exit("session has no anchor frames (dir_frames is empty)")

    (dir_obs, dir_sigma), (frame_axes, frame_sigma), report, _ = \
        detect_session_anchors(
            ds, capture_dir, cap, rig, side=args.side, prior=prior,
            denovo=denovo, assist_camera=args.assist_camera,
            urdf_path=args.urdf)
    summarize(report, dir_obs, frame_axes)

    out_dir = args.out or args.session_dir
    ds_out = replace(
        ds, dir_obs=dir_obs, frame_axes=frame_axes,
        dir_sigma=dir_sigma, frame_axes_sigma=frame_sigma,
        meta={**ds.meta, "anchor_detection": {
            "tower": report.get("tower"),
            "denovo": report.get("denovo", {}),
            "assist_rejected": len(report["assist_rejected"]),
        }},
    )
    save_session(ds_out, out_dir)
    print(f"wrote anchors into {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
