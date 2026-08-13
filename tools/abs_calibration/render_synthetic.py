#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Render a synthetic capture from a known truth, through real lenses.

Unlike the Phase-0.7 study (3D-level observations), this renders actual
images — ChArUco calibration views and dot sweep frames — through cameras
with realistic focal lengths and lens distortion, so the whole camera layer
(board detection, intrinsic/extrinsic calibration, blob detection, tracking,
association, triangulation, link assignment) runs on pixels exactly as it
will on hardware. The hand geometry is left nominal: this validates the
camera pipeline, not the estimator (the study already covered model-error
bias).

Output layout (consumed by validate_camera_layer.py):

    OUT/calib/<cam>/{intrinsic,extrinsic}/*.png   # for calibrate_rig
    OUT/capture/...                               # for build_session
    OUT/truth/{rig_truth.yaml, truth.npz, truth.yaml}
"""
from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass, field

import cv2
import numpy as np
import yaml

from board import BoardSpec, detect_board
from cameras import Camera, Rig
from model import HELD_POSE, JOINT_ROMS, EncoderTruth, KinematicModel
from sim import DOT_LINKS
from sweep_plan import anchor_pose_plan, build_plan

CAPTURE_FORMAT = 1


@dataclass
class SynthConfig:
    n_cameras: int = 5
    camera_indices: tuple | None = None  # subset of the placement specs
    image_size: tuple[int, int] = (1280, 960)
    focal_px: float = 1050.0
    step_scale: float = 1.0         # multiplies the per-joint sweep step sizes
    sweep_joints: tuple | None = None   # None = all joints
    dot_links: tuple | None = None      # None = all standard dot links
    n_anchor: int = 10
    n_assist: int = 3               # hand-held assist shots per anchor frame
    # 7 per link, measured optimum at this working distance: fewer starves
    # the rigidity-voting quorum, more (10 tried) crowds the image and
    # feeds tracking confusion. The guide prescribes 6-8 per link.
    dots_per_link: int = 7
    # 6 mm dots: at the 55-60 cm working distance the board standoff forces,
    # 5 mm dots drop to ~4 px radius and detection SNR with them.
    dot_radius_m: float = 0.003
    min_dot_sep_m: float = 0.010    # physical dots don't touch; helps tracking
    n_clutter: int = 4              # static bright spots near the mount
    n_intrinsic: int = 14
    n_extrinsic: int = 3
    # Splat density: sparse clouds leave speckle holes inside the bodies —
    # pixel-noise gradients no real photograph has, which anchor edge search
    # would have to survive. Dense enough to render solid interiors.
    mesh_points_per_body: int = 16000
    grazing_cos: float = 0.05       # surfaces seen edge-on render no disk
    zbuf_tol_m: float = 0.004       # dot-vs-body depth test tolerance
    dot_dropout: float = 0.02
    abd_flexion_fracs: tuple | None = None  # hardware uses several; see sweep_plan
    # One-finger-at-a-time (park other fingers curled during each finger's
    # flexion sweeps). MEASURED NET-NEGATIVE under mesh occlusion: parked
    # fists block the under camera and no side camera sees through the
    # finger row regardless — the natural held spread already separates
    # the fingers (held min0=0 at >=3 cameras vs 3-4 dead joints cleared).
    # Kept as an option; off by default.
    clear_flexion: tuple | None = None
    # Capture held pose: the natural neutral opposes the thumb ACROSS the
    # fingers — thumb and finger dots pass within ~2 mm (below dot spacing)
    # and steal each other's identities. Parking the thumb clear of the
    # fingers is part of the capture protocol.
    held_override: tuple = (("thumb_abd", 0.0), ("thumb_mcp", -20.0),
                            ("thumb_cmc", -35.0))
    seed: int = 0

    @property
    def held(self) -> dict:
        return {**HELD_POSE, **dict(self.held_override)}


@dataclass
class SynthScene:
    cfg: SynthConfig
    board: BoardSpec
    rig_truth: Rig                   # truth cameras, world = board frame
    kin: KinematicModel
    enc: EncoderTruth
    base_R: np.ndarray
    base_t: np.ndarray
    q: np.ndarray                    # (F, J)
    joints: list[str]
    sweep_frames: dict[str, list[int]]
    anchor_frames: list[int]
    hardstop_b0: dict[str, float]
    mesh: object = field(init=False, default=None)  # urdf_scene.MeshScene
    dots_world: np.ndarray = field(init=False)    # (F, D, 3) all dots + clutter
    normals_world: np.ndarray = field(init=False)  # (F, D, 3), NaN = always visible
    dot_links: list[str] = field(init=False)       # per column; clutter -> "clutter"
    assist_cam: Camera = field(init=False, default=None)  # intrinsics only
    cam_target: np.ndarray = field(init=False, default=None)
    assist_shots: list = field(init=False, default_factory=list)


def _look_at(center: np.ndarray, target: np.ndarray,
             up: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """OpenCV world->cam pose for a camera at ``center`` looking at ``target``."""
    z = target - center
    z = z / np.linalg.norm(z)
    x = np.cross(z, up)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z])
    return R, -R @ center


def _pose_plan(joints: list[str], cfg: SynthConfig, rng: np.random.Generator):
    """The shared sweep protocol plus the shared anchor-pose plan (mid-flexion
    and extended/spread rows), with per-frame held-joint creep noise as on a
    tendon hand."""
    swept = list(cfg.sweep_joints) if cfg.sweep_joints else list(joints)
    plan_q, plan_sweeps = build_plan(
        swept, {j: JOINT_ROMS[j] for j in swept},
        {j: cfg.held[j] for j in swept},
        step_scale=cfg.step_scale, abd_flexion_fracs=cfg.abd_flexion_fracs,
        clear_flexion=dict(cfg.clear_flexion) if cfg.clear_flexion else None)

    jidx = {j: i for i, j in enumerate(joints)}
    held = np.array([cfg.held[j] for j in joints], float)
    q = np.tile(held, (len(plan_q), 1))
    for c, j in enumerate(swept):
        q[:, jidx[j]] = plan_q[:, c]
    q += rng.normal(0, 0.03, size=q.shape)

    anchor_rows, _ = anchor_pose_plan(
        joints, {j: JOINT_ROMS[j] for j in joints}, cfg.held,
        cfg.n_anchor, rng)
    anchor_frames = list(range(len(q), len(q) + cfg.n_anchor))
    return np.vstack([q, anchor_rows]), plan_sweeps, anchor_frames


def build_scene(cfg: SynthConfig, board: BoardSpec | None = None) -> SynthScene:
    """World = board frame. The board stands behind the hand facing the camera
    arc; its printed face looks along -z (OpenCV board convention: x right,
    y down, z into the board), so hand and cameras live at negative z with
    the hand's fingers pointing up (world -y)."""
    rng = np.random.default_rng(cfg.seed)
    board = board or BoardSpec()
    kin = KinematicModel.load("right")
    joints = kin.joint_names

    board_center = np.array([board.squares_x, board.squares_y, 0.0]) \
        * board.effective_square_m / 2.0
    # Board standoff: the wrist sweep swings the fingertips ~15 cm toward
    # the board — the hand mounts well in front so the sweep clears it.
    base_t = board_center + np.array([0.0, 0.055, -0.28]) + rng.normal(0, 0.005, 3)
    # Fingers up: base z -> world -y, plus a small random tilt.
    flip = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    ang = np.deg2rad(rng.normal(0, 5))
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    base_R = (np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)) @ flip

    q, sweep_frames, anchor_frames = _pose_plan(joints, cfg, rng)
    enc = EncoderTruth.sample(rng, joints)
    hardstop_b0 = {j: -enc.offset[j] + rng.normal(0, 1.0) for j in joints}

    scene = SynthScene(
        cfg=cfg, board=board, rig_truth=None, kin=kin, enc=enc,
        base_R=base_R, base_t=base_t, q=q, joints=joints,
        sweep_frames=sweep_frames, anchor_frames=anchor_frames,
        hardstop_b0=hardstop_b0,
    )
    from urdf_scene import MeshScene
    scene.mesh = MeshScene(n_per_body=cfg.mesh_points_per_body)
    _place_cameras(scene, rng)
    _place_dots(scene, rng)
    return scene


def _place_dots(scene: SynthScene, rng: np.random.Generator) -> None:
    """Dots on the ACTUAL mesh surfaces of the dot-bearing links (URDF
    visuals, silicone-skin zones excluded), plus static clutter; world
    positions and mesh normals for every frame.

    Placement mixes a camera-facing band with the phalanx sides (the
    design doc's "3-6 per phalanx side"). The sides are not cosmetic: a
    dot whose normal is parallel to a joint's axis keeps that normal
    fixed while the joint flexes, so axis-aligned side dots are the only
    ones guaranteed visible during their own link's sweep — which is when
    the calibration curve is measured. Distal links get mostly side dots;
    dots on the fully hidden face would never triangulate.
    """
    from scipy.spatial import cKDTree
    cfg = scene.cfg
    dot_links = list(cfg.dot_links) if cfg.dot_links else list(DOT_LINKS)
    cam_list = list(scene.rig_truth.cameras.values())
    cam_centers = [c.center for c in cam_list]
    cam_mid = np.mean(cam_centers, axis=0)
    held_poses = scene.kin.link_poses(
        {j: scene.cfg.held[j] for j in scene.joints})

    # Occlusion-aware candidate filter: a dot only counts as visible to a
    # camera if, at the held pose, it faces it AND no other body covers it
    # (z-buffer over all mesh bodies). This is the stick-where-you-can-see
    # rule a person applies — without it, "side" dots land in the 2-4 mm
    # gaps between fingers where neither stickers nor cameras reach.
    body_w, body_z = [], {}
    for link, (p, n) in scene.mesh.bodies.items():
        if link == "world":
            Rw, tw = scene.base_R, scene.base_t
        else:
            R0, t0 = held_poses[link]
            Rw = scene.base_R @ R0[0]
            tw = scene.base_R @ t0[0] + scene.base_t
        body_w.append(p @ Rw.T + tw)
    body_w = np.vstack(body_w)
    S = 2
    w_px, h_px = cfg.image_size
    for cam in cam_list:
        pc = body_w @ cam.R.T + cam.t
        front = pc[:, 2] > 0.12
        uv = cam.project(body_w[front])
        z = pc[front, 2]
        inb = ((uv[:, 0] >= 0) & (uv[:, 0] < w_px)
               & (uv[:, 1] >= 0) & (uv[:, 1] < h_px))
        uvi = np.round(uv[inb]).astype(int)
        zi = z[inb]
        order = np.argsort(-zi)
        zb = np.full((h_px // S + 1, w_px // S + 1), np.inf)
        zb[uvi[order, 1] // S, uvi[order, 0] // S] = zi[order]
        body_z[cam.name] = zb

    def cams_seeing(p_w, n_w):
        """(N,) count of cameras with a clear, facing view of each point."""
        count = np.zeros(len(p_w), int)
        for cam in cam_list:
            view = cam.center - p_w
            view = view / np.linalg.norm(view, axis=1, keepdims=True)
            facing = np.einsum("nk,nk->n", n_w, view) > 0.1
            pc = p_w @ cam.R.T + cam.t
            ok = facing & (pc[:, 2] > 0.12)
            if not ok.any():
                continue
            uv = cam.project(p_w[ok])
            inb = ((uv[:, 0] >= 0) & (uv[:, 0] < w_px)
                   & (uv[:, 1] >= 0) & (uv[:, 1] < h_px))
            zb = body_z[cam.name]
            uvi = np.round(uv[inb]).astype(int)
            clear = pc[ok][inb][:, 2] <= zb[uvi[:, 1] // S, uvi[:, 0] // S] \
                + cfg.zbuf_tol_m
            idx = np.flatnonzero(ok)[inb][clear]
            count[idx] += 1
        return count

    dot_local, normal_local, links = [], [], []
    for link in dot_links:
        R0, t0 = held_poses[link]
        Rw = scene.base_R @ R0[0]
        tw = scene.base_R @ t0[0] + scene.base_t
        surf, surf_n = scene.mesh.bodies[link]
        if link in scene.mesh.skin:
            d, _ = cKDTree(scene.mesh.skin[link]).query(surf, k=1)
            keep = d > 0.002
            surf, surf_n = surf[keep], surf_n[keep]

        n_w = surf_n @ Rw.T
        p_w = surf @ Rw.T + tw
        n_seen = cams_seeing(p_w, n_w)
        axis = scene.kin.nodes[scene.kin.index[link]].axis
        axis = axis / np.linalg.norm(axis)
        side_score = np.abs(surf_n @ axis)

        # Side band: normal parallel to the flexion axis (flexion-invariant
        # visibility), with a clear view from at least one camera at held.
        # Camera band: squarely facing the cluster with two clear views.
        side_pool = np.flatnonzero((side_score > 0.7) & (n_seen >= 1))
        cam_facing = np.einsum("nk,nk->n", n_w,
                               (cam_mid - p_w)
                               / np.linalg.norm(cam_mid - p_w, axis=1,
                                                keepdims=True))
        cam_pool = np.flatnonzero((cam_facing > 0.3) & (n_seen >= 2))
        side_frac = 0.7 if link.endswith(("_pip", "_dip")) else 0.45

        chosen: list[int] = []
        sep = cfg.min_dot_sep_m
        attempts = 0
        while len(chosen) < cfg.dots_per_link:
            attempts += 1
            if attempts % 300 == 0:
                sep *= 0.8  # small links can't always hold the full spacing
            pool = side_pool if (rng.random() < side_frac and len(side_pool)) \
                else cam_pool
            if not len(pool):
                pool = np.arange(len(surf))
            i = int(rng.choice(pool))
            if all(np.linalg.norm(surf[i] - surf[j]) >= sep for j in chosen):
                chosen.append(i)
        dot_local.append(surf[chosen])
        normal_local.append(surf_n[chosen])
        links.extend([link] * cfg.dots_per_link)

    F = len(scene.q)
    poses = scene.kin.link_poses({j: scene.q[:, i] for i, j in enumerate(scene.joints)})
    pts_all, nrm_all = [], []
    for link, pl, nl in zip(dot_links, dot_local, normal_local):
        R, t = poses[link]
        Rw = np.einsum("ij,fjk->fik", scene.base_R, R)
        tw = np.einsum("ij,fj->fi", scene.base_R, t) + scene.base_t
        pts_all.append(np.einsum("fij,kj->fki", Rw, pl) + tw[:, None, :])
        nrm_all.append(np.einsum("fij,kj->fki", Rw, nl))

    # Static bright spots ON the mount surfaces (screws, glints) — sampled
    # from the tower/forearm meshes so they sit on real geometry.
    mount_pts, mount_nrm = [], []
    for body in ("fixed0", "world"):
        p, n = scene.mesh.bodies[body]
        if body == "fixed0":
            R0, t0 = held_poses[body]
            Rw = scene.base_R @ R0[0]
            tw = scene.base_R @ t0[0] + scene.base_t
        else:
            Rw, tw = scene.base_R, scene.base_t
        mount_pts.append(p @ Rw.T + tw)
        mount_nrm.append(n @ Rw.T)
    mount_pts = np.vstack(mount_pts)
    mount_nrm = np.vstack(mount_nrm)
    facing = np.einsum("nk,nk->n", mount_nrm,
                       (cam_mid - mount_pts)
                       / np.linalg.norm(cam_mid - mount_pts, axis=1,
                                        keepdims=True))
    pool = np.flatnonzero(facing > 0.3)
    pick = rng.choice(pool, size=min(cfg.n_clutter, len(pool)), replace=False)
    clutter = mount_pts[pick]
    pts_all.append(np.broadcast_to(clutter, (F, len(pick), 3)))
    nrm_all.append(np.broadcast_to(mount_nrm[pick], (F, len(pick), 3)))
    links.extend(["clutter"] * len(pick))

    scene.dots_world = np.concatenate(pts_all, axis=1)
    scene.normals_world = np.concatenate(nrm_all, axis=1)
    scene.dot_links = links


def _place_cameras(scene: SynthScene, rng: np.random.Generator) -> None:
    """Cameras on an arc in front of the board, aimed between the hand and
    the board so extrinsic stills see the board near-frontally. One camera
    sits low (large polar angle): curled fingers hide their dots from the
    frontal views exactly during their own sweeps."""
    cfg = scene.cfg
    held_poses = scene.kin.link_poses({j: scene.cfg.held[j] for j in scene.joints})
    origins = np.stack([t[0] for _, t in held_poses.values()])
    hand_center = scene.base_R @ origins.mean(axis=0) + scene.base_t
    board_center = np.array([scene.board.squares_x, scene.board.squares_y, 0.0]) \
        * scene.board.effective_square_m / 2.0
    target = 0.55 * hand_center + 0.45 * board_center
    w, h = cfg.image_size

    cams = {}
    # Probe-optimised under true mesh self-occlusion (mean own-sweep dot
    # coverage 0.51 vs <=0.34 for every 4-camera set tried): left,
    # below-front, right, high-front, and — decisive for flexed
    # fingertips — one camera nearly UNDER the hand looking up. A curling
    # finger folds its dots toward the palm plane; only the under camera
    # keeps them through deep flexion.
    # 0 left, 1 below-front, 2 right, 3 high-front, 4 under, 5 side-profile
    # (pinky side, sees every one-at-a-time curl edge-on), 6 side-thumb.
    specs = ((-165, 42, 0.50), (70, 55, 0.46), (-20, 50, 0.50),
             (-85, 20, 0.50), (60, 78, 0.42), (0, 72, 0.46),
             (180, 72, 0.46))
    indices = cfg.camera_indices or tuple(range(cfg.n_cameras))
    for i, spec_i in enumerate(indices):
        az, pol, r0 = specs[spec_i % len(specs)]
        th = np.deg2rad(pol + rng.normal(0, 2))
        ph = np.deg2rad(az + rng.normal(0, 3))
        r = r0 + rng.normal(0, 0.015)
        center = target + r * np.array([
            np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), -np.cos(th)])
        R, t = _look_at(center, target, up=np.array([0.0, -1.0, 0.0]))
        f = cfg.focal_px * rng.uniform(0.96, 1.04)
        K = np.array([[f, 0, w / 2 + rng.normal(0, 5)],
                      [0, f * rng.uniform(0.999, 1.001), h / 2 + rng.normal(0, 5)],
                      [0, 0, 1.0]])
        dist = np.array([rng.uniform(-0.13, -0.06), rng.uniform(0.01, 0.05),
                         rng.normal(0, 8e-4), rng.normal(0, 8e-4), 0.0])
        cams[f"cam{i}"] = Camera(
            name=f"cam{i}", image_size=cfg.image_size, K=K, dist=dist, R=R, t=t)
    scene.rig_truth = Rig(cameras=cams, board=scene.board)
    scene.cam_target = target

    # The hand-held assist camera: ONE set of intrinsics (one physical
    # camera), a fresh pose per shot (sampled at render time).
    f = cfg.focal_px * rng.uniform(0.96, 1.04)
    K = np.array([[f, 0, w / 2 + rng.normal(0, 5)],
                  [0, f * rng.uniform(0.999, 1.001), h / 2 + rng.normal(0, 5)],
                  [0, 0, 1.0]])
    dist = np.array([rng.uniform(-0.13, -0.06), rng.uniform(0.01, 0.05),
                     rng.normal(0, 8e-4), rng.normal(0, 8e-4), 0.0])
    scene.assist_cam = Camera(name="assist0", image_size=cfg.image_size,
                              K=K, dist=dist, R=np.eye(3), t=np.zeros(3))


def _assist_pose(scene: SynthScene, rng: np.random.Generator):
    """One hand-held shot pose: a wider arc than the fixed cameras (including
    beyond it and lower), aimed so hand and board share the frame."""
    az = np.deg2rad(rng.uniform(-210, 110))
    pol = np.deg2rad(rng.uniform(25, 82))
    r = rng.uniform(0.40, 0.55)
    center = scene.cam_target + r * np.array([
        np.sin(pol) * np.cos(az), np.sin(pol) * np.sin(az), -np.cos(pol)])
    aim = scene.cam_target + rng.normal(0, 0.02, 3)
    return _look_at(center, aim, up=np.array([0.0, -1.0, 0.0]))


# ---- image formation ---------------------------------------------------------

def _distortion_remap(cam: Camera) -> tuple[np.ndarray, np.ndarray]:
    """Remap arrays turning an ideal (undistorted) render into a distorted one:
    for each distorted output pixel, the undistorted source pixel."""
    w, h = cam.image_size
    xs, ys = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    px = np.stack([xs.ravel(), ys.ravel()], axis=1).astype(np.float64)
    und = cv2.undistortPoints(px.reshape(-1, 1, 2), cam.K, cam.dist, P=cam.K)
    und = und.reshape(h, w, 2).astype(np.float32)
    return und[..., 0], und[..., 1]


def _board_texture(spec: BoardSpec, px_per_square: int = 120,
                   margin_squares: float = 0.5):
    """Board image plus the homography from metric board coords to texture px."""
    mg = int(px_per_square * margin_squares)
    size = (spec.squares_x * px_per_square + 2 * mg,
            spec.squares_y * px_per_square + 2 * mg)
    tex = spec.make().generateImage(size, marginSize=mg)
    det = detect_board(tex, spec, min_corners=8)
    if det is None:
        raise RuntimeError("board texture failed self-detection")
    obj, img = det
    H_tex, _ = cv2.findHomography(obj[:, :2], img)
    return tex, H_tex


def _warp_board(cam: Camera, R_bc: np.ndarray, t_bc: np.ndarray,
                tex: np.ndarray, H_tex: np.ndarray,
                canvas: np.ndarray) -> np.ndarray:
    """Blend the board texture at camera-frame pose (R_bc, t_bc) into an
    undistorted-view canvas."""
    H_pose = cam.K @ np.column_stack([R_bc[:, 0], R_bc[:, 1], t_bc])
    H = H_pose @ np.linalg.inv(H_tex)
    h, w = canvas.shape
    warped = cv2.warpPerspective(tex, H, (w, h), flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    mask = cv2.warpPerspective(np.full(tex.shape, 255, np.uint8), H, (w, h),
                               flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    alpha = mask.astype(np.float32) / 255.0
    return (canvas * (1 - alpha) + warped * alpha).astype(np.uint8)


def render_board_image(cam: Camera, R_bc: np.ndarray, t_bc: np.ndarray,
                       tex: np.ndarray, H_tex: np.ndarray,
                       remap_xy: tuple[np.ndarray, np.ndarray],
                       rng: np.random.Generator, bg: int = 135) -> np.ndarray:
    """Board at pose (R_bc, t_bc) in the camera frame, through the real lens."""
    canvas = _warp_board(cam, R_bc, t_bc, tex, H_tex,
                         np.full(cam.image_size[::-1], bg, np.uint8))
    canvas = cv2.remap(canvas, remap_xy[0], remap_xy[1], cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_CONSTANT, borderValue=bg)
    canvas = cv2.GaussianBlur(canvas, (0, 0), 0.6)
    noisy = canvas.astype(np.float32) + rng.normal(0, 2.0, canvas.shape)
    return np.clip(noisy, 0, 255).astype(np.uint8)


def _sample_intrinsic_pose(cam: Camera, spec: BoardSpec, rng: np.random.Generator):
    """Random board pose in the camera frame: varied tilt, roll, and position."""
    w, h = cam.image_size
    z = rng.uniform(0.32, 0.55)
    u = rng.uniform(0.2, 0.8) * w
    v = rng.uniform(0.2, 0.8) * h
    center = z * np.array([(u - cam.K[0, 2]) / cam.K[0, 0],
                           (v - cam.K[1, 2]) / cam.K[1, 1], 1.0])
    rx, ry = np.deg2rad(rng.uniform(-32, 32, size=2))
    rz = np.deg2rad(rng.uniform(0, 360))
    Rx = cv2.Rodrigues(np.array([rx, 0, 0]))[0]
    Ry = cv2.Rodrigues(np.array([0, ry, 0]))[0]
    Rz = cv2.Rodrigues(np.array([0, 0, rz]))[0]
    R = Rx @ Ry @ Rz
    board_center = np.array([spec.squares_x, spec.squares_y, 0.0]) \
        * spec.effective_square_m / 2.0
    t = center - R @ board_center
    return R, t


def render_calibration(scene: SynthScene, calib_dir: str,
                       rng: np.random.Generator) -> None:
    tex, H_tex = _board_texture(scene.board)
    cams = list(scene.rig_truth.cameras.values())
    if scene.cfg.n_assist > 0:
        cams.append(scene.assist_cam)     # intrinsics only: no extrinsic dir
    for cam in cams:
        is_assist = cam is scene.assist_cam
        remap_xy = _distortion_remap(cam)
        d_int = os.path.join(calib_dir, cam.name, "intrinsic")
        os.makedirs(d_int, exist_ok=True)
        for i in range(scene.cfg.n_intrinsic):
            R, t = _sample_intrinsic_pose(cam, scene.board, rng)
            img = render_board_image(cam, R, t, tex, H_tex, remap_xy, rng)
            cv2.imwrite(os.path.join(d_int, f"view_{i:03d}.png"), img)
        if is_assist:
            print(f"  {cam.name}: {scene.cfg.n_intrinsic} intrinsic images "
                  "(hand-held: no extrinsics)")
            continue
        d_ext = os.path.join(calib_dir, cam.name, "extrinsic")
        os.makedirs(d_ext, exist_ok=True)
        for i in range(scene.cfg.n_extrinsic):
            # Board fixed at the world origin: its camera-frame pose IS the
            # camera's world pose.
            img = render_board_image(cam, cam.R, cam.t, tex, H_tex, remap_xy, rng)
            cv2.imwrite(os.path.join(d_ext, f"still_{i:03d}.png"), img)
        print(f"  {cam.name}: {scene.cfg.n_intrinsic} intrinsic + "
              f"{scene.cfg.n_extrinsic} extrinsic images")


class SceneRenderer:
    """Renders session frames with the real URDF meshes: bodies are drawn by
    z-buffered point splatting (Lambert-shaded dark print), the board layer
    sits behind, and dot visibility comes from the z-buffer — genuine
    self-occlusion, finger-over-finger included, replaces the old
    normal-only heuristic."""

    ZBUF_SCALE = 2  # occlusion buffer at half resolution

    def __init__(self, scene: SynthScene):
        self.scene = scene
        cfg = scene.cfg
        self.w, self.h = cfg.image_size
        self._tex, self._H_tex = _board_texture(scene.board)
        self._assist_remap = None
        self.cam_assets = {}
        for cam in scene.rig_truth.cameras.values():
            remap_xy = _distortion_remap(cam)
            board_ud = _warp_board(cam, cam.R, cam.t, self._tex, self._H_tex,
                                   np.full((self.h, self.w), 22, np.uint8))
            layer = cv2.remap(board_ud, remap_xy[0], remap_xy[1],
                              cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                              borderValue=22)
            self.cam_assets[cam.name] = (cam, layer)
        poses = scene.kin.link_poses(
            {j: scene.q[:, i] for i, j in enumerate(scene.joints)})
        self.world_Rt = {}
        for link, (R, t) in poses.items():
            Rw = np.einsum("ij,fjk->fik", scene.base_R, R)
            tw = np.einsum("ij,fj->fi", scene.base_R, t) + scene.base_t
            self.world_Rt[link] = (Rw, tw)
        F = len(scene.q)
        self.world_Rt["world"] = (
            np.broadcast_to(scene.base_R, (F, 3, 3)),
            np.broadcast_to(scene.base_t, (F, 3)))

    def body_points(self, f: int) -> tuple[np.ndarray, np.ndarray]:
        """All body-surface points + normals in scene world at frame ``f``."""
        pts, nrm = [], []
        for link, (p, n) in self.scene.mesh.bodies.items():
            R, t = self.world_Rt[link]
            pts.append(p @ R[f].T + t[f])
            nrm.append(n @ R[f].T)
        return np.vstack(pts), np.vstack(nrm)

    def render(self, cam_name: str, f: int, body_pts, body_nrm,
               rng: np.random.Generator):
        """One frame for one camera: ``(image_uint8, visible_dot_mask)``."""
        cam, board_layer = self.cam_assets[cam_name]
        return self._render_on(cam, board_layer, f, body_pts, body_nrm, rng)

    def render_free(self, cam: Camera, f: int, body_pts, body_nrm,
                    rng: np.random.Generator):
        """One frame through a camera at an arbitrary pose (assist shots).
        The distortion remap is cached per intrinsics — every assist shot
        shares the one physical camera."""
        if self._assist_remap is None:
            self._assist_remap = _distortion_remap(cam)
        board_ud = _warp_board(cam, cam.R, cam.t, self._tex, self._H_tex,
                               np.full((self.h, self.w), 22, np.uint8))
        layer = cv2.remap(board_ud, self._assist_remap[0],
                          self._assist_remap[1], cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=22)
        return self._render_on(cam, layer, f, body_pts, body_nrm, rng)

    def _render_on(self, cam: Camera, board_layer: np.ndarray, f: int,
                   body_pts, body_nrm, rng: np.random.Generator):
        scene, cfg = self.scene, self.scene.cfg
        w, h, S = self.w, self.h, self.ZBUF_SCALE
        canvas = board_layer.copy()

        pc = body_pts @ cam.R.T + cam.t
        front = pc[:, 2] > 0.12
        uv = cam.project(body_pts[front])
        z = pc[front, 2]
        inb = ((uv[:, 0] >= 1) & (uv[:, 0] < w - 1)
               & (uv[:, 1] >= 1) & (uv[:, 1] < h - 1))
        uvi = np.round(uv[inb]).astype(int)
        z_in = z[inb]
        view = cam.center - body_pts[front][inb]
        view /= np.linalg.norm(view, axis=1, keepdims=True)
        lam = np.clip(np.einsum("ij,ij->i", body_nrm[front][inb], view), 0, 1)
        shade = (42 + 55 * lam).astype(np.uint8)

        order = np.argsort(-z_in)          # far to near; near overwrites
        ux, vy = uvi[order, 0], uvi[order, 1]
        sh = shade[order]
        zbuf = np.full((h // S + 1, w // S + 1), np.inf)
        zbuf[vy // S, ux // S] = z_in[order]
        # Cull bin-occluded points before drawing: sampled meshes include
        # internal geometry, and without this it bleeds through sampling
        # holes in the outer shell as bright speckle no real surface has.
        keep = z_in[order] <= zbuf[vy // S, ux // S] + cfg.zbuf_tol_m
        ux, vy, sh = ux[keep], vy[keep], sh[keep]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                canvas[np.clip(vy + dy, 0, h - 1), np.clip(ux + dx, 0, w - 1)] = sh

        pts = scene.dots_world[f]
        nrm = scene.normals_world[f]
        D = len(pts)
        pcd = pts @ cam.R.T + cam.t
        viewd = cam.center - pts
        viewd = viewd / np.maximum(np.linalg.norm(viewd, axis=1, keepdims=True), 1e-9)
        facing = np.einsum("ij,ij->i", nrm, viewd)
        visible = (pcd[:, 2] > 0.15) & (np.isnan(facing) | (facing > cfg.grazing_cos))
        visible &= rng.random(D) >= cfg.dot_dropout
        f_px = 0.5 * (cam.K[0, 0] + cam.K[1, 1])
        if visible.any():
            vis_idx = np.flatnonzero(visible)
            px = cam.project(pts[vis_idx])
            radii = f_px * cfg.dot_radius_m / pcd[vis_idx, 2]
            for k, (uvp, r) in enumerate(zip(px, radii)):
                if not (r + 2 <= uvp[0] < w - r - 2 and r + 2 <= uvp[1] < h - r - 2):
                    visible[vis_idx[k]] = False
                    continue
                zq = zbuf[int(uvp[1]) // S, int(uvp[0]) // S]
                if np.isfinite(zq) and pcd[vis_idx[k], 2] > zq + cfg.zbuf_tol_m:
                    visible[vis_idx[k]] = False   # behind another body
                    continue
                cv2.circle(canvas,
                           (int(round(uvp[0] * 16)), int(round(uvp[1] * 16))),
                           int(round(r * 16)), 235, -1, cv2.LINE_AA, shift=4)
        canvas = cv2.GaussianBlur(canvas, (0, 0), 0.6)
        noisy = canvas.astype(np.float32) + rng.normal(0, 3.0, canvas.shape)
        return np.clip(noisy, 0, 255).astype(np.uint8), visible


def render_capture(scene: SynthScene, capture_dir: str,
                   rng: np.random.Generator) -> np.ndarray:
    """Dot sweep frames for every camera; returns per-dot view counts (F, D)."""
    cfg = scene.cfg
    F, D, _ = scene.dots_world.shape
    n_views = np.zeros((F, D), int)
    renderer = SceneRenderer(scene)
    cam_names = sorted(scene.rig_truth.cameras)
    for name in cam_names:
        os.makedirs(os.path.join(capture_dir, "frames", name), exist_ok=True)
    for f in range(F):
        body_pts, body_nrm = renderer.body_points(f)
        for name in cam_names:
            img, visible = renderer.render(name, f, body_pts, body_nrm, rng)
            cv2.imwrite(os.path.join(capture_dir, "frames", name,
                                     f"frame_{f:05d}.png"), img)
            n_views[f] += visible.astype(int)
        if f % 300 == 0:
            print(f"  frame {f}/{F}")
    print(f"  {F} frames x {len(cam_names)} cameras")

    # Hand-held assist shots at the anchor frames: sampled poses kept only
    # when the board is detectable in the shot (as an operator would keep
    # them); each records its truth pose for scoring.
    scene.assist_shots = []
    if cfg.n_assist > 0:
        os.makedirs(os.path.join(capture_dir, "assist"), exist_ok=True)
        base = scene.assist_cam
        for f in scene.anchor_frames:
            body_pts, body_nrm = renderer.body_points(f)
            kept = attempts = 0
            while kept < cfg.n_assist and attempts < cfg.n_assist * 4:
                attempts += 1
                R, t = _assist_pose(scene, rng)
                cam = Camera(name=base.name, image_size=base.image_size,
                             K=base.K, dist=base.dist, R=R, t=t)
                img, _ = renderer.render_free(cam, f, body_pts, body_nrm, rng)
                if detect_board(img, scene.board, min_corners=10) is None:
                    continue
                fname = f"assist/shot_{f:05d}_{kept}.png"
                cv2.imwrite(os.path.join(capture_dir, fname), img)
                scene.assist_shots.append(
                    {"frame": int(f), "file": fname, "R": R, "t": t})
                kept += 1
        print(f"  {len(scene.assist_shots)} assist shots over "
              f"{len(scene.anchor_frames)} anchor frames")

    m = np.stack([scene.enc.measure(j, scene.q[:, i], rng)
                  for i, j in enumerate(scene.joints)], axis=1)
    np.save(os.path.join(capture_dir, "encoders.npy"), m)
    np.save(os.path.join(capture_dir, "q_true.npy"), scene.q)
    manifest = {
        "format": CAPTURE_FORMAT,
        "joints": scene.joints,
        "sweep_frames": {j: [int(i) for i in v] for j, v in scene.sweep_frames.items()},
        "anchor_frames": [int(i) for i in scene.anchor_frames],
        "cameras": sorted(scene.rig_truth.cameras),
        "meta": {"producer": "render_synthetic", "seed": scene.cfg.seed,
                 "image_size": list(cfg.image_size)},
    }
    if scene.assist_shots:
        manifest["assist"] = {
            "camera": scene.assist_cam.name,
            "shots": [{"frame": s["frame"], "file": s["file"]}
                      for s in scene.assist_shots],
        }
    with open(os.path.join(capture_dir, "capture.yaml"), "w") as f:
        yaml.safe_dump(manifest, f, sort_keys=False)
    return n_views


def write_truth(scene: SynthScene, n_views: np.ndarray, truth_dir: str) -> None:
    os.makedirs(truth_dir, exist_ok=True)
    scene.rig_truth.save(os.path.join(truth_dir, "rig_truth.yaml"))
    np.savez_compressed(
        os.path.join(truth_dir, "truth.npz"),
        q=scene.q, dots_world=scene.dots_world, n_views=n_views,
        base_R=scene.base_R, base_t=scene.base_t,
    )
    with open(os.path.join(truth_dir, "truth.yaml"), "w") as f:
        yaml.safe_dump({
            "joints": scene.joints,
            "dot_links": scene.dot_links,
            "enc_offset": {j: float(scene.enc.offset[j]) for j in scene.joints},
            "hardstop_b0": {j: float(v) for j, v in scene.hardstop_b0.items()},
        }, f, sort_keys=False)
    if scene.assist_shots:
        with open(os.path.join(truth_dir, "assist_truth.yaml"), "w") as f:
            yaml.safe_dump([
                {"frame": s["frame"], "file": s["file"],
                 "R": [[float(v) for v in row] for row in s["R"]],
                 "t": [float(v) for v in s["t"]]}
                for s in scene.assist_shots
            ], f, sort_keys=False)


def quick_config(seed: int = 0) -> SynthConfig:
    """Smoke-run scene: wrist + index + thumb chains only, lower resolution.

    Coverage shrinks; step sizes (and so tracking difficulty) stay at the
    full-run design point — a smoke run must not stress tracking beyond
    what the pipeline is built for.
    """
    return SynthConfig(
        seed=seed, n_cameras=3, camera_indices=(0, 1, 4),
        image_size=(960, 720), focal_px=790.0,
        n_intrinsic=10,
        sweep_joints=("wrist", "index_abd", "index_mcp", "index_pip",
                      "thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"),
        dot_links=("wrist", "index_mcp", "index_pip",
                   "thumb_abd", "thumb_mcp", "thumb_dip"),
    )


def render_all(cfg: SynthConfig, out_dir: str) -> SynthScene:
    rng = np.random.default_rng(cfg.seed + 1)  # image noise; scene uses cfg.seed
    scene = build_scene(cfg)
    print(f"Scene: {len(scene.q)} frames, "
          f"{scene.dots_world.shape[1]} dots ({cfg.n_clutter} clutter), "
          f"{cfg.n_cameras} cameras")
    print("Rendering calibration images...")
    render_calibration(scene, os.path.join(out_dir, "calib"), rng)
    print("Rendering capture frames...")
    n_views = render_capture(scene, os.path.join(out_dir, "capture"), rng)
    write_truth(scene, n_views, os.path.join(out_dir, "truth"))
    print(f"wrote synthetic set to {out_dir}")
    return scene


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--quick", action="store_true",
                    help="Fewer cameras/steps for a fast smoke run")
    args = ap.parse_args()
    cfg = SynthConfig(seed=args.seed)
    if args.quick:
        cfg = quick_config(args.seed)
    render_all(cfg, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
