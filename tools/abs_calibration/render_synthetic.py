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
from sim import DOT_LINKS, LINK_SPAN
from sweep_plan import build_plan

CAPTURE_FORMAT = 1


@dataclass
class SynthConfig:
    n_cameras: int = 4
    image_size: tuple[int, int] = (1280, 960)
    focal_px: float = 1050.0
    step_scale: float = 1.0         # multiplies the per-joint sweep step sizes
    sweep_joints: tuple | None = None   # None = all joints
    dot_links: tuple | None = None      # None = all standard dot links
    n_anchor: int = 8
    dots_per_link: int = 6
    dot_radius_m: float = 0.0025
    min_dot_sep_m: float = 0.009    # physical dots don't touch; helps tracking
    n_clutter: int = 4              # static bright spots near the mount
    n_intrinsic: int = 14
    n_extrinsic: int = 3
    grazing_cos: float = 0.15       # dot visible if normal . view dir exceeds this
    dot_dropout: float = 0.02
    abd_flexion_fracs: tuple | None = None  # hardware uses several; see sweep_plan
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
    dots_world: np.ndarray = field(init=False)    # (F, D, 3) all dots + clutter
    normals_world: np.ndarray = field(init=False)  # (F, D, 3), NaN = always visible
    dot_links: list[str] = field(init=False)       # per column; clutter -> "clutter"


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
    """The shared sweep protocol plus mid-range anchor frames, with per-frame
    held-joint creep noise as on a tendon hand."""
    swept = list(cfg.sweep_joints) if cfg.sweep_joints else list(joints)
    plan_q, plan_sweeps = build_plan(
        swept, {j: JOINT_ROMS[j] for j in swept},
        {j: cfg.held[j] for j in swept},
        step_scale=cfg.step_scale, abd_flexion_fracs=cfg.abd_flexion_fracs)

    jidx = {j: i for i, j in enumerate(joints)}
    held = np.array([cfg.held[j] for j in joints], float)
    q = np.tile(held, (len(plan_q), 1))
    for c, j in enumerate(swept):
        q[:, jidx[j]] = plan_q[:, c]
    q += rng.normal(0, 0.03, size=q.shape)

    anchor_rows = np.array([
        [rng.uniform(lo + 0.3 * (hi - lo), lo + 0.7 * (hi - lo))
         for lo, hi in (JOINT_ROMS[j] for j in joints)]
        for _ in range(cfg.n_anchor)
    ])
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
    base_t = board_center + np.array([0.0, 0.055, -0.13]) + rng.normal(0, 0.005, 3)
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
    _place_cameras(scene, rng)
    _place_dots(scene, rng)
    return scene


def _place_dots(scene: SynthScene, rng: np.random.Generator) -> None:
    """Dots on cylinder surfaces of the dot-bearing links, plus static clutter;
    world positions and outward normals for every frame.

    Angular placement mixes a camera-facing band with the phalanx sides
    (the design doc's "3-6 per phalanx side"). The sides are not cosmetic:
    a dot whose normal is parallel to a joint's axis keeps that normal
    fixed while the joint flexes, so axis-aligned side dots are the only
    ones guaranteed visible during their own link's sweep — which is when
    the calibration curve is measured. Distal links get mostly side dots;
    dots on the fully hidden face would never triangulate.
    """
    cfg = scene.cfg
    dot_links = list(cfg.dot_links) if cfg.dot_links else list(DOT_LINKS)
    cam_mid = np.mean([c.center for c in scene.rig_truth.cameras.values()], axis=0)
    held_poses = scene.kin.link_poses(
        {j: scene.cfg.held[j] for j in scene.joints})

    dot_local, normal_local, links = [], [], []
    for link in dot_links:
        R0, t0 = held_poses[link]
        Rw = scene.base_R @ R0[0]
        tw = scene.base_R @ t0[0] + scene.base_t
        d_local = Rw.T @ (cam_mid - tw)
        phi0 = float(np.arctan2(d_local[1], d_local[0]))
        axis = scene.kin.nodes[scene.kin.index[link]].axis
        phi_axis = float(np.arctan2(axis[1], axis[0]))
        side_frac = 0.7 if link.endswith(("_pip", "_dip")) else 0.45
        # Prefer the side that cameras actually support at the held pose —
        # on a real rig the operator sticks dots where cameras look.
        support = []
        for s in (0.0, np.pi):
            n_w = Rw @ np.array([np.cos(phi_axis + s), np.sin(phi_axis + s), 0.0])
            support.append(sum(
                float(np.dot(n_w, (c.center - tw)
                             / np.linalg.norm(c.center - tw))) > 0.2
                for c in scene.rig_truth.cameras.values()))
        p_side0 = 0.5 if support[0] == support[1] else \
            (0.8 if support[0] > support[1] else 0.2)

        pts, nrms = [], []
        sep = cfg.min_dot_sep_m
        attempts = 0
        while len(pts) < cfg.dots_per_link:
            attempts += 1
            if attempts % 200 == 0:
                sep *= 0.8  # small links can't always hold the full spacing
            z = rng.uniform(0.005, max(LINK_SPAN[link], 0.012))
            if rng.random() < side_frac:
                side = 0.0 if rng.random() < p_side0 else np.pi
                ang = phi_axis + side + rng.uniform(-0.35, 0.35)
            else:
                ang = phi0 + rng.uniform(-1.1, 1.1)          # camera-facing band
            r = rng.uniform(0.007, 0.011)
            cand = np.array([r * np.cos(ang), r * np.sin(ang), z])
            if all(np.linalg.norm(cand - p) >= sep for p in pts):
                pts.append(cand)
                nrms.append(np.array([np.cos(ang), np.sin(ang), 0.0]))
        dot_local.append(np.stack(pts))
        normal_local.append(np.stack(nrms))
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

    # Static bright spots on the mount below the hand (world +y is down).
    clutter = scene.base_t + rng.uniform(
        [-0.06, 0.01, -0.04], [0.06, 0.07, 0.04], size=(cfg.n_clutter, 3))
    pts_all.append(np.broadcast_to(clutter, (F, cfg.n_clutter, 3)))
    nrm_all.append(np.full((F, cfg.n_clutter, 3), np.nan))
    links.extend(["clutter"] * cfg.n_clutter)

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
    # Three cameras above the hand plus one below-front: a curling finger
    # rotates its dots' normals toward world +y ("down"), so without a low
    # camera every finger goes invisible during exactly its own flexion
    # sweep — the frames its calibration curve needs most.
    azimuths = (-165, 70, -20, -85, -115, 25)
    polars = (42, 55, 50, 20, 30, 45)
    for i in range(cfg.n_cameras):
        th = np.deg2rad(polars[i % len(polars)] + rng.normal(0, 2))
        ph = np.deg2rad(azimuths[i % len(azimuths)] + rng.normal(0, 3))
        r = rng.uniform(0.50, 0.58)
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
    for cam in scene.rig_truth.cameras.values():
        remap_xy = _distortion_remap(cam)
        d_int = os.path.join(calib_dir, cam.name, "intrinsic")
        d_ext = os.path.join(calib_dir, cam.name, "extrinsic")
        os.makedirs(d_int, exist_ok=True)
        os.makedirs(d_ext, exist_ok=True)
        for i in range(scene.cfg.n_intrinsic):
            R, t = _sample_intrinsic_pose(cam, scene.board, rng)
            img = render_board_image(cam, R, t, tex, H_tex, remap_xy, rng)
            cv2.imwrite(os.path.join(d_int, f"view_{i:03d}.png"), img)
        for i in range(scene.cfg.n_extrinsic):
            # Board fixed at the world origin: its camera-frame pose IS the
            # camera's world pose.
            img = render_board_image(cam, cam.R, cam.t, tex, H_tex, remap_xy, rng)
            cv2.imwrite(os.path.join(d_ext, f"still_{i:03d}.png"), img)
        print(f"  {cam.name}: {scene.cfg.n_intrinsic} intrinsic + "
              f"{scene.cfg.n_extrinsic} extrinsic images")


def _hand_body_segments(scene: SynthScene) -> list[tuple[np.ndarray, np.ndarray, float]]:
    """Crude dark hand body per frame: a capsule along each dot link, so dots
    sit on hand-colored background and the board behind is partly occluded,
    as on the real rig. Returns per link (p0 (F,3), p1 (F,3), width_m)."""
    poses = scene.kin.link_poses(
        {j: scene.q[:, i] for i, j in enumerate(scene.joints)})
    segs = []
    for link in DOT_LINKS:
        R, t = poses[link]
        Rw = np.einsum("ij,fjk->fik", scene.base_R, R)
        tw = np.einsum("ij,fj->fi", scene.base_R, t) + scene.base_t
        tip = tw + Rw[:, :, 2] * (LINK_SPAN[link] + 0.008)
        segs.append((tw, tip, 0.07 if link == "wrist" else 0.022))
    return segs


def render_capture(scene: SynthScene, capture_dir: str,
                   rng: np.random.Generator) -> np.ndarray:
    """Dot sweep frames for every camera; returns per-dot view counts (F, D)."""
    cfg = scene.cfg
    F, D, _ = scene.dots_world.shape
    w, h = cfg.image_size
    n_views = np.zeros((F, D), int)
    tex, H_tex = _board_texture(scene.board)
    segments = _hand_body_segments(scene)

    for cam in scene.rig_truth.cameras.values():
        cam_dir = os.path.join(capture_dir, "frames", cam.name)
        os.makedirs(cam_dir, exist_ok=True)
        f_px = 0.5 * (cam.K[0, 0] + cam.K[1, 1])
        remap_xy = _distortion_remap(cam)
        board_ud = _warp_board(cam, cam.R, cam.t, tex, H_tex,
                               np.full((h, w), 22, np.uint8))
        board_layer = cv2.remap(board_ud, remap_xy[0], remap_xy[1],
                                cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                                borderValue=22)
        for f in range(F):
            pts = scene.dots_world[f]
            nrm = scene.normals_world[f]
            pc = (cam.R @ pts.T).T + cam.t
            view = cam.center - pts
            view = view / np.maximum(np.linalg.norm(view, axis=1, keepdims=True), 1e-9)
            facing = np.einsum("ij,ij->i", nrm, view)
            visible = (pc[:, 2] > 0.15) & (np.isnan(facing) | (facing > cfg.grazing_cos))
            visible &= rng.random(D) >= cfg.dot_dropout

            canvas = board_layer.copy()
            for p0, p1, width in segments:
                a, b = p0[f], p1[f]
                za = (cam.R @ a + cam.t)[2]
                zb = (cam.R @ b + cam.t)[2]
                if za < 0.15 or zb < 0.15:
                    continue
                uv = cam.project(np.stack([a, b]))
                thick = max(int(round(f_px * width / (0.5 * (za + zb)))), 1)
                cv2.line(canvas, tuple(np.round(uv[0]).astype(int)),
                         tuple(np.round(uv[1]).astype(int)), 55, thick, cv2.LINE_AA)
            if visible.any():
                vis_idx = np.flatnonzero(visible)
                px = cam.project(pts[vis_idx])
                radii = f_px * cfg.dot_radius_m / pc[vis_idx, 2]
                for k, (uv, r) in enumerate(zip(px, radii)):
                    if not (r + 2 <= uv[0] < w - r - 2 and r + 2 <= uv[1] < h - r - 2):
                        visible[vis_idx[k]] = False
                        continue
                    cv2.circle(canvas, (int(round(uv[0] * 16)), int(round(uv[1] * 16))),
                               int(round(r * 16)), 235, -1, cv2.LINE_AA, shift=4)
            canvas = cv2.GaussianBlur(canvas, (0, 0), 0.6)
            noisy = canvas.astype(np.float32) + rng.normal(0, 3.0, canvas.shape)
            cv2.imwrite(os.path.join(cam_dir, f"frame_{f:05d}.png"),
                        np.clip(noisy, 0, 255).astype(np.uint8))
            n_views[f] += visible.astype(int)
        print(f"  {cam.name}: {F} frames")

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


def quick_config(seed: int = 0) -> SynthConfig:
    """Smoke-run scene: wrist + index + thumb chains only, lower resolution.

    Coverage shrinks; step sizes (and so tracking difficulty) stay at the
    full-run design point — a smoke run must not stress tracking beyond
    what the pipeline is built for.
    """
    return SynthConfig(
        seed=seed, n_cameras=3, image_size=(960, 720), focal_px=790.0,
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
