# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Offsets + per-ROM-curve estimator for the absolute-calibration study.

Parameters: base pose (6) and per-joint calibration coefficients
``q̂ = m + b0 + b1·u + b2·u²`` with ``u = (m − rom_mid)/50`` (offsets-only mode
fits b0 alone). Dot positions on the links are nuisance parameters eliminated
in closed form (variable projection): given the pose parameters, each dot's
link-local position is the mean of its back-transformed observations, so the
outer optimiser never sees them. The estimator only ever consults the
*nominal* model — model error injected by the simulation flows into the
recovered parameters, which is the effect the study measures.

Parameter blocks are enable-flagged (offsets / curves; base always on) so the
future kinematic-release block slots in without restructuring — the
compatibility shape decided in the design doc (§5, §12).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from model import JOINT_ROMS, KinematicModel, TIP_POINT_LOCAL, DISTAL_LINK, PALM_LINK, palm_plane_local
from sim import Dataset, DIR_LINKS, DOT_LINKS, TOWER_LINK

U_SCALE = 50.0  # deg; normalizes the curve argument for conditioning

SIGMA_DOT = 1.5e-4    # m
SIGMA_DIR = np.deg2rad(0.1)
SIGMA_PLATE = 1.5e-4  # m
SIGMA_PRIOR = {"normal": 1.0, "wide": 4.0}  # deg


@dataclass
class PriorConfig:
    """Per-joint MAP prior lever: mode, centre; 'off' joints get de-novo init."""
    mode: dict[str, str] = field(default_factory=dict)   # joint -> normal|wide|off
    mean: dict[str, float] = field(default_factory=dict)  # joint -> b0 centre (deg)

    def sigma(self, joint: str) -> float | None:
        m = self.mode.get(joint, "normal")
        return None if m == "off" else SIGMA_PRIOR[m]


def _rodrigues(rvec: np.ndarray) -> np.ndarray:
    theta = np.linalg.norm(rvec)
    if theta < 1e-12:
        return np.eye(3)
    k = rvec / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


class Estimator:
    def __init__(self, nominal: KinematicModel, *, offsets_only: bool = False,
                 prior: PriorConfig | None = None):
        self.nominal = nominal
        self.joints = nominal.joint_names
        self.K = 1 if offsets_only else 3
        self.prior = prior or PriorConfig()
        self.rom_mid = np.array([np.mean(JOINT_ROMS[j]) for j in self.joints])
        n_local, c_local = palm_plane_local(nominal)
        self.palm_n_local, self.palm_c_local = n_local, c_local

    # ---- parameter packing -------------------------------------------------

    def pack(self, rvec, t, b) -> np.ndarray:
        return np.concatenate([rvec, t, np.asarray(b, float).ravel()])

    def unpack(self, x: np.ndarray):
        rvec, t = x[:3], x[3:6]
        b = x[6:].reshape(len(self.joints), self.K)
        return rvec, t, b

    def x_scale(self) -> np.ndarray:
        return np.concatenate([
            np.full(3, 0.05), np.full(3, 0.01),
            np.tile(np.concatenate([[1.0], np.full(self.K - 1, 0.5)]), len(self.joints)),
        ])

    # ---- calibration mapping ----------------------------------------------

    def apply_cal(self, m: np.ndarray, b: np.ndarray) -> np.ndarray:
        u = (m - self.rom_mid) / U_SCALE
        q = m + b[:, 0]
        if self.K > 1:
            q = q + b[:, 1] * u
        if self.K > 2:
            q = q + b[:, 2] * u ** 2
        return q

    # ---- residuals ---------------------------------------------------------

    def _world_poses(self, x: np.ndarray, ds: Dataset):
        rvec, t, b = self.unpack(x)
        q = self.apply_cal(ds.m, b)
        poses = self.nominal.link_poses({j: q[:, i] for i, j in enumerate(self.joints)})
        Rb = _rodrigues(rvec)
        world = {}
        for link, (R, tt) in poses.items():
            world[link] = (np.einsum("ij,fjk->fik", Rb, R),
                           np.einsum("ij,fj->fi", Rb, tt) + t)
        return world

    def residuals(self, x: np.ndarray, ds: Dataset) -> np.ndarray:
        world = self._world_poses(x, ds)
        _, _, b = self.unpack(x)
        res = []

        for link in DOT_LINKS:
            R, t = world[link]
            obs = ds.dot_obs[link]  # (F,K,3)
            local = np.einsum("fji,fkj->fki", R, obs - t[:, None, :])
            p_hat = local.mean(axis=0)  # VarPro closed form
            pred = np.einsum("fij,kj->fki", R, p_hat) + t[:, None, :]
            res.append(((pred - obs) / SIGMA_DOT).ravel())

        for link in DIR_LINKS:
            R, _ = world[link]
            pred = R[ds.dir_frames][:, :, 2]
            res.append((np.cross(pred, ds.dir_obs[link]) / SIGMA_DIR).ravel())

        # Forearm frame anchor: the only residual that can split base pose
        # from the wrist offset (everything else is distal of the wrist).
        Rt = world[TOWER_LINK][0][ds.dir_frames]
        for col, obs in zip((2, 0), ds.tower_axes_obs):
            res.append((np.cross(Rt[:, :, col], obs) / SIGMA_DIR).ravel())

        Rp, tp = world[PALM_LINK]
        n_pred = np.einsum("fij,j->fi", Rp[ds.palm_frames], self.palm_n_local)
        res.append((np.cross(n_pred, ds.palm_normal_obs) / SIGMA_DIR).ravel())
        c_pred = np.einsum("fij,j->fi", Rp[ds.palm_frames], self.palm_c_local) + tp[ds.palm_frames]
        plane_dist = np.einsum("fi,fi->f", c_pred - ds.palm_point_obs, n_pred)
        res.append(plane_dist / SIGMA_DOT)

        for frame_idx, finger, obs in ds.plate_obs:
            R, t = world[DISTAL_LINK[finger]]
            pred = R[frame_idx] @ TIP_POINT_LOCAL[finger] + t[frame_idx]
            res.append((pred - obs) / SIGMA_PLATE)

        for i, j in enumerate(self.joints):
            sigma = self.prior.sigma(j)
            if sigma is not None:
                res.append(np.atleast_1d((b[i, 0] - self.prior.mean.get(j, 0.0)) / sigma))

        return np.concatenate(res)

    # ---- initialisation ----------------------------------------------------

    def initial_x(self, hardstop_b0: dict[str, float], base_guess=None) -> np.ndarray:
        b = np.zeros((len(self.joints), self.K))
        b[:, 0] = [hardstop_b0[j] for j in self.joints]
        rvec, t = (np.zeros(3), np.zeros(3)) if base_guess is None else base_guess
        return self.pack(rvec, t, b)

    def denovo_init(self, x: np.ndarray, ds: Dataset, joints: list[str],
                    span_deg: float = 35.0, step_deg: float = 1.0) -> np.ndarray:
        """1-D coarse search of b0 for joints whose hardstop init is untrusted.

        Robust losses treat the informative residuals of a grossly wrong init
        as outliers, so those joints need a basin-finding pass before the
        solver runs (design doc §5).
        """
        x = x.copy()
        for j in joints:
            i = self.joints.index(j)
            best, best_cost = None, np.inf
            for b0 in np.arange(-span_deg, span_deg + step_deg, step_deg):
                x_try = x.copy()
                x_try[6 + i * self.K] = b0
                r = self.residuals(x_try, ds)
                cost = np.sum(np.minimum(r ** 2, 9.0))  # capped: ignore far outliers
                if cost < best_cost:
                    best, best_cost = b0, cost
            x[6 + i * self.K] = best
        return x

    # ---- solve -------------------------------------------------------------

    def solve(self, ds: Dataset, x0: np.ndarray, max_nfev: int = 120):
        result = least_squares(
            self.residuals, x0, args=(ds,), method="trf", loss="soft_l1",
            f_scale=3.0, x_scale=self.x_scale(), max_nfev=max_nfev,
        )
        return result

    # ---- diagnostics: sweep axis fit vs nominal (§4b) ----------------------

    def axis_diagnostic(self, x: np.ndarray, ds: Dataset) -> dict[str, float]:
        """Per joint: angle (deg) between the sweep-fitted axis and the nominal
        axis, both expressed in the reference link's frame. This is the v1
        geometry-error diagnostic that triggers (or retires) kinematic release.
        """
        obs_link = {
            "wrist": ("wrist", None),
            "thumb_cmc": ("thumb_abd", "wrist"), "thumb_abd": ("thumb_mcp", "wrist"),
            "thumb_mcp": ("thumb_dip", "thumb_abd"), "thumb_dip": ("thumb_dip", "thumb_mcp"),
        }
        for f in ("index", "middle", "ring", "pinky"):
            obs_link[f + "_abd"] = (f + "_mcp", "wrist")
            obs_link[f + "_mcp"] = (f + "_mcp", "wrist")
            obs_link[f + "_pip"] = (f + "_pip", f + "_mcp")

        world = self._world_poses(x, ds)
        out = {}
        for j, (link, ref) in obs_link.items():
            frames = ds.sweep_frames[j]
            if link not in ds.dot_obs:
                continue
            p_local = self._dot_layout(world, ds, link)
            R_fit, t_fit = _kabsch_batch(p_local, ds.dot_obs[link][frames])
            if ref is not None:
                p_ref = self._dot_layout(world, ds, ref)
                R_ref, _ = _kabsch_batch(p_ref, ds.dot_obs[ref][frames])
                R_rel = np.einsum("fji,fjk->fik", R_ref, R_fit)
            else:
                R_rel = R_fit
            axis_fit = _mean_rotation_axis(R_rel)

            node = self.nominal.nodes[self.nominal.index[j]]
            Rw_joint = world[j][0][frames[0]]
            a_world = Rw_joint @ node.axis
            R_ref_w = world[ref][0][frames[0]] if ref else np.eye(3)
            a_nominal = R_ref_w.T @ a_world
            cosang = abs(float(np.clip(axis_fit @ a_nominal, -1, 1)))
            out[j] = float(np.degrees(np.arccos(cosang)))
        return out

    def _dot_layout(self, world, ds: Dataset, link: str) -> np.ndarray:
        R, t = world[link]
        local = np.einsum("fji,fkj->fki", R, ds.dot_obs[link] - t[:, None, :])
        return local.mean(axis=0)


def _kabsch_batch(p_local: np.ndarray, obs: np.ndarray):
    """Rigid fit per frame: obs[f] ≈ R[f] @ p_local + t[f]."""
    p0 = p_local - p_local.mean(axis=0)
    o0 = obs - obs.mean(axis=1, keepdims=True)
    H = np.einsum("fki,kj->fij", o0, p0)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(np.einsum("fij,fjk->fik", U, Vt)))
    S = np.stack([np.diag([1, 1, di]) for di in d])
    R = np.einsum("fij,fjk,fkl->fil", U, S, Vt)
    t = obs.mean(axis=1) - np.einsum("fij,j->fi", R, p_local.mean(axis=0))
    return R, t


def _mean_rotation_axis(R_rel: np.ndarray) -> np.ndarray:
    """Dominant rotation axis of a batch of relative rotations (first→each)."""
    R0 = R_rel[0]
    axes = []
    for R in R_rel[len(R_rel) // 2:]:
        dR = R @ R0.T
        rv = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
        n = np.linalg.norm(rv)
        if n > 1e-9:
            axes.append(rv / n)
    mean = np.mean(axes, axis=0)
    return mean / np.linalg.norm(mean)
