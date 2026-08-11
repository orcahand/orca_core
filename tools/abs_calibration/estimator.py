# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Offsets + per-ROM-curve estimator for the absolute calibration.

Parameters: base pose (6) and per-joint calibration coefficients
``q̂ = m + b0 + b1·u + b2·u²`` with ``u = (m − rom_mid)/50`` (offsets-only mode
fits b0 alone). Dot positions on the links are nuisance parameters eliminated
in closed form (variable projection): given the pose parameters, each dot's
link-local position is the masked mean of its back-transformed observations.
Observations may be missing (NaN) anywhere — occlusion is the norm on a real
rig — and the mask structure is fixed per dataset, so the residual vector
keeps a constant length across solver iterations.

The estimator only ever consults the *nominal* model; model error in the data
flows into the recovered parameters, which is what the synthetic study
measures. Parameter blocks are enable-flagged (offsets / curves; base always
on) so the future kinematic-release block slots in without restructuring.

Solver: scipy ``least_squares`` (trf, soft_l1) with numeric Jacobians — at
~60 parameters with VarPro this is fast on any laptop and keeps the
dependency footprint small; a torch backend only becomes worth it if the
kinematic-release tier multiplies the parameter count.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from dataset import Dataset
from model import JOINT_ROMS, KinematicModel, TIP_POINT_LOCAL, DISTAL_LINK, PALM_LINK

U_SCALE = 50.0  # deg; normalizes the curve argument for conditioning

SIGMA_DOT = 1.5e-4    # m
SIGMA_DIR = np.deg2rad(0.1)
SIGMA_PLATE = 1.5e-4  # m
SIGMA_PRIOR = {"normal": 1.0, "wide": 4.0}  # deg

MIN_DOT_FRAMES = 3     # a dot seen fewer times than this is dropped


@dataclass
class PriorConfig:
    """Per-joint MAP prior lever: mode, centre; 'off' joints get de-novo init."""
    mode: dict[str, str] = field(default_factory=dict)   # joint -> normal|wide|off
    mean: dict[str, float] = field(default_factory=dict)  # joint -> b0 centre (deg)

    def sigma(self, joint: str) -> float | None:
        m = self.mode.get(joint, "normal")
        return None if m == "off" else SIGMA_PRIOR[m]


@dataclass
class SolveResult:
    x: np.ndarray
    b: np.ndarray                  # (J, K) calibration coefficients
    base_rvec: np.ndarray
    base_t: np.ndarray
    sigma_b0_deg: dict[str, float]  # per-joint 1σ of the offset term
    cost: float
    scipy_result: object


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
        self._ctx = None

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

    # ---- masking context ---------------------------------------------------

    def prepare(self, ds: Dataset) -> None:
        """Precompute the (constant) observation masks for this dataset."""
        masks = {}
        for link, obs in ds.dot_obs.items():
            valid = np.isfinite(obs).all(axis=2)          # (F, K)
            keep = valid.sum(axis=0) >= MIN_DOT_FRAMES    # (K,)
            masks[link] = (valid[:, keep], keep)
        dir_masks = {
            link: np.isfinite(obs).all(axis=1)
            for link, obs in ds.dir_obs.items()
        }
        self._ctx = (masks, dir_masks)

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

    def dot_layout(self, world, ds: Dataset, link: str) -> tuple[np.ndarray, np.ndarray]:
        """VarPro closed form: masked-mean link-local dot positions.

        Returns ``(p_hat (Kk,3), keep (K,))`` for the kept dots.
        """
        valid, keep = self._ctx[0][link]
        R, t = world[link]
        obs = ds.dot_obs[link][:, keep, :]
        local = np.einsum("fji,fkj->fki", R, np.nan_to_num(obs) - t[:, None, :])
        w = valid[..., None].astype(float)
        p_hat = (local * w).sum(axis=0) / np.maximum(w.sum(axis=0), 1e-9)
        return p_hat, keep

    def residuals(self, x: np.ndarray, ds: Dataset) -> np.ndarray:
        if self._ctx is None:
            self.prepare(ds)
        masks, dir_masks = self._ctx
        world = self._world_poses(x, ds)
        _, _, b = self.unpack(x)
        res = []

        for link in sorted(ds.dot_obs):
            valid, keep = masks[link]
            if not keep.any():
                continue
            p_hat, _ = self.dot_layout(world, ds, link)
            R, t = world[link]
            obs = ds.dot_obs[link][:, keep, :]
            pred = np.einsum("fij,kj->fki", R, p_hat) + t[:, None, :]
            diff = (pred - np.nan_to_num(obs)) * valid[..., None]
            res.append((diff[valid] / SIGMA_DOT).ravel())

        for link in sorted(ds.dir_obs):
            obs = ds.dir_obs[link]
            mask = dir_masks[link]
            if not mask.any():
                continue
            R, _ = world[link]
            pred = R[ds.dir_frames][mask][:, :, 2]
            res.append((np.cross(pred, obs[mask]) / SIGMA_DIR).ravel())

        # Mesh-referenced frame anchors (forearm — mandatory for the wrist —
        # and the palm dorsal shell): two axes constrain full orientation.
        for link in sorted(ds.frame_axes):
            axes_obs = ds.frame_axes[link]
            R = world[link][0][ds.dir_frames]
            for col, obs in zip((2, 0), axes_obs):
                mask = np.isfinite(obs).all(axis=1)
                if mask.any():
                    res.append((np.cross(R[mask][:, :, col], obs[mask]) / SIGMA_DIR).ravel())

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
        b[:, 0] = [hardstop_b0.get(j, 0.0) for j in self.joints]
        rvec, t = (np.zeros(3), np.zeros(3)) if base_guess is None else base_guess
        return self.pack(rvec, t, b)

    def denovo_init(self, x: np.ndarray, ds: Dataset, joints: list[str],
                    span_deg: float = 35.0, step_deg: float = 1.0) -> np.ndarray:
        """1-D coarse search of b0 for joints whose hardstop init is untrusted.

        Robust losses treat the informative residuals of a grossly wrong init
        as outliers, so those joints need a basin-finding pass before the
        solver runs.
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

    def solve(self, ds: Dataset, x0: np.ndarray, max_nfev: int = 120) -> SolveResult:
        self.prepare(ds)
        result = least_squares(
            self.residuals, x0, args=(ds,), method="trf", loss="soft_l1",
            f_scale=3.0, x_scale=self.x_scale(), max_nfev=max_nfev,
        )
        rvec, t, b = self.unpack(result.x)
        return SolveResult(
            x=result.x, b=b, base_rvec=rvec, base_t=t,
            sigma_b0_deg=self._sigma_b0(result),
            cost=float(result.cost), scipy_result=result,
        )

    def _sigma_b0(self, result) -> dict[str, float]:
        """Approximate 1σ of each joint's offset from the solution Jacobian.

        Gauss-Newton covariance ``s²·(JᵀJ)⁻¹`` with the residual variance
        estimated from the robust cost; the numeric Jacobian of the VarPro'd
        residual already includes the dot-nuisance response. Approximate under
        soft_l1 — honest enough to gate persistence, not a certified bound.
        """
        J = result.jac
        m, n = J.shape
        dof = max(m - n, 1)
        s2 = 2.0 * result.cost / dof
        try:
            cov = s2 * np.linalg.pinv(J.T @ J)
        except np.linalg.LinAlgError:
            return {j: float("inf") for j in self.joints}
        return {
            j: float(np.sqrt(max(cov[6 + i * self.K, 6 + i * self.K], 0.0)))
            for i, j in enumerate(self.joints)
        }

    # ---- diagnostics: sweep axis fit vs nominal (§4b) ----------------------

    def axis_diagnostic(self, x: np.ndarray, ds: Dataset) -> dict[str, dict]:
        """Per joint: sweep-fitted axis vs the nominal axis, in the reference
        link's frame — ``{joint: {"deg": angle, "floor_deg": noise_floor}}``.

        All kept dots of the observed link, expressed per-step in the solved
        model's reference-link frame, trace circles normal to the joint axis;
        the axis is the smallest eigenvector of the summed track covariances.
        ``floor_deg`` is the fit's own noise floor from eigen-perturbation
        (δθ ≈ √(2·λ2·λ3/N)/(λ2−λ3)): flexion sweeps resolve ~0.1°, while
        short abduction arcs with near-parallel dot tangents are honest to
        only ~1° — improvable on a real rig by sweeping abd at 2–3 flexion
        postures. A ``deg`` is meaningful only against its ``floor_deg``.
        This is the v1 geometry-error diagnostic that triggers (or retires)
        kinematic release.
        """
        if self._ctx is None:
            self.prepare(ds)
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
            frames = ds.sweep_frames.get(j)
            if frames is None or link not in ds.dot_obs:
                continue
            pts = self._points_in_ref(world, ds, link, ref, frames)
            if pts is None:
                continue
            fit = _circle_axis(pts)
            if fit is None:
                continue
            axis_fit, floor_deg = fit

            node = self.nominal.nodes[self.nominal.index[j]]
            f0 = frames[0]
            a_world = world[j][0][f0] @ node.axis
            R_ref_w = world[ref][0][f0] if ref else np.eye(3)
            a_nominal = R_ref_w.T @ a_world
            cosang = abs(float(np.clip(axis_fit @ a_nominal, -1, 1)))
            out[j] = {
                "deg": float(np.degrees(np.arccos(cosang))),
                "floor_deg": floor_deg,
            }
        return out

    def _points_in_ref(self, world, ds: Dataset, link: str, ref: str | None,
                       frames: np.ndarray):
        """Observed dots of ``link`` expressed per-step in ``ref``'s frame.

        Returns (S, Kk, 3) with NaN where unobserved, or None if unusable.
        The reference pose per step comes from the solved *model* — held-joint
        drift is visible to the encoders, so the model reference is smooth,
        while re-registering the reference from its own dots would inject
        per-frame Kabsch wobble into every track. ``ref=None`` keeps world
        coordinates (wrist sweep: the base is static).
        """
        valid, keep = self._ctx[0][link]
        if not keep.any():
            return None
        obs = ds.dot_obs[link][frames][:, keep, :]

        if ref is None:
            return obs

        R_ref, t_ref = world[ref]
        return np.einsum(
            "sji,skj->ski", R_ref[frames], obs - t_ref[frames][:, None, :]
        )


def _circle_axis(pts: np.ndarray) -> tuple[np.ndarray, float] | None:
    """Common rotation axis of per-dot point tracks (S, K, 3), with its own
    noise floor.

    Every track lies in a plane normal to the joint axis, so the axis is the
    smallest eigenvector of the summed per-track covariances. The floor comes
    from first-order eigenvector perturbation: δθ ≈ √(2·λ2·λ3/N)/(λ2−λ3) —
    large when the tracks are thin arcs with parallel tangents (λ2 small).
    """
    C = np.zeros((3, 3))
    n_pts = 0
    for k in range(pts.shape[1]):
        track = pts[:, k, :]
        track = track[np.isfinite(track).all(axis=1)]
        if len(track) < 8:
            continue
        d = track - track.mean(axis=0)
        C += d.T @ d
        n_pts += len(track)
    if n_pts == 0:
        return None
    evals, evecs = np.linalg.eigh(C)
    l3, l2 = evals[0], evals[1]
    gap = max(l2 - l3, 1e-12)
    floor = float(np.degrees(np.sqrt(2.0 * l2 * l3 / max(n_pts, 1)) / gap))
    return evecs[:, 0], floor
