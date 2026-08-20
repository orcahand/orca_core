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
# Weak zero-centred prior on the curve terms: a joint whose sweep went
# unobserved (occlusion) must degrade to "no correction", not to a runaway
# polynomial. Far wider than any real INL, so data-rich joints ignore it.
SIGMA_CURVE = 2.0  # deg per curve coefficient

# Tactile tip-press location sigma vs resultant force (pre-characterisation
# placeholders per design doc section 6: grazing touches centroid to
# ~1-1.5 mm, firm multi-taxel blobs to sub-mm; the Phase-3 flat-reference
# characterisation replaces these).
SIGMA_TIP_MAX = 0.0018   # m, at threshold force
SIGMA_TIP_MIN = 0.0008   # m, firm press
SIGMA_TIP_SLOPE = 0.0007  # m per newton of the weaker side's force
SIGMA_BASE_PIN = 1e-4    # pins the base when no world-frame class is present

MIN_DOT_FRAMES = 3     # a dot seen fewer times than this is dropped
# Manual abd captures brace a finger at its physical stop, which can read
# just past the manifold's fitted grid edge; quadratic extrapolation this
# far stays well under the manifold sigma.
ARG_RANGE_SLACK_DEG = 1.5


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


def _anchor_sigmas(sigma_deg: np.ndarray | None, mask: np.ndarray) -> np.ndarray:
    """Per-anchor residual sigmas (rad): the detection's own uncertainty when
    the session carries it, floored at the default so an optimistic sigma
    never outweighs the class; the default otherwise."""
    if sigma_deg is None:
        return np.full(int(mask.sum()), SIGMA_DIR)
    s = np.deg2rad(np.asarray(sigma_deg, float)[mask])
    return np.clip(np.where(np.isfinite(s), s, SIGMA_DIR), SIGMA_DIR, None)


def _rodrigues(rvec: np.ndarray) -> np.ndarray:
    theta = np.linalg.norm(rvec)
    if theta < 1e-12:
        return np.eye(3)
    k = rvec / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


class Estimator:
    def __init__(self, nominal: KinematicModel, *, offsets_only: bool = False,
                 prior: PriorConfig | None = None,
                 manifolds: list | None = None):
        """``manifolds``: precomputed abd-block contact manifolds — a list of
        ``{pair: [jointA, jointB], arg, out, poly, sigma_deg, arg_range,
        posture: {joint: deg}}`` entries (see build_contact_manifold.py),
        one per pair and flexion posture; events with no matching entry are
        skipped with a warning rather than guessed."""
        self.nominal = nominal
        self.joints = nominal.joint_names
        self.K = 1 if offsets_only else 3
        self.prior = prior or PriorConfig()
        self.manifolds = manifolds or []
        self.rom_mid = np.array([np.mean(JOINT_ROMS[j]) for j in self.joints])
        self._ctx = None
        self._m_ordered = None
        self.filled_joints: list = []

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

    def m_in_model_order(self, ds: Dataset) -> np.ndarray:
        """Session encoder columns permuted into ``self.joints`` order.

        Producers store columns in their own joint order; every residual here
        indexes them by the model's, so the two must be reconciled once.

        A contact-only session may lack the WRIST (a hand whose encoder loop
        does not cover it): the wrist is common to both chains of every
        self-contact, so the residuals are exactly invariant to its value —
        it is filled with its nominal zero, its offset stays on the prior,
        and the sigma gate reports it unconstrained (``filled_joints``).
        A missing finger joint does not cancel and stays a hard error, as
        does any missing joint when world-frame observations are present.
        """
        if self._m_ordered is not None:
            return self._m_ordered
        self.filled_joints = []
        if list(ds.joints) == list(self.joints):
            return ds.m
        missing = [j for j in self.joints if j not in ds.joints]
        contact_only = not (ds.dot_obs or ds.dir_obs or ds.frame_axes
                            or ds.plate_obs)
        if missing and not (contact_only
                            and all(j == "wrist" for j in missing)):
            raise ValueError(f"session lacks joints the model needs: {missing}")
        self.filled_joints = missing
        src = list(ds.joints)
        cols = []
        for j in self.joints:
            if j in src:
                cols.append(ds.m[:, src.index(j)])
            else:
                cols.append(np.zeros(len(ds.m)))
        return np.column_stack(cols)

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
        self._m_ordered = None
        self._m_ordered = self.m_in_model_order(ds)
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
        # Contact residuals are base-invariant (both sides share the base),
        # so with no world-frame class the base pose is pure gauge — pin it.
        self._pin_base = not (any(k[1].any() for k in masks.values())
                              or ds.dir_obs or ds.frame_axes or ds.plate_obs)
        self.skipped_contacts = []

    # ---- residuals ---------------------------------------------------------

    def _world_poses(self, x: np.ndarray, ds: Dataset):
        rvec, t, b = self.unpack(x)
        q = self.apply_cal(self.m_in_model_order(ds), b)
        poses = self.nominal.link_poses({j: q[:, i] for i, j in enumerate(self.joints)})
        Rb = _rodrigues(rvec)
        world = {}
        for link, (R, tt) in poses.items():
            world[link] = (np.einsum("ij,fjk->fik", Rb, R),
                           np.einsum("ij,fj->fi", Rb, tt) + t)
        return world

    def world_link_poses(self, x: np.ndarray, ds: Dataset):
        """Posed world frames of every link at every frame of ``ds`` for the
        parameter vector ``x`` — ``{link: (R (F,3,3), t (F,3))}``. This is
        what anchor detection predicts contours from."""
        return self._world_poses(x, ds)

    def base_pose(self, x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        rvec, t, _ = self.unpack(x)
        return _rodrigues(rvec), t

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
            sig = _anchor_sigmas(ds.dir_sigma.get(link), mask)
            res.append((np.cross(pred, obs[mask]) / sig[:, None]).ravel())

        # Mesh-referenced frame anchors (forearm — mandatory for the wrist —
        # and the palm dorsal shell): two axes constrain full orientation.
        for link in sorted(ds.frame_axes):
            axes_obs = ds.frame_axes[link]
            sig_rows = ds.frame_axes_sigma.get(link)
            R = world[link][0][ds.dir_frames]
            for row, (col, obs) in enumerate(zip((2, 0), axes_obs)):
                mask = np.isfinite(obs).all(axis=1)
                if mask.any():
                    sig = _anchor_sigmas(
                        sig_rows[row] if sig_rows is not None else None, mask)
                    res.append((np.cross(R[mask][:, :, col], obs[mask])
                                / sig[:, None]).ravel())

        for frame_idx, finger, obs in ds.plate_obs:
            R, t = world[DISTAL_LINK[finger]]
            pred = R[frame_idx] @ TIP_POINT_LOCAL[finger] + t[frame_idx]
            res.append((pred - obs) / SIGMA_PLATE)

        res.extend(self._contact_residuals(ds, world, b))

        if self._pin_base:
            res.append(x[:6] / SIGMA_BASE_PIN)

        for i, j in enumerate(self.joints):
            sigma = self.prior.sigma(j)
            if sigma is not None:
                res.append(np.atleast_1d((b[i, 0] - self.prior.mean.get(j, 0.0)) / sigma))
        if self.K > 1:
            res.append((b[:, 1:] / SIGMA_CURVE).ravel())

        return np.concatenate(res)

    def _contact_residuals(self, ds: Dataset, world, b) -> list:
        """Contact classes — base-invariant, so they constrain offsets with
        no camera at all (and cannot constrain the wrist or base: no
        self-contact crosses the wrist).

        tip_press: the two pads' contact centroids are one physical point;
        predicted through each finger's chain they must coincide, with a
        force-dependent sigma (grazing touches localise worse and indent
        less — both regimes are wanted).

        abd_block: at first contact between adjacent fingers the pair sits
        on the precomputed mesh-contact manifold ``q_out = poly(q_arg)``.
        Both encoders were READ at the event, so no hardstop value is
        assumed anywhere — the parked finger's stop only provided
        mechanical stiffness. Events without a matching manifold (or
        outside its fitted range/posture) are skipped and reported, never
        guessed.
        """
        out = []
        self.skipped_contacts = []  # rewritten every call; read after solve
        q = self.apply_cal(self.m_in_model_order(ds), b)
        jidx = {j: i for i, j in enumerate(self.joints)}
        for e in ds.contact_obs:
            f = e.frame
            if e.kind == "tip_press":
                mounts = self.nominal.sensor_mounts
                if e.body_a not in mounts or e.body_b not in mounts:
                    self.skipped_contacts.append((e.kind, e.body_a, e.body_b,
                                                  "no sensor mount"))
                    continue
                pts = []
                for finger, p_local in ((e.body_a, e.p_a), (e.body_b, e.p_b)):
                    mnt = mounts[finger]
                    R, t = world[DISTAL_LINK[finger]]
                    p_link = mnt.rotation @ p_local + mnt.translation
                    pts.append(R[f] @ p_link + t[f])
                f_min = min(e.force_a or 0.0, e.force_b or 0.0)
                sigma = max(SIGMA_TIP_MIN, SIGMA_TIP_MAX - SIGMA_TIP_SLOPE * f_min)
                out.append((pts[0] - pts[1]) / sigma)
            elif e.kind == "abd_block":
                pair = sorted((e.body_a, e.body_b))
                man, why = self._match_manifold(
                    pair, self.m_in_model_order(ds)[f], jidx)
                if man is None:
                    self.skipped_contacts.append((e.kind, e.body_a, e.body_b, why))
                    continue
                q_arg = q[f, jidx[man["arg"]]]
                pred = float(np.polyval(man["poly"], q_arg))
                q_out = q[f, jidx[man["out"]]]
                out.append(np.atleast_1d((q_out - pred) / man["sigma_deg"]))
        return out

    def _match_manifold(self, pair: list, m_row: np.ndarray, jidx: dict):
        why = "no manifold"
        for man in self.manifolds:
            if sorted(man["pair"]) != pair:
                continue
            tol = man.get("posture_tol_deg", 3.0)
            if not all(abs(m_row[jidx[j]] - v) <= tol
                       for j, v in man.get("posture", {}).items() if j in jidx):
                if why == "no manifold":
                    why = "posture mismatch"
                continue
            lo, hi = man["arg_range"]
            if not (lo - ARG_RANGE_SLACK_DEG <= m_row[jidx[man["arg"]]]
                    <= hi + ARG_RANGE_SLACK_DEG):
                why = "outside manifold range"
                continue
            return man, ""
        return None, why

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
