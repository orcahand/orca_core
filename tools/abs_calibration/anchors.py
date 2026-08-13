# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Phase-2 anchor detection: mesh-referenced link directions and frame axes
from board-referenced views.

The absolute observables of the calibration are measured here: dot tracks fix
curves and relative geometry, but every per-joint offset is pure gauge to dots
alone (a constant joint shift is absorbed exactly by the unknown dot layout).
What pins the offsets is the printed surface itself — the URDF mesh posed by
the predicted model, aligned to the observed silhouette edges.

One mechanism serves every anchor class: model-guided sparse-contour pose
refinement. Mesh points on the occluding rim (normals grazing the view ray,
z-buffer visible, backdrop far enough behind, silicone-skin zones excluded)
are projected into each view; the image gradient is searched along the local
2D contour normal for the observed edge; a damped Gauss-Newton over the
link's world pose (all views jointly) explains the edge displacements. The
refined rotation yields the link z direction (``dir`` anchors; near-cylinder
roll is prior-held and never used) or the z and x axes (``frame`` anchors:
the forearm tower group — mandatory, the only observation that splits the
wrist offset from base pose — and the palm's dorsal printed shell).

The measurement is anchored to observed gradients: the prediction only
provides correspondence and search regions. Views with no usable edges do
not silently return the prediction — information gates (edge count, fit rms,
data-only rotation covariance along the axes actually used) fail the anchor
into NaN with a reason instead.

A hand-held assist camera slots in as just another view: each shot resolves
its own world pose from the ChArUco board in frame (multi-view without SfM).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.ndimage import convolve1d

from cameras import Camera


@dataclass
class AnchorDetectConfig:
    rim_cos: float = 0.35           # |normal . view| below this = rim band
    zbuf_cell_px: int = 2
    zbuf_tol_m: float = 0.004
    backdrop_gap_m: float = 0.012   # another body closer behind = ambiguous edge
    backdrop_step_px: float = 4.0
    occ_cell_px: int = 4            # own-footprint occupancy grid
    contour_probe_px: float = 9.0   # outward step that must leave the footprint
    skin_margin_m: float = 0.0025   # rim points this close to a skin zone drop
    thin_px: float = 4.0            # one rim point per image cell
    max_points_per_view: int = 350
    search_px: float = 18.0         # covers prediction error + print bias
    grad_min: float = 5.0           # intensity/px; below = no edge
    ambiguity_ratio: float = 0.75   # competing peak this strong = ambiguous
    ambiguity_sep_px: float = 3.0
    near_pref_px: float = 12.0      # soft preference for the near edge
    outer_iters: int = 4            # re-search edges + relinearize
    huber_px: float = 2.0
    # The prior states how far the model prediction is trusted; it is also
    # what pins the directions the contour cannot see (cylinder roll, and
    # slide along a link whose end edges are occluded or vetoed).
    prior_rot_deg: float = 3.0
    prior_t_m: float = 0.005
    # A uniform outward edge offset (as-printed oversize, blooming, defocus)
    # is fitted as one nuisance parameter per refinement — otherwise the
    # solver explains it with depth translation along the weakest direction.
    prior_inflate_px: float = 3.0
    max_step_rot_deg: float = 5.0
    max_step_t_m: float = 0.008
    acc_rot_max_deg: float = 8.0    # accumulated correction beyond this = diverged
    acc_t_max_m: float = 0.012
    converge_rot_deg: float = 0.02
    converge_t_m: float = 1e-4
    min_points_dir: int = 30
    min_points_frame: int = 80
    max_rms_px: float = 2.5
    # Sanity bound, not the quality bar: each anchor ships its own sigma and
    # the estimator weights it; beyond this the measurement is junk.
    max_tilt_sigma_deg: float = 3.0
    denovo_span_deg: float = 35.0
    denovo_step_deg: float = 5.0


@dataclass
class ViewContext:
    """One board-referenced view of one captured pose: a camera with a world
    pose (fixed rig camera, or an assist shot self-localised off the board),
    its image, the predicted world pose of every mesh body at that pose, and
    an optional clutter predictor masking known background texture."""
    cam: Camera
    image: np.ndarray                       # grayscale
    poses: dict[str, tuple[np.ndarray, np.ndarray]]
    clutter: object = None                  # callable (..., 2) px -> intensity
    # Prior width of this view's 2D offset nuisance (see refine_group):
    # tight for calibrated rig cameras, loose for assist shots whose
    # board self-localisation carries the planar-pose tilt ambiguity.
    offset_prior_px: float = 1.5

    def __post_init__(self):
        self.image = np.asarray(self.image, np.float32)


def make_board_clutter(spec, cam: Camera, px_per_square: int = 64):
    """Predicted board-texture intensity behind any pixel of this view.

    The hand's silhouettes stand against the ChArUco board, whose checker
    and marker-bit transitions are strong gradients at arbitrary offsets
    inside the edge-search window — but the board pose is exactly known (it
    IS the world frame), so those gradients are predictable. Returns a
    callable mapping distorted pixels (..., 2) to the board intensity their
    ray hits (0/1; -1 off the board or behind the camera); the edge search
    masks gradients at its transitions.
    """
    tex = (spec.make().generateImage(
        (spec.squares_x * px_per_square, spec.squares_y * px_per_square),
        marginSize=0) > 127).astype(np.float32)
    w_m = spec.squares_x * spec.effective_square_m
    h_m = spec.squares_y * spec.effective_square_m
    C = cam.center

    def board_intensity(pos: np.ndarray) -> np.ndarray:
        sh = pos.shape[:-1]
        p = pos.reshape(-1, 2)
        n = cam.undistort(p)
        d_w = np.column_stack([n, np.ones(len(n))]) @ cam.R   # rows R^T d
        out = np.full(len(p), -1.0)
        ok = np.isfinite(d_w).all(axis=1) & (np.abs(d_w[:, 2]) > 1e-9)
        t = -C[2] / d_w[ok, 2]
        hit = C[None, :] + t[:, None] * d_w[ok]
        x, y = hit[:, 0], hit[:, 1]
        inb = (t > 0.05) & (x >= 0) & (x < w_m) & (y >= 0) & (y < h_m)
        ti = np.full(int(ok.sum()), -1.0)
        tx = np.clip((x[inb] / w_m * tex.shape[1]).astype(int), 0, tex.shape[1] - 1)
        ty = np.clip((y[inb] / h_m * tex.shape[0]).astype(int), 0, tex.shape[0] - 1)
        ti[inb] = tex[ty, tx]
        out[ok] = ti
        return out.reshape(sh)

    return board_intensity


@dataclass
class RefineResult:
    ok: bool
    R: np.ndarray | None                    # refined anchor-body world rotation
    t: np.ndarray | None
    n_points: int
    rms_px: float
    sigma_tilt_deg: float                   # max over the gated rotation axes
    reason: str = ""
    per_view: list = field(default_factory=list)


class AnchorScene:
    """Mesh clouds for anchor detection: per-body points + normals in link
    frames (from urdf_scene.MeshScene) with skin-proximity flags cached —
    rim candidates never sit on or near a silicone zone."""

    def __init__(self, mesh=None, *, n_per_body: int = 6000,
                 skin_margin_m: float = 0.0025):
        if mesh is None:
            from urdf_scene import MeshScene
            mesh = MeshScene(n_per_body=n_per_body)
        self.mesh = mesh
        self.bodies = mesh.bodies
        self.skin_near: dict[str, np.ndarray] = {}
        from scipy.spatial import cKDTree
        for body, (pts, _) in self.bodies.items():
            if body in getattr(mesh, "skin", {}):
                d, _ = cKDTree(mesh.skin[body]).query(pts, k=1)
                self.skin_near[body] = d < skin_margin_m
            else:
                self.skin_near[body] = np.zeros(len(pts), bool)


# ---- image sampling ----------------------------------------------------------

def bilinear_sample(img: np.ndarray, pos: np.ndarray) -> np.ndarray:
    """Sample ``img`` (h, w) at ``pos`` (..., 2) x,y; NaN outside."""
    h, w = img.shape
    x, y = pos[..., 0], pos[..., 1]
    ok = np.isfinite(x) & np.isfinite(y) \
        & (x >= 0) & (x <= w - 1.001) & (y >= 0) & (y <= h - 1.001)
    out = np.full(x.shape, np.nan, np.float32)
    if not ok.any():
        return out
    xs, ys = x[ok], y[ok]
    x0, y0 = np.floor(xs).astype(int), np.floor(ys).astype(int)
    fx, fy = (xs - x0).astype(np.float32), (ys - y0).astype(np.float32)
    out[ok] = (img[y0, x0] * (1 - fx) * (1 - fy)
               + img[y0, x0 + 1] * fx * (1 - fy)
               + img[y0 + 1, x0] * (1 - fx) * fy
               + img[y0 + 1, x0 + 1] * fx * fy)
    return out


_SMOOTH = np.array([0.1174, 0.1975, 0.2349, 0.1975, 0.1174], np.float32)
_SMOOTH = _SMOOTH / _SMOOTH.sum()


def edge_search(image: np.ndarray, uv: np.ndarray, n2d: np.ndarray,
                cfg: AnchorDetectConfig,
                clutter=None) -> tuple[np.ndarray, np.ndarray]:
    """Signed sub-pixel edge displacement along each contour normal.

    Samples the image along ``uv + s*n2d`` for s in [-W, W], smooths, and
    takes the strongest gradient magnitude with a soft preference for edges
    near the prediction. Polarity is not enforced (the backdrop varies:
    bright board squares, dark squares, background, a body further behind).
    A competing peak nearly as strong elsewhere on the line makes the point
    ambiguous — dropped, not guessed. ``clutter`` (see make_board_clutter)
    suppresses gradients at predicted background-texture transitions before
    any of that. Returns ``(delta_px (N,), ok (N,))``.
    """
    image = np.asarray(image, np.float32)
    W = int(round(cfg.search_px))
    s = np.arange(-W, W + 1, dtype=np.float32)
    pos = uv[:, None, :] + s[None, :, None] * n2d[:, None, :]
    prof = bilinear_sample(image, pos)                     # (N, 2W+1)
    bad_row = ~np.isfinite(prof).all(axis=1)
    prof = np.where(np.isfinite(prof), prof, 0.0)
    sm = convolve1d(prof, _SMOOTH, axis=1, mode="nearest")
    g = 0.5 * (sm[:, 2:] - sm[:, :-2])                     # gradient at s[1:-1]
    ag = np.abs(g)
    if clutter is not None:
        b = clutter(pos)                                   # (N, 2W+1)
        trans = np.zeros(b.shape, bool)
        db = np.abs(np.diff(b, axis=1)) > 0.15
        trans[:, 1:] |= db
        trans[:, :-1] |= db
        for _ in range(2):                                 # widen to +-3 px
            trans[:, 1:] |= trans[:, :-1]
            trans[:, :-1] |= trans[:, 1:]
        ag = np.where(trans[:, 1:-1], 0.0, ag)
    sg = s[1:-1]
    near = 1.0 / (1.0 + (sg / cfg.near_pref_px) ** 2)

    pick = np.argmax(ag * near[None, :], axis=1)
    rows = np.arange(len(uv))
    ag_pick = ag[rows, pick]
    ok = (ag_pick >= cfg.grad_min) & ~bad_row

    # Ambiguity: the strongest competing peak away from the pick.
    mask_near_pick = np.abs(sg[None, :] - sg[pick][:, None]) <= cfg.ambiguity_sep_px
    ag_masked = np.where(mask_near_pick, -np.inf, ag)
    second = ag_masked.max(axis=1)
    ok &= second <= cfg.ambiguity_ratio * np.maximum(ag_pick, 1e-9)

    # Sub-pixel parabola on |g| around the pick.
    p = np.clip(pick, 1, ag.shape[1] - 2)
    y0, y1, y2 = ag[rows, p - 1], ag[rows, p], ag[rows, p + 1]
    denom = y0 - 2 * y1 + y2
    safe = np.where(np.abs(denom) > 1e-9, denom, 1.0)
    frac = np.where(np.abs(denom) > 1e-9, 0.5 * (y0 - y2) / safe, 0.0)
    delta = sg[pick] + np.clip(frac, -1, 1) * (pick == p)
    return delta.astype(float), ok


# ---- geometry ----------------------------------------------------------------

def _rodrigues(w: np.ndarray) -> np.ndarray:
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def _proj_jacobian(cam: Camera, X: np.ndarray, eps: float = 1e-4) -> np.ndarray:
    """d(distorted px)/d(world point) by finite differences: (N, 2, 3)."""
    base = cam.project(X)
    cols = [(cam.project(X + eps * e) - base) / eps for e in np.eye(3)]
    return np.stack(cols, axis=2)


def _splat(uv: np.ndarray, z: np.ndarray, zb: np.ndarray, S: int) -> None:
    if not len(uv):
        return
    bx = (uv[:, 0] / S).astype(int)
    by = (uv[:, 1] / S).astype(int)
    inb = (bx >= 0) & (bx < zb.shape[1]) & (by >= 0) & (by < zb.shape[0])
    np.minimum.at(zb, (by[inb], bx[inb]), z[inb])


def _zbuf_lookup(zb: np.ndarray, uv: np.ndarray, S: int) -> np.ndarray:
    bx = np.clip((uv[:, 0] / S).astype(int), 0, zb.shape[1] - 1)
    by = np.clip((uv[:, 1] / S).astype(int), 0, zb.shape[0] - 1)
    return zb[by, bx]


class ViewGeom:
    """Per-view projection caches at the PREDICTED poses, shared by every
    target link of the pose: each body projected once, z-buffers built on
    demand. Veto decisions (visibility, backdrop) are made at the prediction;
    the refinement moves a group a few pixels inside bands this wide."""

    def __init__(self, scene: AnchorScene, ctx: ViewContext,
                 cfg: AnchorDetectConfig):
        self.cfg = cfg
        w, h = ctx.cam.image_size
        S = cfg.zbuf_cell_px
        self.shape = (h // S + 2, w // S + 2)
        self.proj: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._zbufs: dict = {}
        for body, (R, t) in ctx.poses.items():
            pts = scene.bodies[body][0] @ R.T + t
            pc_z = pts @ ctx.cam.R.T[:, 2] + ctx.cam.t[2]
            uv = np.full((len(pts), 2), np.nan)
            front = pc_z > 0.12
            if front.any():
                uv[front] = ctx.cam.project(pts[front])
            self.proj[body] = (uv, pc_z)

    def zbuf(self, exclude: str | None = None) -> np.ndarray:
        if exclude not in self._zbufs:
            zb = np.full(self.shape, np.inf)
            for body, (uv, z) in self.proj.items():
                if body == exclude:
                    continue
                fin = np.isfinite(uv).all(axis=1)
                _splat(uv[fin], z[fin], zb, self.cfg.zbuf_cell_px)
            self._zbufs[exclude] = zb
        return self._zbufs[exclude]


def rim_points(scene: AnchorScene, geom: ViewGeom, ctx: ViewContext,
               body: str, pose: tuple[np.ndarray, np.ndarray],
               cfg: AnchorDetectConfig):
    """Usable occluding-rim points of ``body`` in this view at ``pose``.

    Returns ``(X_world (N,3), uv (N,2), n2d (N,2))`` after the grazing-band,
    skin, z-buffer visibility, backdrop-depth and thinning rules, or None.
    """
    R, t = pose
    pts_l, nrm_l = scene.bodies[body]
    pts = pts_l @ R.T + t
    nrm = nrm_l @ R.T
    cam = ctx.cam
    w, h = cam.image_size
    S = cfg.zbuf_cell_px

    view = cam.center - pts
    view /= np.linalg.norm(view, axis=1, keepdims=True)
    facing = np.einsum("nk,nk->n", nrm, view)
    cand = (np.abs(facing) < cfg.rim_cos) & ~scene.skin_near[body]
    pc_z = pts @ cam.R.T[:, 2] + cam.t[2]
    cand &= pc_z > 0.12
    if not cand.any():
        return None
    idx = np.flatnonzero(cand)
    uv = cam.project(pts[idx])
    m = int(round(cfg.search_px)) + 3
    inb = ((uv[:, 0] >= m) & (uv[:, 0] < w - m)
           & (uv[:, 1] >= m) & (uv[:, 1] < h - m))
    idx, uv = idx[inb], uv[inb]
    if not len(idx):
        return None

    # Visible: nothing meaningfully nearer in the bin. Self-occlusion is
    # judged against the body's OWN splat at its CURRENT pose (it moves
    # between refinement iterations); other bodies stay at their cached
    # predicted poses.
    front = pc_z > 0.12
    uv_own = cam.project(pts[front])
    zb_own = np.full(geom.shape, np.inf)
    fin = np.isfinite(uv_own).all(axis=1)
    _splat(uv_own[fin], pc_z[front][fin], zb_own, S)
    z_near = np.minimum(_zbuf_lookup(geom.zbuf(body), uv, S),
                        _zbuf_lookup(zb_own, uv, S))
    vis = pc_z[idx] <= z_near + cfg.zbuf_tol_m
    idx, uv = idx[vis], uv[vis]
    if not len(idx):
        return None

    # Outward 2D contour normal from the projected surface normal.
    n2d = cam.project(pts[idx] + 1e-3 * nrm[idx]) - uv
    nn = np.linalg.norm(n2d, axis=1)
    good = nn > 1e-6
    idx, uv, n2d = idx[good], uv[good], n2d[good] / nn[good, None]
    if not len(idx):
        return None

    # Contour test: stepping outward must LEAVE the body's own projected
    # footprint. The grazing band alone also passes the interior of any
    # near-edge-on flat face (a box side seen obliquely projects as an
    # area, not a thin rim) — those interior points are not on the
    # occluding contour and would search noise.
    So = cfg.occ_cell_px
    occ = np.full((h // So + 2, w // So + 2), np.inf)
    _splat(uv_own[fin], pc_z[front][fin], occ, So)
    probe_occ = uv + cfg.contour_probe_px * n2d
    on_contour = ~np.isfinite(_zbuf_lookup(occ, probe_occ, So))
    idx, uv, n2d = idx[on_contour], uv[on_contour], n2d[on_contour]
    if not len(idx):
        return None

    # Backdrop: just outside the rim, any OTHER body nearer than the rim
    # point plus the gap makes the edge ambiguous (in front: the visible
    # edge is the occluder's; close behind: the gradient may sit on either).
    probe = uv + cfg.backdrop_step_px * n2d
    z_out = _zbuf_lookup(geom.zbuf(body), probe, S)
    keep = ~(np.isfinite(z_out) & (z_out < pc_z[idx] + cfg.backdrop_gap_m))
    idx, uv, n2d = idx[keep], uv[keep], n2d[keep]
    if not len(idx):
        return None

    # Thin to one point per image cell (most-grazing wins), then cap.
    graze = np.abs(facing[idx])
    cells = (uv / cfg.thin_px).astype(int)
    order = np.lexsort((graze, cells[:, 1], cells[:, 0]))
    _, first = np.unique(cells[order], axis=0, return_index=True)
    sel = order[first]
    if len(sel) > cfg.max_points_per_view:
        sel = sel[np.argsort(graze[sel])[:cfg.max_points_per_view]]
    return pts[idx[sel]], uv[sel], n2d[sel]


# ---- the refinement ----------------------------------------------------------

def _collect_matches(scene: AnchorScene, contexts, geoms, group_pose: dict,
                     cfg: AnchorDetectConfig):
    matches = []
    for vi, (ctx, geom) in enumerate(zip(contexts, geoms)):
        for body, pose in group_pose.items():
            got = rim_points(scene, geom, ctx, body, pose, cfg)
            if got is None:
                continue
            X, uv, n2d = got
            delta, ok = edge_search(ctx.image, uv, n2d, cfg,
                                    clutter=ctx.clutter)
            if ok.any():
                matches.append((vi, ctx.cam, X[ok], n2d[ok], delta[ok]))
    return matches


def _apply_update(group_pose: dict, w: np.ndarray, v: np.ndarray,
                  c: np.ndarray) -> dict:
    dR = _rodrigues(w)
    return {body: (dR @ R, dR @ (t - c) + c + v)
            for body, (R, t) in group_pose.items()}


def refine_group(scene: AnchorScene, contexts: list[ViewContext],
                 group: list[str], anchor_body: str, kind: str,
                 cfg: AnchorDetectConfig | None = None,
                 geoms: list[ViewGeom] | None = None,
                 debug: bool = False) -> RefineResult:
    """Multi-view damped Gauss-Newton on one rigid group's world pose.

    ``group`` lists the bodies moving rigidly together (their initial poses
    come from the contexts, which must agree on them — per-frame links use
    one frame's views; the static tower pools views across frames).
    ``anchor_body`` names the frame whose refined rotation is reported.
    ``kind``: ``dir`` gates the two tilt axes of the body z (roll about z is
    prior-held and unused); ``frame`` gates all three rotation axes.
    """
    cfg = cfg or AnchorDetectConfig()
    if geoms is None:
        geoms = [ViewGeom(scene, ctx, cfg) for ctx in contexts]
    min_pts = cfg.min_points_dir if kind == "dir" else cfg.min_points_frame
    pose = {b: contexts[0].poses[b] for b in group}
    all_pts = np.vstack([scene.bodies[b][0] @ P[0].T + P[1]
                         for b, P in pose.items()])
    c = all_pts.mean(axis=0)

    # Parameters: rotation (3), translation (3), radial inflation kappa (1),
    # and a 2D image-offset nuisance per view. The offsets model per-view
    # pose error — assist shots carry the planar-target tilt ambiguity of
    # their board self-localisation, which displaces all their edges
    # coherently. Marginalizing them in the info gate means a view can only
    # contribute the direction information its pose quality supports.
    V = len(contexts)
    NP = 7 + 2 * V
    xi_acc = np.zeros(NP)
    prior = np.concatenate([
        np.full(3, 1.0 / np.deg2rad(cfg.prior_rot_deg) ** 2),
        np.full(3, 1.0 / cfg.prior_t_m ** 2),
        [1.0 / cfg.prior_inflate_px ** 2],
        np.concatenate([np.full(2, 1.0 / ctx.offset_prior_px ** 2)
                        for ctx in contexts]),
    ])

    # Each iteration scores the pose that PRODUCED its residuals; the best
    # pose wins. A near-null direction (axis slide with the end edges vetoed)
    # can otherwise walk the pose out of the data — points fall off the true
    # silhouette and drop, which must count as worse, not as a smaller rms.
    def cost_of(d, n_ref):
        k = cfg.huber_px
        rho = np.where(np.abs(d) < k, 0.5 * d ** 2, k * (np.abs(d) - 0.5 * k))
        rho_max = k * (cfg.search_px - 0.5 * k)
        return float((rho.sum() + max(n_ref - len(d), 0) * rho_max)
                     / max(n_ref, 1))

    def build_system(matches):
        G, d = [], []
        for vi, cam, X, n2d, delta in matches:
            J = _proj_jacobian(cam, X)                    # (N, 2, 3)
            a = np.einsum("nk,nkj->nj", n2d, J)           # (N, 3)
            O = np.zeros((len(X), 2 * V))
            O[:, 2 * vi:2 * vi + 2] = n2d
            G.append(np.concatenate(
                [np.cross(X - c, a), a, np.ones((len(X), 1)), O], axis=1))
            # Pose state applies to geometry; kappa and view offsets apply
            # here, to the measured displacements.
            d.append(delta - xi_acc[6]
                     - n2d @ xi_acc[7 + 2 * vi:9 + 2 * vi])
        return np.vstack(G), np.concatenate(d)

    def per_view_report(matches):
        return [{"cam": cam.name, "n": int(len(delta))}
                for _, cam, _, _, delta in matches]

    best = None                                           # (cost, state...)
    n_ref = 0
    for it in range(cfg.outer_iters):
        matches = _collect_matches(scene, contexts, geoms, pose, cfg)
        n_pts = sum(len(m[4]) for m in matches)
        if n_pts < min_pts:
            if best is not None:
                break
            return RefineResult(False, None, None, n_pts, np.inf, np.inf,
                                reason=f"only {n_pts} edge points (< {min_pts})")
        G, d = build_system(matches)
        if it == 0:
            n_ref = n_pts

        wt = np.ones(len(d))
        dxi = np.zeros(NP)
        for _ in range(3):                                # IRLS-Huber
            A_data = (G * wt[:, None]).T @ G
            b = (G * wt[:, None]).T @ d
            A = A_data + np.diag(prior)
            try:
                dxi = np.linalg.solve(A, b - prior * xi_acc)
            except np.linalg.LinAlgError:
                return RefineResult(False, None, None, n_pts, np.inf, np.inf,
                                    reason="singular system")
            r = d - G @ dxi
            wt = np.minimum(1.0, cfg.huber_px / np.maximum(np.abs(r), 1e-9))

        rms = float(np.sqrt(np.mean(np.minimum(np.abs(d), 3 * cfg.huber_px) ** 2)))
        state = (cost_of(d, n_ref), dict(pose), n_pts, rms, A_data,
                 per_view_report(matches))
        if debug:
            print(f"    it{it}: n={n_pts} cost={state[0]:.2f} rms={rms:.2f} "
                  f"|d|med={np.median(np.abs(d)):.2f} "
                  f"dxi_rot={np.degrees(np.linalg.norm(dxi[:3])):.2f}deg "
                  f"dxi_t={np.linalg.norm(dxi[3:6]) * 1000:.2f}mm "
                  f"kappa={xi_acc[6]:.2f}+{dxi[6]:.2f}px "
                  f"|o|max={np.abs(xi_acc[7:]).max() if V else 0:.1f}px")
        if best is None or state[0] < best[0]:
            best = state

        scale = max(1.0,
                    np.linalg.norm(dxi[:3]) / np.deg2rad(cfg.max_step_rot_deg),
                    np.linalg.norm(dxi[3:6]) / cfg.max_step_t_m)
        dxi = dxi / scale
        xi_new = xi_acc + dxi
        if (np.linalg.norm(xi_new[:3]) > np.deg2rad(cfg.acc_rot_max_deg)
                or np.linalg.norm(xi_new[3:6]) > cfg.acc_t_max_m):
            break                                         # diverged; keep best
        pose = _apply_update(pose, dxi[:3], dxi[3:6], c)
        xi_acc = xi_new
        if (np.linalg.norm(dxi[:3]) < np.deg2rad(cfg.converge_rot_deg)
                and np.linalg.norm(dxi[3:6]) < cfg.converge_t_m):
            # Converged: score the final pose before leaving.
            matches = _collect_matches(scene, contexts, geoms, pose, cfg)
            n_pts = sum(len(m[4]) for m in matches)
            if n_pts >= min_pts:
                _, d_f = build_system(matches)
                rms_f = float(np.sqrt(np.mean(
                    np.minimum(np.abs(d_f), 3 * cfg.huber_px) ** 2)))
                state = (cost_of(d_f, n_ref), dict(pose), n_pts, rms_f, A_data,
                         per_view_report(matches))
                if state[0] < best[0]:
                    best = state
            break

    _, pose, n_pts, rms, A_data, per_view = best
    if rms > cfg.max_rms_px:
        return RefineResult(False, None, None, n_pts, rms, np.inf,
                            reason=f"edge rms {rms:.2f} px > {cfg.max_rms_px}")

    # Rotation covariance for the honesty gate: rotation gets NO prior (it is
    # the measurement), while the nuisances (translation, inflation, per-view
    # offsets) keep theirs — marginalizing them unbounded would charge the
    # rotation for freedom the offsets do not actually have.
    prior_nuis = prior.copy()
    prior_nuis[:3] = 1e-12
    try:
        C = np.linalg.inv(A_data + np.diag(prior_nuis))[:3, :3]
    except np.linalg.LinAlgError:
        return RefineResult(False, None, None, n_pts, rms, np.inf,
                            reason="rotation not observable")
    C = C * max(rms, 0.5) ** 2
    R_a, t_a = pose[anchor_body]
    if kind == "dir":
        z = R_a[:, 2]
        e1 = np.cross(z, [1.0, 0.0, 0.0])
        if np.linalg.norm(e1) < 1e-6:
            e1 = np.cross(z, [0.0, 1.0, 0.0])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(z, e1)
        sigma = max(float(np.sqrt(max(e @ C @ e, 0.0))) for e in (e1, e2))
    else:
        sigma = float(np.sqrt(max(np.linalg.eigvalsh(C)[-1], 0.0)))
    sigma_deg = float(np.degrees(sigma))
    if sigma_deg > cfg.max_tilt_sigma_deg:
        return RefineResult(False, None, None, n_pts, rms, sigma_deg,
                            reason=f"axis info {sigma_deg:.2f} deg "
                                   f"> {cfg.max_tilt_sigma_deg}")
    return RefineResult(True, R_a, t_a, n_pts, rms, sigma_deg,
                        per_view=per_view)


def edge_support(scene: AnchorScene, contexts: list[ViewContext],
                 group: list[str], cfg: AnchorDetectConfig | None = None
                 ) -> float:
    """Cheap edge-support score of a pose hypothesis (no refinement):
    each matched edge scores by closeness, so both count and agreement pay.
    Used by the de-novo wide search over a grossly-wrong joint offset."""
    cfg = cfg or AnchorDetectConfig()
    geoms = [ViewGeom(scene, ctx, cfg) for ctx in contexts]
    pose = {b: contexts[0].poses[b] for b in group}
    matches = _collect_matches(scene, contexts, geoms, pose, cfg)
    return float(sum(np.sum(1.0 - np.abs(delta) / (cfg.search_px + 1.0))
                     for _, _, _, _, delta in matches))


def denovo_scan(score_fn, span_deg: float, step_deg: float
                ) -> tuple[float, dict[float, float]]:
    """1-D coarse search of a joint-offset correction: the detection-side
    counterpart of the estimator's de-novo init (a grossly wrong offset puts
    the predicted contour outside the edge-search window — or worse, onto a
    neighbour's edges). ``score_fn(offset_deg)`` returns edge support."""
    scores = {}
    for off in np.arange(-span_deg, span_deg + step_deg, step_deg):
        scores[float(off)] = float(score_fn(float(off)))
    best = max(scores, key=scores.get)
    return best, scores
