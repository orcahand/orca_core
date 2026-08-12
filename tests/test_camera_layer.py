"""Tests for the Phase-1 camera layer (tools/abs_calibration).

Pins the load-bearing pieces below the end-to-end synthetic validation
(validate_camera_layer.py, not run here — it renders images for minutes):
triangulation geometry, board detection and pose, tracking identity rules,
cross-camera association, the rigidity/despike filters, motion-signature
link assignment, and the sweep protocol's continuity guarantees.
"""

import os
import sys

import numpy as np
import pytest

TOOLS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "tools", "abs_calibration",
)
sys.path.insert(0, TOOLS_DIR)

from assign import STATIC_LINK, assign_links, link_signatures  # noqa: E402
from board import BoardSpec, board_pose, detect_board  # noqa: E402
from cameras import (Camera, Rig, fundamental_matrix, sampson_distance,  # noqa: E402
                     triangulate)
from dots import (_despike_tracks, associate_tracks, merge_components,  # noqa: E402
                  rigidity_filter, track_2d, triangulate_components)
from model import HELD_POSE, JOINT_ROMS, KinematicModel  # noqa: E402
from sweep_plan import build_plan, sweep_step_deg  # noqa: E402


def _camera(name, az_deg, dist=0.5, f=900.0, distortion=True):
    """Camera on a horizontal arc looking at the origin."""
    az = np.deg2rad(az_deg)
    center = dist * np.array([np.sin(az), -0.2, -np.cos(az)])
    z = -center / np.linalg.norm(center)
    x = np.cross(z, np.array([0.0, -1.0, 0.0]))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z])
    d = np.array([-0.1, 0.03, 1e-3, -8e-4, 0.0]) if distortion else np.zeros(5)
    return Camera(
        name=name, image_size=(1280, 960),
        K=np.array([[f, 0, 640.0], [0, f, 480.0], [0, 0, 1.0]]),
        dist=d, R=R, t=-R @ center,
    )


@pytest.fixture(scope="module")
def cams():
    return {n: _camera(n, az) for n, az in
            (("cam0", -30), ("cam1", 0), ("cam2", 35))}


@pytest.fixture(scope="module")
def nominal():
    return KinematicModel.load("right")


# ---- cameras/geometry --------------------------------------------------------

def test_triangulate_recovers_exact_point(cams):
    rng = np.random.default_rng(0)
    for _ in range(20):
        X = rng.uniform([-0.08, -0.08, -0.05], [0.08, 0.08, 0.05])
        obs = {n: c.project(X)[0] for n, c in cams.items()}
        Xr, rms = triangulate(obs, cams)
        assert np.linalg.norm(Xr - X) < 1e-6
        assert rms < 1e-3


def test_triangulate_needs_two_views(cams):
    X, rms = triangulate({"cam0": np.array([100.0, 100.0])}, cams)
    assert np.isnan(X).all() and np.isnan(rms)


def test_triangulate_passes_nan_views(cams):
    X = np.array([0.01, -0.02, 0.0])
    obs = {n: c.project(X)[0] for n, c in cams.items()}
    obs["cam2"] = np.array([np.nan, np.nan])
    Xr, _ = triangulate(obs, cams)
    assert np.linalg.norm(Xr - X) < 1e-6


def test_undistort_inverts_projection(cams):
    cam = cams["cam0"]
    X = np.array([0.03, 0.01, -0.02])
    px = cam.project(X)
    n = cam.undistort(px)[0]
    pc = cam.R @ X + cam.t
    assert np.allclose(n, pc[:2] / pc[2], atol=1e-8)


def test_sampson_zero_for_true_pair(cams):
    X = np.array([[0.02, -0.03, 0.01]])
    a = cams["cam0"].undistort_px(cams["cam0"].project(X))
    b = cams["cam1"].undistort_px(cams["cam1"].project(X))
    F = fundamental_matrix(cams["cam0"], cams["cam1"])
    assert sampson_distance(F, a, b)[0] < 1e-6
    assert sampson_distance(F, a, b + [0.0, 5.0])[0] > 1.0


def test_rig_yaml_roundtrip(cams, tmp_path):
    rig = Rig(cameras=dict(cams), board=BoardSpec(measured_square_m=0.0299))
    path = str(tmp_path / "rig.yaml")
    rig.save(path)
    back = Rig.load(path)
    assert set(back.cameras) == set(cams)
    for n in cams:
        assert np.allclose(back.cameras[n].K, cams[n].K)
        assert np.allclose(back.cameras[n].R, cams[n].R)
    assert back.board.measured_square_m == pytest.approx(0.0299)


# ---- board -------------------------------------------------------------------

def test_board_scale_from_calipers():
    spec = BoardSpec(square_m=0.030, marker_m=0.022, measured_square_m=0.0306)
    assert spec.effective_square_m == pytest.approx(0.0306)
    assert spec.effective_marker_m == pytest.approx(0.022 * 1.02)


def test_board_detect_and_pose():
    spec = BoardSpec()
    tex = spec.make().generateImage((700, 500), marginSize=40)
    det = detect_board(tex, spec)
    assert det is not None and len(det[0]) == 24

    cam = _camera("c", 0, distortion=False)
    import cv2
    from render_synthetic import _board_texture, _warp_board
    tex_hi, H_tex = _board_texture(spec)
    R_bc = cv2.Rodrigues(np.array([0.2, -0.15, 0.1]))[0]
    t_bc = np.array([-0.09, -0.06, 0.45])
    img = _warp_board(cam, R_bc, t_bc, tex_hi, H_tex,
                      np.full(cam.image_size[::-1], 130, np.uint8))
    got = board_pose(img, spec, cam.K, cam.dist)
    assert got is not None
    R, t, n, rms = got
    ang = np.degrees(np.arccos(np.clip((np.trace(R_bc.T @ R) - 1) / 2, -1, 1)))
    assert ang < 0.2
    assert np.linalg.norm(t - t_bc) < 0.002
    assert rms < 1.0


# ---- tracking ----------------------------------------------------------------

def test_track_2d_keeps_identity_and_bridges_gap():
    a = np.stack([np.linspace(100, 160, 30), np.full(30, 100.0)], axis=1)
    b = np.stack([np.linspace(160, 100, 30), np.full(30, 130.0)], axis=1)
    dets = []
    for f in range(30):
        pts = [a[f]] if f not in (10, 11) else []   # a missing briefly
        pts.append(b[f])
        dets.append(np.array(pts))
    tracks = track_2d(dets, max_step_px=6.0)
    assert len(tracks) == 2
    by_start = sorted(tracks, key=lambda t: t[0, 1])
    assert np.allclose(by_start[0][12], a[12], atol=0.1)   # reacquired
    assert np.allclose(by_start[1][12], b[12], atol=0.1)


def test_track_2d_no_adoption_beyond_reacquire_gate():
    # Dot X dies at frame 10; dot Y appears nearby two frames later, within
    # the wide motion gate but beyond the tight re-acquisition gate that
    # applies after a miss — Y must start a fresh track, not extend X's.
    x = np.stack([np.linspace(50, 70, 10), np.full(10, 50.0)], axis=1)
    dets = [np.array([p]) for p in x]
    dets += [np.empty((0, 2))]
    y0 = x[-1] + np.array([16.0, 0.0])
    dets += [np.array([y0 + [1.0 * i, 0]]) for i in range(8)]
    tracks = track_2d(dets, max_step_px=30.0, min_len=5)
    assert len(tracks) == 2


# ---- association + filters ---------------------------------------------------

def _orbit_tracks(cams, centers, n_frames=40, r=0.02):
    """Perfect per-camera 2D tracks of dots orbiting their centers."""
    t = np.linspace(0, 2 * np.pi, n_frames)
    world = np.stack([
        c + np.stack([r * np.cos(t + i), r * np.sin(t + i), 0 * t], axis=1)
        for i, c in enumerate(centers)
    ])  # (D, F, 3)
    tracks = {}
    for n, cam in cams.items():
        tr = np.stack([cam.project(world[d]) for d in range(len(centers))])
        tracks[n] = tr
    return world, tracks


def test_associate_and_triangulate_components(cams):
    centers = [np.array([0.0, 0.0, 0.0]), np.array([0.05, 0.02, -0.01])]
    world, tracks = _orbit_tracks(cams, centers)
    junk = np.full((1, world.shape[1], 2), np.nan)
    junk[0, :10] = np.stack([np.linspace(30, 60, 10), np.full(10, 40.0)], axis=1)
    tracks["cam0"] = np.concatenate([tracks["cam0"], junk])

    rig = Rig(cameras=dict(cams), board=BoardSpec())
    comps = associate_tracks(tracks, rig)
    assert len(comps) == 2
    pts, stats = triangulate_components(comps, tracks, rig, world.shape[1])
    assert stats["n_rejected"] == 0
    for d in range(2):
        err = [np.nanmin(np.linalg.norm(pts[f] - world[d, f], axis=1))
               for f in range(world.shape[1])]
        assert np.nanmax(err) < 1e-4


def test_association_rejects_3d_incoherent_merge(cams):
    # Same epipolar plane, different depth: cam0/cam1 see a phantom pair
    # whose 3D sits 30 mm from the component's anchors one frame away.
    centers = [np.array([0.0, 0.0, 0.0])]
    world, tracks = _orbit_tracks(cams, centers, n_frames=30)
    shifted = world[0] + np.array([0.0, 0.0, 0.03])
    for n in ("cam0", "cam1"):
        ghost = cams[n].project(shifted)
        ghost_tr = np.full_like(tracks[n][0], np.nan)
        ghost_tr[15:] = ghost[15:]
        real = tracks[n][0].copy()
        real[15:] = np.nan
        tracks[n] = np.stack([real, ghost_tr])
    tracks["cam2"] = tracks["cam2"][:, :15]
    tracks["cam2"] = np.pad(tracks["cam2"], ((0, 0), (0, 15), (0, 0)),
                            constant_values=np.nan)

    rig = Rig(cameras=dict(cams), board=BoardSpec())
    comps = associate_tracks(tracks, rig)
    for comp in comps:
        member_sets = [(c, i) for c, idx in comp.items() for i in idx]
        reals = {(c, 0) for c in ("cam0", "cam1")}
        ghosts = {(c, 1) for c in ("cam0", "cam1")}
        assert not (set(member_sets) >= reals and set(member_sets) & ghosts)


def test_merge_components_rejoins_split_dot(cams):
    # One dot associated as two camera-pair components (association
    # under-merges by design); a second dot stays distinct.
    cams4 = dict(cams)
    cams4["cam3"] = _camera("cam3", 70)
    centers = [np.array([0.0, 0.0, 0.0]), np.array([0.05, 0.02, -0.01])]
    world, tracks = _orbit_tracks(cams4, centers, n_frames=40)
    rig = Rig(cameras=dict(cams4), board=BoardSpec())
    comps = [
        {"cam0": [0], "cam1": [0]},
        {"cam2": [0], "cam3": [0]},   # same dot, disjoint camera subset
        {"cam0": [1], "cam1": [1], "cam2": [1], "cam3": [1]},
    ]
    pts, _ = triangulate_components(comps, tracks, rig, world.shape[1])
    merged = merge_components(comps, tracks, pts)
    assert len(merged) == 2
    sizes = sorted(len(sum(c.values(), [])) for c in merged)
    assert sizes == [4, 4]


def test_despike_removes_interior_and_boundary_spikes():
    pts = np.zeros((30, 1, 3))
    pts[:, 0, 0] = np.linspace(0, 0.03, 30)
    pts[14, 0, 2] = 0.02          # interior spike
    pts[29, 0, 2] = 0.05          # boundary spike (velocity gate)
    n = _despike_tracks(pts)
    assert n == 2
    assert np.isnan(pts[14, 0]).all() and np.isnan(pts[29, 0]).all()


def test_rigidity_filter_drops_stolen_epoch():
    # Identifying WHICH epoch was stolen needs a majority of clean epochs
    # (the production plan has 17); use three, corrupt the last.
    rng = np.random.default_rng(1)
    F = 90
    sweep_frames = {"j": np.arange(0, 30), "k": np.arange(30, 60),
                    "l": np.arange(60, 90)}
    sigs = {"A": frozenset({"j"}), "B": frozenset()}
    base = rng.uniform(-0.05, 0.05, size=(4, 3))
    ang = np.concatenate([np.linspace(0, 1.2, 30), np.zeros(60)])
    Rz = np.stack([[[np.cos(a), -np.sin(a), 0],
                    [np.sin(a), np.cos(a), 0], [0, 0, 1]] for a in ang])
    moving = np.einsum("fij,kj->fki", Rz, base)          # 4 dots on link A
    static = np.broadcast_to(rng.uniform(-0.05, 0.05, (3, 3)), (F, 3, 3)).copy()
    corrupted = moving.copy()
    corrupted[60:, 0] += np.array([0.0, 0.03, 0.0])      # stolen in epoch l
    dot_obs = {"A": corrupted, "B": static}
    n = rigidity_filter(dot_obs, sweep_frames, sigs)
    assert n >= 25
    assert np.isnan(dot_obs["A"][60:, 0]).all()
    assert np.isfinite(dot_obs["A"][:60, 0]).all()
    assert np.isfinite(dot_obs["A"][:, 1:]).all()


# ---- assignment --------------------------------------------------------------

def test_link_signatures_unique(nominal):
    sigs = link_signatures(nominal)
    assert sigs["fixed0"] == frozenset()
    assert sigs["index_pip"] == {"wrist", "index_abd", "index_mcp", "index_pip"}
    vals = list(sigs.values())
    assert len(set(vals)) == len(vals)


def _fk_dots(nominal, q_rows, joints, placements):
    poses = nominal.link_poses({j: q_rows[:, i] for i, j in enumerate(joints)})
    cols = []
    for link, local in placements:
        R, t = poses[link]
        cols.append(np.einsum("fij,j->fi", R, local) + t)
    return np.stack(cols, axis=1)


def test_assign_links_from_fk(nominal):
    joints = nominal.joint_names
    roms = {j: JOINT_ROMS[j] for j in joints}
    q, sweeps = build_plan(joints, roms, dict(HELD_POSE), step_scale=4.0)
    placements = [("index_pip", np.array([0.008, 0.002, 0.02])),
                  ("wrist", np.array([0.01, -0.005, 0.03])),
                  ("thumb_mcp", np.array([-0.006, 0.004, 0.015]))]
    pts = _fk_dots(nominal, q, joints, placements)
    static = np.broadcast_to([0.1, 0.1, -0.05], (len(q), 1, 3))
    pts = np.concatenate([pts, static], axis=1)
    m = q.copy()  # encoders = truth is fine for span checking
    assignment, report = assign_links(pts, sweeps, nominal, m, joints)
    got = {k: link for link, idxs in assignment.items() for k in idxs}
    assert got == {0: "index_pip", 1: "wrist", 2: "thumb_mcp", 3: STATIC_LINK}


def test_assign_chain_falls_back_to_most_proximal(nominal):
    # An index_pip dot hidden during every sweep that moves it is static in
    # all its observed frames: the candidate set is the fixed0..index_pip
    # chain, and the equivalence assignment parks it on the static link.
    joints = nominal.joint_names
    roms = {j: JOINT_ROMS[j] for j in joints}
    q, sweeps = build_plan(joints, roms, dict(HELD_POSE), step_scale=4.0)
    pts = _fk_dots(nominal, q, joints,
                   [("index_pip", np.array([0.008, 0.002, 0.02]))])
    own = set(np.concatenate([sweeps[j] for j in
                              ("wrist", "index_abd", "index_mcp", "index_pip")]))
    pts[sorted(own)] = np.nan
    assignment, report = assign_links(pts, sweeps, nominal, q, joints)
    assert assignment == {STATIC_LINK: [0]}
    assert report[0]["chain"][0] == STATIC_LINK


# ---- sweep protocol ----------------------------------------------------------

def test_build_plan_continuity_and_membership():
    joints = ["wrist", "index_abd", "index_mcp"]
    roms = {j: JOINT_ROMS[j] for j in joints}
    held = {j: HELD_POSE[j] for j in joints}
    q, sweeps = build_plan(joints, roms, held,
                           abd_flexion_fracs=(0.2, 0.7))
    steps = np.abs(np.diff(q, axis=0))
    for c, j in enumerate(joints):
        assert steps[:, c].max() <= sweep_step_deg(j) + 1e-9
    assert np.allclose(q[0], [held[j] for j in joints])
    assert np.allclose(q[-1], [held[j] for j in joints])
    # abd sweep frames hold the flexion posture fixed within each leg
    abd_frames = np.asarray(sweeps["index_abd"])
    assert len(abd_frames) > 0
    mcp_at_abd = q[abd_frames, 2]
    assert set(np.round(np.unique(mcp_at_abd), 6)).issubset(
        {round(roms["index_mcp"][0] + 2 + f * (roms["index_mcp"][1] - 2
         - roms["index_mcp"][0] - 2), 6) for f in (0.2, 0.7)})
