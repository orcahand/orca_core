"""Tests for the Phase-2 anchor detection layer (tools/abs_calibration).

Pins the load-bearing geometry below the end-to-end rendered validation
(validate_camera_layer.py): sub-pixel edge search, occluding-rim extraction
vetoes, multi-view contour pose refinement (recovery, failure honesty,
de-novo wide search), the shared anchor-pose sampler, and session merging.
All on synthetic clouds and images — no mesh assets, no full renders.
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

from anchors import (AnchorDetectConfig, AnchorScene, RefineResult,  # noqa: E402
                     ViewContext, ViewGeom, bilinear_sample, denovo_scan,
                     edge_search, edge_support, make_board_clutter,
                     refine_group, rim_points)
from board import BoardSpec  # noqa: E402
from cameras import Camera, Rig  # noqa: E402
from dataset import ContactEvent, Dataset, load_session, save_session  # noqa: E402
from merge_sessions import merge_datasets  # noqa: E402
from model import JOINT_ROMS, HELD_POSE  # noqa: E402
from plate import (PlateSpec, dimple_plate, dimple_world, plate_pose,  # noqa: E402
                   plate_pose_world, tag_corners_plate, write_scad)
from solve_session import split_plate_holdout  # noqa: E402
from sweep_plan import anchor_pose_plan  # noqa: E402


def _camera(name, az_deg, pol_deg=60.0, dist=0.5, f=1050.0):
    """Camera looking at the origin from (azimuth, polar)."""
    az, pol = np.deg2rad(az_deg), np.deg2rad(pol_deg)
    center = dist * np.array([
        np.sin(pol) * np.cos(az), np.sin(pol) * np.sin(az), -np.cos(pol)])
    z = -center / np.linalg.norm(center)
    x = np.cross(z, np.array([0.0, -1.0, 0.0]))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z])
    return Camera(
        name=name, image_size=(1280, 960),
        K=np.array([[f, 0, 640.0], [0, f, 480.0], [0, 0, 1.0]]),
        dist=np.array([-0.08, 0.02, 5e-4, -5e-4, 0.0]),
        R=R, t=-R @ center,
    )


def _cylinder_cloud(n, radius=0.008, length=0.07, seed=0):
    """Open cylinder surface, axis = local z, centred at the origin."""
    rng = np.random.default_rng(seed)
    theta = rng.uniform(0, 2 * np.pi, n)
    z = rng.uniform(-length / 2, length / 2, n)
    pts = np.stack([radius * np.cos(theta), radius * np.sin(theta), z], axis=1)
    nrm = np.stack([np.cos(theta), np.sin(theta), np.zeros(n)], axis=1)
    return pts, nrm


class _FakeMesh:
    def __init__(self, bodies, skin=None):
        self.bodies = bodies
        self.skin = skin or {}


def _render_bodies(cam, body_pts, bg=200, fg=60):
    """Solid dark silhouettes on a bright background by dilated splatting."""
    import cv2
    w, h = cam.image_size
    img = np.full((h, w), bg, np.uint8)
    pts = np.vstack(body_pts)
    pc = pts @ cam.R.T + cam.t
    front = pc[:, 2] > 0.05
    uv = np.round(cam.project(pts[front])).astype(int)
    inb = (uv[:, 0] >= 1) & (uv[:, 0] < w - 1) & (uv[:, 1] >= 1) & (uv[:, 1] < h - 1)
    uv = uv[inb]
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            img[uv[:, 1] + dy, uv[:, 0] + dx] = fg
    return cv2.GaussianBlur(img, (0, 0), 0.8)


def _rot(axis, deg):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    a = np.deg2rad(deg)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)


@pytest.fixture(scope="module")
def cyl_scene():
    pts, nrm = _cylinder_cloud(24000)
    return AnchorScene(mesh=_FakeMesh({"cyl": (pts, nrm)}))


@pytest.fixture(scope="module")
def cyl_views(cyl_scene):
    """Three views of the cylinder at a known truth pose; contexts predict a
    perturbed pose (2.5 deg tilt + 3 mm shift) — what detection must fix."""
    R_true = _rot([1.0, 0.3, 0.2], 12.0)
    t_true = np.array([0.005, -0.004, 0.01])
    R_pred = _rot([0.9, -0.4, 0.1], 2.5) @ R_true
    t_pred = t_true + np.array([0.002, 0.002, -0.001])
    cams = [_camera("c0", -30), _camera("c1", 40), _camera("c2", 100, pol_deg=75)]
    pts = cyl_scene.bodies["cyl"][0]
    contexts = []
    for cam in cams:
        img = _render_bodies(cam, [pts @ R_true.T + t_true])
        contexts.append(ViewContext(cam=cam, image=img,
                                    poses={"cyl": (R_pred, t_pred)}))
    return contexts, (R_true, t_true), (R_pred, t_pred)


# ---- image sampling / edge search --------------------------------------------

def test_bilinear_sample_values_and_bounds():
    img = np.arange(20, dtype=np.float32).reshape(4, 5)
    pos = np.array([[1.5, 1.0], [0.0, 0.0], [10.0, 1.0], [np.nan, 1.0]])
    got = bilinear_sample(img, pos)
    assert got[0] == pytest.approx(6.5)
    assert got[1] == pytest.approx(0.0)
    assert np.isnan(got[2]) and np.isnan(got[3])


def test_edge_search_subpixel_accuracy():
    import cv2
    img = np.full((60, 200), 60, np.uint8)
    img[:, 103:] = 200                      # edge at x ~= 102.5
    img = cv2.GaussianBlur(img, (0, 0), 0.8)
    uv = np.array([[97.0, y] for y in np.linspace(10, 50, 9)])
    n2d = np.tile([1.0, 0.0], (9, 1))
    delta, ok = edge_search(img, uv, n2d, AnchorDetectConfig())
    assert ok.all()
    assert np.allclose(delta, 5.5, atol=0.6)


def test_edge_search_rejects_flat_and_ambiguous():
    import cv2
    flat = np.full((40, 200), 128, np.uint8)
    uv = np.array([[100.0, 20.0]])
    n2d = np.array([[1.0, 0.0]])
    _, ok = edge_search(flat, uv, n2d, AnchorDetectConfig())
    assert not ok[0]

    two = np.full((40, 200), 60, np.uint8)
    two[:, 106:] = 200                      # +6 px and -8 px, equally strong
    two[:, :92] = 200
    two = cv2.GaussianBlur(two, (0, 0), 0.8)
    _, ok = edge_search(two, uv, n2d, AnchorDetectConfig())
    assert not ok[0]


# ---- rim extraction ----------------------------------------------------------

def test_rim_points_lie_on_silhouette(cyl_views, cyl_scene):
    contexts, _, (R_pred, t_pred) = cyl_views
    cfg = AnchorDetectConfig()
    ctx = contexts[0]
    geom = ViewGeom(cyl_scene, ctx, cfg)
    got = rim_points(cyl_scene, geom, ctx, "cyl", (R_pred, t_pred), cfg)
    assert got is not None
    X, uv, n2d = got
    assert len(X) >= 60
    # Rim normals are grazing: |n . view| small, and the 2D normals are unit.
    assert np.allclose(np.linalg.norm(n2d, axis=1), 1.0, atol=1e-6)
    # Silhouette points sit near the projected axis offset by ~radius: their
    # local radial coordinate is close to the cylinder radius.
    local = (X - t_pred) @ R_pred
    assert np.percentile(np.abs(np.linalg.norm(local[:, :2], axis=1) - 0.008),
                         90) < 0.002


def test_rim_backdrop_veto_drops_facing_sides(cyl_scene):
    """Two parallel cylinders 6 mm apart: rim points between them are
    ambiguous (backdrop within the gap) and must drop; outer sides survive."""
    pts, nrm = _cylinder_cloud(24000)
    mesh = _FakeMesh({"a": (pts, nrm), "b": (pts, nrm)})
    scene = AnchorScene(mesh=mesh)
    cam = _camera("c", 0)
    sep = np.array([0.022, 0.0, 0.0])       # 22 mm apart -> 6 mm face gap
    poses = {"a": (np.eye(3), np.zeros(3)), "b": (np.eye(3), sep)}
    img = _render_bodies(cam, [pts, pts + sep])
    ctx = ViewContext(cam=cam, image=img, poses=poses)
    cfg = AnchorDetectConfig()
    geom = ViewGeom(scene, ctx, cfg)
    got = rim_points(scene, geom, ctx, "a", poses["a"], cfg)
    assert got is not None
    X, uv, n2d = got
    # Survivors' outward normals must not point toward the neighbour (+x).
    toward = (X[:, 0] > 0.004) & (n2d[:, 0] > 0.3)
    assert toward.mean() < 0.05


# ---- refinement --------------------------------------------------------------

def test_refine_recovers_perturbed_cylinder(cyl_views, cyl_scene):
    contexts, (R_true, t_true), (R_pred, _) = cyl_views
    res = refine_group(cyl_scene, contexts, ["cyl"], "cyl", "dir")
    assert res.ok, res.reason
    z_err = np.degrees(np.arccos(np.clip(res.R[:, 2] @ R_true[:, 2], -1, 1)))
    z_init = np.degrees(np.arccos(np.clip(R_pred[:, 2] @ R_true[:, 2], -1, 1)))
    assert z_init > 2.0                      # the perturbation was real
    assert z_err < 0.25, (z_err, res)        # and detection removed it
    # Translation ALONG a capless cylinder's axis is a true nullspace (the
    # prior holds it at the prediction); only the perpendicular part is
    # measured, and must recover.
    z = R_true[:, 2]
    t_err = res.t - t_true
    assert np.linalg.norm(t_err - (t_err @ z) * z) < 0.0015
    # Sigma prices in what the per-view offset nuisances could absorb.
    assert res.sigma_tilt_deg < 0.4


def test_refine_fails_honestly_without_edges(cyl_views, cyl_scene):
    contexts, _, _ = cyl_views
    blank = [ViewContext(cam=c.cam, image=np.full_like(c.image, 128),
                         poses=c.poses) for c in contexts]
    res = refine_group(cyl_scene, blank, ["cyl"], "cyl", "dir")
    assert not res.ok
    assert res.R is None
    assert "edge points" in res.reason


def test_denovo_scan_recovers_gross_offset(cyl_views, cyl_scene):
    """A 20 deg wrong prediction about a joint-like axis is outside the edge
    search window; the wide scan must find the offset that restores support."""
    contexts, (R_true, t_true), _ = cyl_views
    pivot = t_true + R_true @ np.array([0.0, 0.0, -0.05])
    axis = R_true[:, 0]
    true_off = 20.0

    def posed(off_deg):
        dR = _rot(axis, off_deg - true_off)   # off = true_off -> truth pose
        out = []
        for c in contexts:
            R = dR @ R_true
            t = dR @ (t_true - pivot) + pivot
            out.append(ViewContext(cam=c.cam, image=c.image,
                                   poses={"cyl": (R, t)}))
        return out

    best, scores = denovo_scan(
        lambda off: edge_support(cyl_scene, posed(off), ["cyl"]),
        span_deg=30.0, step_deg=5.0)
    assert best == pytest.approx(true_off)
    # Wrong offsets keep partial spurious support (the displaced body still
    # crosses the true silhouette); the argmax must still stand clear.
    assert scores[best] > 1.5 * max(scores[-30.0], scores[0.0], 1e-9)


# ---- board clutter mask ------------------------------------------------------

def test_board_clutter_masks_checker_edges():
    """An edge search crossing a predicted checker transition must not lock
    onto it; a body edge away from any transition still resolves."""
    import cv2
    spec = BoardSpec()
    cam = _camera("c", 0, dist=0.6)
    clutter = make_board_clutter(spec, cam)

    # The board occupies world z=0; find a pixel whose ray hits it.
    center = np.array([spec.squares_x, spec.squares_y, 0.0]) \
        * spec.effective_square_m / 2.0
    px0 = cam.project(center)[0]
    b = clutter(np.array([[px0, px0 + np.array([200.0, 0.0])]])[0])
    assert set(np.unique(b)).issubset({-1.0, 0.0, 1.0})

    # Synthetic image: a strong intensity step exactly at a predicted
    # checker transition. Without the mask the search would report it.
    line = np.array([px0 + np.array([s, 0.0]) for s in range(-20, 21)])
    binten = clutter(line[None, :, :])[0]
    trans = np.flatnonzero(np.abs(np.diff(binten)) > 0.15)
    if not len(trans):
        pytest.skip("no checker transition on this line (board layout)")
    s_tr = trans[0] - 20
    img = np.full((960, 1280), 100, np.uint8)
    xcut = int(round(px0[0] + s_tr))
    img[:, xcut:] = 220
    img = cv2.GaussianBlur(img, (0, 0), 0.8)
    uv = px0[None, :].astype(float)
    n2d = np.array([[1.0, 0.0]])
    _, ok_masked = edge_search(img, uv, n2d, AnchorDetectConfig(),
                               clutter=clutter)
    _, ok_plain = edge_search(img, uv, n2d, AnchorDetectConfig())
    assert ok_plain[0] and not ok_masked[0]


# ---- plate -------------------------------------------------------------------

def test_plate_spec_roundtrip_and_dimples(tmp_path):
    spec = PlateSpec(measured_pitch_m=0.01612)
    path = str(tmp_path / "plate.yaml")
    spec.save(path)
    back = PlateSpec.load(path)
    assert back.effective_pitch_m == pytest.approx(0.01612)
    assert back.tag_ids == spec.tag_ids
    d = dimple_plate(back, 2, 3)
    assert np.allclose(d, [2 * 0.01612, 3 * 0.01612, 0.0])
    with pytest.raises(ValueError):
        dimple_plate(back, 9, 0)


def _render_plate(cam, spec, R_pw, t_pw, px_per_m=8000):
    """Warp a rasterized plate face (white + tags) into the camera view."""
    import cv2
    xs = [c[0] for c in spec.tag_centers] + [0.0, (spec.cols - 1) * spec.pitch_m]
    ys = [c[1] for c in spec.tag_centers] + [0.0, (spec.rows - 1) * spec.pitch_m]
    s = spec.effective_tag_m
    x0, y0 = min(xs) - s, min(ys) - s
    x1, y1 = max(xs) + s, max(ys) + s
    tex = np.full((int((y1 - y0) * px_per_m), int((x1 - x0) * px_per_m)),
                  255, np.uint8)
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.dictionary))
    for tid, (cx, cy) in zip(spec.tag_ids, spec.tag_centers):
        mark = cv2.aruco.generateImageMarker(dic, tid, 120)
        half = spec.effective_tag_m / 2
        # texture rows run +y DOWN: marker top (toward +y) at the lower row idx
        r0 = int((y1 - (cy + half)) * px_per_m)
        c0 = int((cx - half - x0) * px_per_m)
        n = int(2 * half * px_per_m)
        tex[r0:r0 + n, c0:c0 + n] = cv2.resize(mark, (n, n),
                                               interpolation=cv2.INTER_NEAREST)
    # Homography: plate (x, y) meters -> texture pixels.
    src = np.array([[x0, y1], [x1, y1], [x1, y0], [x0, y0]], np.float32)
    dst = np.array([[0, 0], [tex.shape[1], 0],
                    [tex.shape[1], tex.shape[0]], [0, tex.shape[0]]], np.float32)
    H_tex = cv2.getPerspectiveTransform(src[:, :2], dst)
    R_pc = cam.R @ R_pw
    t_pc = cam.R @ t_pw + cam.t
    H_pose = cam.K @ np.column_stack([R_pc[:, 0], R_pc[:, 1], t_pc])
    H = H_pose @ np.linalg.inv(H_tex)
    w, h = cam.image_size
    img = cv2.warpPerspective(tex, H, (w, h), flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=60)
    return cv2.GaussianBlur(img, (0, 0), 0.6)


def test_plate_pose_recovers_truth(tmp_path):
    spec = PlateSpec()
    cams = {"cam0": _camera("cam0", -25, dist=0.45),
            "cam1": _camera("cam1", 30, dist=0.5)}
    rig = Rig(cameras=cams, board=BoardSpec())
    # Plate face normal toward the cameras, with a modest tilt.
    zdir = sum(c.center for c in cams.values())
    zdir /= np.linalg.norm(zdir)
    x = np.cross([0.0, 0.0, 1.0], zdir)
    x /= np.linalg.norm(x)
    R_pw = _rot(zdir, 15.0) @ np.column_stack([x, np.cross(zdir, x), zdir])
    t_pw = np.array([-0.03, 0.02, -0.02])
    images = {n: _render_plate(c, spec, R_pw, t_pw) for n, c in cams.items()}

    got = plate_pose(images["cam0"], spec, cams["cam0"].K, cams["cam0"].dist)
    assert got is not None and got[2] == len(spec.tag_ids)

    R, t, stats = plate_pose_world(images, rig, spec)
    rot_err = np.degrees(np.arccos(np.clip((np.trace(R_pw.T @ R) - 1) / 2, -1, 1)))
    # Small planar tags carry the usual tilt ambiguity (~0.5 deg at this
    # range); the dimple coordinates it induces stay sub-mm at grid scale.
    assert rot_err < 1.2
    assert np.linalg.norm(t - t_pw) < 0.002
    assert stats["n_views"] == 2
    d = dimple_world(R, t, spec, 1, 2)
    d_true = R_pw @ dimple_plate(spec, 1, 2) + t_pw
    assert np.linalg.norm(d - d_true) < 0.002


def test_plate_scad_has_all_dimples(tmp_path):
    spec = PlateSpec()
    path = str(tmp_path / "plate.scad")
    write_scad(spec, path)
    text = open(path).read()
    assert text.count("cylinder(") == spec.rows * spec.cols
    assert text.count("cube([") == 1 + len(spec.tag_centers)


# ---- plate holdout -----------------------------------------------------------

def test_plate_holdout_split_deterministic():
    obs = [(i, "index", np.zeros(3)) for i in range(10)]
    ds = Dataset(m=np.zeros((10, 1)), dot_obs={}, dir_obs={}, frame_axes={},
                 dir_frames=np.array([], int), plate_obs=obs,
                 sweep_frames={}, joints=["wrist"])
    ds1, held1 = split_plate_holdout(ds, 0.3)
    ds2, held2 = split_plate_holdout(ds, 0.3)
    assert len(held1) == 3 and len(ds1.plate_obs) == 7
    assert [h[0] for h in held1] == [h[0] for h in held2]
    ds3, held3 = split_plate_holdout(ds, 0.0)
    assert held3 == [] and len(ds3.plate_obs) == 10


# ---- session merge -----------------------------------------------------------

def _mini_session(F=6, links=("index_mcp",), n_anchor=2, with_contact=False,
                  with_plate=False, sweep_joint="index_mcp"):
    joints = ["wrist", "index_mcp"]
    rng = np.random.default_rng(F)
    ds = Dataset(
        m=rng.normal(0, 10, (F, len(joints))),
        dot_obs={l: rng.normal(0, 0.05, (F, 2, 3)) for l in links},
        dir_obs={l: rng.normal(0, 1, (n_anchor, 3)) for l in links},
        frame_axes={"fixed0": rng.normal(0, 1, (2, n_anchor, 3))},
        dir_frames=np.arange(F - n_anchor, F),
        plate_obs=[(0, "index", np.zeros(3))] if with_plate else [],
        sweep_frames={sweep_joint: np.arange(0, F - n_anchor)},
        joints=joints,
        contact_obs=[ContactEvent(frame=1, kind="abd_block", body_a="a",
                                  body_b="b")] if with_contact else [],
        dir_sigma={l: np.full(n_anchor, 0.5) for l in links},
    )
    return ds


def test_merge_offsets_and_block_structure():
    a = _mini_session(F=6)
    b = _mini_session(F=8, links=("index_mcp", "wrist"), with_contact=True)
    merged = merge_datasets([a, b], same_mount=True)
    assert merged.m.shape == (14, 2)
    # Dot columns are block-diagonal: session a's columns NaN in b's frames.
    arr = merged.dot_obs["index_mcp"]
    assert arr.shape == (14, 4, 3)
    assert np.isfinite(arr[:6, :2]).all() and np.isnan(arr[6:, :2]).all()
    assert np.isnan(arr[:6, 2:]).all() and np.isfinite(arr[6:, 2:]).all()
    # wrist link only in b -> NaN rows in a's frames.
    assert np.isnan(merged.dot_obs["wrist"][:6]).all()
    # Sweeps of the same joint concatenate with offsets.
    assert list(merged.sweep_frames["index_mcp"]) == list(range(0, 4)) + \
        [6 + i for i in range(0, 6)]
    # Anchor rows concatenate; dir sigma rides along.
    assert merged.dir_obs["index_mcp"].shape == (4, 3)
    assert merged.dir_sigma["index_mcp"].shape == (4,)
    assert list(merged.dir_frames) == [4, 5, 12, 13]
    # Contact frames offset into the merged index space.
    assert merged.contact_obs[0].frame == 7


def test_merge_guards(tmp_path):
    a = _mini_session(F=5)
    b = _mini_session(F=5)
    with pytest.raises(ValueError, match="same-mount|same_mount"):
        merge_datasets([a, b])
    contact_only = Dataset(
        m=np.zeros((3, 2)), dot_obs={}, dir_obs={}, frame_axes={},
        dir_frames=np.array([], int), plate_obs=[], sweep_frames={},
        joints=["wrist", "index_mcp"],
        contact_obs=[ContactEvent(frame=0, kind="abd_block",
                                  body_a="a", body_b="b")])
    merged = merge_datasets([a, contact_only])   # 1 world-frame session: fine
    assert merged.contact_obs[0].frame == 5
    bad = _mini_session(F=4)
    bad.joints = ["wrist"]
    bad.m = bad.m[:, :1]
    with pytest.raises(ValueError, match="joint lists differ"):
        merge_datasets([a, bad], same_mount=True)
    # Merged sessions round-trip through the on-disk format.
    merged2 = merge_datasets([a, b], same_mount=True)
    save_session(merged2, str(tmp_path / "s"))
    back = load_session(str(tmp_path / "s"))
    np.testing.assert_array_equal(back.m, merged2.m)
    assert back.dir_sigma["index_mcp"].shape == (4,)


# ---- anchor pose sampler -----------------------------------------------------

def test_anchor_pose_plan_kinds_and_ranges():
    joints = list(JOINT_ROMS)
    roms = {j: JOINT_ROMS[j] for j in joints}
    held = dict(HELD_POSE)
    rng = np.random.default_rng(3)
    q, kinds = anchor_pose_plan(joints, roms, held, 10, rng)
    assert q.shape == (10, len(joints))
    assert kinds.count("mid") == 5 and kinds.count("extended") == 5
    jidx = {j: i for i, j in enumerate(joints)}
    for row, kind in zip(q, kinds):
        for j in joints:
            lo, hi = roms[j]
            frac = (row[jidx[j]] - lo) / (hi - lo)
            assert -0.01 <= frac <= 1.01
            if kind == "mid":
                assert 0.28 <= frac <= 0.72
            elif j.endswith(("_mcp", "_pip", "_dip", "_cmc")):
                assert frac <= 0.28          # extended: flexion near ROM start
            elif j.endswith("_abd"):
                # pushed wide on the side the held value leans toward
                mid = 0.5 * (lo + hi)
                if held[j] < mid:
                    assert frac <= 0.28
                else:
                    assert frac >= 0.72
