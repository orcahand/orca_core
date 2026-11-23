"""The skin/frame colour boundaries as a fit term (ablation variant `color`).

Model: orca-ui's coloured hand meshes (one GLB per link, white silicone skin `#e8eaed` and dark
printed frame `#212529`, same link frames as the fit's STLs). Per link with both materials, surface
points with their class and normal; in a view, the points facing the camera are drawn per class and
where the skin and frame drawings meet (within that link's own pixels, so other links occlude it) is
the model's colour line.
Photo: inside the hand mask (eroded, off its rim), light vs dark by Otsu; the boundaries of that
map are the photo's colour lines. The term: each photo colour-line sample's distance to the model's
colour line, capped like the edge term. Low-poly meshes: a seam is only as exact as its triangles.
"""
from __future__ import annotations

import json
import os
import re
import struct
from pathlib import Path

import numpy as np

GLB_DIR = Path(os.environ.get("STUDY_GLB_DIR", Path.home() / "orca_dev/orca_ui/orca_ui/models/hand_v2/right"))
SKIN, FRAME = "#e8eaed", "#212529"
SPACING_MM = 1.0
N_COLOR = 600
CAP_PX = 6.0
ERODE_PX = 3                 # the mask's rim is the silhouette's business
SKIN_BIAS_MM = 1.5
NO_CULL = True               # the meshes' triangle windings are not consistent: the depth test alone decides
NEAR_PX = 12                 # photo colour lines only this close to the coloured links (at the seed)
DU, DV = np.meshgrid([-1, 0, 1], [-1, 0, 1])
DU, DV = DU.ravel(), DV.ravel()


def skin_tone(bench: str | Path | None) -> str:
    """'white' (orcahand-left/right: white silicone) or 'black' (touch and full hands: Paxini's black
    skin, on a black frame: no usable skin/frame contrast, the colour term is off). From the bench
    folder's model name; STUDY_SKIN_TONE overrides."""
    if os.environ.get("STUDY_SKIN_TONE") in ("white", "black"):
        return os.environ["STUDY_SKIN_TONE"]
    name = Path(str(bench or "")).name
    return "black" if ("touch" in name or "full" in name) else "white"


def tone_of_study(study: str) -> str:
    """The skin of a study's hand: its night spec's `skin`, else from its bench folder (ser-4485: touch)."""
    from hand_study.analysis import STUDY
    spec = STUDY / "night" / f"{study}.json"
    if spec.exists():
        d = json.loads(spec.read_text())
        return d.get("skin") or skin_tone(d.get("bench"))
    hand = study if study.startswith("ser-") else None
    if hand:
        found = sorted((Path.home() / "orca_bench" / hand).glob("orcahand-*"))
        return skin_tone(found[0]) if found else "white"
    return "white"


def read_glb(path: Path) -> list[tuple[str, np.ndarray]]:
    """(material name, triangles (n, 3, 3) in metres) per primitive; node transforms are identity in these files."""
    b = path.read_bytes()
    n = struct.unpack("<I", b[12:16])[0]
    j = json.loads(b[20:20 + n])
    off = 20 + n
    blob = b[off + 8:off + 8 + struct.unpack("<I", b[off:off + 4])[0]]

    def acc(i):
        a = j["accessors"][i]
        bv = j["bufferViews"][a["bufferView"]]
        dt = {5126: "<f4", 5125: "<u4", 5123: "<u2", 5121: "u1"}[a["componentType"]]
        k = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[a["type"]]
        x = np.frombuffer(blob, dt, count=a["count"] * k, offset=bv.get("byteOffset", 0) + a.get("byteOffset", 0))
        return x.reshape(-1, k) if k > 1 else x
    out = []
    for m in j["meshes"]:
        for p in m["primitives"]:
            v = acc(p["attributes"]["POSITION"]).astype(float)
            out.append((j["materials"][p["material"]]["name"], v[acc(p["indices"]).reshape(-1, 3)]))
    return out


def link_files(glb_dir: Path = GLB_DIR) -> dict[str, Path]:
    """The fit's link name -> its GLB, from orca-ui's hand.urdf (fingertip_assembly is the fit's dp)."""
    text = (glb_dir / "hand.urdf").read_text()
    out = {}
    for name, body in re.findall(r'<link name="([^"]+)">(.*?)</link>', text, re.S):
        m = re.search(r'filename="([^"]+\.glb)"', body)
        if m:
            out[name.replace("_fingertip_assembly", "_dp")] = glb_dir / m.group(1)
    return out


def sample(tri: np.ndarray, spacing: float, rng) -> tuple[np.ndarray, np.ndarray]:
    """Points (mm) about `spacing` apart over the triangles, with their faces' unit normals."""
    tri = tri * 1000.0
    nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = 0.5 * np.linalg.norm(nrm, axis=1)
    nrm = nrm / np.maximum(2 * area[:, None], 1e-9)
    k = np.floor(area / spacing ** 2 + rng.random(len(tri))).astype(int)
    idx = np.repeat(np.arange(len(tri)), k)
    a, b = rng.random((2, len(idx)))
    flip = a + b > 1
    a[flip], b[flip] = 1 - a[flip], 1 - b[flip]
    t = tri[idx]
    pts = t[:, 0] + a[:, None] * (t[:, 1] - t[:, 0]) + b[:, None] * (t[:, 2] - t[:, 0])
    return np.vstack([pts, tri.mean(1)]), np.vstack([nrm[idx], nrm])


def clouds(model, spacing: float = SPACING_MM) -> dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Per link with skin and frame: points (mm, link frame), normals, class (1 skin, 2 frame)."""
    rng = np.random.default_rng(0)
    out = {}
    for link, path in link_files().items():
        vis = model.visuals.get(link)
        if vis is None or not path.exists() or link in ("forearm", "tower"):
            continue
        prims = read_glb(path)
        if {m for m, _ in prims} < {SKIN, FRAME}:
            continue
        P, N, C = [], [], []
        for mat, tri in prims:
            if mat not in (SKIN, FRAME):
                continue
            p, n = sample(tri, spacing, rng)
            P.append(p)
            N.append(n)
            C.append(np.full(len(p), 1 if mat == SKIN else 2, np.uint8))
        R, t = vis.origin.rotation, vis.origin.translation * 1000.0
        P, N = np.vstack(P) @ R.T + t, np.vstack(N) @ R.T
        out[link] = (P, N, np.concatenate(C))
    return out


def model_lines(fit, poses: dict, lens, lab: np.ndarray, offset, cl: dict) -> np.ndarray:
    """Where the camera-facing skin and frame of each coloured link meet, on that link's own pixels (0/1)."""
    cv2 = fit_cv2()
    h, w = lab.shape
    out = np.zeros((h, w), np.uint8)
    k = np.ones((3, 3), np.uint8)
    for link, (P, N, C) in cl.items():
        T = poses.get(link)
        if T is None:
            continue
        p = T.apply_to_points(P)
        n = N @ T.rotation.T
        front = (p[:, 2] > 1.0) if NO_CULL else (np.einsum("ij,ij->i", n, -p) > 0) & (p[:, 2] > 1.0)
        if not front.any():
            continue
        p, c = p[front], C[front]
        u, v = np.round(lens.project(p) - offset).astype(int).T
        ok = (u >= 1) & (u < w - 1) & (v >= 1) & (v < h - 1)
        if ok.sum() < 10:
            continue
        o = np.argsort(-(p[ok, 2] - SKIN_BIAS_MM * (c[ok] == 1)))   # far first, nearest written last; the skin lies on the frame
        u, v, c = u[ok][o], v[ok][o], c[ok][o]
        U = (u[:, None] + DU[None, :]).ravel()      # 3x3 splat: points ~2 px apart at the fit's scale
        V = (v[:, None] + DV[None, :]).ravel()
        u0, v0, u1, v1 = U.min(), V.min(), U.max() + 1, V.max() + 1
        cls = np.zeros((v1 - v0, u1 - u0), np.uint8)
        cls[V - v0, U - u0] = np.repeat(c, 9)
        cls = cv2.medianBlur(cls, 3)
        seam = np.zeros(cls.shape, bool)
        dh = (cls[:, 1:] != cls[:, :-1]) & (cls[:, 1:] > 0) & (cls[:, :-1] > 0)
        dv = (cls[1:, :] != cls[:-1, :]) & (cls[1:, :] > 0) & (cls[:-1, :] > 0)
        seam[:, 1:] |= dh
        seam[1:, :] |= dv
        own = lab[v0:v1, u0:u1] == fit.ids[link]
        out[v0:v1, u0:u1] |= (seam & own).astype(np.uint8)
    return out


def photo_lines(crop_bgr: np.ndarray, fg: np.ndarray, near: np.ndarray | None = None) -> np.ndarray:
    """Samples (u, v) of the light/dark boundaries inside the hand mask (crop coordinates), only where
    `near` (the coloured links at the seed, dilated) when given: the dark palm has no colour lines."""
    cv2 = fit_cv2()
    inner = cv2.erode(fg.astype(np.uint8), np.ones((2 * ERODE_PX + 1,) * 2, np.uint8)) > 0
    if inner.sum() < 100:
        return np.zeros((0, 2), int)
    gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
    t, _ = cv2.threshold(gray[inner].reshape(-1, 1), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    light = ((gray > t) & inner).astype(np.uint8)
    k = np.ones((3, 3), np.uint8)
    light = cv2.morphologyEx(cv2.morphologyEx(light, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
    b = (cv2.morphologyEx(light, cv2.MORPH_GRADIENT, k) > 0) & (cv2.erode(inner.astype(np.uint8), k) > 0)
    if near is not None:
        b &= near
    ys, xs = np.nonzero(b)
    pts = np.c_[xs, ys]
    return pts[np.linspace(0, len(pts) - 1, N_COLOR).astype(int)] if len(pts) > N_COLOR else pts


def fit_cv2():
    from visual_calibration import live_fit as lf
    return lf._cv2()


def near_mask(fit, x, angle: float, lens, shape, offset, cl: dict) -> np.ndarray:
    """The coloured links' pixels at pose x, dilated by NEAR_PX."""
    cv2 = fit_cv2()
    lab = fit.label(x, angle, lens, shape, offset)
    ids = [fit.ids[l] for l in cl if l in fit.ids]
    m = np.isin(lab, ids).astype(np.uint8)
    return cv2.dilate(m, np.ones((2 * NEAR_PX + 1,) * 2, np.uint8)) > 0
