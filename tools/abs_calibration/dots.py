# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Dot pipeline: blob detection, per-camera tracking, cross-camera
association, triangulation.

Dots are uncoded, so identity comes from time: the rig captures at rest per
sweep step, steps are small, and a constant-velocity nearest-neighbour
tracker with Hungarian assignment carries identity through each camera.
Across cameras, whole 2D tracks are matched by their median epipolar
(Sampson) distance over shared frames — track-level matching is far more
robust than per-frame point matching because a single track accumulates
hundreds of frames of evidence. An occluded dot that reappears starts a new
track; components may therefore hold several temporally disjoint tracks of
one camera, and a dot lost to all cameras for a while simply continues as a
separate 3D track (the estimator's per-dot masking absorbs the split).
"""
from __future__ import annotations

import warnings
from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

from cameras import Rig, fundamental_matrix, sampson_distance, triangulate


@dataclass
class DetectorConfig:
    blob_color: int = 255          # 255: bright dots on dark print; 0: inverse
    min_area_px: float = 20.0
    max_area_px: float = 800.0     # a blurred square passes circularity; area
                                   # is what rejects board squares and glare
    min_circularity: float = 0.70
    min_threshold: float = 40.0
    max_threshold: float = 240.0
    threshold_step: float = 8.0
    min_dist_px: float = 4.0


def detect_dots(gray: np.ndarray, cfg: DetectorConfig) -> np.ndarray:
    """Blob centroids (N, 2) in one grayscale image."""
    p = cv2.SimpleBlobDetector_Params()
    p.blobColor = cfg.blob_color
    p.filterByColor = True
    p.filterByArea = True
    p.minArea = cfg.min_area_px
    p.maxArea = cfg.max_area_px
    p.filterByCircularity = True
    p.minCircularity = cfg.min_circularity
    p.filterByConvexity = False
    p.filterByInertia = False
    p.minThreshold = cfg.min_threshold
    p.maxThreshold = cfg.max_threshold
    p.thresholdStep = cfg.threshold_step
    p.minDistBetweenBlobs = cfg.min_dist_px
    kps = cv2.SimpleBlobDetector_create(p).detect(gray)
    if not kps:
        return np.empty((0, 2))
    return np.array([kp.pt for kp in kps], float)


def track_2d(detections: list[np.ndarray], max_step_px: float = 12.0,
             max_gap: int = 3, min_len: int = 5,
             reacquire_px: float = 8.0) -> np.ndarray:
    """Frame-wise detections -> tracks (T, F, 2) with NaN where unobserved.

    Constant-velocity prediction + Hungarian assignment, gated at
    ``max_step_px``. After a missed frame a track may only re-acquire
    within the tight ``reacquire_px`` gate: adopting a neighbour during an
    occlusion is how identity swaps happen, and a swap is far worse than a
    fragment (association reunites fragments; nothing un-mixes a swap).
    A track unmatched for more than ``max_gap`` frames closes; a
    reappearing dot starts a new track.
    """
    F = len(detections)
    open_tracks: list[dict] = []   # {points: {f: (2,)}, last_f}
    closed: list[dict] = []

    for f, dets in enumerate(detections):
        dets = np.asarray(dets, float).reshape(-1, 2)
        still_open = [tr for tr in open_tracks if f - tr["last_f"] <= max_gap]
        closed.extend(tr for tr in open_tracks if f - tr["last_f"] > max_gap)
        open_tracks = still_open

        matched_det = np.zeros(len(dets), bool)
        if open_tracks and len(dets):
            preds = np.array([_predict(tr, f) for tr in open_tracks])
            gates = np.array([
                max_step_px if tr["last_f"] == f - 1 else reacquire_px
                for tr in open_tracks
            ])
            cost = np.linalg.norm(preds[:, None, :] - dets[None, :, :], axis=2)
            cost = np.where(cost > gates[:, None], 1e6, cost)
            rows, cols = linear_sum_assignment(cost)
            for r, c in zip(rows, cols):
                if cost[r, c] < 1e6:
                    open_tracks[r]["points"][f] = dets[c]
                    open_tracks[r]["last_f"] = f
                    matched_det[c] = True
        for c in np.flatnonzero(~matched_det):
            open_tracks.append({"points": {f: dets[c]}, "last_f": f})

    closed.extend(open_tracks)
    keep = [tr for tr in closed if len(tr["points"]) >= min_len]
    out = np.full((len(keep), F, 2), np.nan)
    for i, tr in enumerate(keep):
        for f, pt in tr["points"].items():
            out[i, f] = pt
    return out


def _predict(tr: dict, f: int) -> np.ndarray:
    frames = sorted(tr["points"])
    last = tr["points"][frames[-1]]
    if len(frames) < 2:
        return last
    prev = tr["points"][frames[-2]]
    v = (last - prev) / (frames[-1] - frames[-2])
    return last + v * (f - frames[-1])


def associate_tracks(tracks: dict[str, np.ndarray], rig: Rig,
                     gate_px: float = 3.0, min_shared: int = 8,
                     overlap_tol: int = 2, merge_3d_gate_m: float = 0.003,
                     merge_3d_vel_m: float = 0.004,
                     merge_3d_max_gap: int = 6) -> list[dict[str, list[int]]]:
    """Group per-camera 2D tracks into physical-dot components.

    Edges: pairs of tracks from different cameras whose 90th-percentile
    Sampson distance over shared frames is under ``gate_px`` (on undistorted
    pixel coordinates) — the high percentile keeps a track that mixes two
    identities from passing on its majority-correct frames. Components are
    grown best-edge-first under two vetoes:

    - a camera may not observe the same dot twice at the same time (beyond
      ``overlap_tol`` frames), which keeps distinct dots on a shared
      epipolar plane apart;
    - each component carries a sparse 3D anchor track (triangulated from
      its accepted pairs), and a merge must be 3D-continuous with it —
      epipolar consistency alone cannot tell two dots apart when one is
      occluded, but their 3D positions a few frames apart can.
    """
    und = {
        cam: np.stack([rig.cameras[cam].undistort_px(tr) for tr in trs])
        if len(trs) else trs
        for cam, trs in tracks.items()
    }
    cams = sorted(tracks)
    edges = []
    for i, ca in enumerate(cams):
        for cb in cams[i + 1:]:
            if not (len(und[ca]) and len(und[cb])):
                continue
            Fm = fundamental_matrix(rig.cameras[ca], rig.cameras[cb])
            fin_a = np.isfinite(und[ca][:, :, 0])
            fin_b = np.isfinite(und[cb][:, :, 0])
            for ta in range(len(und[ca])):
                for tb in range(len(und[cb])):
                    shared = fin_a[ta] & fin_b[tb]
                    if shared.sum() < min_shared:
                        continue
                    d = sampson_distance(
                        Fm, und[ca][ta][shared], und[cb][tb][shared])
                    score = float(np.percentile(d, 90))
                    if score < gate_px:
                        edges.append((score, (ca, ta), (cb, tb)))

    edges.sort(key=lambda e: e[0])
    parent: dict[tuple, tuple] = {}
    occupancy: dict[tuple, dict[str, np.ndarray]] = {}
    anchors3d: dict[tuple, dict[int, np.ndarray]] = {}

    def find(n):
        while parent.get(n, n) != n:
            parent[n] = parent.get(parent[n], parent[n])
            n = parent[n]
        return n

    def occ(n):
        if n not in occupancy:
            cam, idx = n
            occupancy[n] = {cam: np.isfinite(und[cam][idx][:, 0])}
        return occupancy[n]

    def pair_3d(a, b):
        """Sparse 3D samples of an accepted pair over its shared frames."""
        (ca, ta), (cb, tb) = a, b
        shared = np.flatnonzero(np.isfinite(tracks[ca][ta, :, 0])
                                & np.isfinite(tracks[cb][tb, :, 0]))
        out = {}
        for f in shared[np.linspace(0, len(shared) - 1,
                                    min(len(shared), 15)).astype(int)]:
            X, rms = triangulate({ca: tracks[ca][ta, f], cb: tracks[cb][tb, f]},
                                 rig.cameras)
            if np.isfinite(rms):
                out[int(f)] = X
        return out

    def coherent_3d(pa: dict, pb: dict) -> bool:
        common = sorted(set(pa) & set(pb))
        if common:
            d = np.median([np.linalg.norm(pa[f] - pb[f]) for f in common])
            return d < merge_3d_gate_m
        fa, fb = np.asarray(sorted(pa)), np.asarray(sorted(pb))
        gaps = np.abs(fa[:, None] - fb[None, :])
        i, j = np.unravel_index(np.argmin(gaps), gaps.shape)
        gap = int(gaps[i, j])
        if gap > merge_3d_max_gap:
            return True  # far apart in time: 3D says nothing either way
        d = np.linalg.norm(pa[int(fa[i])] - pb[int(fb[j])])
        return d < merge_3d_gate_m + merge_3d_vel_m * gap

    for _, a, b in edges:
        ra, rb = find(a), find(b)
        if ra == rb:
            continue
        oa, ob = occ(ra), occ(rb)
        if any((oa[c] & ob[c]).sum() > overlap_tol for c in oa if c in ob):
            continue
        p_edge = pair_3d(a, b)
        if not p_edge:
            continue
        pa, pb = anchors3d.get(ra, {}), anchors3d.get(rb, {})
        if not (coherent_3d(pa, p_edge) if pa else True):
            continue
        if not (coherent_3d(pb, p_edge) if pb else True):
            continue
        if pa and pb and not coherent_3d(pa, pb):
            continue
        parent[rb] = ra
        merged = dict(oa)
        for c, mask in ob.items():
            merged[c] = merged[c] | mask if c in merged else mask
        occupancy[ra] = merged
        anchors3d[ra] = {**pb, **pa, **p_edge}
        anchors3d.pop(rb, None)

    groups: dict[tuple, dict[str, list[int]]] = {}
    for ca in cams:
        for ta in range(len(und[ca])):
            root = find((ca, ta))
            groups.setdefault(root, {}).setdefault(ca, []).append(ta)
    return [g for g in groups.values() if len(g) >= 2]


def triangulate_components(components: list[dict[str, list[int]]],
                           tracks: dict[str, np.ndarray], rig: Rig, n_frames: int,
                           reproj_gate_px: float = 3.0) -> tuple[np.ndarray, dict]:
    """3D dot tracks (F, K, 3) in the world (board) frame, NaN = unobserved.

    Per frame and component, every camera holding a finite track point
    contributes a view; points needing >= 2 views triangulate, and frames
    whose reprojection rms exceeds the gate are dropped rather than written.
    """
    K = len(components)
    pts = np.full((n_frames, K, 3), np.nan)
    rms_all, rejected, salvaged = [], 0, 0
    for k, comp in enumerate(components):
        for f in range(n_frames):
            obs = {}
            for cam, idxs in comp.items():
                for ti in idxs:
                    if np.isfinite(tracks[cam][ti, f]).all():
                        obs[cam] = tracks[cam][ti, f]
                        break
            if len(obs) < 2:
                continue
            X, rms = triangulate(obs, rig.cameras)
            if not np.isfinite(rms) or rms > reproj_gate_px:
                # With >= 4 views the consensus can identify one bad camera
                # (a track that swapped identity) instead of losing the
                # frame; at 3 views the survivor would be an unverifiable
                # pair, which is how depth-wrong points sneak in.
                if len(obs) >= 4:
                    best = None
                    for skip in obs:
                        sub = {c: p for c, p in obs.items() if c != skip}
                        Xs, rs = triangulate(sub, rig.cameras)
                        if np.isfinite(rs) and (best is None or rs < best[1]):
                            best = (Xs, rs)
                    if best is not None and best[1] <= reproj_gate_px:
                        X, rms = best
                        salvaged += 1
                    else:
                        rejected += 1
                        continue
                else:
                    rejected += 1
                    continue
            pts[f, k] = X
            rms_all.append(rms)
    n_spikes = _despike_tracks(pts)
    stats = {
        "n_components": K,
        "n_points": int(np.isfinite(pts[:, :, 0]).sum()),
        "n_rejected": rejected,
        "n_salvaged": salvaged,
        "n_despiked": n_spikes,
        "mean_reproj_px": float(np.mean(rms_all)) if rms_all else float("nan"),
    }
    return pts, stats


def merge_components(components: list[dict[str, list[int]]],
                     tracks: dict[str, np.ndarray], pts: np.ndarray,
                     gate_m: float = 0.0015, min_shared: int = 8,
                     overlap_tol: int = 2) -> list[dict[str, list[int]]]:
    """Merge components that are the same physical dot, by 3D agreement.

    Association under-merges by design (its vetoes prefer a split to a
    chimera), leaving one dot as several components seen by different
    camera subsets. Merging matters twice: duplicate columns vote for each
    other in the rigidity filter (they are copies, not independent peers),
    and a merged component sees >= 3 views where the splits saw 2, which is
    what lets the reprojection consensus actually reject bad pairings. The
    same-camera exclusivity veto still applies: two tracks of one camera
    disagreeing at the same instant are not the same dot.
    """
    fin = np.isfinite(pts[:, :, 0])
    parent = list(range(len(components)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def occupancy(comp):
        occ = {}
        for cam, idxs in comp.items():
            mask = np.zeros(pts.shape[0], bool)
            for ti in idxs:
                mask |= np.isfinite(tracks[cam][ti, :, 0])
            occ[cam] = mask
        return occ

    occ = [occupancy(c) for c in components]
    for i in range(len(components)):
        for j in range(i + 1, len(components)):
            shared = fin[:, i] & fin[:, j]
            if shared.sum() < min_shared:
                continue
            d = np.linalg.norm(pts[shared, i] - pts[shared, j], axis=1)
            if np.median(d) > gate_m:
                continue
            ri, rj = find(i), find(j)
            if ri == rj:
                continue
            oi, oj = occ[ri], occ[rj]
            if any((oi[c] & oj[c]).sum() > overlap_tol for c in oi if c in oj):
                continue
            parent[rj] = ri
            merged = dict(oi)
            for c, m in oj.items():
                merged[c] = merged[c] | m if c in merged else m
            occ[ri] = merged

    groups: dict[int, dict[str, list[int]]] = {}
    for i, comp in enumerate(components):
        g = groups.setdefault(find(i), {})
        for cam, idxs in comp.items():
            g.setdefault(cam, []).extend(idxs)
    return list(groups.values())


def rigidity_filter(dot_obs: dict[str, np.ndarray],
                    sweep_frames: dict[str, np.ndarray],
                    signatures: dict[str, frozenset],
                    tol_m: float = 0.003, min_pair_obs: int = 8) -> int:
    """NaN out per-frame points that break sweep-epoch rigidity, in place.

    An identity swap that survives triangulation (image-proximity adoptions
    produce smooth, epipolar-consistent runs at the wrong depth) still
    shifts the point by tens of millimetres in 3D. During a single joint's
    sweep the hand splits into exactly two rigid bodies — everything the
    joint moves and everything it doesn't — so within each side, pairwise
    distances between ALL dots (across links) are constant over that sweep.
    That pools far more voting peers than per-link rigidity: even a link
    carrying one dot is checked against the whole rigid group.

    Two stages, both peer-voting (a point or chunk contradicted by more
    peers than agree with it is dropped; a bare deviating pair with no
    majority drops both sides):

    1. per frame within a sweep epoch — catches transient mismatches;
    2. per (column, epoch) across epochs — when both cameras hand a track
       to the SAME neighbouring dot at once (self-occlusion events are
       pose-correlated across cameras), the stolen epoch is internally
       rigid-consistent but its pair medians disagree with the column's
       other epochs.
    """
    links = sorted(dot_obs)
    cols = np.concatenate([dot_obs[l] for l in links], axis=1)  # (F, Kall, 3)
    col_link = [l for l in links for _ in range(dot_obs[l].shape[1])]
    Kall = cols.shape[1]
    drop_all = np.zeros(cols.shape[:2], bool)
    epoch_meds: dict[tuple[int, int], dict[str, float]] = {}

    for joint, frames in sweep_frames.items():
        frames = np.asarray(frames, int)
        for moving in (True, False):
            group = [k for k, l in enumerate(col_link)
                     if (joint in signatures.get(l, frozenset())) == moving]
            if len(group) < 2:
                continue
            arr = cols[frames][:, group, :]
            d = np.linalg.norm(arr[:, :, None, :] - arr[:, None, :, :], axis=3)
            enough = np.isfinite(d).sum(axis=0) >= min_pair_obs
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN pairs
                med = np.where(enough, np.nanmedian(d, axis=0), np.nan)
            dev = np.abs(d - med[None, :, :])
            finite = np.isfinite(dev)
            bad = ((dev > tol_m) & finite).sum(axis=2)
            good = ((dev <= tol_m) & finite).sum(axis=2) - 1
            drop = (bad >= 1) & (bad > np.maximum(good, 0))
            fi, ki = np.nonzero(drop)
            drop_all[frames[fi], np.asarray(group)[ki]] = True
            for a in range(len(group)):
                for b in range(a + 1, len(group)):
                    if np.isfinite(med[a, b]):
                        epoch_meds.setdefault(
                            (group[a], group[b]), {})[joint] = float(med[a, b])

    epochs = sorted(sweep_frames)
    e_idx = {j: i for i, j in enumerate(epochs)}
    sus_votes = np.zeros((Kall, len(epochs)), int)
    ok_votes = np.zeros((Kall, len(epochs)), int)
    for (ka, kb), per_epoch in epoch_meds.items():
        if len(per_epoch) < 2:
            continue
        ref = float(np.median(list(per_epoch.values())))
        for joint, med in per_epoch.items():
            target = sus_votes if abs(med - ref) > tol_m else ok_votes
            target[ka, e_idx[joint]] += 1
            target[kb, e_idx[joint]] += 1
    for k, e in zip(*np.nonzero((sus_votes >= 1)
                                & (sus_votes > ok_votes))):
        drop_all[np.asarray(sweep_frames[epochs[e]], int), k] = True

    n = int(drop_all.sum())
    k0 = 0
    for l in links:
        k1 = k0 + dot_obs[l].shape[1]
        dot_obs[l][drop_all[:, k0:k1]] = np.nan
        k0 = k1
    return n


def _despike_tracks(pts: np.ndarray, max_dev_m: float = 0.006,
                    max_span: int = 4, max_vel_m: float = 0.008) -> int:
    """NaN out 3D points inconsistent with their temporal neighbours.

    Two-view triangulations have no redundancy — any epipolar-consistent
    wrong pairing passes the reprojection gate — but the sweeps are slow and
    eased, so a genuine track is locally linear in time: points far from
    the line through their neighbours, or jumping faster than any eased
    sweep moves a dot, are mismatches. Iterated so a removed spike exposes
    its neighbours to the same test.
    """
    n = 0
    for _ in range(3):
        removed = 0
        for k in range(pts.shape[1]):
            # Velocity gate first: it reaches run boundaries the
            # interpolation can't, and a boundary spike left in place would
            # corrupt its neighbour's interpolation below.
            fin = np.flatnonzero(np.isfinite(pts[:, k, 0]))
            if len(fin) >= 2:
                df = np.diff(fin)
                step = np.linalg.norm(np.diff(pts[fin, k, :], axis=0), axis=1)
                for i in np.flatnonzero(step / df > max_vel_m):
                    # Drop whichever endpoint is the more isolated one.
                    lone_a = (i == 0) or (fin[i] - fin[i - 1] > max_span)
                    lone_b = (i + 2 >= len(fin)) or (fin[i + 2] - fin[i + 1] > max_span)
                    if lone_b and not lone_a:
                        pts[fin[i + 1], k] = np.nan
                    elif lone_a and not lone_b:
                        pts[fin[i], k] = np.nan
                    removed += int(lone_a != lone_b)

            fin = np.flatnonzero(np.isfinite(pts[:, k, 0]))
            if len(fin) < 3:
                continue
            prev, cur, nxt = fin[:-2], fin[1:-1], fin[2:]
            ok = (nxt - prev) <= 2 * max_span
            w = (cur[ok] - prev[ok]) / (nxt[ok] - prev[ok])
            interp = (1 - w[:, None]) * pts[prev[ok], k] + w[:, None] * pts[nxt[ok], k]
            dev = np.linalg.norm(pts[cur[ok], k] - interp, axis=1)
            bad_frames = cur[ok][dev > max_dev_m]
            # A spike also inflates its neighbours' deviations (their
            # interpolation uses it); per consecutive run drop only the
            # worst point and let the next pass re-judge the rest.
            if len(bad_frames):
                dev_of = dict(zip(cur[ok], dev))
                runs = np.split(bad_frames,
                                np.flatnonzero(np.diff(bad_frames) > 1) + 1)
                for run in runs:
                    worst = run[int(np.argmax([dev_of[f] for f in run]))]
                    pts[worst, k] = np.nan
                    removed += 1
        n += removed
        if not removed:
            break
    return n
