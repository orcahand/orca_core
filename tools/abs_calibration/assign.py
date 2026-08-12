# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Assign 3D dot tracks to links by motion signature.

A dot on link L moves during a sweep of joint j exactly when j is an
ancestor of L (including L's own joint), so the set of sweeps that move a
track identifies its link uniquely — mesh-free and immune to calibration
offsets. Sweeps during which a track was occluded are treated as unknown,
and a link is accepted only if it is the *unique* link consistent with every
observed sweep. Tracks that never move under any observed sweep are the
static class: forearm dots and any background clutter, both assigned to the
forearm link where an unknown static layout is pure gauge for the estimator.
"""
from __future__ import annotations

import numpy as np

from model import JOINT_ROMS, KinematicModel

STATIC_LINK = "fixed0"  # forearm/base shell: the only fixed node in the tree
MOVE_THRESH_M = 0.004   # smallest true sweep motion ~8 mm; held-joint creep ~1 mm
MIN_SWEEP_OBS = 4       # finite frames needed to judge a track against a sweep


def link_signatures(model: KinematicModel) -> dict[str, frozenset[str]]:
    """``{link: joints whose sweep moves it}`` — ancestors plus its own joint."""
    sigs = {}
    for i, node in enumerate(model.nodes):
        joints = set()
        k = i
        while k >= 0:
            n = model.nodes[k]
            if n.axis is not None:
                joints.add(n.name)
            k = n.parent
        sigs[node.name] = frozenset(joints)
    return sigs


def assign_links(pts: np.ndarray, sweep_frames: dict[str, np.ndarray],
                 model: KinematicModel, m: np.ndarray, joints: list[str],
                 move_thresh_m: float = MOVE_THRESH_M,
                 min_sweep_obs: int = MIN_SWEEP_OBS,
                 min_span_frac: float = 0.35,
                 ) -> tuple[dict[str, list[int]], list[dict]]:
    """3D tracks (F, K, 3) -> ``({link: [track indices]}, per-track report)``.

    Motion metric per (track, sweep): 95th-percentile distance from the
    track's median position over that sweep's observed frames — robust to a
    stray triangulation while catching real motion anywhere in the sweep.
    A sweep counts as observed only if the track additionally saw a real
    fraction of the joint's travel (``min_span_frac`` of its ROM, judged
    from the encoders), and the motion threshold scales with that fraction:
    over a third of the arc, a genuinely moving dot travels a third as far.

    Only links whose complete signature was exercised by the capture's
    sweeps are identifiable; with a sweep missing (dead joint, reduced
    capture), its distal links drop out of the candidate set rather than
    being mistaken for their proximal neighbours.

    A track must match exactly one candidate, with one exception: when the
    candidates form a proximal-to-distal chain (a dot occluded during the
    distal sweeps that would tell them apart — fingers curl their dots away
    from the cameras during exactly their own sweeps), the hypotheses are
    observationally identical wherever the track was seen with the
    distinguishing joints at their held values. The track is assigned to
    the most proximal candidate and the few frames violating that
    equivalence are masked out of ``pts`` (in place). Everything else
    ambiguous is dropped, not guessed.
    """
    sigs = link_signatures(model)
    swept = set(sweep_frames)
    candidates_all = {link: sig for link, sig in sigs.items() if sig <= swept}
    jidx = {j: i for i, j in enumerate(joints)}
    held_est = np.median(m, axis=0)  # sweeps are brief; the median sits at held
    K = pts.shape[1]
    assignment: dict[str, list[int]] = {}
    report = []
    for k in range(K):
        moving: dict[str, bool] = {}
        for joint, frames in sweep_frames.items():
            frames = np.asarray(frames, int)
            fin = np.isfinite(pts[frames, k, :]).all(axis=1)
            if fin.sum() < min_sweep_obs:
                continue
            lo, hi = JOINT_ROMS[joint]
            seen = m[frames[fin], jidx[joint]]
            span_frac = (seen.max() - seen.min()) / (hi - lo)
            if span_frac < min_span_frac:
                continue
            p = pts[frames[fin], k, :]
            d = np.linalg.norm(p - np.median(p, axis=0), axis=1)
            thresh = max(0.0015, move_thresh_m * span_frac)
            moving[joint] = bool(np.percentile(d, 95) > thresh)

        if not moving:
            report.append({"track": k, "link": None, "reason": "no sweep coverage"})
            continue
        moved = {j for j, mv in moving.items() if mv}
        candidates = [
            link for link, sig in candidates_all.items()
            if all((j in sig) == mv for j, mv in moving.items())
        ]
        if len(candidates) == 1:
            link = candidates[0]
            assignment.setdefault(link, []).append(k)
            report.append({"track": k, "link": link,
                           "moved": sorted(moved), "observed": len(moving)})
            continue
        chain = sorted(candidates, key=lambda l: len(candidates_all[l]))
        if candidates and all(
            candidates_all[a] <= candidates_all[b]
            for a, b in zip(chain, chain[1:])
        ):
            link = chain[0]
            distinguishing = sorted(
                set().union(*(candidates_all[l] for l in chain[1:]))
                - candidates_all[link]
            )
            dev = np.abs(m[:, [jidx[j] for j in distinguishing]]
                         - held_est[[jidx[j] for j in distinguishing]])
            masked = (dev > 1.0).any(axis=1) & np.isfinite(pts[:, k, 0])
            pts[masked, k] = np.nan
            assignment.setdefault(link, []).append(k)
            report.append({"track": k, "link": link, "moved": sorted(moved),
                           "observed": len(moving),
                           "chain": chain, "masked_frames": int(masked.sum())})
        else:
            reason = "no link matches" if not candidates else \
                f"ambiguous: {sorted(candidates)}"
            report.append({"track": k, "link": None, "reason": reason,
                           "moved": sorted(moved), "observed": len(moving)})
    return assignment, report
