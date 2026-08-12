# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""The sweep protocol, shared by the synthetic renderer and hardware capture.

The protocol IS the dot-tracking guarantee. Dots are uncoded, so identity
must survive on temporal continuity alone, which needs two properties from
the commanded motion:

- every sweep starts and ends at the held pose, so consecutive captured
  frames stay one small step apart across sweep boundaries too;
- each leg is cosine-eased, taking the velocity to zero at reversals so a
  constant-velocity tracker never overshoots.

Step sizes are per-joint, scaled by lever arm (the wrist swings the whole
hand; a pip only its own phalanx) so the fastest dot moves ~10 px between
frames at the Tier-0 rig geometry.

Abduction sweeps run at several flexion postures when requested (hardware
default): a single abd arc has near-parallel dot tangents and its fitted
axis is honest only to ~0.5-1 deg; two or three postures diversify the
tangents and bring the abd axis floors down to the flexion class. The
inter-posture flexion transits are eased and captured like everything else
(they are ordinary observation frames; only the abd legs enter the sweep's
frame list).
"""
from __future__ import annotations

import numpy as np


def sweep_step_deg(joint: str) -> float:
    if joint == "wrist":
        return 1.2
    if joint.endswith("_cmc"):
        return 2.0
    if joint.endswith("_abd"):
        return 2.3
    if joint.endswith("_mcp"):
        return 2.8
    return 5.0


def eased_leg(a: float, b: float, step: float) -> np.ndarray:
    """Cosine-eased motion from a to b with peak per-frame change <= step.
    Excludes the endpoint b (legs chain)."""
    n = max(int(np.ceil(abs(b - a) / step * (np.pi / 2))), 3)
    tau = np.linspace(0.0, 1.0, n, endpoint=False)
    return a + (b - a) * 0.5 * (1 - np.cos(np.pi * tau))


def triangle_path(held: float, lo: float, hi: float, step: float) -> np.ndarray:
    """held -> hi -> lo -> held, cosine-eased per leg, ending at held."""
    return np.concatenate([
        eased_leg(held, hi, step),
        eased_leg(hi, lo, step),
        eased_leg(lo, held, step),
        np.array([held]),
    ])


def _paired_mcp(abd_joint: str) -> str:
    return abd_joint.replace("_abd", "_mcp")


def build_plan(joints: list[str], roms: dict[str, tuple[float, float]],
               held: dict[str, float], *, step_scale: float = 1.0,
               margin_deg: float = 2.0,
               abd_flexion_fracs: tuple[float, ...] | None = None,
               ) -> tuple[np.ndarray, dict[str, list[int]]]:
    """Commanded pose rows for a full capture: ``(q (F, J), sweep_frames)``.

    ``sweep_frames[j]`` lists the frames of j's sweep legs (what the circle
    fit and link assignment consume); flexion-posture transit frames are
    captured but belong to no sweep. All rows start from and return to the
    held pose.
    """
    jidx = {j: i for i, j in enumerate(joints)}
    held_row = np.array([held[j] for j in joints], float)
    rows: list[np.ndarray] = []
    sweep_frames: dict[str, list[int]] = {}

    def emit(base: np.ndarray, joint: str, value: float, in_sweep: bool):
        row = base.copy()
        row[jidx[joint]] = value
        if in_sweep:
            sweep_frames[joint].append(len(rows))
        rows.append(row)

    for j in joints:
        lo, hi = roms[j]
        lo, hi = lo + margin_deg, hi - margin_deg
        step = sweep_step_deg(j) * step_scale
        sweep_frames[j] = []
        mcp = _paired_mcp(j) if j.endswith("_abd") else None
        if abd_flexion_fracs and mcp in jidx:
            m_lo, m_hi = roms[mcp]
            m_lo, m_hi = m_lo + margin_deg, m_hi - margin_deg
            m_step = sweep_step_deg(mcp) * step_scale
            m_prev = held[mcp]
            base = held_row.copy()
            for frac in abd_flexion_fracs:
                m_target = m_lo + frac * (m_hi - m_lo)
                for v in eased_leg(m_prev, m_target, m_step):
                    emit(held_row, mcp, v, in_sweep=False)
                base = held_row.copy()
                base[jidx[mcp]] = m_target
                for v in triangle_path(held[j], lo, hi, step):
                    emit(base, j, v, in_sweep=True)
                m_prev = m_target
            for v in eased_leg(m_prev, held[mcp], m_step):
                emit(held_row, mcp, v, in_sweep=False)
            rows.append(held_row.copy())
        else:
            for v in triangle_path(held[j], lo, hi, step):
                emit(held_row, j, v, in_sweep=True)

    return np.asarray(rows), sweep_frames
