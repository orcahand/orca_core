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


def _finger_of(joint: str) -> str:
    return joint.rsplit("_", 1)[0]


def build_plan(joints: list[str], roms: dict[str, tuple[float, float]],
               held: dict[str, float], *, step_scale: float = 1.0,
               margin_deg: float = 2.0,
               abd_flexion_fracs: tuple[float, ...] | None = None,
               clear_flexion: dict[str, float] | None = None,
               ) -> tuple[np.ndarray, dict[str, list[int]]]:
    """Commanded pose rows for a full capture: ``(q (F, J), sweep_frames)``.

    ``sweep_frames[j]`` lists the frames of j's sweep legs (what the circle
    fit and link assignment consume); transit frames are captured but
    belong to no sweep. All rows start from and return to the held pose.

    ``clear_flexion`` enables the one-finger-at-a-time protocol: while one
    finger's flexion joints sweep, every OTHER finger's mcp/pip park at the
    given values (e.g. ``{"mcp": 80, "pip": 85}`` — curled out of a side
    camera's line of sight), reached and left through eased transits so
    dot-tracking continuity survives. Wrist and abduction sweeps run from
    the plain held pose.
    """
    jidx = {j: i for i, j in enumerate(joints)}
    held_row = np.array([held[j] for j in joints], float)
    rows: list[np.ndarray] = []
    sweep_frames: dict[str, list[int]] = {j: [] for j in joints}

    def emit(base: np.ndarray, joint: str, value: float, in_sweep: bool):
        row = base.copy()
        row[jidx[joint]] = value
        if in_sweep:
            sweep_frames[joint].append(len(rows))
        rows.append(row)

    def rom_of(j):
        lo, hi = roms[j]
        return lo + margin_deg, hi - margin_deg

    def transit(base: np.ndarray, targets: dict[str, float]) -> np.ndarray:
        """Ease every listed joint from base to its target SIMULTANEOUSLY —
        each dot's per-frame motion stays bounded by its own joint's step,
        so tracking continuity holds while the transit stays short. Returns
        the new base."""
        legs = {}
        for j, v in targets.items():
            step = sweep_step_deg(j) * step_scale
            legs[j] = eased_leg(base[jidx[j]], v, step)
        n = max((len(l) for l in legs.values()), default=0)
        out = base.copy()
        for k in range(n):
            row = out.copy()
            for j, leg in legs.items():
                row[jidx[j]] = leg[k] if k < len(leg) else targets[j]
            rows.append(row)
        for j, v in targets.items():
            out[jidx[j]] = v
        rows.append(out.copy())
        return out

    flexion_stages = ("_mcp", "_pip", "_dip")
    fingers = []
    for j in joints:
        f = _finger_of(j)
        if j.endswith(flexion_stages) and f not in fingers:
            fingers.append(f)

    def clear_targets(active: str) -> dict[str, float]:
        t = {}
        for f in fingers:
            if f == active:
                continue
            for stage, key in (("_mcp", "mcp"), ("_pip", "pip"), ("_dip", "pip")):
                j = f + stage
                if j in jidx and key in (clear_flexion or {}):
                    lo, hi = rom_of(j)
                    t[j] = float(np.clip(clear_flexion[key], lo, hi))
        return t

    def sweep(j: str, base: np.ndarray):
        lo, hi = rom_of(j)
        step = sweep_step_deg(j) * step_scale
        for v in triangle_path(base[jidx[j]], lo, hi, step):
            emit(base, j, v, in_sweep=True)

    # Wrist, cmc, and abduction sweeps from the plain held pose.
    for j in joints:
        if j.endswith(flexion_stages):
            continue
        if j.endswith("_abd") and abd_flexion_fracs \
                and _paired_mcp(j) in jidx:
            mcp = _paired_mcp(j)
            m_lo, m_hi = rom_of(mcp)
            m_step = sweep_step_deg(mcp) * step_scale
            m_prev = held[mcp]
            for frac in abd_flexion_fracs:
                m_target = m_lo + frac * (m_hi - m_lo)
                for v in eased_leg(m_prev, m_target, m_step):
                    emit(held_row, mcp, v, in_sweep=False)
                base = held_row.copy()
                base[jidx[mcp]] = m_target
                for v in triangle_path(held[j], *rom_of(j),
                                       sweep_step_deg(j) * step_scale):
                    emit(base, j, v, in_sweep=True)
                m_prev = m_target
            for v in eased_leg(m_prev, held[mcp], m_step):
                emit(held_row, mcp, v, in_sweep=False)
            rows.append(held_row.copy())
        else:
            sweep(j, held_row)

    # Flexion sweeps, one finger at a time.
    for f in fingers:
        own = [f + s for s in flexion_stages if f + s in jidx]
        if clear_flexion:
            base = transit(held_row, clear_targets(f))
        else:
            base = held_row
        for j in own:
            sweep(j, base)
        if clear_flexion:
            transit(base, {j: held[j] for j in clear_targets(f)})

    return np.asarray(rows), sweep_frames
