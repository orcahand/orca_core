#!/usr/bin/env python3
# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Record a camera-free contact session: abd-block and tip-press events.

NOT YET RUN ON HARDWARE. Writes a solver session (dataset.py, format 2)
containing only encoder rows and contact events — no cameras anywhere.

abd-manual  (default) The abd pair contacts, FULLY hand-guided: torque
            goes off on every motor at mode start and no joint is ever
            driven — the operator arranges everything, uninvolved fingers
            simply hang wherever convenient. Postures are HARDSTOP-defined
            (operator-reproducible without motors, and coincident with the
            encoder anchor points): "extended" — both fingers straight
            against their mcp+pip extension stops, contact near the pip
            knuckle (the pinky meets the ring at its fingertip instead —
            fine, it is in the mesh) — and "flexed" — braced finger
            straight, moved finger's mcp at its flexion stop (pip free,
            its influence is measured into the manifold sigma), contact
            near the palm. Per capture: brace one finger at its abd stop
            toward the neighbour, bring the neighbour to a just-touch,
            press ENTER; the encoder stream is averaged over a short
            window, gated on stillness and posture, and checked live
            against the mesh manifold. Both encoders are READ — stops are
            braces, no hardstop angle is assumed in the residual.
            FRAME NOTE: the hand's abd convention is mirrored vs the
            kinematic model (measured 2026-08-13 on the full-right dev
            hand: pushing any finger toward the thumb INCREASES its
            encoder angle; the model has the opposite — and the loop
            tracks, so motor and encoder agree with each other and the
            whole config frame is mirrored, not faulty). Recorded abd
            angles are sign-flipped into the model frame at capture, so
            sessions solve against the mesh manifolds directly; a
            per-joint override table can be passed via --directions.

abd         The motor-driven variant: one finger parks at its abduction
            stop with normal current, the neighbour approaches in small
            steps under a LOW current limit; first contact is the onset of
            tracking error with dying velocity (contact_detect), captured
            and backed off before force builds. Pairwise in both
            directions and at two flexion postures. Solving either abd
            class needs the mesh contact manifolds
            (build_contact_manifold.py).

tip-press   Torque off (the hand goes limp; encoders stay live). The taxel
            stream is started and the pads are zeroed at mode start (keep
            fingertips untouched for a moment). The operator presses the
            prompted fingertip pair together by hand; when both pads report
            a localisable resultant and the PAIR's joints are still (other
            fingers may tremble), the event auto-captures (contact centroid
            per pad in its sensor frame + forces). A live readout shows
            both forces and the hold progress; ENTER skips the current
            pair, q ends the mode. Both light and firm presses are wanted
            (sigma(F) is modelled). Full hands only.

The wrist is constrained by NEITHER class (no self-contact crosses the
wrist) and keeps its hardstop calibration; the solver's sigma gate reports
this honestly.

Approach directions are derived from the packaged kinematic model per
pair (motor/encoder orientation is already baked into the joint
convention); the dry-run prints them for a one-time eyeball check against
what the hardstop calibration does on your hand.

Usage:
    uv run python tools/abs_calibration/record_contacts.py CONFIG_PATH \
        --out SESSION_DIR [--modes abd-manual,tip] [--dry-run]
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import yaml

from contact_detect import StallDetector, TipPressDetector, taxel_centroid
from dataset import ContactEvent, Dataset, save_session

ROM_MARGIN_DEG = 1.0
PARK = {"thumb_abd": 0.0, "thumb_mcp": -20.0, "thumb_cmc": -35.0}

# Adjacent abd pairs (parked, pusher) — both directions are visited.
ABD_PAIRS = [
    ("pinky_abd", "ring_abd"),
    ("ring_abd", "middle_abd"),
    ("middle_abd", "index_abd"),
]
# Flexion postures (deg) applied to BOTH fingers' mcp during the approach:
# contact at two postures touches different mesh regions (conditioning, and
# it averages print error over patches). Both pips are held at ABD_PIP_DEG.
# These are protocol constants shared with build_contact_manifold.py — the
# manifolds are only valid at the postures they were computed for.
ABD_POSTURES = (15.0, 40.0)
ABD_PIP_DEG = 8.0

# Approach directions are DERIVED FROM THE KINEMATICS, not assumed: for
# each (pusher, target) pair the packaged model answers "does increasing
# the pusher's abduction bring its fingertip toward the target finger?".
# Motor and encoder orientation are already baked into the joint-angle
# convention the model shares, so this holds for either hand side. The
# dry-run prints every derived direction for a one-time eyeball check.

# Fingertip pads all face palmward: only the opposing thumb can meet a
# finger pad-to-pad, so the thumb is every pair's hub (finger-to-finger
# information flows through the shared thumb chain).
TIP_PAIRS = [
    ("thumb", "index"), ("thumb", "middle"), ("thumb", "ring"),
    ("thumb", "pinky"),
]
TIP_EVENTS_PER_PAIR = 6

# Manual abd capture: encoder averaging window and acceptance gates.
MANUAL_SAMPLES = 10
MANUAL_INTERVAL_S = 0.08
MANUAL_STILL_TOL_DEG = 0.25   # max per-joint std over the window
POSTURE_TOL_DEG = 4.0         # inside the hardstop manifolds' 5.0 deg gate
# A finger braced at its physical stop can read just past the manifold's
# fitted grid edge; the estimator admits the same slack.
ARG_RANGE_SLACK_DEG = 1.5
# Hardstop posture variants: both fingers straight, or the moved finger's
# mcp at its flexion stop (contact near the palm; its pip stays free).
MANUAL_VARIANTS = ("extended", "flexed")

FINGER_ORDER = ("index", "middle", "ring", "pinky")  # thumb side first

# Measured abd convention of the hand's encoder/config frame (full-right
# dev hand, 2026-08-13): pushing any finger toward the thumb INCREASES its
# encoder angle, uniformly — the packaged model has the opposite, and the
# joint loop tracks, so the frame is coherent but mirrored vs the model.
# Recorded abd angles are flipped into the model frame with
# ``abd_frame_flips``; --directions overrides this table per joint.
ENCODER_TOWARD_THUMB_SIGN = 1.0


def _finger(joint: str) -> str:
    return joint.rsplit("_", 1)[0]


def compute_approach_signs(side: str, held: dict) -> dict:
    """``{(joint_from, joint_to): +-1}`` — the sign of ``joint_from``
    motion that brings its fingertip toward ``joint_to``'s finger,
    answered by the packaged kinematic model at the held pose."""
    from model import DISTAL_LINK, KinematicModel
    kin = KinematicModel.load(side)

    def tip(link, q):
        poses = kin.link_poses({j: np.array([v]) for j, v in q.items()})
        return poses[link][1][0]

    signs = {}
    for a, b in ABD_PAIRS:
        for j_from, j_to in ((a, b), (b, a)):
            lf, lt = DISTAL_LINK[_finger(j_from)], DISTAL_LINK[_finger(j_to)]
            base = dict(held)
            d = {}
            for s in (+1.0, -1.0):
                q = dict(base)
                q[j_from] = base.get(j_from, 0.0) + s * 5.0
                d[s] = np.linalg.norm(tip(lf, q) - tip(lt, q))
            signs[(j_from, j_to)] = 1.0 if d[1.0] < d[-1.0] else -1.0
    return signs


def compute_toward_thumb_signs(side: str, held: dict) -> dict:
    """``{abd_joint: +-1}`` — the abd direction that brings each fingertip
    toward the thumb, answered by the packaged kinematic model.

    This is the MODEL frame's convention. The hand's encoder/config frame
    is mirrored on the abds (see ENCODER_TOWARD_THUMB_SIGN); recorded
    angles are flipped into the model frame via ``abd_frame_flips``, so
    everything downstream of capture — gates, diagnostics, the solver —
    speaks the model convention.
    """
    from model import DISTAL_LINK, KinematicModel
    kin = KinematicModel.load(side)

    def tip(link, q):
        poses = kin.link_poses({j: np.array([v]) for j, v in q.items()})
        return poses[link][1][0]

    signs = {}
    for f in FINGER_ORDER:
        j = f + "_abd"
        d = {}
        for s in (+1.0, -1.0):
            q = dict(held)
            q[j] = held.get(j, 0.0) + s * 5.0
            d[s] = np.linalg.norm(tip(DISTAL_LINK[f], q)
                                  - tip(DISTAL_LINK["thumb"], q))
        signs[j] = 1.0 if d[1.0] < d[-1.0] else -1.0
    return signs


def derive_pair_signs(thumb_signs: dict) -> dict:
    """Adjacent-pair approach signs from per-joint toward-thumb signs:
    in every ABD_PAIRS tuple the second joint is the thumb-side one, so
    the first reaches it moving toward the thumb and vice versa."""
    signs = {}
    for a, b in ABD_PAIRS:
        signs[(a, b)] = thumb_signs[a]
        signs[(b, a)] = -thumb_signs[b]
    return signs


def abd_frame_flips(model_thumb_signs: dict, config_roms: dict,
                    measured_signs: dict | None = None) -> dict:
    """``{abd_joint: (sign, offset_deg)}`` mapping recorded abd angles into
    the MODEL frame: ``m_model ≈ sign·m + offset``. A joint flips when the
    hand's measured toward-thumb direction differs from the model's; the
    offset aligns the mirrored config ROM onto the model's — nonzero only
    for asymmetric ROMs (index: −5), confirmed by both stops mapping to
    the same value. Remaining per-joint error is the solver's b0."""
    from model import JOINT_ROMS
    out = {}
    for j, ms in model_thumb_signs.items():
        s = float(ms * (measured_signs or {}).get(j,
                                                  ENCODER_TOWARD_THUMB_SIGN))
        k = float(JOINT_ROMS[j][0] + config_roms[j][1]) if s < 0 else 0.0
        out[j] = (s, k)
    return out


def apply_frame(mean: dict, flips: dict) -> dict:
    """Recorded angles into the model frame per ``abd_frame_flips``."""
    out = dict(mean)
    for j, (s, k) in flips.items():
        if j in out:
            out[j] = out[j] * s + k
    return out


def _reconnect(hand) -> bool:
    """Offer recovery after the encoder link died mid-session; True when
    the stream is back (torque stays off — this is the manual mode)."""
    while True:
        cmd = input("   r=reconnect and retry  q=quit (events so far are "
                    "saved): ").strip().lower()
        if cmd == "q":
            return False
        if cmd != "r":
            continue
        try:
            hand.disconnect()
            ok, msg = hand.connect()
            if ok and hand.loop_joint_names is not None:
                hand.disable_torque()
                print("   reconnected — carry on")
                return True
            print(f"   reconnect failed: {msg}")
        except Exception as e:
            print(f"   reconnect failed: {e}")


def manual_expected(roms: dict, parked: str, pusher: str,
                    variant: str) -> dict:
    """Hardstop posture angles a manual capture is gated against — the
    hand's OWN rom ends (the encoder anchors sit exactly there). The
    braced finger is always straight; ``extended`` straightens the moved
    finger too, ``flexed`` puts its mcp at the flexion stop (pip free)."""
    fp, fb = _finger(parked), _finger(pusher)
    exp = {}
    for f in (fp,) if variant == "flexed" else (fp, fb):
        for jt, end in ((f + "_mcp", 0), (f + "_pip", 0)):
            if jt in roms:
                exp[jt] = float(roms[jt][end])
    if variant == "flexed" and fb + "_mcp" in roms:
        exp[fb + "_mcp"] = float(roms[fb + "_mcp"][1])
    return exp


class SessionRecorder:
    def __init__(self, joints: list[str]):
        self.joints = joints
        self.rows: list[np.ndarray] = []
        self.events: list[ContactEvent] = []

    def add(self, m_row: dict, **event_kwargs) -> None:
        frame = len(self.rows)
        self.rows.append(np.array([m_row.get(j, np.nan) for j in self.joints]))
        self.events.append(ContactEvent(frame=frame, **event_kwargs))

    def save(self, out_dir: str, meta: dict) -> None:
        ds = Dataset(
            m=np.asarray(self.rows).reshape(-1, len(self.joints)),
            dot_obs={}, dir_obs={}, frame_axes={},
            dir_frames=np.array([], int), plate_obs=[], sweep_frames={},
            joints=list(self.joints), contact_obs=self.events, meta=meta,
        )
        save_session(ds, out_dir)
        print(f"wrote {len(self.events)} contact events to {out_dir}")


def read_m(hand, samples: int = 3, interval: float = 0.03) -> dict:
    rows = []
    for _ in range(samples):
        rows.append(hand.get_measured_joints())
        time.sleep(interval)
    return {j: float(np.nanmean([r.get(j, np.nan) for r in rows]))
            for j in rows[0]}


def sample_still(hand, samples: int, interval: float) -> tuple[dict, dict]:
    """Average the encoder stream over a short window; per-joint (mean, std)."""
    rows = []
    for _ in range(samples):
        rows.append(hand.get_measured_joints())
        time.sleep(interval)
    mean, std = {}, {}
    for j in rows[0]:
        v = np.array([r.get(j, np.nan) for r in rows], float)
        v = v[np.isfinite(v)]
        mean[j] = float(v.mean()) if v.size else float("nan")
        std[j] = float(v.std()) if v.size else float("nan")
    return mean, std


def abd_pose(held: dict, roms: dict, parked: str, pusher: str,
             posture: float) -> tuple[dict, dict]:
    """Held pose with both fingers' flexion at the manifold posture.

    Returns ``(pose, expected)`` — ``expected`` is the mcp/pip angles a
    capture is gated against (the manifolds are only valid there).
    """
    pose = dict(held)
    expected = {}
    for j in (parked, pusher):
        mcp = _finger(j) + "_mcp"
        pip = _finger(j) + "_pip"
        if mcp in roms:
            pose[mcp] = float(np.clip(posture, roms[mcp][0] + 2,
                                      roms[mcp][1] - 2))
            expected[mcp] = pose[mcp]
        if pip in roms:
            pose[pip] = float(np.clip(ABD_PIP_DEG, roms[pip][0] + 2,
                                      roms[pip][1] - 2))
            expected[pip] = pose[pip]
    return pose, expected


def find_manifold(manifolds: list, parked: str, pusher: str,
                  mean: dict) -> tuple[dict | None, str]:
    """Manifold entry for ``(arg=parked, out=pusher)`` matching the measured
    posture and parked range; ``(entry, why_not)``. A posture mismatch from
    OTHER posture families never masks the reason the intended entry
    failed, so the reported reason is the one that applies."""
    why = "no manifold for this pair/direction"
    default_why = why
    for man in manifolds:
        if man.get("arg") != parked or man.get("out") != pusher:
            continue
        tol = float(man.get("posture_tol_deg", 3.0))
        off = [j for j, v in man.get("posture", {}).items()
               if j in mean and np.isfinite(mean[j]) and abs(mean[j] - v) > tol]
        if off:
            if why == default_why:
                why = "posture outside manifold tolerance: " + ", ".join(off)
            continue
        lo, hi = man["arg_range"]
        if not (lo - ARG_RANGE_SLACK_DEG <= mean[parked]
                <= hi + ARG_RANGE_SLACK_DEG):
            why = (f"{parked} {mean[parked]:+.1f} deg outside manifold range "
                   f"[{lo:+.1f}, {hi:+.1f}]")
            continue
        return man, ""
    return None, why


def evaluate_manual_sample(parked: str, pusher: str, mean: dict, std: dict,
                           expected: dict, manifolds: list, signs: dict,
                           roms: dict, *,
                           still_tol_deg: float = MANUAL_STILL_TOL_DEG,
                           ) -> tuple[bool, list[str]]:
    """Gate one hand-held contact sample; ``(ok, notes)``.

    Pure so the gates are unit-testable: encoder presence, stillness of the
    pair, flexion posture within tolerance, and a matching manifold entry —
    whose deviation is reported as live into-mesh / short-of-contact
    feedback under the current calibration.
    """
    missing = sorted(j for j in {parked, pusher, *expected}
                     if not np.isfinite(mean.get(j, float("nan"))))
    if missing:
        return False, ["no encoder reading for " + ", ".join(missing)]
    ok, notes = True, []
    unsteady = [f"{j} std {std[j]:.2f} deg" for j in (parked, pusher)
                if std[j] > still_tol_deg]
    if unsteady:
        ok = False
        notes.append("unsteady hold: " + ", ".join(unsteady))
    off = [f"{j} {mean[j] - v:+.1f} deg" for j, v in expected.items()
           if abs(mean[j] - v) > POSTURE_TOL_DEG]
    if off:
        ok = False
        notes.append("flexion posture off (the motors hold these — avoid "
                     "leaning on the finger): " + ", ".join(off))
    sign_p = signs[(parked, pusher)]
    stop = roms[parked][1] if sign_p > 0 else roms[parked][0]
    notes.append(f"{parked} braced at stop reads {mean[parked]:+.2f} deg "
                 f"(rom limit {stop:+.1f}, delta {mean[parked] - stop:+.2f})")
    man, why = find_manifold(manifolds, parked, pusher, mean)
    if man is None:
        if manifolds:
            ok = False
            notes.append("solver would skip this event: " + why)
    else:
        dev = mean[pusher] - float(np.polyval(man["poly"], mean[parked]))
        depth = dev * signs[(pusher, parked)]
        side = "into the mesh" if depth > 0 else "short of mesh contact"
        notes.append(f"manifold deviation {dev:+.2f} deg — {abs(depth):.2f} "
                     f"deg {side} under the current calibration "
                     f"(sigma {man['sigma_deg']:.2f})")
    return ok, notes


def run_abd_block(hand, rec: SessionRecorder, roms: dict, held: dict,
                  signs: dict, push_currents_ma: list,
                  hold_current_ma: float) -> None:
    jidx = {j: i for i, j in enumerate(hand.config.motor_ids)}
    joint_motor = hand.config.joint_to_motor_map

    def currents(weak_joint=None, ma=None):
        base = [hold_current_ma] * len(hand.config.motor_ids)
        if weak_joint is not None:
            base[jidx[joint_motor[weak_joint]]] = ma
        return base

    for a, b in ABD_PAIRS:
        for parked, pusher in ((a, b), (b, a)):
            for posture, push_ma in [(p, c) for p in ABD_POSTURES
                                     for c in push_currents_ma]:
                pose, _ = abd_pose(held, roms, parked, pusher, posture)
                # Park at the stop FACING the pusher: contact then wedges
                # nothing (we read encoders, stiffness is all we need).
                sign_p = signs[(parked, pusher)]
                lo, hi = roms[parked]
                pose[parked] = (hi - ROM_MARGIN_DEG if sign_p > 0
                                else lo + ROM_MARGIN_DEG)
                sign_b = signs[(pusher, parked)]
                blo, bhi = roms[pusher]
                start = held[pusher] - sign_b * 5.0
                pose[pusher] = float(np.clip(start, blo + 1, bhi - 1))
                hand.set_joint_positions(pose)
                time.sleep(0.8)

                hand.set_max_current(currents(weak_joint=pusher, ma=push_ma))
                det = StallDetector()
                target = pose[pusher]
                limit = bhi - ROM_MARGIN_DEG if sign_b > 0 else blo + ROM_MARGIN_DEG
                contact = False
                while (target - limit) * sign_b < 0:
                    target += sign_b * 0.4
                    hand.set_joint_positions({pusher: float(target)})
                    time.sleep(0.15)
                    m = read_m(hand, samples=2)
                    if det.update(target, m[pusher]):
                        rec.add(m, kind="abd_block", body_a=parked,
                                body_b=pusher, push_current_ma=push_ma)
                        contact = True
                        print(f"  contact {pusher} -> {parked} @ posture "
                              f"{posture:.0f} / {push_ma:.0f} mA: "
                              f"{m[pusher]:.2f} deg "
                              f"(parked reads {m[parked]:.2f})")
                        break
                if not contact:
                    print(f"  WARNING: no contact {pusher} -> {parked} @ "
                          f"{posture:.0f} — stall thresholds or an obstruction; "
                          f"the approach direction came from the model")
                hand.set_joint_positions(
                    {pusher: float(target - sign_b * 6.0)})
                time.sleep(0.4)
                hand.set_max_current(currents())


def _manual_capture(hand, rec: SessionRecorder, parked: str, pusher: str,
                    expected: dict, manifolds: list, signs: dict, roms: dict,
                    flips: dict, *, still_tol_deg: float, samples: int,
                    interval: float) -> bool:
    """One ENTER-triggered capture; False when the operator quits the mode."""
    last = None
    while True:
        cmd = input("   ENTER=capture  f=save-anyway  s=skip  q=quit: "
                    ).strip().lower()
        if cmd == "q":
            return False
        if cmd == "s":
            return True
        if cmd == "f":
            if last is None:
                print("   nothing sampled yet — ENTER first")
                continue
            mean = last
        elif cmd == "":
            try:
                mean, std = sample_still(hand, samples, interval)
            except RuntimeError as e:
                print(f"   encoder stream lost: {e}")
                if not _reconnect(hand):
                    return False
                continue
            mean = apply_frame(mean, flips)
            ok, notes = evaluate_manual_sample(
                parked, pusher, mean, std, expected, manifolds, signs, roms,
                still_tol_deg=still_tol_deg)
            for n in notes:
                print(f"   {n}")
            if not ok:
                last = mean
                print("   NOT saved — adjust and ENTER again "
                      "(f saves this sample anyway)")
                continue
        else:
            print("   ENTER captures; f keeps the last rejected sample; "
                  "s skips this pairing; q ends the manual abd mode")
            continue
        rec.add(mean, kind="abd_block", body_a=parked, body_b=pusher,
                push_current_ma=0.0)
        print(f"   saved ({len(rec.events)} events total)")
        return True


def manual_plan() -> list[tuple[str, str, str]]:
    """(parked, pusher, variant) captures, grouped per pair, pinky side
    first; the easier both-extended variant leads within each pair."""
    return [(parked, pusher, variant)
            for a, b in ABD_PAIRS
            for variant in MANUAL_VARIANTS
            for parked, pusher in ((a, b), (b, a))]


def has_manifold(manifolds: list, parked: str, pusher: str,
                 expected: dict) -> bool:
    """Whether any entry could match this capture's commanded posture — a
    combination without one (mesh contact unreachable there) is skipped
    upfront rather than captured and dropped by the solver."""
    for man in manifolds:
        if man.get("arg") != parked or man.get("out") != pusher:
            continue
        tol = float(man.get("posture_tol_deg", 3.0))
        if all(abs(expected.get(j, v) - v) <= tol
               for j, v in man.get("posture", {}).items()):
            return True
    return False


def _variant_instructions(parked: str, pusher: str, variant: str) -> list[str]:
    fp, fb = _finger(parked).upper(), _finger(pusher).upper()
    if variant == "extended":
        return [
            f"1. Hold BOTH {fp} and {fb} fully straight — mcp and pip "
            "against their extension stops.",
            f"2. Push {fp} sideways toward {fb} until it rests on its own "
            "abd hard stop; keep it braced there.",
            f"3. Bring {fb} sideways until the two JUST touch (near the "
            "pip knuckle) — no squeeze.",
            "4. Hold everything still and press ENTER.",
        ]
    return [
        f"1. Hold {fp} fully straight — mcp and pip against their "
        "extension stops.",
        f"2. Curl {fb}'s mcp fully to its flexion stop (pip position is "
        "free).",
        f"3. Push {fp} sideways toward {fb} onto its own abd hard stop; "
        f"keep it braced. Bring the curled {fb} sideways until its "
        "knuckle JUST touches, near the palm — no squeeze.",
        "4. Hold everything still and press ENTER.",
    ]


def run_abd_manual(hand, rec: SessionRecorder, roms: dict, signs: dict,
                   flips: dict, manifolds: list, *, still_tol_deg: float,
                   samples: int, interval: float) -> None:
    hand.disable_torque()
    plan = manual_plan()
    print(f"\nManual abd contacts: {len(plan)} captures. Torque is OFF on "
          "every motor — the hand is yours, nothing will be driven. "
          "Uninvolved fingers can hang wherever convenient.")
    for i, (parked, pusher, variant) in enumerate(plan, 1):
        expected = manual_expected(roms, parked, pusher, variant)
        if manifolds and not has_manifold(manifolds, parked, pusher, expected):
            print(f"\n== [{i:2d}/{len(plan)}] {parked} + {pusher} "
                  f"({variant}): no manifold for this combination "
                  "— skipped ==")
            continue
        print(f"\n== [{i:2d}/{len(plan)}] {parked} + {pusher} "
              f"({variant}) ==")
        for line in _variant_instructions(parked, pusher, variant):
            print(f"   {line}")
        if not _manual_capture(hand, rec, parked, pusher, expected,
                               manifolds, signs, roms, flips,
                               still_tol_deg=still_tol_deg,
                               samples=samples, interval=interval):
            print("manual abd mode ended by operator")
            return
    n = sum(1 for e in rec.events if e.kind == "abd_block")
    print(f"\nmanual abd done: {n} events")


def _stdin_command() -> str | None:
    """Non-blocking read of a pending input line; None when there is none."""
    import select
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().lower()
    return None


def _tip_pair_loop(hand, rec: SessionRecorder, det: TipPressDetector,
                   geom: dict, fa: str, fb: str, flips: dict) -> bool:
    """Capture TIP_EVENTS_PER_PAIR presses of one pair, with a live force
    readout; ENTER skips the pair, q ends the mode (False)."""
    # Stillness gates only the joints in this event's constraint (base
    # through both fingers' chains) — tremor elsewhere is irrelevant.
    still = [j for j in rec.joints
             if j == "wrist" or _finger(j) in (fa, fb)]
    idx = [rec.joints.index(j) for j in still]
    n = 0
    last_status = 0.0
    while n < TIP_EVENTS_PER_PAIR:
        cmd = _stdin_command()
        if cmd == "q":
            print("\ntip mode ended by operator")
            return False
        if cmd is not None:
            print(f"\n  {fa}+{fb} skipped at {n}/{TIP_EVENTS_PER_PAIR}")
            return True
        try:
            taxels = hand.get_tactile_taxels()
            if taxels is None:
                time.sleep(0.05)
                continue
            ca, Fa = taxel_centroid(geom[fa].positions, taxels.as_array(fa))
            cb, Fb = taxel_centroid(geom[fb].positions, taxels.as_array(fb))
            meas = apply_frame(hand.get_measured_joints(), flips)
        except RuntimeError as e:
            print(f"\n  sensor stream lost: {e}")
            if not _reconnect(hand):
                return False
            continue
        m_row = np.array([meas.get(j, np.nan) for j in rec.joints])
        fired = det.update(Fa, Fb, m_row[idx])
        now = time.monotonic()
        if now - last_status > 0.3:
            print(f"\r  F {fa} {Fa:5.2f} / {fb} {Fb:5.2f} N "
                  f"(need >= {det.force_thresh_n:.2f} on both)  "
                  f"hold {det.hold}/{det.hold_samples}   ",
                  end="", flush=True)
            last_status = now
        if fired and np.isfinite(ca).all() and np.isfinite(cb).all():
            rec.add(meas, kind="tip_press", body_a=fa, body_b=fb,
                    p_a=ca, p_b=cb, force_a=Fa, force_b=Fb)
            n += 1
            print(f"\n  event {n}/{TIP_EVENTS_PER_PAIR}: "
                  f"F={Fa:.2f}/{Fb:.2f} N")
        time.sleep(0.05)
    return True


def run_tip_press(hand, rec: SessionRecorder, flips: dict,
                  pairs: list | None = None) -> None:
    if not hasattr(hand, "get_tactile_taxels"):
        print("no tactile sensing on this hand; skipping tip-press")
        return
    geom = hand.get_taxel_geometry()
    # The taxel stream is opt-in — nothing starts it at connect.
    try:
        hand.start_tactile_stream(resultant=False, taxels=True,
                                  min_sensors=1)
    except Exception as e:
        print(f"tip-press: cannot start the taxel stream: {e}")
        return
    hand.disable_torque()
    while _stdin_command() is not None:
        pass  # drain buffered keypresses so they don't skip pairs
    print("Torque OFF — guide the fingertips by hand. ENTER skips the "
          "current pair, q+ENTER ends tip mode.")
    try:
        print("zeroing the pads — keep all fingertips untouched...")
        try:
            hand.zero_tactile_sensors(num_samples=50)
        except Exception as e:
            print(f"  WARNING: could not zero pads: {e}")
        det = TipPressDetector()
        for fa, fb in (pairs if pairs is not None else TIP_PAIRS):
            if fa not in geom or fb not in geom:
                print(f"  {fa}+{fb}: pad not connected, skipped")
                continue
            print(f"\n== press {fa} and {fb} tips together — "
                  f"{TIP_EVENTS_PER_PAIR} events (mix light and firm, "
                  "vary the curl between presses) ==")
            if not _tip_pair_loop(hand, rec, det, geom, fa, fb, flips):
                return
    finally:
        try:
            hand.stop_tactile_stream()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("config_path")
    ap.add_argument("--out", required=True)
    ap.add_argument("--modes", default="abd-manual,tip",
                    help="Comma list of: abd-manual (hand-guided, default), "
                         "abd (motor-driven), tip")
    ap.add_argument("--push-current", default="80,160",
                    help="abd mode: pusher current limits, mA, comma list — "
                         "each approach repeats per current; two currents "
                         "let a later pass extrapolate the skin compression "
                         "out of the stall depth")
    ap.add_argument("--hold-current", type=float, default=350.0,
                    help="abd mode: current limit for every other motor, mA")
    ap.add_argument("--manifolds", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "contact_manifolds_right.yaml"),
        help="Manifold YAML for live abd-manual feedback "
             "(capture still records without it)")
    ap.add_argument("--still-tol", type=float, default=MANUAL_STILL_TOL_DEG,
                    help="abd-manual: max per-joint std (deg) over the "
                         "averaging window")
    ap.add_argument("--directions", default=None,
                    help="YAML {abd_joint: +-1} of measured toward-thumb "
                         "encoder signs, overriding the built-in "
                         "convention (see ENCODER_TOWARD_THUMB_SIGN)")
    ap.add_argument("--tip-pairs", default=None,
                    help="tip mode: comma list like thumb-index,index-middle "
                         "to capture only those pairs (e.g. to leave out a "
                         "finger with a known-bad joint)")
    ap.add_argument("--samples", type=int, default=MANUAL_SAMPLES,
                    help="abd-manual: encoder samples per capture "
                         f"({MANUAL_INTERVAL_S:.2f} s apart)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    modes = {m.strip() for m in args.modes.split(",") if m.strip()}
    unknown = modes - {"abd", "abd-manual", "tip"}
    if unknown:
        raise SystemExit(f"unknown mode(s): {sorted(unknown)} "
                         "(choose from abd-manual, abd, tip)")

    from orca_core import load_hand

    hand = load_hand(args.config_path)
    if not hasattr(hand, "get_measured_joints"):
        raise SystemExit("contact calibration needs a joint-feedback hand "
                         "(the encoders are the whole measurement)")

    manifolds = []
    if "abd-manual" in modes:
        if os.path.exists(args.manifolds):
            with open(args.manifolds) as f:
                manifolds = yaml.safe_load(f) or []
        else:
            print(f"NOTE: no manifold file at {args.manifolds} — capturing "
                  "without live contact feedback")

    if args.dry_run:
        side = getattr(hand.config, "type", "right")
        held0 = {j: PARK.get(j, hand.config.neutral_position.get(j, 0.0))
                 for j in hand.config.joint_roms_dict}
        signs = compute_approach_signs(side, held0)
        print(f"approach directions from the {side}-hand model "
              "(sanity-check once against your hardstop calibration):")
        for (jf, jt), sgn in signs.items():
            word = "increasing" if sgn > 0 else "decreasing"
            print(f"  {jf:>11} -> toward {_finger(jt):<7}: {word} angle")
        if "abd-manual" in modes:
            plan = manual_plan()
            roms0 = {j: tuple(v) for j, v in
                     hand.config.joint_roms_dict.items()}
            span = args.samples * MANUAL_INTERVAL_S
            print(f"abd-manual: {len(plan)} ENTER-triggered captures, fully "
                  "hand-guided (torque off everywhere, nothing driven), "
                  f"encoders averaged over ~{span:.1f} s "
                  f"({len(manifolds)} manifold entries for live feedback):")
            for i, (parked, pusher, variant) in enumerate(plan, 1):
                expected = manual_expected(roms0, parked, pusher, variant)
                skip = (manifolds and
                        not has_manifold(manifolds, parked, pusher, expected))
                note = "  (no manifold — will be skipped)" if skip else ""
                desc = ("both straight" if variant == "extended"
                        else f"{_finger(pusher)} mcp curled")
                print(f"  [{i:2d}] {variant:>8}: brace {parked} at its stop "
                      f"toward {_finger(pusher)}, touch {pusher} against it "
                      f"({desc}){note}")
        if "abd" in modes:
            currents = [v for v in args.push_current.split(",") if v]
            n_abd = len(ABD_PAIRS) * 2 * len(ABD_POSTURES) * len(currents)
            print(f"abd: {n_abd} motor-driven approaches "
                  f"({[p for p in ABD_PAIRS]} x 2 dir x {list(ABD_POSTURES)} "
                  f"x {currents} mA)")
        if "tip" in modes:
            n_tip = len(TIP_PAIRS) * TIP_EVENTS_PER_PAIR
            print(f"tip-press: up to {n_tip} events over "
                  f"{len(TIP_PAIRS)} pairs")
        print("wrist: not constrained by any contact class "
              "(kept on hardstop cal)")
        return 0

    ok, msg = hand.connect()
    if not ok:
        raise SystemExit(f"connect failed: {msg}")
    try:
        joints = hand.loop_joint_names
        if joints is None:
            raise SystemExit("joint loop not running after connect()")
        roms = {j: tuple(hand.config.joint_roms_dict[j]) for j in joints}
        held = {
            j: float(np.clip(PARK.get(j, hand.config.neutral_position.get(j, 0.0)),
                             roms[j][0] + 2, roms[j][1] - 2))
            for j in joints
        }
        rec = SessionRecorder(list(joints))
        if "abd" in modes:
            hand.init_joints()
        side = getattr(hand.config, "type", "right")
        model_thumb = compute_toward_thumb_signs(side, {})
        measured = None
        if args.directions:
            with open(args.directions) as f:
                measured = {j: float(v) for j, v in
                            (yaml.safe_load(f) or {}).items()}
            print(f"using measured abd directions from {args.directions}")
        flips = abd_frame_flips(model_thumb, roms, measured)
        flipped = sorted(j for j, (s, _) in flips.items() if s < 0)
        if flipped:
            print("abd angles are mapped into the model frame at capture: "
                  + ", ".join(f"{j} (x{flips[j][0]:+.0f}"
                              + (f" {flips[j][1]:+.0f} deg" if flips[j][1]
                                 else "") + ")" for j in flipped))
            if "abd" in modes:
                print("WARNING: the motorized abd mode records raw "
                      "config-frame angles — do not mix it with the "
                      "manual modes on this hand")
        try:
            if "abd" in modes:
                push_ma = [float(v) for v in args.push_current.split(",") if v]
                run_abd_block(hand, rec, roms, held,
                              compute_approach_signs(side, held), push_ma,
                              args.hold_current)
            if "abd-manual" in modes:
                # Abd gates run in the model frame: model ROMs for the
                # abds, the hand's own (model-consistent) mcp/pip roms.
                from model import JOINT_ROMS as MODEL_ROMS
                gate_roms = dict(roms)
                gate_roms.update({j: tuple(MODEL_ROMS[j]) for j in flips
                                  if j in MODEL_ROMS})
                run_abd_manual(hand, rec, gate_roms,
                               derive_pair_signs(model_thumb), flips,
                               manifolds,
                               still_tol_deg=args.still_tol,
                               samples=args.samples,
                               interval=MANUAL_INTERVAL_S)
            if "tip" in modes:
                pairs = None
                if args.tip_pairs:
                    want = {frozenset(p.split("-"))
                            for p in args.tip_pairs.split(",") if p}
                    known = {frozenset(p) for p in TIP_PAIRS}
                    bad = want - known
                    if bad:
                        raise SystemExit(
                            f"unknown tip pair(s): {sorted(map(sorted, bad))}")
                    pairs = [p for p in TIP_PAIRS if frozenset(p) in want]
                run_tip_press(hand, rec, flips, pairs)
        finally:
            # An abort (Ctrl-C, a fault) must not lose captured events.
            if rec.events:
                rec.save(args.out, meta={
                    "producer": "record_contacts",
                    "config_path": os.path.abspath(args.config_path),
                    "modes": sorted(modes),
                    "still_tol_deg": args.still_tol,
                    "samples_per_capture": args.samples,
                    "abd_frame": {j: [int(s), float(k)]
                                  for j, (s, k) in flips.items()},
                    "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                })
        return 0
    finally:
        if "abd" in modes:
            # Only the motor-driven mode restores neutral; the manual
            # modes end with the hand deliberately limp.
            try:
                hand.enable_torque()
                hand.set_neutral_position()
                time.sleep(1.0)
            except Exception as e:
                print(f"WARNING: could not return to neutral: {e}")
        else:
            print("hand left torque-free (manual session over — arrange "
                  "it as you like)")
        hand.disconnect()


if __name__ == "__main__":
    sys.exit(main())
