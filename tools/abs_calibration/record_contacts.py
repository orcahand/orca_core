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

abd-block   One finger parks at its abduction stop with normal current (the
            stop only provides stiffness — its encoder is READ at contact,
            so no hardstop angle is ever assumed). The neighbour approaches
            in small steps under a LOW current limit; first contact is the
            onset of tracking error with dying velocity (contact_detect),
            captured and backed off before force builds. Pairwise in both
            directions and at two flexion postures. Solving these needs the
            mesh contact manifolds (build_contact_manifold.py).

tip-press   Torque off (the hand goes limp; encoders stay live). The
            operator presses the prompted fingertip pair together by hand;
            when both pads report a localisable resultant and the encoders
            are still, the event auto-captures (contact centroid per pad in
            its sensor frame + forces). Both light and firm presses are
            wanted (sigma(F) is modelled). Full hands only.

The wrist is constrained by NEITHER class (no self-contact crosses the
wrist) and keeps its hardstop calibration; the solver's sigma gate reports
this honestly.

CAUTION before first hardware use: verify the per-pair approach directions
in APPROACH below with manual_control.py sliders — abduction sign
conventions decide which way "toward the neighbour" is, and driving the
wrong way just visits the far hardstop.

Usage:
    uv run python tools/abs_calibration/record_contacts.py CONFIG_PATH \
        --out SESSION_DIR [--modes abd,tip] [--push-current 100] [--dry-run]
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

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

# Approach directions, VERIFY ON HARDWARE: for each abd joint, +1 means
# "toward larger angle is toward the pinky side of the hand". Derived from
# the v2 neutral spread (index -14, middle -4, ring +10, pinky +22): larger
# abd = toward the pinky. Flip here if the first --dry-run-on-hardware
# check disagrees.
ABD_TOWARD_PINKY_SIGN = {
    "index_abd": +1, "middle_abd": +1, "ring_abd": +1, "pinky_abd": +1,
}
FINGER_ORDER = ["index", "middle", "ring", "pinky"]  # thumb side -> pinky side

TIP_PAIRS = [
    ("thumb", "index"), ("thumb", "middle"), ("thumb", "ring"),
    ("thumb", "pinky"), ("index", "middle"), ("middle", "ring"),
    ("ring", "pinky"),
]
TIP_EVENTS_PER_PAIR = 4


def _finger(joint: str) -> str:
    return joint.rsplit("_", 1)[0]


def _toward(joint_from: str, joint_to: str) -> float:
    """Sign of motion of ``joint_from`` that approaches ``joint_to``'s finger."""
    a = FINGER_ORDER.index(_finger(joint_from))
    b = FINGER_ORDER.index(_finger(joint_to))
    toward_pinky = 1.0 if b > a else -1.0
    return toward_pinky * ABD_TOWARD_PINKY_SIGN[joint_from]


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


def run_abd_block(hand, rec: SessionRecorder, roms: dict, held: dict,
                  push_currents_ma: list, hold_current_ma: float) -> None:
    jidx = {j: i for i, j in enumerate(hand.config.motor_ids)}
    joint_motor = hand.config.joint_ids

    def currents(weak_joint=None, ma=None):
        base = [hold_current_ma] * len(hand.config.motor_ids)
        if weak_joint is not None:
            base[jidx[joint_motor[weak_joint]]] = ma
        return base

    for a, b in ABD_PAIRS:
        for parked, pusher in ((a, b), (b, a)):
            for posture, push_ma in [(p, c) for p in ABD_POSTURES
                                     for c in push_currents_ma]:
                pose = dict(held)
                for j in (parked, pusher):
                    mcp = _finger(j) + "_mcp"
                    pip = _finger(j) + "_pip"
                    if mcp in roms:
                        pose[mcp] = float(np.clip(posture, roms[mcp][0] + 2,
                                                  roms[mcp][1] - 2))
                    if pip in roms:
                        pose[pip] = float(np.clip(ABD_PIP_DEG, roms[pip][0] + 2,
                                                  roms[pip][1] - 2))
                # Park at the stop FACING the pusher: contact then wedges
                # nothing (we read encoders, stiffness is all we need).
                sign_p = _toward(parked, pusher)
                lo, hi = roms[parked]
                pose[parked] = (hi - ROM_MARGIN_DEG if sign_p > 0
                                else lo + ROM_MARGIN_DEG)
                sign_b = _toward(pusher, parked)
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
                          f"{posture:.0f} — check ABD_TOWARD_PINKY_SIGN")
                hand.set_joint_positions(
                    {pusher: float(target - sign_b * 6.0)})
                time.sleep(0.4)
                hand.set_max_current(currents())


def run_tip_press(hand, rec: SessionRecorder) -> None:
    if not hasattr(hand, "get_tactile_taxels"):
        print("no tactile sensing on this hand; skipping tip-press")
        return
    geom = hand.get_taxel_geometry()
    hand.disable_torque()
    print("Torque OFF — guide the fingertips by hand.")
    det = TipPressDetector()
    for fa, fb in TIP_PAIRS:
        if fa not in geom or fb not in geom:
            print(f"  {fa}+{fb}: pad not connected, skipped")
            continue
        print(f"\n== press {fa} and {fb} tips together — "
              f"{TIP_EVENTS_PER_PAIR} events (mix light and firm) ==")
        n = 0
        while n < TIP_EVENTS_PER_PAIR:
            taxels = hand.get_tactile_taxels()
            if taxels is None:
                time.sleep(0.05)
                continue
            ca, Fa = taxel_centroid(geom[fa].positions, taxels.as_array(fa))
            cb, Fb = taxel_centroid(geom[fb].positions, taxels.as_array(fb))
            meas = hand.get_measured_joints()
            m_row = np.array([meas.get(j, np.nan) for j in rec.joints])
            if det.update(Fa, Fb, m_row) and np.isfinite(ca).all() \
                    and np.isfinite(cb).all():
                rec.add(meas, kind="tip_press", body_a=fa, body_b=fb,
                        p_a=ca, p_b=cb, force_a=Fa, force_b=Fb)
                n += 1
                print(f"  event {n}/{TIP_EVENTS_PER_PAIR}: "
                      f"F={Fa:.2f}/{Fb:.2f} N")
            time.sleep(0.05)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("config_path")
    ap.add_argument("--out", required=True)
    ap.add_argument("--modes", default="abd,tip")
    ap.add_argument("--push-current", default="80,160",
                    help="Pusher current limits, mA, comma list — each "
                         "approach repeats per current; two currents let a "
                         "later pass extrapolate the skin compression out "
                         "of the stall depth")
    ap.add_argument("--hold-current", type=float, default=350.0,
                    help="Current limit for every other motor, mA")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    modes = {m.strip() for m in args.modes.split(",")}

    from orca_core import load_hand

    hand = load_hand(args.config_path)
    if not hasattr(hand, "get_measured_joints"):
        raise SystemExit("contact calibration needs a joint-feedback hand "
                         "(the encoders are the whole measurement)")

    if args.dry_run:
        n_abd = len(ABD_PAIRS) * 2 * len(ABD_POSTURES)
        n_tip = len(TIP_PAIRS) * TIP_EVENTS_PER_PAIR
        print(f"abd-block: {n_abd} approaches "
              f"({[p for p in ABD_PAIRS]} x 2 dir x {list(ABD_POSTURES)})")
        print(f"tip-press: up to {n_tip} events over {len(TIP_PAIRS)} pairs")
        print("wrist: not constrained by either class (kept on hardstop cal)")
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
        hand.init_joints()
        if "abd" in modes:
            push_ma = [float(v) for v in args.push_current.split(",") if v]
            run_abd_block(hand, rec, roms, held, push_ma, args.hold_current)
        if "tip" in modes:
            run_tip_press(hand, rec)
        rec.save(args.out, meta={
            "producer": "record_contacts",
            "config_path": os.path.abspath(args.config_path),
            "modes": sorted(modes),
            "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        })
        return 0
    finally:
        try:
            hand.enable_torque()
            hand.set_neutral_position()
            time.sleep(1.0)
        except Exception as e:
            print(f"WARNING: could not return to neutral: {e}")
        hand.disconnect()


if __name__ == "__main__":
    sys.exit(main())
