# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Maintainer tool: extract the hand kinematic chains from an orcahand_description URDF.

Regenerates the ``chains`` section of ``orca_core/kinematics/data/*_kinematics.yaml``
(per-joint origins/axes for wrist and each finger, mapped to orca_core joint ids).
Run it when the mechanical design changes

Each joint's ``sign`` relates an orca_core joint angle to the rotation direction
about the URDF axis. It is derived two ways:

  * against the config's ``joint_roms``: URDF limits that mirror the config ROM
    mean an inverted axis (``sign: -1``). Symmetric limits carry no information.
  * for finger abduction joints with symmetric limits, by axis coherence: all
    four rotate about the palm normal, so their sign-applied axes must be
    parallel once expressed in the palm frame. The sign is resolved against a
    reference abduction joint whose limits *were* auditable.

An audit report goes to stderr, along with a warning for any config ROM that
commands past the URDF's mechanical limit. A sign that stays undetermined is
fatal, and so is a limit-verified sign that contradicts axis coherence: the
tool refuses to emit data it could not verify.

Usage:
    uv run python tools/extract_urdf_kinematics.py <orcahand_right.urdf> \
        --config orca_core/models/v2/orcahand-right/config.yaml [--out chains.yaml]
"""
from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from orca_core.kinematics.transforms import Transform

THUMB_CHAIN_JOINTS = ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"]
FINGER_CHAIN_JOINTS = ["abd", "mcp", "pip"]
ABD_FINGERS = ("index", "middle", "ring", "pinky")

# orca joint ids per finger, in chain order (root to tip).
CHAIN_JOINTS = {"thumb": THUMB_CHAIN_JOINTS}
CHAIN_JOINTS.update(
    {f: [f"{f}_{j}" for j in FINGER_CHAIN_JOINTS] for f in ABD_FINGERS}
)

# Minimum |cos| between two finger abduction axes in the palm frame before we
# trust them to be the same physical axis and resolve a sign from coherence.
AXIS_PARALLEL_TOL = 0.9

# A range whose midpoint sits within this many degrees of zero is treated as
# symmetric, and so tells us nothing about the axis direction.
LEAN_TOL_DEG = 1.0


def _floats(text: str | None, default: str) -> list[float]:
    return [float(v) for v in (text or default).split()]


def _snap(values: list[float], tol: float = 5e-7) -> list[float]:
    """Round away float noise; snap near-integers (0, ±1) and near-multiples of pi/2."""
    out = []
    for v in values:
        if abs(v - round(v)) < tol:
            v = float(round(v))
        else:
            half_pi_multiple = round(v / (math.pi / 2))
            if half_pi_multiple != 0 and abs(v - half_pi_multiple * math.pi / 2) < tol:
                v = half_pi_multiple * math.pi / 2
        out.append(round(v, 9))
    return out


class UrdfModel:
    def __init__(self, urdf_path: str):
        root = ET.parse(urdf_path).getroot()
        self.joints = {}
        self.child_joint = {}  # child link -> joint dict
        children = set()
        links = {link.get("name") for link in root.findall("link")}
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            limit = joint.find("limit")
            axis = joint.find("axis")
            info = {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "xyz": _snap(_floats(origin.get("xyz") if origin is not None else None, "0 0 0")),
                "rpy": _snap(_floats(origin.get("rpy") if origin is not None else None, "0 0 0")),
                "axis": _snap(_floats(axis.get("xyz") if axis is not None else None, "1 0 0")),
                "limits": (
                    [float(limit.get("lower")), float(limit.get("upper"))]
                    if limit is not None
                    else None
                ),
            }
            self.joints[info["name"]] = info
            self.child_joint[info["child"]] = info
            children.add(info["child"])
        roots = links - children
        if len(roots) != 1:
            raise ValueError(f"expected a single root link, got {sorted(roots)}")
        self.root = next(iter(roots))

    def chain_to(self, link: str) -> list[dict]:
        """Joints from the root down to ``link``, in order."""
        chain = []
        while link != self.root:
            joint = self.child_joint[link]
            chain.append(joint)
            link = joint["parent"]
        return list(reversed(chain))


def identify_chains(model: UrdfModel) -> tuple[dict, str, dict[str, list[dict]]]:
    """Return (wrist joint, carpals link, {finger: [joint, ...]}).

    v2 URDF joints are named ``{side}_{orca_joint_id}`` (e.g. right_index_mcp),
    so identification is a name lookup; the chain connectivity is still
    verified structurally.
    """
    by_orca: dict[str, dict] = {}
    for joint in model.joints.values():
        if joint["type"] != "revolute":
            continue
        side, _, orca_id = joint["name"].partition("_")
        if side not in ("right", "left") or not orca_id:
            raise ValueError(
                f"revolute joint {joint['name']!r} is not named "
                "{side}_{orca_joint_id}"
            )
        if orca_id in by_orca:
            raise ValueError(f"duplicate joint id {orca_id!r}")
        by_orca[orca_id] = joint

    expected = {"wrist"} | {jid for chain in CHAIN_JOINTS.values() for jid in chain}
    if set(by_orca) != expected:
        raise ValueError(
            f"unexpected joint set: missing {sorted(expected - set(by_orca))}, "
            f"extra {sorted(set(by_orca) - expected)}"
        )

    wrist = by_orca["wrist"]
    carpals = wrist["child"]
    fingers: dict[str, list[dict]] = {}
    for finger, names in CHAIN_JOINTS.items():
        chain = [by_orca[n] for n in names]
        if chain[0]["parent"] != carpals:
            raise ValueError(f"{finger}: {chain[0]['name']} is not rooted at {carpals}")
        for above, below in zip(chain, chain[1:]):
            if below["parent"] != above["child"]:
                raise ValueError(
                    f"{finger}: chain is not connected at {below['name']}"
                )
        fingers[finger] = chain
    return wrist, carpals, fingers


def audit_sign(joint: dict, rom_deg: list[float] | None) -> tuple[int, str]:
    """Derive a joint's sign by comparing URDF limits with the config ROM.

    Exact mirrored limits prove an inverted axis. A config ROM is often clipped
    relative to the mechanical limit, so when the two ranges merely differ we
    fall back to which side of zero each one leans: ranges leaning opposite ways
    mean an inverted axis. Ranges centred on zero carry no information.
    """
    if joint["limits"] is None or rom_deg is None:
        return 1, "unverified (no urdf limits or no config ROM)"

    lo, hi = (math.degrees(v) for v in joint["limits"])
    same = abs(lo - rom_deg[0]) < 0.5 and abs(hi - rom_deg[1]) < 0.5
    mirrored = abs(lo + rom_deg[1]) < 0.5 and abs(hi + rom_deg[0]) < 0.5
    if same and mirrored:
        return 1, "unverified (symmetric limits)"
    if same:
        return 1, "verified (limits match)"
    if mirrored:
        return -1, "verified (mirrored limits)"

    urdf_lean = (lo + hi) / 2.0
    rom_lean = (rom_deg[0] + rom_deg[1]) / 2.0
    if abs(urdf_lean) < LEAN_TOL_DEG or abs(rom_lean) < LEAN_TOL_DEG:
        return 1, "unverified (symmetric limits)"
    if urdf_lean * rom_lean > 0:
        return 1, "verified (limits lean the same way)"
    return -1, "verified (limits lean opposite ways)"


def rom_exceeds_urdf(joint: dict, rom_deg: list[float] | None, sign: int) -> str | None:
    """Warn when a config ROM commands past the URDF's mechanical limit."""
    if joint["limits"] is None or rom_deg is None:
        return None
    lo, hi = (math.degrees(v) for v in joint["limits"])
    rom_lo, rom_hi = sorted(sign * v for v in rom_deg)
    if rom_lo < lo - 0.5 or rom_hi > hi + 0.5:
        return (f"config ROM [{rom_deg[0]}, {rom_deg[1]}] reaches past the URDF limit "
                f"[{lo:.1f}, {hi:.1f}] (sign={sign})")
    return None


def joint_entry(joint: dict, orca_id: str, rom_deg: list[float] | None) -> tuple[dict, str]:
    """Return the packaged entry plus its audit status (the status is reported,
    not written to the YAML — the data file carries only what FK consumes)."""
    sign, audit = audit_sign(joint, rom_deg)
    entry = {
        "joint": orca_id,
        "xyz": joint["xyz"],
        "rpy": joint["rpy"],
        "axis": joint["axis"],
        "sign": sign,
    }
    return entry, audit


def _palm_frame_abduction_axis(chain: list[dict]) -> np.ndarray:
    """Sign-applied abduction axis of one finger, expressed in the palm frame."""
    pose = Transform.identity()
    for entry in chain:
        pose = pose @ Transform.from_xyz_rpy(entry["xyz"], entry["rpy"])
        if entry.get("joint", "").endswith("_abd"):
            axis = pose.rotation @ np.array(entry["axis"])
            return entry["sign"] * axis / np.linalg.norm(axis)
    raise ValueError("no abduction joint in chain")


def resolve_abduction_signs(chains: dict, audits: dict[str, str]) -> list[str]:
    """Fix the sign of abduction joints whose limits were symmetric.

    All four finger abduction joints rotate about the palm normal, so their
    sign-applied axes must be parallel in the palm frame. Anchor on one whose
    sign the limit audit could determine and flip the rest to match. Only
    limit-unverified signs may be flipped: a limit-verified sign that turns out
    anti-parallel means the URDF and config disagree, which is fatal.
    """
    fingers = chains["fingers"]
    reference = next(
        (f for f in ABD_FINGERS if audits[f"{f}_abd"].startswith("verified")), None
    )
    if reference is None:
        raise ValueError(
            "no finger abduction joint has auditable (asymmetric) limits; "
            "the abduction sign convention cannot be anchored"
        )

    ref_axis = _palm_frame_abduction_axis(fingers[reference])
    notes = []
    for finger in ABD_FINGERS:
        if finger == reference:
            continue
        axis = _palm_frame_abduction_axis(fingers[finger])
        alignment = float(np.dot(ref_axis, axis))
        if abs(alignment) < AXIS_PARALLEL_TOL:
            raise ValueError(
                f"{finger}_abd axis is not parallel to {reference}_abd in the palm "
                f"frame (cos={alignment:.3f}); the coherence assumption does not hold"
            )
        joint_id = f"{finger}_abd"
        if alignment < 0:
            if audits[joint_id].startswith("verified"):
                raise ValueError(
                    f"{joint_id} sign was {audits[joint_id]} yet its sign-applied "
                    f"axis is anti-parallel to {reference}_abd in the palm frame; "
                    "the URDF limits and the config ROM contradict axis coherence "
                    "and must be reconciled by hand"
                )
            entry = next(e for e in fingers[finger] if e.get("joint") == joint_id)
            entry["sign"] = -entry["sign"]
        if audits[joint_id].startswith("unverified"):
            audits[joint_id] = f"resolved by axis coherence with {reference}_abd"
        notes.append(finger)
    return notes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("urdf_path")
    parser.add_argument("--config", help="hand config.yaml with joint_roms for the sign audit")
    parser.add_argument("--out", help="output YAML path (stdout if omitted)")
    args = parser.parse_args()

    roms = {}
    if args.config:
        with open(args.config) as f:
            roms = yaml.safe_load(f).get("joint_roms", {})

    model = UrdfModel(args.urdf_path)
    wrist, _carpals, fingers = identify_chains(model)
    audits: dict[str, str] = {}
    urdf_by_orca: dict[str, dict] = {}

    base_chain = []
    for joint in model.chain_to(wrist["parent"]):
        if joint["type"] != "fixed":
            raise ValueError(f"unexpected non-fixed joint above the wrist: {joint['name']}")
        base_chain.append({"fixed": True, "xyz": joint["xyz"], "rpy": joint["rpy"]})
    entry, audits["wrist"] = joint_entry(wrist, "wrist", roms.get("wrist"))
    urdf_by_orca["wrist"] = wrist
    base_chain.append(entry)

    chains: dict = {"base_chain": base_chain, "fingers": {}}
    for finger in ["thumb", "index", "middle", "ring", "pinky"]:
        names = CHAIN_JOINTS[finger]
        chain = []
        for joint, orca_id in zip(fingers[finger], names, strict=True):
            entry, audits[orca_id] = joint_entry(joint, orca_id, roms.get(orca_id))
            urdf_by_orca[orca_id] = joint
            chain.append(entry)
        chains["fingers"][finger] = chain

    resolve_abduction_signs(chains, audits)

    signs = {e["joint"]: e["sign"] for e in base_chain if "joint" in e}
    signs.update({e["joint"]: e["sign"] for c in chains["fingers"].values() for e in c})

    print("Sign audit (reported, not written to the YAML):", file=sys.stderr)
    warnings = []
    for joint_id, status in audits.items():
        print(f"  {joint_id:14} sign={signs[joint_id]:>2}  {status}", file=sys.stderr)
        warn = rom_exceeds_urdf(urdf_by_orca[joint_id], roms.get(joint_id), signs[joint_id])
        if warn:
            warnings.append(f"  {joint_id:14} {warn}")

    if warnings:
        print("\nROM / mechanical-limit warnings (not fatal):", file=sys.stderr)
        print("\n".join(warnings), file=sys.stderr)

    unresolved = [j for j, s in audits.items() if s.startswith("unverified")]
    if unresolved:
        print(
            f"\nFAIL: sign could not be determined for {unresolved}. "
            "Provide a config whose joint_roms lean the same way as the URDF limits, "
            "or verify these joints on hardware.",
            file=sys.stderr,
        )
        return 1

    output = yaml.safe_dump({"chains": chains}, sort_keys=False, default_flow_style=None)
    if args.out:
        with open(args.out, "w") as f:
            f.write(output)
    else:
        print(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
