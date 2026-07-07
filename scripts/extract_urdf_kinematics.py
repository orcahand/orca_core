# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Extract the hand kinematic chains from an orcahand_description v2 URDF.

Produces the ``chains`` section of ``orca_core/kinematics/data/*_kinematics.yaml``:
per-joint origins/axes for wrist and each finger, mapped to orca_core joint ids.
Joint signs are audited against a hand config's ``joint_roms``: URDF limits that
mirror the config ROM indicate an inverted axis (``sign: -1``); symmetric limits
cannot be audited and are flagged for physical verification.

Usage:
    uv run python scripts/extract_urdf_kinematics.py <orcahand_right.urdf> \
        --config orca_core/models/v2/orcahand_touch_right/config.yaml [--out chains.yaml]
"""
from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET

import numpy as np
import yaml

FINGER_AP_PREFIXES = {"I-AP": "index", "P-AP": "pinky"}
THUMB_CHAIN_JOINTS = ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"]
FINGER_CHAIN_JOINTS = ["abd", "mcp", "pip"]


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

    def revolute_children(self, parent: str) -> list[dict]:
        return [j for j in self.joints.values() if j["parent"] == parent and j["type"] == "revolute"]


def identify_chains(model: UrdfModel) -> tuple[dict, str, dict[str, list[dict]]]:
    """Return (wrist joint, carpals link, {finger: [joint, ...]})."""
    revolute = [j for j in model.joints.values() if j["type"] == "revolute"]
    wrist_candidates = [j for j in revolute if "Carpals" in j["child"]]
    if len(wrist_candidates) != 1:
        raise ValueError("could not identify the wrist joint")
    wrist = wrist_candidates[0]
    carpals = wrist["child"]

    ap_joints = model.revolute_children(carpals)
    fingers: dict[str, list[dict]] = {}
    middles = []
    for joint in ap_joints:
        child = joint["child"]
        prefix = child.split("_")[0]
        if prefix.startswith("T-TP"):
            fingers["thumb"] = [joint]
        elif prefix.startswith("M-AP"):
            middles.append(joint)
        else:
            for known, finger in FINGER_AP_PREFIXES.items():
                if prefix.startswith(known):
                    fingers[finger] = [joint]
    if len(middles) != 2 or {"index", "pinky", "thumb"} - set(fingers):
        raise ValueError(f"unexpected finger layout: {sorted(fingers)} + {len(middles)} M-AP chains")

    # Ring sits between middle and pinky: of the two M-AP chains, ring's
    # abduction origin is the one closer to the pinky's.
    pinky_origin = np.array(fingers["pinky"][0]["xyz"])
    distances = [np.linalg.norm(np.array(j["xyz"]) - pinky_origin) for j in middles]
    ring_joint = middles[int(np.argmin(distances))]
    middle_joint = middles[1 - int(np.argmin(distances))]
    fingers["ring"] = [ring_joint]
    fingers["middle"] = [middle_joint]

    for finger, chain in fingers.items():
        expected = 4 if finger == "thumb" else 3
        while len(chain) < expected:
            nxt = model.revolute_children(chain[-1]["child"])
            if len(nxt) != 1:
                raise ValueError(f"{finger}: expected a single child joint after {chain[-1]['name']}")
            chain.append(nxt[0])
    return wrist, carpals, fingers


def audit_sign(joint: dict, rom_deg: list[float] | None) -> tuple[int, str]:
    """Compare URDF limits with the config ROM: mirrored limits mean an inverted axis."""
    if joint["limits"] is None or rom_deg is None:
        return 1, "unverified"
    lo, hi = (math.degrees(v) for v in joint["limits"])
    same = abs(lo - rom_deg[0]) < 0.5 and abs(hi - rom_deg[1]) < 0.5
    mirrored = abs(lo + rom_deg[1]) < 0.5 and abs(hi + rom_deg[0]) < 0.5
    if same and mirrored:
        return 1, "unverified (symmetric limits)"
    if same:
        return 1, "verified"
    if mirrored:
        return -1, "verified (mirrored limits)"
    return 1, f"MISMATCH: urdf=[{lo:.1f}, {hi:.1f}] config={rom_deg}"


def joint_entry(joint: dict, orca_id: str, rom_deg: list[float] | None) -> dict:
    sign, audit = audit_sign(joint, rom_deg)
    entry = {
        "joint": orca_id,
        "urdf_joint": joint["name"],
        "xyz": joint["xyz"],
        "rpy": joint["rpy"],
        "axis": joint["axis"],
        "sign": sign,
        "audit": audit,
    }
    if joint["limits"] is not None:
        entry["urdf_limits_deg"] = [round(math.degrees(v), 3) for v in joint["limits"]]
    return entry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf_path")
    parser.add_argument("--config", help="hand config.yaml with joint_roms for the sign audit")
    parser.add_argument("--out", help="output YAML path (stdout if omitted)")
    args = parser.parse_args()

    roms = {}
    if args.config:
        with open(args.config) as f:
            roms = yaml.safe_load(f).get("joint_roms", {})

    model = UrdfModel(args.urdf_path)
    wrist, carpals, fingers = identify_chains(model)

    base_chain = []
    for joint in model.chain_to(wrist["parent"]):
        if joint["type"] != "fixed":
            raise ValueError(f"unexpected non-fixed joint above the wrist: {joint['name']}")
        base_chain.append({"fixed": joint["name"], "xyz": joint["xyz"], "rpy": joint["rpy"]})
    base_chain.append(joint_entry(wrist, "wrist", roms.get("wrist")))

    chains: dict = {"base_link": model.root, "palm_link": carpals, "base_chain": base_chain, "fingers": {}}
    for finger in ["thumb", "index", "middle", "ring", "pinky"]:
        names = (
            THUMB_CHAIN_JOINTS if finger == "thumb" else [f"{finger}_{j}" for j in FINGER_CHAIN_JOINTS]
        )
        chains["fingers"][finger] = [
            joint_entry(joint, orca_id, roms.get(orca_id))
            for joint, orca_id in zip(fingers[finger], names, strict=True)
        ]

    output = yaml.safe_dump({"chains": chains}, sort_keys=False, default_flow_style=None)
    if args.out:
        with open(args.out, "w") as f:
            f.write(output)
    else:
        print(output)


if __name__ == "__main__":
    main()
