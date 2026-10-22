# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""What the slack test and the tension sweep share: which joints they drive,
which ones move together, which way a motor moves its joint, and the move to
a start pose."""

from __future__ import annotations

import math
import time
from typing import Dict, List, Sequence, Tuple

from ..constants import WRIST
from .motor_reads import read_motor_pos_checked

DIRECTIONS = (1, -1)
SENSES = ("flex", "extend")
# Start poses: every driven joint at the middle of its ROM (furthest from both
# limits), or the config's neutral position.
POSE_MID = "mid"
POSE_NEUTRAL = "neutral"
POSE_MOVE_STEPS = 50
POSE_MOVE_STEP_S = 0.02
POSE_SETTLE_S = 1.0

# Joints driven at the same time: every thumb joint on its own (the thumb
# crosses the fingers), and the four fingers together per joint kind, moving
# the same way in joint terms (all abductions to one side, all MCPs flexing,
# ...), as calibration drives them, so no finger ever meets a neighbour.
FINGERS = ("index", "middle", "ring", "pinky")
GROUP_TEMPLATE: Tuple[Tuple[str, ...], ...] = (
    ("thumb_cmc",),
    ("thumb_abd",),
    ("thumb_mcp",),
    ("thumb_dip",),
) + tuple(tuple(f"{finger}_{kind}" for finger in FINGERS) for kind in ("abd", "mcp", "pip"))


class Clock:
    """Monotonic time and sleeping, swappable for tests."""

    def __init__(self, now=time.monotonic, sleep=time.sleep):
        self.now = now
        self.sleep = sleep


def finger_joints(config, absent: Sequence[str] = ()) -> List[str]:
    """Every non-wrist joint with a motor, minus ``absent``, in the hand's joint order."""
    skip = set(absent)
    return [
        j
        for j in config.joint_ids
        if j != WRIST and j in config.joint_to_motor_map and j not in skip
    ]


def joint_groups(joints: Sequence[str], parallel: bool = True) -> List[List[str]]:
    """``joints`` split into the sets that are driven together (GROUP_TEMPLATE
    order, members in the order given); every joint on its own when
    ``parallel`` is false. Joints the template does not name run alone."""
    if not parallel:
        return [[j] for j in joints]
    wanted = set(joints)
    groups: List[List[str]] = []
    for template in GROUP_TEMPLATE:
        members = [j for j in joints if j in template and j in wanted]
        if members:
            groups.append(members)
    known = {j for g in groups for j in g}
    groups.extend([j] for j in joints if j not in known)
    return groups


def joint_sense(config, joint: str, direction: int) -> str:
    """Which way the joint goes when its motor turns ``direction``: calibration
    records flex as +1 unless the joint is inverted."""
    flex_sign = -1 if config.joint_inversion_dict.get(joint, False) else 1
    return "flex" if direction == flex_sign else "extend"


def direction_for(config, joint: str, sense: str) -> int:
    """The motor direction that moves ``joint`` in ``sense``."""
    return 1 if joint_sense(config, joint, 1) == sense else -1


def move_to_pose(
    hand,
    joints,
    motors,
    clock: Clock,
    pose: str,
    idx,
    *,
    hold_current_ma: float,
    pose_current_ma: float,
    pose_tolerance_deg: float,
) -> Dict[str, float]:
    """Drive ``joints`` to the start pose (mid-ROM or the config's neutral
    position) at ``pose_current_ma`` (stiff joints need more than the hold
    current), let them settle, then drop back to the hold current. Returns
    the joints that ended further than ``pose_tolerance_deg`` from the pose,
    in motor degrees off."""
    hand._compute_wrap_offsets_dict()
    config = hand.config
    driven = {int(config.joint_to_motor_map[j]) for j in joints}
    hand.set_max_current(
        [pose_current_ma if m in driven else hold_current_ma for m in config.motor_ids]
    )
    hand.enable_torque(motors)
    if pose == POSE_MID:
        roms = config.joint_roms_dict
        target = {j: (float(roms[j][0]) + float(roms[j][1])) / 2 for j in joints}
    else:
        target = {j: float(config.neutral_position.get(j, 0.0)) for j in joints}
    hand.set_joint_positions(target, num_steps=POSE_MOVE_STEPS, step_size=POSE_MOVE_STEP_S)
    clock.sleep(POSE_SETTLE_S)
    short = _pose_shortfall(hand, target, pose_tolerance_deg, idx)
    hand.disable_torque(motors)
    hand.set_max_current([hold_current_ma for _ in config.motor_ids])
    clock.sleep(0.1)
    return short


def _pose_shortfall(hand, target: Dict[str, float], tolerance_deg: float, idx) -> Dict[str, float]:
    """Joints whose motor is further than the tolerance from where the pose
    puts it, by motor degrees — measured on the motors, so it needs no joint
    feedback."""
    try:
        wanted = hand._joint_to_motor_pos(target)
    except Exception:
        return {}
    actual = read_motor_pos_checked(hand)
    out: Dict[str, float] = {}
    for joint in target:
        motor = int(hand.config.joint_to_motor_map[joint])
        goal = wanted[idx[motor]]
        if goal is None:
            continue
        off = abs(math.degrees(float(actual[idx[motor]]) - float(goal)))
        if off > tolerance_deg:
            out[joint] = round(off, 1)
    return out
