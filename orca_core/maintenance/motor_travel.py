# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Per-joint motor-travel baselines.

The motor rotation between a joint's two hardstops is set by that joint's
tendon spool diameter, so it is a property of the *joint*, not of the motor
that happens to drive it. Baselines therefore live in ``config.yaml`` under
``joint_motor_travel:`` keyed by joint name, and are resolved through
``joint_to_motor_map`` only at the point of use — a hand whose motor IDs were
assigned in a different order still reads the right baseline.

The calibration routine compares each joint's freshly measured travel against
its baseline; a joint that falls short of it stalled before its hardstop
(usually an over-tensioned tendon) and is re-driven at a higher current. See
:mod:`orca_core.maintenance.calibration_routine`.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, TYPE_CHECKING

from ..constants import JOINT_MOTOR_TRAVEL
from ..utils.utils import read_yaml, write_yaml_atomic

if TYPE_CHECKING:
    from ..calibration import CalibrationResult
    from ..hand_config import OrcaHandConfig


def motor_travel_deg(limits: Optional[List]) -> float | None:
    """Motor travel implied by a ``[lower, upper]`` limit pair, in degrees.

    Returns ``None`` when either bound is missing, so a half-calibrated joint
    reads as unmeasured rather than as zero travel.
    """
    if not limits or len(limits) != 2:
        return None
    lower, upper = limits
    if lower is None or upper is None:
        return None
    travel = math.degrees(abs(float(upper) - float(lower)))
    return travel if math.isfinite(travel) else None


def measured_travel_by_joint(
    config: "OrcaHandConfig", calibration: "CalibrationResult"
) -> Dict[str, float]:
    """Motor travel per joint name, in degrees, from recorded motor limits.

    Joints whose motor has no complete limit pair are omitted.
    """
    travel: Dict[str, float] = {}
    for joint, motor_id in config.joint_to_motor_map.items():
        measured = motor_travel_deg(calibration.motor_limits_dict.get(motor_id))
        if measured is not None:
            travel[joint] = measured
    return travel


def travel_deviation(measured: float, expected: float) -> float:
    """Signed fraction ``measured`` deviates from ``expected`` by."""
    return (measured - expected) / expected


def write_joint_motor_travel(
    config_path: str, travel_by_joint: Dict[str, float], *, merge: bool = True
) -> Dict[str, float]:
    """Persist ``joint_motor_travel:`` into ``config.yaml`` and return the block.

    With ``merge`` (the default) the joints named update in place and the rest
    of the stored baseline is kept, so a partial sweep does not erase the
    joints it did not visit. Every other config key keeps its value and its
    position in the file.
    """
    doc = read_yaml(config_path) or {}
    stored = dict(doc.get(JOINT_MOTOR_TRAVEL) or {}) if merge else {}
    stored.update({str(j): round(float(t), 2) for j, t in travel_by_joint.items()})

    # Follow joint_ids so the block reads in the hand's canonical joint order
    # rather than the order the sweep happened to visit.
    order = list(doc.get("joint_ids") or [])
    ordered = {joint: stored[joint] for joint in order if joint in stored}
    ordered.update({j: t for j, t in stored.items() if j not in ordered})

    doc[JOINT_MOTOR_TRAVEL] = ordered
    write_yaml_atomic(config_path, doc)
    return ordered
