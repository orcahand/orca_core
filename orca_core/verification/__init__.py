# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Producer-bench verification routines for an ORCA hand.

These measure a hand and report; unlike ``orca_core.maintenance``, none of
them changes the hand's persisted state. Every routine restores what it
touched — calibration, pose, torque — on every exit path, so a hand is left
exactly as it was found whether the routine completed, was stopped, or raised.

They follow the same contract as the maintenance routines
(``progress_callback(dict)``, ``should_stop()``), so a terminal and a test
station drive them identically. Thresholds are keyword parameters, never
fixed constants, and every result echoes the thresholds it applied: a limit
set from fleet data lives with the test plan, not in this package, and a
stored result stays re-judgeable when that limit moves.

Result dataclasses hold plain scalars, lists, and dicts, so
``dataclasses.asdict()`` serializes one straight into a report. Each stays as
rich as its own domain needs while implementing :class:`StepResult`, the
narrow common face a plan runner and a report see.

Ordinary hand users do not need any of this; nothing here is exported from
the top-level ``orca_core`` namespace.
"""

from .anchor_repeatability import (
    AnchorRepeatabilityResult,
    JointAnchorStats,
    run_anchor_repeatability,
)
from .as_shipped import AsShippedResult, run_as_shipped_state
from .calibration_consistency import (
    CalibrationConsistencyResult,
    JointLimitDelta,
    run_calibration_consistency,
)
from .calibration_step import (
    CalibrationStepResult,
    JointCalibration,
    Traverse,
    run_calibration_step,
)
from .encoder_mapping import (
    EncoderMappingResult,
    JointSweep,
    SlotCrosstalk,
    run_encoder_mapping_sweep,
)
from .identity import (
    IdentityInventoryResult,
    PortsBusyError,
    run_identity_inventory,
)
from .motion_soak import MotionSoakResult, MotorSoak, run_motion_soak
from .motor_health import MotorHealth, MotorHealthResult, run_motor_health
from .step_result import StepResult, flat_measurements
from .tendon_friction import (
    TendonFrictionResult,
    TraverseFriction,
    analyse_tendon_friction,
)
from .tensioning_step import MotorWind, TensioningResult, run_tensioning
from .visual_inspection import VisualInspectionResult, run_visual_inspection

__all__ = [
    "AnchorRepeatabilityResult",
    "AsShippedResult",
    "CalibrationConsistencyResult",
    "CalibrationStepResult",
    "EncoderMappingResult",
    "IdentityInventoryResult",
    "JointAnchorStats",
    "JointCalibration",
    "JointLimitDelta",
    "JointSweep",
    "MotionSoakResult",
    "MotorHealth",
    "MotorHealthResult",
    "MotorSoak",
    "MotorWind",
    "PortsBusyError",
    "SlotCrosstalk",
    "StepResult",
    "TendonFrictionResult",
    "TensioningResult",
    "Traverse",
    "TraverseFriction",
    "VisualInspectionResult",
    "analyse_tendon_friction",
    "flat_measurements",
    "run_anchor_repeatability",
    "run_as_shipped_state",
    "run_calibration_consistency",
    "run_calibration_step",
    "run_encoder_mapping_sweep",
    "run_identity_inventory",
    "run_motion_soak",
    "run_motor_health",
    "run_tensioning",
    "run_visual_inspection",
]
