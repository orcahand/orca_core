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
``dataclasses.asdict()`` serializes one straight into a report.

Ordinary hand users do not need any of this; nothing here is exported from
the top-level ``orca_core`` namespace.
"""

from .encoder_mapping import (
    EncoderMappingResult,
    JointSweep,
    SlotCrosstalk,
    run_encoder_mapping_sweep,
)

__all__ = [
    "EncoderMappingResult",
    "JointSweep",
    "SlotCrosstalk",
    "run_encoder_mapping_sweep",
]
