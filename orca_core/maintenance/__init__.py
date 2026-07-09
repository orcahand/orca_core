# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Hardware maintenance operations shared by the CLI scripts and external tools.

These are interaction-free: they report progress through ``progress_callback``
and request human action through ``prompt_callback``, so the same operation
drives a terminal, a GUI, or a web front-end.
"""

from .calibration_routine import run_calibration
from .tensioning import run_jitter, run_tension
from .motor_chain import (
    ChainScan,
    MotorChainAborted,
    MotorChainError,
    MotorChainPlan,
    change_all_baudrates,
    configure_motor_chain,
    detect_motor_type,
    plan_motor_chain,
    reset_all_motors,
    resolve_port,
    scan_configured_motors,
    valid_baudrates,
)

__all__ = [
    "ChainScan",
    "MotorChainAborted",
    "MotorChainError",
    "MotorChainPlan",
    "change_all_baudrates",
    "configure_motor_chain",
    "detect_motor_type",
    "plan_motor_chain",
    "reset_all_motors",
    "resolve_port",
    "run_calibration",
    "run_jitter",
    "run_tension",
    "scan_configured_motors",
    "valid_baudrates",
]
