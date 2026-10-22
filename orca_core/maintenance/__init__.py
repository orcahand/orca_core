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
from .guided_tension import read_last_tension, run_guided_tension
from .motor_travel import (
    measured_travel_by_joint,
    motor_travel_deg,
    travel_deviation,
    write_joint_motor_travel,
)
from .tensioning import run_jitter, run_tension
from .slack_test import SlackTestSettings, run_slack_test
from .tension_sweep import SweepSettings, run_tension_sweep
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
    set_all_return_delays,
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
    "measured_travel_by_joint",
    "motor_travel_deg",
    "plan_motor_chain",
    "read_last_tension",
    "reset_all_motors",
    "resolve_port",
    "run_calibration",
    "run_guided_tension",
    "run_jitter",
    "run_tension",
    "run_slack_test",
    "run_tension_sweep",
    "SlackTestSettings",
    "SweepSettings",
    "scan_configured_motors",
    "set_all_return_delays",
    "travel_deviation",
    "valid_baudrates",
    "write_joint_motor_travel",
]
