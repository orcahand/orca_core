# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
from .base_hand import BaseHand
from .calibration import CalibrationResult
from .hand_config import BaseHandConfig
from .hand_config import OrcaHandConfig
from .hand_config import OrcaHandTouchConfig
from .hand_config import canonical_joint_ids
from .hardware_hand import MockOrcaHand, OrcaHand
from .hardware_hand_sensing import (
    JointFeedbackConnectError,
    MockOrcaHandFull,
    MockOrcaHandJointFeedback,
    MockOrcaHandTouch,
    OrcaHandFull,
    OrcaHandJointFeedback,
    OrcaHandTouch,
)
from .hand_factory import load_hand
from .hardware.sensing.types import TaxelData
from .joint_position import OrcaJointPositions
from .kinematics import HandKinematics, Transform, frames
from .version import LATEST_VERSION

def __getattr__(name):
    # Keep `orca_core.hardware_hand_joint_feedback` attribute access working
    # for one release; importing the shim module raises its DeprecationWarning.
    if name == "hardware_hand_joint_feedback":
        import importlib

        return importlib.import_module(f".{name}", __name__)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "CalibrationResult",
    "BaseHand",
    "BaseHandConfig",
    "OrcaHandConfig",
    "OrcaHandTouchConfig",
    "OrcaHand",
    "OrcaHandTouch",
    "OrcaHandJointFeedback",
    "OrcaHandFull",
    "MockOrcaHand",
    "MockOrcaHandTouch",
    "MockOrcaHandJointFeedback",
    "MockOrcaHandFull",
    "JointFeedbackConnectError",
    "load_hand",
    "OrcaJointPositions",
    "HandKinematics",
    "TaxelData",
    "Transform",
    "frames",
    "canonical_joint_ids",
    "LATEST_VERSION",
]
