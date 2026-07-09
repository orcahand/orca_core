# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Deprecated module path kept for one release.

The joint-feedback classes live in :mod:`orca_core.hardware_hand_sensing`;
import them from there or from the package root.
"""

import warnings

from .hardware_hand import (  # noqa: F401  (importable from here pre-split)
    MockMotorResolutionMixin,
    OrcaHand,
)
from .hardware_hand_sensing import (  # noqa: F401
    JointFeedbackConnectError,
    MockOrcaHandFull,
    MockOrcaHandJointFeedback,
    MockOrcaHandTouch,
    OrcaHandFull,
    OrcaHandJointFeedback,
    OrcaHandTouch,
)

warnings.warn(
    "orca_core.hardware_hand_joint_feedback is deprecated; import from "
    "orca_core (package root) or orca_core.hardware_hand_sensing instead.",
    DeprecationWarning,
    stacklevel=2,
)
