# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Factory that selects the right hand class from a model config.

A hand's capabilities are declared in ``config.yaml``: ``joint_encoder_joints``
plus ``use_joint_feedback`` decide whether the closed-loop joint-feedback
controller engages, and a ``sensors`` block declares tactile sensing.
:func:`load_hand` reads those once and returns the matching concrete class so
callers (and scripts) don't have to hand-pick hand classes.

| feedback | tactile | class                    |
|----------|---------|--------------------------|
| yes       | no      | ``OrcaHandJointFeedback``|
| yes       | yes     | ``OrcaHandFull``         |
| no      | no      | ``OrcaHand``             |
| no      | yes     | ``OrcaHandTouch``        |

"""

from __future__ import annotations

from .hand_config import (
    OrcaHandConfig,
    OrcaHandTouchConfig,
    _resolve_config_path,
)
from .hardware_hand import (
    MockOrcaHand,
    MockOrcaHandTouch,
    OrcaHand,
    OrcaHandTouch,
)
from .hardware_hand_joint_feedback import (
    MockOrcaHandFull,
    MockOrcaHandJointFeedback,
    OrcaHandFull,
    OrcaHandJointFeedback,
)
from .utils.utils import read_yaml


# (feedback, tactile, mock) -> hand class
_CLASS_MATRIX = {
    (True, False, False): OrcaHandJointFeedback,
    (True, True, False): OrcaHandFull,
    (False, False, False): OrcaHand,
    (False, True, False): OrcaHandTouch,
    (True, False, True): MockOrcaHandJointFeedback,
    (True, True, True): MockOrcaHandFull,
    (False, False, True): MockOrcaHand,
    (False, True, True): MockOrcaHandTouch,
}


def load_hand(
    config_path: str | None = None,
    calibration_path: str | None = None,
    model_version: str | None = None,
    model_name: str | None = None,
    mock: bool = False,
    engage_feedback: bool = True,
) -> OrcaHand:
    """Construct the hand class that matches a model's declared capabilities.

    Args:
        config_path: Path to a ``config.yaml``. ``None`` uses the bundled
            default model (optionally narrowed by ``model_version`` /
            ``model_name``).
        calibration_path: Companion ``calibration.yaml``; defaults to the
            sibling of ``config.yaml``.
        model_version, model_name: Select a packaged model when
            ``config_path`` is ``None``.
        mock: Return the in-memory ``Mock*`` variant (no serial I/O).
        engage_feedback: When ``False``, return the motor-only class even if
            the config enables joint feedback. The config still carries the
            encoder declaration so calibration's encoder pass runs.

    Returns:
        A constructed (not yet connected) hand instance.
    """
    resolved_config_path = _resolve_config_path(
        config_path,
        model_version=model_version,
        model_name=model_name,
    )
    raw = read_yaml(resolved_config_path) or {}
    tactile = "sensors" in raw

    config_cls = OrcaHandTouchConfig if tactile else OrcaHandConfig
    config = config_cls.from_config_path(
        config_path=resolved_config_path,
        calibration_path=calibration_path,
    )

    feedback = engage_feedback and config.joint_feedback_enabled

    hand_cls = _CLASS_MATRIX[(bool(feedback), tactile, bool(mock))]
    return hand_cls(config=config)
