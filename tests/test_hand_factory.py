# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import logging
import os
import shutil

import pytest

from orca_core import load_hand
from orca_core.hand_config import HandConfigValidationError, OrcaHandConfig
from orca_core import MockOrcaHand, MockOrcaHandTouch
from orca_core.hardware_hand_sensing import (
    MockOrcaHandFull,
    MockOrcaHandJointFeedback,
)
from orca_core.utils import update_yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "v2")
JOINT_CONFIG = os.path.join(MODELS_DIR, "orcahand-joint-right", "config.yaml")


def _config(name: str) -> str:
    return os.path.join(MODELS_DIR, name, "config.yaml")


# ----- load_hand class-selection matrix -----------------------------------

@pytest.mark.parametrize(
    ("model", "expected"),
    [
        ("orcahand-right", MockOrcaHand),
        ("orcahand-touch-right", MockOrcaHandTouch),
        ("orcahand-joint-right", MockOrcaHandJointFeedback),
        ("orcahand-full-right", MockOrcaHandFull),
    ],
)
def test_load_hand_selects_class_from_config(model, expected):
    hand = load_hand(config_path=_config(model), mock=True)
    assert type(hand) is expected


@pytest.mark.parametrize(
    ("model", "expected"),
    [
        ("orcahand-joint-right", MockOrcaHand),
        ("orcahand-full-right", MockOrcaHandTouch),
        ("orcahand-right", MockOrcaHand),
        ("orcahand-touch-right", MockOrcaHandTouch),
    ],
)
def test_load_hand_engage_feedback_false_returns_motor_only(model, expected):
    hand = load_hand(config_path=_config(model), mock=True, engage_feedback=False)
    assert type(hand) is expected


@pytest.mark.parametrize(
    ("model", "expected", "fallback_model"),
    [
        ("orcahand-joint-left", MockOrcaHandJointFeedback, "orcahand-left"),
        ("orcahand-full-left", MockOrcaHandFull, "orcahand-touch-left"),
    ],
)
def test_load_hand_warns_on_left_feedback_config(
    caplog, model, expected, fallback_model
):
    """Left feedback configs still map to the feedback classes (connect()
    gates the loop), but the factory must warn and name the escape hatches."""
    with caplog.at_level(logging.WARNING, logger="orca_core.hand_factory"):
        hand = load_hand(config_path=_config(model), mock=True)
    assert type(hand) is expected
    messages = [r.getMessage() for r in caplog.records]
    assert any(
        "engage_feedback=False" in m and fallback_model in m for m in messages
    )


@pytest.mark.parametrize(
    "kwargs",
    [
        {"config_path": _config("orcahand-joint-right")},
        {"config_path": _config("orcahand-joint-left"), "engage_feedback": False},
    ],
)
def test_load_hand_does_not_warn_when_loop_can_engage_or_is_suppressed(
    caplog, kwargs
):
    with caplog.at_level(logging.WARNING, logger="orca_core.hand_factory"):
        load_hand(mock=True, **kwargs)
    assert not [r for r in caplog.records if "unvalidated" in r.getMessage()]


def test_load_hand_carries_full_config_when_feedback_suppressed(tmp_path):
    """engage_feedback=False still loads the encoder declaration so the
    calibration encoder pass can run."""
    config_path = tmp_path / "config.yaml"
    shutil.copy(JOINT_CONFIG, config_path)
    hand = load_hand(config_path=str(config_path), mock=True, engage_feedback=False)
    assert type(hand) is MockOrcaHand
    assert hand.config.has_joint_encoders
    assert hand.config.joint_feedback_enabled  # config policy unchanged


def test_load_hand_respects_explicit_feedback_override(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(JOINT_CONFIG, config_path)
    update_yaml(str(config_path), "use_joint_feedback", False)
    hand = load_hand(config_path=str(config_path), mock=True)
    assert type(hand) is MockOrcaHand


# ----- joint_feedback_enabled policy resolution ---------------------------

@pytest.mark.parametrize(
    ("use_joint_feedback", "expected"),
    [
        (None, True),   # unset + encoders present -> auto-on
        (True, True),
        (False, False),  # explicit override -> motor-only
    ],
)
def test_joint_feedback_enabled_with_encoders(tmp_path, use_joint_feedback, expected):
    config_path = tmp_path / "config.yaml"
    shutil.copy(JOINT_CONFIG, config_path)
    update_yaml(str(config_path), "use_joint_feedback", use_joint_feedback)
    config = OrcaHandConfig.from_config_path(config_path=str(config_path))
    assert config.has_joint_encoders is True
    assert config.joint_feedback_enabled is expected


def test_joint_feedback_disabled_without_encoders(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(JOINT_CONFIG, config_path)
    update_yaml(str(config_path), "joint_encoder_joints", None)
    update_yaml(str(config_path), "use_joint_feedback", None)
    config = OrcaHandConfig.from_config_path(config_path=str(config_path))
    assert config.has_joint_encoders is False
    assert config.joint_feedback_enabled is False


def test_use_joint_feedback_true_without_encoders_is_rejected(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(JOINT_CONFIG, config_path)
    update_yaml(str(config_path), "joint_encoder_joints", None)
    update_yaml(str(config_path), "use_joint_feedback", True)
    with pytest.raises(HandConfigValidationError, match="requires joint_encoder_joints"):
        OrcaHandConfig.from_config_path(config_path=str(config_path))
