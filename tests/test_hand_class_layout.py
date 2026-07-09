"""Pins the hand-class MRO and the deprecated module paths.

The OrcaHandFull diamond is load-bearing: base-tuple order decides which
class serves each seam, and the mock diamond must put the mixin before any
production connect logic. These tests fail loudly if a refactor reorders
the bases or moves a seam.
"""

import importlib
import sys

import pytest

import orca_core
from orca_core import (
    MockOrcaHand,
    MockOrcaHandFull,
    MockOrcaHandJointFeedback,
    MockOrcaHandTouch,
    OrcaHand,
    OrcaHandFull,
    OrcaHandJointFeedback,
    OrcaHandTouch,
)
from orca_core.base_hand import BaseHand
from orca_core.hardware_hand import MockMotorResolutionMixin


def test_full_hand_mro_is_pinned():
    assert OrcaHandFull.__mro__ == (
        OrcaHandFull,
        OrcaHandTouch,
        OrcaHandJointFeedback,
        OrcaHand,
        BaseHand,
        *BaseHand.__mro__[1:],
    )


def test_mock_full_hand_mro_is_pinned():
    assert MockOrcaHandFull.__mro__ == (
        MockOrcaHandFull,
        MockOrcaHandTouch,
        MockOrcaHandJointFeedback,
        MockMotorResolutionMixin,
        OrcaHandFull,
        OrcaHandTouch,
        OrcaHandJointFeedback,
        OrcaHand,
        BaseHand,
        *BaseHand.__mro__[1:],
    )


@pytest.mark.parametrize(
    "cls, method, expected_owner",
    [
        # Full owns the two-stream orchestration; each capability serves its seam.
        (OrcaHandFull, "connect", OrcaHandFull),
        (OrcaHandFull, "disconnect", OrcaHandFull),
        (OrcaHandFull, "_set_joint_positions", OrcaHandJointFeedback),
        (OrcaHandFull, "_get_joint_positions", OrcaHandJointFeedback),
        (OrcaHandFull, "get_tactile_forces", OrcaHandTouch),
        (OrcaHandFull, "_attach_tactile_client", OrcaHandTouch),
        (OrcaHandFull, "calibrate", OrcaHandJointFeedback),
        # The mock mixin must interpose before any production connect logic.
        (MockOrcaHandFull, "_create_motor_client", MockMotorResolutionMixin),
        (MockOrcaHandFull, "_resolve_motor_driver", MockMotorResolutionMixin),
        (MockOrcaHandFull, "_create_tactile_link", MockOrcaHandTouch),
        (MockOrcaHandFull, "_create_encoder_link", MockOrcaHandJointFeedback),
        (MockOrcaHandFull, "connect", MockMotorResolutionMixin),
        (MockOrcaHand, "_create_motor_client", MockMotorResolutionMixin),
    ],
)
def test_seam_resolves_to_expected_class(cls, method, expected_owner):
    assert getattr(cls, method) is getattr(expected_owner, method)


def test_joint_feedback_module_shim_warns_and_reexports():
    sys.modules.pop("orca_core.hardware_hand_joint_feedback", None)
    with pytest.warns(DeprecationWarning):
        shim = importlib.import_module("orca_core.hardware_hand_joint_feedback")
    assert shim.OrcaHandFull is OrcaHandFull
    assert shim.OrcaHandJointFeedback is OrcaHandJointFeedback
    assert shim.MockOrcaHandFull is MockOrcaHandFull
    assert shim.MockOrcaHandJointFeedback is MockOrcaHandJointFeedback
    assert shim.JointFeedbackConnectError is orca_core.JointFeedbackConnectError


def test_moved_touch_names_warn_and_reexport():
    import orca_core.hardware_hand as hardware_hand

    with pytest.warns(DeprecationWarning):
        assert hardware_hand.OrcaHandTouch is OrcaHandTouch
    with pytest.warns(DeprecationWarning):
        assert hardware_hand.MockOrcaHandTouch is MockOrcaHandTouch
    with pytest.raises(AttributeError):
        hardware_hand.NoSuchName
