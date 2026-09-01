"""Pins the hand-class MRO and the class-level seams it resolves.

The OrcaHandFull diamond is load-bearing: base-tuple order decides which
class serves each seam, and the mock diamond must put the mixin before any
production connect logic. These tests fail loudly if a refactor reorders
the bases or moves a seam.
"""

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
from orca_core.constants import MOTOR_PORT_CLOSE_SETTLE_S, MOTOR_TORQUE_DISABLE_SETTLE_S
from orca_core.hardware.sensing.constants import TACTILE_PORT_OPEN_SETTLE_S
from orca_core.hardware_hand import MockMotorResolutionMixin


def test_full_hand_mro_orders_capabilities_before_bases():
    """Base order decides which class serves each shared seam.

    Only the relative order is pinned, so inserting a mixin stays free while a
    swapped base tuple still fails.
    """
    mro = OrcaHandFull.__mro__
    assert (
        mro.index(OrcaHandTouch)
        < mro.index(OrcaHandJointFeedback)
        < mro.index(OrcaHand)
        < mro.index(BaseHand)
    )


def test_mock_full_hand_resolves_the_mock_mixin_first():
    mro = MockOrcaHandFull.__mro__
    assert (
        mro.index(MockOrcaHandTouch)
        < mro.index(MockOrcaHandJointFeedback)
        < mro.index(MockMotorResolutionMixin)
        < mro.index(OrcaHandFull)
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


def test_all_hand_classes_import_from_package_root():
    for name in (
        "OrcaHand",
        "OrcaHandTouch",
        "OrcaHandJointFeedback",
        "OrcaHandFull",
        "MockOrcaHand",
        "MockOrcaHandTouch",
        "MockOrcaHandJointFeedback",
        "MockOrcaHandFull",
        "JointFeedbackConnectError",
        "BaseHand",
    ):
        assert name in orca_core.__all__
        getattr(orca_core, name)


def test_every_declared_export_is_bound_at_the_package_root():
    """``__all__`` and the actual bindings must not drift apart."""
    for name in orca_core.__all__:
        assert hasattr(orca_core, name), name


@pytest.mark.parametrize(
    "attr, expected",
    [
        ("_port_close_settle_s", MOTOR_PORT_CLOSE_SETTLE_S),
        ("_torque_disable_settle_s", MOTOR_TORQUE_DISABLE_SETTLE_S),
    ],
)
def test_real_hands_keep_their_hardware_settle_waits(attr, expected):
    """Zeroing these on a production class would race a real serial port."""
    assert getattr(OrcaHand, attr) == expected
    assert expected > 0


def test_real_touch_hand_keeps_the_tactile_port_settle():
    assert OrcaHandTouch._tactile_port_open_settle_s == TACTILE_PORT_OPEN_SETTLE_S
    assert TACTILE_PORT_OPEN_SETTLE_S > 0


@pytest.mark.parametrize(
    "cls, attr",
    [
        (MockOrcaHand, "_port_close_settle_s"),
        (MockOrcaHand, "_torque_disable_settle_s"),
        (MockOrcaHandFull, "_port_close_settle_s"),
        (MockOrcaHandFull, "_torque_disable_settle_s"),
        (MockOrcaHandTouch, "_tactile_port_open_settle_s"),
        (MockOrcaHandFull, "_tactile_port_open_settle_s"),
    ],
)
def test_mock_hands_wait_on_nothing(cls, attr):
    """Mock backends have no port to settle; waiting for one only costs time."""
    assert getattr(cls, attr) == 0.0
