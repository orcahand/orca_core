"""Pins orca_core's public API surface.

orca_core ships to PyPI and is imported by sibling repos (orca_ui, orca_teleop,
orca_stress_tests), so every name below is load-bearing for someone whose code
we cannot see. Removing or renaming one is a breaking change: update these sets
deliberately, bump the minor version, and run scripts/check_downstream.py to
find the callers.
"""

import inspect

import pytest

import orca_core
from orca_core import (
    BaseHand,
    OrcaHand,
    OrcaHandFull,
    OrcaHandJointFeedback,
    OrcaHandTouch,
    OrcaJointPositions,
)

PACKAGE_EXPORTS = frozenset({
    "BaseHand",
    "BaseHandConfig",
    "CalibrationResult",
    "EncodersNotAvailableError",
    "HandConfigValidationError",
    "HandDetection",
    "HandKinematics",
    "JointFeedbackConnectError",
    "LATEST_VERSION",
    "LinkHealth",
    "MockOrcaHand",
    "MockOrcaHandFull",
    "MockOrcaHandJointFeedback",
    "MockOrcaHandTouch",
    "OrcaHand",
    "OrcaHandConfig",
    "OrcaHandFull",
    "OrcaHandJointFeedback",
    "OrcaHandTouch",
    "OrcaHandTouchConfig",
    "OrcaJointPositions",
    "TaxelData",
    "Transform",
    "canonical_joint_ids",
    "detect_hand",
    "frames",
    "load_hand",
})

BASE_HAND = frozenset({
    "config_cls",
    "get_joint_position",
    "play_named_positions",
    "pose_from_fractions",
    "register_position",
    "remove_position",
    "set_joint_positions",
    "set_named_position",
    "set_neutral_position",
    "set_zero_position",
})

ORCA_HAND = BASE_HAND | {
    "calibrate",
    "calibrated",
    "calibration",
    "connect",
    "disable_torque",
    "disconnect",
    "enable_torque",
    "encoder_backed_joints",
    "get_motor_current",
    "get_motor_pos",
    "get_motor_temp",
    "init_joints",
    "is_calibrated",
    "is_connected",
    "jitter",
    "joint_to_motor_ratios_dict",
    "motor_client",
    "motor_limits_dict",
    "set_control_mode",
    "set_max_current",
    "stop_task",
    "task_running",
    "tension",
    "wait_for_motion",
    "wrist_calibrated",
    "write_motor_pos",
}

TACTILE_SURFACE = frozenset({
    "clear_tactile_zero",
    "connect_sensors_only",
    "get_base_pose",
    "get_sensor_transforms",
    "get_tactile_configuration",
    "get_tactile_data",
    "get_tactile_forces",
    "get_tactile_link_health",
    "get_tactile_stats",
    "get_tactile_taxels",
    "get_taxel_data",
    "get_taxel_geometry",
    "kinematics",
    "set_base_pose",
    "start_tactile_stream",
    "stop_tactile_stream",
    "zero_tactile_sensors",
})

JOINT_FEEDBACK_SURFACE = frozenset({
    "get_encoder_link_health",
    "get_loop_correction",
    "get_loop_stats",
    "get_measured_joints",
    "loop_joint_names",
    "loop_skipped_joints",
    "rebase_loop",
    "set_pid_gains",
})

JOINT_POSITIONS = frozenset({
    "as_array",
    "as_dict",
    "as_list",
    "from_dict",
    "from_ndarray",
    "register_joint_names",
})


def public_names(cls) -> frozenset:
    return frozenset(name for name, _ in inspect.getmembers(cls) if not name.startswith("_"))


def test_package_exports_are_pinned():
    assert set(orca_core.__all__) == PACKAGE_EXPORTS


@pytest.mark.parametrize(
    "cls, expected",
    [
        (BaseHand, BASE_HAND),
        (OrcaHand, ORCA_HAND),
        (OrcaHandTouch, ORCA_HAND | TACTILE_SURFACE),
        (OrcaHandJointFeedback, ORCA_HAND | JOINT_FEEDBACK_SURFACE),
        (OrcaHandFull, ORCA_HAND | TACTILE_SURFACE | JOINT_FEEDBACK_SURFACE),
        (OrcaJointPositions, JOINT_POSITIONS),
    ],
    ids=lambda value: getattr(value, "__name__", ""),
)
def test_public_method_surface_is_pinned(cls, expected):
    assert public_names(cls) == expected


def test_full_hand_adds_nothing_beyond_its_capabilities():
    """OrcaHandFull is exactly the union of its two capability mixins."""
    touch_only = public_names(OrcaHandTouch) - public_names(OrcaHand)
    feedback_only = public_names(OrcaHandJointFeedback) - public_names(OrcaHand)
    assert public_names(OrcaHandFull) == public_names(OrcaHand) | touch_only | feedback_only
