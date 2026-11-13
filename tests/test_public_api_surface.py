"""Pins orca_core's public API surface.

orca_core ships to PyPI and is imported by sibling repos (orca_ui, orca_teleop,
orca_stress_tests), so every name below is load-bearing for someone whose code
we cannot see. Removing or renaming one is a breaking change: update these sets
deliberately, bump the minor version, and run scripts/check_downstream.py to
find the callers.
"""

import dataclasses
import inspect

import pytest

import orca_core
from orca_core import (
    BaseHand,
    BaseHandConfig,
    CalibrationResult,
    OrcaHand,
    OrcaHandFull,
    OrcaHandConfig,
    OrcaHandJointFeedback,
    OrcaHandTouch,
    OrcaHandTouchConfig,
    OrcaJointPositions,
)

PACKAGE_EXPORTS = frozenset({
    "BaseHand",
    "BaseHandConfig",
    "CalibrationResult",
    "EncodersNotAvailableError",
    "FingertipHealth",
    "HandConfigValidationError",
    "HandDetection",
    "HandKinematics",
    "JointFeedbackConnectError",
    "JointGains",
    "LATEST_VERSION",
    "LinkHealth",
    "MotorRead",
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
    "TactileBoardError",
    "TaxelData",
    "Transform",
    "canonical_joint_ids",
    "detect_hand",
    "frames",
    "load_config",
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
    "absent_joints",
    "absent_motor_ids",
    "calibrate",
    "calibrate_joint_encoder_manual",
    "calibrated",
    "calibration",
    "connect",
    "disable_torque",
    "disconnect",
    "effective_joint_roms_dict",
    "enable_torque",
    "encoder_backed_joints",
    "get_motor_current",
    "get_motor_pos",
    "get_motor_state",
    "get_motor_temp",
    "clear_fault_record",
    "fault_records",
    "init_joints",
    "poll_faults",
    "is_calibrated",
    "last_read_ok",
    "is_connected",
    "jitter",
    "joint_to_motor_ratios_dict",
    "motor_client",
    "motor_limits_dict",
    "get_servo_gains",
    "get_servo_profile",
    "rom_frame",
    "set_rom_frame",
    "set_control_mode",
    "set_max_current",
    "set_servo_gains",
    "set_servo_profile",
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
    "get_fingertip_health",
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
    "reboot_tactile_board",
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
    "get_pid_gains",
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

CALIBRATION_RESULT = frozenset({
    "empty",
    "from_calibration_path",
})

# Configs are read by every front-end: their fields and methods are API too.
BASE_HAND_CONFIG = frozenset({
    "clamp_joint_positions",
    "config_path",
    "from_config_path",
    "joint_ids",
    "joint_roms_dict",
    "model_path",
    "neutral_position",
    "type",
    "validate",
})

ORCA_HAND_CONFIG = BASE_HAND_CONFIG | {
    "baudrate",
    "calibration_current",
    "calibration_current_for",
    "calibration_current_v21_dict",
    "calibration_group_currents",
    "calibration_max_current",
    "calibration_num_stable",
    "calibration_path",
    "calibration_retry_current",
    "calibration_retry_current_for",
    "calibration_retry_current_resolved",
    "calibration_sequence",
    "calibration_step_period",
    "calibration_step_size",
    "calibration_threshold",
    "calibration_travel_margin",
    "calibration_travel_retries",
    "control_mode",
    "currents_resolved",
    "encoder_baudrate",
    "encoder_serial_port",
    "expected_motor_travel_deg",
    "guided_tension_currents_dict",
    "guided_tension_currents_for",
    "hardware_version",
    "has_joint_encoders",
    "is_v21",
    "joint_encoder_joints",
    "joint_feedback_enabled",
    "joint_gains_all",
    "joint_gains_baseline",
    "joint_gains_dict",
    "joint_gains_for",
    "joint_gains_overrides",
    "joint_inversion_dict",
    "joint_motor_travel_dict",
    "joint_motor_travel_v21_dict",
    "joint_roms_v211_dict",
    "joint_to_motor_map",
    "joint_to_motor_map_v21",
    "manual_calibration_sequence",
    "max_current",
    "measured_joint_roms_dict",
    "motor_id_to_idx_dict",
    "motor_ids",
    "motor_to_joint_dict",
    "motor_type",
    "port",
    "retry_current_for_attempt",
    "retry_steps_by_default",
    "tension_current",
    "tension_hold_current_for",
    "tension_hold_joint_currents",
    "use_joint_feedback",
    "uses_motor_travel",
    "validate_config",
    "validate_control_mode",
    "with_family_currents",
    "with_hardware_version",
    "with_measured_ranges",
    "wrist_calibration_current",
}

ORCA_HAND_TOUCH_CONFIG = ORCA_HAND_CONFIG | {
    "finger_to_sensor_id",
    "sensor_baudrate",
    "sensor_port",
}


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
        (CalibrationResult, CALIBRATION_RESULT),
    ],
    ids=lambda value: getattr(value, "__name__", ""),
)
def test_public_method_surface_is_pinned(cls, expected):
    assert public_names(cls) == expected


@pytest.mark.parametrize(
    "cls, expected",
    [
        (BaseHandConfig, BASE_HAND_CONFIG),
        (OrcaHandConfig, ORCA_HAND_CONFIG),
        (OrcaHandTouchConfig, ORCA_HAND_TOUCH_CONFIG),
    ],
    ids=lambda value: getattr(value, "__name__", ""),
)
def test_config_surface_is_pinned(cls, expected):
    fields = frozenset(f.name for f in dataclasses.fields(cls))
    assert public_names(cls) | fields == expected


def test_full_hand_adds_nothing_beyond_its_capabilities():
    """OrcaHandFull is exactly the union of its two capability mixins."""
    touch_only = public_names(OrcaHandTouch) - public_names(OrcaHand)
    feedback_only = public_names(OrcaHandJointFeedback) - public_names(OrcaHand)
    assert public_names(OrcaHandFull) == public_names(OrcaHand) | touch_only | feedback_only
