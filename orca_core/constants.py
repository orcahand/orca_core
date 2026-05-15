MOTOR_IDS = "motor_ids"
JOINT_IDS = "joint_ids"
JOINT_TO_MOTOR_MAP = "joint_to_motor_map"
JOINT_ROM_DICT = "joint_roms"
JOINT_INVERSION_DICT = "joint_inversion"
MOTOR_LIMITS_DICT = "motor_limits"
MOTOR_TO_JOINT_DICT = "motor_to_joint"
MOTOR_TO_JOINT_RATIOS_DICT = "motor_to_joint_ratios"
MOTOR_TO_JOINT_RATIOS_DICT = "motor_to_joint_ratios"
SUPPORTED_MOTOR_TYPES = ["dynamixel", "feetech"]
JOINT_TO_MOTOR_RATIOS = "joint_to_motor_ratios"
DEFAULT_MODEL_NAME = "orcahand_right"

KNOWN_VIDS: dict[str, list[int]] = {
    "dynamixel": [
        0x0403,  # FTDI (U2D2, most common)
        0x16D0,  # MCS Electronics (some Robotis boards)
    ],
    "feetech": [
        0x1A86,  # QinHeng Electronics CH340 (most Feetech USB adapters)
        0x10C4,  # Silicon Labs CP210x (some Feetech boards)
    ],
    "tactile_sensor": [
        0x28E9,  # Paxini tactile sensor USB adapter
    ],
}

# Baudrates the connect-time probe will try, per motor family, in priority
# order. Used only when ``baudrate`` is not pinned in config.yaml. Feetech
# motors ship at 1M; Dynamixels run at 1M (v2 hands) or 3M (v1 hands).
MOTOR_BAUD_RATES: dict[str, list[int]] = {
    "dynamixel": [1_000_000, 3_000_000],
    "feetech": [1_000_000],
}

"""
Dynamixel specific! TODO(fracapuano): Add feetech control modes too.
PWM (id: 2) control modeis omitted becayse it bypasses PID controllers entirely.
"""

CONTROL_MODES: list[str] = [
    "current_based_position",
    "position",
    "current",
    "velocity",
    "multi_turn_position",
]

CURRENT = "current"
VELOCITY = "velocity"
POSITION = "position"
MULTI_TURN_POSITION = "multi_turn_position"
CURRENT_BASED_POSITION = "current_based_position"

MODE_MAP = {
    CURRENT: 0,
    VELOCITY: 1,
    POSITION: 3,
    MULTI_TURN_POSITION: 4,
    CURRENT_BASED_POSITION: 5,
}

WRIST_MODE_VALUE = 4

WRIST = "wrist"
FLEX = "flex"
EXTEND = "extend"
JOINTS = "joints"
STEP = "step"

TINY_SLEEP = 5e-2

WRIST_CALIBRATED = "wrist_calibrated"
CALIBRATED = "calibrated"
NUM_STEPS = 50
STEP_SIZE = 0.01
