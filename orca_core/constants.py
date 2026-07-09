from typing import Literal

FingerName = Literal["thumb", "index", "middle", "ring", "pinky"]
"""Type alias for valid finger names. Use in public APIs that take a single
finger name so type checkers flag typos like ``reading["thmub"]``."""

FINGER_NAMES: list[FingerName] = ["thumb", "index", "middle", "ring", "pinky"]

MOTOR_IDS = "motor_ids"
JOINT_IDS = "joint_ids"
JOINT_TO_MOTOR_MAP = "joint_to_motor_map"
JOINT_ROM_DICT = "joint_roms"
JOINT_INVERSION_DICT = "joint_inversion"
MOTOR_LIMITS_DICT = "motor_limits"
MOTOR_TO_JOINT_DICT = "motor_to_joint"
MOTOR_TO_JOINT_RATIOS_DICT = "motor_to_joint_ratios"

DYNAMIXEL = "dynamixel"
FEETECH = "feetech"
SUPPORTED_MOTOR_TYPES = [DYNAMIXEL, FEETECH]

DEFAULT_MOTOR_BAUDRATE = 1_000_000
JOINT_TO_MOTOR_RATIOS = "joint_to_motor_ratios"
JOINT_ENCODER_CALIBRATION = "joint_encoder_calibration"
DEFAULT_MODEL_NAME = "orcahand-right"

KNOWN_VIDS: dict[str, list[int]] = {
    DYNAMIXEL: [
        0x0403,  # FTDI (U2D2, most common)
        0x16D0,  # MCS Electronics (some Robotis boards)
    ],
    FEETECH: [
        0x1A86,  # QinHeng Electronics CH340 (most Feetech USB adapters)
        0x10C4,  # Silicon Labs CP210x (some Feetech boards)
    ],
    "tactile_sensor": [
        0x28E9,  # Paxini tactile sensor USB adapter
    ],
    "oh_board": [
        0x2F5D,  # ORCA Dexterity OH board (dual-CDC; PID 0x2202)
    ],
}

# OH board exposes two CDCs sharing VID/PID; ORCA_ID_QUERY lets the host
# distinguish motor vs sensor. ORCA_INFO_QUERY additionally returns the hand
# identity ("ORCA:<role>;SIDE=<L|R>;HW=<n>;FW=<n>;SN=<hex>"); boards that
# predate it stay silent. ORCA_SET_QUERY provisions side/hardware version.
ORCA_ID_QUERY = b"ORCA_ID?\n"
ORCA_ID_RESP_MOTOR = b"ORCA:MOTOR\n"
ORCA_ID_RESP_SENSOR = b"ORCA:SENSOR\n"
ORCA_INFO_QUERY = b"ORCA_INFO?\n"
ORCA_INFO_MARKER_MOTOR = b"ORCA:MOTOR;"
ORCA_INFO_MARKER_SENSOR = b"ORCA:SENSOR;"
ORCA_SET_RESP_OK = b"ORCA:OK\n"
ORCA_SET_RESP_ERR = b"ORCA:ERR\n"
ORCA_ID_PROBE_TIMEOUT_S = 0.2
ORCA_ID_PROBE_BAUDRATE = 921600

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
FINGER = "finger"
FLEX = "flex"
EXTEND = "extend"
JOINTS = "joints"
STEP = "step"

MOTOR_MODELS: dict[str, dict[str, str]] = {
    DYNAMIXEL: {WRIST: "XC430", FINGER: "XC330"},
    FEETECH: {WRIST: "HLS3930", FINGER: "HLS3915"},
}

TINY_SLEEP = 5e-2

WRIST_CALIBRATED = "wrist_calibrated"
CALIBRATED = "calibrated"
STEPS_TO_NEUTRAL = 25
STEP_SIZE_NEUTRAL = 0.001

# Defaults for interpolated multi-step motions (BaseHand.set_joint_positions).
NUM_STEPS = 50
STEP_SIZE = 0.01

# Baudrates the connect-time probe tries per motor family, in priority order,
# when ``baudrate`` is not pinned in config.yaml.
MOTOR_BAUD_RATES: dict[str, list[int]] = {
    DYNAMIXEL: [1_000_000, 3_000_000],
    FEETECH: [1_000_000],
}
