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

JOINT_TO_MOTOR_RATIOS = "joint_to_motor_ratios"
JOINT_ENCODER_CALIBRATION = "joint_encoder_calibration"
JOINT_ROMS_MEASURED = "joint_roms_measured"
JOINT_MOTOR_TRAVEL = "joint_motor_travel"
MOTOR_TRAVEL_MEASURED = "motor_travel_measured"
DEFAULT_MODEL_NAME = "orcahand-right"

# Current the calibration routine retries a short-travel joint at, as a
# multiple of ``calibration_current``, when config.yaml pins no explicit
# ``calibration_retry_current``.
DEFAULT_RETRY_CURRENT_SCALE = 1.5

# Ceiling the calibration short-travel re-drive may escalate to (mA).
# Deliberately independent of ``max_current``: the re-drive needs more
# torque than normal operation to break a joint past an over-tensioned
# tendon, and it is bounded in time — the routine restores max_current
# the moment calibration ends, so nothing else ever sees this current.
DEFAULT_CALIBRATION_MAX_CURRENT = 500

# A calibration sweep that ends with its two motor limits this close together
# did not find two hardstops: the motor never turned. Committing such a pair
# writes a zero (or near-zero) joint-to-motor ratio, which silently makes the
# joint uncommandable, and leaves the wrap detector comparing live positions
# against a single point — so the corruption survives into every later run.
# Rejected instead, leaving the joint's previous calibration in place.
MIN_MOTOR_TRAVEL_RAD = 0.05  # ~2.9 deg of motor shaft

# Wall-clock backstop for one drive step. A motor whose tendon is disconnected
# spins freely, never stabilises, and would otherwise drive until stopped.
DRIVE_STEP_TIMEOUT_S = 30.0

# Fraction of its ``joint_motor_travel`` baseline below which a joint is
# treated as not having moved at all rather than as short-travelled. Raising
# the current cannot help a joint that never left its starting point, so the
# re-drive is skipped and the limits are rejected.
MIN_TRAVEL_FRACTION = 0.1

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
        0x2F5D,  # ORCA Dexterity hand controller board (dual-CDC; PID 0x2202)
    ],
}

# The hand's controller board exposes two CDCs sharing VID/PID; ORCA_ID_QUERY
# lets the host distinguish motor vs sensor.
ORCA_ID_QUERY = b"ORCA_ID?\n"
ORCA_ID_RESP_MOTOR = b"ORCA:MOTOR\n"
ORCA_ID_RESP_SENSOR = b"ORCA:SENSOR\n"
ORCA_INFO_QUERY = b"ORCA_INFO?\n"
"""Identity query; the reply is
``ORCA:<role>;SIDE=<L|R>;HW=<n>;FW=<n>;SN=<serial>;BID=<hex>``.
Boards that predate it stay silent."""
ORCA_INFO_MARKER_MOTOR = b"ORCA:MOTOR;"
ORCA_INFO_MARKER_SENSOR = b"ORCA:SENSOR;"
ORCA_ID_PROBE_TIMEOUT_S = 0.2
ORCA_ID_PROBE_BAUDRATE = 921600

MOTOR_PORT_CLOSE_SETTLE_S = 0.5
"""Pause after closing the motor serial port before it's safe to reopen. Some
USB-CDC adapters (observed on CH340/CH343-family chips) briefly deny a reopen
right after close; without this, a health-check-triggered reconnect can lose
that race and get stuck denied on every subsequent attempt within the same
process, since nothing else ever prompts the OS to release the handle."""

MOTOR_TORQUE_DISABLE_SETTLE_S = 0.1
"""Let the torque-disable write land before the port closes under it."""

# The union of control modes across motor families; a family accepts only the
# subset in its client's ``supported_modes``. PWM control mode (id 2) is
# omitted because it bypasses PID controllers entirely.
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

# Defaults for interpolated multi-step motions (set_neutral_position,
# set_zero_position, and scripts).
NUM_STEPS = 50
STEP_SIZE = 0.01

# Baudrates the connect-time probe tries per motor family, in priority order,
# when ``baudrate`` is not pinned in config.yaml.
MOTOR_BAUD_RATES: dict[str, list[int]] = {
    DYNAMIXEL: [1_000_000, 3_000_000],
    FEETECH: [1_000_000],
}
