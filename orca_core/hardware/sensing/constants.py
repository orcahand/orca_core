"""Constants for ORCA tactile sensing."""

from typing import Literal

# ---------------------------------------------------------------------------
# Client configuration defaults
# ---------------------------------------------------------------------------

FingerName = Literal["thumb", "index", "middle", "ring", "pinky"]
"""Type alias for valid finger names. Use in public APIs that take a single
finger name so type checkers flag typos like ``reading["thmub"]``."""

FINGER_NAMES: list[FingerName] = ["thumb", "index", "middle", "ring", "pinky"]
VALID_SENSOR_IDS = set(range(5))
DEFAULT_SENSOR_PORT = "/dev/ttyACM1"
DEFAULT_SENSOR_BAUDRATE = 921600
DEFAULT_FINGER_TO_SENSOR_ID = {
    "thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4,
}

# Default taxel counts per finger (must match sensor model configs)
DEFAULT_TAXEL_COUNTS = {
    "thumb": 51, "index": 87, "middle": 87, "ring": 87, "pinky": 51,
}

FINGER_MODELS = {
    "thumb": "touch-sensor-thumb",
    "index": "touch-sensor-finger",
    "middle": "touch-sensor-finger",
    "ring": "touch-sensor-finger",
    "pinky": "touch-sensor-pinky",
}

# ---------------------------------------------------------------------------
# Protocol wire format (shared between I/O layer and codec)
# ---------------------------------------------------------------------------

PROTOCOL_HEADER_REQUEST = bytes([0x55, 0xAA])
PROTOCOL_HEADER_RESPONSE = bytes([0xAA, 0x55])
PROTOCOL_HEADER_AUTO = bytes([0xAA, 0x56])
PROTOCOL_RESERVED = 0x00
FUNC_CODE_READ = 0x03
FUNC_CODE_WRITE = 0x10

# ---------------------------------------------------------------------------
# Register addresses (used by tactile_client for read/write targets)
# ---------------------------------------------------------------------------

ADDR_RESET = 0x0022
ADDR_CONNECTED_SENSORS_START = 0x0010
ADDR_CONNECTED_SENSORS_LENGTH = 4
ADDR_NUM_TAXELS_START = 0x0030
ADDR_NUM_TAXELS_LENGTH = 56
ADDR_RESULTANT_FORCE_START = 0x0500
RESULTANT_BLOCK_SIZE = 168
ADDR_AUTO_DATA_TYPE = 0x0016
ADDR_AUTO_ENABLE = 0x0017

# Register write values
REGISTER_ENABLE = bytes([0x01])
REGISTER_DISABLE = bytes([0x00])

# ---------------------------------------------------------------------------
# Codec internals (used by protocol.py decoders)
# ---------------------------------------------------------------------------

# Auto data type bitmasks
AUTO_DATA_RESULTANT = 0x01
AUTO_DATA_TAXELS = 0x02

# Force resolution
RESOLUTION_N_PER_LSB = 0.1

# Byte sizes per data element
BYTES_PER_RESULTANT = 6  # 3 axes × 2-byte slot (low byte = data, high byte = padding)
BYTES_PER_TAXEL = 3      # fx(int8) + fy(int8) + fz(uint8)

# Frame metadata sizes
RESPONSE_META_SIZE = 6
"""Bytes between response header and variable data: reserved(1) + func(1) + addr(2) + count/nbytes(2)."""

AUTO_FRAME_META_SIZE = 3
"""Bytes between auto-stream header and payload: reserved(1) + eff_len(2)."""

# Minimum valid frame sizes
MIN_READ_RESPONSE_SIZE = 9
"""Minimum valid read response frame: header(2) + meta(6) + LRC(1)."""

MIN_WRITE_RESPONSE_SIZE = 9
"""Minimum valid write response frame: header(2) + meta(6) + LRC(1)."""

# Maximum valid effective frame length in auto-stream frames.
# Typical payloads are 6-200 bytes; this guards against corrupted eff_len fields.
MAX_AUTO_FRAME_EFF_LEN = 8192

# Register block structure
MODULES_PER_SLOT = 4
"""Modules per sensor slot in the resultant force register block (proximal, middle, distal, nail)."""

DISTAL_MODULE_OFFSET = 2
"""Offset of the distal phalanx module within a slot's module group."""

# Slot bit positions (byte_idx, bit_pos) in the connected-sensors register.
# Hardware-fixed; finger mapping is applied on top via finger_to_sensor_id.
SLOT_CONNECTED_BIT_POSITIONS = [(0, 2), (0, 6), (1, 2), (1, 6), (2, 2)]

# Register address per slot for distal-phalanx taxel count. Hardware-fixed.
SLOT_DISTAL_TAXEL_REGISTER_OFFSETS = [0x0034, 0x003C, 0x0044, 0x004C, 0x0054]
