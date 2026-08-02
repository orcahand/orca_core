"""Constants for ORCA tactile sensing and joint encoders."""

# ---------------------------------------------------------------------------
# Baud-rate defaults (the motor-bus default lives in orca_core/constants.py)
# ---------------------------------------------------------------------------

DEFAULT_SENSOR_BAUDRATE = 921600
"""Tactile link on its own adapter; the ``sensors.baudrate`` config key overrides."""

DEFAULT_ENCODER_BAUDRATE = 2_000_000
"""Joint-encoder link, also carrying tactile when both share one port; the
``encoder_baudrate`` config key overrides."""

# ---------------------------------------------------------------------------
# Serial discovery
# ---------------------------------------------------------------------------

FTDI_VID = 0x0403
"""USB vendor ID for FTDI USB-serial adapters, used as a discovery fallback
(see serial_discovery.detect_encoder_stream)."""

# ---------------------------------------------------------------------------
# Client configuration defaults
# ---------------------------------------------------------------------------

VALID_SENSOR_IDS = set(range(5))
DEFAULT_SENSOR_PORT = "auto"
DEFAULT_FINGER_TO_SENSOR_ID = {
    "thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4,
}

# Default taxel counts per finger (must match sensor model configs)
DEFAULT_TAXEL_COUNTS = {
    "thumb": 51, "index": 87, "middle": 87, "ring": 87, "pinky": 51,
}

# Sensor model (taxel-geometry YAML under sensing/models/) per finger.
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

# Frame-type tag
PROTOCOL_BYTE_RESPONSE = PROTOCOL_HEADER_RESPONSE[1]  # 0x55
PROTOCOL_BYTE_AUTO = PROTOCOL_HEADER_AUTO[1]          # 0x56

# ---------------------------------------------------------------------------
# Register addresses (used by tactile_client for read/write targets)
# ---------------------------------------------------------------------------

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

# Decimal places to round returned force values to (matches RESOLUTION_N_PER_LSB granularity).
FORCE_ROUND_DECIMALS = 1

# Decimal places for averaged zeroing offsets: finer than the sensor's 0.1 N
# resolution to avoid bias; subtracted forces still round to FORCE_ROUND_DECIMALS.
OFFSET_CAPTURE_DECIMALS = 2

# Byte sizes per data element
BYTES_PER_RESULTANT = 6  # 3 axes × 2-byte slot (low byte = data, high byte = padding)
BYTES_PER_TAXEL = 3      # fx(int8) + fy(int8) + fz(uint8)

# Frame metadata sizes
RESPONSE_META_SIZE = 6
"""Bytes between response header and variable data: reserved(1) + func(1) + addr(2) + count/nbytes(2)."""

AUTO_FRAME_META_SIZE = 3
"""Bytes between auto-stream header and payload: reserved(1) + effective_length(2)."""

# Minimum valid frame sizes
MIN_READ_RESPONSE_SIZE = 9
"""Minimum valid read response frame: header(2) + meta(6) + LRC(1)."""

MIN_WRITE_RESPONSE_SIZE = 9
"""Minimum valid write response frame: header(2) + meta(6) + LRC(1)."""

# Maximum valid effective frame length in auto-stream frames.
# Typical payloads are 6-200 bytes; this guards against corrupted effective_length fields.
MAX_AUTO_FRAME_EFFECTIVE_LENGTH = 8192

# Same bound applied to the ``count`` field in AA 55 register responses — a
# higher value is treated as corruption rather than a multi-megabyte read.
MAX_RESPONSE_DATA_LEN = MAX_AUTO_FRAME_EFFECTIVE_LENGTH

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

# ---------------------------------------------------------------------------
# Encoder auto-stream
# ---------------------------------------------------------------------------

PROTOCOL_HEADER_AUTO_ENC = bytes([0xAA, 0xA9])
PROTOCOL_BYTE_AUTO_ENC = PROTOCOL_HEADER_AUTO_ENC[1]  # 0xA9

AUTO_ENC_NUM_JOINTS = 17
"""16 finger-joint encoders + 1 wrist slot"""

AUTO_ENC_PAYLOAD_BYTES = AUTO_ENC_NUM_JOINTS * 2
AUTO_ENC_PAYLOAD_OFFSET = 6
"""Byte offset to the joint payload (past header + reserved + effective_length + error_byte)."""

AUTO_ENC_EFFECTIVE_LENGTH = 1 + AUTO_ENC_PAYLOAD_BYTES
"""Value of the effective_length field for a valid encoder frame: error byte + payload."""

AUTO_ENC_FRAME_SIZE = AUTO_ENC_PAYLOAD_OFFSET + AUTO_ENC_PAYLOAD_BYTES + 1
"""Wire size: header(2) + reserved(1) + effective_length(2) + error_byte(1) + payload + LRC(1)."""

# Per-joint u16 layout in the encoder payload
AUTO_ENC_PARITY_BIT = 1 << 15
AUTO_ENC_ANGLE_ERROR_BIT = 1 << 14
AUTO_ENC_ANGLE_MASK = 0x3FFF

# Encoder hardware properties (14-bit absolute rotary encoder)
ENCODER_COUNTS_PER_REV = 16384
ENCODER_LSB_DEG = 360.0 / ENCODER_COUNTS_PER_REV

# Joint name → encoder slot.
JOINT_TO_ENCODER_SLOT = {
    "thumb_dip":  0, "thumb_mcp":  1, "thumb_abd":  2, "thumb_cmc":  3,
    "index_pip":  4, "index_mcp":  5, "index_abd":  6,
    "middle_pip": 7, "middle_mcp": 8, "middle_abd": 9,
    "ring_pip":  10, "ring_mcp":  11, "ring_abd":  12,
    "pinky_pip": 13, "pinky_mcp": 14, "pinky_abd": 15,
    "wrist":     16,
}
ENCODER_SLOT_TO_JOINT = {v: k for k, v in JOINT_TO_ENCODER_SLOT.items()}

# Sentinel for ``config.joint_encoder_joints``: a single ``"all"`` entry selects every
# slotted, motor-driven joint (the sensing-hand default); an explicit list narrows it.
ENCODER_JOINTS_ALL = "all"

# Per-joint encoder polarity, fixed by encoder mounting + magnet orientation.
# Validated on right-hand assemblies only; mirroring changes mounting senses.
JOINT_ENCODER_POLARITY = {
    "thumb_cmc": -1, "thumb_abd": -1, "thumb_mcp": 1, "thumb_dip": -1,
    "index_abd": 1, "index_mcp": 1, "index_pip": -1,
    "middle_abd": 1, "middle_mcp": 1, "middle_pip": -1,
    "ring_abd": 1, "ring_mcp": 1, "ring_pip": -1,
    "pinky_abd": 1, "pinky_mcp": 1, "pinky_pip": -1,
    "wrist": 1,
}

# Polarity tables by hand side; a side is absent until its table is validated
# on hardware ("left" deliberately has none yet).
JOINT_ENCODER_POLARITY_BY_SIDE = {"right": JOINT_ENCODER_POLARITY}


def joint_encoder_polarity_for_side(side):
    """Per-joint encoder polarity table for hands of ``side``.

    Raises ``ValueError`` for sides without a validated table (currently
    everything but ``"right"``), so closed-loop consumers fail loudly
    instead of decoding with wrong signs.
    """
    try:
        return JOINT_ENCODER_POLARITY_BY_SIDE[side]
    except KeyError:
        raise ValueError(
            f"hand side {side!r} (config field 'type') has no validated "
            f"joint-encoder polarity table, so encoder angles cannot be "
            f"decoded. Set 'type:' in config.yaml to one of "
            f"{sorted(JOINT_ENCODER_POLARITY_BY_SIDE)}."
        ) from None

# Slots the production hand ships with encoders wired (all 17, wrist included).
# Diagnostics treat a stuck slot outside this set as reserved rather than faulty.
EXPECTED_ENCODER_SLOTS = frozenset(range(17))

# ---------------------------------------------------------------------------
# Hand serial link
# ---------------------------------------------------------------------------

LINK_RESPONSE_QUEUE_MAXSIZE = 4
LINK_DEMUX_READ_TIMEOUT_S = 0.5
"""Serial-port read timeout. Bounds how long the demuxer waits before re-checking
its run flag, so disconnect joins promptly."""

LINK_HANDLER_ERROR_LOG_INTERVAL_S = 1.0
"""Per-second-byte rate limit for handler-exception traceback logging."""

LINK_DEFAULT_BAUDRATE = DEFAULT_ENCODER_BAUDRATE
"""Transport default for ``HandSerialLink`` when no explicit baud is given."""

LINK_DEFAULT_RESPONSE_TIMEOUT_S = 0.5
"""Default wait for a register response before ``send_register_request`` raises."""

TACTILE_REGISTER_ATTEMPTS = 3
"""Times a tactile register read/write is attempted before giving up. A single
round-trip is occasionally lost on a busy link; retrying recovers it. Reads are
pure and the register writes are idempotent, so re-sending is safe."""

LINK_DEMUX_JOIN_TIMEOUT_S = 1.0
"""How long ``disconnect`` waits for the demuxer thread to exit."""

ENCODER_FIRST_FRAME_TIMEOUT_S = 0.1
"""How long ``JointEncoderClient.start_stream`` waits for the first valid
AA A9 frame before raising ``EncodersNotAvailableError``."""

# ---------------------------------------------------------------------------
# Auto-stream timing
# ---------------------------------------------------------------------------

# One auto-stream period @ ~1 kHz. Used after clearing offsets so the reader
# emits at least one fresh frame before averaging.
OFFSET_CLEAR_SETTLE_S = 0.01

OFFSET_CAPTURE_POLL_S = 0.002
"""Poll interval while collecting frames to average into zeroing offsets."""

OFFSET_CAPTURE_FRAME_BUDGET_S = 0.05
"""Slowest per-frame budget assumed when deriving the default offset-capture
deadline; well above any real stream period so healthy captures never trip it."""

TACTILE_FIRST_FRAME_TIMEOUT_S = 2.0
"""Default wait for the first stored tactile frame in ``wait_for_first_frame``."""

TACTILE_STREAM_STALE_REARM_S = 2.0
"""Mid-stream silence (seconds) after which a read triggers a stream re-arm.
"""

TACTILE_STREAM_REARM_MIN_INTERVAL_S = 5.0
"""Minimum spacing between re-arm attempts, so a genuinely dead device can't
cause a register-write retry storm from a fast reader loop."""
