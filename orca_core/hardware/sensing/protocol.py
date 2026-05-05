"""Binary protocol codec for ORCA tactile sensor communication.

Converts between raw bytes (as defined by the sensor hardware protocol)
and Python objects. Pure functions only — no I/O, no state, no threading.

Naming conventions:
    build_*   — assemble an outgoing request frame (bytes)
    parse_*   — validate an incoming frame and extract raw data (bytes)
    decode_*  — interpret raw bytes into domain objects (dicts of forces)
    encode_*  — convert domain values into register bytes
    extract_* — pull a single field from frame metadata
    unpack_*  — split a payload into its component parts
    compute_* — calculate sizes or indices from configuration
"""
from __future__ import annotations

from typing import TypedDict

from orca_core.hardware.sensing.constants import (
    PROTOCOL_HEADER_REQUEST,
    PROTOCOL_HEADER_RESPONSE,
    PROTOCOL_HEADER_AUTO,
    PROTOCOL_RESERVED,
    FUNC_CODE_READ,
    FUNC_CODE_WRITE,
    AUTO_DATA_RESULTANT,
    AUTO_DATA_TAXELS,
    ADDR_NUM_TAXELS_START,
    ADDR_NUM_TAXELS_LENGTH,
    RESULTANT_BLOCK_SIZE,
    RESOLUTION_N_PER_LSB,
    BYTES_PER_RESULTANT,
    BYTES_PER_TAXEL,
    SLOT_CONNECTED_BIT_POSITIONS,
    SLOT_DISTAL_TAXEL_REGISTER_OFFSETS,
    MAX_AUTO_FRAME_EFF_LEN,
    RESPONSE_META_SIZE,
    AUTO_FRAME_META_SIZE,
    MODULES_PER_SLOT,
    DISTAL_MODULE_OFFSET,
    MIN_READ_RESPONSE_SIZE,
    MIN_WRITE_RESPONSE_SIZE,
)

# =========================================================================
# Types
# =========================================================================

ForceVector = list[float]
"""[fx, fy, fz] force components in Newtons. Always exactly 3 elements.
fx/fy are signed (shear); fz is unsigned (normal force, always >= 0).
Mutable list (not tuple) because callers apply zeroing offsets in-place."""

ResultantForces = dict[str, ForceVector]
"""{finger_name: [fx, fy, fz]} resultant forces per sensor."""

TaxelForces = dict[str, list[ForceVector]]
"""{finger_name: [[fx, fy, fz], ...]} per-taxel forces per sensor."""


class AutoDataTypeInfo(TypedDict):
    """Parsed auto-data-type register value."""
    raw: str
    resultant: bool
    taxels: bool


# =========================================================================
# Protocol Constants
# =========================================================================

FORCE_DECIMAL_PLACES = 1
"""Decimal places for rounding decoded force values.

This is a codec-level choice, not a wire-format specification. The sensor
transmits integer LSB counts; this module converts to Newtons and rounds.
"""


# =========================================================================
# Checksum
# =========================================================================

def calculate_checksum(frame: bytes) -> int:
    """LRC checksum: two's complement of the low byte of the sum."""
    return (0x100 - (sum(frame) & 0xFF)) & 0xFF


def validate_auto_frame_lrc(meta: bytes, payload: bytes, lrc: int) -> bool:
    """Check LRC of an auto-stream frame. Returns True if valid.

    Reconstructs the full frame (header + meta + payload) internally so
    the caller doesn't need to know the frame assembly recipe.

    Note: returns bool (not raises) because the auto-stream reader counts
    bad-LRC frames as a recoverable statistic rather than aborting.

    Args:
        meta: 3 bytes after the AA56 header (reserved + eff_len)
        payload: The payload bytes (eff_len bytes)
        lrc: The LRC byte to validate against
    """
    frame_without_lrc = PROTOCOL_HEADER_AUTO + meta + payload
    return calculate_checksum(frame_without_lrc) == lrc


def _validate_frame_lrc(frame: bytes, context: str) -> None:
    """Validate LRC of a request-response frame. Raises on mismatch."""
    if frame[-1] != calculate_checksum(frame[:-1]):
        raise IOError(f"{context} LRC mismatch")


# =========================================================================
# Frame Size Helpers
# =========================================================================

def read_response_body_size(count: int) -> int:
    """Total bytes after the AA55 header in a read response.

    Args:
        count: Number of data bytes requested (same value passed to build_read_request)
    """
    return RESPONSE_META_SIZE + count + 1  # meta + data + LRC


# =========================================================================
# Frame Builders
# =========================================================================

def _validate_u16(value: int, name: str) -> None:
    """Validate that a value fits in a uint16 field."""
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"{name} must be 0x0000-0xFFFF, got {value}")


def build_read_request(address: int, count: int) -> bytes:
    """Build a read-register request frame (55 AA | 00 | 03 | addr | count | LRC)."""
    _validate_u16(address, "address")
    if count <= 0:
        raise ValueError(f"count must be > 0, got {count}")
    _validate_u16(count, "count")
    body = (
        PROTOCOL_HEADER_REQUEST
        + bytes([PROTOCOL_RESERVED, FUNC_CODE_READ])
        + address.to_bytes(2, "little")
        + count.to_bytes(2, "little")
    )
    return body + bytes([calculate_checksum(body)])


def build_write_request(address: int, data: bytes) -> bytes:
    """Build a write-register request frame (55 AA | 00 | 10 | addr | len | data | LRC)."""
    _validate_u16(address, "address")
    if len(data) == 0:
        raise ValueError("data must not be empty")
    _validate_u16(len(data), "data length")
    body = (
        PROTOCOL_HEADER_REQUEST
        + bytes([PROTOCOL_RESERVED, FUNC_CODE_WRITE])
        + address.to_bytes(2, "little")
        + len(data).to_bytes(2, "little")
        + data
    )
    return body + bytes([calculate_checksum(body)])


# =========================================================================
# Frame Parsers — response frames (request-response mode)
# =========================================================================

def parse_read_response(frame: bytes) -> bytes:
    """Validate and extract data from a read response frame.

    Frame layout: header(2) + reserved(1) + func(1) + addr(2) + count(2) + data(count) + LRC(1)

    Returns:
        The data bytes from the response

    Raises:
        IOError: If frame is too short, func code is wrong, or LRC validation fails
    """
    if len(frame) < MIN_READ_RESPONSE_SIZE:
        raise IOError(
            f"Read response frame too short: {len(frame)} bytes "
            f"(minimum {MIN_READ_RESPONSE_SIZE}), data={frame.hex()}"
        )
    if frame[:2] != PROTOCOL_HEADER_RESPONSE:
        raise IOError(
            f"Expected response header {PROTOCOL_HEADER_RESPONSE.hex()}, "
            f"got {frame[:2].hex()}"
        )
    _validate_frame_lrc(frame, "Read response")
    if frame[3] != FUNC_CODE_READ:
        raise IOError(
            f"Expected read response (func=0x{FUNC_CODE_READ:02X}), "
            f"got func=0x{frame[3]:02X}"
        )
    declared_count = int.from_bytes(frame[6:8], "little")
    actual_data = frame[8:-1]
    if len(actual_data) != declared_count:
        raise IOError(
            f"Read response length mismatch: header declares {declared_count} bytes "
            f"but frame contains {len(actual_data)}, data={frame.hex()}"
        )
    return bytes(actual_data)


def extract_write_response_data_length(meta: bytes) -> int:
    """Extract payload length from write response meta bytes.

    Args:
        meta: Exactly 6 bytes (reserved + func + addr + nbytes)

    Returns:
        Number of payload bytes that follow the meta

    Raises:
        ValueError: If meta is not exactly 6 bytes
    """
    if len(meta) != RESPONSE_META_SIZE:
        raise ValueError(f"Write response meta must be {RESPONSE_META_SIZE} bytes, got {len(meta)}")
    return int.from_bytes(meta[4:6], "little")


def parse_write_response(frame: bytes) -> None:
    """Validate a write response frame (LRC and status check).

    Frame layout: header(2) + reserved(1) + func(1) + addr(2) + nbytes(2) + payload(nbytes) + LRC(1)

    Raises:
        IOError: If frame is too short, LRC validation fails, or status byte indicates failure
    """
    if len(frame) < MIN_WRITE_RESPONSE_SIZE:
        raise IOError(
            f"Write response frame too short: {len(frame)} bytes "
            f"(minimum {MIN_WRITE_RESPONSE_SIZE}), data={frame.hex()}"
        )
    if frame[:2] != PROTOCOL_HEADER_RESPONSE:
        raise IOError(
            f"Expected response header {PROTOCOL_HEADER_RESPONSE.hex()}, "
            f"got {frame[:2].hex()}"
        )
    _validate_frame_lrc(frame, "Write response")
    nbytes = int.from_bytes(frame[6:8], "little")
    if nbytes >= 1:
        if len(frame) < 9 + nbytes:
            raise IOError(
                f"Write response frame truncated: claims {nbytes} payload bytes "
                f"but frame is only {len(frame)} bytes, data={frame.hex()}"
            )
        status = frame[8]
        if status != 0:
            raise IOError(f"Write failed, status=0x{status:02X}")


# =========================================================================
# Frame Parsers — auto-stream frames
# =========================================================================

def extract_auto_frame_eff_len(meta: bytes) -> int:
    """Extract effective length from auto-stream frame meta bytes.

    Args:
        meta: 3 bytes after the AA56 header (reserved(1) + eff_len(2))

    Returns:
        Effective payload length (includes error_code byte)

    Raises:
        ValueError: If eff_len exceeds MAX_AUTO_FRAME_EFF_LEN (possible corruption)
    """
    eff_len = int.from_bytes(meta[1:3], "little")
    if eff_len > MAX_AUTO_FRAME_EFF_LEN:
        raise ValueError(f"Invalid eff_len in auto frame: {eff_len} (possible corruption)")
    return eff_len


def unpack_auto_payload(payload: bytes) -> tuple[int, bytes]:
    """Unpack auto-stream payload into error code and force data.

    The protocol packs a 1-byte sensor error code (0 = no error) followed
    by the actual force data into a single payload. This separates them.

    Returns:
        (error_code, force_data) tuple

    Raises:
        ValueError: If payload is empty
    """
    if len(payload) == 0:
        raise ValueError("Auto-stream payload is empty (expected at least error code byte)")
    return payload[0], payload[1:]


# =========================================================================
# Payload Size Computation
# =========================================================================

def compute_resultant_payload_size(num_sensors: int) -> int:
    """Compute payload size for resultant-only auto-stream mode."""
    return num_sensors * BYTES_PER_RESULTANT


def compute_taxel_payload_size(
    active_sensors: list[str], num_taxels: dict[str, int],
) -> int:
    """Compute payload size for taxel-only auto-stream mode."""
    return sum(num_taxels[f] for f in active_sensors) * BYTES_PER_TAXEL


def compute_combined_payload_size(
    active_sensors: list[str], num_taxels: dict[str, int],
) -> int:
    """Compute payload size for combined (resultant + taxels) auto-stream mode."""
    return (
        compute_resultant_payload_size(len(active_sensors))
        + compute_taxel_payload_size(active_sensors, num_taxels)
    )


def compute_expected_payload_size(
    mode_resultant: bool,
    mode_taxels: bool,
    active_sensors: list[str],
    num_taxels: dict[str, int],
) -> int:
    """Compute expected auto-stream payload size for the given streaming mode.

    Args:
        mode_resultant: Whether resultant force data is enabled
        mode_taxels: Whether taxel data is enabled
        active_sensors: List of active finger names
        num_taxels: {finger: taxel_count} for each finger
    """
    if mode_resultant and mode_taxels:
        return compute_combined_payload_size(active_sensors, num_taxels)
    elif mode_resultant:
        return compute_resultant_payload_size(len(active_sensors))
    elif mode_taxels:
        return compute_taxel_payload_size(active_sensors, num_taxels)
    return 0


# =========================================================================
# Payload Decoders — auto-stream formats
# =========================================================================

def _validate_payload_size(data: bytes, expected: int, context: str) -> None:
    """Validate that payload data matches expected size. Raises ValueError on mismatch."""
    if len(data) != expected:
        preview = data[:16].hex() if data else "(empty)"
        raise ValueError(
            f"{context} size mismatch: expected {expected} bytes, "
            f"got {len(data)} bytes (first bytes: {preview})"
        )


def _unpack_taxel(data: bytes, offset: int) -> ForceVector:
    """Unpack one taxel force vector: 3 contiguous bytes (int8 fx, int8 fy, uint8 fz)."""
    fx_byte, fy_byte, fz_byte = data[offset], data[offset + 1], data[offset + 2]
    fx = (fx_byte - 256 if fx_byte > 127 else fx_byte) * RESOLUTION_N_PER_LSB
    fy = (fy_byte - 256 if fy_byte > 127 else fy_byte) * RESOLUTION_N_PER_LSB
    fz = fz_byte * RESOLUTION_N_PER_LSB
    return [round(fx, FORCE_DECIMAL_PLACES), round(fy, FORCE_DECIMAL_PLACES), round(fz, FORCE_DECIMAL_PLACES)]


def _unpack_resultant(data: bytes, offset: int) -> ForceVector:
    """Unpack one resultant force vector from a 6-byte module slot.

    Each axis occupies a 2-byte slot; only the low byte carries data
    (signed int8 for fx/fy, unsigned uint8 for fz). The high byte is
    sign-extension of the low byte cast to int8 and must be discarded.
    """
    fx_lo, fy_lo, fz_lo = data[offset], data[offset + 2], data[offset + 4]
    fx = (fx_lo - 256 if fx_lo > 127 else fx_lo) * RESOLUTION_N_PER_LSB
    fy = (fy_lo - 256 if fy_lo > 127 else fy_lo) * RESOLUTION_N_PER_LSB
    fz = fz_lo * RESOLUTION_N_PER_LSB
    return [round(fx, FORCE_DECIMAL_PLACES), round(fy, FORCE_DECIMAL_PLACES), round(fz, FORCE_DECIMAL_PLACES)]


def decode_resultant_auto(
    data: bytes,
    active_sensors: list[str],
) -> ResultantForces:
    """Decode auto-stream resultant forces (6 bytes/sensor, sequential).

    Args:
        data: Raw byte data from auto-stream
        active_sensors: Finger names sorted by hardware slot ID ascending.
            Auto-stream data arrives in slot order, so this ordering is
            required for correct finger-to-data mapping. Ordering is the
            caller's responsibility — the codec does not validate it because
            it has no access to the slot-ID mapping.

    Returns:
        Resultant forces for each active sensor

    Raises:
        ValueError: If data size doesn't match expected
    """
    expected_size = len(active_sensors) * BYTES_PER_RESULTANT
    _validate_payload_size(data, expected_size, f"Resultant auto ({len(active_sensors)} sensors)")

    result = {}
    for i, finger in enumerate(active_sensors):
        result[finger] = _unpack_resultant(data, i * BYTES_PER_RESULTANT)
    return result


def decode_taxels_auto(
    data: bytes,
    active_sensors: list[str],
    num_taxels: dict[str, int],
) -> TaxelForces:
    """Decode auto-stream taxel data (3 bytes/taxel, sequential by sensor).

    Args:
        data: Raw byte data from auto-stream
        active_sensors: Finger names sorted by hardware slot ID ascending.
            Auto-stream data arrives in slot order, so this ordering is
            required for correct finger-to-data mapping. Ordering is the
            caller's responsibility — the codec does not validate it because
            it has no access to the slot-ID mapping.
        num_taxels: {finger: taxel_count} for each finger

    Returns:
        Per-taxel forces for each active sensor

    Raises:
        ValueError: If data size doesn't match expected
    """
    expected_size = compute_taxel_payload_size(active_sensors, num_taxels)
    _validate_payload_size(data, expected_size, "Taxels auto")

    result = {}
    offset = 0
    for finger in active_sensors:
        finger_taxels = []
        for _ in range(num_taxels[finger]):
            finger_taxels.append(_unpack_taxel(data, offset))
            offset += BYTES_PER_TAXEL
        result[finger] = finger_taxels
    return result


def decode_combined_auto(
    data: bytes,
    active_sensors: list[str],
    num_taxels: dict[str, int],
) -> tuple[ResultantForces, TaxelForces]:
    """Decode auto-stream combined format (resultant + taxels interleaved per sensor).

    For each sensor in slot order: resultant(6 bytes) then taxels(3 bytes each),
    followed by the next sensor's resultant + taxels, and so on.

    Args:
        data: Raw byte data from auto-stream
        active_sensors: Finger names sorted by hardware slot ID ascending.
            Auto-stream data arrives in slot order, so this ordering is
            required for correct finger-to-data mapping. Ordering is the
            caller's responsibility — the codec does not validate it because
            it has no access to the slot-ID mapping.
        num_taxels: {finger: taxel_count} for each finger

    Returns:
        Tuple of (resultant forces, per-taxel forces)

    Raises:
        ValueError: If data size doesn't match expected
    """
    expected_size = compute_combined_payload_size(
        active_sensors, num_taxels,
    )
    _validate_payload_size(data, expected_size, "Combined auto")

    offset = 0
    resultant_forces = {}
    taxels = {}

    for finger in active_sensors:
        resultant_forces[finger] = _unpack_resultant(data, offset)
        offset += BYTES_PER_RESULTANT

        finger_taxels = []
        for _ in range(num_taxels[finger]):
            finger_taxels.append(_unpack_taxel(data, offset))
            offset += BYTES_PER_TAXEL
        taxels[finger] = finger_taxels

    return resultant_forces, taxels


# =========================================================================
# Payload Decoders — register block format (request-response mode)
# =========================================================================

def compute_distal_module_index(sensor_id: int) -> int:
    """Compute the register-block module index for a fingertip (distal phalanx) sensor.

    The resultant force register block contains MODULES_PER_SLOT modules per
    sensor slot (proximal, middle, distal, nail). The distal phalanx is at
    offset DISTAL_MODULE_OFFSET within each slot's group.

    Args:
        sensor_id: Hardware slot ID (0-4)

    Returns:
        Module index into the 28-module resultant force register block
    """
    return sensor_id * MODULES_PER_SLOT + DISTAL_MODULE_OFFSET


def decode_resultant_register(
    data: bytes,
    active_sensors: list[str],
    module_indices: dict[str, int],
) -> ResultantForces:
    """Decode the full 168-byte resultant force register block (0x0500-0x05A7).

    Used in request-response mode where the full register block is returned.
    The block contains 28 modules (MODULES_PER_SLOT per sensor slot: proximal,
    middle, distal, nail; plus 8 palm modules), each with 6 bytes (fx, fy, fz).
    Use compute_distal_module_index() to get the correct module index for
    fingertip sensors.

    Args:
        data: Raw 168-byte register block
        active_sensors: Finger names to extract
        module_indices: {finger: module_idx} — zero-based index into the
            28-module block. Byte offset = module_idx * BYTES_PER_RESULTANT.
            For fingertip sensors, use compute_distal_module_index(slot_id)
            to get the correct index.

    Returns:
        Resultant forces for each active sensor

    Raises:
        ValueError: If data is too short
    """
    if len(data) < RESULTANT_BLOCK_SIZE:
        raise ValueError(f"Resultant force block too short: {len(data)} bytes")

    result = {}
    for finger in active_sensors:
        result[finger] = _unpack_resultant(data, module_indices[finger] * BYTES_PER_RESULTANT)
    return result


# =========================================================================
# Register Codecs
# =========================================================================

def decode_connected_sensors(
    data: bytes,
    sensor_id_to_finger: dict[int, str],
) -> dict[str, bool]:
    """Decode connected-sensors register (4 bytes) into {finger: bool}.

    Args:
        data: 4-byte register block
        sensor_id_to_finger: {slot_id: finger_name} mapping for all slots

    Returns:
        {finger: is_connected} for each slot

    Raises:
        ValueError: If data is too short or sensor_id_to_finger doesn't cover all slots
    """
    num_slots = len(SLOT_CONNECTED_BIT_POSITIONS)
    if len(data) < 4:
        raise ValueError(f"Expected 4 bytes, got {len(data)}")
    if len(sensor_id_to_finger) != num_slots:
        raise ValueError(
            f"sensor_id_to_finger must have {num_slots} entries, "
            f"got {len(sensor_id_to_finger)}"
        )
    status = [bool(data[byte_idx] & (1 << bit_pos)) for byte_idx, bit_pos in SLOT_CONNECTED_BIT_POSITIONS]
    return {sensor_id_to_finger[i]: status[i] for i in range(num_slots)}


def decode_num_taxels(
    data: bytes,
    sensor_id_to_finger: dict[int, str],
) -> dict[str, int]:
    """Decode taxel-count register block into {finger: count}.

    Args:
        data: 56-byte register block (28 x uint16 little-endian)
        sensor_id_to_finger: {slot_id: finger_name} mapping for all slots

    Returns:
        {finger: taxel_count} for each slot

    Raises:
        ValueError: If data is too short or sensor_id_to_finger doesn't cover all slots
    """
    num_slots = len(SLOT_DISTAL_TAXEL_REGISTER_OFFSETS)
    if len(data) != ADDR_NUM_TAXELS_LENGTH:
        raise ValueError(
            f"Taxel count register block size mismatch: "
            f"expected {ADDR_NUM_TAXELS_LENGTH} bytes, got {len(data)}"
        )
    if len(sensor_id_to_finger) != num_slots:
        raise ValueError(
            f"sensor_id_to_finger must have {num_slots} entries, "
            f"got {len(sensor_id_to_finger)}"
        )
    # Length validated above as ADDR_NUM_TAXELS_LENGTH (56), guaranteeing even byte count
    taxel_counts = [
        int.from_bytes(data[i:i+2], byteorder="little")
        for i in range(0, len(data), 2)
    ]
    num_entries = len(taxel_counts)
    distal_indices = {}
    for slot, addr in enumerate(SLOT_DISTAL_TAXEL_REGISTER_OFFSETS):
        idx = (addr - ADDR_NUM_TAXELS_START) // 2
        if idx < 0 or idx >= num_entries:
            raise ValueError(
                f"Slot {slot} register offset {addr:#x} maps to index {idx}, "
                f"but register block only has {num_entries} entries"
            )
        distal_indices[sensor_id_to_finger[slot]] = idx
    return {finger: taxel_counts[idx] for finger, idx in distal_indices.items()}


def decode_auto_data_type(data: bytes) -> AutoDataTypeInfo:
    """Decode auto-data-type register (1 byte) into parsed flags."""
    if len(data) != 1:
        raise ValueError(f"Auto-data-type register must be exactly 1 byte, got {len(data)}")
    val = data[0]
    return {
        "raw": f"{val:08b}",
        "resultant": bool(val & AUTO_DATA_RESULTANT),
        "taxels": bool(val & AUTO_DATA_TAXELS),
    }


def encode_auto_data_type(resultant: bool, taxels: bool) -> bytes:
    """Encode auto-data-type register value."""
    val = (AUTO_DATA_RESULTANT if resultant else 0) | (AUTO_DATA_TAXELS if taxels else 0)
    return bytes([val])


# =========================================================================
# Wire-format encoders — fake-hardware fixtures only
#
# Production code never encodes resultant or taxel wire bytes (the sensor
# board is the only encoder on the real bus). These exist so the mock
# client and decoder unit tests can round-trip through the real decoders,
# exercising the wire-format logic instead of bypassing it. Do NOT import
# from runtime code paths.
# =========================================================================

def _pack_resultant_for_mock(force: ForceVector) -> bytes:
    """Encode one [fx, fy, fz] vector into a 6-byte module slot.

    Each axis is packed as low_byte + sign-extended high byte (0xFF when
    low byte > 127, else 0x00) for fx, fy, AND fz — matching the firmware.
    """
    def _pack_axis(value_n: float) -> bytes:
        lo = round(value_n / RESOLUTION_N_PER_LSB) & 0xFF
        hi = 0xFF if lo > 127 else 0x00
        return bytes([lo, hi])

    return _pack_axis(force[0]) + _pack_axis(force[1]) + _pack_axis(force[2])


def _pack_taxel_for_mock(force: ForceVector) -> bytes:
    """Encode one [fx, fy, fz] taxel vector into 3 contiguous bytes."""
    return bytes([
        round(force[0] / RESOLUTION_N_PER_LSB) & 0xFF,
        round(force[1] / RESOLUTION_N_PER_LSB) & 0xFF,
        round(force[2] / RESOLUTION_N_PER_LSB) & 0xFF,
    ])


def encode_resultant_auto_for_mock(
    forces: ResultantForces,
    active_sensors: list[str],
) -> bytes:
    """Encode resultant-only auto-stream payload for mock use."""
    return b"".join(_pack_resultant_for_mock(forces[f]) for f in active_sensors)


def encode_taxels_auto_for_mock(
    taxels: TaxelForces,
    active_sensors: list[str],
) -> bytes:
    """Encode taxel-only auto-stream payload for mock use."""
    return b"".join(_pack_taxel_for_mock(t) for f in active_sensors for t in taxels[f])


def encode_combined_auto_for_mock(
    forces: ResultantForces,
    taxels: TaxelForces,
    active_sensors: list[str],
) -> bytes:
    """Encode interleaved (resultant + taxels) auto-stream payload for mock use."""
    out = bytearray()
    for f in active_sensors:
        out += _pack_resultant_for_mock(forces[f])
        for t in taxels[f]:
            out += _pack_taxel_for_mock(t)
    return bytes(out)
