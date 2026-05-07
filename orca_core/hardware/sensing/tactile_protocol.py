"""Binary protocol codec for ORCA tactile sensor communication.

Converts between raw bytes (as defined by the sensor hardware protocol)
and Python objects. Pure functions only — no I/O, no state, no threading.
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
# Checksum
# =========================================================================

def calculate_checksum(frame: bytes) -> int:
    """LRC checksum: two's complement of the low byte of the sum."""
    return (0x100 - (sum(frame) & 0xFF)) & 0xFF


def validate_auto_frame_lrc(meta: bytes, payload: bytes, lrc: int) -> bool:
    """Return ``True`` if ``lrc`` matches the checksum over ``header + meta + payload``.

    Returns bool rather than raising because the auto-stream reader treats a
    bad LRC as a recoverable counter, not an abort condition.
    """
    frame_without_lrc = PROTOCOL_HEADER_AUTO + meta + payload
    return calculate_checksum(frame_without_lrc) == lrc


def _validate_frame_lrc(frame: bytes, context: str) -> None:
    if frame[-1] != calculate_checksum(frame[:-1]):
        raise IOError(f"{context} LRC mismatch")


# =========================================================================
# Frame Size Helpers
# =========================================================================

def read_response_body_size(count: int) -> int:
    """Bytes after the AA55 header in a read response with ``count`` data bytes."""
    return RESPONSE_META_SIZE + count + 1  # meta + data + LRC


# =========================================================================
# Frame Builders
# =========================================================================

def _validate_u16(value: int, name: str) -> None:
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
    """Validate a read-response frame and return its data bytes.

    Frame layout: header(2) + reserved(1) + func(1) + addr(2) + count(2) + data(count) + LRC(1).
    Raises ``IOError`` on size, func-code, or LRC mismatch.
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
    """Read the payload length from the 6-byte write-response meta block."""
    if len(meta) != RESPONSE_META_SIZE:
        raise ValueError(f"Write response meta must be {RESPONSE_META_SIZE} bytes, got {len(meta)}")
    return int.from_bytes(meta[4:6], "little")


def parse_write_response(frame: bytes) -> None:
    """Validate a write-response frame's LRC and status byte.

    Frame layout: header(2) + reserved(1) + func(1) + addr(2) + nbytes(2) + payload(nbytes) + LRC(1).
    Raises ``IOError`` on size, LRC, or non-zero status.
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
    """Read eff_len (includes error_code byte) from the 3-byte auto-frame meta.

    Raises ``ValueError`` if it exceeds ``MAX_AUTO_FRAME_EFF_LEN``, which would
    almost always indicate stream corruption rather than a real giant payload.
    """
    eff_len = int.from_bytes(meta[1:3], "little")
    if eff_len > MAX_AUTO_FRAME_EFF_LEN:
        raise ValueError(f"Invalid eff_len in auto frame: {eff_len} (possible corruption)")
    return eff_len


def unpack_auto_payload(payload: bytes) -> tuple[int, bytes]:
    """Split an auto-stream payload into (error_code, force_data).

    The protocol prefixes force data with a single error-code byte
    (0 = no error). Empty payloads raise ``ValueError``.
    """
    if len(payload) == 0:
        raise ValueError("Auto-stream payload is empty (expected at least error code byte)")
    return payload[0], payload[1:]


# =========================================================================
# Payload Size Computation
# =========================================================================

def compute_resultant_payload_size(num_sensors: int) -> int:
    return num_sensors * BYTES_PER_RESULTANT


def compute_taxel_payload_size(
    active_sensors: list[str], num_taxels: dict[str, int],
) -> int:
    return sum(num_taxels[f] for f in active_sensors) * BYTES_PER_TAXEL


def compute_combined_payload_size(
    active_sensors: list[str], num_taxels: dict[str, int],
) -> int:
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
    """Dispatch to the right ``compute_*_payload_size`` for the active mode."""
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
    return [round(fx, 1), round(fy, 1), round(fz, 1)]


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
    return [round(fx, 1), round(fy, 1), round(fz, 1)]


def decode_resultant_auto(
    data: bytes,
    active_sensors: list[str],
) -> ResultantForces:
    """Decode auto-stream resultant forces (6 bytes/sensor, in slot order).

    ``active_sensors`` must already be sorted by hardware slot ID ascending —
    the codec cannot validate this and assumes the caller has done so.
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

    ``active_sensors`` must already be in slot order (see ``decode_resultant_auto``).
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
    """Decode interleaved auto-stream: per sensor, resultant(6) + taxels(3 each).

    ``active_sensors`` must already be in slot order (see ``decode_resultant_auto``).
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
    """Module index of the distal phalanx for ``sensor_id`` (slot 0-4).

    The register block packs ``MODULES_PER_SLOT`` modules per slot
    (proximal, middle, distal, nail); the distal one sits at
    ``DISTAL_MODULE_OFFSET`` inside each group.
    """
    return sensor_id * MODULES_PER_SLOT + DISTAL_MODULE_OFFSET


def decode_resultant_register(
    data: bytes,
    active_sensors: list[str],
    module_indices: dict[str, int],
) -> ResultantForces:
    """Decode the 168-byte resultant register block (0x0500-0x05A7) at given indices.

    The block packs 28 modules of 6 bytes each (4 per slot + 8 palm).
    ``module_indices[finger]`` is the zero-based module index; for fingertip
    sensors use ``compute_distal_module_index(slot_id)`` to derive it.
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
    """Decode the 4-byte connected-sensors register into ``{finger: is_connected}``."""
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
    """Decode the 56-byte taxel-count register block (28 × uint16 LE) into ``{finger: count}``."""
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
    return b"".join(_pack_resultant_for_mock(forces[f]) for f in active_sensors)


def encode_taxels_auto_for_mock(
    taxels: TaxelForces,
    active_sensors: list[str],
) -> bytes:
    return b"".join(_pack_taxel_for_mock(t) for f in active_sensors for t in taxels[f])


def encode_combined_auto_for_mock(
    forces: ResultantForces,
    taxels: TaxelForces,
    active_sensors: list[str],
) -> bytes:
    """Build interleaved [resultant_i, taxels_i, resultant_{i+1}, ...] payload."""
    out = bytearray()
    for f in active_sensors:
        out += _pack_resultant_for_mock(forces[f])
        for t in taxels[f]:
            out += _pack_taxel_for_mock(t)
    return bytes(out)
