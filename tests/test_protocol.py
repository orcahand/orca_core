"""Tests for the protocol codec layer (pure functions, no I/O)."""

import struct

import pytest

from orca_core.hardware.sensing.protocol import (
    calculate_checksum,
    validate_auto_frame_lrc,
    read_response_body_size,
    build_read_request,
    build_write_request,
    parse_read_response,
    parse_write_response,
    extract_write_response_data_length,
    extract_auto_frame_eff_len,
    unpack_auto_payload,
    compute_resultant_payload_size,
    compute_taxel_payload_size,
    compute_combined_payload_size,
    decode_resultant_auto,
    decode_taxels_auto,
    decode_combined_auto,
    decode_resultant_register,
    decode_connected_sensors,
    decode_num_taxels,
    decode_auto_data_type,
    encode_auto_data_type,
    encode_combined_auto_for_mock,
    encode_resultant_auto_for_mock,
)
from orca_core.hardware.sensing.constants import (
    ADDR_NUM_TAXELS_LENGTH,
    ADDR_NUM_TAXELS_START,
    DEFAULT_TAXEL_COUNTS,
    PROTOCOL_HEADER_REQUEST,
    PROTOCOL_HEADER_RESPONSE,
    PROTOCOL_HEADER_AUTO,
    FUNC_CODE_READ,
    FUNC_CODE_WRITE,
    AUTO_DATA_RESULTANT,
    AUTO_DATA_TAXELS,
    BYTES_PER_RESULTANT,
    BYTES_PER_TAXEL,
    MAX_AUTO_FRAME_EFF_LEN,
    SLOT_DISTAL_TAXEL_REGISTER_OFFSETS,
)


ID_TO_FINGER = {0: "thumb", 1: "index", 2: "middle", 3: "ring", 4: "pinky"}


def _build_read_response(data: bytes) -> bytes:
    meta = bytes([0x00, FUNC_CODE_READ]) + (0x0010).to_bytes(2, "little") + len(data).to_bytes(2, "little")
    frame_wo_lrc = PROTOCOL_HEADER_RESPONSE + meta + data
    return frame_wo_lrc + bytes([calculate_checksum(frame_wo_lrc)])


def _build_write_response(status: int) -> bytes:
    meta = bytes([0x00, FUNC_CODE_WRITE]) + (0x0017).to_bytes(2, "little") + (1).to_bytes(2, "little")
    frame_wo_lrc = PROTOCOL_HEADER_RESPONSE + meta + bytes([status])
    return frame_wo_lrc + bytes([calculate_checksum(frame_wo_lrc)])


# ---------------------------------------------------------------------------
# Checksum
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("frame,expected", [
    (b"\x01\x02\x03", 0xFA),
    (b"", 0),
    (b"\x01", 0xFF),
    (b"\xAA\x55\x00\x03\x10\x00\x04\x00", 0xEA),
])
def test_calculate_checksum(frame, expected):
    """Each case checks the concrete LRC value AND the LRC invariant
    (frame + checksum sums to 0 mod 256)."""
    checksum = calculate_checksum(frame)
    assert checksum == expected
    assert (sum(frame) + checksum) & 0xFF == 0


@pytest.mark.parametrize("lrc,expected", [
    pytest.param(None, True, id="matching"),   # None = compute the correct LRC
    pytest.param(0xFF, False, id="bad"),
])
def test_validate_auto_frame_lrc(lrc, expected):
    meta = b"\x00" + (3).to_bytes(2, "little")
    payload = b"\x00\x01\x02"
    if lrc is None:
        lrc = calculate_checksum(PROTOCOL_HEADER_AUTO + meta + payload)
    assert validate_auto_frame_lrc(meta, payload, lrc) is expected


# ---------------------------------------------------------------------------
# Frame size helpers
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("count,expected", [
    (4, 11),  # meta(6) + data(4) + LRC(1)
    (1, 8),
])
def test_read_response_body_size(count, expected):
    assert read_response_body_size(count) == expected


# ---------------------------------------------------------------------------
# Frame builders
# ---------------------------------------------------------------------------

def test_build_read_request():
    frame = build_read_request(address=0x0010, count=4)
    assert frame[:2] == PROTOCOL_HEADER_REQUEST
    assert frame[2] == 0x00
    assert frame[3] == FUNC_CODE_READ
    assert int.from_bytes(frame[4:6], "little") == 0x0010
    assert int.from_bytes(frame[6:8], "little") == 4
    assert calculate_checksum(frame[:-1]) == frame[-1]


def test_build_write_request():
    frame = build_write_request(address=0x0017, data=b"\x01")
    assert frame[:2] == PROTOCOL_HEADER_REQUEST
    assert frame[2] == 0x00
    assert frame[3] == FUNC_CODE_WRITE
    assert int.from_bytes(frame[4:6], "little") == 0x0017
    assert int.from_bytes(frame[6:8], "little") == 1
    assert frame[8] == 0x01
    assert calculate_checksum(frame[:-1]) == frame[-1]


@pytest.mark.parametrize("address,count,error_match", [
    (0x0010, 0, "count must be > 0"),
    (0x0010, -1, "count must be > 0"),
    (0x10000, 1, "address"),
    (-1, 1, "address"),
])
def test_build_read_request_invalid_inputs(address, count, error_match):
    with pytest.raises(ValueError, match=error_match):
        build_read_request(address=address, count=count)


@pytest.mark.parametrize("address,data,error_match", [
    (0x0017, b"", "data must not be empty"),
    (0x10000, b"\x01", "address"),
])
def test_build_write_request_invalid_inputs(address, data, error_match):
    with pytest.raises(ValueError, match=error_match):
        build_write_request(address=address, data=data)


# ---------------------------------------------------------------------------
# Response frame parsers
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("data", [
    b"\xAB\xCD\xEF\x01",
    b"\x42",
])
def test_parse_read_response_extracts_data(data):
    frame = _build_read_response(data)
    assert parse_read_response(frame) == data


def test_parse_read_response_bad_lrc_raises():
    frame = bytearray(_build_read_response(b"\x01\x02"))
    frame[-1] ^= 0xFF
    with pytest.raises(IOError, match="LRC mismatch"):
        parse_read_response(bytes(frame))


def test_parse_read_response_too_short_raises():
    with pytest.raises(IOError, match="too short"):
        parse_read_response(b"\xAA\x55\x00")


def test_parse_read_response_wrong_func_code_raises():
    meta = bytes([0x00, FUNC_CODE_WRITE]) + (0x0010).to_bytes(2, "little") + (1).to_bytes(2, "little")
    frame_wo_lrc = PROTOCOL_HEADER_RESPONSE + meta + b"\x00"
    frame = frame_wo_lrc + bytes([calculate_checksum(frame_wo_lrc)])
    with pytest.raises(IOError, match="Expected read response"):
        parse_read_response(frame)


def test_parse_read_response_wrong_header_raises():
    frame = bytearray(_build_read_response(b"\x01\x02"))
    frame[0:2] = PROTOCOL_HEADER_AUTO
    frame[-1] = calculate_checksum(bytes(frame[:-1]))
    with pytest.raises(IOError, match="Expected response header"):
        parse_read_response(bytes(frame))


def test_parse_write_response_success():
    parse_write_response(_build_write_response(status=0x00))


def test_parse_write_response_failure_status_raises():
    with pytest.raises(IOError, match="Write failed"):
        parse_write_response(_build_write_response(status=0x01))


def test_parse_write_response_bad_lrc_raises():
    frame = bytearray(_build_write_response(status=0x00))
    frame[-1] ^= 0xFF
    with pytest.raises(IOError, match="LRC mismatch"):
        parse_write_response(bytes(frame))


def test_parse_write_response_too_short_raises():
    with pytest.raises(IOError, match="too short"):
        parse_write_response(b"\xAA\x55\x00")


def test_parse_write_response_truncated_payload_raises():
    # Frame claims 10 payload bytes but only has 1
    meta = bytes([0x00, FUNC_CODE_WRITE]) + (0x0017).to_bytes(2, "little") + (10).to_bytes(2, "little")
    frame_wo_lrc = PROTOCOL_HEADER_RESPONSE + meta + b"\x00"
    frame = frame_wo_lrc + bytes([calculate_checksum(frame_wo_lrc)])
    with pytest.raises(IOError, match="truncated"):
        parse_write_response(frame)


def test_parse_write_response_wrong_header_raises():
    frame = bytearray(_build_write_response(status=0x00))
    frame[0:2] = PROTOCOL_HEADER_AUTO
    frame[-1] = calculate_checksum(bytes(frame[:-1]))
    with pytest.raises(IOError, match="Expected response header"):
        parse_write_response(bytes(frame))


def test_extract_write_response_data_length_known_value():
    # meta: reserved(1) + func(1) + addr(2) + nbytes(2)
    meta = bytes([0x00, FUNC_CODE_WRITE, 0x17, 0x00, 0x03, 0x00])
    assert extract_write_response_data_length(meta) == 3


def test_extract_write_response_data_length_wrong_size_raises():
    with pytest.raises(ValueError, match="must be 6 bytes"):
        extract_write_response_data_length(b"\x00\x00\x00\x00")


# ---------------------------------------------------------------------------
# Auto-stream frame parsers
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("value", [42, MAX_AUTO_FRAME_EFF_LEN])
def test_extract_auto_frame_eff_len_valid(value):
    # meta layout: reserved(1) + eff_len(2 LE) = 3 bytes
    meta = b"\x00" + value.to_bytes(2, "little")
    assert extract_auto_frame_eff_len(meta) == value


def test_extract_auto_frame_eff_len_exceeds_max_raises():
    meta = b"\x00" + (MAX_AUTO_FRAME_EFF_LEN + 1).to_bytes(2, "little")
    with pytest.raises(ValueError, match="Invalid eff_len"):
        extract_auto_frame_eff_len(meta)


@pytest.mark.parametrize("payload,expected_err,expected_data", [
    (b"\x00\x01\x02\x03", 0, b"\x01\x02\x03"),
    (b"\x05\xAB", 5, b"\xAB"),
    (b"\x01", 1, b""),
])
def test_unpack_auto_payload(payload, expected_err, expected_data):
    err, data = unpack_auto_payload(payload)
    assert err == expected_err
    assert data == expected_data


# ---------------------------------------------------------------------------
# Payload size computation
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("n", [0, 1, 3, 5])
def test_compute_resultant_payload_size(n):
    assert compute_resultant_payload_size(n) == n * BYTES_PER_RESULTANT


def test_compute_taxel_payload_size():
    active = ["thumb", "index"]
    num_taxels = {"thumb": 51, "index": 87}
    assert compute_taxel_payload_size(active, num_taxels) == (51 + 87) * BYTES_PER_TAXEL


def test_compute_taxel_payload_size_missing_finger_raises():
    with pytest.raises(KeyError):
        compute_taxel_payload_size(["thumb"], {"index": 87})


def test_compute_combined_payload_size():
    active = ["thumb", "index"]
    num_taxels = {"thumb": 51, "index": 87}
    expected = 2 * BYTES_PER_RESULTANT + (51 + 87) * BYTES_PER_TAXEL
    assert compute_combined_payload_size(active, num_taxels) == expected


# ---------------------------------------------------------------------------
# Payload decoders — auto-stream
#
# Wire format:
#   Resultant per sensor: 6 bytes — 3 axes (fx, fy, fz) × 2-byte slot.
#       Only the low byte carries data (signed int8 for fx/fy, unsigned uint8
#       for fz). High byte is padding the firmware fills with sign-extension
#       of the low byte cast to int8 — must be ignored regardless of value.
#   Taxel per element:    3 bytes — int8 fx, int8 fy, uint8 fz
#   Combined frames interleave: [resultant_sensor_i, taxels_sensor_i, ...]
#   Newtons = LSB count × 0.1
# ---------------------------------------------------------------------------

def _resultant_slot(fx: int, fy: int, fz: int, hi: int = 0x00) -> bytes:
    """Build a 6-byte resultant slot from raw axis bytes plus a high-byte filler.

    Uses two's-complement low-byte encoding for fx/fy. The same `hi` byte goes
    into all three high-byte positions, which lets parametrize cases prove the
    decoder ignores high-byte content.
    """
    return bytes([fx & 0xFF, hi, fy & 0xFF, hi, fz & 0xFF, hi])


@pytest.mark.parametrize("hi_byte", [0x00, 0xFF, 0xA5])
@pytest.mark.parametrize(
    "raw,expected",
    [
        ([(100, -50, 200)], {"thumb": [10.0, -5.0, 20.0]}),
        ([(0, 0, 110)], {"thumb": [0.0, 0.0, 11.0]}),       # fz < 128 (unambiguous)
        ([(0, 0, 200)], {"thumb": [0.0, 0.0, 20.0]}),       # fz > 127 (firmware sign-extends)
        ([(0, 0, 250)], {"thumb": [0.0, 0.0, 25.0]}),       # fz near uint8 saturation
        ([(0, 0, 255)], {"thumb": [0.0, 0.0, 25.5]}),       # fz at uint8 saturation
        (
            [(10, 20, 30), (-10, -20, 40)],
            {"thumb": [1.0, 2.0, 3.0], "index": [-1.0, -2.0, 4.0]},
        ),
    ],
)
def test_decode_resultant_auto(raw, expected, hi_byte):
    data = b"".join(_resultant_slot(fx, fy, fz, hi=hi_byte) for (fx, fy, fz) in raw)
    fingers = list(expected.keys())
    assert decode_resultant_auto(data, fingers) == expected


def test_decode_resultant_auto_wrong_size_raises():
    with pytest.raises(ValueError, match="size mismatch"):
        decode_resultant_auto(b"\x00" * 5, ["thumb"])


def test_decode_resultant_auto_error_includes_hex():
    with pytest.raises(ValueError, match="first bytes"):
        decode_resultant_auto(b"\xDE\xAD", ["thumb"])


def test_decode_taxels_auto_known_values():
    data = struct.pack("bbB", 10, -5, 20) + struct.pack("bbB", 0, 0, 50)
    result = decode_taxels_auto(data, ["thumb"], {"thumb": 2})
    assert len(result["thumb"]) == 2
    assert result["thumb"][0] == [1.0, -0.5, 2.0]
    assert result["thumb"][1] == [0.0, 0.0, 5.0]


def test_decode_taxels_auto_wrong_size_raises():
    with pytest.raises(ValueError, match="size mismatch"):
        decode_taxels_auto(b"\x00" * 5, ["thumb"], {"thumb": 2})


def test_decode_combined_auto_interleaved():
    data = (
        _resultant_slot(100, 0, 50)
        + struct.pack("bbB", 10, 0, 5)
        + _resultant_slot(0, -100, 200, hi=0xFF)  # index fz > 127, firmware sign-extends
        + struct.pack("bbB", 0, -10, 20)
    )
    forces, taxels = decode_combined_auto(
        data, ["thumb", "index"], {"thumb": 1, "index": 1},
    )
    assert forces["thumb"] == [10.0, 0.0, 5.0]
    assert forces["index"] == [0.0, -10.0, 20.0]
    assert taxels["thumb"][0] == [1.0, 0.0, 0.5]
    assert taxels["index"][0] == [0.0, -1.0, 2.0]


@pytest.mark.parametrize("forces", [
    {"thumb": [0.0, 0.0, 0.0]},
    {"thumb": [10.0, -5.0, 20.0]},
    {"thumb": [-12.8, 12.7, 25.5]},                          # all axes at signed-byte boundary
    {"thumb": [0.0, 0.0, 12.8]},                             # fz where firmware high byte flips
    {"thumb": [0.0, 0.0, 25.5]},                             # fz at uint8 saturation
    {"thumb": [4.2, -3.0, 20.0], "index": [-1.0, 0.5, 0.1]},
])
def test_resultant_encode_decode_roundtrip(forces):
    fingers = list(forces.keys())
    wire = encode_resultant_auto_for_mock(forces, fingers)
    assert decode_resultant_auto(wire, fingers) == forces


def test_combined_encode_decode_roundtrip():
    forces = {"thumb": [10.0, -5.0, 20.0], "index": [-1.0, 0.5, 25.5]}
    taxels = {"thumb": [[1.0, -0.5, 2.0]], "index": [[0.0, 0.0, 0.5]]}
    fingers = ["thumb", "index"]
    wire = encode_combined_auto_for_mock(forces, taxels, fingers)
    decoded_forces, decoded_taxels = decode_combined_auto(
        wire, fingers, {"thumb": 1, "index": 1},
    )
    assert decoded_forces == forces
    assert decoded_taxels == taxels


# ---------------------------------------------------------------------------
# Payload decoders — register block
# ---------------------------------------------------------------------------

def test_decode_resultant_register_known_values():
    # Module index for thumb (slot 0): 0*4+2 = 2, byte offset = 12
    data = bytearray(168)
    data[12:18] = _resultant_slot(100, -50, 200, hi=0xFF)  # fz > 127
    result = decode_resultant_register(bytes(data), ["thumb"], {"thumb": 2})
    assert result["thumb"] == [10.0, -5.0, 20.0]


def test_decode_resultant_register_too_short_raises():
    with pytest.raises(ValueError, match="too short"):
        decode_resultant_register(b"\x00" * 10, ["thumb"], {"thumb": 2})


# ---------------------------------------------------------------------------
# Register decoders — connected sensors
# ---------------------------------------------------------------------------

# Slot i bit mask: slot 0 → byte 0 bit 2, slot 1 → byte 0 bit 6,
# slot 2 → byte 1 bit 2, slot 3 → byte 1 bit 6, slot 4 → byte 2 bit 2.
@pytest.mark.parametrize("data,expected", [
    pytest.param(
        bytes([0x44, 0x44, 0x04, 0x00]),
        {"thumb": True, "index": True, "middle": True, "ring": True, "pinky": True},
        id="all_connected",
    ),
    pytest.param(
        bytes([0x00, 0x00, 0x00, 0x00]),
        {"thumb": False, "index": False, "middle": False, "ring": False, "pinky": False},
        id="none_connected",
    ),
    pytest.param(
        bytes([0x04, 0x00, 0x04, 0x00]),
        {"thumb": True, "index": False, "middle": False, "ring": False, "pinky": True},
        id="partial",
    ),
])
def test_decode_connected_sensors(data, expected):
    assert decode_connected_sensors(data, ID_TO_FINGER) == expected


def test_decode_connected_sensors_too_short_raises():
    with pytest.raises(ValueError, match="Expected 4 bytes"):
        decode_connected_sensors(b"\x00\x00", ID_TO_FINGER)


def test_decode_connected_sensors_wrong_mapping_size_raises():
    with pytest.raises(ValueError, match="must have 5 entries"):
        decode_connected_sensors(bytes([0x00, 0x00, 0x00, 0x00]), {0: "thumb", 1: "index"})


# ---------------------------------------------------------------------------
# Register decoders — num taxels
# ---------------------------------------------------------------------------

def test_decode_num_taxels_known_values():
    """Pack DEFAULT_TAXEL_COUNTS at the configured slot register offsets, decode,
    assert round-trip. Both the byte offsets and the values are derived from
    constants the decoder reads, so the test follows the source of truth."""
    data = bytearray(ADDR_NUM_TAXELS_LENGTH)
    for slot_id, addr in enumerate(SLOT_DISTAL_TAXEL_REGISTER_OFFSETS):
        finger = ID_TO_FINGER[slot_id]
        byte_offset = addr - ADDR_NUM_TAXELS_START
        struct.pack_into("<H", data, byte_offset, DEFAULT_TAXEL_COUNTS[finger])
    result = decode_num_taxels(bytes(data), ID_TO_FINGER)
    assert result == {f: DEFAULT_TAXEL_COUNTS[f] for f in ID_TO_FINGER.values()}


def test_decode_num_taxels_wrong_data_length_raises():
    with pytest.raises(ValueError, match="size mismatch"):
        decode_num_taxels(b"\x00" * 10, ID_TO_FINGER)


def test_decode_num_taxels_wrong_mapping_size_raises():
    with pytest.raises(ValueError, match="must have 5 entries"):
        decode_num_taxels(bytes(56), {0: "thumb"})


# ---------------------------------------------------------------------------
# Auto data-type codec
# ---------------------------------------------------------------------------

AUTO_DATA_TYPE_CASES = [
    (False, False, bytes([0x00])),
    (True, False, bytes([AUTO_DATA_RESULTANT])),
    (False, True, bytes([AUTO_DATA_TAXELS])),
    (True, True, bytes([AUTO_DATA_RESULTANT | AUTO_DATA_TAXELS])),
]


@pytest.mark.parametrize("resultant,taxels,encoded", AUTO_DATA_TYPE_CASES)
def test_encode_auto_data_type(resultant, taxels, encoded):
    assert encode_auto_data_type(resultant=resultant, taxels=taxels) == encoded


@pytest.mark.parametrize("resultant,taxels,encoded", AUTO_DATA_TYPE_CASES)
def test_decode_auto_data_type(resultant, taxels, encoded):
    result = decode_auto_data_type(encoded)
    assert result["resultant"] is resultant
    assert result["taxels"] is taxels


@pytest.mark.parametrize("resultant,taxels,encoded", AUTO_DATA_TYPE_CASES)
def test_auto_data_type_round_trip(resultant, taxels, encoded):
    decoded = decode_auto_data_type(encode_auto_data_type(resultant=resultant, taxels=taxels))
    assert decoded["resultant"] is resultant
    assert decoded["taxels"] is taxels


@pytest.mark.parametrize("data", [b"", b"\x01\x02"])
def test_decode_auto_data_type_wrong_size_raises(data):
    with pytest.raises(ValueError, match="exactly 1 byte"):
        decode_auto_data_type(data)
