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
    compute_distal_module_index,
    decode_resultant_auto,
    decode_taxels_auto,
    decode_combined_auto,
    decode_resultant_register,
    decode_connected_sensors,
    decode_num_taxels,
    decode_auto_data_type,
    encode_auto_data_type,
)
from orca_core.hardware.sensing.constants import (
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
    MIN_READ_RESPONSE_SIZE,
    MIN_WRITE_RESPONSE_SIZE,
    MODULES_PER_SLOT,
    DISTAL_MODULE_OFFSET,
)


# ---------------------------------------------------------------------------
# Checksum
# ---------------------------------------------------------------------------

class TestCalculateChecksum:
    def test_known_value(self):
        assert calculate_checksum(b"\x01\x02\x03") == 0xFA

    def test_round_trip(self):
        frame = b"\xAA\x55\x00\x03\x10\x00\x04\x00"
        checksum = calculate_checksum(frame)
        assert (sum(frame) + checksum) & 0xFF == 0

    def test_empty_frame(self):
        assert calculate_checksum(b"") == 0

    def test_single_byte(self):
        assert calculate_checksum(b"\x01") == 0xFF


class TestValidateAutoFrameLrc:
    def test_valid_frame(self):
        meta = b"\x00" + (3).to_bytes(2, "little")
        payload = b"\x00\x01\x02"
        frame_wo_lrc = b"\xAA\x56" + meta + payload
        lrc = calculate_checksum(frame_wo_lrc)
        assert validate_auto_frame_lrc(meta, payload, lrc) is True

    def test_invalid_lrc(self):
        meta = b"\x00" + (3).to_bytes(2, "little")
        payload = b"\x00\x01\x02"
        assert validate_auto_frame_lrc(meta, payload, 0xFF) is False


# ---------------------------------------------------------------------------
# Frame Size Helpers
# ---------------------------------------------------------------------------

class TestReadResponseBodySize:
    def test_known_value(self):
        # count=4 → meta(6) + data(4) + LRC(1) = 11
        assert read_response_body_size(4) == 11

    def test_single_byte(self):
        assert read_response_body_size(1) == 8


# ---------------------------------------------------------------------------
# Frame Builders
# ---------------------------------------------------------------------------

class TestBuildReadRequest:
    def test_structure(self):
        frame = build_read_request(address=0x0010, count=4)
        assert frame[:2] == PROTOCOL_HEADER_REQUEST
        assert frame[2] == 0x00  # reserved
        assert frame[3] == FUNC_CODE_READ
        assert int.from_bytes(frame[4:6], "little") == 0x0010
        assert int.from_bytes(frame[6:8], "little") == 4

    def test_lrc_valid(self):
        frame = build_read_request(address=0x0010, count=4)
        assert calculate_checksum(frame[:-1]) == frame[-1]


class TestBuildWriteRequest:
    def test_structure(self):
        frame = build_write_request(address=0x0017, data=b"\x01")
        assert frame[:2] == PROTOCOL_HEADER_REQUEST
        assert frame[2] == 0x00  # reserved
        assert frame[3] == FUNC_CODE_WRITE
        assert int.from_bytes(frame[4:6], "little") == 0x0017
        assert int.from_bytes(frame[6:8], "little") == 1
        assert frame[8] == 0x01  # data byte

    def test_lrc_valid(self):
        frame = build_write_request(address=0x0017, data=b"\x01")
        assert calculate_checksum(frame[:-1]) == frame[-1]


# ---------------------------------------------------------------------------
# Frame Parsers — response frames
# ---------------------------------------------------------------------------

def _build_read_response(data: bytes) -> bytes:
    """Helper: build a valid read response frame for testing."""
    meta = bytes([0x00, FUNC_CODE_READ]) + (0x0010).to_bytes(2, "little") + len(data).to_bytes(2, "little")
    frame_wo_lrc = PROTOCOL_HEADER_RESPONSE + meta + data
    return frame_wo_lrc + bytes([calculate_checksum(frame_wo_lrc)])


def _build_write_response(status: int) -> bytes:
    """Helper: build a valid write response frame for testing."""
    meta = bytes([0x00, FUNC_CODE_WRITE]) + (0x0017).to_bytes(2, "little") + (1).to_bytes(2, "little")
    frame_wo_lrc = PROTOCOL_HEADER_RESPONSE + meta + bytes([status])
    return frame_wo_lrc + bytes([calculate_checksum(frame_wo_lrc)])


class TestParseReadResponse:
    def test_extracts_data(self):
        frame = _build_read_response(b"\xAB\xCD\xEF\x01")
        assert parse_read_response(frame) == b"\xAB\xCD\xEF\x01"

    def test_single_byte(self):
        frame = _build_read_response(b"\x42")
        assert parse_read_response(frame) == b"\x42"

    def test_bad_lrc_raises(self):
        frame = bytearray(_build_read_response(b"\x01\x02"))
        frame[-1] ^= 0xFF  # corrupt LRC
        with pytest.raises(IOError, match="LRC mismatch"):
            parse_read_response(bytes(frame))

    def test_too_short_raises(self):
        with pytest.raises(IOError, match="too short"):
            parse_read_response(b"\xAA\x55\x00")

    def test_wrong_func_code_raises(self):
        meta = bytes([0x00, FUNC_CODE_WRITE]) + (0x0010).to_bytes(2, "little") + (1).to_bytes(2, "little")
        frame_wo_lrc = PROTOCOL_HEADER_RESPONSE + meta + b"\x00"
        frame = frame_wo_lrc + bytes([calculate_checksum(frame_wo_lrc)])
        with pytest.raises(IOError, match="Expected read response"):
            parse_read_response(frame)

    def test_wrong_header_raises(self):
        frame = bytearray(_build_read_response(b"\x01\x02"))
        frame[0:2] = PROTOCOL_HEADER_AUTO  # AA 56 instead of AA 55
        frame[-1] = calculate_checksum(bytes(frame[:-1]))  # fix LRC
        with pytest.raises(IOError, match="Expected response header"):
            parse_read_response(bytes(frame))


class TestParseWriteResponse:
    def test_success(self):
        frame = _build_write_response(status=0x00)
        parse_write_response(frame)  # should not raise

    def test_failure_status_raises(self):
        frame = _build_write_response(status=0x01)
        with pytest.raises(IOError, match="Write failed"):
            parse_write_response(frame)

    def test_bad_lrc_raises(self):
        frame = bytearray(_build_write_response(status=0x00))
        frame[-1] ^= 0xFF
        with pytest.raises(IOError, match="LRC mismatch"):
            parse_write_response(bytes(frame))

    def test_too_short_raises(self):
        with pytest.raises(IOError, match="too short"):
            parse_write_response(b"\xAA\x55\x00")

    def test_truncated_payload_raises(self):
        # Build a frame that claims 10 payload bytes but only has 1
        meta = bytes([0x00, FUNC_CODE_WRITE]) + (0x0017).to_bytes(2, "little") + (10).to_bytes(2, "little")
        frame_wo_lrc = PROTOCOL_HEADER_RESPONSE + meta + b"\x00"
        frame = frame_wo_lrc + bytes([calculate_checksum(frame_wo_lrc)])
        with pytest.raises(IOError, match="truncated"):
            parse_write_response(frame)

    def test_wrong_header_raises(self):
        frame = bytearray(_build_write_response(status=0x00))
        frame[0:2] = PROTOCOL_HEADER_AUTO
        frame[-1] = calculate_checksum(bytes(frame[:-1]))
        with pytest.raises(IOError, match="Expected response header"):
            parse_write_response(bytes(frame))


class TestExtractWriteResponseDataLength:
    def test_known_value(self):
        # meta: reserved(1) + func(1) + addr(2) + nbytes(2)
        meta = bytes([0x00, FUNC_CODE_WRITE, 0x17, 0x00, 0x03, 0x00])
        assert extract_write_response_data_length(meta) == 3

    def test_wrong_size_raises(self):
        with pytest.raises(ValueError, match="must be 6 bytes"):
            extract_write_response_data_length(b"\x00\x00\x00\x00")


# ---------------------------------------------------------------------------
# Frame Parsers — auto-stream frames
# ---------------------------------------------------------------------------

class TestExtractAutoFrameEffLen:
    def test_known_value(self):
        # reserved(1) + eff_len(2 LE) = 3 bytes
        meta = b"\x00" + (42).to_bytes(2, "little")
        assert extract_auto_frame_eff_len(meta) == 42

    def test_max_valid(self):
        meta = b"\x00" + MAX_AUTO_FRAME_EFF_LEN.to_bytes(2, "little")
        assert extract_auto_frame_eff_len(meta) == MAX_AUTO_FRAME_EFF_LEN

    def test_exceeds_max_raises(self):
        meta = b"\x00" + (MAX_AUTO_FRAME_EFF_LEN + 1).to_bytes(2, "little")
        with pytest.raises(ValueError, match="Invalid eff_len"):
            extract_auto_frame_eff_len(meta)


class TestSplitAutoPayload:
    def test_splits_error_code_and_data(self):
        err, data = unpack_auto_payload(b"\x00\x01\x02\x03")
        assert err == 0
        assert data == b"\x01\x02\x03"

    def test_nonzero_error_code(self):
        err, data = unpack_auto_payload(b"\x05\xAB")
        assert err == 5
        assert data == b"\xAB"

    def test_error_code_only(self):
        err, data = unpack_auto_payload(b"\x01")
        assert err == 1
        assert data == b""


# ---------------------------------------------------------------------------
# Payload Size Computation
# ---------------------------------------------------------------------------

class TestPayloadSizeComputation:
    def test_resultant_size(self):
        assert compute_resultant_payload_size(3) == 3 * BYTES_PER_RESULTANT

    def test_resultant_size_zero(self):
        assert compute_resultant_payload_size(0) == 0

    def test_taxel_size(self):
        active = ["thumb", "index"]
        num_taxels = {"thumb": 51, "index": 87}
        assert compute_taxel_payload_size(active, num_taxels) == (51 + 87) * BYTES_PER_TAXEL

    def test_taxel_size_missing_finger_raises(self):
        with pytest.raises(KeyError):
            compute_taxel_payload_size(["thumb"], {"index": 87})

    def test_combined_size(self):
        active = ["thumb", "index"]
        num_taxels = {"thumb": 51, "index": 87}
        expected = 2 * BYTES_PER_RESULTANT + (51 + 87) * BYTES_PER_TAXEL
        assert compute_combined_payload_size(active, num_taxels) == expected


# ---------------------------------------------------------------------------
# Module Index Computation
# ---------------------------------------------------------------------------

class TestComputeDistalModuleIndex:
    def test_slot_zero(self):
        assert compute_distal_module_index(0) == DISTAL_MODULE_OFFSET

    def test_slot_four(self):
        assert compute_distal_module_index(4) == 4 * MODULES_PER_SLOT + DISTAL_MODULE_OFFSET


# ---------------------------------------------------------------------------
# Payload Decoders — auto-stream
# ---------------------------------------------------------------------------

class TestDecodeResultantAuto:
    def test_known_values(self):
        data = struct.pack("<hhH", 100, -50, 200)
        result = decode_resultant_auto(data, ["thumb"])
        assert result["thumb"] == [10.0, -5.0, 20.0]

    def test_two_sensors(self):
        data = struct.pack("<hhH", 10, 20, 30) + struct.pack("<hhH", -10, -20, 40)
        result = decode_resultant_auto(data, ["thumb", "index"])
        assert result["thumb"] == [1.0, 2.0, 3.0]
        assert result["index"] == [-1.0, -2.0, 4.0]

    def test_wrong_size_raises(self):
        with pytest.raises(ValueError, match="size mismatch"):
            decode_resultant_auto(b"\x00" * 5, ["thumb"])

    def test_error_includes_hex(self):
        with pytest.raises(ValueError, match="first bytes"):
            decode_resultant_auto(b"\xDE\xAD", ["thumb"])


class TestDecodeTaxelsAuto:
    def test_known_values(self):
        data = struct.pack("bbB", 10, -5, 20) + struct.pack("bbB", 0, 0, 50)
        result = decode_taxels_auto(data, ["thumb"], {"thumb": 2})
        assert len(result["thumb"]) == 2
        assert result["thumb"][0] == [1.0, -0.5, 2.0]
        assert result["thumb"][1] == [0.0, 0.0, 5.0]

    def test_wrong_size_raises(self):
        with pytest.raises(ValueError, match="size mismatch"):
            decode_taxels_auto(b"\x00" * 5, ["thumb"], {"thumb": 2})


class TestDecodeCombinedAuto:
    def test_interleaved(self):
        data = (
            struct.pack("<hhH", 100, 0, 50)
            + struct.pack("bbB", 10, 0, 5)
            + struct.pack("<hhH", 0, -100, 200)
            + struct.pack("bbB", 0, -10, 20)
        )
        forces, taxels = decode_combined_auto(
            data, ["thumb", "index"], {"thumb": 1, "index": 1},
        )
        assert forces["thumb"] == [10.0, 0.0, 5.0]
        assert forces["index"] == [0.0, -10.0, 20.0]
        assert taxels["thumb"][0] == [1.0, 0.0, 0.5]
        assert taxels["index"][0] == [0.0, -1.0, 2.0]


# ---------------------------------------------------------------------------
# Payload Decoders — register block
# ---------------------------------------------------------------------------

class TestDecodeResultantBlock:
    def test_known_values(self):
        # Module index for thumb (slot 0): 0*4+2 = 2, byte offset = 12
        data = b"\x00" * 168
        data_mut = bytearray(data)
        struct.pack_into("<hhH", data_mut, 12, 100, -50, 200)
        result = decode_resultant_register(data_mut, ["thumb"], {"thumb": 2})
        assert result["thumb"] == [10.0, -5.0, 20.0]

    def test_too_short_raises(self):
        with pytest.raises(ValueError, match="too short"):
            decode_resultant_register(b"\x00" * 10, ["thumb"], {"thumb": 2})


# ---------------------------------------------------------------------------
# Register Decoders
# ---------------------------------------------------------------------------

class TestDecodeConnectedSensors:
    def test_all_connected(self):
        # Slot 0: byte 0 bit 2 → 0x04
        # Slot 1: byte 0 bit 6 → 0x40
        # Slot 2: byte 1 bit 2 → 0x04
        # Slot 3: byte 1 bit 6 → 0x40
        # Slot 4: byte 2 bit 2 → 0x04
        data = bytes([0x44, 0x44, 0x04, 0x00])
        id_to_finger = {0: "thumb", 1: "index", 2: "middle", 3: "ring", 4: "pinky"}
        result = decode_connected_sensors(data, id_to_finger)
        assert all(result.values())

    def test_none_connected(self):
        data = bytes([0x00, 0x00, 0x00, 0x00])
        id_to_finger = {0: "thumb", 1: "index", 2: "middle", 3: "ring", 4: "pinky"}
        result = decode_connected_sensors(data, id_to_finger)
        assert not any(result.values())

    def test_partial(self):
        # Only slot 0 (bit 2 of byte 0) and slot 4 (bit 2 of byte 2)
        data = bytes([0x04, 0x00, 0x04, 0x00])
        id_to_finger = {0: "thumb", 1: "index", 2: "middle", 3: "ring", 4: "pinky"}
        result = decode_connected_sensors(data, id_to_finger)
        assert result["thumb"] is True
        assert result["index"] is False
        assert result["pinky"] is True

    def test_too_short_raises(self):
        with pytest.raises(ValueError, match="Expected 4 bytes"):
            decode_connected_sensors(b"\x00\x00", {0: "thumb", 1: "index", 2: "middle", 3: "ring", 4: "pinky"})


class TestDecodeNumTaxels:
    def test_known_values(self):
        # 28 uint16 values (56 bytes total). The distal register offsets are
        # [0x0034, 0x003C, 0x0044, 0x004C, 0x0054], which at base 0x0030
        # correspond to indices [2, 6, 10, 14, 18] in the uint16 array.
        data = bytearray(56)
        struct.pack_into("<H", data, 2 * 2, 51)   # slot 0 → index 2
        struct.pack_into("<H", data, 6 * 2, 87)   # slot 1 → index 6
        struct.pack_into("<H", data, 10 * 2, 87)  # slot 2 → index 10
        struct.pack_into("<H", data, 14 * 2, 87)  # slot 3 → index 14
        struct.pack_into("<H", data, 18 * 2, 51)  # slot 4 → index 18
        id_to_finger = {0: "thumb", 1: "index", 2: "middle", 3: "ring", 4: "pinky"}
        result = decode_num_taxels(bytes(data), id_to_finger)
        assert result == {"thumb": 51, "index": 87, "middle": 87, "ring": 87, "pinky": 51}


# ---------------------------------------------------------------------------
# Register Codecs
# ---------------------------------------------------------------------------

class TestDecodeAutoDataType:
    def test_resultant_only(self):
        result = decode_auto_data_type(bytes([AUTO_DATA_RESULTANT]))
        assert result["resultant"] is True
        assert result["taxels"] is False

    def test_combined(self):
        result = decode_auto_data_type(bytes([AUTO_DATA_RESULTANT | AUTO_DATA_TAXELS]))
        assert result["resultant"] is True
        assert result["taxels"] is True

    def test_round_trip(self):
        encoded = encode_auto_data_type(resultant=True, taxels=True)
        decoded = decode_auto_data_type(encoded)
        assert decoded["resultant"] is True
        assert decoded["taxels"] is True


class TestEncodeAutoDataType:
    def test_resultant_only(self):
        assert encode_auto_data_type(resultant=True, taxels=False) == bytes([AUTO_DATA_RESULTANT])

    def test_taxels_only(self):
        assert encode_auto_data_type(resultant=False, taxels=True) == bytes([AUTO_DATA_TAXELS])

    def test_combined(self):
        assert encode_auto_data_type(resultant=True, taxels=True) == bytes([AUTO_DATA_RESULTANT | AUTO_DATA_TAXELS])

    def test_none(self):
        assert encode_auto_data_type(resultant=False, taxels=False) == bytes([0x00])


# ---------------------------------------------------------------------------
# Validation paths — frame builders
# ---------------------------------------------------------------------------

class TestBuildReadRequestValidation:
    def test_zero_count_raises(self):
        with pytest.raises(ValueError, match="count must be > 0"):
            build_read_request(address=0x0010, count=0)

    def test_negative_count_raises(self):
        with pytest.raises(ValueError, match="count must be > 0"):
            build_read_request(address=0x0010, count=-1)

    def test_address_overflow_raises(self):
        with pytest.raises(ValueError, match="address"):
            build_read_request(address=0x10000, count=1)

    def test_negative_address_raises(self):
        with pytest.raises(ValueError, match="address"):
            build_read_request(address=-1, count=1)


class TestBuildWriteRequestValidation:
    def test_empty_data_raises(self):
        with pytest.raises(ValueError, match="data must not be empty"):
            build_write_request(address=0x0017, data=b"")

    def test_address_overflow_raises(self):
        with pytest.raises(ValueError, match="address"):
            build_write_request(address=0x10000, data=b"\x01")


# ---------------------------------------------------------------------------
# Validation paths — register decoders
# ---------------------------------------------------------------------------

class TestDecodeConnectedSensorsValidation:
    def test_wrong_mapping_size_raises(self):
        data = bytes([0x00, 0x00, 0x00, 0x00])
        with pytest.raises(ValueError, match="must have 5 entries"):
            decode_connected_sensors(data, {0: "thumb", 1: "index"})


class TestDecodeNumTaxelsValidation:
    def test_wrong_data_length_raises(self):
        id_to_finger = {0: "thumb", 1: "index", 2: "middle", 3: "ring", 4: "pinky"}
        with pytest.raises(ValueError, match="size mismatch"):
            decode_num_taxels(b"\x00" * 10, id_to_finger)

    def test_wrong_mapping_size_raises(self):
        data = bytes(56)
        with pytest.raises(ValueError, match="must have 5 entries"):
            decode_num_taxels(data, {0: "thumb"})


class TestDecodeAutoDataTypeValidation:
    def test_empty_data_raises(self):
        with pytest.raises(ValueError, match="exactly 1 byte"):
            decode_auto_data_type(b"")

    def test_oversized_data_raises(self):
        with pytest.raises(ValueError, match="exactly 1 byte"):
            decode_auto_data_type(b"\x01\x02")
