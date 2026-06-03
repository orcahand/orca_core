"""Tests for the encoder protocol codec (pure functions, no I/O)."""

from __future__ import annotations

import numpy as np
import pytest

from orca_core.hardware.sensing.constants import (
    AUTO_ENC_ANGLE_ERROR_BIT,
    AUTO_ENC_EFFECTIVE_LENGTH,
    AUTO_ENC_FRAME_SIZE,
    AUTO_ENC_NUM_JOINTS,
    AUTO_ENC_PARITY_BIT,
    ENCODER_COUNTS_PER_REV,
    ENCODER_LSB_DEG,
    PROTOCOL_HEADER_AUTO,
    PROTOCOL_HEADER_AUTO_ENC,
)
from orca_core.hardware.sensing.encoder_protocol import (
    encoder_to_joint_angle,
    parse_encoder_frame,
)
from orca_core.hardware.sensing.types import EncoderReading

from tests._encoder_helpers import make_encoder_frame


# Wire layout (built by ``make_encoder_frame``): header(2) + reserved(1) +
# effective_length(2 LE) + err(1) + payload(2 × AUTO_ENC_NUM_JOINTS) + LRC(1), where
# effective_length == 1 + payload bytes. Per-joint u16: bit 15 = even-parity over the
# whole word, bit 14 = angle error, bits 13:0 = 14-bit absolute angle.

def _zero_counts() -> np.ndarray:
    return np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)


# ---------------------------------------------------------------------------
# parse_encoder_frame
# ---------------------------------------------------------------------------

def test_parse_encoder_frame_decodes_parity_and_angle_error():
    """``parity_ok`` checks AS5048A even parity over the whole word; ``angle_error`` is bit 14."""
    raw = _zero_counts()
    raw[0] = 0x0000  # popcount=0 (even) → parity OK; bit 14 clear → no angle error
    raw[1] = 0x8000  # popcount=1 (odd) → parity BAD; bit 14 clear
    raw[2] = 0x8001  # popcount=2 (even) → parity OK; bit 14 clear
    raw[3] = 0x4000  # popcount=1 (odd) → parity BAD; bit 14 set → angle error
    raw[4] = 0xFFFF  # popcount=16 (even) → parity OK; bit 14 set
    raw[5] = 0xC003  # popcount=4 (even) → parity OK; bit 14 set
    raw[6] = 0x3FFF  # popcount=14 (even) → parity OK; bit 14 clear

    reading = parse_encoder_frame(make_encoder_frame(raw))
    np.testing.assert_array_equal(reading.raw_counts, raw)
    assert list(reading.parity_ok[:7]) == [True, False, True, False, True, True, True]
    assert list(reading.angle_error[:7]) == [False, False, False, True, True, True, False]


@pytest.mark.parametrize("frame,error_match", [
    pytest.param(make_encoder_frame(_zero_counts(), bad_lrc=True),                            "LRC",     id="bad_lrc"),
    pytest.param(make_encoder_frame(_zero_counts(), header=bytes([0xAA, 0x99])),              "header",  id="bad_header"),
    pytest.param(make_encoder_frame(_zero_counts(), header=PROTOCOL_HEADER_AUTO),             "header",  id="tactile_header"),
    pytest.param(make_encoder_frame(_zero_counts(), effective_length=AUTO_ENC_EFFECTIVE_LENGTH + 1),            "effective_length", id="bad_effective_length"),
    pytest.param(PROTOCOL_HEADER_AUTO_ENC,                                                    "size",    id="too_short"),
])
def test_parse_encoder_frame_rejects(frame, error_match):
    with pytest.raises(IOError, match=error_match):
        parse_encoder_frame(frame)


# ---------------------------------------------------------------------------
# wire-format round-trip
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("error_byte", [0x00, 0xFF])
def test_make_encoder_frame_round_trips_through_parser(error_byte):
    """``make_encoder_frame`` is the test-side wire-format builder; verify
    its output round-trips through the production parser so any bug in
    either surfaces as a test failure."""
    raw = np.arange(AUTO_ENC_NUM_JOINTS, dtype=np.uint16) * 1031
    raw[3] |= AUTO_ENC_PARITY_BIT
    raw[7] |= AUTO_ENC_ANGLE_ERROR_BIT
    raw[11] |= AUTO_ENC_PARITY_BIT | AUTO_ENC_ANGLE_ERROR_BIT

    frame = make_encoder_frame(raw_counts=raw, error_byte=error_byte)
    assert len(frame) == AUTO_ENC_FRAME_SIZE

    reading = parse_encoder_frame(frame, timestamp=12.5)
    assert isinstance(reading, EncoderReading)
    assert reading.error_byte == error_byte
    assert reading.timestamp == 12.5
    np.testing.assert_array_equal(reading.raw_counts, raw)


# ---------------------------------------------------------------------------
# encoder_to_joint_angle
# ---------------------------------------------------------------------------

def test_encoder_to_joint_angle_identity():
    """raw == anchor → joint angle == anchor angle, regardless of polarity."""
    midpoint = ENCODER_COUNTS_PER_REV // 2
    raw = np.full(AUTO_ENC_NUM_JOINTS, midpoint, dtype=np.uint16)
    anchor = np.full(AUTO_ENC_NUM_JOINTS, midpoint, dtype=np.int32)
    polarity = np.tile([1, -1], (AUTO_ENC_NUM_JOINTS + 1) // 2)[:AUTO_ENC_NUM_JOINTS].astype(np.int8)
    anchor_angle = np.linspace(-1.0, 1.0, AUTO_ENC_NUM_JOINTS)

    out = encoder_to_joint_angle(raw, anchor, polarity, anchor_angle)
    np.testing.assert_allclose(out, anchor_angle, atol=1e-9)


def test_encoder_to_joint_angle_strips_top_bits_from_raw():
    """Bits 15 and 14 (flags) must not affect the computed angle."""
    raw_clean = np.full(AUTO_ENC_NUM_JOINTS, 1234, dtype=np.uint16)
    raw_flagged = raw_clean | AUTO_ENC_PARITY_BIT | AUTO_ENC_ANGLE_ERROR_BIT
    anchor = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.int32)
    polarity = np.ones(AUTO_ENC_NUM_JOINTS, dtype=np.int8)
    anchor_angle = np.zeros(AUTO_ENC_NUM_JOINTS)

    a = encoder_to_joint_angle(raw_clean, anchor, polarity, anchor_angle)
    b = encoder_to_joint_angle(raw_flagged, anchor, polarity, anchor_angle)
    np.testing.assert_allclose(a, b, atol=1e-12)


@pytest.mark.parametrize("raw,anchor,expected_lsb", [
    pytest.param(0,                          ENCODER_COUNTS_PER_REV - 1, +1, id="wrap_near_zero"),
    pytest.param(ENCODER_COUNTS_PER_REV - 1, 0,                          -1, id="wrap_near_top"),
])
def test_encoder_to_joint_angle_wraparound(raw, anchor, expected_lsb):
    """Naive subtraction crosses the wrap; correction brings the result into (-180°, 180°]."""
    out = encoder_to_joint_angle(
        np.array([raw], dtype=np.uint16),
        np.array([anchor], dtype=np.int32),
        np.array([1], dtype=np.int8),
        np.array([0.0]),
    )
    np.testing.assert_allclose(out, expected_lsb * ENCODER_LSB_DEG, atol=1e-9)


def test_encoder_to_joint_angle_polarity_flip():
    """polarity=-1 mirrors (joint - anchor) around the anchor."""
    midpoint = ENCODER_COUNTS_PER_REV // 2
    raw = np.array([midpoint + 100], dtype=np.uint16)
    anchor = np.array([midpoint], dtype=np.int32)
    anchor_angle = np.array([0.5])

    plus = encoder_to_joint_angle(raw, anchor, np.array([1], dtype=np.int8), anchor_angle)
    minus = encoder_to_joint_angle(raw, anchor, np.array([-1], dtype=np.int8), anchor_angle)
    np.testing.assert_allclose(plus - anchor_angle, -(minus - anchor_angle), atol=1e-12)
