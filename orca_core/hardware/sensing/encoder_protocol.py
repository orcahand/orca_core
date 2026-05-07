"""Codec for the joint-encoder auto-stream (AA A9 frames).

Pure functions only — no I/O, no state, no threading.
"""
from __future__ import annotations

import math

import numpy as np

from orca_core.hardware.sensing.constants import (
    AUTO_ENC_ANGLE_ERROR_BIT,
    AUTO_ENC_ANGLE_MASK,
    AUTO_ENC_EFF_LEN,
    AUTO_ENC_FRAME_SIZE,
    AUTO_ENC_PARITY_BIT,
    AUTO_ENC_PAYLOAD_BYTES,
    AUTO_ENC_PAYLOAD_OFFSET,
    ENCODER_LSB_RAD,
    PROTOCOL_HEADER_AUTO_ENC,
)
from orca_core.hardware.sensing.types import EncoderReading


# =========================================================================
# Checksum
# =========================================================================

def calculate_checksum(frame: bytes) -> int:
    """LRC checksum: two's complement of the low byte of the sum."""
    return (0x100 - (sum(frame) & 0xFF)) & 0xFF


# =========================================================================
# Frame parser
# =========================================================================

def parse_auto_enc_frame(frame: bytes, *, timestamp: float = 0.0) -> EncoderReading:
    """Validate an encoder auto-stream frame and decode the joint payload.

    Raises ``IOError`` on size, header, eff_len, or LRC mismatch.
    """
    if len(frame) != AUTO_ENC_FRAME_SIZE:
        raise IOError(
            f"Encoder frame size mismatch: expected {AUTO_ENC_FRAME_SIZE} bytes, "
            f"got {len(frame)}"
        )
    if frame[:2] != PROTOCOL_HEADER_AUTO_ENC:
        raise IOError(
            f"Encoder frame header mismatch: expected {PROTOCOL_HEADER_AUTO_ENC.hex()}, "
            f"got {frame[:2].hex()}"
        )
    if calculate_checksum(frame[:-1]) != frame[-1]:
        raise IOError("Encoder frame LRC mismatch")
    eff_len = int.from_bytes(frame[3:5], "little")
    if eff_len != AUTO_ENC_EFF_LEN:
        raise IOError(
            f"Encoder frame eff_len mismatch: expected {AUTO_ENC_EFF_LEN}, got {eff_len}"
        )

    payload = frame[AUTO_ENC_PAYLOAD_OFFSET : AUTO_ENC_PAYLOAD_OFFSET + AUTO_ENC_PAYLOAD_BYTES]
    raw_counts = np.frombuffer(payload, dtype="<u2").copy()
    return EncoderReading(
        raw_counts=raw_counts,
        parity_ok=(raw_counts & AUTO_ENC_PARITY_BIT) != 0,
        angle_error=(raw_counts & AUTO_ENC_ANGLE_ERROR_BIT) != 0,
        err_byte=frame[5],
        timestamp=timestamp,
    )


# =========================================================================
# Raw counts → joint angle
# =========================================================================

def encoder_to_joint_angle(
    raw_counts: np.ndarray,
    anchor_count: np.ndarray,
    polarity: np.ndarray,
    anchor_angle_rad: np.ndarray,
) -> np.ndarray:
    """Convert raw 14-bit encoder counts into joint angles in radians.

    Inputs broadcast against each other; typical use is shape ``(16,)``.
    Wrap correction is required because the encoder is absolute over a
    14-bit range — naive subtraction can land on either side of the wrap
    once the joint moves more than π/16384 from the calibration anchor.
    """
    a14 = raw_counts.astype(np.int64) & AUTO_ENC_ANGLE_MASK
    delta = (a14 - anchor_count.astype(np.int64)) * ENCODER_LSB_RAD
    delta = np.where(delta > math.pi, delta - 2.0 * math.pi, delta)
    delta = np.where(delta <= -math.pi, delta + 2.0 * math.pi, delta)
    return polarity.astype(np.float64) * delta + anchor_angle_rad


# =========================================================================
# Wire-format encoder — for mocking hardware in tests only
# =========================================================================

def encode_auto_enc_frame_for_mock(
    raw_counts: np.ndarray,
    err_byte: int = 0,
) -> bytes:
    """Encode a (16,) array of raw u16 counts into a wire-format encoder frame."""
    body = (
        PROTOCOL_HEADER_AUTO_ENC
        + bytes([0x00])
        + AUTO_ENC_EFF_LEN.to_bytes(2, "little")
        + bytes([err_byte])
        + raw_counts.astype("<u2").tobytes()
    )
    return body + bytes([calculate_checksum(body)])
