"""Codec for the joint-encoder auto-stream (AA A9 frames).

Pure functions only — no I/O, no state, no threading.
"""
from __future__ import annotations

import numpy as np

from orca_core.hardware.sensing.constants import (
    AUTO_ENC_ANGLE_ERROR_BIT,
    AUTO_ENC_ANGLE_MASK,
    AUTO_ENC_EFF_LEN,
    AUTO_ENC_FRAME_SIZE,
    AUTO_ENC_PARITY_BIT,
    AUTO_ENC_PAYLOAD_BYTES,
    AUTO_ENC_PAYLOAD_OFFSET,
    ENCODER_LSB_DEG,
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

def parse_encoder_frame(frame: bytes, *, timestamp: float = 0.0) -> EncoderReading:
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
        parity_ok=_check_even_parity(raw_counts),
        angle_error=(raw_counts & AUTO_ENC_ANGLE_ERROR_BIT) != 0,
        err_byte=frame[5],
        timestamp=timestamp,
    )


def _check_even_parity(raw_counts: np.ndarray) -> np.ndarray:
    """True per joint where the encoder's even-parity check passes.

    The chip sets bit 15 such that the popcount across the full 16-bit
    word is even.
    """
    x = raw_counts.astype(np.uint16)
    x = (x & 0x5555) + ((x >> 1) & 0x5555)
    x = (x & 0x3333) + ((x >> 2) & 0x3333)
    x = (x & 0x0F0F) + ((x >> 4) & 0x0F0F)
    x = (x & 0x00FF) + ((x >> 8) & 0x00FF)
    return (x & 1) == 0


# =========================================================================
# Raw counts → joint angle
# =========================================================================

def encoder_to_joint_angle(
    raw_counts: np.ndarray,
    anchor_count: np.ndarray,
    polarity: np.ndarray,
    anchor_angle_deg: np.ndarray,
) -> np.ndarray:
    """Convert raw 14-bit encoder counts into joint angles in degrees.

    All four inputs are per-joint arrays of the same shape; the math
    runs element-wise. Wrap correction is needed because the encoder is
    absolute over one turn
    """
    a14 = raw_counts.astype(np.int64) & AUTO_ENC_ANGLE_MASK
    delta_deg = (a14 - anchor_count.astype(np.int64)) * ENCODER_LSB_DEG
    delta_deg = np.where(delta_deg > 180.0, delta_deg - 360.0, delta_deg)
    delta_deg = np.where(delta_deg <= -180.0, delta_deg + 360.0, delta_deg)
    return polarity.astype(np.float64) * delta_deg + anchor_angle_deg


