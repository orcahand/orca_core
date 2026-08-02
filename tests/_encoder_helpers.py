"""Wire-format encoder-frame builders for tests on top of ``MockHandSerialLink``."""
from __future__ import annotations

import time

import numpy as np

from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    ENCODER_COUNTS_PER_REV,
    JOINT_TO_ENCODER_SLOT,
    PROTOCOL_HEADER_AUTO_ENC,
    PROTOCOL_RESERVED,
)
from orca_core.hardware.sensing.framing import calculate_checksum
from orca_core.hardware.sensing.types import EncoderReading


def make_encoder_frame(
    raw_counts: np.ndarray | None = None,
    error_byte: int = 0,
    *,
    bad_lrc: bool = False,
    header: bytes = PROTOCOL_HEADER_AUTO_ENC,
    effective_length: int | None = None,
) -> bytes:
    """Build a wire-format AA A9 encoder frame.

    The payload is always built from ``raw_counts``, so the frame keeps a
    consistent size and reaches the parser's later checks. ``header`` and
    ``effective_length`` forge malformed frames: a non-A9 ``header`` exercises the
    header check, and an ``effective_length`` that disagrees with the payload
    exercises the length check.
    """
    if raw_counts is None:
        raw_counts = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    payload = bytes([error_byte]) + raw_counts.astype("<u2").tobytes()
    effective_length_field = len(payload) if effective_length is None else effective_length
    body = (
        header
        + bytes([PROTOCOL_RESERVED])
        + effective_length_field.to_bytes(2, "little")
        + payload
    )
    lrc = calculate_checksum(body) ^ (0xFF if bad_lrc else 0)
    return body + bytes([lrc])


def feed_encoder_frame(
    link: MockHandSerialLink,
    raw_counts: np.ndarray | None = None,
    error_byte: int = 0,
    *,
    bad_lrc: bool = False,
    header: bytes = PROTOCOL_HEADER_AUTO_ENC,
    effective_length: int | None = None,
) -> None:
    link.feed_bytes(
        make_encoder_frame(
            raw_counts, error_byte, bad_lrc=bad_lrc, header=header, effective_length=effective_length
        )
    )


# ---------------------------------------------------------------------------
# Mock joint-encoder source for calibration integration tests
# ---------------------------------------------------------------------------


_DEFAULT_COUNTS_PER_RAD = ENCODER_COUNTS_PER_REV / (2.0 * np.pi)


class MockJointEncoderSource:
    """Synthesise per-joint encoder counts from mock motor positions.

    Implements ``get_latest`` (the only method the
    calibration sweep calls on its encoder client). Counts wrap into the
    14-bit range so feeding them back through ``_raw_to_joint_angle``
    yields the joint-space the mock motor is in.
    """

    def __init__(
        self,
        dxl_client,
        joint_to_motor_map: dict[str, int],
        counts_per_rad: float = _DEFAULT_COUNTS_PER_RAD,
        polarities: dict[str, int] | None = None,
        motor_offsets: dict[str, float] | None = None,
    ):
        self._dxl = dxl_client
        self._joint_to_motor_map = dict(joint_to_motor_map)
        self._counts_per_rad = float(counts_per_rad)
        self._polarities = dict(polarities or {j: 1 for j in joint_to_motor_map})
        self._motor_offsets = dict(motor_offsets or {})

    def _raw_count_for_joint(self, joint: str) -> int:
        motor_id = self._joint_to_motor_map.get(joint)
        if motor_id is None:
            return 0
        motor_pos = float(self._dxl._pos.get(motor_id, 0.0))
        offset = self._motor_offsets.get(joint, 0.0)
        polarity = self._polarities.get(joint, 1)
        count = int(
            round(polarity * (motor_pos - offset) * self._counts_per_rad)
        )
        return count % ENCODER_COUNTS_PER_REV

    def get_latest(self) -> EncoderReading:
        raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
        for joint, slot in JOINT_TO_ENCODER_SLOT.items():
            if joint == "wrist":
                continue
            raw[slot] = self._raw_count_for_joint(joint)
        return EncoderReading(
            raw_counts=raw,
            parity_ok=np.ones(AUTO_ENC_NUM_JOINTS, dtype=bool),
            angle_error=np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool),
            error_byte=0,
            timestamp=time.monotonic(),
        )
