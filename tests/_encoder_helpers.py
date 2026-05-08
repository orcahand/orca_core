"""Wire-format encoder-frame builders for tests on top of ``MockHandSerialLink``."""
from __future__ import annotations

import time
from typing import Dict

import numpy as np

from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_EFF_LEN,
    AUTO_ENC_NUM_JOINTS,
    ENCODER_COUNTS_PER_REV,
    JOINT_TO_ENCODER_SLOT,
    PROTOCOL_HEADER_AUTO_ENC,
    PROTOCOL_RESERVED,
)
from orca_core.hardware.sensing.encoder_protocol import calculate_checksum
from orca_core.hardware.sensing.types import EncoderReading


def make_encoder_frame(
    raw_counts: np.ndarray | None = None,
    err_byte: int = 0,
    *,
    bad_lrc: bool = False,
    override_eff_len: int | None = None,
) -> bytes:
    """Build a wire-format AA A9 encoder frame.

    With ``override_eff_len`` set, the meta and payload sizes follow that
    value so the link still accepts the frame; the resulting total size
    differs from the encoder spec's 39 bytes, which exercises the client's
    exact-size check.
    """
    if raw_counts is None:
        raw_counts = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    if override_eff_len is None:
        eff_len = AUTO_ENC_EFF_LEN
        payload = bytes([err_byte]) + raw_counts.astype("<u2").tobytes()
    else:
        eff_len = override_eff_len
        payload_bytes = max(eff_len - 1, 0)
        payload = bytes([err_byte]) + b"\x00" * payload_bytes
    body = (
        PROTOCOL_HEADER_AUTO_ENC
        + bytes([PROTOCOL_RESERVED])
        + eff_len.to_bytes(2, "little")
        + payload
    )
    lrc = calculate_checksum(body)
    if bad_lrc:
        lrc ^= 0xFF
    return body + bytes([lrc])


def feed_encoder_frame(
    link: MockHandSerialLink,
    raw_counts: np.ndarray | None = None,
    err_byte: int = 0,
    *,
    bad_lrc: bool = False,
    override_eff_len: int | None = None,
) -> None:
    link.feed_bytes(
        make_encoder_frame(
            raw_counts,
            err_byte,
            bad_lrc=bad_lrc,
            override_eff_len=override_eff_len,
        )
    )


# ---------------------------------------------------------------------------
# Mock joint-encoder source for calibration integration tests
# ---------------------------------------------------------------------------


_DEFAULT_COUNTS_PER_RAD = ENCODER_COUNTS_PER_REV / (2.0 * np.pi)


class MockJointEncoderSource:
    """Synthesise per-joint encoder counts from mock motor positions.

    Implements ``get_latest_encoder_reading`` (the only method the
    calibration sweep calls on its encoder client). Counts wrap into the
    14-bit range so feeding them back through ``_raw_to_joint_angle``
    yields the joint-space the mock motor is in.
    """

    def __init__(
        self,
        dxl_client,
        joint_to_motor_map: Dict[str, int],
        counts_per_rad: float = _DEFAULT_COUNTS_PER_RAD,
        polarities: Dict[str, int] | None = None,
        motor_offsets: Dict[str, float] | None = None,
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

    def get_latest_encoder_reading(self) -> EncoderReading:
        raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
        for joint, slot in JOINT_TO_ENCODER_SLOT.items():
            if joint == "wrist":
                continue
            raw[slot] = self._raw_count_for_joint(joint)
        return EncoderReading(
            raw_counts=raw,
            parity_ok=np.ones(AUTO_ENC_NUM_JOINTS, dtype=bool),
            angle_error=np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool),
            err_byte=0,
            timestamp=time.monotonic(),
        )
