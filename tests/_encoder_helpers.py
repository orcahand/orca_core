"""Wire-format encoder-frame builders for tests on top of ``MockHandSerialLink``."""
from __future__ import annotations

import time

import numpy as np

from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    AUTO_ENC_PARITY_BIT,
    ENCODER_COUNTS_PER_REV,
    ENCODER_LSB_DEG,
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


def with_even_parity(counts) -> np.ndarray:
    """Set bit 15 so each word passes the chip's even-parity check."""
    counts = np.asarray(counts, dtype=np.uint16) & ~np.uint16(AUTO_ENC_PARITY_BIT)
    odd = np.array([bin(int(c)).count("1") & 1 for c in counts.ravel()], dtype=bool)
    return counts | (odd.reshape(counts.shape) * np.uint16(AUTO_ENC_PARITY_BIT))


def full_counts(value: int) -> np.ndarray:
    """A parity-clean, full-width count array with every slot at ``value``."""
    return with_even_parity(np.full(AUTO_ENC_NUM_JOINTS, value, dtype=np.uint16))


# ---------------------------------------------------------------------------
# Mock joint-encoder source for calibration integration tests
# ---------------------------------------------------------------------------


_DEFAULT_COUNTS_PER_RAD = ENCODER_COUNTS_PER_REV / (2.0 * np.pi)

# The mock motors simulate hardstops at ±1.0 rad, so a calibration sweep
# always travels this far in motor space (mock_dynamixel_client).
MOCK_MOTOR_TRAVEL_RAD = 2.0


def rom_consistent_counts_per_rad(
    joint_roms: dict[str, list],
    span_scale: dict[str, float] | float = 1.0,
) -> dict[str, float]:
    """Per-joint ``counts_per_rad`` making a mock sweep measure each joint's
    configured ROM span.

    The flat default rate makes every joint measure the same span whatever its
    configured ROM, which is fine while the counts are only used to anchor the
    encoder frame but meaningless once the sweep's span is itself a
    measurement. ``span_scale`` fakes a hand whose real travel differs from
    nominal — 0.95 for a joint that falls 5% short of its configured ROM.
    """
    scales = (
        {joint: float(span_scale) for joint in joint_roms}
        if isinstance(span_scale, (int, float))
        else dict(span_scale)
    )
    return {
        joint: (rom[1] - rom[0]) * scales.get(joint, 1.0)
        / MOCK_MOTOR_TRAVEL_RAD / ENCODER_LSB_DEG
        for joint, rom in joint_roms.items()
    }


class MockJointEncoderSource:
    """Synthesise per-joint encoder counts from mock motor positions.

    Implements ``get_latest_unfiltered`` (the only method the
    calibration sweep calls on its encoder client). Counts wrap into the
    14-bit range so feeding them back through ``_raw_to_joint_angle``
    yields the joint-space the mock motor is in.

    ``counts_per_rad`` accepts a per-joint mapping — see
    :func:`rom_consistent_counts_per_rad` — for tests that care what span the
    sweep measures.
    """

    def __init__(
        self,
        dxl_client,
        joint_to_motor_map: dict[str, int],
        counts_per_rad: float | dict[str, float] = _DEFAULT_COUNTS_PER_RAD,
        polarities: dict[str, int] | None = None,
        motor_offsets: dict[str, float] | None = None,
        dead_slots: frozenset[int] = frozenset(),
    ):
        self._dxl = dxl_client
        self._joint_to_motor_map = dict(joint_to_motor_map)
        self._counts_per_rad = (
            dict(counts_per_rad)
            if isinstance(counts_per_rad, dict)
            else {j: float(counts_per_rad) for j in joint_to_motor_map}
        )
        self._polarities = dict(polarities or {j: 1 for j in joint_to_motor_map})
        self._motor_offsets = dict(motor_offsets or {})
        self._dead_slots = frozenset(dead_slots)

    def _raw_count_for_joint(self, joint: str) -> int:
        motor_id = self._joint_to_motor_map.get(joint)
        if motor_id is None:
            return 0
        motor_pos = float(self._dxl._pos.get(motor_id, 0.0))
        offset = self._motor_offsets.get(joint, 0.0)
        polarity = self._polarities.get(joint, 1)
        rate = self._counts_per_rad.get(joint, _DEFAULT_COUNTS_PER_RAD)
        count = int(round(polarity * (motor_pos - offset) * rate))
        return count % ENCODER_COUNTS_PER_REV

    def get_latest_unfiltered(self) -> EncoderReading:
        raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
        for joint, slot in JOINT_TO_ENCODER_SLOT.items():
            if slot in self._dead_slots:
                continue  # a dead slot reads a constant, parity-clean 0
            raw[slot] = self._raw_count_for_joint(joint)
        return EncoderReading(
            raw_counts=raw,
            parity_ok=np.ones(AUTO_ENC_NUM_JOINTS, dtype=bool),
            angle_error=np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool),
            error_byte=0,
            timestamp=time.monotonic(),
        )

    get_latest = get_latest_unfiltered
