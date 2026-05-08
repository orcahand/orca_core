"""Wire-format encoder-frame builders for tests on top of ``MockHandSerialLink``."""
from __future__ import annotations

import numpy as np

from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_EFF_LEN,
    AUTO_ENC_NUM_JOINTS,
    PROTOCOL_HEADER_AUTO_ENC,
    PROTOCOL_RESERVED,
)
from orca_core.hardware.sensing.encoder_protocol import calculate_checksum


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
