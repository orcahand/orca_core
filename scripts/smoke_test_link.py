"""End-to-end smoke test for HandSerialLink against the encoder firmware.

Covers both link paths:
  - asynchronous broadcast: counts AA A9 frames over a 5 s window, decodes
    the latest one, prints joint angles for the first 6 channels.
  - synchronous request: reads the firmware's stats register and cross-checks
    its tx counter against host-side frame count.

Usage:
    uv run python scripts/smoke_test_link.py /dev/cu.usbserial-XXXX
    PORT=/dev/cu.usbserial-XXXX uv run python scripts/smoke_test_link.py
"""
from __future__ import annotations

import math
import os
import struct
import sys
import time

import numpy as np

from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    ENCODER_COUNTS_PER_REV,
    PROTOCOL_BYTE_AUTO_ENC,
)
from orca_core.hardware.sensing.encoder_protocol import parse_auto_enc_frame
from orca_core.hardware.sensing.tactile_protocol import (
    build_read_request,
    parse_read_response,
)

# Firmware register layout (orca_firmware/.../commboard.h)
REG_STATS_ADDR = 0x0700
REG_STATS_LEN = 32

STREAM_DURATION_S = 5.0
EXPECTED_FRAME_RATE_HZ = 500
PASS_RATE_THRESHOLD = 0.9


def _decode_first_joints(frame: bytes, n: int = 6) -> str:
    reading = parse_auto_enc_frame(frame)
    a14 = (reading.raw_counts.astype(np.int64) & 0x3FFF)[:n]
    angles_deg = a14 * 360.0 / ENCODER_COUNTS_PER_REV
    parity = reading.parity_ok[:n]
    return "  ".join(
        f"j{i}={angles_deg[i]:6.2f}°{'' if parity[i] else '!'}"
        for i in range(n)
    )


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("PORT")
    if not port:
        sys.exit("Usage: smoke_test_link.py <port>  (or set $PORT)")

    print(f"--- opening {port} @ 921600 ---")
    link = HandSerialLink(port=port, baudrate=921600)

    received: list[bytes] = []
    link.connect()
    try:
        link.register_frame_handler(PROTOCOL_BYTE_AUTO_ENC, received.append)

        print(f"--- streaming for {STREAM_DURATION_S}s ---")
        time.sleep(STREAM_DURATION_S)

        rate = len(received) / STREAM_DURATION_S
        stats = link.get_link_stats()
        print(f"frames received: {len(received)} ({rate:.1f} Hz)")
        print(f"stats: bad_lrc={stats.frames_bad_lrc[PROTOCOL_BYTE_AUTO_ENC]}  "
              f"resyncs={stats.bad_header_resyncs}  "
              f"dropped_no_handler={stats.frames_dropped_no_handler[PROTOCOL_BYTE_AUTO_ENC]}")
        if received:
            print(f"latest frame joints: {_decode_first_joints(received[-1])}")
            print(f"  (! marks parity-fail — expected for unconnected channels)")

        print("--- reading firmware stats register ---")
        request = build_read_request(REG_STATS_ADDR, REG_STATS_LEN)
        data = parse_read_response(link.send_register_request(request))
        (rx_ok, rx_bad_lrc, rx_resyncs, tx_dropped,
         tx_aa55, tx_aaa9, tx_aaa9_freshened, _reset) = struct.unpack("<8I", data)
        print(f"firmware tx_aa55_responses = {tx_aa55} (host requests answered)")
        print(f"firmware tx_aaa9_auto      = {tx_aaa9} (host saw {len(received)})")
        print(f"firmware rx_frames_bad_lrc = {rx_bad_lrc}, rx_frame_resyncs = {rx_resyncs}")

        ok = True
        if rate < EXPECTED_FRAME_RATE_HZ * PASS_RATE_THRESHOLD:
            print(f"FAIL: frame rate {rate:.1f} Hz < "
                  f"{EXPECTED_FRAME_RATE_HZ * PASS_RATE_THRESHOLD:.0f} Hz")
            ok = False
        if stats.frames_bad_lrc[PROTOCOL_BYTE_AUTO_ENC]:
            print(f"FAIL: {stats.frames_bad_lrc[PROTOCOL_BYTE_AUTO_ENC]} bad-LRC encoder frames")
            ok = False
        print("PASS" if ok else "FAIL")
        return 0 if ok else 1
    finally:
        link.disconnect()


if __name__ == "__main__":
    sys.exit(main())
