"""Hardware bring-up tool for the joint-encoder AA A9 stream.

Opens a HandSerialLink on the given port, starts the encoder stream, and
once per second prints the latest frame's raw counts, per-slot parity /
angle-error flags, frame-level err byte, and link-side counters.

Useful for confirming wire-level health and inspecting what the firmware
emits for unplugged encoder slots — the per-slot flags and raw values on
disconnected slots are what later layers will use for auto-detect.

Usage:
    uv run python scripts/verify_encoder_stream.py /dev/cu.usbmodem103
    uv run python scripts/verify_encoder_stream.py /dev/cu.usbmodem103 --duration 60
"""
from __future__ import annotations

import argparse
import logging
import sys
import time

import numpy as np

from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.joint_encoder_client import (
    EncodersNotAvailableError,
    JointEncoderClient,
)
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_ANGLE_MASK,
    AUTO_ENC_NUM_JOINTS,
    ENCODER_SLOT_TO_JOINT,
    EXPECTED_ENCODER_SLOTS,
    LINK_DEFAULT_BAUDRATE,
    PROTOCOL_BYTE_AUTO_ENC,
)


# Slots whose 14-bit angle stays within EDGE_LSB of either rail for more than
# EDGE_FRACTION of the run are flagged as "stuck bus / no chip". Catches both
# all-0x0000, all-0x3FFF, and the alternating bus-floating case the hardware
# bring-up showed on unplugged chains.
EDGE_LSB = 64
EDGE_FRACTION = 0.5


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Verify the joint-encoder AA A9 stream.")
    p.add_argument("port", help="Serial port (e.g. /dev/cu.usbmodem103)")
    p.add_argument("--baudrate", type=int, default=LINK_DEFAULT_BAUDRATE)
    p.add_argument("--duration", type=float, default=30.0,
                   help="Seconds to stream before exiting (default: 30)")
    return p.parse_args()


def print_failure_diagnostics(client: JointEncoderClient, link: HandSerialLink) -> None:
    """Surface link + client counters and suggest the likely cause."""
    link_stats = link.get_link_stats()
    client_stats = client.get_encoder_stats()

    print("Link diagnostics:")
    print(f"  bad_header_resyncs           = {link_stats.bad_header_resyncs}")
    print(f"  responses_received (AA 55)   = {link_stats.responses_received}")
    print(f"  frames_routed                = {dict(link_stats.frames_routed)}")
    print(f"  frames_dropped_no_handler    = {dict(link_stats.frames_dropped_no_handler)}")
    print(f"  frames_bad_lrc               = {dict(link_stats.frames_bad_lrc)}")
    print(f"  handler_errors               = {dict(link_stats.handler_errors)}")
    print(f"Encoder client diagnostics:")
    print(f"  frames_ok                    = {client_stats.frames_ok}")
    print(f"  frames_bad_eff_len           = {client_stats.frames_bad_eff_len}")
    print()
    print("Likely causes:")
    if client_stats.frames_bad_eff_len > 0:
        print("  → Firmware/host wire-format mismatch. Most likely the firmware")
        print("    has not been re-flashed with COMMBOARD_NUM_JOINTS=17 yet, so")
        print("    it emits eff_len=33 but the host expects 35.")
    if link_stats.frames_dropped_no_handler:
        unhandled = ", ".join(f"0x{b:02X}" for b in link_stats.frames_dropped_no_handler)
        print(f"  → Frames arriving on un-routed second-byte(s): {unhandled}.")
        print("    If 0x57 appears, the firmware still uses the old encoder header")
        print("    byte and has not been re-flashed since the 0x57→0xA9 change.")
    if (link_stats.bad_header_resyncs > 0
            and not link_stats.frames_routed
            and not link_stats.frames_dropped_no_handler):
        print("  → Bytes are arriving but never align to an AA-XX header.")
        print("    Wrong baudrate, or you are reading a non-protocol stream")
        print("    (e.g. the motor CDC instead of the sensor CDC).")
    if (link_stats.bad_header_resyncs == 0
            and not link_stats.frames_routed
            and not link_stats.frames_dropped_no_handler
            and link_stats.responses_received == 0):
        print("  → Zero bytes received in 2s. OH bridge off, cable unplugged,")
        print("    or wrong serial port — try `ls /dev/cu.usbmodem*` to confirm.")


class SlotHealth:
    """Tracks per-slot extreme-value fraction over the run."""

    def __init__(self) -> None:
        self.total = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint64)
        self.extreme = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint64)

    def update(self, raw_counts: np.ndarray) -> None:
        angle = raw_counts.astype(np.uint16) & AUTO_ENC_ANGLE_MASK
        is_extreme = (angle <= EDGE_LSB) | (angle >= AUTO_ENC_ANGLE_MASK - EDGE_LSB)
        self.total += 1
        self.extreme += is_extreme.astype(np.uint64)

    def print_summary(self) -> None:
        if self.total[0] == 0:
            print("Slot health: no frames captured.")
            return
        print()
        print("Slot health (extreme-value fraction over run):")
        print("  slot  joint           extreme%   verdict")
        for slot in range(AUTO_ENC_NUM_JOINTS):
            joint = ENCODER_SLOT_TO_JOINT.get(slot, "?")
            frac = self.extreme[slot] / self.total[slot] if self.total[slot] else 0.0
            expected = slot in EXPECTED_ENCODER_SLOTS
            stuck = frac > EDGE_FRACTION
            if not expected:
                verdict = "reserved (not yet wired)"
            elif stuck:
                verdict = "DISCONNECTED (stuck/floating bus)"
            else:
                verdict = "ok"
            print(f"  {slot:>4}  {joint:<14}  {frac * 100:>6.1f}%   {verdict}")


def print_snapshot(client: JointEncoderClient, link: HandSerialLink,
                   t_elapsed: float, last_frames_ok: int,
                   health: SlotHealth) -> int:
    stats = client.get_encoder_stats()
    link_stats = link.get_link_stats()
    reading = client.get_latest_encoder_reading()

    rate = stats.frames_ok - last_frames_ok
    bad_lrc = link_stats.frames_bad_lrc[PROTOCOL_BYTE_AUTO_ENC]
    handler_errs = link_stats.handler_errors[PROTOCOL_BYTE_AUTO_ENC]

    header = (
        f"[t={t_elapsed:5.1f}s] rate={rate:4d} Hz  "
        f"frames_ok={stats.frames_ok}  "
        f"bad_lrc={bad_lrc}  bad_eff_len={stats.frames_bad_eff_len}  "
        f"handler_errs={handler_errs}  "
        f"err_byte=0x{stats.last_err_byte:02X}  "
        f"freshness={stats.last_freshness_ms:5.1f}ms"
    )
    print(header)

    if reading is None:
        print("  (no frame yet)")
        return stats.frames_ok

    health.update(reading.raw_counts)

    raw_row = "  raw   :" + "".join(f" {int(c):04X}" for c in reading.raw_counts)
    par_row = "  parity:" + "".join(
        "   OK" if ok else "  BAD" for ok in reading.parity_ok
    )
    ang_row = "  angle :" + "".join(
        "   OK" if not err else "  ERR" for err in reading.angle_error
    )
    print(raw_row)
    print(par_row)
    print(ang_row)
    return stats.frames_ok


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")

    print(f"Opening {args.port} @ {args.baudrate} baud (expecting {AUTO_ENC_NUM_JOINTS} joints).")
    link = HandSerialLink(args.port, baudrate=args.baudrate)
    link.connect()

    client = JointEncoderClient(link)
    client.connect()

    try:
        client.start_encoder_stream(timeout=2.0)
    except EncodersNotAvailableError as e:
        print(f"FAIL: no encoder frames within timeout ({e})")
        print()
        print_failure_diagnostics(client, link)
        client.disconnect()
        link.disconnect()
        return 1

    print(f"Stream active. Sampling for {args.duration:.0f}s (Ctrl-C to stop early).")
    print()
    health = SlotHealth()
    started = time.monotonic()
    deadline = started + args.duration
    last_frames_ok = 0
    try:
        while time.monotonic() < deadline:
            time.sleep(1.0)
            t_elapsed = time.monotonic() - started
            last_frames_ok = print_snapshot(client, link, t_elapsed, last_frames_ok, health)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        client.disconnect()
        link.disconnect()

    final_stats = client.get_encoder_stats()
    final_link = link.get_link_stats()
    bad_lrc = final_link.frames_bad_lrc[PROTOCOL_BYTE_AUTO_ENC]
    print()
    print(
        f"Summary: frames_ok={final_stats.frames_ok}  "
        f"bad_lrc={bad_lrc}  bad_eff_len={final_stats.frames_bad_eff_len}  "
        f"handler_errs={final_link.handler_errors[PROTOCOL_BYTE_AUTO_ENC]}"
    )
    health.print_summary()
    if bad_lrc > 0 or final_stats.frames_bad_eff_len > 0:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
