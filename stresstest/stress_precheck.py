"""Read-only connectivity pre-check for the ORCA comms stress test.

No torque, no motion. Confirms three data paths independently before we
calibrate / stress:
  1. Motor bus  (ACM0): reads positions + temperatures of all 17 motors.
  2. Encoder stream (shared ACM1): AA A9 frames, per-joint raw counts + how
     many slots are actually moving when you wiggle the hand by hand.
  3. Tactile stream (shared ACM1): AA 56 frames (resultant + taxels), frame
     counters and active-sensor enumeration.

Usage:
    uv run python stresstest/stress_precheck.py \
        orca_core/models/v2/orcahand_stresstest/config.yaml
"""

import argparse
import time

import numpy as np

from orca_core import load_hand
from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.joint_encoder_client import (
    EncodersNotAvailableError,
    JointEncoderClient,
)
from orca_core.hardware.tactile_client import TactileClient
from orca_core.hardware.sensing.serial_discovery import resolve_sensing_ports
from orca_core.hardware.sensing.constants import (
    JOINT_TO_ENCODER_SLOT,
    ENCODER_SLOT_TO_JOINT,
)

VALID_JOINTS = [
    "thumb_abd", "thumb_mcp",
    "index_abd", "index_mcp", "index_pip",
    "middle_abd", "middle_mcp", "middle_pip",
    "ring_mcp", "pinky_pip",
]


def check_motors(config_path):
    print("\n=== 1. MOTOR BUS ===")
    hand = load_hand(config_path=config_path, engage_feedback=False)
    ok, msg = hand.connect()
    print("connect:", ok, msg)
    if not ok:
        return False
    try:
        pos = hand.get_motor_pos(as_dict=True)
        temp = hand.get_motor_temp(as_dict=True)
        print(f"motors read: {len(pos)} positions, {len(temp)} temps")
        hot = {m: t for m, t in temp.items() if t >= 50}
        print(f"temps: min={min(temp.values()):.0f} max={max(temp.values()):.0f}"
              f"  hot(>=50C)={hot}")
        return True
    finally:
        hand.disconnect()


def check_encoders(sample_secs=6.0):
    print("\n=== 2. ENCODER STREAM (wiggle joints now to see them move) ===")
    ports = resolve_sensing_ports(tactile_override="disabled", encoder_override="auto")
    if ports.encoder is None:
        print("FAIL: no encoder port resolved")
        return False
    link = HandSerialLink(ports.encoder, baudrate=2_000_000)
    link.connect()
    client = JointEncoderClient(link)
    client.connect()
    try:
        client.start_stream(timeout=3.0)
    except EncodersNotAvailableError as e:
        print("FAIL: encoder stream did not start:", e)
        client.disconnect(); link.disconnect()
        return False

    print(f"encoder stream up on {ports.encoder}")
    counts_hist = {slot: [] for slot in ENCODER_SLOT_TO_JOINT}
    t0 = time.monotonic()
    n = 0
    last = None
    while time.monotonic() - t0 < sample_secs:
        r = client.get_latest()
        if r is not None and (last is None or r.timestamp != last):
            last = r.timestamp
            n += 1
            for slot in counts_hist:
                if slot < len(r.raw_counts):
                    counts_hist[slot].append(int(r.raw_counts[slot]) & 0x3FFF)
        time.sleep(0.002)

    print(f"frames seen: {n} in {sample_secs:.0f}s (~{n/sample_secs:.0f} fps)")
    print(f"{'joint':<12}{'slot':>5}{'span(counts)':>14}{'valid-read?':>14}")
    for slot in sorted(ENCODER_SLOT_TO_JOINT):
        joint = ENCODER_SLOT_TO_JOINT[slot]
        vals = counts_hist[slot]
        if not vals:
            span = -1
        else:
            span = max(vals) - min(vals)
        tag = "VALID" if joint in VALID_JOINTS else "(noise/na)"
        print(f"{joint:<12}{slot:>5}{span:>14}{tag:>14}")
    client.stop_stream(); client.disconnect(); link.disconnect()
    return n > 0


def check_tactile(sample_secs=4.0):
    print("\n=== 3. TACTILE STREAM (resultant + taxels) ===")
    ports = resolve_sensing_ports(tactile_override="auto", encoder_override="disabled")
    if ports.tactile is None:
        print("FAIL: no tactile port resolved")
        return False
    baud = ports.tactile_baudrate or 2_000_000
    link = HandSerialLink(ports.tactile, baudrate=baud)
    link.connect()
    tc = TactileClient(link)
    tc.connect()
    try:
        tc.start_stream(resultant=True, taxels=True, min_sensors=1)
    except Exception as e:
        print("FAIL: tactile stream did not start:", repr(e))
        tc.disconnect(); link.disconnect()
        return False

    cfg = tc.get_tactile_configuration()
    time.sleep(sample_secs)
    stats = tc.get_stats()
    latest = tc.get_latest()
    print(f"tactile stream up on {ports.tactile} @ {baud}")
    print(f"config: {cfg}")
    print(f"stats: frames_ok={stats.frames_ok} bad_size={stats.frames_bad_payload_size}"
          f" bad={stats.frames_bad_payload} last_err={stats.last_error_code}")
    if latest is not None and latest.forces is not None:
        print("resultant fingers:", latest.forces.fingers)
    tc.stop_stream(); tc.disconnect(); link.disconnect()
    return stats.frames_ok > 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config_path")
    args = ap.parse_args()
    r1 = check_motors(args.config_path)
    r2 = check_encoders()
    r3 = check_tactile()
    print("\n=== SUMMARY ===")
    print(f"motors: {'OK' if r1 else 'FAIL'} | encoders: {'OK' if r2 else 'FAIL'}"
          f" | tactile: {'OK' if r3 else 'FAIL'}")


if __name__ == "__main__":
    main()
