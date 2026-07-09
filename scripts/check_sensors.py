#!/usr/bin/env python
"""Sensor health check for an assembled ORCA hand.

Reads the hand's config via :func:`orca_core.load_hand` and runs the checks
that its declared hardware supports:

  * joint encoders  — stream comes up, frames are clean, every configured slot
    reports a live, non-stuck, parity-clean angle. Non-interactive.
  * tactile sensors — all 5 sensors enumerate, stream at ~1 kHz, respond to
    finger presses, and zero correctly. Interactive: you will be asked to press
    each fingertip.

A hand with both runs both, encoders first. Motors do not need to be powered or
connected — only the sensing links.

``--port`` skips the config entirely and runs the encoder checks against a raw
serial port, for bringing up a connector board before a hand config exists.

For a live visualization of the data rather than a pass/fail verdict, use
``scripts/monitor_sensors.py``. For force arrows and a taxel heatmap, see
orca_ui: https://github.com/orcahand/orca_ui

Usage:
    uv run python scripts/check_sensors.py orca_core/models/v2/orcahand-touch
    uv run python scripts/check_sensors.py CONFIG --encoder-duration 20
    uv run python scripts/check_sensors.py --port /dev/cu.usbmodem103
"""
from __future__ import annotations

import argparse
import logging
import sys
import time

import numpy as np

from orca_core import OrcaHandTouch, load_hand
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
    JOINT_TO_ENCODER_SLOT,
    LINK_DEFAULT_BAUDRATE,
    PROTOCOL_BYTE_AUTO_ENC,
)
from orca_core.hardware.sensing.serial_discovery import resolve_sensing_ports

# Thresholds and taxel layouts are intentionally script-local — they only
# drive pass/fail decisions and the ASCII renderer here, not the runtime API.

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]

PRESS_THRESHOLD_N = 1.0
ZERO_TOLERANCE_N = 0.5
TAXEL_ACTIVE_N = 0.1  # |fz| above this counts as "pressed" in the ASCII grid

ENCODER_DURATION_S = 10.0
ENCODER_MIN_RATE_HZ = 50.0

# A slot whose 14-bit angle sits within EDGE_LSB of either rail for more than
# EDGE_FRACTION of the run reads as a stuck/floating bus rather than a chip.
# Catches all-0x0000, all-0x3FFF, and the alternating floating case.
EDGE_LSB = 64
EDGE_FRACTION = 0.5

# (rows, cols, positions) per (role, taxel_count). role ∈ {"thumb","finger","pinky"}.
# positions[i] = (row, col) of taxel i in the auto-stream sequence; row=0 is the
# fingertip. Multiple taxels may share a cell — the renderer ORs hot states.
TAXEL_LAYOUTS: dict[tuple[str, int], tuple[int, int, tuple[tuple[int, int], ...]]] = {
    ("finger", 87): (18, 19, (
        (1,13), (3,16), (3,16), (1,13), (3,15), (3,9),
        (3,12), (1,9), (0,9), (17,17), (17,14), (17,9),
        (15,9), (14,9), (12,9), (10,9), (8,9), (7,9),
        (5,9), (5,16), (7,16), (8,15), (10,15), (12,15),
        (14,15), (15,16), (10,12), (14,12), (15,13), (12,12),
        (7,13), (8,12), (5,13), (4,17), (6,17), (8,17),
        (10,17), (11,17), (13,17), (15,18), (17,18), (17,18),
        (10,17), (14,17), (15,18), (12,17), (6,17), (8,17),
        (5,17), (1,5), (3,2), (3,2), (1,5), (3,3),
        (3,6), (17,1), (17,4), (5,2), (7,2), (8,3),
        (10,3), (12,3), (14,3), (15,2), (10,6), (14,6),
        (15,5), (12,6), (7,5), (8,6), (5,5), (4,1),
        (6,1), (8,1), (10,1), (11,1), (13,1), (15,0),
        (17,0), (17,0), (10,1), (14,1), (15,0), (12,1),
        (6,1), (8,1), (5,1),
    )),
    ("thumb", 51): (11, 21, (
        (10,0), (10,1), (8,0), (8,1), (6,0), (6,1),
        (4,1), (10,3), (8,3), (6,3), (4,2), (4,3),
        (2,3), (2,3), (10,6), (8,6), (6,6), (3,4),
        (4,6), (0,6), (1,7), (2,7), (9,10), (8,10),
        (6,10), (4,10), (2,10), (1,10), (0,10), (2,13),
        (10,14), (6,14), (4,14), (1,13), (8,14), (0,14),
        (3,16), (10,17), (8,17), (6,17), (4,17), (2,17),
        (10,19), (10,20), (8,19), (8,20), (6,19), (6,20),
        (4,19), (4,18), (2,17),
    )),
    ("pinky", 51): (11, 17, (
        (10,0), (10,0), (8,0), (8,0), (6,0), (6,0),
        (4,1), (10,1), (8,1), (6,1), (5,1), (5,2),
        (2,2), (3,2), (10,5), (8,5), (6,4), (3,2),
        (4,4), (1,4), (1,4), (2,5), (10,8), (8,8),
        (6,8), (4,8), (2,8), (1,8), (0,8), (2,11),
        (10,11), (6,12), (4,12), (1,12), (8,11), (1,12),
        (3,14), (10,15), (8,15), (6,15), (5,14), (3,14),
        (10,16), (10,16), (8,16), (8,16), (6,16), (6,16),
        (4,15), (5,15), (2,14),
    )),
}

FINGER_TO_ROLE = {"thumb": "thumb", "index": "finger", "middle": "finger",
                  "ring": "finger", "pinky": "pinky"}


def render_taxel_grid(role: str, num_taxels: int, taxels, threshold_n: float = TAXEL_ACTIVE_N) -> str | None:
    """Return multi-line ASCII art of taxel state (X = active, O = inactive,
    space = no taxel at that cell), or None if no layout exists for this
    (role, taxel_count) combination."""
    layout = TAXEL_LAYOUTS.get((role, num_taxels))
    if layout is None:
        return None
    rows, cols, positions = layout
    grid = [[" "] * cols for _ in range(rows)]
    for idx, (r, c) in enumerate(positions):
        active = abs(taxels[idx][2]) > threshold_n
        if active:
            grid[r][c] = "X"
        elif grid[r][c] == " ":
            grid[r][c] = "O"
    return "\n".join("    " + " ".join(row) for row in grid)


def banner(name):
    print(f"\n===== {name} =====")


def pause(msg):
    input(f">>> {msg} (press Enter) ")


def wait_for_frame(getter, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if getter() is not None:
            return
        time.sleep(0.005)
    raise TimeoutError(f"No auto-stream frame within {timeout}s")


def _redraw_in_place(prev_lines: int, lines: list[str]) -> int:
    """Move the cursor up `prev_lines`, clear each line, write the new ones,
    and flush. Returns the number of lines just written. Uses ANSI escape
    sequences (works on macOS / Linux terminals; Windows cmd not supported)."""
    if prev_lines:
        sys.stdout.write(f"\033[{prev_lines}F")  # cursor up N lines, to col 0
    for ln in lines:
        sys.stdout.write("\033[2K" + ln + "\n")  # clear line, write, advance
    sys.stdout.flush()
    return len(lines)


def live_press_resultant(hand, target, fingers, duration_s=1.5, fps=20):
    """Live in-place display of resultant |fz| during press of `target`.
    Polls at ~200 Hz for accurate peak tracking; redraws at `fps`.
    Returns dict {finger: peak |fz|} across the full window."""
    peaks = {f: 0.0 for f in fingers}
    end = time.time() + duration_s
    interval = 1.0 / fps
    next_render = time.time()
    prev_lines = 0
    while time.time() < end:
        reading = hand.get_tactile_forces()
        if reading is not None:
            for f in fingers:
                if f in reading:
                    v = abs(reading[f][2])
                    if v > peaks[f]:
                        peaks[f] = v
        if time.time() >= next_render:
            cur = abs(reading[target][2]) if reading and target in reading else 0.0
            line = f"  {target}: |fz| now={cur:5.2f} N  peak={peaks[target]:5.2f} N"
            prev_lines = _redraw_in_place(prev_lines, [line])
            next_render = time.time() + interval
        time.sleep(0.005)
    return peaks


def live_press_taxels(hand, target, role, n_taxels, fingers, duration_s=1.5, fps=12):
    """Live in-place display of taxel ASCII grid + peak during press of
    `target`. Polls at ~200 Hz, redraws at `fps`. Returns dict
    {finger: peak |fz| at hottest taxel} across the full window."""
    peaks = {f: 0.0 for f in fingers}
    end = time.time() + duration_s
    interval = 1.0 / fps
    next_render = time.time()
    prev_lines = 0
    no_layout = (f"    (no ASCII layout for {role}-{n_taxels}; "
                 f"standard models are thumb-51, finger-87, pinky-51)")
    while time.time() < end:
        reading = hand.get_tactile_taxels()
        if reading is not None:
            for f in fingers:
                if f in reading:
                    v = max((abs(t[2]) for t in reading[f]), default=0.0)
                    if v > peaks[f]:
                        peaks[f] = v
        if time.time() >= next_render:
            if reading is not None and target in reading:
                cur_max = max((abs(t[2]) for t in reading[target]), default=0.0)
                grid = render_taxel_grid(role, n_taxels, reading[target])
            else:
                cur_max = 0.0
                grid = None
            header = (f"  {target}: |fz| now={cur_max:5.2f} N  peak={peaks[target]:5.2f} N"
                      f"  ({n_taxels} taxels, X = >{TAXEL_ACTIVE_N:.1f}N)")
            lines = [header]
            if grid is None:
                lines.append(no_layout)
            else:
                lines.extend(grid.split("\n"))
            prev_lines = _redraw_in_place(prev_lines, lines)
            next_render = time.time() + interval
        time.sleep(0.005)
    return peaks


def detect_wiring_mismatch(target_finger, peaks, wiring, threshold=PRESS_THRESHOLD_N):
    """If user pressed `target_finger` but the largest signal showed up
    under a different finger name, return a helpful suggestion string.
    Otherwise return None."""
    target_peak = peaks.get(target_finger, 0.0)
    if target_peak >= threshold:
        return None
    others = {f: p for f, p in peaks.items() if f != target_finger and p >= threshold}
    if not others:
        return None
    other_finger = max(others, key=others.get)
    target_slot = wiring.get(target_finger)
    other_slot = wiring.get(other_finger)
    return (
        f"  WIRING MISMATCH: pressed {target_finger.upper()} but force showed under "
        f"{other_finger.upper()} ({others[other_finger]:.2f} N vs {target_peak:.2f} N on {target_finger}).\n"
        f"    → Likely fix: in config.yaml's finger_to_sensor_id (or "
        f"hardware/sensing/constants.py), set '{target_finger}' to {other_slot} "
        f"(currently {target_slot}). You'll then need to reassign '{other_finger}' too."
    )


def phase_enumerate(hand):
    banner("tactile: connect & enumerate")
    cfg = hand.get_tactile_configuration()
    print(f"  {cfg}")
    print("  Per-finger status (canonical order):")
    for f in FINGERS:
        connected = cfg.connected.get(f, False)
        n_taxels = cfg.num_taxels.get(f, 0)
        slot = hand.config.finger_to_sensor_id.get(f)
        print(f"    {f:7s}  connected={connected!s:5s}  taxels={n_taxels:3d}  slot={slot}")

    missing = [f for f in FINGERS if not cfg.connected.get(f, False)]
    if missing:
        return False, f"missing sensors: {missing} (need all 5)"
    return True, "all 5 sensors connected"


def prep_zero_baseline(hand):
    """Capture and apply per-taxel zero offsets so the press phases display
    zero-relative readings. Setup, not a scored phase — the zeroing phase
    still independently re-tests the zeroing workflow."""
    print("\n----- PREP: zero baseline (applied to subsequent phases) -----")
    hand.start_tactile_stream(resultant=True, taxels=True, min_sensors=1)
    try:
        wait_for_frame(hand.get_tactile_taxels)
        pause("ensure NOTHING is touching any sensor")
        offsets = hand.zero_tactile_sensors(num_samples=200)
        max_baseline = 0.0
        for f in FINGERS:
            if f in offsets and offsets[f]:
                max_baseline = max(max_baseline, max(abs(t[2]) for t in offsets[f]))
        print(f"  Zero captured (max raw baseline |fz| was {max_baseline:.2f} N at hottest taxel)")
    finally:
        hand.stop_tactile_stream()


def phase_resultant_press(hand):
    banner("tactile: resultant stream + finger press")
    hand.start_tactile_stream(resultant=True, taxels=False, min_sensors=1)
    try:
        wait_for_frame(hand.get_tactile_forces)
        time.sleep(0.5)
        s0 = hand.get_tactile_stats()
        time.sleep(5.0)
        s1 = hand.get_tactile_stats()
        rate = (s1.frames_ok - s0.frames_ok) / 5.0
        print(f"  Frame rate: {rate:.0f} fps")
        print(f"  Stats: ok={s1.frames_ok} bad_lrc={s1.frames_bad_checksum} "
              f"parse_err={s1.parse_errors} resyncs={s1.resyncs}")

        if rate < 50:
            return False, f"frame rate {rate:.0f} fps < 50 (stream stalled?)"
        if s1.frames_bad_checksum or s1.parse_errors or s1.resyncs:
            return False, "non-zero error counters during idle stream"

        wiring = hand.config.finger_to_sensor_id
        peaks_per_press = {}
        warnings = []
        for f in FINGERS:
            pause(f"press {f.upper()} (vary pressure to watch the live readout)")
            all_peaks = live_press_resultant(hand, f, FINGERS, duration_s=1.5)
            peaks_per_press[f] = all_peaks[f]
            mismatch = detect_wiring_mismatch(f, all_peaks, wiring)
            if mismatch:
                print(mismatch)
                warnings.append(f)

        weak = [f for f, p in peaks_per_press.items() if p < PRESS_THRESHOLD_N]
        if warnings:
            return False, f"wiring mismatch suspected on: {warnings} (see suggestions above)"
        if weak:
            return False, f"no/weak response on: {weak}"
        return True, f"~{rate:.0f} fps clean, all fingers responded"
    finally:
        hand.stop_tactile_stream()


def phase_taxels_press(hand):
    banner("tactile: taxel stream + finger press")
    hand.start_tactile_stream(resultant=False, taxels=True, min_sensors=1)
    try:
        wait_for_frame(hand.get_tactile_taxels)
        reading = hand.get_tactile_taxels()
        for finger in FINGERS:
            if finger not in reading:
                return False, f"{finger} missing from taxel frame"
            num_expected_taxels = hand.get_tactile_configuration().num_taxels[finger]
            if len(reading[finger]) != num_expected_taxels:
                return False, (f"{finger} taxel array length {len(reading[finger])} "
                               f"!= reported {num_expected_taxels}")
            if any(len(t) != 3 for t in reading[finger]):
                return False, f"{finger} has malformed taxel vectors"
        print(f"  Taxel array shapes verified for all 5 fingers")

        wiring = hand.config.finger_to_sensor_id
        peaks_per_press = {}
        warnings = []
        for finger in FINGERS:
            pause(f"press {finger.upper()} (move your finger around to light up different taxels)")
            num_expected_taxels = hand.get_tactile_configuration().num_taxels[finger]
            all_peaks = live_press_taxels(hand, finger, FINGER_TO_ROLE[finger], num_expected_taxels, FINGERS, duration_s=2.5)
            peaks_per_press[finger] = all_peaks[finger]
            mismatch = detect_wiring_mismatch(finger, all_peaks, wiring)
            if mismatch:
                print(mismatch)
                warnings.append(finger)

        weak = [f for f, p in peaks_per_press.items() if p < PRESS_THRESHOLD_N]
        if warnings:
            return False, f"wiring mismatch suspected on: {warnings} (see suggestions above)"
        if weak:
            return False, f"no/weak taxel response on: {weak}"
        return True, "all fingers show per-taxel response"
    finally:
        hand.stop_tactile_stream()


def phase_combined(hand):
    banner("tactile: combined mode (resultant + taxels)")
    hand.start_tactile_stream(resultant=True, taxels=True, min_sensors=1)
    try:
        wait_for_frame(hand.get_tactile_forces)
        forces = hand.get_tactile_forces()
        taxels = hand.get_tactile_taxels()
        if forces is None or taxels is None:
            return False, f"combined snapshot missing: forces={forces is not None} taxels={taxels is not None}"

        s0 = hand.get_tactile_stats()
        time.sleep(3.0)
        s1 = hand.get_tactile_stats()
        rate = (s1.frames_ok - s0.frames_ok) / 3.0
        print(f"  Frame rate (combined): {rate:.0f} fps")
        if rate < 50:
            return False, f"combined frame rate {rate:.0f} fps < 50 (stream stalled?)"
        return True, f"both data types in single snapshot, ~{rate:.0f} fps"
    finally:
        hand.stop_tactile_stream()


def phase_zeroing(hand):
    banner("tactile: zeroing")
    pause("ensure NOTHING is touching any sensor")
    hand.start_tactile_stream(resultant=True, taxels=True, min_sensors=1)
    try:
        wait_for_frame(hand.get_tactile_taxels)
        offsets = hand.zero_tactile_sensors(num_samples=200)
        print("  Captured offsets per finger (avg |fz| per taxel):")
        for f in FINGERS:
            if f in offsets and offsets[f]:
                fz_vals = [abs(t[2]) for t in offsets[f]]
                avg = sum(fz_vals) / len(fz_vals)
                print(f"    {f:7s}  avg={avg:.3f} N  ({len(fz_vals)} taxels)")

        time.sleep(0.2)
        forces = hand.get_tactile_forces()
        max_resting = max(abs(forces[f][2]) for f in FINGERS) if forces else None
        print(f"  Max resting |fz| after zero: {max_resting:.3f} N")
        if max_resting is None or max_resting > ZERO_TOLERANCE_N:
            return False, f"resting fz not near zero (max={max_resting})"

        hand.clear_tactile_zero()
        time.sleep(0.2)
        forces = hand.get_tactile_forces()
        if forces is None:
            return False, "no frame after clear_tactile_zero"

        return True, f"zero captured, applied (max resting={max_resting:.2f}N), cleared"
    finally:
        hand.stop_tactile_stream()


def phase_lifecycle(hand):
    banner("tactile: lifecycle (stop -> restart in new mode)")

    print("  [1/4] Start resultant-only stream...")
    hand.start_tactile_stream(resultant=True, taxels=False, min_sensors=1)
    wait_for_frame(hand.get_tactile_forces)
    print("        OK - forces frame received")

    print("  [2/4] Stop stream and verify cache cleared...")
    hand.stop_tactile_stream()
    if hand.get_tactile_forces() is not None:
        return False, "stale forces after stop_tactile_stream"
    print("        OK - cache cleared")

    print("  [3/4] Restart in taxels-only mode...")
    hand.start_tactile_stream(resultant=False, taxels=True, min_sensors=1)
    try:
        wait_for_frame(hand.get_tactile_taxels)
        if hand.get_tactile_taxels() is None:
            return False, "no taxels after restart in taxels-only mode"
        print("        OK - taxels frame received")

        print("  [4/4] Verify mode isolation (no resultant cache)...")
        if hand.get_tactile_forces() is not None:
            return False, "resultant cache populated in taxels-only mode"
        print("        OK - resultant cache empty")

        return True, "stop -> restart in different mode works"
    finally:
        hand.stop_tactile_stream()


# ---------------------------------------------------------------------------
# Joint encoders
# ---------------------------------------------------------------------------


class SlotHealth:
    """Per-slot accumulators over an encoder run: how often the angle sat at a
    rail, and how many frames reported a parity or chip angle-error flag."""

    def __init__(self) -> None:
        self.frames = 0
        self.extreme = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint64)
        self.parity_bad = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint64)
        self.angle_err = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint64)
        self.angle_min = np.full(AUTO_ENC_NUM_JOINTS, AUTO_ENC_ANGLE_MASK, dtype=np.int64)
        self.angle_max = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.int64)

    def update(self, reading) -> None:
        angle = reading.raw_counts.astype(np.uint16) & AUTO_ENC_ANGLE_MASK
        self.frames += 1
        self.extreme += (
            (angle <= EDGE_LSB) | (angle >= AUTO_ENC_ANGLE_MASK - EDGE_LSB)
        ).astype(np.uint64)
        self.parity_bad += (~np.asarray(reading.parity_ok, dtype=bool)).astype(np.uint64)
        self.angle_err += np.asarray(reading.angle_error, dtype=bool).astype(np.uint64)
        self.angle_min = np.minimum(self.angle_min, angle.astype(np.int64))
        self.angle_max = np.maximum(self.angle_max, angle.astype(np.int64))

    def verdict(self, slot: int) -> tuple[bool, str]:
        """Pass/fail plus a human-readable reason for one expected slot."""
        frac = self.extreme[slot] / self.frames
        if self.parity_bad[slot]:
            return False, f"{int(self.parity_bad[slot])} parity errors"
        if self.angle_err[slot]:
            return False, f"{int(self.angle_err[slot])} chip angle-error flags"
        if frac > EDGE_FRACTION:
            return False, f"stuck/floating bus ({frac * 100:.0f}% at a rail)"
        return True, "ok"

    def print_table(self, expected_slots) -> None:
        if not self.frames:
            print("  no frames captured.")
            return
        print(f"  Per-slot health over {self.frames} frames:")
        print("    slot  joint           span   rail%   parity  angErr  verdict")
        for slot in range(AUTO_ENC_NUM_JOINTS):
            joint = ENCODER_SLOT_TO_JOINT.get(slot, "?")
            frac = self.extreme[slot] / self.frames
            span = int(self.angle_max[slot] - self.angle_min[slot])
            if slot not in expected_slots:
                verdict = "not configured"
            else:
                _, verdict = self.verdict(slot)
            print(
                f"    {slot:>4}  {joint:<14} {span:>5}  {frac * 100:>5.1f}%  "
                f"{int(self.parity_bad[slot]):>6}  {int(self.angle_err[slot]):>6}  {verdict}"
            )


def print_link_diagnostics(client: JointEncoderClient, link: HandSerialLink) -> None:
    """Surface link + client counters and suggest the likely cause."""
    link_stats = link.get_link_stats()
    client_stats = client.get_stats()

    print("  Link diagnostics:")
    print(f"    bad_header_resyncs        = {link_stats.bad_header_resyncs}")
    print(f"    responses_received        = {link_stats.responses_received}")
    print(f"    frames_routed             = {dict(link_stats.frames_routed)}")
    print(f"    frames_dropped_no_handler = {dict(link_stats.frames_dropped_no_handler)}")
    print(f"    frames_bad_lrc            = {dict(link_stats.frames_bad_lrc)}")
    print(f"    handler_errors            = {dict(link_stats.handler_errors)}")
    print(f"    frames_ok                 = {client_stats.frames_ok}")
    print(f"    frames_bad_eff_length     = {client_stats.frames_bad_effective_length}")
    print("  Likely causes:")
    if client_stats.frames_bad_effective_length > 0:
        print("    → Firmware/host wire-format mismatch: the firmware emits a")
        print("      different joint count than the host expects. Re-flash it.")
    if link_stats.frames_dropped_no_handler:
        unhandled = ", ".join(f"0x{b:02X}" for b in link_stats.frames_dropped_no_handler)
        print(f"    → Frames arriving on un-routed second-byte(s): {unhandled}.")
        print("      Firmware may predate the current encoder header byte.")
    if (link_stats.bad_header_resyncs > 0
            and not link_stats.frames_routed
            and not link_stats.frames_dropped_no_handler):
        print("    → Bytes arrive but never align to an AA-XX header. Wrong")
        print("      baudrate, or you are reading the motor port by mistake.")
    if (link_stats.bad_header_resyncs == 0
            and not link_stats.frames_routed
            and not link_stats.frames_dropped_no_handler
            and link_stats.responses_received == 0):
        print("    → Zero bytes received. Board unpowered, cable unplugged, or")
        print("      wrong serial port — try `ls /dev/cu.usbmodem*`.")


def phase_encoder_stream(client, link, expected_slots, duration_s):
    """Stream for `duration_s`, then judge frame integrity and per-slot health.
    Non-interactive; the hand can sit still throughout."""
    banner("encoders: stream health")
    try:
        client.start_stream(timeout=2.0)
    except EncodersNotAvailableError as e:
        print(f"  no encoder frames within timeout ({e})")
        print_link_diagnostics(client, link)
        return False, "encoder stream never started"

    print(f"  Streaming for {duration_s:.0f}s (Ctrl-C to stop early)...")
    health = SlotHealth()
    started = time.monotonic()
    last_ts = None
    try:
        while time.monotonic() - started < duration_s:
            reading = client.get_latest()
            if reading is not None and reading.timestamp != last_ts:
                health.update(reading)
                last_ts = reading.timestamp
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\n  Interrupted; judging on what was captured.")

    elapsed = time.monotonic() - started
    stats = client.get_stats()
    link_stats = link.get_link_stats()
    bad_lrc = link_stats.frames_bad_lrc[PROTOCOL_BYTE_AUTO_ENC]
    handler_errs = link_stats.handler_errors[PROTOCOL_BYTE_AUTO_ENC]
    rate = stats.frames_ok / elapsed if elapsed else 0.0

    print(f"  Frame rate: {rate:.0f} Hz over {elapsed:.1f}s")
    print(f"  Stats: ok={stats.frames_ok} bad_lrc={bad_lrc} "
          f"bad_eff_length={stats.frames_bad_effective_length} "
          f"handler_errs={handler_errs} error_byte=0x{stats.last_error_byte:02X}")
    health.print_table(expected_slots)

    if not health.frames:
        print_link_diagnostics(client, link)
        return False, "no encoder frames decoded"
    if bad_lrc or stats.frames_bad_effective_length or handler_errs:
        return False, (f"corrupt frames: bad_lrc={bad_lrc} "
                       f"bad_eff_length={stats.frames_bad_effective_length} "
                       f"handler_errs={handler_errs}")
    if rate < ENCODER_MIN_RATE_HZ:
        return False, f"frame rate {rate:.0f} Hz < {ENCODER_MIN_RATE_HZ:.0f} (stream stalled?)"

    bad = {ENCODER_SLOT_TO_JOINT.get(s, str(s)): health.verdict(s)[1]
           for s in sorted(expected_slots) if not health.verdict(s)[0]}
    if bad:
        return False, f"unhealthy slots: {bad}"
    return True, f"~{rate:.0f} Hz clean, {len(expected_slots)} slot(s) healthy"


def run_encoder_checks(port, baudrate, expected_slots, duration_s):
    """Open a dedicated link, run the encoder phase, and always close it.
    The link is closed before any tactile phase runs so the two never contend
    for the same serial device when they share a port."""
    print(f"\nEncoder port: {port} @ {baudrate} baud")
    print(f"Expecting {len(expected_slots)} slot(s): "
          f"{sorted(ENCODER_SLOT_TO_JOINT.get(s, s) for s in expected_slots)}")
    link = HandSerialLink(port, baudrate=baudrate)
    link.connect()
    client = None
    try:
        client = JointEncoderClient(link)
        client.connect()
        return phase_encoder_stream(client, link, expected_slots, duration_s)
    finally:
        if client is not None:
            client.disconnect()
        link.disconnect()


def encoder_slots_for(hand) -> set[int]:
    """Slots the config actually declares, so an unwired slot isn't judged."""
    return {JOINT_TO_ENCODER_SLOT[j] for j in hand._encoder_backed_joints()}


# ---------------------------------------------------------------------------
# Tactile
# ---------------------------------------------------------------------------


def run_tactile_checks(hand) -> list[tuple[str, bool, str]]:
    print(f"\nSensor port: {hand.config.sensor_port}")
    print(f"Baudrate:    {hand.config.sensor_baudrate}")
    print(f"Wiring:      {hand.config.finger_to_sensor_id}")

    ok, msg = hand.connect_sensors_only()
    print(msg)
    if not ok:
        return [("tactile: connect", False, msg)]

    results: list[tuple[str, bool, str]] = []
    try:
        results.append(("tactile: enumerate", *run_phase(phase_enumerate, hand)))

        if results[0][1]:
            try:
                prep_zero_baseline(hand)
            except Exception as e:
                print(f"  WARNING: zero baseline prep failed ({type(e).__name__}: {e}); "
                      "subsequent phases will see raw readings")

        for label, fn in [
            ("tactile: resultant press", phase_resultant_press),
            ("tactile: taxel press", phase_taxels_press),
            ("tactile: combined mode", phase_combined),
            ("tactile: zeroing", phase_zeroing),
            ("tactile: lifecycle", phase_lifecycle),
        ]:
            results.append((label, *run_phase(fn, hand)))
    finally:
        hand.disconnect()
    return results


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------


def run_phase(fn, *fn_args) -> tuple[bool, str]:
    try:
        return fn(*fn_args)
    except Exception as e:
        return False, f"raised {type(e).__name__}: {e}"


def print_summary(results) -> bool:
    print("\n===== SUMMARY =====")
    for label, ok, detail in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {label}: {detail}")
    all_ok = all(ok for _, ok, _ in results)
    print("\nAll checks passed." if all_ok else "\nOne or more checks failed.")
    return all_ok


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument(
        "config_path", nargs="?", default=None,
        help="Path to the hand model directory or config.yaml.",
    )
    p.add_argument(
        "--port", default=None,
        help="Run encoder checks on this raw serial port and skip the config "
             "entirely (connector-board bring-up before a hand config exists).",
    )
    p.add_argument("--baudrate", type=int, default=LINK_DEFAULT_BAUDRATE,
                   help="Encoder link baudrate; only used with --port.")
    p.add_argument("--encoder-duration", type=float, default=ENCODER_DURATION_S,
                   help=f"Seconds to sample the encoder stream (default: {ENCODER_DURATION_S:.0f}).")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")

    if args.port:
        if args.config_path:
            raise SystemExit("Pass either config_path or --port, not both.")
        result = run_phase(
            run_encoder_checks,
            args.port, args.baudrate, set(EXPECTED_ENCODER_SLOTS), args.encoder_duration,
        )
        return 0 if print_summary([("encoders: stream health", *result)]) else 1

    hand = load_hand(config_path=args.config_path, engage_feedback=False)
    has_encoders = hand.config.has_joint_encoders
    has_tactile = isinstance(hand, OrcaHandTouch)
    if not (has_encoders or has_tactile):
        raise SystemExit(
            "This hand config declares no sensors: no joint_encoder_joints and "
            "no sensors block. Nothing to check."
        )

    results: list[tuple[str, bool, str]] = []

    # Encoders first: run_encoder_checks closes its link before tactile opens
    # one, which matters when both streams share a single serial port.
    if has_encoders:
        ports = resolve_sensing_ports(
            tactile_override="disabled",
            encoder_override=hand.config.encoder_serial_port,
        )
        if ports.encoder is None:
            results.append(("encoders: port", False,
                            f"no encoder port resolved "
                            f"(encoder_serial_port={hand.config.encoder_serial_port!r})"))
        else:
            results.append((
                "encoders: stream health",
                *run_phase(run_encoder_checks, ports.encoder,
                           hand.config.encoder_baudrate, encoder_slots_for(hand),
                           args.encoder_duration),
            ))

    if has_tactile:
        results.extend(run_tactile_checks(hand))

    return 0 if print_summary(results) else 1


if __name__ == "__main__":
    sys.exit(main())
