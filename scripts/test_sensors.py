#!/usr/bin/env python
"""Tactile sensor health check for an assembled OrcaHandTouch.

Walks through 6 interactive phases that verify all 5 tactile sensors
enumerate, stream at ~1 kHz, respond to finger presses, and zero correctly.
Run on a fully assembled hand with all 5 sensors plugged in. Motors do not
need to be powered or connected — only the sensor adapter.

For a complementary live visualization (force arrows, taxel heatmap), see
orca_ui: https://github.com/orcahand/orca_ui

Usage:
    uv run python scripts/test_sensors.py orca_core/models/v2/orcahand-touch
"""

import argparse
import sys
import threading
import time

from orca_core import OrcaHandTouch

# Thresholds and taxel layouts are intentionally script-local — they only
# drive pass/fail decisions and the ASCII renderer here, not the runtime API.

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]

PRESS_THRESHOLD_N = 1.0
ZERO_TOLERANCE_N = 0.5
TAXEL_ACTIVE_N = 0.1  # |fz| above this counts as "pressed" in the ASCII grid

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


def banner(n, name):
    print(f"\n===== PHASE {n}: {name} =====")


def pause(msg):
    input(f">>> {msg} (press Enter) ")


def _enter_event():
    """Return a threading.Event that is set when the user presses Enter."""
    event = threading.Event()
    threading.Thread(target=lambda: (input(), event.set()), daemon=True).start()
    return event


def wait_for_frame(getter, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if getter() is not None:
            return
        time.sleep(0.005)
    raise TimeoutError(f"No auto-stream frame within {timeout}s")


def measure_all_peaks(getter, fingers, duration_s=1.5, taxels=False):
    """Return dict {finger: peak |fz|} over the duration, for every finger
    in `fingers`. Lets us detect wiring mismatches (signal showing up under
    the wrong finger name)."""
    peaks = {f: 0.0 for f in fingers}
    end = time.time() + duration_s
    while time.time() < end:
        reading = getter()
        if reading is not None:
            for f in fingers:
                if f not in reading:
                    continue
                if taxels:
                    val = max((abs(t[2]) for t in reading[f]), default=0.0)
                else:
                    val = abs(reading[f][2])
                if val > peaks[f]:
                    peaks[f] = val
        time.sleep(0.002)
    return peaks


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


def live_press_resultant(hand, target, fingers, stop_event=None, duration_s=1.5, fps=20):
    """Live in-place display of resultant |fz| during press of `target`.
    Polls at ~200 Hz for accurate peak tracking; redraws at `fps`.
    If stop_event is given, runs until the event is set (Enter pressed);
    otherwise falls back to duration_s.
    Returns dict {finger: peak |fz|} across the full window."""
    peaks = {f: 0.0 for f in fingers}
    if stop_event is None:
        end = time.time() + duration_s
        running = lambda: time.time() < end
    else:
        running = lambda: not stop_event.is_set()
    interval = 1.0 / fps
    next_render = time.time()
    prev_lines = 0
    while running():
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


def live_press_taxels(hand, target, role, n_taxels, fingers, stop_event=None, duration_s=1.5, fps=12):
    """Live in-place display of taxel ASCII grid + peak during press of
    `target`. Polls at ~200 Hz, redraws at `fps`.
    If stop_event is given, runs until the event is set (Enter pressed);
    otherwise falls back to duration_s.
    Returns dict {finger: peak |fz| at hottest taxel} across the full window."""
    peaks = {f: 0.0 for f in fingers}
    if stop_event is None:
        end = time.time() + duration_s
        running = lambda: time.time() < end
    else:
        running = lambda: not stop_event.is_set()
    interval = 1.0 / fps
    next_render = time.time()
    prev_lines = 0
    no_layout = (f"    (no ASCII layout for {role}-{n_taxels}; "
                 f"standard models are thumb-51, finger-87, pinky-51)")
    while running():
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


def phase_1_enumerate(hand):
    banner(1, "Connect & enumerate")
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
    """Capture and apply per-taxel zero offsets so all subsequent phases
    display zero-relative readings. Not a numbered test phase — it's setup
    for the press phases. Phase 5 still independently re-tests the zeroing
    workflow (re-zero, press detection, clear)."""
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


def phase_2_resultant_press(hand):
    banner(2, "Auto-stream resultant + finger press")
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
            print(f"\n  >>> Press {f.upper()} now (vary pressure). Press Enter when done.")
            stop = _enter_event()
            all_peaks = live_press_resultant(hand, f, FINGERS, stop_event=stop)
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


def phase_3_taxels_press(hand):
    banner(3, "Auto-stream taxels + finger press")
    hand.start_tactile_stream(resultant=False, taxels=True, min_sensors=1)
    try:
        wait_for_frame(hand.get_tactile_taxels)
        reading = hand.get_tactile_taxels()
        for f in FINGERS:
            if f not in reading:
                return False, f"{f} missing from taxel frame"
            n_expected = hand.get_tactile_configuration().num_taxels[f]
            if len(reading[f]) != n_expected:
                return False, (f"{f} taxel array length {len(reading[f])} "
                               f"!= reported {n_expected}")
            if any(len(t) != 3 for t in reading[f]):
                return False, f"{f} has malformed taxel vectors"
        print(f"  Taxel array shapes verified for all 5 fingers")

        wiring = hand.config.finger_to_sensor_id
        peaks_per_press = {}
        warnings = []
        for f in FINGERS:
            print(f"\n  >>> Press {f.upper()} now (move around to light up taxels). Press Enter when done.")
            n = hand.get_tactile_configuration().num_taxels[f]
            stop = _enter_event()
            all_peaks = live_press_taxels(hand, f, FINGER_TO_ROLE[f], n, FINGERS, stop_event=stop)
            peaks_per_press[f] = all_peaks[f]
            mismatch = detect_wiring_mismatch(f, all_peaks, wiring)
            if mismatch:
                print(mismatch)
                warnings.append(f)

        weak = [f for f, p in peaks_per_press.items() if p < PRESS_THRESHOLD_N]
        if warnings:
            return False, f"wiring mismatch suspected on: {warnings} (see suggestions above)"
        if weak:
            return False, f"no/weak taxel response on: {weak}"
        return True, "all fingers show per-taxel response"
    finally:
        hand.stop_tactile_stream()


def phase_4_combined(hand):
    banner(4, "Combined mode (resultant + taxels)")
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


def phase_5_zeroing(hand):
    banner(5, "Zeroing")
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


def phase_6_lifecycle(hand):
    banner(6, "Lifecycle: stop -> restart in new mode")

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


def print_summary(results):
    print("\n===== SUMMARY =====")
    for n in sorted(results):
        ok, detail = results[n]
        flag = "PASS" if ok else "FAIL"
        print(f"  Phase {n}: {flag} - {detail}")
    if all(ok for ok, _ in results.values()):
        print("\nAll phases passed.")
    else:
        print("\nOne or more phases failed.")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "config_path",
        nargs="?",
        default=None,
        help="Path to the hand model directory or config.yaml",
    )
    args = parser.parse_args()

    hand = OrcaHandTouch(config_path=args.config_path)
    print(f"Sensor port: {hand.config.sensor_port}")
    print(f"Baudrate:    {hand.config.sensor_baudrate}")
    print(f"Wiring:      {hand.config.finger_to_sensor_id}")

    ok, msg = hand.connect_sensors_only()
    print(msg)
    if not ok:
        return

    results = {}
    phases_after_prep = [
        (2, phase_2_resultant_press),
        (3, phase_3_taxels_press),
        (4, phase_4_combined),
        (5, phase_5_zeroing),
        (6, phase_6_lifecycle),
    ]
    try:
        try:
            results[1] = phase_1_enumerate(hand)
        except Exception as e:
            results[1] = (False, f"raised {type(e).__name__}: {e}")

        if results[1][0]:
            try:
                prep_zero_baseline(hand)
            except Exception as e:
                print(f"  WARNING: zero baseline prep failed ({type(e).__name__}: {e}); "
                      "subsequent phases will see raw readings")

        for n, fn in phases_after_prep:
            try:
                results[n] = fn(hand)
            except Exception as e:
                results[n] = (False, f"raised {type(e).__name__}: {e}")
    finally:
        hand.disconnect()

    print_summary(results)


if __name__ == "__main__":
    main()
