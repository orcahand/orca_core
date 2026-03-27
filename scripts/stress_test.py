"""
Stress test for the ORCA Hand.

Cycles the hand open/close for a given duration, logging temperature, current,
and position data.  Produces plots at the end (or run plot_stress_test.py
separately on the saved data).

Usage:
    python scripts/stress_test.py orca_core/configs/orcahand_v1_left --duration 5
    python scripts/stress_test.py orca_core/configs/orcahand_v1_left --duration 0.01  # quick smoke test
"""

import argparse
import csv
import json
import os
import signal
import sys
import time
from datetime import datetime

from orca_core import OrcaHand

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MAX_TEMP = 70  # °C — conservative across motor families
TEMP_CHECK_INTERVAL = 2.0  # seconds
SAMPLE_INTERVAL = 0.05  # 50 ms — current/position sampling during moves
FLUSH_INTERVAL = 30.0  # seconds between CSV flushes
SLACK_CURRENT_THRESHOLD = 30  # mA — below this during motion ≈ no tension
DEFAULT_DURATION_HOURS = 5

OPEN = {
    "index_abd": -30, "middle_abd": 0, "ring_abd": 10, "pinky_abd": 25,
    "thumb_abd": -20,
    "index_mcp": -40, "middle_mcp": -40, "ring_mcp": -40, "pinky_mcp": -40,
    "thumb_mcp": 45,
    "index_pip": -10, "middle_pip": -10, "ring_pip": -10, "pinky_pip": -10,
    "thumb_dip": 90,
    "thumb_cmc": 40,
    "wrist": -25,
}

CLOSE = {
    "index_abd": 15, "middle_abd": 15, "ring_abd": -15, "pinky_abd": -15,
    "thumb_abd": 55,
    "index_mcp": 45, "middle_mcp": 45, "ring_mcp": 45, "pinky_mcp": 45,
    "thumb_mcp": -40,
    "index_pip": 90, "middle_pip": 90, "ring_pip": 90, "pinky_pip": 90,
    "thumb_dip": -30,
    "thumb_cmc": -50,
    "wrist": 10,
}

# ---------------------------------------------------------------------------
# Terminal colours (for live display)
# ---------------------------------------------------------------------------
RST = "\033[0m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
DIM = "\033[2m"


def temp_color(pct):
    if pct >= 90:
        return RED
    if pct >= 70:
        return YELLOW
    return GREEN


# ---------------------------------------------------------------------------
# Logging helpers
# ---------------------------------------------------------------------------
READING_FIELDS = ["timestamp", "elapsed_s", "phase", "cycle",
                  "motor_id", "joint_name",
                  "temperature", "current", "position"]

EVENT_FIELDS = ["timestamp", "elapsed_s", "motor_id", "joint_name",
                "event", "temperature"]


class DataLogger:
    """Buffered CSV writer that flushes periodically and on close."""

    def __init__(self, output_dir, motor_to_joint, metadata):
        os.makedirs(output_dir, exist_ok=True)
        self.output_dir = output_dir
        self.motor_to_joint = motor_to_joint

        # readings
        self._readings_path = os.path.join(output_dir, "readings.csv")
        self._readings_file = open(self._readings_path, "w", newline="")
        self._readings_writer = csv.DictWriter(self._readings_file,
                                               fieldnames=READING_FIELDS)
        self._readings_writer.writeheader()

        # events
        self._events_path = os.path.join(output_dir, "events.csv")
        self._events_file = open(self._events_path, "w", newline="")
        self._events_writer = csv.DictWriter(self._events_file,
                                             fieldnames=EVENT_FIELDS)
        self._events_writer.writeheader()

        self._last_flush = time.monotonic()

        # metadata
        with open(os.path.join(output_dir, "metadata.json"), "w") as f:
            json.dump(metadata, f, indent=2)

    # -- writing ----------------------------------------------------------
    def write_readings(self, rows):
        self._readings_writer.writerows(rows)
        self._maybe_flush()

    def write_event(self, row):
        self._events_writer.writerow(row)
        self._events_file.flush()

    # -- flush / close ----------------------------------------------------
    def _maybe_flush(self):
        now = time.monotonic()
        if now - self._last_flush >= FLUSH_INTERVAL:
            self.flush()
            self._last_flush = now

    def flush(self):
        self._readings_file.flush()
        self._events_file.flush()

    def close(self):
        self.flush()
        self._readings_file.close()
        self._events_file.close()


# ---------------------------------------------------------------------------
# Live display
# ---------------------------------------------------------------------------
def print_status(hand, temps, elapsed, cycle, shutdown_motors):
    motor_to_joint = hand.motor_to_joint_dict
    fingers = ["thumb", "index", "middle", "ring", "pinky", "wrist"]
    grouped = {f: [] for f in fingers}
    for mid, t in temps.items():
        joint = motor_to_joint.get(mid, f"motor_{mid}")
        finger = joint.split("_")[0]
        if finger in grouped:
            grouped[finger].append((joint, mid, t))

    print("\033[2J\033[H", end="")
    hrs, rem = divmod(int(elapsed), 3600)
    mins, secs = divmod(rem, 60)
    print(f"{BOLD}  ORCA Hand Stress Test{RST}  "
          f"[{hrs:02d}:{mins:02d}:{secs:02d}]  cycle {cycle}")
    if shutdown_motors:
        names = [motor_to_joint.get(m, str(m)) for m in shutdown_motors]
        print(f"  {RED}SHUTDOWN: {', '.join(names)}{RST}")
    print(f"  {DIM}Max operating temp: {MAX_TEMP}°C{RST}\n")

    print(f"  {BOLD}{'Joint':<14} {'Motor':>5} {'Temp':>6} {'%Max':>6}  {'':>10}{RST}")
    print(f"  {'─' * 48}")
    for finger in fingers:
        for joint, mid, t in grouped.get(finger, []):
            pct = t / MAX_TEMP * 100
            c = temp_color(pct)
            bar_len = int(pct / 100 * 10)
            bar = f"{c}{'█' * bar_len}{DIM}{'░' * (10 - bar_len)}{RST}"
            print(f"  {joint:<14} {mid:>5} {c}{t:>4.0f}°C {pct:>5.0f}%{RST}  {bar}")
    print(f"  {'─' * 48}")
    max_t = max(temps.values())
    max_pct = max_t / MAX_TEMP * 100
    c = temp_color(max_pct)
    print(f"  {'Peak':<14} {'':>5} {c}{max_t:>4.0f}°C {max_pct:>5.0f}%{RST}\n")


# ---------------------------------------------------------------------------
# Sampling
# ---------------------------------------------------------------------------
def sample_all(hand, start_time, phase, cycle, last_temps):
    """Read current, position, and (if due) temperature. Return reading rows."""
    now = time.time()
    elapsed = now - start_time
    currents = hand.get_motor_current(as_dict=True)
    positions = hand.get_motor_pos(as_dict=True)
    motor_to_joint = hand.motor_to_joint_dict

    rows = []
    for mid in hand.motor_ids:
        rows.append({
            "timestamp": now,
            "elapsed_s": round(elapsed, 3),
            "phase": phase,
            "cycle": cycle,
            "motor_id": mid,
            "joint_name": motor_to_joint.get(mid, f"motor_{mid}"),
            "temperature": last_temps.get(mid, ""),
            "current": currents.get(mid, ""),
            "position": positions.get(mid, ""),
        })
    return rows


def sample_temps(hand, start_time, phase, cycle):
    """Read temperatures and return (temps_dict, reading_rows)."""
    now = time.time()
    elapsed = now - start_time
    temps = hand.get_motor_temp(as_dict=True)
    motor_to_joint = hand.motor_to_joint_dict
    rows = []
    for mid in hand.motor_ids:
        rows.append({
            "timestamp": now,
            "elapsed_s": round(elapsed, 3),
            "phase": phase,
            "cycle": cycle,
            "motor_id": mid,
            "joint_name": motor_to_joint.get(mid, f"motor_{mid}"),
            "temperature": temps.get(mid, ""),
            "current": "",
            "position": "",
        })
    return temps, rows


# ---------------------------------------------------------------------------
# Shutdown tracking
# ---------------------------------------------------------------------------
class ShutdownTracker:
    """Track per-motor thermal shutdown events."""

    def __init__(self, motor_ids, motor_to_joint):
        self.motor_to_joint = motor_to_joint
        self.in_shutdown = {}  # motor_id -> timestamp when shutdown started
        for mid in motor_ids:
            self.in_shutdown[mid] = None

    def update(self, temps, start_time, logger):
        now = time.time()
        elapsed = now - start_time
        active = set()
        for mid, temp in temps.items():
            joint = self.motor_to_joint.get(mid, f"motor_{mid}")
            if temp >= MAX_TEMP and self.in_shutdown[mid] is None:
                self.in_shutdown[mid] = now
                logger.write_event({
                    "timestamp": now, "elapsed_s": round(elapsed, 3),
                    "motor_id": mid, "joint_name": joint,
                    "event": "shutdown", "temperature": temp,
                })
            elif temp < MAX_TEMP - 5 and self.in_shutdown[mid] is not None:
                # 5°C hysteresis before considering recovered
                logger.write_event({
                    "timestamp": now, "elapsed_s": round(elapsed, 3),
                    "motor_id": mid, "joint_name": joint,
                    "event": "recovered", "temperature": temp,
                })
                self.in_shutdown[mid] = None
            if self.in_shutdown[mid] is not None:
                active.add(mid)
        return active


# ---------------------------------------------------------------------------
# Movement with sampling
# ---------------------------------------------------------------------------
def move_with_sampling(hand, target, phase, cycle, start_time,
                       last_temps, logger):
    """Execute a movement while sampling current/position during it."""
    # Start the movement
    hand.set_joint_pos(target, num_steps=25, step_size=0.001)

    # Sample a few times right after the motion to capture transient currents
    for _ in range(3):
        rows = sample_all(hand, start_time, phase, cycle, last_temps)
        logger.write_readings(rows)
        time.sleep(SAMPLE_INTERVAL)


def hold_with_sampling(hand, phase, cycle, start_time, last_temps, logger,
                       hold_time=2.0):
    """Hold position for hold_time while sampling periodically."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < hold_time:
        rows = sample_all(hand, start_time, phase, cycle, last_temps)
        logger.write_readings(rows)
        time.sleep(0.5)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="ORCA Hand stress test")
    parser.add_argument("model_path", type=str, nargs="?", default=None)
    parser.add_argument("--port", type=str, default=None)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_HOURS,
                        help="Test duration in hours (default: 5)")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory for logs and plots")
    parser.add_argument("--no-plot", action="store_true",
                        help="Skip plot generation at the end")
    args = parser.parse_args()

    # Output directory
    if args.output_dir is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output_dir = os.path.join("stress_test_results", ts)

    # Connect
    hand = OrcaHand(args.model_path)
    status = hand.connect(port=args.port)
    if not status[0]:
        print(f"Failed to connect: {status[1]}")
        sys.exit(1)

    hand.enable_torque()
    motor_to_joint = hand.motor_to_joint_dict

    metadata = {
        "start_time": datetime.now().isoformat(),
        "duration_hours": args.duration,
        "model_path": args.model_path or "auto",
        "motor_ids": hand.motor_ids,
        "joint_ids": hand.joint_ids,
        "motor_to_joint": {str(k): v for k, v in motor_to_joint.items()},
        "max_temp": MAX_TEMP,
        "slack_current_threshold": SLACK_CURRENT_THRESHOLD,
    }

    logger = DataLogger(args.output_dir, motor_to_joint, metadata)
    tracker = ShutdownTracker(hand.motor_ids, motor_to_joint)

    duration_s = args.duration * 3600
    start_time = time.time()
    cycle = 0
    last_temps = {}
    last_temp_check = 0
    shutdown_motors = set()

    # Graceful shutdown on Ctrl+C
    stop = False

    def on_sigint(sig, frame):
        nonlocal stop
        stop = True
        print(f"\n{YELLOW}Stopping after current cycle...{RST}")

    signal.signal(signal.SIGINT, on_sigint)

    print(f"Stress test started — {args.duration}h, logging to {args.output_dir}/")

    try:
        while not stop and (time.time() - start_time) < duration_s:
            cycle += 1

            # Temperature check
            now_mono = time.monotonic()
            if now_mono - last_temp_check >= TEMP_CHECK_INTERVAL:
                last_temp_check = now_mono
                last_temps, temp_rows = sample_temps(
                    hand, start_time, "temp_check", cycle)
                logger.write_readings(temp_rows)
                shutdown_motors = tracker.update(last_temps, start_time, logger)
                print_status(hand, last_temps, time.time() - start_time,
                             cycle, shutdown_motors)

            # Open
            move_with_sampling(hand, OPEN, "opening", cycle,
                               start_time, last_temps, logger)
            hold_with_sampling(hand, "open_hold", cycle,
                               start_time, last_temps, logger)

            # Temperature check mid-cycle
            now_mono = time.monotonic()
            if now_mono - last_temp_check >= TEMP_CHECK_INTERVAL:
                last_temp_check = now_mono
                last_temps, temp_rows = sample_temps(
                    hand, start_time, "temp_check", cycle)
                logger.write_readings(temp_rows)
                shutdown_motors = tracker.update(last_temps, start_time, logger)
                print_status(hand, last_temps, time.time() - start_time,
                             cycle, shutdown_motors)

            # Close
            move_with_sampling(hand, CLOSE, "closing", cycle,
                               start_time, last_temps, logger)
            hold_with_sampling(hand, "close_hold", cycle,
                               start_time, last_temps, logger)

    finally:
        elapsed = time.time() - start_time
        hrs, rem = divmod(int(elapsed), 3600)
        mins, secs = divmod(rem, 60)
        print(f"\nCompleted {cycle} cycles in {hrs:02d}:{mins:02d}:{secs:02d}")

        hand.disable_torque()
        hand.disconnect()
        logger.close()
        print(f"Data saved to {args.output_dir}/")

        if not args.no_plot:
            print("Generating plots...")
            try:
                script_dir = os.path.dirname(os.path.abspath(__file__))
                sys.path.insert(0, script_dir)
                from plot_stress_test import generate_plots
                generate_plots(args.output_dir)
            except Exception as e:
                print(f"Plot generation failed: {e}")
                print(f"Run manually: python scripts/plot_stress_test.py {args.output_dir}")


if __name__ == "__main__":
    main()
