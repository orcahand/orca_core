"""Continuous open/close comms + motor stress test for the ORCA Hand.

Purpose
-------
Drive the hand open<->close forever (like scripts/test.py) at maximum data
traffic and watch for the communication or the motors giving out over a long
soak (hours). It runs the full hand:

  * joint-encoder feedback on the joints with valid encoder readings,
  * the normal motor controller on the remaining moving joints,
  * the tactile stream in combined mode (resultant + per-taxel) for the
    largest possible sensor payload,
  * the 100 Hz joint loop, which is the heaviest motor-write path there is.

Two joints are never commanded (kept perfectly still) because of a
mis-routed pinky tendon:  ``wrist`` and ``pinky_pip``.

Drop-out detection
------------------
The failure modes we care about, and how we tell them apart:

  * Encoder sensor dies -> it streams the *exact same* value (bit-identical)
    or stops streaming entirely.  Caught as either
      - a joint whose measured angle is exactly constant while commanded to
        move  ("FROZEN-EXACT"), or
      - the joint loop's ``cycles_no_reading`` climbing / ``fallback_active``.
  * Tactile stream dies -> ``frames_ok`` stops advancing ("TACTILE-STALL").
  * Motor overheats / gives out -> holds position with *slight* noise (not
    bit-identical) and/or temperature spikes.  Caught as "STALL-NOISY" plus
    the per-motor temperature log.

Everything is logged to a wide CSV (one row per monitor tick) plus a
human-readable event log, both flushed continuously so a multi-hour run
survives an abrupt stop.

Usage
-----
    uv run python stresstest/stress_comm.py \
        orca_core/models/v2/orcahand_stresstest/config.yaml
    uv run python stresstest/stress_comm.py <config> --duration-hours 4
    uv run python stresstest/stress_comm.py <config> --half-cycle-s 2.0 --no-tactile
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import time
from collections import deque
from datetime import datetime, timezone

from orca_core import load_hand
from orca_core.constants import CURRENT_BASED_POSITION
from orca_core.hardware_hand_joint_feedback import JointFeedbackConnectError

# --- joints -----------------------------------------------------------------

# Never commanded: mis-routed pinky tendon means moving these risks damage.
FROZEN_JOINTS = ("wrist", "pinky_pip")

# Joints driven by the joint-encoder feedback loop (valid encoder readings).
# Routing is actually decided by config.joint_encoder_joints; listed here only
# for labelling and for the "commanded to move" amplitude gate.
FEEDBACK_JOINTS = (
    "thumb_abd", "thumb_mcp",
    "index_abd", "index_mcp", "index_pip",
    "middle_abd", "middle_mcp", "middle_pip",
    "ring_mcp",
)

# Open / closed target poses (degrees). Derived from scripts/test.py minus the
# frozen joints. Values are clamped to the config ROM before being sent.
OPEN_POSE = {
    "index_abd": -30, "middle_abd": 0, "ring_abd": 10, "pinky_abd": 25,
    "thumb_abd": -18,
    "index_mcp": -40, "middle_mcp": -40, "ring_mcp": -40, "pinky_mcp": -40,
    "thumb_mcp": 45,
    "index_pip": -10, "middle_pip": -10, "ring_pip": -10,
    "thumb_dip": 90, "thumb_cmc": 40,
}
CLOSE_POSE = {
    "index_abd": 15, "middle_abd": 15, "ring_abd": -15, "pinky_abd": -15,
    "thumb_abd": 55,
    "index_mcp": 45, "middle_mcp": 45, "ring_mcp": 45, "pinky_mcp": 45,
    "thumb_mcp": -40,
    "index_pip": 90, "middle_pip": 90, "ring_pip": 90,
    "thumb_dip": -30, "thumb_cmc": -50,
}
FINGERS = ("thumb", "index", "middle", "ring", "pinky")

# --- detection thresholds ---------------------------------------------------

MOVE_MIN_DEG = 12.0        # only flag freeze on joints commanded to swing this far
FREEZE_RANGE_DEG = 2.0     # measured swing below this over a full cycle == frozen
MOTOR_FREEZE_RANGE_RAD = 0.03   # ~1.7 deg at the motor; below == stalled
MOTOR_MOVE_MIN_RAD = 0.05

RST, GREEN, YELLOW, RED, BOLD, DIM = (
    "\033[0m", "\033[92m", "\033[93m", "\033[91m", "\033[1m", "\033[2m")


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().strftime("%Y-%m-%d %H:%M:%S")


def lerp_pose(a: dict, b: dict, t: float) -> dict:
    return {j: a[j] * (1.0 - t) + b[j] * t for j in a}


class EventLog:
    """Human-readable append log; every anomaly is timestamped and de-duped so
    a persistent fault does not spam a line per tick."""

    def __init__(self, path):
        self._f = open(path, "a", buffering=1)  # line-buffered
        self._active = {}  # key -> first-seen elapsed
        self.write(f"===== stress run started {now_iso()} =====")

    def write(self, msg):
        line = f"[{now_iso()}] {msg}"
        self._f.write(line + "\n")
        print(line)

    def anomaly(self, key, msg, elapsed):
        """Log an anomaly on its rising edge and when it clears."""
        if key not in self._active:
            self._active[key] = elapsed
            self.write(f"{RED}{BOLD}ANOMALY{RST} {msg}")

    def clear(self, key, elapsed):
        if key in self._active:
            dur = elapsed - self._active.pop(key)
            self.write(f"{GREEN}RECOVERED{RST} {key} (lasted {dur:.1f}s)")

    def active_keys(self):
        return set(self._active)

    def close(self):
        self.write(f"===== stress run ended {now_iso()} =====")
        self._f.close()


class SignalWindow:
    """Sliding time window of (t, value) samples for range-based freeze checks."""

    def __init__(self, span_s):
        self.span_s = span_s
        self.buf = deque()

    def push(self, t, v):
        self.buf.append((t, v))
        cutoff = t - self.span_s
        while self.buf and self.buf[0][0] < cutoff:
            self.buf.popleft()

    def full(self, t):
        return self.buf and (t - self.buf[0][0]) >= self.span_s * 0.95

    def stats(self):
        vals = [v for _, v in self.buf]
        return min(vals), max(vals), len(vals), (len(set(vals)) == 1)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("config_path", help="Path to the hand config.yaml")
    p.add_argument("--duration-hours", type=float, default=None,
                   help="Stop after N hours (default: run until Ctrl-C)")
    p.add_argument("--log-dir", default="stresstest/logs")
    p.add_argument("--half-cycle-s", type=float, default=2.5,
                   help="Seconds for one open->close (or close->open) sweep")
    p.add_argument("--interp-steps", type=int, default=50,
                   help="Waypoints per sweep")
    p.add_argument("--hold-s", type=float, default=0.4,
                   help="Pause at each open/close extreme")
    p.add_argument("--motor-read-s", type=float, default=1.5,
                   help="How often to bulk-read motor temp/current/pos")
    p.add_argument("--heartbeat-s", type=float, default=5.0)
    p.add_argument("--warn-temp", type=float, default=60.0)
    p.add_argument("--pause-temp", type=float, default=68.0,
                   help="Hold motion when any motor reaches this temp")
    p.add_argument("--resume-temp", type=float, default=58.0)
    p.add_argument("--max-current", type=float, default=None,
                   help="Override config max_current (mA)")
    p.add_argument("--no-tactile", action="store_true",
                   help="Skip the tactile stream (still runs joint feedback)")
    p.add_argument("--motors-only", action="store_true",
                   help="Skip both joint feedback and tactile: drive every moving "
                        "joint through the normal motor controller and monitor motors "
                        "only. Use on hosts (e.g. macOS) where the motor bus and the "
                        "sensor CDC cannot stream simultaneously.")
    return p.parse_args()


class StressTest:
    def __init__(self, args):
        self.args = args
        os.makedirs(args.log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.events = EventLog(os.path.join(args.log_dir, f"stress-{stamp}.events.log"))
        self._csv_path = os.path.join(args.log_dir, f"stress-{stamp}.csv")
        self.hand = None
        self._stop = False
        self._csv_f = None
        self._csv = None

        # cycle window spans one full open->close->open plus the holds
        self._window_s = 2 * args.half_cycle_s + 2 * args.hold_s + 1.0
        self.enc_win = {j: SignalWindow(self._window_s) for j in FEEDBACK_JOINTS}
        self.mot_win = {}  # normal moving joints -> SignalWindow (built after connect)
        self._motion_active_since = None
        self._amp = {j: abs(CLOSE_POSE[j] - OPEN_POSE[j])
                     for j in OPEN_POSE if j in CLOSE_POSE}

        # rate baselines
        self._prev = {"t": None, "cycles_ok": 0, "no_reading": 0, "tac_frames": 0}
        # carried-forward slow signals
        self._temps = {}
        self._currents = {}
        self._motor_read_fails = 0
        self._cmd_fails = 0

    # -- lifecycle -----------------------------------------------------------

    def connect(self):
        a = self.args
        self._feedback = not a.motors_only
        self._tactile = not a.no_tactile and not a.motors_only
        self.hand = load_hand(config_path=a.config_path, engage_feedback=self._feedback)
        cls = type(self.hand).__name__
        self.events.write(f"loaded hand class: {cls} "
                          f"(feedback={self._feedback} tactile={self._tactile})")
        try:
            ok, msg = self.hand.connect()
        except JointFeedbackConnectError as exc:
            self.events.write(f"{RED}connect failed: {exc}{RST}")
            raise
        self.events.write(f"connect: {ok} | {msg}")
        if not ok:
            raise RuntimeError("connect returned False")

        # normal moving joints = everything commanded but not encoder-backed
        commanded = set(OPEN_POSE)
        enc_backed = set(self.hand._encoder_backed_joints()) if self._feedback else set()
        self._normal_joints = sorted(commanded - enc_backed)
        self.events.write(f"encoder-feedback joints: {sorted(enc_backed)}")
        self.events.write(f"normal-controller joints: {self._normal_joints}")
        self.events.write(f"frozen (never moved): {list(FROZEN_JOINTS)}")

        self._j2m = self.hand.config.joint_to_motor_map
        self._m2j = self.hand.config.motor_to_joint_dict
        for j in self._normal_joints:
            self.mot_win[j] = SignalWindow(self._window_s)

        if a.max_current is not None:
            self.hand.set_max_current(a.max_current)
        else:
            self.hand.set_max_current(self.hand.config.max_current)
        # Explicit: motors must be in current_based_position for the loop's
        # goal-position writes (wrist is auto-kept in multi_turn_position).
        self.hand.set_control_mode(CURRENT_BASED_POSITION)
        self.hand.enable_torque()
        if self._feedback:
            # Re-anchor the loop to the live pose so enabling torque doesn't lurch.
            self.hand.rebase_loop()

        if self._tactile:
            self.hand.start_tactile_stream(resultant=True, taxels=True, min_sensors=1)
            cfg = self.hand.get_tactile_configuration()
            self.events.write(f"tactile stream: {cfg}")

        self._open_csv(sorted(enc_backed))

    def _open_csv(self, enc_joints):
        self._enc_joints = enc_joints
        motor_joints = [self._m2j.get(m, f"motor_{m}") for m in self.hand.config.motor_ids]
        self._motor_joints = motor_joints
        cols = ["iso", "elapsed_s", "cycle", "phase", "motion_active",
                "loop_cycles_ok", "loop_no_reading", "loop_overrun", "loop_estops",
                "loop_fallback", "loop_last_dt_ms", "loop_cmds",
                "loop_ok_rate", "loop_noread_rate",
                "tac_frames_ok", "tac_frame_rate", "tac_bad", "tac_last_err"]
        cols += [f"enc_{j}" for j in enc_joints]
        cols += [f"tacfz_{f}" for f in FINGERS]
        cols += [f"temp_{j}" for j in motor_joints]
        cols += [f"cur_{j}" for j in motor_joints]
        cols += ["motor_read_fails", "cmd_fails", "anomalies"]
        self._csv_f = open(self._csv_path, "w", newline="", buffering=1)
        self._csv = csv.DictWriter(self._csv_f, fieldnames=cols)
        self._csv.writeheader()
        self.events.write(f"CSV -> {self._csv_path} ({len(cols)} cols)")

    def disconnect(self):
        if self.hand is None:
            return
        try:
            self.hand.disable_torque()
        except Exception as exc:
            self.events.write(f"disable_torque error: {exc!r}")
        try:
            self.hand.disconnect()
        except Exception as exc:
            self.events.write(f"disconnect error: {exc!r}")

    # -- command path --------------------------------------------------------

    def _send(self, pose):
        try:
            self.hand.set_joint_positions(pose, num_steps=1)
        except Exception as exc:
            self._cmd_fails += 1
            self.events.anomaly("cmd_fail", f"set_joint_positions raised: {exc!r}", self._elapsed)

    # -- monitoring ----------------------------------------------------------

    def _sample_cheap(self, t):
        """Cache-only reads (no motor bus): loop stats, encoder joints, tactile."""
        stats = self.hand.get_loop_stats() if self._feedback else {}
        measured = self.hand.get_measured_joints() if self._feedback else {}
        for j in FEEDBACK_JOINTS:
            if j in measured:
                self.enc_win[j].push(t, round(measured[j], 4))
        tac_fz = {f: float("nan") for f in FINGERS}
        tac_stats = None
        if self._tactile:
            try:
                tac_stats = self.hand.get_tactile_stats()
                forces = self.hand.get_tactile_forces()
                if forces is not None:
                    for f in FINGERS:
                        if f in forces:
                            tac_fz[f] = round(forces[f][2], 2)
            except Exception:
                pass
        return stats, measured, tac_fz, tac_stats

    def _sample_motors(self, t):
        """Bulk motor-bus reads (temp/current/pos). Serialized with the loop by
        the hand's motor lock; failures are counted, not fatal."""
        try:
            self._temps = self.hand.get_motor_temp(as_dict=True)
            self._currents = self.hand.get_motor_current(as_dict=True)
            pos = self.hand.get_motor_pos(as_dict=True)
            for j in self._normal_joints:
                mid = abs(self._j2m[j])
                if mid in pos:
                    self.mot_win[j].push(t, round(float(pos[mid]), 5))
        except Exception as exc:
            self._motor_read_fails += 1
            self.events.anomaly("motor_read_fail", f"motor read raised: {exc!r}", self._elapsed)
            return False
        self.events.clear("motor_read_fail", self._elapsed)
        return True

    def _detect(self, t, stats, tac_stats, motion_active):
        """Evaluate drop-out conditions; return set of active anomaly keys."""
        anomalies = set()
        e = self._elapsed

        # rates since last tick
        dt = (t - self._prev["t"]) if self._prev["t"] else None
        if dt and dt > 0:
            ok_rate = (stats.get("cycles_ok", 0) - self._prev["cycles_ok"]) / dt
            noread_rate = (stats.get("cycles_no_reading", 0) - self._prev["no_reading"]) / dt
        else:
            ok_rate = noread_rate = float("nan")

        # loop / encoder-stream health (feedback mode only)
        if self._feedback:
            if stats.get("fallback_active"):
                anomalies.add("loop_fallback")
                self.events.anomaly("loop_fallback", "joint loop FALLBACK/e-stop active", e)
            else:
                self.events.clear("loop_fallback", e)
            if motion_active and not math.isnan(ok_rate) and ok_rate < 1.0:
                anomalies.add("loop_stalled")
                self.events.anomaly("loop_stalled", f"joint loop not cycling (ok_rate={ok_rate:.1f}/s)", e)
            else:
                self.events.clear("loop_stalled", e)
            if not math.isnan(noread_rate) and noread_rate > 20.0:
                anomalies.add("enc_noreading")
                self.events.anomaly("enc_noreading",
                                    f"encoder frames missing (no_reading +{noread_rate:.0f}/s)", e)
            else:
                self.events.clear("enc_noreading", e)

        # per-joint encoder freeze (only after a full clean-motion window)
        if motion_active and self._motion_active_since and (t - self._motion_active_since) >= self._window_s:
            for j in FEEDBACK_JOINTS:
                if self._amp.get(j, 0) < MOVE_MIN_DEG:
                    continue
                w = self.enc_win[j]
                if not w.full(t):
                    continue
                lo, hi, n, exact = w.stats()
                if (hi - lo) < FREEZE_RANGE_DEG:
                    kind = "FROZEN-EXACT (sensor dropout?)" if exact else "STALL-NOISY (motor?)"
                    key = f"enc_frozen_{j}"
                    anomalies.add(key)
                    self.events.anomaly(key, f"encoder joint {j}: {kind} "
                                        f"range={hi-lo:.2f} deg over {n} samples", e)
                else:
                    self.events.clear(f"enc_frozen_{j}", e)
            # normal-joint (motor-pos) stall
            for j in self._normal_joints:
                if self._amp.get(j, 0) < MOVE_MIN_DEG:
                    continue
                w = self.mot_win[j]
                if len(w.buf) < 4 or not w.full(t):
                    continue
                lo, hi, n, exact = w.stats()
                if (hi - lo) < MOTOR_FREEZE_RANGE_RAD:
                    kind = "EXACT (read stuck?)" if exact else "NOISY (motor stall/overload?)"
                    key = f"mot_frozen_{j}"
                    anomalies.add(key)
                    self.events.anomaly(key, f"motor joint {j}: not moving {kind} "
                                        f"range={hi-lo:.4f} rad over {n} samples", e)
                else:
                    self.events.clear(f"mot_frozen_{j}", e)

        # tactile stream health
        if tac_stats is not None and dt and dt > 0:
            frate = (tac_stats.frames_ok - self._prev["tac_frames"]) / dt
            if motion_active and frate < 1.0:
                anomalies.add("tactile_stall")
                self.events.anomaly("tactile_stall", "tactile stream stalled (0 frames)", e)
            else:
                self.events.clear("tactile_stall", e)

        # thermal
        if self._temps:
            tmax = max(self._temps.values())
            if tmax >= self.args.warn_temp:
                mj = self._m2j.get(max(self._temps, key=self._temps.get), "?")
                self.events.anomaly("temp_warn", f"motor {mj} hot: {tmax:.0f}C", e)
            if tmax < self.args.warn_temp - 2:
                self.events.clear("temp_warn", e)

        self._prev = {"t": t, "cycles_ok": stats.get("cycles_ok", 0),
                      "no_reading": stats.get("cycles_no_reading", 0),
                      "tac_frames": tac_stats.frames_ok if tac_stats else 0}
        return anomalies, ok_rate, noread_rate

    def _thermal_gate(self):
        """Return True if it is safe to keep moving; hold when any motor is too
        hot until it cools past resume-temp."""
        if not self._temps:
            return True
        tmax = max(self._temps.values())
        if getattr(self, "_thermal_hold", False):
            if tmax <= self.args.resume_temp:
                self._thermal_hold = False
                self.events.write(f"{GREEN}thermal recovered ({tmax:.0f}C) resuming motion{RST}")
                return True
            return False
        if tmax >= self.args.pause_temp:
            self._thermal_hold = True
            self.events.write(f"{RED}thermal pause: {tmax:.0f}C >= {self.args.pause_temp}C "
                              f"holding motion{RST}")
            return False
        return True

    def _log_row(self, t, cycle, phase, motion_active, stats, tac_fz, tac_stats,
                 ok_rate, noread_rate, anomalies):
        row = {
            "iso": now_iso(), "elapsed_s": round(self._elapsed, 2),
            "cycle": cycle, "phase": phase, "motion_active": int(motion_active),
            "loop_cycles_ok": stats.get("cycles_ok", 0),
            "loop_no_reading": stats.get("cycles_no_reading", 0),
            "loop_overrun": stats.get("cycles_overrun", 0),
            "loop_estops": stats.get("e_stops", 0),
            "loop_fallback": int(bool(stats.get("fallback_active"))),
            "loop_last_dt_ms": round(stats.get("last_dt_s", float("nan")) * 1000, 2),
            "loop_cmds": stats.get("commands_sent", 0),
            "loop_ok_rate": round(ok_rate, 1) if not math.isnan(ok_rate) else "",
            "loop_noread_rate": round(noread_rate, 1) if not math.isnan(noread_rate) else "",
            "tac_frames_ok": tac_stats.frames_ok if tac_stats else "",
            "tac_frame_rate": "",
            "tac_bad": (tac_stats.frames_bad_payload + tac_stats.frames_bad_payload_size)
                        if tac_stats else "",
            "tac_last_err": tac_stats.last_error_code if tac_stats else "",
            "motor_read_fails": self._motor_read_fails,
            "cmd_fails": self._cmd_fails,
            "anomalies": ";".join(sorted(anomalies)) if anomalies else "",
        }
        measured = self.hand.get_measured_joints() if self._feedback else {}
        for j in self._enc_joints:
            row[f"enc_{j}"] = round(measured.get(j, float("nan")), 3)
        for f in FINGERS:
            row[f"tacfz_{f}"] = tac_fz.get(f, "")
        for j in self._motor_joints:
            mid = self._j2m.get(j)
            row[f"temp_{j}"] = self._temps.get(abs(mid), "") if mid else ""
            row[f"cur_{j}"] = self._currents.get(abs(mid), "") if mid else ""
        self._csv.writerow(row)

    # -- main loop -----------------------------------------------------------

    def run(self):
        a = self.args
        self.events.write(f"config: half_cycle={a.half_cycle_s}s steps={a.interp_steps} "
                          f"hold={a.hold_s}s window={self._window_s:.1f}s")
        # gentle ramp from the live full pose to OPEN so nothing lurches. The
        # base-class read gives all 17 joints from motor positions (the full
        # hand's own _get_joint_positions only returns the encoder joints).
        from orca_core.hardware_hand import OrcaHand
        try:
            cur = OrcaHand._get_joint_positions(self.hand).as_dict()
        except Exception:
            cur = {}
        start_pose = {}
        for j in OPEN_POSE:
            v = cur.get(j)
            start_pose[j] = float(v) if isinstance(v, (int, float)) else OPEN_POSE[j]
        self.events.write("ramping to OPEN pose...")
        for i in range(a.interp_steps + 1):
            if self._stop:
                return
            self._send(lerp_pose(start_pose, OPEN_POSE, i / a.interp_steps))
            time.sleep(a.half_cycle_s / a.interp_steps)

        self._t0 = time.monotonic()
        self._elapsed = 0.0
        last_motor_read = 0.0
        last_heartbeat = 0.0
        cycle = 0
        deadline = (self._t0 + a.duration_hours * 3600) if a.duration_hours else None
        targets = [("close", CLOSE_POSE), ("open", OPEN_POSE)]
        prev_pose = dict(OPEN_POSE)
        self._motion_active_since = time.monotonic()

        while not self._stop:
            for phase, target in targets:
                for i in range(1, a.interp_steps + 1):
                    if self._stop:
                        break
                    t = time.monotonic()
                    self._elapsed = t - self._t0
                    if deadline and t >= deadline:
                        self.events.write("duration reached; stopping")
                        self._stop = True
                        break

                    moving = self._thermal_gate()
                    if moving:
                        if self._motion_active_since is None:
                            self._motion_active_since = t
                        self._send(lerp_pose(prev_pose, target, i / a.interp_steps))
                    else:
                        self._motion_active_since = None

                    # slow motor read
                    if t - last_motor_read >= a.motor_read_s:
                        last_motor_read = t
                        self._sample_motors(t)

                    stats, measured, tac_fz, tac_stats = self._sample_cheap(t)
                    anomalies, ok_rate, noread_rate = self._detect(t, stats, tac_stats,
                                                                   moving and self._motion_active_since is not None)
                    self._log_row(t, cycle, phase if moving else "thermal_hold",
                                  moving, stats, tac_fz, tac_stats, ok_rate, noread_rate, anomalies)

                    if t - last_heartbeat >= a.heartbeat_s:
                        last_heartbeat = t
                        self._heartbeat(cycle, phase, moving, stats, tac_stats, anomalies)

                    time.sleep(a.half_cycle_s / a.interp_steps)
                if self._stop:
                    break
                prev_pose = dict(target)
                time.sleep(a.hold_s)
            cycle += 1

    def _heartbeat(self, cycle, phase, moving, stats, tac_stats, anomalies):
        tmax = max(self._temps.values()) if self._temps else float("nan")
        tcol = RED if tmax >= self.args.warn_temp else (YELLOW if tmax >= 50 else GREEN)
        acol = RED if anomalies else GREEN
        tac = f" tac_ok={tac_stats.frames_ok}" if tac_stats else ""
        hrs = self._elapsed / 3600
        self.events.write(
            f"{DIM}hb{RST} t={hrs:5.2f}h cyc={cycle} {phase:<5} "
            f"{'MOVE' if moving else 'HOLD'} loop_ok={stats.get('cycles_ok',0)} "
            f"noread={stats.get('cycles_no_reading',0)} overrun={stats.get('cycles_overrun',0)} "
            f"estop={stats.get('e_stops',0)}{tac} "
            f"{tcol}Tmax={tmax:.0f}C{RST} rdfail={self._motor_read_fails} "
            f"{acol}anom={sorted(anomalies) if anomalies else 'none'}{RST}")

    def close(self):
        if self._csv_f:
            self._csv_f.close()
        self.events.close()


def main():
    args = parse_args()
    test = StressTest(args)

    def _handle(sig, frame):
        test._stop = True
        test.events.write(f"signal {sig} received; shutting down")
    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    try:
        test.connect()
        test.run()
    except Exception as exc:
        import traceback
        test.events.write(f"{RED}FATAL: {exc!r}{RST}\n{traceback.format_exc()}")
        raise
    finally:
        test.disconnect()
        test.close()


if __name__ == "__main__":
    sys.exit(main())
