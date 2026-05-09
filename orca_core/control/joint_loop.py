"""Host-side closed-loop joint control thread.

Reads joint angles from a ``JointEncoderClient``, runs each encoder-backed
joint through a ``JointPIDController``, and writes signed Goal_Current to
the matching motors via ``MotorClient.sync_write_current``.

The watchdog escalates on encoder freshness:

  > 10 ms   → rate-limited WARNING; control unchanged.
  > 50 ms   → freeze the integrator; PID still runs (P + D + frozen I).
  > 200 ms  → write zero current to every encoder-backed motor each cycle.
  > 1000 ms → stop the loop, restore the operating modes captured at start,
              and set ``fallback_active = True``.

A loop-period jitter monitor runs in parallel: ten consecutive cycles
slower than 2× target log a WARNING; one cycle slower than 10× target
triggers the > 1000 ms tier.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, List, Optional, Union

import numpy as np

from ..constants import WRIST
from ..hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    JOINT_ENCODER_POLARITY,
    JOINT_TO_ENCODER_SLOT,
)
from ..hardware.sensing.encoder_protocol import encoder_to_joint_angle
from .constants import (
    DEFAULT_LOOP_HZ,
    WATCHDOG_DROP_TORQUE_MS,
    WATCHDOG_HOLD_MS,
    WATCHDOG_STOP_LOOP_MS,
    WATCHDOG_WARN_MS,
)
from .joint_pid import JointPIDController


logger = logging.getLogger(__name__)


_FRESHNESS_WARN_INTERVAL_S = 1.0
_JITTER_WARN_RATIO = 2.0
_JITTER_ESTOP_RATIO = 10.0
_JITTER_WARN_CONSECUTIVE = 10
_JITTER_ESTOP_CONSECUTIVE = 5


JointTargets = Union[Dict[str, float], np.ndarray]


class JointLoopThread:
    """Background thread that drives the host-side joint PID loop.

    Calibration and prior operating modes are snapshotted on
    ``prime_for_step()`` (called by ``start()``). Mid-run calibration
    changes require ``stop()`` then ``start()``.
    """

    def __init__(
        self,
        orca_hand: Any,
        joint_encoder_client: Any,
        joint_pid: JointPIDController,
        target_hz: int = DEFAULT_LOOP_HZ,
    ):
        self._hand = orca_hand
        self._encoder_client = joint_encoder_client
        self._pid = joint_pid
        self._target_hz = int(target_hz)
        self._target_period = 1.0 / float(self._target_hz)

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        self._joint_names: List[str] = []
        self._slots = np.zeros(0, dtype=np.int64)
        self._anchors = np.zeros(0, dtype=np.int64)
        self._enc_polarities = np.zeros(0, dtype=np.int64)
        self._anchor_angles = np.zeros(0, dtype=np.float64)
        self._motor_ids: List[int] = []
        self._motor_polarity_signs = np.zeros(0, dtype=np.float64)

        self._target_rad = np.zeros(0, dtype=np.float64)
        self._latest_measured = np.zeros(0, dtype=np.float64)

        self._stats = {
            "cycles_ok": 0,
            "cycles_overrun": 0,
            "cycles_no_reading": 0,
            "cycles_held": 0,
            "commands_sent": 0,
            "e_stops": 0,
            "last_dt_s": float("nan"),
            "fallback_active": False,
        }
        self._prior_modes: Dict[int, int] = {}
        self._last_freshness_warn_time: float = 0.0
        self._slow_cycle_streak: int = 0
        self._pathological_cycle_streak: int = 0

    def prime_for_step(self) -> None:
        """Snapshot calibration, latch the PID target to the measured pose
        (bumpless), and reset the integrator. ``start()`` calls this; tests
        driving ``step_once`` synchronously call it too.
        """
        self._snapshot_calibration()
        measured = self._measure_joint_angles_now()
        if measured is None:
            measured = np.zeros(len(self._joint_names), dtype=np.float64)
        with self._lock:
            self._target_rad = measured.copy()
            self._latest_measured = measured.copy()
        self._pid.set_target(self._target_rad)
        self._pid.reset()

    def step_once(self, dt: float) -> None:
        """Run one cycle: encoder read → watchdog → PID → motor write.

        Watchdog tiers run before the PID step. ``dt`` is clamped inside
        the PID; cycles with no encoder reading skip the motor write.
        """
        if self._stats["fallback_active"]:
            return

        self._stats["last_dt_s"] = float(dt)

        reading = self._encoder_client.get_latest_encoder_reading()
        if reading is None:
            self._stats["cycles_no_reading"] += 1
            return

        freshness_ms = float(reading.freshness_ms)

        if freshness_ms > WATCHDOG_STOP_LOOP_MS:
            self._trigger_estop(
                f"encoder freshness {freshness_ms:.0f} ms exceeds stop threshold"
            )
            return

        if freshness_ms > WATCHDOG_DROP_TORQUE_MS:
            zeros = np.zeros(len(self._motor_ids), dtype=np.float64)
            self._hand._motor_client.sync_write_current(self._motor_ids, zeros)
            self._stats["e_stops"] += 1
            self._stats["commands_sent"] += 1
            return

        if freshness_ms > WATCHDOG_HOLD_MS:
            self._pid.freeze_integral()
            self._stats["cycles_held"] += 1
        else:
            self._pid.unfreeze_integral()

        if freshness_ms > WATCHDOG_WARN_MS:
            self._maybe_log_freshness_warning(freshness_ms)

        raw_counts = np.asarray(reading.raw_counts)
        if raw_counts.shape != (AUTO_ENC_NUM_JOINTS,):
            raise RuntimeError(
                f"encoder reading has shape {raw_counts.shape}, "
                f"expected ({AUTO_ENC_NUM_JOINTS},)"
            )
        measured = encoder_to_joint_angle(
            raw_counts[self._slots],
            self._anchors,
            self._enc_polarities,
            self._anchor_angles,
        )

        with self._lock:
            target = self._target_rad.copy()
            self._latest_measured = measured.copy()
        self._pid.set_target(target)
        currents_mA = self._pid.step(measured, dt) * self._motor_polarity_signs

        self._hand._motor_client.sync_write_current(self._motor_ids, currents_mA)

        self._stats["cycles_ok"] += 1
        self._stats["commands_sent"] += 1

    def set_target(self, joint_targets: JointTargets) -> None:
        """Update the PID setpoint. Accepts a ``{joint_name: angle_rad}``
        dict or a ``(num_joints,)`` array indexed in the snapshotted joint
        order (the same order as ``get_measured_joints``).
        """
        if isinstance(joint_targets, dict):
            unknown = set(joint_targets) - set(self._joint_names)
            if unknown:
                raise ValueError(f"unknown joints in target: {sorted(unknown)}")
            new_target = self._target_rad.copy()
            for joint, value in joint_targets.items():
                new_target[self._joint_names.index(joint)] = float(value)
        else:
            array = np.asarray(joint_targets, dtype=np.float64)
            if array.shape != (len(self._joint_names),):
                raise ValueError(
                    f"target array must have shape ({len(self._joint_names)},), "
                    f"got {array.shape}"
                )
            new_target = array.copy()
        with self._lock:
            self._target_rad = new_target

    def get_measured_joints(self) -> Dict[str, float]:
        with self._lock:
            measured = self._latest_measured.copy()
        return {name: float(measured[i]) for i, name in enumerate(self._joint_names)}

    def get_stats(self) -> Dict[str, float]:
        return dict(self._stats)

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self.prime_for_step()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_loop, name="JointLoopThread", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 1.0) -> bool:
        if self._thread is None:
            return True
        self._stop_event.set()
        self._thread.join(timeout=timeout)
        clean = not self._thread.is_alive()
        self._thread = None
        return clean

    def _snapshot_calibration(self) -> None:
        """Freeze the joint set and calibration arrays for one loop run.
        Wrist is excluded; joints without a complete encoder calibration
        entry are skipped.

        Raises:
            RuntimeError: no encoder-backed joints to control.
            ValueError: PID size disagrees with the resolved joint set.
        """
        encoder_dict = self._hand.calibration.joint_encoder_calibration_dict
        joint_to_motor = self._hand.config.joint_to_motor_map
        inversion = self._hand.config.joint_inversion_dict

        joints = [
            j for j in JOINT_TO_ENCODER_SLOT
            if j != WRIST and j in joint_to_motor and j in encoder_dict
        ]
        if not joints:
            raise RuntimeError("no encoder-backed joints to control")
        if self._pid.num_joints != len(joints):
            raise ValueError(
                f"PID was constructed with num_joints={self._pid.num_joints} "
                f"but {len(joints)} encoder-backed joints are available"
            )

        self._joint_names = joints
        self._slots = np.array([JOINT_TO_ENCODER_SLOT[j] for j in joints], dtype=np.int64)
        self._anchors = np.array(
            [encoder_dict[j].enc_at_anchor_count for j in joints], dtype=np.int64
        )
        self._enc_polarities = np.array(
            [JOINT_ENCODER_POLARITY[j] for j in joints], dtype=np.int64
        )
        self._anchor_angles = np.array(
            [encoder_dict[j].anchor_angle_rad for j in joints], dtype=np.float64
        )
        self._motor_ids = [joint_to_motor[j] for j in joints]
        self._motor_polarity_signs = np.array(
            [-1.0 if inversion.get(j, False) else 1.0 for j in joints],
            dtype=np.float64,
        )
        self._target_rad = np.zeros(len(joints), dtype=np.float64)
        self._latest_measured = np.zeros(len(joints), dtype=np.float64)

        self._prior_modes = self._hand._motor_client.read_operating_modes(
            self._motor_ids
        )

    def _measure_joint_angles_now(self) -> Optional[np.ndarray]:
        reading = self._encoder_client.get_latest_encoder_reading()
        if reading is None:
            return None
        raw_counts = np.asarray(reading.raw_counts)
        if raw_counts.shape != (AUTO_ENC_NUM_JOINTS,):
            raise RuntimeError(
                f"encoder reading has shape {raw_counts.shape}, "
                f"expected ({AUTO_ENC_NUM_JOINTS},)"
            )
        return encoder_to_joint_angle(
            raw_counts[self._slots],
            self._anchors,
            self._enc_polarities,
            self._anchor_angles,
        )

    def _trigger_estop(self, reason: str) -> None:
        if self._stats["fallback_active"]:
            return
        logger.error("joint loop e-stop: %s", reason)
        self._stats["fallback_active"] = True
        self._stats["e_stops"] += 1
        if self._motor_ids and self._prior_modes:
            try:
                modes = [self._prior_modes[mid] for mid in self._motor_ids]
                self._hand._motor_client.set_operating_mode_per_motor(
                    self._motor_ids, modes
                )
            except Exception:
                logger.exception("failed to restore operating modes after e-stop")
        self._stop_event.set()

    def _maybe_log_freshness_warning(self, freshness_ms: float) -> None:
        now = time.monotonic()
        if now - self._last_freshness_warn_time >= _FRESHNESS_WARN_INTERVAL_S:
            logger.warning(
                "encoder freshness %.0f ms exceeds %d ms warn threshold",
                freshness_ms,
                WATCHDOG_WARN_MS,
            )
            self._last_freshness_warn_time = now

    def _record_loop_period(self, period_s: float) -> None:
        """Update the jitter streak counters and escalate if needed.

        A single transient >10× cycle (USB stall, GC pause, OS scheduling
        hiccup) is tolerated; only a sustained run of pathological cycles
        e-stops. The encoder-freshness watchdog catches genuinely wedged
        loops first via tier-3 / tier-4.
        """
        is_pathological = period_s >= self._target_period * _JITTER_ESTOP_RATIO
        is_slow = period_s > self._target_period * _JITTER_WARN_RATIO

        if is_pathological:
            self._pathological_cycle_streak += 1
        else:
            self._pathological_cycle_streak = 0

        if is_slow:
            self._slow_cycle_streak += 1
        else:
            self._slow_cycle_streak = 0

        if self._pathological_cycle_streak >= _JITTER_ESTOP_CONSECUTIVE:
            self._trigger_estop(
                f"loop period exceeded {_JITTER_ESTOP_RATIO}× target for "
                f"{_JITTER_ESTOP_CONSECUTIVE} consecutive cycles "
                f"(last={period_s * 1000:.1f} ms)"
            )
            self._pathological_cycle_streak = 0
            self._slow_cycle_streak = 0
            return

        if self._slow_cycle_streak == _JITTER_WARN_CONSECUTIVE:
            logger.warning(
                "loop jitter: %d consecutive cycles slower than %.1f× target",
                _JITTER_WARN_CONSECUTIVE,
                _JITTER_WARN_RATIO,
            )

    def _run_loop(self) -> None:
        period = self._target_period
        next_deadline = time.monotonic()
        prev_time: Optional[float] = None
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now < next_deadline:
                time.sleep(min(next_deadline - now, period))
                continue
            if now > next_deadline + period:
                self._stats["cycles_overrun"] += 1
            dt = (now - prev_time) if prev_time is not None else period
            try:
                self.step_once(dt=dt)
            except Exception:
                # Keep the thread alive on transient errors so a bus glitch
                # doesn't silently kill the loop. Counted as overrun so a
                # sick loop is visible via stats.
                self._stats["cycles_overrun"] += 1
            self._record_loop_period(dt)
            prev_time = now
            next_deadline += period
