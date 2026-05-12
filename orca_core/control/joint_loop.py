"""Host-side joint-loop thread.

Each cycle reads joint angles from a ``JointEncoderClient``, asks the
``JointController`` for a per-joint correction (degrees), maps
``target + correction`` to motor positions, and writes via
``write_desired_pos``. The motor's internal position PID tracks the motor
target on the motor encoder; this thread trims the residual offset
between motor angle and joint angle.

Encoder-freshness watchdog (the motor PID keeps holding the last
commanded position even without host updates, so the higher tiers do not
drop torque):

  > WATCHDOG_WARN_MS         rate-limited warning, control unchanged.
  > WATCHDOG_HOLD_MS         freeze the integrator; P term still runs.
  > WATCHDOG_DROP_TORQUE_MS  write the un-trimmed base motor target.
  > WATCHDOG_STOP_LOOP_MS    stop the loop, set ``fallback_active``.

A loop-period jitter monitor escalates to the stop tier on five
consecutive cycles slower than 10× target.
"""

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
from .joint_controller import JointController
from .constants import (
    DEFAULT_LOOP_HZ,
    WATCHDOG_DROP_TORQUE_MS,
    WATCHDOG_HOLD_MS,
    WATCHDOG_STOP_LOOP_MS,
    WATCHDOG_WARN_MS,
)


logger = logging.getLogger(__name__)


_FRESHNESS_WARN_INTERVAL_S = 1.0
_JITTER_WARN_RATIO = 2.0
_JITTER_ESTOP_RATIO = 10.0
_JITTER_WARN_CONSECUTIVE = 10
_JITTER_ESTOP_CONSECUTIVE = 5


JointTargets = Union[Dict[str, float], np.ndarray]


class JointLoopThread:
    """Host-side joint-loop thread.

    Calibration (encoder + motor kinematics + wrap offsets) is snapshotted
    on ``prime_for_step()``. Mid-run calibration changes require ``stop()``
    then ``start()``.
    """

    def __init__(
        self,
        orca_hand: Any,
        joint_encoder_client: Any,
        controller: JointController,
        target_hz: int = DEFAULT_LOOP_HZ,
    ):
        self._hand = orca_hand
        self._encoder_client = joint_encoder_client
        self._controller = controller
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
        self._motor_limits_lower = np.zeros(0, dtype=np.float64)
        self._joint_to_motor_ratios = np.zeros(0, dtype=np.float64)
        self._joint_inversion_mask = np.zeros(0, dtype=bool)
        self._joint_rom_lower = np.zeros(0, dtype=np.float64)
        self._joint_rom_upper = np.zeros(0, dtype=np.float64)
        self._wrap_offsets = np.zeros(0, dtype=np.float64)

        self._target_deg = np.zeros(0, dtype=np.float64)
        self._latest_measured = np.zeros(0, dtype=np.float64)
        self._last_correction = np.zeros(0, dtype=np.float64)

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
        self._last_freshness_warn_time: float = 0.0
        self._slow_cycle_streak: int = 0
        self._pathological_cycle_streak: int = 0

    def prime_for_step(self) -> None:
        """Snapshot calibration, latch the target to the measured pose for
        bumpless entry, and reset the controller."""
        self._snapshot_calibration()
        measured = self._measure_joint_angles_now()
        if measured is None:
            measured = np.zeros(len(self._joint_names), dtype=np.float64)
        with self._lock:
            self._target_deg = measured.copy()
            self._latest_measured = measured.copy()
        self._controller.reset()

    def step_once(self, dt: float) -> None:
        """One cycle: encoder read → watchdog → PI → motor-pos write."""
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

        with self._lock:
            target = self._target_deg.copy()

        if freshness_ms > WATCHDOG_DROP_TORQUE_MS:
            zero_correction = np.zeros(len(self._joint_names), dtype=np.float64)
            motor_targets = self._joint_to_motor_pos(target + zero_correction)
            self._hand._motor_client.write_desired_pos(self._motor_ids, motor_targets)
            self._stats["e_stops"] += 1
            self._stats["commands_sent"] += 1
            return

        if freshness_ms > WATCHDOG_HOLD_MS:
            self._controller.freeze_integral()
            self._stats["cycles_held"] += 1
        else:
            self._controller.unfreeze_integral()

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

        correction = self._controller.step(target, measured, dt)
        motor_targets = self._joint_to_motor_pos(target + correction)
        self._hand._motor_client.write_desired_pos(self._motor_ids, motor_targets)

        with self._lock:
            self._latest_measured = measured.copy()
            self._last_correction = correction.copy()

        self._stats["cycles_ok"] += 1
        self._stats["commands_sent"] += 1

    def set_target(self, joint_targets: JointTargets) -> None:
        """Update the joint setpoint. Accepts a ``{joint_name: angle_deg}``
        dict or a ``(num_joints,)`` array indexed in the snapshotted joint
        order."""
        if isinstance(joint_targets, dict):
            unknown = set(joint_targets) - set(self._joint_names)
            if unknown:
                raise ValueError(f"unknown joints in target: {sorted(unknown)}")
            new_target = self._target_deg.copy()
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
            self._target_deg = new_target

    def get_measured_joints(self) -> Dict[str, float]:
        with self._lock:
            measured = self._latest_measured.copy()
        return {name: float(measured[i]) for i, name in enumerate(self._joint_names)}

    def get_correction(self) -> Dict[str, float]:
        with self._lock:
            correction = self._last_correction.copy()
        return {name: float(correction[i]) for i, name in enumerate(self._joint_names)}

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
        """Freeze the joint set and the kinematics arrays for one loop run.
        Wrist is excluded; joints without an encoder calibration entry are
        skipped.

        Raises:
            RuntimeError: no encoder-backed joints to control.
            ValueError: controller size disagrees with the resolved set.
        """
        encoder_dict = self._hand.calibration.joint_encoder_calibration_dict
        joint_to_motor = self._hand.config.joint_to_motor_map
        inversion = self._hand.config.joint_inversion_dict
        joint_roms = self._hand.config.joint_roms_dict
        motor_limits = self._hand.motor_limits_dict
        ratios = self._hand.calibration.joint_to_motor_ratios_dict

        joints = [
            j for j in JOINT_TO_ENCODER_SLOT
            if j != WRIST and j in joint_to_motor and j in encoder_dict
        ]
        if not joints:
            raise RuntimeError("no encoder-backed joints to control")
        if self._controller.num_joints != len(joints):
            raise ValueError(
                f"controller was constructed with num_joints={self._controller.num_joints} "
                f"but {len(joints)} encoder-backed joints are available"
            )

        if self._hand._wrap_offsets_dict is None:
            self._hand._compute_wrap_offsets_dict()

        self._joint_names = joints
        self._slots = np.array([JOINT_TO_ENCODER_SLOT[j] for j in joints], dtype=np.int64)
        self._anchors = np.array(
            [encoder_dict[j].enc_at_anchor_count for j in joints], dtype=np.int64
        )
        self._enc_polarities = np.array(
            [JOINT_ENCODER_POLARITY[j] for j in joints], dtype=np.int64
        )
        self._anchor_angles = np.array(
            [joint_roms[j][1] for j in joints], dtype=np.float64
        )
        self._motor_ids = [joint_to_motor[j] for j in joints]
        self._motor_limits_lower = np.array(
            [motor_limits[mid][0] for mid in self._motor_ids], dtype=np.float64
        )
        self._joint_to_motor_ratios = np.array(
            [ratios[mid] for mid in self._motor_ids], dtype=np.float64
        )
        self._joint_inversion_mask = np.array(
            [bool(inversion.get(j, False)) for j in joints], dtype=bool
        )
        self._joint_rom_lower = np.array(
            [joint_roms[j][0] for j in joints], dtype=np.float64
        )
        self._joint_rom_upper = np.array(
            [joint_roms[j][1] for j in joints], dtype=np.float64
        )
        self._wrap_offsets = np.array(
            [self._hand._wrap_offsets_dict.get(mid, 0.0) for mid in self._motor_ids],
            dtype=np.float64,
        )
        self._target_deg = np.zeros(len(joints), dtype=np.float64)
        self._latest_measured = np.zeros(len(joints), dtype=np.float64)
        self._last_correction = np.zeros(len(joints), dtype=np.float64)

    def _joint_to_motor_pos(self, joint_command_deg: np.ndarray) -> np.ndarray:
        """Vectorised joint→motor mapping for the snapshotted joint set.

        Joint angles are degrees; motor positions are radians (the
        ``joint_to_motor_ratios`` is motor-rad per joint-deg). See
        :meth:`OrcaHand._joint_to_motor_pos` for the equivalent scalar
        implementation.
        """
        inverted_term = (
            self._motor_limits_lower
            + (self._joint_rom_upper - joint_command_deg) * self._joint_to_motor_ratios
        )
        forward_term = (
            self._motor_limits_lower
            + (joint_command_deg - self._joint_rom_lower) * self._joint_to_motor_ratios
        )
        motor_pos = np.where(self._joint_inversion_mask, inverted_term, forward_term)
        return motor_pos + self._wrap_offsets

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
        """Set ``fallback_active`` and signal the thread to stop. The motor
        PID holds the last commanded position; nothing else is touched."""
        if self._stats["fallback_active"]:
            return
        logger.error("joint loop e-stop: %s", reason)
        self._stats["fallback_active"] = True
        self._stats["e_stops"] += 1
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
        """Update jitter streak counters; e-stop on a sustained run of
        pathological cycles. A single transient >10× cycle is tolerated."""
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
                # Keep the thread alive on transient bus errors; the cycle
                # is counted as overrun so a sick loop surfaces in stats.
                self._stats["cycles_overrun"] += 1
            self._record_loop_period(dt)
            prev_time = now
            next_deadline += period
