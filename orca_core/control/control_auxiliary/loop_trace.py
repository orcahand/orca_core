"""Record what the joint loop is doing, at loop rate, while something else drives.

Command-pacing tells you whether setpoints went out on time. When they did and
the hand still stutters, the cause is downstream of the command stream, and the
question becomes what the loop and the joints were doing at that moment.

:class:`LoopTracer` samples the three things that answer it, none of which cost
any bus traffic — the loop has already decoded them:

``measured``
    Encoder joint angle. A freeze is this going flat; a spasm is it jumping.

``correction``
    The PI trim, in joint degrees. Its shape is the diagnosis. A slow ramp
    toward ``correction_max_deg`` followed by a snap back is the integrator
    winding up against static friction and then releasing — a limit cycle,
    which reaches the eye as judder. Steady small values mean the controller
    is not the source.

``loop counters``
    ``cycles_overrun`` rising during a stutter means the control thread itself
    was starved, which is a host problem and not a mechanical one;
    ``cycles_no_reading`` and ``cycles_held`` implicate the encoder stream
    instead.

Motor temperature is sampled far more slowly and *does* touch the bus, so it is
opt-in: it costs a read per tick, contending with the loop's own writes.
"""

from __future__ import annotations

import csv
import logging
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np


logger = logging.getLogger(__name__)


DEFAULT_TRACE_HZ = 100.0
DEFAULT_TEMP_HZ = 1.0

# Counters copied from the loop each sample, in this column order.
COUNTER_KEYS = (
    "cycles_ok",
    "cycles_overrun",
    "cycles_no_reading",
    "cycles_held",
    "cycles_held_base",
    "cycles_paused",
    "cycles_exception",
    "cycles_clamped",
    "e_stops",
    "last_dt_s",
    "fallback_active",
)

# A correction this close to its clamp counts as saturated.
NEAR_CLAMP_FRACTION = 0.9

_INITIAL_CAPACITY = 4096


@dataclass(frozen=True)
class JointTrace:
    """One joint's behaviour over a trace."""

    joint: str
    corr_max_abs: float
    corr_p99_abs: float
    near_clamp_fraction: float
    """Share of samples with |correction| at or above 90% of the clamp."""
    angle_max_jump_deg: float
    """Largest single-sample change in measured angle — the spasm signature."""
    angle_still_longest_s: float
    """Longest run of samples with essentially no movement — the freeze signature."""
    temp_max_c: Optional[float]


@dataclass(frozen=True)
class TraceSummary:
    samples: int
    duration_s: float
    achieved_hz: float
    correction_clamp_deg: Optional[float]
    cycles_overrun: int
    cycles_exception: int
    cycles_no_reading: int
    cycles_held: int
    e_stops: int
    joints: "tuple[JointTrace, ...]"
    marks: "tuple[tuple[float, str], ...]" = ()
    """Operator observations, as ``(trace time, label)``. What the eye saw,
    on the same clock as what the instruments recorded."""


class LoopTracer:
    """Sample the running joint loop from a background thread.

    Use as a context manager around whatever is driving the hand::

        with LoopTracer(hand) as tracer:
            replay(...)
        tracer.write_csv("trace.csv")

    Sampling is best-effort: a tick that cannot read the loop (no loop, or
    after the watchdog e-stop) is skipped rather than raising, so a trace
    survives the very fault it is trying to capture.
    """

    def __init__(
        self,
        hand,
        joints: Optional[Sequence[str]] = None,
        rate_hz: float = DEFAULT_TRACE_HZ,
        temp_hz: float = 0.0,
    ):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        self._hand = hand
        self._rate_hz = float(rate_hz)
        self._temp_hz = float(temp_hz)
        self.joints: List[str] = list(
            joints if joints is not None else (getattr(hand, "loop_joint_names", None) or [])
        )
        if not self.joints:
            raise ValueError(
                "no loop-controlled joints to trace; the hand has no running "
                "joint loop, so there is nothing at loop rate to record"
            )
        self._index = {name: i for i, name in enumerate(self.joints)}
        self._motor_to_joint = {
            motor: joint
            for joint, motor in hand.config.joint_to_motor_map.items()
            if joint in self._index
        }

        width = len(self.joints)
        self._n = 0
        self._t = np.empty(_INITIAL_CAPACITY, dtype=np.float64)
        self._angles = np.full((_INITIAL_CAPACITY, width), np.nan)
        self._corr = np.full((_INITIAL_CAPACITY, width), np.nan)
        self._temps = np.full((_INITIAL_CAPACITY, width), np.nan)
        self._counters = np.full((_INITIAL_CAPACITY, len(COUNTER_KEYS)), np.nan)

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._origin = 0.0
        self._marks: List["tuple[float, str]"] = []
        self._marks_lock = threading.Lock()

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._origin = time.perf_counter()
        self._thread = threading.Thread(
            target=self._run, name="LoopTracer", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=timeout)
        self._thread = None

    def __enter__(self) -> "LoopTracer":
        self.start()
        return self

    def __exit__(self, *_exc) -> None:
        self.stop()

    def mark(self, label: str = "mark") -> float:
        """Record an operator observation at the current trace time.

        Called from whatever notices the event — a keypress at the terminal, a
        button in a UI — so a stutter the eye caught can be located in a trace
        that otherwise has no idea when anything interesting happened.
        Returns the trace time the mark landed at.
        """
        at = time.perf_counter() - self._origin
        with self._marks_lock:
            self._marks.append((at, label or "mark"))
        return at

    # -- sampling ----------------------------------------------------------

    def _grow(self) -> None:
        capacity = len(self._t) * 2
        self._t = np.resize(self._t, capacity)
        for name in ("_angles", "_corr", "_temps"):
            array = getattr(self, name)
            grown = np.full((capacity, array.shape[1]), np.nan)
            grown[: array.shape[0]] = array
            setattr(self, name, grown)
        grown = np.full((capacity, len(COUNTER_KEYS)), np.nan)
        grown[: self._counters.shape[0]] = self._counters
        self._counters = grown

    def _run(self) -> None:
        period = 1.0 / self._rate_hz
        temp_period = (1.0 / self._temp_hz) if self._temp_hz > 0 else None
        next_temp = 0.0
        while not self._stop.is_set():
            mark = time.perf_counter()
            try:
                self._sample(mark, temps=temp_period is not None and mark >= next_temp)
            except Exception:
                logger.debug("trace sample failed", exc_info=True)
            if temp_period is not None and mark >= next_temp:
                next_temp = mark + temp_period
            self._stop.wait(max(0.0, period - (time.perf_counter() - mark)))

    def _sample(self, mark: float, temps: bool) -> None:
        if self._n >= len(self._t):
            self._grow()
        row = self._n

        try:
            measured = self._hand.get_measured_joints()
            correction = self._hand.get_loop_correction()
        except RuntimeError:
            # No loop, or the watchdog e-stopped it. Nothing to record this
            # tick, but the trace keeps running so a recovery is captured.
            return

        for joint, value in measured.items():
            if joint in self._index:
                self._angles[row, self._index[joint]] = value
        for joint, value in correction.items():
            if joint in self._index:
                self._corr[row, self._index[joint]] = value

        stats = self._hand.get_loop_stats()
        for column, key in enumerate(COUNTER_KEYS):
            value = stats.get(key)
            self._counters[row, column] = (
                float(value) if isinstance(value, (int, float)) else float(bool(value))
            )

        if temps:
            try:
                for motor, celsius in self._hand.get_motor_temp(as_dict=True).items():
                    joint = self._motor_to_joint.get(motor)
                    if joint is not None:
                        self._temps[row, self._index[joint]] = celsius
            except Exception:
                logger.debug("temperature sample failed", exc_info=True)

        self._t[row] = mark - self._origin
        self._n += 1

    # -- output ------------------------------------------------------------

    def write_csv(self, path) -> Path:
        """One row per sample: time, per-joint angle, correction and
        temperature, then the loop counters."""
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        header = ["t_s", "marker"]
        header += [f"{j}_deg" for j in self.joints]
        header += [f"{j}_corr" for j in self.joints]
        header += [f"{j}_temp_c" for j in self.joints]
        header += list(COUNTER_KEYS)

        def cell(value):
            return "" if np.isnan(value) else round(float(value), 5)

        # Each mark lands on the sample nearest in time to when it was made.
        markers = [""] * self._n
        times = self._t[: self._n]
        with self._marks_lock:
            for at, label in self._marks:
                if self._n:
                    markers[int(np.argmin(np.abs(times - at)))] = label

        with path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            for row in range(self._n):
                writer.writerow(
                    [round(float(self._t[row]), 5), markers[row]]
                    + [cell(v) for v in self._angles[row]]
                    + [cell(v) for v in self._corr[row]]
                    + [cell(v) for v in self._temps[row]]
                    + [cell(v) for v in self._counters[row]]
                )
        return path

    def summary(self) -> Optional[TraceSummary]:
        """Reduce the trace to the numbers that answer "was it the loop, the
        controller, or the mechanism", or ``None`` if nothing was sampled."""
        if self._n < 2:
            return None
        times = self._t[: self._n]
        duration = float(times[-1] - times[0])
        counters = self._counters[: self._n]

        def span(key: str) -> int:
            column = counters[:, COUNTER_KEYS.index(key)]
            valid = column[~np.isnan(column)]
            return int(valid[-1] - valid[0]) if valid.size else 0

        clamp = None
        try:
            gains = self._hand.get_pid_gains()
            clamps = [g.correction_max_deg for g in gains.values()]
            clamp = float(max(clamps)) if clamps else None
        except Exception:
            logger.debug("gain read failed", exc_info=True)

        still_limit = 0.02  # degrees; below encoder noise
        traces: List[JointTrace] = []
        for joint, column in self._index.items():
            angles = self._angles[: self._n, column]
            corr = np.abs(self._corr[: self._n, column])
            corr = corr[~np.isnan(corr)]
            steps = np.abs(np.diff(angles[~np.isnan(angles)]))

            still = longest = 0
            for step in steps:
                still = still + 1 if step < still_limit else 0
                longest = max(longest, still)

            temps = self._temps[: self._n, column]
            temps = temps[~np.isnan(temps)]
            traces.append(JointTrace(
                joint=joint,
                corr_max_abs=float(corr.max()) if corr.size else 0.0,
                corr_p99_abs=float(np.percentile(corr, 99)) if corr.size else 0.0,
                near_clamp_fraction=(
                    float((corr >= NEAR_CLAMP_FRACTION * clamp).mean())
                    if corr.size and clamp else 0.0
                ),
                angle_max_jump_deg=float(steps.max()) if steps.size else 0.0,
                angle_still_longest_s=longest / self._rate_hz,
                temp_max_c=float(temps.max()) if temps.size else None,
            ))

        return TraceSummary(
            samples=self._n,
            duration_s=duration,
            achieved_hz=(self._n - 1) / duration if duration > 0 else 0.0,
            correction_clamp_deg=clamp,
            cycles_overrun=span("cycles_overrun"),
            cycles_exception=span("cycles_exception"),
            cycles_no_reading=span("cycles_no_reading"),
            cycles_held=span("cycles_held"),
            e_stops=span("e_stops"),
            joints=tuple(traces),
            marks=tuple(sorted(self._marks)),
        )
