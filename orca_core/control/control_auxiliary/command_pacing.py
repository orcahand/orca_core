"""Measure whether a host-generated trajectory actually keeps its cadence.

Under position control a gap in the command stream is indistinguishable from a
gap on the wire: the motors hold their last target either way, so the hand
stops for as long as the gap lasts and then jumps. Diagnosing a stutter
therefore starts by finding out whether the commands were issued on time at
all, before anything about the bus is worth investigating.

:class:`PacingMonitor` wraps each command and records two numbers:

``interval``
    Start-to-start time between consecutive commands. This is the cadence the
    motors actually see.

``call``
    How long the command call itself took. Everything else in an interval —
    sleeping, interpreter overhead, garbage collection, waiting on the GIL —
    is the difference between the two.

That split is the diagnosis. A long interval made of a long *call* means the
command blocked, which points at the bus or the driver. A long interval whose
call was short means the host simply did not get around to issuing it, which
points at scheduling, GC, or thread contention — and no amount of work on the
wire will help.
"""

from __future__ import annotations

import csv
import logging
import statistics as st
import time
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence


logger = logging.getLogger(__name__)


# An interval this many times the target period counts as late but survivable;
# past the stall multiple it is long enough to see as a pause in the motion.
LATE_MULTIPLE = 1.5
STALL_MULTIPLE = 3.0


@dataclass(frozen=True)
class PacingStats:
    """What a run's command cadence looked like."""

    commands: int
    target_period_s: float
    interval_median_s: float
    interval_p95_s: float
    interval_p99_s: float
    interval_max_s: float
    call_median_s: float
    call_p95_s: float
    call_max_s: float
    late: int
    stalls: int
    worst_stall_call_s: Optional[float]
    """Duration of the command call that opened the worst interval — the whole
    of it means the call blocked, a sliver of it means the host was elsewhere."""
    loop_cycles_overrun: Optional[int]
    loop_cycles_exception: Optional[int]
    verdict: str


def _percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]


class PacingMonitor:
    """Times each command in a host-generated trajectory.

    Wrap every command site::

        monitor = PacingMonitor(target_period_s=0.02, hand=hand)
        for pose in trajectory:
            with monitor.command():
                hand.set_joint_positions(pose)

    ``hand`` is optional; when it exposes ``get_loop_stats`` the joint loop's
    overrun counters are sampled at both ends, which says whether the control
    thread was starved at the same moments — shared starvation implicates the
    host rather than anything the commands did.
    """

    def __init__(self, target_period_s: float, hand=None):
        if target_period_s <= 0:
            raise ValueError("target_period_s must be positive")
        self.target_period_s = float(target_period_s)
        self._hand = hand
        self._starts: List[float] = []
        self._intervals: List[float] = []
        self._calls: List[float] = []
        self._loop_before = self._loop_stats()

    def _loop_stats(self) -> Optional[dict]:
        getter = getattr(self._hand, "get_loop_stats", None)
        if getter is None:
            return None
        try:
            return dict(getter())
        except Exception:
            logger.debug("loop stats unavailable", exc_info=True)
            return None

    @contextmanager
    def command(self):
        """Time one command. Records the interval since the previous one and
        how long this one took, even if it raises."""
        started = time.perf_counter()
        if self._starts:
            self._intervals.append(started - self._starts[-1])
        self._starts.append(started)
        try:
            yield
        finally:
            self._calls.append(time.perf_counter() - started)

    def stats(self) -> Optional[PacingStats]:
        """Summarise the run, or ``None`` before two commands have been timed."""
        if len(self._intervals) < 1:
            return None
        late_limit = self.target_period_s * LATE_MULTIPLE
        stall_limit = self.target_period_s * STALL_MULTIPLE
        stalls = [i for i in self._intervals if i > stall_limit]

        worst_call = None
        if stalls:
            worst_index = self._intervals.index(max(stalls))
            # Interval n spans command n to command n+1, so the call that
            # opened it is the one at the same index.
            if worst_index < len(self._calls):
                worst_call = self._calls[worst_index]

        after = self._loop_stats()
        overrun = exceptions = None
        if self._loop_before is not None and after is not None:
            overrun = int(
                after.get("cycles_overrun", 0) - self._loop_before.get("cycles_overrun", 0)
            )
            exceptions = int(
                after.get("cycles_exception", 0)
                - self._loop_before.get("cycles_exception", 0)
            )

        return PacingStats(
            commands=len(self._starts),
            target_period_s=self.target_period_s,
            interval_median_s=st.median(self._intervals),
            interval_p95_s=_percentile(self._intervals, 0.95),
            interval_p99_s=_percentile(self._intervals, 0.99),
            interval_max_s=max(self._intervals),
            call_median_s=st.median(self._calls) if self._calls else 0.0,
            call_p95_s=_percentile(self._calls, 0.95) if self._calls else 0.0,
            call_max_s=max(self._calls) if self._calls else 0.0,
            late=sum(1 for i in self._intervals if i > late_limit),
            stalls=len(stalls),
            worst_stall_call_s=worst_call,
            loop_cycles_overrun=overrun,
            loop_cycles_exception=exceptions,
            verdict=_verdict(
                stalls=len(stalls),
                worst_interval=max(self._intervals),
                worst_call=worst_call,
            ),
        )

    def write_csv(self, path) -> Path:
        """Write one row per command: when it went out, the gap before it, and
        how long it took. Enough to plot the cadence and find the outliers."""
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        origin = self._starts[0] if self._starts else 0.0
        with path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                ["index", "t_s", "interval_s", "call_s", "target_period_s"]
            )
            for index, start in enumerate(self._starts):
                interval = self._intervals[index - 1] if index else ""
                call = self._calls[index] if index < len(self._calls) else ""
                writer.writerow([
                    index,
                    round(start - origin, 6),
                    "" if interval == "" else round(interval, 6),
                    "" if call == "" else round(call, 6),
                    self.target_period_s,
                ])
        return path


def _verdict(stalls: int, worst_interval: float, worst_call: Optional[float]) -> str:
    """One line on where the gaps came from, by the rule in the module
    docstring: a stall made mostly of the command call blocked on something,
    otherwise the host never issued it on time."""
    if stalls == 0:
        return "cadence held — no stall long enough to see"
    if worst_call is None:
        return f"{stalls} stall(s); no call timing to attribute them to"
    if worst_call > 0.5 * worst_interval:
        return (
            f"{stalls} stall(s), and the worst one is mostly the command call "
            f"({worst_call * 1000:.0f} of {worst_interval * 1000:.0f} ms) — "
            "the call blocked, look at the bus or the driver"
        )
    return (
        f"{stalls} stall(s), and the worst one is mostly between calls "
        f"(call only {worst_call * 1000:.0f} of {worst_interval * 1000:.0f} ms) — "
        "the host did not issue on time; scheduling, GC or GIL, not the wire"
    )
