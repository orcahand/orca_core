"""Summary statistics for a waypoint-test run.

Reduces a run's :class:`~.waypoint_runner.WaypointRecord` list to one row per
driven joint, so runs are compared by concatenating stats files rather than
re-deriving them from the raw data every time.

Three things the per-joint row carries beyond the obvious averages:

``hysteresis_gap``
    Mean residual when flexing minus mean residual when extending. Negative
    means the joint falls short whichever way it travels — friction. Positive
    means it runs past in both directions, which friction cannot cause; that
    is momentum carrying it through the target.

``creep_mean``
    How far the joint kept moving during the post-settle dwell. Large values
    say the motion threshold fired mid-approach and the residuals are
    snapshots of an unfinished move, not steady-state error.

``fit_target_pct`` / ``fit_move_pct``
    Least-squares split of the residual into a part proportional to the target
    angle (a scale error in the joint↔motor map) and a part proportional to the
    commanded displacement (friction lag when negative, overshoot when
    positive), with ``fit_r2`` saying how much of the error those two explain.
    A low ``fit_r2`` alongside a large ``fit_resid_sd`` means the error is
    mostly random — the mechanism's repeatability floor, which no gain change
    will move.
"""

from __future__ import annotations

import csv
import logging
import statistics as st
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np

from .waypoint_runner import WaypointRecord


logger = logging.getLogger(__name__)


# Waypoints whose commanded displacement is smaller than this carry no usable
# direction, so they are left out of the flex/extend split.
MIN_DIRECTIONAL_MOVE_DEG = 5.0

# Least-squares fit needs more points than coefficients to say anything.
MIN_FIT_SAMPLES = 10


@dataclass(frozen=True)
class JointStats:
    """One driven joint's behaviour over a run."""

    joint: str
    n: int
    err_mean: float
    err_sd: float
    abs_err_median: float
    abs_err_p95: float
    abs_err_max: float
    creep_mean: float
    abs_creep_p95: float
    abs_settle_err_median: float
    """Median |error| at the settle instant: overshoot before any recovery."""
    settle_hysteresis_gap: Optional[float]
    """Flex minus extend at the settle instant. A large positive value is the
    momentum signature; a ramp that removes momentum should drive it to zero."""
    checkpoint_times_s: "tuple[float, ...]"
    abs_err_at_checkpoint: "tuple[Optional[float], ...]"
    """Median |error| at each dwell checkpoint: the convergence curve."""
    flex_n: int
    flex_err_mean: Optional[float]
    extend_n: int
    extend_err_mean: Optional[float]
    hysteresis_gap: Optional[float]
    fit_target_pct: Optional[float]
    fit_move_pct: Optional[float]
    fit_const: Optional[float]
    fit_r2: Optional[float]
    fit_resid_sd: Optional[float]


@dataclass(frozen=True)
class RunStats:
    """A whole run: outcome counts and timing, plus one entry per joint."""

    run_id: str
    finger: str
    source: str
    angle_source: str
    waypoints: int
    settled: int
    stalled: int
    timed_out: int
    stopped: int
    duration_mean_s: float
    duration_median_s: float
    duration_min_s: float
    duration_max_s: float
    time_to_move_mean_s: Optional[float]
    time_to_move_median_s: Optional[float]
    time_in_tolerance_median_s: Optional[float]
    time_in_tolerance_n: int
    dwell_travel_median_deg: Optional[float]
    dwell_travel_p95_deg: Optional[float]
    joints: "tuple[JointStats, ...]"


STATS_FIELDS = (
    "run_id",
    "finger",
    "source",
    "angle_source",
    "waypoints",
    "settled",
    "stalled",
    "timed_out",
    "stopped",
    "duration_mean_s",
    "duration_median_s",
    "duration_min_s",
    "duration_max_s",
    "time_to_move_mean_s",
    "time_to_move_median_s",
    "time_in_tolerance_median_s",
    "time_in_tolerance_n",
    "dwell_travel_median_deg",
    "dwell_travel_p95_deg",
    "joint",
    "n",
    "err_mean",
    "err_sd",
    "abs_err_median",
    "abs_err_p95",
    "abs_err_max",
    "creep_mean",
    "abs_creep_p95",
    "abs_settle_err_median",
    "settle_hysteresis_gap",
    "t1_s",
    "abs_err_t1",
    "t2_s",
    "abs_err_t2",
    "t3_s",
    "abs_err_t3",
    "flex_n",
    "flex_err_mean",
    "extend_n",
    "extend_err_mean",
    "hysteresis_gap",
    "fit_target_pct",
    "fit_move_pct",
    "fit_const",
    "fit_r2",
    "fit_resid_sd",
    "motion_threshold_deg",
    "settle_window_s",
    "arrival_tolerance_deg",
    "in_tolerance_deg",
    "settle_dwell_s",
    "ramp_speed_deg_s",
    "ramp_steps",
    "ramp_time_s",
    "ramp_accel_deg_s2",
    "ramp_per_joint",
)


def _percentile(values: Sequence[float], fraction: float) -> float:
    """Nearest-rank percentile of an already-sortable sequence."""
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(fraction * len(ordered)))
    return ordered[index]


def _mean_or_none(values: Sequence[float]) -> Optional[float]:
    return st.mean(values) if values else None


def _fit_error(
    targets: Sequence[float],
    moves: Sequence[float],
    errors: Sequence[float],
) -> "tuple[Optional[float], Optional[float], Optional[float], Optional[float], Optional[float]]":
    """Least-squares ``error = a·target + b·move + c``.

    Returns ``(a, b, c, r2, residual_sd)``, all ``None`` when there are too few
    points or the design matrix is degenerate (e.g. every target identical).
    """
    if len(errors) < MIN_FIT_SAMPLES:
        return None, None, None, None, None
    design = np.column_stack([targets, moves, np.ones(len(errors))])
    observed = np.asarray(errors, dtype=float)
    try:
        coefficients, *_ = np.linalg.lstsq(design, observed, rcond=None)
    except np.linalg.LinAlgError:
        logger.debug("residual fit failed", exc_info=True)
        return None, None, None, None, None
    predicted = design @ coefficients
    residual = observed - predicted
    spread = float(((observed - observed.mean()) ** 2).sum())
    r2 = float(1.0 - (residual**2).sum() / spread) if spread > 0 else None
    return (
        float(coefficients[0]),
        float(coefficients[1]),
        float(coefficients[2]),
        r2,
        float(np.std(residual)),
    )


def summarize_run(
    records: Sequence[WaypointRecord],
    *,
    run_id: str,
) -> Optional[RunStats]:
    """Reduce one run's records to its statistics, or ``None`` if it is empty.

    Only waypoints that produced a per-joint outcome contribute; a run that
    stopped before any joint was recorded yields ``None``.
    """
    if not records:
        return None

    outcomes = Counter(record.outcome for record in records)
    durations = [record.duration_s for record in records]
    latencies = [
        record.time_to_move_s
        for record in records
        if record.time_to_move_s is not None
    ]
    in_tol = [
        record.time_in_tolerance_s
        for record in records
        if record.time_in_tolerance_s is not None
    ]
    dwell = [
        record.dwell_travel_deg
        for record in records
        if record.dwell_travel_deg is not None
    ]

    joint_names: List[str] = []
    for record in records:
        for outcome in record.joints:
            if outcome.joint not in joint_names:
                joint_names.append(outcome.joint)

    joint_stats: List[JointStats] = []
    for joint in joint_names:
        samples = [o for record in records for o in record.joints if o.joint == joint]
        if not samples:
            continue
        errors = [o.error_deg for o in samples]
        creeps = [o.final_deg - o.settle_deg for o in samples]
        moves = [o.target_deg - o.start_deg for o in samples]
        targets = [o.target_deg for o in samples]

        settle_errors = [o.settle_deg - o.target_deg for o in samples]
        settle_flex = [
            e for e, m in zip(settle_errors, moves) if m > MIN_DIRECTIONAL_MOVE_DEG
        ]
        settle_extend = [
            e for e, m in zip(settle_errors, moves) if m < -MIN_DIRECTIONAL_MOVE_DEG
        ]

        flexing = [e for e, m in zip(errors, moves) if m > MIN_DIRECTIONAL_MOVE_DEG]
        extending = [e for e, m in zip(errors, moves) if m < -MIN_DIRECTIONAL_MOVE_DEG]
        flex_mean = _mean_or_none(flexing)
        extend_mean = _mean_or_none(extending)

        times = records[0].checkpoint_times_s if records else ()
        curve: List[Optional[float]] = []
        for slot in range(len(times)):
            at_slot = [
                abs(o.checkpoint_deg[slot] - o.target_deg)
                for o in samples
                if slot < len(o.checkpoint_deg) and o.checkpoint_deg[slot] is not None
            ]
            curve.append(st.median(at_slot) if at_slot else None)

        a, b, c, r2, resid_sd = _fit_error(targets, moves, errors)
        joint_stats.append(
            JointStats(
                joint=joint,
                n=len(samples),
                err_mean=st.mean(errors),
                err_sd=st.pstdev(errors) if len(errors) > 1 else 0.0,
                abs_err_median=st.median([abs(e) for e in errors]),
                abs_err_p95=_percentile([abs(e) for e in errors], 0.95),
                abs_err_max=max(abs(e) for e in errors),
                creep_mean=st.mean(creeps),
                abs_creep_p95=_percentile([abs(x) for x in creeps], 0.95),
                abs_settle_err_median=st.median([abs(e) for e in settle_errors]),
                settle_hysteresis_gap=(
                    None if not settle_flex or not settle_extend
                    else st.mean(settle_flex) - st.mean(settle_extend)
                ),
                checkpoint_times_s=tuple(times),
                abs_err_at_checkpoint=tuple(curve),
                flex_n=len(flexing),
                flex_err_mean=flex_mean,
                extend_n=len(extending),
                extend_err_mean=extend_mean,
                hysteresis_gap=(
                    None if flex_mean is None or extend_mean is None
                    else flex_mean - extend_mean
                ),
                # Reported as percent: a degree of error per degree of target
                # or of commanded travel.
                fit_target_pct=None if a is None else a * 100.0,
                fit_move_pct=None if b is None else b * 100.0,
                fit_const=c,
                fit_r2=r2,
                fit_resid_sd=resid_sd,
            )
        )

    first = records[0]
    return RunStats(
        run_id=run_id,
        finger=first.finger,
        source=first.source,
        angle_source=first.angle_source,
        waypoints=len(records),
        settled=outcomes.get("settled", 0),
        stalled=outcomes.get("stalled", 0),
        timed_out=outcomes.get("timeout", 0),
        stopped=outcomes.get("stopped", 0),
        duration_mean_s=st.mean(durations),
        duration_median_s=st.median(durations),
        duration_min_s=min(durations),
        duration_max_s=max(durations),
        time_to_move_mean_s=_mean_or_none(latencies),
        time_to_move_median_s=st.median(latencies) if latencies else None,
        time_in_tolerance_median_s=st.median(in_tol) if in_tol else None,
        time_in_tolerance_n=len(in_tol),
        dwell_travel_median_deg=st.median(dwell) if dwell else None,
        dwell_travel_p95_deg=_percentile(dwell, 0.95) if dwell else None,
        joints=tuple(joint_stats),
    )


def _checkpoint_cells(joint: JointStats) -> dict:
    """Flatten the convergence curve into the fixed ``t{n}_s`` / ``abs_err_t{n}``
    column pairs."""
    cells = {}
    for slot in range(3):
        at = (
            joint.checkpoint_times_s[slot]
            if slot < len(joint.checkpoint_times_s) else None
        )
        value = (
            joint.abs_err_at_checkpoint[slot]
            if slot < len(joint.abs_err_at_checkpoint) else None
        )
        cells[f"t{slot + 1}_s"] = "" if at is None else at
        cells[f"abs_err_t{slot + 1}"] = "" if value is None else round(value, 4)
    return cells


def _cell(value, digits: int = 4):
    return "" if value is None else round(value, digits)


def write_stats_csv(
    path,
    stats: RunStats,
    *,
    motion_threshold_deg: float,
    settle_window_s: float,
    arrival_tolerance_deg: float,
    in_tolerance_deg: float,
    settle_dwell_s: float,
    ramp_speed_deg_s: float = 0.0,
    ramp_steps: int = 1,
    ramp_time_s: float = 0.0,
    ramp_accel_deg_s2: float = 0.0,
    ramp_per_joint: bool = False,
    append: bool = True,
) -> Path:
    """Write ``stats`` as one row per joint, run-level fields repeated.

    Appends by default so successive runs — a controller variant per run —
    accumulate into one comparable table. Refuses to append under a header
    from a different schema, which would misalign every row.
    """
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not append or not path.exists() or path.stat().st_size == 0
    if not write_header:
        with path.open(newline="") as handle:
            existing = next(csv.reader(handle), [])
        if existing and tuple(existing) != STATS_FIELDS:
            raise ValueError(
                f"{path} was written with a different set of columns "
                f"({', '.join(existing)}). Appending would misalign every row. "
                "Write to a new file, or move the old one aside."
            )

    with path.open("a" if append else "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=STATS_FIELDS)
        if write_header:
            writer.writeheader()
        for joint in stats.joints:
            writer.writerow({
                "run_id": stats.run_id,
                "finger": stats.finger,
                "source": stats.source,
                "angle_source": stats.angle_source,
                "waypoints": stats.waypoints,
                "settled": stats.settled,
                "stalled": stats.stalled,
                "timed_out": stats.timed_out,
                "stopped": stats.stopped,
                "duration_mean_s": _cell(stats.duration_mean_s),
                "duration_median_s": _cell(stats.duration_median_s),
                "duration_min_s": _cell(stats.duration_min_s),
                "duration_max_s": _cell(stats.duration_max_s),
                "time_to_move_mean_s": _cell(stats.time_to_move_mean_s),
                "time_to_move_median_s": _cell(stats.time_to_move_median_s),
                "time_in_tolerance_median_s": _cell(stats.time_in_tolerance_median_s),
                "time_in_tolerance_n": stats.time_in_tolerance_n,
                "dwell_travel_median_deg": _cell(stats.dwell_travel_median_deg),
                "dwell_travel_p95_deg": _cell(stats.dwell_travel_p95_deg),
                "joint": joint.joint,
                "n": joint.n,
                "err_mean": _cell(joint.err_mean),
                "err_sd": _cell(joint.err_sd),
                "abs_err_median": _cell(joint.abs_err_median),
                "abs_err_p95": _cell(joint.abs_err_p95),
                "abs_err_max": _cell(joint.abs_err_max),
                "creep_mean": _cell(joint.creep_mean),
                "abs_creep_p95": _cell(joint.abs_creep_p95),
                "abs_settle_err_median": _cell(joint.abs_settle_err_median),
                "settle_hysteresis_gap": _cell(joint.settle_hysteresis_gap),
                **_checkpoint_cells(joint),
                "flex_n": joint.flex_n,
                "flex_err_mean": _cell(joint.flex_err_mean),
                "extend_n": joint.extend_n,
                "extend_err_mean": _cell(joint.extend_err_mean),
                "hysteresis_gap": _cell(joint.hysteresis_gap),
                "fit_target_pct": _cell(joint.fit_target_pct, 3),
                "fit_move_pct": _cell(joint.fit_move_pct, 3),
                "fit_const": _cell(joint.fit_const),
                "fit_r2": _cell(joint.fit_r2),
                "fit_resid_sd": _cell(joint.fit_resid_sd),
                "motion_threshold_deg": motion_threshold_deg,
                "settle_window_s": settle_window_s,
                "arrival_tolerance_deg": arrival_tolerance_deg,
                "in_tolerance_deg": in_tolerance_deg,
                "settle_dwell_s": settle_dwell_s,
                "ramp_speed_deg_s": ramp_speed_deg_s,
                "ramp_steps": ramp_steps,
                "ramp_time_s": ramp_time_s,
                "ramp_accel_deg_s2": ramp_accel_deg_s2,
                "ramp_per_joint": ramp_per_joint,
            })
    return path
