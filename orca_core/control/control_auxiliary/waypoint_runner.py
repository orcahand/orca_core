"""Waypoint-following bench test for one finger.

Commands a finger to a target pose, waits until it stops moving, then issues
the next one — either the next waypoint of a preset trajectory or a fresh
pose sampled inside each joint's reachable range. Every waypoint's targets,
settle time and residual error come back as a :class:`WaypointRecord` the
caller writes out with :func:`write_results_csv`.

Arrival is inferred from motion alone: a finger counts as settled once every
one of its joints has moved less than ``motion_threshold_deg`` peak-to-peak
over the last ``settle_window_s``, and only after it has actually set off —
otherwise the still moment between the command and the first movement reads
as arrival and every waypoint terminates at the settle window.

Motion cannot tell arrival from a stall, since a joint jammed against an
obstacle is equally still. So the residual decides the label: a finger that
stopped further than ``arrival_tolerance_deg`` from its target is recorded as
``stalled``, and a waypoint that never stops ends as ``timeout`` rather than
blocking.
"""

from __future__ import annotations

import csv
import logging
import math
import random
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

from ...joint_position import OrcaJointPositions


logger = logging.getLogger(__name__)


# Peak-to-peak joint travel over the settle window that still counts as
# stationary. Comfortably above joint-encoder noise (a few tenths of a degree)
# and below any real motion.
DEFAULT_MOTION_THRESHOLD_DEG = 1.0

# Window the peak-to-peak is measured over, and therefore the shortest
# possible time-to-settle a waypoint can report.
DEFAULT_SETTLE_WINDOW_S = 0.25

DEFAULT_WAYPOINT_TIMEOUT_S = 6.0
DEFAULT_POLL_HZ = 100.0

# A waypoint can only be declared settled once the finger has actually moved
# this far from where it started; without that guard the still moment before
# it sets off reads as arrival, and every waypoint terminates at the settle
# window. A target the finger is already at never trips it, so the grace
# period below bounds how long that case waits.
DEFAULT_START_TRAVEL_DEG = 1.0
DEFAULT_START_GRACE_S = 0.75

# Held at the target after motion stops, before the residual is read. The
# motion test fires as soon as travel drops below the threshold, which on a
# slow approach is still mid-convergence; the dwell lets that finish so the
# recorded error is steady-state rather than a snapshot of the approach.
DEFAULT_SETTLE_DWELL_S = 0.5

# Times after the settle instant at which the angle is snapshotted, so one
# long-dwell run yields the whole convergence curve instead of only its end
# point. Checkpoints past ``dwell_s`` are left blank.
DEFAULT_DWELL_CHECKPOINTS_S = (0.5, 1.0, 1.5)
MAX_DWELL_CHECKPOINTS = 3

# Band used to time convergence: how close counts as "arrived" for
# ``time_in_tolerance_s``. Deliberately separate from the stall tolerance below
# and much tighter — a band wider than the overshoot would be entered during
# the initial fast move and would never see the recovery at all.
DEFAULT_IN_TOLERANCE_DEG = 1.0

# Residual above which a finger that stopped moving is recorded as ``stalled``
# rather than ``settled``. Detection itself stays motion-only; this only
# labels the outcome.
DEFAULT_ARRIVAL_TOLERANCE_DEG = 3.0

# Fraction of each joint's span held back from both ends when sampling, so
# random targets don't sit on a hardstop the joint can only approach.
DEFAULT_TARGET_INSET = 0.05


ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]
WaypointSource = Callable[[int], Dict[str, float]]


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the run."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("waypoint-test progress callback failed")


# =========================================================================
# Joint selection and target ranges
# =========================================================================

def available_fingers(config) -> List[str]:
    """Finger prefixes this hand actually has, in ``joint_ids`` order.

    Read off the joint names rather than assumed, so a hand missing a finger —
    or carrying one this package has never heard of — still resolves. Joints
    without a ``{finger}_{type}`` shape (the wrist) contribute nothing.
    """
    fingers: List[str] = []
    for joint in config.joint_ids:
        prefix, sep, _ = joint.partition("_")
        if sep and prefix not in fingers:
            fingers.append(prefix)
    return fingers


def finger_joints(config, finger: str) -> List[str]:
    """Every joint belonging to ``finger``, in ``joint_ids`` order."""
    return [j for j in config.joint_ids if j.startswith(f"{finger}_")]


def resolve_joints(
    config, finger: str, joints: Optional[Sequence[str]] = None
) -> List[str]:
    """The joints of ``finger`` a run should drive, in ``joint_ids`` order.

    ``joints`` narrows the set to a subset of that finger — driving one joint
    while its siblings hold is how a coupled joint's own behaviour is
    separated from the disturbance its neighbours impose. ``None`` takes the
    whole finger.

    Raises:
        ValueError: the hand has no such finger, or a named joint does not
            belong to it.
    """
    available = finger_joints(config, finger)
    if not available:
        raise ValueError(
            f"hand has no joints for finger {finger!r}; "
            f"available fingers: {', '.join(available_fingers(config))}"
        )
    if joints is None:
        return available
    unknown = [j for j in joints if j not in available]
    if unknown:
        raise ValueError(
            f"joint(s) {', '.join(unknown)} do not belong to {finger!r}, "
            f"which has {', '.join(available)}"
        )
    selected = [j for j in available if j in set(joints)]
    if not selected:
        raise ValueError(f"no joints selected for {finger!r}")
    return selected


def joint_target_range(
    hand, joint: str, inset: float = DEFAULT_TARGET_INSET
) -> "tuple[float, float]":
    """Command range for ``joint`` that is both legal and reachable.

    ``set_joint_positions`` clamps to the config ROM while the joint↔motor map
    runs in the effective (encoder-measured) ROM, so only the overlap of the
    two is worth aiming at. In practice the measured span only ever narrows the
    extension end — the flex end is the encoder anchor, which is *defined* as
    the config ROM upper — so this catches joints whose configured extension
    travel is optimistic. The ends are then inset so targets don't land on a
    hardstop.
    """
    config_rom = hand.config.joint_roms_dict[joint]
    effective_rom = hand.effective_joint_roms_dict.get(joint, config_rom)
    lower = max(config_rom[0], effective_rom[0])
    upper = min(config_rom[1], effective_rom[1])
    if upper <= lower:
        # The two frames don't overlap — trust the clamp and say so, rather
        # than sampling an empty interval.
        logger.warning(
            "joint %s has no overlap between its config ROM %s and its "
            "measured ROM %s; sampling the config ROM instead",
            joint, list(config_rom), list(effective_rom),
        )
        lower, upper = config_rom[0], config_rom[1]
    pad = (upper - lower) * inset
    return lower + pad, upper - pad


# =========================================================================
# Waypoint sources
# =========================================================================

class RandomWaypoints:
    """Uniform random poses inside each joint's reachable range.

    ``seed`` makes a run repeatable, which is what lets two controller
    variants be compared on the same sequence of targets.
    """

    name = "random"

    def __init__(
        self,
        hand,
        joints: Sequence[str],
        seed: Optional[int] = None,
        inset: float = DEFAULT_TARGET_INSET,
    ):
        self.ranges = {j: joint_target_range(hand, j, inset) for j in joints}
        self._rng = random.Random(seed)

    def __call__(self, index: int) -> Dict[str, float]:
        return {
            joint: self._rng.uniform(lower, upper)
            for joint, (lower, upper) in self.ranges.items()
        }


class TrajectoryWaypoints:
    """Replay a fixed list of poses, wrapping once the list runs out."""

    name = "trajectory"

    def __init__(self, waypoints: Sequence[Dict[str, float]]):
        if not waypoints:
            raise ValueError("trajectory contains no waypoints")
        self.waypoints = [dict(wp) for wp in waypoints]

    def __len__(self) -> int:
        return len(self.waypoints)

    def __call__(self, index: int) -> Dict[str, float]:
        return dict(self.waypoints[index % len(self.waypoints)])


def read_trajectory_csv(path, joints: Sequence[str]) -> List[Dict[str, float]]:
    """Read a waypoint trajectory for ``joints``.

    One row per joint: the first column names the joint, every remaining
    column is one waypoint in joint degrees. Columns are ordered left to
    right, so the header names are labels only::

        joint,waypoint_0,waypoint_1,waypoint_2
        index_abd,-14.0,5.0,-20.0
        index_mcp,2.0,60.0,15.0
        index_pip,6.0,45.0,80.0

    Rows for joints outside ``joints`` are ignored, so one file can cover the
    whole hand. Joints the file never names are simply left uncommanded.

    Raises:
        FileNotFoundError: no file at ``path``.
        ValueError: the file is empty, names none of ``joints``, or holds a
            value that isn't a number.
    """
    path = Path(path).expanduser()
    wanted = set(joints)
    per_joint: Dict[str, List[float]] = {}

    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        for row in reader:
            cells = [cell.strip() for cell in row]
            if not cells or not cells[0] or cells[0].startswith("#"):
                continue
            joint = cells[0]
            if joint == "joint" or joint not in wanted:
                continue  # header row, or a joint this run doesn't drive
            try:
                per_joint[joint] = [float(cell) for cell in cells[1:] if cell]
            except ValueError as exc:
                raise ValueError(
                    f"{path}: row for joint {joint!r} holds a non-numeric value"
                ) from exc

    if not per_joint:
        raise ValueError(
            f"{path} names none of the joints under test ({', '.join(joints)}). "
            "The first column must be the joint name."
        )

    lengths = {len(values) for values in per_joint.values()}
    if len(lengths) != 1:
        counts = ", ".join(f"{j}={len(v)}" for j, v in per_joint.items())
        raise ValueError(
            f"{path}: every joint row must hold the same number of waypoints ({counts})"
        )
    count = lengths.pop()
    if count == 0:
        raise ValueError(f"{path}: joint rows hold no waypoints")

    return [
        {joint: values[i] for joint, values in per_joint.items()}
        for i in range(count)
    ]


# =========================================================================
# Results
# =========================================================================

@dataclass(frozen=True)
class JointOutcome:
    """What one joint did over one waypoint."""

    joint: str
    target_deg: float
    start_deg: float
    settle_deg: float
    final_deg: float
    error_deg: float
    moved_deg: float
    checkpoint_deg: "tuple[Optional[float], ...]" = ()
    """Angle at each dwell checkpoint, ``None`` where the dwell ended first."""


@dataclass(frozen=True)
class WaypointRecord:
    """One commanded pose and how the finger answered it."""

    finger: str
    source: str
    waypoint_index: int
    started_at: float
    duration_s: float
    time_to_move_s: Optional[float]
    dwell_travel_deg: Optional[float]
    time_in_tolerance_s: Optional[float]
    """Seconds from the command to the last entry into the tolerance band that
    was never left again — settling time in the control sense. ``None`` when
    the joint was outside the band when observation ended.

    The *last* entry, not the first: a fast approach clips the band on its way
    through the target, so timing the first entry would record the fly-through
    rather than the convergence. Condition-independent, unlike ``duration_s``,
    which on a ramped move includes the ramp."""
    outcome: str
    angle_source: str
    joints: "tuple[JointOutcome, ...]"
    checkpoint_times_s: "tuple[float, ...]" = ()


RESULT_FIELDS = (
    "run_id",
    "started_at",
    "finger",
    "source",
    "waypoint_index",
    "joint",
    "target_deg",
    "start_deg",
    "settle_deg",
    "final_deg",
    "error_deg",
    "moved_deg",
    "duration_s",
    "time_to_move_s",
    "dwell_travel_deg",
    "outcome",
    "angle_source",
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
    "time_in_tolerance_s",
    "t1_s",
    "err_t1",
    "t2_s",
    "err_t2",
    "t3_s",
    "err_t3",
)


def _cell_or_blank(value, digits: int = 4):
    return "" if value is None else round(value, digits)


def _checkpoint_cells(times, joint) -> "Dict[str, object]":
    """Flatten a joint's dwell checkpoints into the fixed ``t{n}_s`` / ``err_t{n}``
    column pairs. Unused slots stay blank."""
    cells: Dict[str, object] = {}
    for slot in range(MAX_DWELL_CHECKPOINTS):
        at = times[slot] if slot < len(times) else None
        angle = (
            joint.checkpoint_deg[slot]
            if slot < len(joint.checkpoint_deg) else None
        )
        cells[f"t{slot + 1}_s"] = "" if at is None else at
        cells[f"err_t{slot + 1}"] = (
            "" if angle is None else round(angle - joint.target_deg, 4)
        )
    return cells


def write_results_csv(
    path,
    records: Sequence[WaypointRecord],
    *,
    run_id: str,
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
    """Write ``records`` as one row per (waypoint, joint).

    The long shape keeps a 3-joint finger and a 4-joint thumb on one schema,
    and the run-level settings ride on every row so runs with different
    thresholds stay comparable once concatenated. Appends by default, writing
    the header only when the file is new.
    """
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not append or not path.exists() or path.stat().st_size == 0
    if not write_header:
        # Appending under a header from an older schema writes every value one
        # column off, silently. Refuse rather than corrupt the file.
        with path.open(newline="") as handle:
            existing = next(csv.reader(handle), [])
        if existing and tuple(existing) != RESULT_FIELDS:
            raise ValueError(
                f"{path} was written with a different set of columns "
                f"({', '.join(existing)}). Appending would misalign every row. "
                "Write to a new file, or move the old one aside."
            )

    with path.open("a" if append else "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=RESULT_FIELDS)
        if write_header:
            writer.writeheader()
        for record in records:
            started_at = datetime.fromtimestamp(
                record.started_at, tz=timezone.utc
            ).isoformat(timespec="milliseconds")
            for joint in record.joints:
                writer.writerow({
                    "run_id": run_id,
                    "started_at": started_at,
                    "finger": record.finger,
                    "source": record.source,
                    "waypoint_index": record.waypoint_index,
                    "joint": joint.joint,
                    "target_deg": round(joint.target_deg, 4),
                    "start_deg": round(joint.start_deg, 4),
                    "settle_deg": round(joint.settle_deg, 4),
                    "final_deg": round(joint.final_deg, 4),
                    "error_deg": round(joint.error_deg, 4),
                    "moved_deg": round(joint.moved_deg, 4),
                    "duration_s": round(record.duration_s, 4),
                    "time_to_move_s": (
                        "" if record.time_to_move_s is None
                        else round(record.time_to_move_s, 4)
                    ),
                    "dwell_travel_deg": (
                        "" if record.dwell_travel_deg is None
                        else round(record.dwell_travel_deg, 4)
                    ),
                    "outcome": record.outcome,
                    "angle_source": record.angle_source,
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
                    "time_in_tolerance_s": _cell_or_blank(record.time_in_tolerance_s),
                    **_checkpoint_cells(record.checkpoint_times_s, joint),
                })
    return path


# =========================================================================
# The run
# =========================================================================

class JointReader:
    """Current angles for a fixed joint set, from the cheapest live source.

    A running joint loop already decodes every encoder frame at its own rate,
    so its latest measurement costs a lock and a copy. ``get_joint_position()``
    does a full motor-bus read *first* and only then overlays those same
    angles, which would put an observer's traffic on the lock the loop needs
    hundreds of times a second — so it is the fallback for hands without a
    loop, not the default.

    ``source`` names the live source and switches itself to the fallback for
    the rest of the run if the loop stops serving measurements.
    """

    LOOP = "encoder-loop"
    MOTOR = "motor-estimate"

    def __init__(self, hand, joints: Sequence[str]):
        self._hand = hand
        self._joints = list(joints)
        loop_joints = getattr(hand, "loop_joint_names", None)
        covered = loop_joints is not None and set(self._joints).issubset(loop_joints)
        self.source = self.LOOP if covered else self.MOTOR

    def __call__(self) -> Dict[str, Optional[float]]:
        if self.source == self.LOOP:
            try:
                measured = self._hand.get_measured_joints()
            except RuntimeError:
                logger.warning(
                    "joint loop stopped serving measurements (watchdog e-stop?); "
                    "falling back to motor-derived joint angles for the rest "
                    "of the run"
                )
                self.source = self.MOTOR
            else:
                return {joint: measured.get(joint) for joint in self._joints}
        pose = self._hand.get_joint_position().as_dict()
        return {joint: pose.get(joint) for joint in self._joints}


def _peak_to_peak(
    samples: "deque[tuple[float, Dict[str, Optional[float]]]]",
    joints: Sequence[str],
) -> Optional[float]:
    """Largest peak-to-peak travel across ``joints`` over the buffered window,
    or ``None`` while any joint has no readable sample."""
    widest = 0.0
    for joint in joints:
        values = [
            angles[joint] for _, angles in samples if angles.get(joint) is not None
        ]
        if not values:
            return None
        widest = max(widest, max(values) - min(values))
    return widest


def _within_tolerance(
    angles: Dict[str, Optional[float]],
    targets: Dict[str, float],
    tolerance_deg: float,
) -> bool:
    """True when every commanded joint with a reading sits inside tolerance."""
    seen = False
    for joint, target in targets.items():
        value = angles.get(joint)
        if value is None:
            continue
        seen = True
        if abs(value - target) > tolerance_deg:
            return False
    return seen


def _travel_from(
    start: Dict[str, Optional[float]], now: Dict[str, Optional[float]]
) -> float:
    """Largest per-joint displacement between two poses. Joints missing an
    angle in either pose contribute nothing."""
    return max(
        (
            abs(value - start[joint])
            for joint, value in now.items()
            if value is not None and start.get(joint) is not None
        ),
        default=0.0,
    )


def _profile_points(
    span: float,
    speed_deg_s: float,
    accel_deg_s2: float,
    poll_hz: float,
) -> "List[tuple[float, float]]":
    """``(time, fraction-of-span)`` samples describing one move.

    ``accel_deg_s2 <= 0`` gives a constant-velocity ramp: the joint is still
    travelling at ``speed_deg_s`` when the last setpoint lands on the target,
    so the endpoint is hit at full speed. A positive acceleration gives a
    trapezoid — accelerate, cruise, decelerate — that arrives at rest, which
    is what a motor-side velocity profile does and what a plain position step
    approximates through its own shrinking error.

    Short moves come out triangular: the cruise speed is never reached.
    """
    if span <= 0 or speed_deg_s <= 0:
        return [(0.0, 1.0)]

    if accel_deg_s2 <= 0:
        duration = span / speed_deg_s
        steps = max(2, int(round(duration * poll_hz)))
        return [
            (duration * i / (steps - 1), (i + 1) / steps) for i in range(steps)
        ]

    ramp_time = speed_deg_s / accel_deg_s2
    ramp_span = 0.5 * accel_deg_s2 * ramp_time * ramp_time
    if 2.0 * ramp_span >= span:
        ramp_time = math.sqrt(span / accel_deg_s2)
        cruise_time = 0.0
        ramp_span = 0.5 * accel_deg_s2 * ramp_time * ramp_time
    else:
        cruise_time = (span - 2.0 * ramp_span) / speed_deg_s
    duration = 2.0 * ramp_time + cruise_time

    steps = max(2, int(round(duration * poll_hz)))
    points = []
    for i in range(steps):
        at = duration * i / (steps - 1)
        if at <= ramp_time:
            travelled = 0.5 * accel_deg_s2 * at * at
        elif at <= ramp_time + cruise_time:
            travelled = ramp_span + speed_deg_s * (at - ramp_time)
        else:
            remaining = duration - at
            travelled = span - 0.5 * accel_deg_s2 * remaining * remaining
        points.append((at, min(1.0, travelled / span)))
    return points


def _ramp_schedule(
    start: Dict[str, Optional[float]],
    targets: Dict[str, float],
    steps: int,
    duration_s: float,
    speed_deg_s: float,
    accel_deg_s2: float,
    poll_hz: float,
    per_joint: bool = False,
) -> "List[tuple[float, Dict[str, float]]]":
    """Interpolated setpoints from the current pose to ``targets``, each with
    the offset from command time at which it should be issued.

    ``speed_deg_s`` derives the duration from the distance, so every move runs
    at the same commanded speed whatever its size; ``accel_deg_s2`` shapes it
    (see :func:`_profile_points`). Without a speed, ``steps`` setpoints are
    spread linearly over ``duration_s``.

    By default every joint shares one timeline sized by the joint furthest
    from its target, so the finger arrives as a unit — which means the shorter
    movers travel proportionally slower than the commanded speed.
    ``per_joint`` gives each joint its own profile at the commanded speed
    instead, at the cost of them finishing at different times.

    Setpoints are released on the run's own poll clock rather than through
    ``set_joint_positions(num_steps=...)``, whose ``time.sleep`` between
    waypoints would block the loop and leave the whole approach unsampled.
    A single-step ramp is just the target at t=0.
    """
    spans = {
        joint: abs(value - start[joint])
        for joint, value in targets.items()
        if start.get(joint) is not None
    }
    widest = max(spans.values(), default=0.0)

    def blend(fractions: Dict[str, float]) -> Dict[str, float]:
        return {
            joint: (
                start[joint] + (value - start[joint]) * fractions.get(joint, 1.0)
                if start.get(joint) is not None else value
            )
            for joint, value in targets.items()
        }

    if speed_deg_s > 0 and widest > 0:
        if not per_joint:
            points = _profile_points(widest, speed_deg_s, accel_deg_s2, poll_hz)
            return [(at, blend({j: f for j in targets})) for at, f in points]
        # Each joint on its own profile; sample them on the longest timeline
        # and hold whichever have already arrived.
        per = {
            joint: _profile_points(span, speed_deg_s, accel_deg_s2, poll_hz)
            for joint, span in spans.items() if span > 0
        }
        if not per:
            return [(0.0, dict(targets))]
        grid = max(per.values(), key=lambda pts: pts[-1][0])
        schedule = []
        for at, _ in grid:
            fractions = {}
            for joint, pts in per.items():
                fractions[joint] = next(
                    (f for t, f in pts if t >= at), 1.0
                ) if at <= pts[-1][0] else 1.0
            schedule.append((at, blend(fractions)))
        return schedule

    if steps <= 1 or duration_s <= 0 or widest == 0.0:
        return [(0.0, dict(targets))]
    return [
        (
            duration_s * (step - 1) / (steps - 1),
            blend({j: step / steps for j in targets}),
        )
        for step in range(1, steps + 1)
    ]


def run_waypoint_test(
    hand,
    finger: str,
    source: WaypointSource,
    num_waypoints: int,
    *,
    joints: Optional[Sequence[str]] = None,
    motion_threshold_deg: float = DEFAULT_MOTION_THRESHOLD_DEG,
    settle_window_s: float = DEFAULT_SETTLE_WINDOW_S,
    timeout_s: float = DEFAULT_WAYPOINT_TIMEOUT_S,
    poll_hz: float = DEFAULT_POLL_HZ,
    dwell_s: float = DEFAULT_SETTLE_DWELL_S,
    dwell_checkpoints_s: Sequence[float] = DEFAULT_DWELL_CHECKPOINTS_S,
    ramp_steps: int = 1,
    ramp_time_s: float = 0.0,
    ramp_speed_deg_s: float = 0.0,
    ramp_accel_deg_s2: float = 0.0,
    ramp_per_joint: bool = False,
    start_travel_deg: float = DEFAULT_START_TRAVEL_DEG,
    start_grace_s: float = DEFAULT_START_GRACE_S,
    arrival_tolerance_deg: float = DEFAULT_ARRIVAL_TOLERANCE_DEG,
    in_tolerance_deg: float = DEFAULT_IN_TOLERANCE_DEG,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> List[WaypointRecord]:
    """Drive ``finger`` through ``num_waypoints`` poses, one settle at a time.

    Only ``finger``'s joints are ever commanded; the rest of the hand keeps
    whatever pose the caller left it in.

    Args:
        hand: A connected hand. Joint angles come from
            ``get_joint_position()``, so they are encoder-measured when the
            joint loop is engaged and motor-derived otherwise.
        finger: Finger prefix, e.g. ``"index"``.
        source: Called with the waypoint index, returns ``{joint: degrees}``.
            Entries outside ``finger`` are dropped.
        num_waypoints: How many poses to command.
        joints: Restrict to these joints of ``finger``; ``None`` drives the
            whole finger. The unnamed ones are never commanded and hold
            whatever pose they were left in.
        motion_threshold_deg: Peak-to-peak travel over the settle window at or
            below which the finger counts as stopped.
        dwell_s: Seconds to hold the target after motion stops before the
            residual is read, so a slow approach is not scored mid-flight.
            ``duration_s`` still reports time-to-settle, excluding this.
        dwell_checkpoints_s: Times after the settle instant at which to
            snapshot the angle, so one run yields the convergence curve rather
            than only its end point. At most three; any past ``dwell_s`` come
            back empty.
        ramp_steps: Split each move into this many interpolated setpoints
            issued over ``ramp_time_s``. ``1`` (the default) commands the
            target as a single step, which is what makes the finger arrive
            with velocity.
        ramp_time_s: Wall time the ramp is spread over. Ignored when
            ``ramp_steps`` is 1, or when ``ramp_speed_deg_s`` is set.
        ramp_speed_deg_s: Commanded joint speed. Overrides ``ramp_steps`` and
            ``ramp_time_s``, deriving both from the distance so every move
            runs at the same speed. This is the parameterisation a motor-side
            velocity profile uses.
        ramp_accel_deg_s2: Acceleration shaping the ramp. ``0`` holds the
            speed right up to the target, so the joint arrives moving; a
            positive value gives a trapezoid that decelerates in and arrives
            at rest.
        ramp_per_joint: Give each joint its own profile at the commanded
            speed, instead of one shared timeline sized by the furthest-
            travelling joint. Off by default, so the finger arrives as a unit
            and shorter movers run proportionally slower.
        start_travel_deg: How far the finger must move from its starting pose
            before stillness is allowed to mean arrival.
        start_grace_s: How long to wait for that motion before settling
            anyway, so a target the finger already holds still terminates.
        arrival_tolerance_deg: Residual above which a finger that stopped is
            recorded as ``stalled`` rather than ``settled``.
        in_tolerance_deg: Band for ``time_in_tolerance_s``. Keep it near the
            joint's steady-state capability: a band wider than the overshoot
            is entered on the way in and never times the recovery.
        settle_window_s: Width of that window, and the shortest settle time
            any waypoint can report.
        timeout_s: Give up on a waypoint after this long and record it as
            ``timeout``.
        poll_hz: Joint-readback rate. Reads the running loop's latest
            measurement, so it costs no bus traffic; above the loop's own rate
            it only repeats samples.
        progress_callback: Receives ``waypoint_started`` / ``waypoint_finished``
            events as plain dicts.
        should_stop: Polled between and during waypoints; returning ``True``
            ends the run, recording the waypoint in flight as ``stopped``.

    Returns:
        One :class:`WaypointRecord` per commanded waypoint, in order. Each
        outcome is ``settled``, ``stalled`` (stopped short of the target),
        ``timeout`` or ``stopped``.

    Raises:
        ValueError: ``finger`` has no joints on this hand, ``joints`` names one
            outside it, ``source`` names none of them, or a joint reads no
            angle (uncalibrated).
    """
    joints = resolve_joints(hand.config, finger, joints)
    checkpoints = tuple(sorted(float(t) for t in dwell_checkpoints_s))
    if len(checkpoints) > MAX_DWELL_CHECKPOINTS:
        raise ValueError(
            f"at most {MAX_DWELL_CHECKPOINTS} dwell checkpoints are recorded, "
            f"got {len(checkpoints)}"
        )

    read_joints = JointReader(hand, joints)
    unreadable = [j for j, v in read_joints().items() if v is None]
    if unreadable:
        raise ValueError(
            f"joint(s) {', '.join(unreadable)} report no angle — they are not "
            "calibrated, so settle cannot be measured on them. Run "
            "scripts/calibrate.py, or test another finger."
        )

    period = 1.0 / float(poll_hz)
    records: List[WaypointRecord] = []

    for index in range(num_waypoints):
        if should_stop is not None and should_stop():
            break

        targets = {
            joint: float(value)
            for joint, value in source(index).items()
            if joint in joints
        }
        if not targets:
            raise ValueError(
                f"waypoint {index} names none of {finger}'s joints "
                f"({', '.join(joints)})"
            )

        start_angles = read_joints()
        started_at = time.time()
        _emit(
            progress_callback, "waypoint_started",
            finger=finger, index=index, targets=dict(targets),
            start=dict(start_angles),
        )

        schedule = _ramp_schedule(
            start_angles, targets, ramp_steps, ramp_time_s,
            ramp_speed_deg_s, ramp_accel_deg_s2, poll_hz, ramp_per_joint,
        )
        pending = list(schedule)
        hand.set_joint_positions(pending.pop(0)[1])
        began = time.monotonic()

        samples: "deque[tuple[float, Dict[str, Optional[float]]]]" = deque()
        outcome = "timeout"
        time_to_move: Optional[float] = None
        in_tolerance: Optional[float] = None
        while True:
            if should_stop is not None and should_stop():
                outcome = "stopped"
                break

            now = time.monotonic()
            elapsed = now - began
            # Release each ramp setpoint on schedule, without blocking the
            # sampling that has to run through the whole approach.
            while pending and pending[0][0] <= elapsed:
                hand.set_joint_positions(pending.pop(0)[1])

            angles = read_joints()
            samples.append((now, angles))
            # Keep exactly one sample older than the window so the buffer
            # always spans the full settle_window_s once it is warm.
            cutoff = now - settle_window_s
            while len(samples) > 2 and samples[1][0] <= cutoff:
                samples.popleft()

            if _within_tolerance(angles, targets, in_tolerance_deg):
                if in_tolerance is None:
                    in_tolerance = elapsed
            else:
                in_tolerance = None

            if time_to_move is None and _travel_from(
                start_angles, angles
            ) >= start_travel_deg:
                time_to_move = now - began

            # Stillness only means arrival once the finger has actually set
            # off. The grace period releases that guard for a target the
            # finger already sits at, which never trips it.
            # A ramp still issuing setpoints has not finished commanding the
            # move, so stillness cannot mean arrival yet.
            may_settle = not pending and (
                time_to_move is not None or elapsed >= start_grace_s
            )
            if may_settle and samples[-1][0] - samples[0][0] >= settle_window_s:
                travel = _peak_to_peak(samples, joints)
                if travel is not None and travel < motion_threshold_deg:
                    outcome = "settled"
                    break

            if elapsed >= timeout_s:
                outcome = "timeout"
                break

            time.sleep(max(0.0, period - (time.monotonic() - now)))

        duration = time.monotonic() - began
        settle_angles = read_joints()

        # Hold the target and let the approach finish before reading the
        # residual: motion drops below the threshold while a slow convergence
        # is still running, so the settle-instant angle understates arrival.
        dwell_travel: Optional[float] = None
        final_angles = settle_angles
        captured: "List[Optional[Dict[str, Optional[float]]]]" = [
            None for _ in checkpoints
        ]
        if dwell_s > 0 and outcome == "settled":
            dwell_began = time.monotonic()
            dwell_samples: "deque[tuple[float, Dict[str, Optional[float]]]]" = deque(
                [(dwell_began, settle_angles)]
            )
            while time.monotonic() - dwell_began < dwell_s:
                if should_stop is not None and should_stop():
                    break
                mark = time.monotonic()
                angles = read_joints()
                dwell_samples.append((mark, angles))
                if _within_tolerance(angles, targets, in_tolerance_deg):
                    if in_tolerance is None:
                        in_tolerance = mark - began
                else:
                    in_tolerance = None
                # First sample at or past each checkpoint wins it.
                since = mark - dwell_began
                for slot, at in enumerate(checkpoints):
                    if captured[slot] is None and since >= at:
                        captured[slot] = angles
                time.sleep(max(0.0, period - (time.monotonic() - mark)))
            final_angles = dwell_samples[-1][1]
            dwell_travel = _peak_to_peak(dwell_samples, joints)

        joint_outcomes = tuple(
            JointOutcome(
                joint=joint,
                target_deg=targets[joint],
                start_deg=start_angles[joint],
                settle_deg=settle_angles[joint],
                final_deg=final_angles[joint],
                error_deg=final_angles[joint] - targets[joint],
                moved_deg=final_angles[joint] - start_angles[joint],
                checkpoint_deg=tuple(
                    None if snapshot is None else snapshot.get(joint)
                    for snapshot in captured
                ),
            )
            for joint in joints
            if joint in targets
            and start_angles[joint] is not None
            and settle_angles[joint] is not None
            and final_angles[joint] is not None
        )
        # Motion said "stopped"; the residual says whether that was arrival.
        if outcome == "settled" and joint_outcomes:
            worst = max(abs(j.error_deg) for j in joint_outcomes)
            if worst > arrival_tolerance_deg:
                outcome = "stalled"

        record = WaypointRecord(
            finger=finger,
            source=getattr(source, "name", type(source).__name__),
            waypoint_index=index,
            started_at=started_at,
            duration_s=duration,
            time_to_move_s=time_to_move,
            dwell_travel_deg=dwell_travel,
            time_in_tolerance_s=in_tolerance,
            checkpoint_times_s=checkpoints,
            outcome=outcome,
            angle_source=read_joints.source,
            joints=joint_outcomes,
        )
        records.append(record)
        _emit(
            progress_callback, "waypoint_finished",
            finger=finger, index=index, outcome=outcome,
            duration_s=duration, record=record,
        )

        if outcome == "stopped":
            break

    return records
