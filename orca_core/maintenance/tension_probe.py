# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Tension probe: how much slack each tendon has, and how hard its motor must
pull before the joint follows, measured one motor at a time.

Two methods, run from the same held pose so they can be compared side by side:

``breakaway``
    The current limit is stepped up from a low start while the motor aims
    past the hold position. The first current that moves the motor more than
    ``play_deg`` is the breakaway current; the first that moves it more than
    ``move_deg`` is the current the joint follows at. Every step's
    displacement is kept, so the whole current/displacement curve is there.
    On a well-tensioned hand this is the current it takes to move the joint;
    a side that breaks away at much less than the other is the loose one.

``takeup``
    The motor is driven in one direction at a fixed low current and the
    angle it turns before stalling is recorded. The motor cannot tell a
    tendon going taut from a finger following it, so this only reads slack on
    a joint that does not move at that current. Off by default in the console.

``stroke``
    The motors of a group are driven together, position-controlled at
    ``stroke_speed_deg_s`` under a ``stroke_current_ma`` limit, ``max_travel_deg``
    away from the hold pose and back, while each one's current is sampled: the
    mean current over the stroke is the friction the tendon puts on the motor,
    the same quantity the tension sweep measures over the whole travel. Every
    motor of the group starts and moves at the same instant, which a breakaway
    ramp cannot do. This is the console's tension check. The result carries
    ``mean_ma`` and ``peak_ma``; ``breakaway_ma`` repeats the mean so the
    scoring that compares sides against a reference reads either method.

Both run in both motor directions, ``repeats`` times over, so the currents
can be averaged. With ``parallel`` the joints of one kind (all abductions, all
MCPs, the PIPs with the thumb dip, the thumb cmc) ramp together, the way
calibration drives them, so a run takes a fifth of the time. Every other motor
holds its position at ``hold_current_ma`` meanwhile, every probed motor is
driven back to where it started before the other direction, and none goes
closer than ``limit_margin_deg`` to a calibrated motor limit. Interaction-free like the other maintenance
routines: ``progress_callback`` events, ``should_stop``.
"""

from __future__ import annotations

import dataclasses
import logging
import math
import time
from dataclasses import dataclass

import numpy as np
from typing import Callable, Dict, List, Optional, Sequence, Tuple, TYPE_CHECKING

from ..constants import CURRENT_BASED_POSITION, WRIST
from .motor_reads import read_motor_pos_checked

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

TAKEUP = "takeup"
BREAKAWAY = "breakaway"
STROKE = "stroke"
METHODS = (TAKEUP, BREAKAWAY, STROKE)
# Share of the stroke at its start left out of the mean (the motor getting going).
STROKE_EDGE = 0.2
DIRECTIONS = (1, -1)
# Where the probe starts: the hand's neutral pose (repeatable, clear of the
# limits), or wherever the hand stands.
POSE_MID = "mid"          # every probed joint at the middle of its ROM: furthest from both limits
POSE_NEUTRAL = "neutral"
POSE_HOLD = "hold"
POSES = (POSE_MID, POSE_NEUTRAL, POSE_HOLD)
NEUTRAL_MOVE_STEPS = 50
NEUTRAL_MOVE_STEP_S = 0.02
NEUTRAL_SETTLE_S = 1.0
# Float slack for threshold comparisons on values that went through radians.
EPS_DEG = 1e-6


@dataclass(frozen=True)
class ProbeSettings:
    """Currents in mA, angles in motor degrees, times in seconds."""

    hold_current_ma: float = 200.0  # every motor not being probed
    pose_current_ma: float = 350.0  # driving the probed joints to the start pose
    pose_tolerance_deg: float = 10.0  # further off than this = pose not reached
    takeup_current_ma: float = 60.0  # the fixed pull of the takeup method
    start_current_ma: float = 10.0  # breakaway ramp start ...
    step_current_ma: float = 5.0  # ... increment ...
    max_current_ma: float = 300.0  # ... and ceiling
    step_period_s: float = 0.3  # dwell per ramp step
    play_deg: float = 1.0  # motion that counts as "moved"
    move_deg: float = 10.0  # motion that counts as "the joint follows"
    max_travel_deg: float = 40.0  # never drive further than this from the hold pose
    limit_margin_deg: float = 10.0  # ... nor closer than this to a calibrated limit
    stall_deg_per_s: float = 2.0  # takeup: slower than this for stall_hold_s is a stall
    stall_hold_s: float = 0.5
    stroke_current_ma: float = 150.0  # stroke: the current limit the group moves under
    stroke_speed_deg_s: float = 30.0  # ... and how fast the goal advances
    timeout_s: float = 8.0
    sample_period_s: float = 0.05
    return_timeout_s: float = 2.0

    def validate(self) -> None:
        if not 0 < self.start_current_ma <= self.max_current_ma:
            raise ValueError(
                "start_current_ma must be positive and at most max_current_ma"
            )
        if self.step_current_ma <= 0:
            raise ValueError("step_current_ma must be positive")
        if not 0 < self.play_deg < self.move_deg <= self.max_travel_deg:
            raise ValueError("need 0 < play_deg < move_deg <= max_travel_deg")
        if self.takeup_current_ma <= 0 or self.hold_current_ma <= 0:
            raise ValueError("currents must be positive")
        if self.pose_current_ma <= 0 or self.pose_tolerance_deg <= 0:
            raise ValueError("pose_current_ma and pose_tolerance_deg must be positive")
        if self.stroke_current_ma <= 0 or self.stroke_speed_deg_s <= 0:
            raise ValueError("stroke_current_ma and stroke_speed_deg_s must be positive")


@dataclass
class ProbeResult:
    joint: str
    motor_id: int
    direction: int  # +1 / -1 in motor rotation
    sense: str  # "flex" or "extend" for the joint
    method: str
    cap_deg: float  # how far this probe was allowed to go
    free_deg: Optional[float] = None  # takeup: angle turned before the stall
    stalled: Optional[bool] = None  # takeup: False = reached the cap or timed out
    breakaway_ma: Optional[float] = None
    move_ma: Optional[float] = None
    mean_ma: Optional[float] = None  # stroke: mean |current| over the stroke past its start
    peak_ma: Optional[float] = None
    curve: List[List[float]] = dataclasses.field(
        default_factory=list
    )  # [[mA, deg], ...]
    duration_s: float = 0.0
    capped: bool = False                # breakaway: hit the travel cap before move_deg
    skipped: Optional[str] = None
    run: int = 1                        # which pass of ``repeats`` this came from

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        for key in ("free_deg", "breakaway_ma", "move_ma", "mean_ma", "peak_ma", "cap_deg", "duration_s"):
            if d[key] is not None:
                d[key] = round(d[key], 2)
        d["curve"] = [[round(ma, 1), round(deg, 2)] for ma, deg in self.curve]
        return d


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("tension probe progress callback failed")


def probe_joints(config, absent: Sequence[str] = ()) -> List[str]:
    """Every non-wrist joint with a motor, minus ``absent``, in the hand's joint order."""
    skip = set(absent)
    return [
        j
        for j in config.joint_ids
        if j != WRIST and j in config.joint_to_motor_map and j not in skip
    ]


SENSES = ("flex", "extend")
# Joints driven at the same time: every thumb joint on its own (the thumb
# crosses the fingers), and the four fingers together per joint kind, moving
# the same way in joint terms (all abductions to one side, all MCPs flexing,
# ...), as calibration drives them, so no finger ever meets a neighbour.
FINGERS = ("index", "middle", "ring", "pinky")
GROUP_TEMPLATE: Tuple[Tuple[str, ...], ...] = (
    ("thumb_cmc",),
    ("thumb_abd",),
    ("thumb_mcp",),
    ("thumb_dip",),
) + tuple(tuple(f"{finger}_{kind}" for finger in FINGERS) for kind in ("abd", "mcp", "pip"))


def probe_groups(joints: Sequence[str], parallel: bool = True) -> List[List[str]]:
    """``joints`` split into the sets that are driven together (GROUP_TEMPLATE
    order, members in the order given); every joint on its own when
    ``parallel`` is false. Joints the template does not name run alone."""
    if not parallel:
        return [[j] for j in joints]
    wanted = set(joints)
    groups: List[List[str]] = []
    for template in GROUP_TEMPLATE:
        members = [j for j in joints if j in template and j in wanted]
        if members:
            groups.append(members)
    known = {j for g in groups for j in g}
    groups.extend([j] for j in joints if j not in known)
    return groups


def direction_for(config, joint: str, sense: str) -> int:
    """The motor direction that moves ``joint`` in ``sense``."""
    return 1 if joint_sense(config, joint, 1) == sense else -1


def joint_sense(config, joint: str, direction: int) -> str:
    """Which way the joint goes when its motor turns ``direction``: calibration
    records flex as +1 unless the joint is inverted."""
    flex_sign = -1 if config.joint_inversion_dict.get(joint, False) else 1
    return "flex" if direction == flex_sign else "extend"


def travel_cap_deg(
    limits: Optional[List], centre_rad: float, direction: int, settings: ProbeSettings
) -> float:
    """How far (motor degrees) the probe may go in ``direction`` from ``centre_rad``.

    A multi-turn motor reports its position in whichever turn it powered up
    in, while the calibrated limits sit in the turn calibration saw, so the
    centre is first shifted by whole turns onto the limits (as the hand's own
    wrap-offset logic does); otherwise a side with plenty of room reads as
    hundreds of degrees past its limit and is skipped.
    """
    cap = settings.max_travel_deg
    if limits and None not in limits:
        lower, upper = sorted(float(v) for v in limits)
        mid = (lower + upper) / 2
        turn = 2 * math.pi
        centre_rad = centre_rad - round((centre_rad - mid) / turn) * turn
        room = (upper - centre_rad) if direction > 0 else (centre_rad - lower)
        cap = min(cap, math.degrees(room) - settings.limit_margin_deg)
    return cap


class _Clock:
    """Monotonic time and sleeping, swappable for tests."""

    def __init__(self, now=time.monotonic, sleep=time.sleep):
        self.now = now
        self.sleep = sleep


def run_tension_probe(
    hand: "OrcaHand",
    *,
    methods: Sequence[str] = METHODS,
    joints: Optional[Sequence[str]] = None,
    settings: ProbeSettings = ProbeSettings(),
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
    clock: Optional[_Clock] = None,
    pose: str = POSE_MID,
    repeats: int = 1,
    use_limits: bool = True,
    parallel: bool = False,
) -> dict:
    """Probe every finger tendon in both directions with each of ``methods``,
    the whole sequence ``repeats`` times from the same held pose.

    With ``parallel`` the breakaway ramps run in collision-free groups (see
    :func:`probe_groups`), one sense at a time: every motor of the group aims
    the same way in joint terms and the current limit steps up for all of
    them; a motor that has moved is sent home while the others keep ramping.
    Take-up always runs one motor at a time. Events carry ``group`` (the
    joints ramping together).

    With ``use_limits=False`` the calibrated motor limits are ignored: every
    probe may go the full ``max_travel_deg`` and none is skipped near a limit.
    For a check between tensioning rounds, when the last calibration no
    longer describes the hand (pair it with ``pose="hold"``).

    With ``pose="mid"`` (the default) the probed joints are first driven to
    the middle of their range of motion at the hold current, the pose furthest
    from both limits, so every run starts the same way and no probe is skipped
    at a limit; ``pose="neutral"`` uses the config's neutral position instead.
    Both need a calibrated hand and fall back to holding in place
    (``pose_skipped`` event) without one. ``pose="hold"`` holds the hand where
    it stands. Returns::

        {"centre_deg": {joint: motor deg}, "results": [ProbeResult dicts],
         "settings": {...}, "methods": [...], "joints": [...]}

    Progress events: ``probe_started`` / ``probe_sample`` / ``probe_done``
    (``joint``, ``motor``, ``direction``, ``sense``, ``method``, ``index``,
    ``total``; samples carry ``current_ma`` and ``deg``; done carries the
    result) and ``cleanup_failed``. On any exit torque is off and the
    configured control mode and current limit are restored.

    Progress also carries ``pose`` (``probe_plan``), ``pose_skipped`` and
    ``pose_short`` (joints that stayed further than ``pose_tolerance_deg``
    from the start pose, with the motor degrees they are off by).

    Raises:
        ValueError: On an unknown method or inconsistent settings.
        InterruptedError: If ``should_stop`` fires.
    """
    settings.validate()
    if pose not in POSES:
        raise ValueError(f"unknown pose {pose!r}; expected one of {POSES}")
    for method in methods:
        if method not in METHODS:
            raise ValueError(
                f"unknown probe method {method!r}; expected one of {METHODS}"
            )
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731
    clock = clock or _Clock()
    config = hand.config
    absent = [j for j in getattr(hand, "absent_joints", []) if j != WRIST]
    targets = probe_joints(config, absent)
    if joints is not None:
        wanted = set(joints)
        targets = [j for j in targets if j in wanted]
    if not targets:
        raise ValueError("no finger joint with a motor to probe")
    motor_of = {j: int(config.joint_to_motor_map[j]) for j in targets}
    all_present = [
        m
        for m in config.motor_ids
        if m not in set(getattr(hand, "absent_motor_ids", []))
    ]
    idx = config.motor_id_to_idx_dict
    limits = (
        (getattr(getattr(hand, "calibration", None), "motor_limits_dict", {}) or {})
        if use_limits
        else {}
    )
    control_mode = config.control_mode

    def _restore() -> None:
        hand.disable_torque()
        hand.set_max_current(config.max_current)
        hand.set_control_mode(control_mode, all_present)

    def _check() -> None:
        if should_stop():
            raise InterruptedError("tension probe stopped")

    def _currents(probed, ma: float) -> List[float]:
        """Per-motor current limits: ``ma`` for the probed motor(s), hold for the rest."""
        if probed is None:
            probed = ()
        elif isinstance(probed, int):
            probed = (probed,)
        return [
            ma if m in probed else settings.hold_current_ma for m in config.motor_ids
        ]

    if repeats < 1:
        raise ValueError("repeats must be at least 1")
    results: List[ProbeResult] = []
    total = len(targets) * len(DIRECTIONS) * len(methods) * repeats
    index = 0
    try:
        hand.set_control_mode(CURRENT_BASED_POSITION, all_present)
        hand.disable_torque()
        hand.set_max_current(_currents(None, settings.hold_current_ma))
        posed = pose
        if pose in (POSE_MID, POSE_NEUTRAL):
            if getattr(hand, "calibrated", False):
                short = _move_to_pose(hand, targets, all_present, clock, pose, settings, idx)
                if short:
                    _emit(progress_callback, "pose_short", joints=short)
            else:
                posed = POSE_HOLD
                _emit(progress_callback, "pose_skipped",
                      reason="hand not calibrated; holding where it stands")
        centre = read_motor_pos_checked(hand)
        hand.write_motor_pos(all_present, centre[[idx[m] for m in all_present]])
        hand.enable_torque(all_present)
        _emit(
            progress_callback,
            "probe_plan",
            joints=list(targets),
            methods=list(methods),
            pose=posed,
            total=total,
            repeats=repeats,
            centre_deg={
                j: math.degrees(float(centre[idx[motor_of[j]]])) for j in targets
            },
        )

        groups = probe_groups(targets, parallel)
        for run in range(1, repeats + 1):
            for group in groups:
                # One sense at a time for the whole group: every member moves
                # the same way in joint terms, so none meets another.
                for sense in SENSES:
                    for method in methods:
                        members = []
                        for joint in group:
                            motor = motor_of[joint]
                            start = float(centre[idx[motor]])
                            direction = direction_for(config, joint, sense)
                            cap = travel_cap_deg(limits.get(motor), start, direction, settings)
                            index += 1
                            result = ProbeResult(
                                joint, motor, direction, sense, method, round(cap, 2), run=run
                            )
                            meta = dict(
                                joint=joint,
                                motor=motor,
                                direction=direction,
                                sense=sense,
                                method=method,
                                index=index,
                                total=total,
                                run=run,
                                group=list(group),
                            )
                            members.append(_Member(joint, motor, start, direction, cap, result, meta))
                        _check()
                        for mb in members:
                            _emit(progress_callback, "probe_started", **mb.meta)
                            if mb.cap < settings.play_deg + 1.0:
                                mb.result.skipped = "too close to the calibrated motor limit"
                        live = [mb for mb in members if mb.result.skipped is None]
                        t0 = clock.now()
                        if method == TAKEUP:
                            for mb in live:
                                _takeup(hand, mb, settings, _currents, progress_callback,
                                        should_stop, clock, idx)
                                _return_home(hand, [mb], settings, _currents, clock, idx)
                        elif method == STROKE and live:
                            _stroke(hand, live, settings, _currents, progress_callback,
                                    should_stop, clock, idx)
                            _return_home(hand, live, settings, _currents, clock, idx)
                        elif live:
                            _breakaway(hand, live, settings, _currents, progress_callback,
                                       should_stop, clock, idx)
                            _return_home(hand, live, settings, _currents, clock, idx)
                        for mb in members:
                            if mb.result.skipped is None:
                                mb.result.duration_s = clock.now() - t0
                            results.append(mb.result)
                            _emit(progress_callback, "probe_done", result=mb.result.to_dict(), **mb.meta)
    except BaseException:
        try:
            _restore()
        except Exception as e:
            _emit(progress_callback, "cleanup_failed", error=str(e))
            logger.warning("cleanup after aborted tension probe failed: %s", e)
        raise
    _restore()
    calibrated = (getattr(getattr(hand, "calibration", None), "motor_limits_dict", {}) or {})
    return {
        "centre_deg": {
            j: round(math.degrees(float(centre[idx[motor_of[j]]])), 2) for j in targets
        },
        # The calibration in force when the probe ran, for the record even when ignored.
        "limits_deg": {
            j: [round(math.degrees(float(v)), 2) for v in sorted(calibrated[motor_of[j]])]
            for j in targets
            if calibrated.get(motor_of[j]) and None not in calibrated[motor_of[j]]
        },
        "results": [r.to_dict() for r in results],
        "settings": dataclasses.asdict(settings),
        "methods": list(methods),
        "joints": list(targets),
        "repeats": repeats,
        "use_limits": use_limits,
        "parallel": parallel,
    }


def _move_to_pose(
    hand, joints, motors, clock, pose: str, settings: ProbeSettings, idx
) -> Dict[str, float]:
    """Drive ``joints`` to the start pose (mid-ROM or the config's neutral
    position) at ``pose_current_ma`` (stiff joints need more than the hold
    current), let them settle, then drop back to the hold current. Returns
    the joints that ended further than ``pose_tolerance_deg`` from the pose,
    in motor degrees off."""
    hand._compute_wrap_offsets_dict()
    config = hand.config
    probed = {int(config.joint_to_motor_map[j]) for j in joints}
    hand.set_max_current(
        [settings.pose_current_ma if m in probed else settings.hold_current_ma
         for m in config.motor_ids]
    )
    hand.enable_torque(motors)
    if pose == POSE_MID:
        roms = config.joint_roms_dict
        target = {j: (float(roms[j][0]) + float(roms[j][1])) / 2 for j in joints}
    else:
        target = {j: float(config.neutral_position.get(j, 0.0)) for j in joints}
    hand.set_joint_positions(target, num_steps=NEUTRAL_MOVE_STEPS, step_size=NEUTRAL_MOVE_STEP_S)
    clock.sleep(NEUTRAL_SETTLE_S)
    short = _pose_shortfall(hand, target, settings, idx)
    hand.disable_torque(motors)
    hand.set_max_current([settings.hold_current_ma for _ in config.motor_ids])
    clock.sleep(0.1)
    return short


def _pose_shortfall(
    hand, target: Dict[str, float], settings: ProbeSettings, idx
) -> Dict[str, float]:
    """Joints whose motor is further than the tolerance from where the pose
    puts it, by motor degrees — measured on the motors, so it needs no joint
    feedback."""
    try:
        wanted = hand._joint_to_motor_pos(target)
    except Exception:
        return {}
    actual = read_motor_pos_checked(hand)
    out: Dict[str, float] = {}
    for joint in target:
        motor = int(hand.config.joint_to_motor_map[joint])
        goal = wanted[idx[motor]]
        if goal is None:
            continue
        off = abs(math.degrees(float(actual[idx[motor]]) - float(goal)))
        if off > settings.pose_tolerance_deg:
            out[joint] = round(off, 1)
    return out


@dataclass
class _Member:
    """One motor in a probe step: where it starts, how far it may go, its result."""

    joint: str
    motor: int
    start: float  # rad
    direction: int
    cap: float  # deg
    result: ProbeResult
    meta: dict

    @property
    def goal(self) -> float:
        return self.start + self.direction * math.radians(self.cap)

    def displacement(self, pos, idx) -> float:
        return max(0.0, self.direction * math.degrees(float(pos[idx[self.motor]]) - self.start))


def _takeup(hand, mb: _Member, settings, currents, progress_callback, should_stop, clock, idx) -> None:
    result = mb.result
    hand.set_max_current(currents(mb.motor, settings.takeup_current_ma))
    hand.write_motor_pos([mb.motor], np.array([mb.goal]))
    t0 = clock.now()
    last_deg, last_t = 0.0, t0
    slow_since = None
    stalled = False
    deg = 0.0
    while True:
        if should_stop():
            raise InterruptedError("tension probe stopped")
        clock.sleep(settings.sample_period_s)
        deg = mb.displacement(read_motor_pos_checked(hand), idx)
        now = clock.now()
        speed = abs(deg - last_deg) / max(now - last_t, 1e-6)
        last_deg, last_t = deg, now
        _emit(
            progress_callback,
            "probe_sample",
            current_ma=settings.takeup_current_ma,
            deg=deg,
            **mb.meta,
        )
        if deg >= mb.cap - 0.5:
            break
        if speed < settings.stall_deg_per_s:
            slow_since = slow_since if slow_since is not None else now
            if now - slow_since >= settings.stall_hold_s:
                stalled = True
                break
        else:
            slow_since = None
        if now - t0 >= settings.timeout_s:
            break
    result.free_deg = deg
    result.stalled = stalled
    result.curve = [[settings.takeup_current_ma, deg]]


def _breakaway(hand, members: List[_Member], settings, currents, progress_callback,
               should_stop, clock, idx) -> None:
    """Ramp the current limit of every member together; a member that has
    moved ``move_deg`` (or hit its cap) is sent home at the hold current while
    the others keep ramping."""
    ma = settings.start_current_ma
    pending = list(members)
    hand.write_motor_pos([mb.motor for mb in members], np.array([mb.goal for mb in members]))
    while pending:
        if should_stop():
            raise InterruptedError("tension probe stopped")
        hand.set_max_current(currents([mb.motor for mb in pending], ma))
        clock.sleep(settings.step_period_s)
        pos = read_motor_pos_checked(hand)
        finished = []
        for mb in pending:
            result = mb.result
            deg = mb.displacement(pos, idx)
            result.curve.append([ma, deg])
            _emit(progress_callback, "probe_sample", current_ma=ma, deg=deg, **mb.meta)
            if result.breakaway_ma is None and deg >= settings.play_deg - EPS_DEG:
                result.breakaway_ma = ma
            if deg >= settings.move_deg - EPS_DEG:
                result.move_ma = ma
                finished.append(mb)
            elif deg >= mb.cap - 0.5:
                result.capped = True
                finished.append(mb)
        if finished:
            hand.write_motor_pos([mb.motor for mb in finished], np.array([mb.start for mb in finished]))
            pending = [mb for mb in pending if mb not in finished]
        if ma >= settings.max_current_ma:
            break
        ma = min(ma + settings.step_current_ma, settings.max_current_ma)


def _read_state(hand, clock, retries: int = 5):
    """Position and current of every motor in one bus transaction."""
    for _ in range(retries):
        with hand._motor_lock:
            state = hand.motor_client.read_position_velocity_current()
            ok = hand.motor_client.last_read_ok
        if ok:
            return state
        clock.sleep(0.005)
    raise RuntimeError("motor read failed: the motor bus returned no status packets")


def _stroke(hand, members: List[_Member], settings, currents, progress_callback,
            should_stop, clock, idx) -> None:
    """Drive every member together to its cap at the stroke speed under the
    stroke current, sampling each one's current against its displacement."""
    hand.set_max_current(currents([mb.motor for mb in members], settings.stroke_current_ma))
    step = math.radians(settings.stroke_speed_deg_s * settings.sample_period_s)
    goals = {mb.motor: mb.start for mb in members}
    done = {mb.motor: False for mb in members}
    still: Dict[int, tuple] = {}
    deadline = clock.now() + max(mb.cap for mb in members) / settings.stroke_speed_deg_s * 3 + 3.0
    while not all(done.values()):
        if should_stop():
            raise InterruptedError("tension probe stopped")
        moving = [mb for mb in members if not done[mb.motor]]
        for mb in moving:
            target = mb.goal
            goals[mb.motor] = (min(goals[mb.motor] + step, target) if mb.direction > 0
                               else max(goals[mb.motor] - step, target))
        hand.write_motor_pos([mb.motor for mb in moving], np.array([goals[mb.motor] for mb in moving]))
        clock.sleep(settings.sample_period_s)
        state = _read_state(hand, clock)
        now = clock.now()
        for mb in moving:
            pos = float(state.position[idx[mb.motor]])
            ma = abs(float(state.current[idx[mb.motor]]))
            deg = mb.displacement(state.position, idx)
            mb.result.curve.append([ma, deg])
            _emit(progress_callback, "probe_sample", current_ma=ma, deg=deg, **mb.meta)
            at_goal = goals[mb.motor] == mb.goal
            if deg >= mb.cap - 0.5 or (at_goal and abs(pos - mb.goal) < math.radians(1.0)):
                done[mb.motor] = True
                mb.result.capped = True
                continue
            last = still.get(mb.motor)
            if last is None or abs(pos - last[0]) > math.radians(settings.stall_deg_per_s * settings.stall_hold_s):
                still[mb.motor] = (pos, now)
            elif at_goal and now - last[1] >= settings.stall_hold_s:
                done[mb.motor] = True
                mb.result.stalled = True
        if now >= deadline:
            break
    for mb in members:
        result = mb.result
        degs = [d for _, d in result.curve]
        result.free_deg = max(degs) if degs else 0.0
        body = [ma for ma, d in result.curve if d >= STROKE_EDGE * mb.cap]
        if not body:
            body = [ma for ma, _ in result.curve]
        if body:
            result.mean_ma = sum(body) / len(body)
            result.peak_ma = max(body)
            result.breakaway_ma = result.mean_ma
        if result.free_deg < settings.play_deg:
            result.stalled = True


def _return_home(hand, members: List[_Member], settings, currents, clock, idx) -> None:
    """Bring the probed motors back to where they started and leave them holding there."""
    hand.set_max_current(currents(None, settings.hold_current_ma))
    hand.write_motor_pos([mb.motor for mb in members], np.array([mb.start for mb in members]))
    deadline = clock.now() + settings.return_timeout_s
    while clock.now() < deadline:
        clock.sleep(settings.sample_period_s)
        pos = read_motor_pos_checked(hand)
        if all(abs(math.degrees(float(pos[idx[mb.motor]]) - mb.start)) < 1.0 for mb in members):
            break
