# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Tension sweep: the current each motor draws while it slowly walks its joint
from one calibrated limit to the other and back.

One group at a time (see tension_probe.probe_groups: every thumb joint
alone, the four fingers together per joint kind; or one joint at a time with
``parallel`` off), each
motor is driven in position mode at a fixed slow speed between the two motor
limits calibration recorded, flexing first then extending, so a group always
moves the same way in joint terms. The thumb is parked fully extended while a
finger group runs and the fingers are parked straight while a thumb joint
runs; the rest hold at mid-range. Each motor's present current is sampled
against its position the whole way, and
the present current the motor reports is sampled against its position the
whole way. The curve is the load the tendon and skin put on the motor over the
whole travel; its level, compared joint for joint with a reference hand, says
whether a tendon is tighter or looser than it should be, and a stretch of near
zero current after the reversal is slack: the spool turning before the other
tendon takes up.

Needs a calibrated hand (the limits). ``repeats`` passes out and back are
recorded per joint. Interaction-free like the other maintenance routines:
``progress_callback`` events, ``should_stop``.
"""

from __future__ import annotations

import dataclasses
import logging
import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, TYPE_CHECKING

import numpy as np

from ..constants import CURRENT_BASED_POSITION, TINY_SLEEP
from .motor_reads import read_motor_pos_checked
from .tension_probe import (
    POSE_MID,
    SENSES,
    ProbeSettings,
    _Clock,
    _move_to_pose,
    direction_for,
    joint_sense,
    probe_groups,
    probe_joints,
)

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

# Share of the travel at each end left out of the mean: the ramp off one stop
# and the preload onto the other are reported separately.
EDGE_FRACTION = 0.2
# Consecutive samples above the dead-band current that end the dead band.
DEADBAND_CONFIRM = 3


@dataclass(frozen=True)
class SweepSettings:
    """Currents in mA, angles in motor degrees, speeds in motor degrees per second."""

    hold_current_ma: float = 200.0  # every motor not being swept
    pose_current_ma: float = 350.0  # driving the swept joints to mid-range first
    pose_tolerance_deg: float = 10.0
    sweep_current_ma: float = 300.0  # the limit the swept motor may draw
    speed_deg_s: float = 30.0
    sample_period_s: float = 0.05
    limit_margin_deg: float = 2.0  # the sweep turns around this far before each limit
    deadband_ma: float = 25.0  # below this the tendon is not loading the motor
    lag_deg: float = 15.0  # further behind the goal than this ...
    lag_hold_s: float = 2.0  # ... for this long = stalled short of the limit
    settle_s: float = 0.5  # pause at each end before turning around
    stall_deg: float = 0.5  # goal reached, motor moving less than this ...
    stall_hold_s: float = 0.5  # ... for this long = at the stop, done
    park_margin_deg: float = 5.0  # a parked joint stops this far inside its limit

    def validate(self) -> None:
        for name in ("hold_current_ma", "pose_current_ma", "sweep_current_ma", "speed_deg_s"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.sample_period_s <= 0:
            raise ValueError("sample_period_s must be positive")
        if self.limit_margin_deg < 0 or self.deadband_ma < 0:
            raise ValueError("limit_margin_deg and deadband_ma must not be negative")


@dataclass
class SweepResult:
    joint: str
    motor_id: int
    direction: int  # +1 / -1 in motor rotation
    sense: str  # flex / extend for the joint
    run: int
    travel_deg: float  # the calibrated travel of the joint, motor degrees
    start_deg: float = 0.0  # where the sweep began, from the lower limit
    end_deg: float = 0.0  # where it ended
    mean_ma: Optional[float] = None  # mean |current| over the central travel
    end_ma: Optional[float] = None  # mean |current| over the last fifth, into the stop
    peak_ma: Optional[float] = None
    deadband_deg: Optional[float] = None  # travel after the reversal before the motor loaded
    short_deg: float = 0.0  # how far short of the far limit it stalled
    samples: List[List[float]] = dataclasses.field(default_factory=list)  # [[deg, mA], ...]
    duration_s: float = 0.0
    skipped: Optional[str] = None

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        for key in ("travel_deg", "start_deg", "end_deg", "mean_ma", "end_ma",
                    "peak_ma", "deadband_deg", "short_deg", "duration_s"):
            if d[key] is not None:
                d[key] = round(d[key], 2)
        d["samples"] = [[round(deg, 2), round(ma, 1)] for deg, ma in self.samples]
        return d


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("tension sweep progress callback failed")


def summarize(samples: List[List[float]], direction: int, travel_deg: float,
              deadband_ma: float, start_deg: Optional[float] = None) -> dict:
    """Level and shape of one sweep's current curve.

    ``samples`` are ``[deg from the lower limit, signed mA]`` in time order.
    The mean leaves out ``EDGE_FRACTION`` of the travel at each end; ``end_ma``
    is the mean over the last fifth before the far limit; the dead band is the
    travel from ``start_deg`` (the reversal; the first sample without it) to
    the first of ``DEADBAND_CONFIRM`` consecutive samples above ``deadband_ma``.
    """
    if not samples:
        return {"mean_ma": None, "end_ma": None, "peak_ma": None, "deadband_deg": None}
    degs = np.array([s[0] for s in samples], dtype=float)
    mas = np.abs(np.array([s[1] for s in samples], dtype=float))
    lo, hi = EDGE_FRACTION * travel_deg, (1 - EDGE_FRACTION) * travel_deg
    central = mas[(degs >= lo) & (degs <= hi)]
    far = mas[degs >= hi] if direction > 0 else mas[degs <= lo]
    origin = degs[0] if start_deg is None else start_deg
    deadband = None
    above = 0
    for i, ma in enumerate(mas):
        above = above + 1 if ma >= deadband_ma else 0
        if above >= DEADBAND_CONFIRM:
            deadband = abs(degs[i - DEADBAND_CONFIRM + 1] - origin)
            break
    if deadband is None:
        deadband = abs(degs[-1] - origin)
    return {
        "mean_ma": float(central.mean()) if central.size else float(mas.mean()),
        "end_ma": float(far.mean()) if far.size else None,
        "peak_ma": float(mas.max()),
        "deadband_deg": float(deadband),
    }


def _read_state_checked(hand, retries: int = 5, retry_interval: float = TINY_SLEEP, clock=None):
    """Position and current in one bus transaction, rejecting a stale read."""
    for _ in range(retries):
        with hand._motor_lock:
            state = hand.motor_client.read_position_velocity_current()
            read_ok = hand.motor_client.last_read_ok
        if read_ok:
            return state
        (clock.sleep if clock else __import__("time").sleep)(retry_interval)
    raise RuntimeError("motor read failed: the motor bus returned no status packets")


def run_tension_sweep(
    hand: "OrcaHand",
    *,
    joints: Optional[Sequence[str]] = None,
    settings: SweepSettings = SweepSettings(),
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
    clock: Optional[_Clock] = None,
    repeats: int = 1,
    parallel: bool = True,
) -> dict:
    """Sweep every finger joint limit to limit and back ``repeats`` times,
    one joint kind at a time (``parallel``) with the others holding, recording
    current against position per motor.

    Returns::

        {"joints": [...], "limits_deg": {joint: [lower, upper]}, "travel_deg": {joint: deg},
         "results": [SweepResult dicts], "settings": {...}, "repeats": n}

    Progress events: ``sweep_plan`` (``joints``, ``total``, ``repeats``,
    ``travel_deg``, ``groups``, ``parked`` per group), ``sweep_started`` / ``sweep_sample`` /
    ``sweep_done`` (``joint``, ``motor``, ``direction``, ``sense``, ``run``,
    ``index``, ``total``, ``group``; samples carry ``deg`` and ``current_ma``;
    done carries the whole result, samples included), ``pose_short`` and
    ``cleanup_failed``. A sweep ends when the motor is within 1° of the far
    limit or has stopped moving there (``stall_deg`` over ``stall_hold_s``),
    so no motor sits against a stop at the sweep current. On any
    exit torque is off and the configured control mode and current limit are
    restored.

    Raises:
        ValueError: On inconsistent settings, no joint to sweep, or an
            uncalibrated hand.
        InterruptedError: If ``should_stop`` fires.
    """
    settings.validate()
    if repeats < 1:
        raise ValueError("repeats must be at least 1")
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731
    clock = clock or _Clock()
    config = hand.config
    absent = [j for j in getattr(hand, "absent_joints", []) if j != "wrist"]
    targets = probe_joints(config, absent)
    if joints is not None:
        wanted = set(joints)
        targets = [j for j in targets if j in wanted]
    if not targets:
        raise ValueError("no finger joint with a motor to sweep")
    limits = getattr(getattr(hand, "calibration", None), "motor_limits_dict", {}) or {}
    motor_of = {j: int(config.joint_to_motor_map[j]) for j in targets}
    missing = [j for j in targets if not limits.get(motor_of[j]) or None in limits[motor_of[j]]]
    if missing or not getattr(hand, "calibrated", False):
        raise ValueError(
            "the tension sweep needs a calibrated hand: no motor limits for "
            + (", ".join(missing) if missing else "this hand")
        )
    all_present = [
        m
        for m in config.motor_ids
        if m not in set(getattr(hand, "absent_motor_ids", []))
    ]
    idx = config.motor_id_to_idx_dict
    control_mode = config.control_mode
    pose_settings = ProbeSettings(
        hold_current_ma=settings.hold_current_ma,
        pose_current_ma=settings.pose_current_ma,
        pose_tolerance_deg=settings.pose_tolerance_deg,
    )

    def _restore() -> None:
        hand.disable_torque()
        hand.set_max_current(config.max_current)
        hand.set_control_mode(control_mode, all_present)

    def _check() -> None:
        if should_stop():
            raise InterruptedError("tension sweep stopped")

    def _currents(swept, ma: float) -> List[float]:
        swept = set(swept or ())
        return [ma if m in swept else settings.hold_current_ma for m in config.motor_ids]

    travel = {
        j: math.degrees(abs(float(limits[motor_of[j]][1]) - float(limits[motor_of[j]][0])))
        for j in targets
    }
    results: List[SweepResult] = []
    total = len(targets) * len(SENSES) * repeats
    index = 0
    try:
        hand.set_control_mode(CURRENT_BASED_POSITION, all_present)
        hand.disable_torque()
        hand.set_max_current(_currents(None, settings.hold_current_ma))
        short = _move_to_pose(hand, targets, all_present, clock, POSE_MID, pose_settings, idx)
        if short:
            _emit(progress_callback, "pose_short", joints=short)
        centre = read_motor_pos_checked(hand)
        hand.write_motor_pos(all_present, centre[[idx[m] for m in all_present]])
        hand.enable_torque(all_present)
        groups = probe_groups(targets, parallel)
        # Every calibrated motor's limits in the turn it reports, swept or not,
        # so the thumb and the fingers can be parked out of each other's way.
        known: Dict[str, _Motor] = {}
        for joint, motor in ((j, int(config.joint_to_motor_map[j])) for j in config.joint_ids
                             if j != "wrist" and j in config.joint_to_motor_map and motor_in(j, limits, config)):
            here = float(centre[idx[motor]])
            lower, upper = _limits_in_turn(limits[motor], here)
            flex_dir = direction_for(config, joint, "flex")
            known[joint] = _Motor(joint, motor, here, lower,
                                  lower + math.radians(settings.limit_margin_deg),
                                  upper - math.radians(settings.limit_margin_deg), flex_dir)
        parked_plan = [{j: pose for j, pose in parking(group, known).items()} for group in groups]
        _emit(
            progress_callback,
            "sweep_plan",
            joints=list(targets),
            total=total,
            repeats=repeats,
            travel_deg={j: round(travel[j], 2) for j in targets},
            groups=[list(g) for g in groups],
            parked=parked_plan,
        )
        for group, parked in zip(groups, parked_plan):
            members: List[_Motor] = []
            for joint in group:
                mb = known.get(joint)
                if mb is None or mb.hi <= mb.lo:
                    for run in range(1, repeats + 1):
                        for sense in SENSES:
                            index += 1
                            result = SweepResult(joint, motor_of[joint], direction_for(config, joint, sense), sense,
                                                 run, travel[joint], skipped="calibrated travel narrower than the margins")
                            results.append(result)
                            _emit(progress_callback, "sweep_done", result=result.to_dict(),
                                  index=index, total=total, group=list(group))
                    continue
                members.append(mb)
            if not members:
                continue
            # Park the others out of the way at the pose current, then hold them there.
            park = {j: known[j] for j in parked if j in known and j not in group}
            if park:
                hand.set_max_current(_currents([mb.motor for mb in park.values()], settings.pose_current_ma))
                pos_now = read_motor_pos_checked(hand)
                _walk(hand, {mb.motor: (float(pos_now[idx[mb.motor]]), mb.park(parked[j], settings))
                             for j, mb in park.items()}, settings, should_stop, clock, idx)
                hand.set_max_current(_currents(None, settings.hold_current_ma))
            hand.set_max_current(_currents([mb.motor for mb in members], settings.sweep_current_ma))
            # Approach the extend-side turnaround first, then every pass is limit to limit.
            pos_now = read_motor_pos_checked(hand)
            _walk(hand, {mb.motor: (float(pos_now[idx[mb.motor]]), mb.end("extend")) for mb in members},
                  settings, should_stop, clock, idx)
            clock.sleep(settings.settle_s)
            for run in range(1, repeats + 1):
                for sense in SENSES:
                    _check()
                    pos_now = read_motor_pos_checked(hand)
                    pending: dict = {}
                    for mb in members:
                        index += 1
                        direction = mb.flex_dir if sense == "flex" else -mb.flex_dir
                        meta = dict(joint=mb.joint, motor=mb.motor, direction=direction, sense=sense,
                                    run=run, index=index, total=total, group=[m.joint for m in members])
                        _emit(progress_callback, "sweep_started", **meta)
                        result = SweepResult(mb.joint, mb.motor, direction, sense, run, travel[mb.joint])
                        start_pos = float(pos_now[idx[mb.motor]])
                        pending[mb.motor] = (mb, result, meta, start_pos, mb.end(sense))
                    t0 = clock.now()
                    walked = _walk(
                        hand, {m: (p[3], p[4]) for m, p in pending.items()}, settings, should_stop, clock, idx,
                        on_sample=lambda m, deg, ma: _emit(progress_callback, "sweep_sample", deg=deg,
                                                           current_ma=ma, **pending[m][2]),
                        lower={m: p[0].lower for m, p in pending.items()},
                    )
                    for m, (mb, result, meta, start_pos, target) in pending.items():
                        samples, end_pos, short_deg = walked[m]
                        result.duration_s = clock.now() - t0
                        result.start_deg = math.degrees(start_pos - mb.lower)
                        result.end_deg = math.degrees(end_pos - mb.lower)
                        result.short_deg = short_deg
                        result.samples = samples
                        for k, v in summarize(samples, result.direction, travel[mb.joint], settings.deadband_ma,
                                              result.start_deg).items():
                            setattr(result, k, v)
                        results.append(result)
                        _emit(progress_callback, "sweep_done", result=result.to_dict(), **meta)
                    clock.sleep(settings.settle_s)
            # Back to the middle, holding, before the next group.
            pos_now = read_motor_pos_checked(hand)
            _walk(hand, {mb.motor: (float(pos_now[idx[mb.motor]]), mb.here) for mb in members},
                  settings, should_stop, clock, idx)
            hand.set_max_current(_currents(None, settings.hold_current_ma))
    except BaseException:
        try:
            _restore()
        except Exception as e:
            _emit(progress_callback, "cleanup_failed", error=str(e))
            logger.warning("cleanup after aborted tension sweep failed: %s", e)
        raise
    _restore()
    return {
        "joints": list(targets),
        "limits_deg": {
            j: [round(math.degrees(float(v)), 2) for v in sorted(limits[motor_of[j]])] for j in targets
        },
        "travel_deg": {j: round(travel[j], 2) for j in targets},
        "results": [r.to_dict() for r in results],
        "settings": dataclasses.asdict(settings),
        "repeats": repeats,
        "parallel": parallel,
    }


def _limits_in_turn(limits, here: float):
    """The calibrated limits shifted by whole turns onto the turn the motor
    reports its position in (a multi-turn motor powers up in any turn)."""
    lower, upper = sorted(float(v) for v in limits)
    mid = (lower + upper) / 2
    turn = 2 * math.pi
    shift = round((here - mid) / turn) * turn
    return lower + shift, upper + shift


def motor_in(joint: str, limits: dict, config) -> bool:
    motor = int(config.joint_to_motor_map[joint])
    return bool(limits.get(motor)) and None not in limits[motor]


THUMB = "thumb"


def parking(group: Sequence[str], known: Dict[str, "_Motor"]) -> Dict[str, str]:
    """Where every other calibrated joint is parked while ``group`` sweeps:
    the thumb straight while a finger group runs, the fingers straight (their
    abductions at mid) while a thumb joint runs; ``"extend"`` or ``"mid"``."""
    thumb_group = all(j.startswith(THUMB) for j in group)
    out: Dict[str, str] = {}
    for joint in known:
        if joint in group:
            continue
        if thumb_group:
            out[joint] = "mid" if joint.endswith("_abd") and not joint.startswith(THUMB) else "extend"
        else:
            out[joint] = "extend" if joint.startswith(THUMB) else "mid"
    return out


@dataclass
class _Motor:
    """One motor of a sweep group and its calibrated limits in the turn it reports."""

    joint: str
    motor: int
    here: float  # rad, the mid pose it returns to
    lower: float  # rad, the calibrated lower limit
    lo: float  # rad, the turnaround points inside the margins
    hi: float
    flex_dir: int = 1  # motor direction that flexes the joint

    def end(self, sense: str) -> float:
        """The turnaround point on the ``sense`` side."""
        flex_end = self.hi if self.flex_dir > 0 else self.lo
        return flex_end if sense == "flex" else (self.lo if self.flex_dir > 0 else self.hi)

    def park(self, pose: str, settings: "SweepSettings") -> float:
        if pose == "mid":
            return self.here
        extend = self.end("extend")
        return extend + self.flex_dir * math.radians(settings.park_margin_deg - settings.limit_margin_deg)


def _walk(hand, legs: Dict[int, tuple], settings, should_stop, clock, idx, on_sample=None, lower=None):
    """Drive each motor in ``legs`` (``motor: (start, target)``) toward its
    target at the sweep speed, one goal step per sample period for all of them
    at once, reading position and current each step. A motor is done when it
    is within 1° of its target, has stopped moving there (``stall_deg`` over
    ``stall_hold_s``) or lags the goal by ``lag_deg`` for ``lag_hold_s``. With
    ``lower`` (``motor: rad``) returns ``{motor: (samples, end_pos, short_deg)}``
    with samples in degrees from that limit; samples stop once the motor is done."""
    step = math.radians(settings.speed_deg_s * settings.sample_period_s)
    state = {}
    longest = 0.0
    for motor, (start, target) in legs.items():
        state[motor] = dict(start=start, target=target, direction=1 if target >= start else -1,
                            goal=start, pos=start, samples=[], done=False,
                            lag_since=None, still_since=None, still_pos=None)
        longest = max(longest, abs(target - start))
    deadline = clock.now() + longest / math.radians(settings.speed_deg_s) * 3 + 5.0
    while True:
        if should_stop():
            raise InterruptedError("tension sweep stopped")
        moving = [m for m, st in state.items() if not st["done"]]
        if not moving:
            break
        for m in moving:
            st = state[m]
            st["goal"] = (min(st["goal"] + step, st["target"]) if st["direction"] > 0
                          else max(st["goal"] - step, st["target"]))
        hand.write_motor_pos(moving, np.array([state[m]["goal"] for m in moving]))
        clock.sleep(settings.sample_period_s)
        read = _read_state_checked(hand, clock=clock)
        now = clock.now()
        for m in moving:
            st = state[m]
            pos = float(read.position[idx[m]])
            ma = float(read.current[idx[m]])
            st["pos"] = pos
            if lower is not None:
                deg = math.degrees(pos - lower[m])
                st["samples"].append([deg, ma])
                if on_sample is not None:
                    on_sample(m, round(deg, 2), round(ma, 1))
            at_goal = st["goal"] == st["target"]
            if at_goal and abs(pos - st["target"]) < math.radians(1.0):
                st["done"] = True
                continue
            if at_goal:
                # The goal is at the limit; a motor that no longer moves is on its stop.
                if st["still_pos"] is None or abs(pos - st["still_pos"]) > math.radians(settings.stall_deg):
                    st["still_pos"], st["still_since"] = pos, now
                elif now - st["still_since"] >= settings.stall_hold_s:
                    st["done"] = True
                    continue
            if abs(st["goal"] - pos) > math.radians(settings.lag_deg):
                st["lag_since"] = st["lag_since"] if st["lag_since"] is not None else now
                if now - st["lag_since"] >= settings.lag_hold_s:
                    st["done"] = True
            else:
                st["lag_since"] = None
        if now >= deadline:
            break
    return {
        m: (st["samples"], st["pos"], max(0.0, math.degrees(st["direction"] * (st["target"] - st["pos"]))))
        for m, st in state.items()
    }
