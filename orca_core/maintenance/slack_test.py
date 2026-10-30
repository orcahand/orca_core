# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Slack test: how far each finger motor turns while the operator holds its
joint still.

One joint at a time: the operator holds the finger so the joint cannot move,
then the motor winds toward a goal at a low current, first one way and then
the other. With the joint held, everything the spool turns before it stalls
is slack in the tendons; the free rotation between the two stalls is the
joint's slack in motor degrees. A joint whose spool turns more than
``max_free_deg`` (half a turn) fails: calibrating it would unspool the tendon,
and ``blocked`` is set.

No calibration is needed; the test runs on the raw motor positions and leaves
the motors where it found them, torque off. ``hold_callback(joint)`` blocks
until the operator holds the joint and returns False to skip it, or
``HOLD_BACK`` to test the previous joint again; ``review_callback()`` after
the last joint returns ``HOLD_BACK`` to test that one again. A spool has
stalled once its net movement over ``stall_hold_s`` stays under
``stall_deg_per_s``, so encoder jitter on a held motor cannot keep a phase
running; one that has not started moving gets ``start_grace_s`` to break away
first. Like the other maintenance routines: ``progress_callback`` events,
``should_stop``.
"""

from __future__ import annotations

import dataclasses
import logging
import math
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, TYPE_CHECKING

import numpy as np

from ..constants import CURRENT_BASED_POSITION
from .motor_reads import read_motor_pos_checked
from .joint_motion import DIRECTIONS, Clock, finger_joints, joint_sense

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

HoldCallback = Callable[[str], "bool | str"]
ReviewCallback = Callable[[], "bool | str"]

# A hold answer that discards the previous joint's result and tests it again.
HOLD_BACK = "back"

PASS = "pass"
FAIL = "fail"
SKIPPED = "skipped"
# Wound this far past the budget, a spool has shown more slack than allowed.
OVER_MARGIN_DEG = 5.0


@dataclass(frozen=True)
class SlackTestSettings:
    """Currents in mA, angles in motor degrees, times in seconds."""

    current_ma: float = 60.0  # winds the spool through its slack against a held joint
    max_free_deg: float = 180.0  # more free rotation than this fails the joint
    stall_deg_per_s: float = 2.0  # slower than this for stall_hold_s is a stall
    stall_hold_s: float = 0.5
    start_grace_s: float = 3.0  # a motor that has not moved yet is not stalled before this
    phase_timeout_s: float = 10.0
    sample_period_s: float = 0.05
    return_timeout_s: float = 4.0

    def validate(self) -> None:
        if self.current_ma <= 0:
            raise ValueError("current_ma must be positive")
        if not 0 < self.max_free_deg <= 180.0:
            raise ValueError("max_free_deg must be within (0, 180]")


@dataclass
class SlackResult:
    joint: str
    motor_id: int
    plus_deg: float = 0.0  # free rotation with the motor turning positive
    minus_deg: float = 0.0
    plus_sense: str = ""  # which way the joint goes for each motor direction
    minus_sense: str = ""
    skipped: bool = False

    @property
    def total_deg(self) -> float:
        return self.plus_deg + self.minus_deg

    def status(self, settings: SlackTestSettings) -> str:
        if self.skipped:
            return SKIPPED
        return FAIL if self.total_deg > settings.max_free_deg else PASS

    def to_dict(self, settings: SlackTestSettings) -> dict:
        d = dataclasses.asdict(self)
        for key in ("plus_deg", "minus_deg"):
            d[key] = round(d[key], 2)
        d["total_deg"] = round(self.total_deg, 2)
        d["status"] = self.status(settings)
        return d


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("slack test progress callback failed")


def run_slack_test(
    hand: "OrcaHand",
    *,
    joints: Optional[Sequence[str]] = None,
    settings: SlackTestSettings = SlackTestSettings(),
    hold_callback: Optional[HoldCallback] = None,
    review_callback: Optional[ReviewCallback] = None,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
    clock: Optional[Clock] = None,
) -> dict:
    """Test each finger joint in turn while the operator holds it: wind its
    motor each way at ``settings.current_ma`` and record how far it turns.

    ``hold_callback(joint)`` is called before each joint with every motor
    torque off; it returns once the operator holds the joint, False to skip
    it, ``HOLD_BACK`` to test the previous joint again (ignored on the first).
    Without one every joint is tested straight away. ``review_callback()`` is
    called once every joint is done, torque off; ``HOLD_BACK`` discards the
    last joint's result and asks for it again, anything else ends the test.

    Returns::

        {"joints": [...], "results": [SlackResult dicts], "blocked": bool,
         "settings": {...}}

    ``blocked`` is True when any joint turned more than ``max_free_deg``.

    Progress events: ``slack_plan`` (``joints``, ``motors``), ``slack_joint``
    (``joint``, ``motor``, ``index``), ``slack_phase`` (``joint``,
    ``direction``, ``sense``), ``slack_sample`` (``joint``, ``direction``,
    ``deg``), ``slack_phase_done`` (``joint``, ``direction``, ``deg``,
    ``reason``: stall / budget / timeout, ``seconds``, ``returned``: arrived /
    stall / timeout, ``return_s``), ``slack_joint_done`` (the joint's result
    dict), ``slack_joint_back`` (``joint``, the one tested again),
    ``cleanup_failed``. On any exit torque is off and the configured control
    mode and current limit are restored.

    Raises:
        ValueError: On inconsistent settings or no finger motor to test.
        InterruptedError: If ``should_stop`` fires.
    """
    settings.validate()
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731
    clock = clock or Clock()
    config = hand.config
    absent = [j for j in getattr(hand, "absent_joints", []) if j != "wrist"]
    targets = finger_joints(config, absent)
    if joints is not None:
        wanted = set(joints)
        targets = [j for j in targets if j in wanted]
    if not targets:
        raise ValueError("no finger joint with a motor to test")
    motor_of = {j: int(config.joint_to_motor_map[j]) for j in targets}
    all_present = [
        m
        for m in config.motor_ids
        if m not in set(getattr(hand, "absent_motor_ids", []))
    ]
    idx = config.motor_id_to_idx_dict
    control_mode = config.control_mode
    results = {j: SlackResult(j, motor_of[j]) for j in targets}

    def _restore() -> None:
        hand.disable_torque()
        hand.set_max_current(config.max_current)
        hand.set_control_mode(control_mode, all_present)

    def _check() -> None:
        if should_stop():
            raise InterruptedError("slack test stopped")

    def _position(motor: int) -> float:
        return float(read_motor_pos_checked(hand)[idx[motor]])

    try:
        hand.disable_torque()
        hand.set_control_mode(CURRENT_BASED_POSITION, all_present)
        hand.set_max_current([settings.current_ma for _ in config.motor_ids])
        _emit(progress_callback, "slack_plan", joints=list(targets),
              motors=[motor_of[j] for j in targets])
        n = 0
        while True:
            _check()
            if n == len(targets):
                if review_callback is None or review_callback() != HOLD_BACK:
                    break
                n -= 1
                results[targets[n]] = SlackResult(targets[n], motor_of[targets[n]])
                _emit(progress_callback, "slack_joint_back", joint=targets[n])
                continue
            joint = targets[n]
            motor = motor_of[joint]
            _emit(progress_callback, "slack_joint", joint=joint, motor=motor, index=n)
            answer = True if hold_callback is None else hold_callback(joint)
            if answer == HOLD_BACK:
                if n > 0:
                    n -= 1
                    results[targets[n]] = SlackResult(targets[n], motor_of[targets[n]])
                    _emit(progress_callback, "slack_joint_back", joint=targets[n])
                continue
            r = results[joint]
            n += 1
            if not answer:
                r.skipped = True
                _emit(progress_callback, "slack_joint_done", result=r.to_dict(settings))
                continue
            _check()
            start = _position(motor)
            hand.write_motor_pos([motor], np.array([start]))
            hand.enable_torque([motor])
            budget = settings.max_free_deg + OVER_MARGIN_DEG
            for direction in DIRECTIONS:
                sense = joint_sense(config, joint, direction)
                _emit(progress_callback, "slack_phase", joint=joint, direction=direction, sense=sense)
                here = direction * math.degrees(_position(motor) - start)
                hand.write_motor_pos([motor], np.array([start + direction * math.radians(budget)]))
                t0 = clock.now()
                deg, reason = _wind(joint, motor, start, here, direction, budget, settings, progress_callback,
                                    should_stop, clock, _position)
                wound_s = clock.now() - t0
                if direction > 0:
                    r.plus_deg, r.plus_sense = deg, sense
                else:
                    r.minus_deg, r.minus_sense = deg, sense
                t0 = clock.now()
                returned = _return(hand, motor, start, settings, clock, _position)
                _emit(progress_callback, "slack_phase_done", joint=joint, direction=direction,
                      deg=round(deg, 2), reason=reason, seconds=round(wound_s, 2),
                      returned=returned, return_s=round(clock.now() - t0, 2))
                budget -= deg
                if budget <= OVER_MARGIN_DEG:
                    break
            hand.disable_torque([motor])
            _emit(progress_callback, "slack_joint_done", result=r.to_dict(settings))
    except BaseException:
        try:
            _restore()
        except Exception as e:
            _emit(progress_callback, "cleanup_failed", error=str(e))
            logger.warning("cleanup after aborted slack test failed: %s", e)
        raise
    _restore()
    out = [results[j].to_dict(settings) for j in targets]
    return {
        "joints": list(targets),
        "results": out,
        "blocked": any(r["status"] == FAIL for r in out),
        "settings": dataclasses.asdict(settings),
    }


class _StallWatch:
    """Net movement over the last ``stall_hold_s``: under ``stall_deg_per_s``
    over a whole window is a stall, however the readings jitter inside it. Until
    the motor has moved that far from where it started, it has ``start_grace_s``
    to break away before it counts as stalled."""

    def __init__(self, settings: SlackTestSettings, t0: float, deg0: float):
        self._hold_s = settings.stall_hold_s
        self._limit_deg = settings.stall_deg_per_s * settings.stall_hold_s
        self._grace_until = t0 + settings.start_grace_s
        self._deg0 = deg0
        self._started = False
        self._samples: deque = deque([(t0, deg0)])

    def stalled(self, now: float, deg: float) -> bool:
        samples = self._samples
        samples.append((now, deg))
        while len(samples) > 1 and now - samples[1][0] >= self._hold_s:
            samples.popleft()
        self._started = self._started or abs(deg - self._deg0) >= self._limit_deg
        if not self._started:
            return now >= self._grace_until
        if now - samples[0][0] < self._hold_s:
            return False
        degs = [d for _, d in samples]
        return max(degs) - min(degs) < self._limit_deg


def _wind(joint, motor, start, here, direction, budget, settings, progress_callback, should_stop, clock, position):
    """Let the motor wind from ``here`` (its signed offset from ``start`` when the
    goal was set) until it stalls on a taut tendon, turns ``budget`` degrees or
    the phase times out. Returns the degrees turned from ``start`` and which of
    the three ended it."""
    t0 = clock.now()
    watch = _StallWatch(settings, t0, here)
    while True:
        if should_stop():
            raise InterruptedError("slack test stopped")
        clock.sleep(settings.sample_period_s)
        now = clock.now()
        # The watch sees the signed offset: a motor still coming back across `start` is moving.
        raw = direction * math.degrees(position(motor) - start)
        deg = max(0.0, raw)
        _emit(progress_callback, "slack_sample", joint=joint, direction=direction, deg=round(deg, 2))
        if deg >= budget - 1.0:
            return deg, "budget"
        if watch.stalled(now, raw):
            return deg, "stall"
        if now - t0 >= settings.phase_timeout_s:
            return deg, "timeout"


def _return(hand, motor, start, settings, clock, position) -> str:
    """Bring the motor back to where the joint's test started, at the test
    current, until it arrives or stops short. Returns which."""
    hand.write_motor_pos([motor], np.array([start]))
    now = clock.now()
    deadline = now + settings.return_timeout_s
    watch = _StallWatch(settings, now, math.degrees(position(motor) - start))
    while clock.now() < deadline:
        clock.sleep(settings.sample_period_s)
        off = math.degrees(position(motor) - start)
        if abs(off) < 2.0:
            return "arrived"
        if watch.stalled(clock.now(), off):
            return "stall"
    return "timeout"
