# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Slack test: how far each finger motor turns at a current too low to move a
tensioned joint.

Every finger motor at once is given a goal half a turn away at a few mA. A
tendon that is taut stops the spool at once; a spool that turns is winding up
slack, and the angle it turns before stalling is that slack in motor degrees.
Both motor directions are tried, so the result per joint is the free rotation
of its spool between the two tendons going taut. A motor that completes the
whole half turn has so much slack that calibration would unspool the tendon:
the report marks it and ``blocked`` is set.

No calibration is needed; the test runs on the raw motor positions and leaves
the motors where it found them, torque off. Interaction-free like the other
maintenance routines: ``progress_callback`` events, ``should_stop``.
"""

from __future__ import annotations

import dataclasses
import logging
import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, TYPE_CHECKING

import numpy as np

from ..constants import CURRENT_BASED_POSITION
from .motor_reads import read_motor_pos_checked
from .tension_probe import DIRECTIONS, _Clock, joint_sense, probe_joints

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

TAUT = "taut"
SLACK = "slack"
UNSPOOL = "unspool"
# A turn that got within this of the half-turn goal did not stall on a tendon.
CAP_MARGIN_DEG = 5.0


@dataclass(frozen=True)
class SlackTestSettings:
    """Currents in mA, angles in motor degrees, times in seconds."""

    current_ma: float = 20.0  # below what a tensioned joint needs to move at all
    max_turn_deg: float = 180.0  # never wind further than this per direction
    slack_tolerance_deg: float = 5.0  # less free rotation than this counts as taut
    stall_deg_per_s: float = 2.0  # slower than this for stall_hold_s is a stall
    stall_hold_s: float = 0.5
    phase_timeout_s: float = 10.0
    sample_period_s: float = 0.05
    return_timeout_s: float = 4.0

    def validate(self) -> None:
        if self.current_ma <= 0:
            raise ValueError("current_ma must be positive")
        if not 0 < self.max_turn_deg <= 180.0:
            raise ValueError("max_turn_deg must be within (0, 180]")
        if self.slack_tolerance_deg < 0:
            raise ValueError("slack_tolerance_deg must not be negative")


@dataclass
class SlackResult:
    joint: str
    motor_id: int
    plus_deg: float = 0.0  # free rotation with the motor turning positive
    minus_deg: float = 0.0
    plus_sense: str = ""  # which way the joint goes for each motor direction
    minus_sense: str = ""
    plus_capped: bool = False  # reached the half turn: no tendon went taut
    minus_capped: bool = False

    @property
    def total_deg(self) -> float:
        return self.plus_deg + self.minus_deg

    def status(self, settings: SlackTestSettings) -> str:
        if self.plus_capped or self.minus_capped:
            return UNSPOOL
        return SLACK if self.total_deg >= settings.slack_tolerance_deg else TAUT

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
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
    clock: Optional[_Clock] = None,
) -> dict:
    """Wind every finger motor half a turn each way at ``settings.current_ma``
    and record how far each one got before its tendon stopped it.

    Returns::

        {"joints": [...], "results": [SlackResult dicts], "blocked": bool,
         "settings": {...}}

    ``blocked`` is True when any motor completed the half turn in either
    direction: calibrating such a hand would unspool the tendon.

    Progress events: ``slack_plan`` (``joints``, ``motors``, ``total``),
    ``slack_phase`` (``direction``, ``senses`` per joint), ``slack_sample``
    (``direction``, ``deg`` per joint), ``slack_phase_done`` (``direction``,
    ``deg`` and ``capped`` per joint), ``cleanup_failed``. On any exit torque is
    off and the configured control mode and current limit are restored.

    Raises:
        ValueError: On inconsistent settings or no finger motor to test.
        InterruptedError: If ``should_stop`` fires.
    """
    settings.validate()
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
        raise ValueError("no finger joint with a motor to test")
    motor_of = {j: int(config.joint_to_motor_map[j]) for j in targets}
    motors = [motor_of[j] for j in targets]
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

    def _positions() -> Dict[str, float]:
        pos = read_motor_pos_checked(hand)
        return {j: float(pos[idx[motor_of[j]]]) for j in targets}

    try:
        hand.set_control_mode(CURRENT_BASED_POSITION, all_present)
        hand.disable_torque()
        hand.set_max_current([settings.current_ma for _ in config.motor_ids])
        start = _positions()
        hand.write_motor_pos(motors, np.array([start[j] for j in targets]))
        hand.enable_torque(motors)
        _emit(
            progress_callback,
            "slack_plan",
            joints=list(targets),
            motors=list(motors),
            total=len(DIRECTIONS),
            start_deg={j: round(math.degrees(start[j]), 2) for j in targets},
        )
        for direction in DIRECTIONS:
            _check()
            senses = {j: joint_sense(config, j, direction) for j in targets}
            _emit(progress_callback, "slack_phase", direction=direction, senses=senses)
            base = _positions()
            goals = np.array(
                [base[j] + direction * math.radians(settings.max_turn_deg) for j in targets]
            )
            hand.write_motor_pos(motors, goals)
            deg, capped = _wind(
                hand, targets, base, direction, settings, progress_callback, should_stop, clock, _positions
            )
            for j in targets:
                r = results[j]
                if direction > 0:
                    r.plus_deg, r.plus_capped, r.plus_sense = deg[j], capped[j], senses[j]
                else:
                    r.minus_deg, r.minus_capped, r.minus_sense = deg[j], capped[j], senses[j]
            _emit(
                progress_callback,
                "slack_phase_done",
                direction=direction,
                deg={j: round(deg[j], 2) for j in targets},
                capped=dict(capped),
            )
            _return(hand, motors, targets, start, settings, clock, _positions)
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
        "blocked": any(r["status"] == UNSPOOL for r in out),
        "settings": dataclasses.asdict(settings),
    }


def _wind(
    hand, targets, base, direction, settings, progress_callback, should_stop, clock, positions
):
    """Let every motor wind toward its goal until each has stalled, reached the
    half turn or the phase timed out. Returns the degrees turned and whether
    each reached the cap."""
    t0 = clock.now()
    deg = {j: 0.0 for j in targets}
    capped = {j: False for j in targets}
    done = {j: False for j in targets}
    last_deg = dict(deg)
    last_t = t0
    slow_since: Dict[str, Optional[float]] = {j: None for j in targets}
    while not all(done.values()):
        if should_stop():
            raise InterruptedError("slack test stopped")
        clock.sleep(settings.sample_period_s)
        now = clock.now()
        pos = positions()
        dt = max(now - last_t, 1e-6)
        for j in targets:
            if done[j]:
                continue
            deg[j] = max(0.0, direction * math.degrees(pos[j] - base[j]))
            speed = abs(deg[j] - last_deg[j]) / dt
            last_deg[j] = deg[j]
            if deg[j] >= settings.max_turn_deg - CAP_MARGIN_DEG:
                capped[j] = True
                done[j] = True
            elif speed < settings.stall_deg_per_s:
                slow_since[j] = slow_since[j] if slow_since[j] is not None else now
                if now - slow_since[j] >= settings.stall_hold_s:
                    done[j] = True
            else:
                slow_since[j] = None
        last_t = now
        _emit(
            progress_callback,
            "slack_sample",
            direction=direction,
            deg={j: round(deg[j], 2) for j in targets},
        )
        if now - t0 >= settings.phase_timeout_s:
            break
    return deg, capped


def _return(hand, motors, targets, start, settings, clock, positions) -> None:
    """Bring every motor back to where the test found it, at the test current."""
    hand.write_motor_pos(motors, np.array([start[j] for j in targets]))
    deadline = clock.now() + settings.return_timeout_s
    while clock.now() < deadline:
        clock.sleep(settings.sample_period_s)
        pos = positions()
        if all(abs(math.degrees(pos[j] - start[j])) < 2.0 for j in targets):
            break
