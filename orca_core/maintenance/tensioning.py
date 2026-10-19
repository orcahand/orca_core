# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Tendon tensioning and seating routines for an ORCA hand.

Interaction-free like the other maintenance routines: progress is reported
through ``progress_callback({"event": ..., ...})`` and cooperative
cancellation through ``should_stop()``. The usual entry points are
:meth:`OrcaHand.tension` and :meth:`OrcaHand.jitter`, which wire both to the
hand's background-task plumbing.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Callable, List, Optional, TYPE_CHECKING, Sequence

import numpy as np

from ..constants import CURRENT_BASED_POSITION, WRIST
from .motor_reads import read_motor_pos_checked

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

# How far the finger motors turn counterclockwise (the positive motor direction) under the tensioning
# current, loading the bottom spools so the ratchet takes up more slack; each
# "pull more" adds TENSION_EXTRA_DEG. The pull is ramped at PULL_SPEED_DEG_S.
TENSION_OFFSET_DEG = 15.0
TENSION_EXTRA_DEG = 15.0
# A pull ends when the motors stop moving, not when they reach the commanded
# offset: a taut tendon stalls short and would never arrive. Settled means no
# motor moved more than SETTLE_THRESHOLD_RAD for SETTLE_HOLD_S.
SETTLE_THRESHOLD_RAD = 0.01
SETTLE_HOLD_S = 0.5
SETTLE_TIMEOUT_S = 5.0
PULL_SPEED_DEG_S = 10.0
PULL_STEP_S = 0.05


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the routine."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("tension progress callback failed")


def _read_motor_pos_or_none(hand: "OrcaHand") -> Optional[np.ndarray]:
    """A checked position read that yields ``None`` while the bus stays silent."""
    try:
        return read_motor_pos_checked(hand)
    except RuntimeError as exc:
        logger.warning("%s; skipping this winding tick", exc)
        return None


def finger_motor_ids(config, absent: Sequence[int] = ()) -> List[int]:
    """Every finger motor, minus ``absent`` (the ones that did not answer at connect)."""
    skip = set(absent)
    return [m for joint, m in config.joint_to_motor_map.items()
            if WRIST not in joint.lower() and m in config.motor_ids and m not in skip]


def uncapped_currents(hand: "OrcaHand") -> List[float]:
    """Each motor's own goal-current ceiling, in motor order.

    The hold is a position loop: at its goal a motor draws nothing, so the
    ceiling only binds while something pushes the hand off that goal. Capping
    it at ``max_current`` instead caps how hard the hand can refuse to be
    ratcheted, which on a family that spends a few hundred mA before producing
    any torque leaves very little to refuse with.
    """
    ceilings = hand.motor_client.read_current_limits()
    capped = float(hand.config.max_current)
    fingers = set(finger_motor_ids(hand.config))
    # Only the motors a ratchet is applied to. The wrist is never tensioned by
    # hand, and its ceiling is far higher than a finger's, so it stays capped.
    return [float(ceilings.get(m) or capped) if m in fingers else capped
            for m in hand.config.motor_ids]


def settle_at_rest(
    hand: "OrcaHand",
    motor_ids: List[int],
    *,
    should_stop: Optional[ShouldStop] = None,
) -> np.ndarray:
    """Wait for ``motor_ids`` to stop moving, then command exactly where they are.

    Re-baselining the goal onto the position actually reached is what makes an
    uncapped hold safe. A motor left commanded somewhere it cannot get to holds
    a standing error, and with the ceiling raised it will sit at that ceiling
    indefinitely rather than for the moment it takes to ratchet.
    """
    deadline = time.time() + SETTLE_TIMEOUT_S
    idx = [hand.config.motor_id_to_idx_dict[m] for m in motor_ids]
    prev = _read_motor_pos_or_none(hand)
    still_since = None
    while time.time() < deadline:
        if should_stop is not None and should_stop():
            break
        time.sleep(0.05)
        now = _read_motor_pos_or_none(hand)
        if now is None or prev is None:
            prev = now if now is not None else prev
            continue
        if np.max(np.abs(now[idx] - prev[idx])) < SETTLE_THRESHOLD_RAD:
            if still_since is None:
                still_since = time.time()
            elif time.time() - still_since >= SETTLE_HOLD_S:
                break
        else:
            still_since = None
        prev = now
    else:
        logger.warning("motors still moving after %.1fs; holding them where they are now",
                       SETTLE_TIMEOUT_S)
    return hold_here(hand, motor_ids)


def hold_here(hand: "OrcaHand", motor_ids: List[int]) -> np.ndarray:
    """Command ``motor_ids`` to the position they are already in.

    Without this, ``enable_torque`` servos each motor to whatever its goal
    register happened to hold, which is stale from whatever last commanded it:
    a motor whose stale goal is elsewhere drives toward it instead of holding.
    """
    pos = read_motor_pos_checked(hand)
    goals = pos[[hand.config.motor_id_to_idx_dict[m] for m in motor_ids]]
    hand.write_motor_pos(motor_ids, goals)
    # Holding takes torque: a caller that jumps straight here can arrive with
    # it off, and writing a goal to a limp motor holds nothing.
    hand.enable_torque(motor_ids)
    hand.write_motor_pos(motor_ids, goals)
    return goals


def pull_tendons(
    hand: "OrcaHand",
    motor_ids: List[int],
    offset_deg: float,
    *,
    start: Optional[np.ndarray] = None,
    should_stop: Optional[ShouldStop] = None,
) -> np.ndarray:
    """Slowly turn ``motor_ids`` ``offset_deg`` counterclockwise and hold there, at the current limit.

    Starts from ``start`` (the goals of an earlier pull, so a stalled motor still
    gains the full extra) or else from where the motors stand. Returns the goals.
    """
    if start is None:
        pos = read_motor_pos_checked(hand)
        start = pos[[hand.config.motor_id_to_idx_dict[m] for m in motor_ids]]
        hand.write_motor_pos(motor_ids, start)
    hand.enable_torque(motor_ids)
    offset = math.radians(offset_deg)
    steps = max(1, math.ceil(abs(offset_deg) / (PULL_SPEED_DEG_S * PULL_STEP_S)))
    goals = start
    for step in range(1, steps + 1):
        if should_stop is not None and should_stop():
            break
        goals = start + offset * step / steps
        hand.write_motor_pos(motor_ids, goals)
        if step < steps:
            time.sleep(PULL_STEP_S)
    return goals


def run_tension(
    hand: "OrcaHand",
    move_motors: bool = True,
    *,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
    offset_deg: float = TENSION_OFFSET_DEG,
    pull_more: Optional[Callable[[], bool]] = None,
    extra_deg: float = TENSION_EXTRA_DEG,
    uncapped: bool = True,
) -> None:
    """Hold motors under current so tendons can be tensioned by hand.

    Optionally pre-conditions the tendons first by winding each direction
    until the motors stall. Holds until ``should_stop`` returns ``True``.
    On any exit — cooperative stop or exception — torque is disabled and the
    configured control mode and current limit are restored.

    Args:
        hand: A connected :class:`~orca_core.OrcaHand`.
        move_motors: When ``True``, wind the tendons taut before holding.
        offset_deg: How far the finger motors slowly turn counterclockwise once holding,
            so the bottom spools are loaded while they are ratcheted; 0 holds still.
        pull_more: Optional ``callable() -> bool`` polled while holding; each
            True pulls ``extra_deg`` further (emits ``pulled`` with the total).
        progress_callback: Optional ``callable(dict)`` invoked with
            structured progress events (``phase`` with
            winding/ramp/holding/released, ``winding_progress``, ``pulled``
            with ``total_deg``, ``cleanup_failed``). Must be fast and non-blocking; exceptions
            it raises are swallowed.
        should_stop: Optional ``callable() -> bool`` polled between motor
            commands; return ``True`` to leave the hold cooperatively.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    control_mode = hand.config.control_mode

    def _restore_hand() -> None:
        hand.set_max_current(hand.config.max_current)
        hand.set_control_mode(control_mode)
        hand.disable_torque()

    hand.set_control_mode(CURRENT_BASED_POSITION)
    try:
        if move_motors:
            _emit(progress_callback, "phase", phase="winding")
            motors_to_move = finger_motor_ids(hand.config, hand.absent_motor_ids)
            hand.set_max_current(hand.config.calibration_current)

            increment_per_step = 0.1
            motor_increments_right = {
                motor_id: increment_per_step for motor_id in motors_to_move
            }
            motor_increments_left = {
                motor_id: -increment_per_step for motor_id in motors_to_move
            }

            # Drive each direction until the commanded motors stall (tendons
            # taut) for stall_hold seconds, bounded by max_wind_s per direction.
            stall_threshold = 0.01
            stall_hold = 1.0
            max_wind_s = 20.0
            moved_idx = np.array(
                [hand.config.motor_ids.index(mid) for mid in motors_to_move]
            )

            for wind_pass, increments in enumerate(
                (motor_increments_left, motor_increments_right)
            ):
                _emit(progress_callback, "winding_progress", stage=wind_pass + 1, stages=2)
                stall_start = None
                phase_start = time.time()
                # Checked reads: a stale sample repeats the previous position,
                # which reads as zero motion and ends the winding with slack.
                # A read that stays stale skips the tick instead of aborting.
                prev_pos = _read_motor_pos_or_none(hand)
                while not should_stop() and time.time() - phase_start < max_wind_s:
                    hand._set_motor_pos(increments, rel_to_current=True)
                    time.sleep(0.1)
                    cur_pos = _read_motor_pos_or_none(hand)
                    if cur_pos is None:
                        continue
                    if prev_pos is None:
                        prev_pos = cur_pos
                        continue
                    delta = np.max(np.abs(cur_pos[moved_idx] - prev_pos[moved_idx]))
                    if delta < stall_threshold:
                        if stall_start is None:
                            stall_start = time.time()
                        elif time.time() - stall_start >= stall_hold:
                            break
                    else:
                        stall_start = None
                    prev_pos = cur_pos

        max_cur = hand.config.max_current
        if move_motors:
            # Gradually release torque so tendons don't snap back, then
            # re-engage at the relaxed position for a stable hold.
            _emit(progress_callback, "phase", phase="ramp")
            hand.set_max_current(max_cur)
            hand.enable_torque()
            steps = 20
            for i in range(steps):
                if should_stop():
                    break
                hand.set_max_current(max_cur * (1 - (i + 1) / steps))
                time.sleep(1.0 / steps)
            hand.disable_torque()
            time.sleep(0.05)
        fingers = finger_motor_ids(hand.config, hand.absent_motor_ids)

        def pull_then_hold(offset: float, start: Optional[np.ndarray]) -> np.ndarray:
            """Pull ``offset`` at the capped current, let it settle, then hold there.

            The pull runs capped so a tendon that stalls short limits what the
            motor can draw while it pushes. Only once the goal has been
            re-baselined onto the position actually reached, leaving no standing
            error, is the ceiling raised for the ratcheting that follows.
            """
            hand.set_max_current(max_cur)
            if offset:
                pull_tendons(hand, fingers, offset, start=start, should_stop=should_stop)
            reached = settle_at_rest(hand, fingers, should_stop=should_stop)
            if uncapped:
                hand.set_max_current(uncapped_currents(hand))
            return reached

        hand.set_max_current(max_cur)
        hold_here(hand, fingers)
        hand.enable_torque()
        pulled = 0.0
        goals = pull_then_hold(offset_deg, None)
        if offset_deg:
            pulled = offset_deg
            _emit(progress_callback, "pulled", total_deg=pulled)
        logger.info("holding motors for manual tendon tensioning")
        _emit(progress_callback, "phase", phase="holding")
        while not should_stop():
            if pull_more is not None and pull_more():
                # From where the motors actually are, not from an accumulated
                # command: a stalled motor gains nothing more, which is the
                # point, because that is the error an uncapped hold would cook on.
                goals = pull_then_hold(extra_deg, goals)
                pulled += extra_deg
                _emit(progress_callback, "pulled", total_deg=pulled)
            time.sleep(0.1)
    except BaseException:
        # Cleanup failures are reported, not raised, so the original error
        # propagates instead of being masked.
        _emit(progress_callback, "phase", phase="released")
        try:
            _restore_hand()
        except Exception as e:
            _emit(progress_callback, "cleanup_failed", error=str(e))
            logger.warning("cleanup after aborted tension routine failed: %s", e)
        raise
    _emit(progress_callback, "phase", phase="released")
    _restore_hand()


def run_jitter(
    hand: "OrcaHand",
    motor_ids: Optional[List[int]] = None,
    amplitude: float = 5.0,
    frequency: float = 10.0,
    duration: float = 3.0,
    include_wrist: bool = False,
    *,
    should_stop: Optional[ShouldStop] = None,
) -> None:
    """Oscillate motors around their current position to seat the tendons.

    Args:
        hand: A connected :class:`~orca_core.OrcaHand`.
        motor_ids: Motors to jitter. Defaults to all non-wrist motors (or
            all motors when ``include_wrist`` is ``True``).
        amplitude: Peak amplitude in degrees; motors swing ±amplitude
            around their start position (max ``10.0``).
        frequency: Oscillation frequency in Hz.
        duration: Total jitter duration in seconds.
        include_wrist: Include the wrist motor when ``motor_ids`` is ``None``.
        should_stop: Optional ``callable() -> bool`` polled every cycle;
            return ``True`` to stop early.

    Raises:
        ValueError: If ``amplitude`` exceeds 10°.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    max_amplitude_deg = 10.0
    if amplitude > max_amplitude_deg:
        raise ValueError(
            f"Amplitude must be <= {max_amplitude_deg} degrees for safety. Got {amplitude}."
        )

    amplitude_rad = np.deg2rad(amplitude)

    if motor_ids is None:
        wrist_motor_id = hand.config.joint_to_motor_map.get("wrist")
        motor_ids = [
            mid
            for mid in hand.config.motor_ids
            if include_wrist or mid != wrist_motor_id
        ]

    start_positions = hand.get_motor_pos(as_dict=True)
    start_pos_array = np.array([start_positions[mid] for mid in motor_ids])

    # Motor families with one bus transaction per motor per update flood the
    # serial link without a throttle, causing "no status packet" failures.
    jitter_period_s = 0.01

    start_time = time.time()
    while time.time() - start_time < duration and not should_stop():
        t = time.time() - start_time
        offset = amplitude_rad * math.sin(2 * math.pi * frequency * t)
        hand.write_motor_pos(motor_ids, start_pos_array + offset)
        time.sleep(jitter_period_s)

    hand.write_motor_pos(motor_ids, start_pos_array)
