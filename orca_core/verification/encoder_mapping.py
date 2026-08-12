# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Verify each encoder slot against the joint it is assigned to.

Drives one joint at a time by a known amount and watches every slot, so a
single sweep separates four faults: a slot that does not respond, one that
responds by the wrong amount, one that responds with the wrong sign, and one
that responds to a *different* joint than the map says it belongs to.

The measurement is differential, so it needs no encoder anchors: count
deltas cancel the anchor. A joint whose anchor pass failed can still be
checked here. It does need a calibrated hand — commanding a known joint
angle needs the joint-to-motor map.

The polarity table is a claim about this hand's side, not a measurement, so
verifying it per unit is the point of the sign check.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, TYPE_CHECKING

import numpy as np

from ..constants import NUM_STEPS, STEP_SIZE
from ..hardware.joint_encoder_client import (
    JointEncoderCalibrationError,
    average_anchor_count,
    signed_count_delta,
)
from ..hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    ENCODER_LSB_DEG,
    JOINT_TO_ENCODER_SLOT,
    joint_encoder_polarity_for_side,
)

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

MOVE_DEG = 15.0
"""Joint travel per sweep, shrunk to fit joints whose ROM is smaller."""

MAX_SPAN_FRACTION = 0.6
"""Largest share of a joint's ROM one sweep may use, so it stays clear of
both hardstops and measures free travel rather than a stall."""

SETTLE_S = 0.5
NUM_SAMPLES = 100

RESPONSE_LIMIT_DEG = 2.0
"""Below this the driven slot counts as not having moved at all."""

MAGNITUDE_TOLERANCE = 0.5
"""Allowed fractional deviation from the commanded travel. Wide because
tendon compliance makes the joint undershoot the motor; set from fleet data."""

CROSSTALK_LIMIT_DEG = 3.0
"""Motion on a slot other than the driven joint's. Antagonistic routing
through each joint's centre of rotation keeps real coupling small, but not
zero; set from fleet data."""


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the routine."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("encoder mapping progress callback failed")


@dataclass(frozen=True)
class SlotCrosstalk:
    """A slot that moved while a different joint was being driven.

    ``joint`` is the joint that slot is assigned to, or ``None`` for a slot
    this hand does not configure.
    """

    slot: int
    joint: str | None
    delta_deg: float


@dataclass(frozen=True)
class JointSweep:
    """One joint's sweep: what was commanded, what every slot did.

    ``measured_deg`` is the driven slot's travel in joint degrees, signed so
    a correctly mapped slot matches the sign of ``commanded_deg``.
    ``slot_deltas`` records every slot regardless of verdict, so a limit that
    moves later can be re-applied to stored results. ``angle_error_samples``
    counts the chip's own error flag per slot; it is reported, not acted on.
    """

    joint: str
    slot: int
    commanded_deg: float
    measured_deg: float | None
    polarity: int
    slot_deltas: Dict[int, float]
    angle_error_samples: Dict[int, int]
    slot_responded: bool
    sign_ok: bool
    magnitude_ok: bool
    crosstalk: List[SlotCrosstalk] = field(default_factory=list)
    messages: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return (
            self.slot_responded
            and self.sign_ok
            and self.magnitude_ok
            and not self.crosstalk
        )


@dataclass(frozen=True)
class EncoderMappingResult:
    """Every joint's sweep plus the thresholds that judged them."""

    side: str
    sweeps: List[JointSweep]
    thresholds: Dict[str, float]

    @property
    def failed(self) -> List[JointSweep]:
        return [sweep for sweep in self.sweeps if not sweep.ok]


def _sample_all_slots(
    joint_encoder_client,
    *,
    num_samples: int,
    sample_period_s: float = 0.002,
    timeout_s: float = 5.0,
) -> tuple[Dict[int, int | None], Dict[int, int]]:
    """Cosine-average every slot over the same ``num_samples`` distinct frames.

    Returns ``({slot: mean count or None}, {slot: angle-error sample count})``.
    Only the parity check gates a sample: a parity failure means the word
    itself is corrupt, whereas the chip's angle-error flag leaves the count
    usable and is reported rather than acted on — discarding those samples
    would starve the sweep and turn a flag into a wiring conclusion this
    measurement cannot support.
    """
    counts: Dict[int, List[int]] = {s: [] for s in range(AUTO_ENC_NUM_JOINTS)}
    angle_errors: Dict[int, int] = {s: 0 for s in range(AUTO_ENC_NUM_JOINTS)}
    last_ts: float | None = None
    collected = 0
    deadline = time.monotonic() + timeout_s

    while collected < num_samples:
        reading = joint_encoder_client.get_latest()
        if reading is not None and reading.timestamp != last_ts:
            last_ts = reading.timestamp
            for slot in range(AUTO_ENC_NUM_JOINTS):
                if bool(reading.angle_error[slot]):
                    angle_errors[slot] += 1
                if bool(reading.parity_ok[slot]):
                    counts[slot].append(int(reading.raw_counts[slot]) & 0x3FFF)
            collected += 1
            continue
        if time.monotonic() > deadline:
            raise JointEncoderCalibrationError(
                f"timed out waiting for encoder frames "
                f"(got {collected}/{num_samples} in {timeout_s}s)"
            )
        time.sleep(sample_period_s)

    means = {
        slot: (average_anchor_count(np.array(samples, dtype=np.uint16))
               if samples else None)
        for slot, samples in counts.items()
    }
    return means, angle_errors


def _sweep_bounds(rom: tuple[float, float], move_deg: float) -> tuple[float, float]:
    """Start and end angle for a sweep centred in ``rom``."""
    lower, upper = rom
    span = upper - lower
    move = min(move_deg, span * MAX_SPAN_FRACTION)
    start = lower + (span - move) / 2.0
    return start, start + move


def run_encoder_mapping_sweep(
    hand: "OrcaHand",
    *,
    joint_encoder_client,
    joints: List[str] | None = None,
    move_deg: float = MOVE_DEG,
    settle_s: float = SETTLE_S,
    num_samples: int = NUM_SAMPLES,
    response_limit_deg: float = RESPONSE_LIMIT_DEG,
    magnitude_tolerance: float = MAGNITUDE_TOLERANCE,
    crosstalk_limit_deg: float = CROSSTALK_LIMIT_DEG,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> EncoderMappingResult | None:
    """Sweep each encoder-backed joint and check its slot against the map.

    Drives one joint at a time between two angles centred in its ROM,
    sampling every slot at both ends. Returns ``None`` on early exit
    (``should_stop`` triggered). The hand is returned to the pose it started
    in and its torque released on every exit path, including exceptions.

    Run this against a motor-only :class:`~orca_core.OrcaHand` with the joint
    control loop stopped — the loop would fight the commanded moves, and a
    sensing hand's ``connect()`` holds the serial port this routine's encoder
    client needs.

    Args:
        hand: A connected, calibrated :class:`~orca_core.OrcaHand`.
        joint_encoder_client: A streaming ``JointEncoderClient``.
        joints: Restrict to these joints. Defaults to every encoder-backed joint.
        move_deg: Joint travel per sweep, shrunk to fit a smaller ROM.
        settle_s: Seconds to wait after each move before sampling.
        num_samples: Encoder frames averaged at each end of a sweep.
        response_limit_deg: Below this the driven slot counts as unresponsive.
        magnitude_tolerance: Allowed fractional deviation from commanded travel.
        crosstalk_limit_deg: Motion on another slot that counts as cross-talk.
        progress_callback: Optional ``callable(dict)`` invoked with structured
            progress events (``sweep_started``, ``joint_started``,
            ``joint_swept``, ``sweep_aborted``, ``sweep_done``,
            ``cleanup_failed``). Must be fast and non-blocking; exceptions it
            raises are swallowed.
        should_stop: Optional ``callable() -> bool`` polled between moves.

    Raises:
        RuntimeError: If the hand is not calibrated.
        ValueError: If this hand's side has no validated encoder polarity table.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    # Motor limits and ratios only: the sweep is differential, so it works on
    # a hand whose encoder anchors are missing or were rejected.
    if not hand.is_calibrated(use_joint_feedback=False):
        raise RuntimeError(
            "encoder mapping sweep needs a calibrated hand: commanding a known "
            "joint angle requires the joint-to-motor map"
        )

    polarity = joint_encoder_polarity_for_side(hand.config.type)
    target_joints = list(joints) if joints is not None else list(hand.encoder_backed_joints)
    slot_to_joint = {
        JOINT_TO_ENCODER_SLOT[joint]: joint
        for joint in hand.encoder_backed_joints
        if joint in JOINT_TO_ENCODER_SLOT
    }
    thresholds = {
        "move_deg": float(move_deg),
        "response_limit_deg": float(response_limit_deg),
        "magnitude_tolerance": float(magnitude_tolerance),
        "crosstalk_limit_deg": float(crosstalk_limit_deg),
    }

    home = hand.get_joint_position().as_dict()
    sweeps: List[JointSweep] = []

    def _restore_hand() -> None:
        hand.set_joint_positions(home, num_steps=NUM_STEPS, step_size=STEP_SIZE)
        hand.disable_torque()

    _emit(progress_callback, "sweep_started", joints=list(target_joints))
    hand.enable_torque()
    aborted = False
    try:
        for joint in target_joints:
            if should_stop():
                aborted = True
                break

            slot = JOINT_TO_ENCODER_SLOT[joint]
            start, end = _sweep_bounds(hand.config.joint_roms_dict[joint], move_deg)
            _emit(
                progress_callback,
                "joint_started",
                joint=joint,
                slot=slot,
                start_deg=start,
                end_deg=end,
            )

            hand.set_joint_positions({joint: start}, num_steps=NUM_STEPS, step_size=STEP_SIZE)
            time.sleep(settle_s)
            before, errors_before = _sample_all_slots(
                joint_encoder_client, num_samples=num_samples
            )

            hand.set_joint_positions({joint: end}, num_steps=NUM_STEPS, step_size=STEP_SIZE)
            time.sleep(settle_s)
            after, errors_after = _sample_all_slots(
                joint_encoder_client, num_samples=num_samples
            )

            sweep = _judge_sweep(
                joint=joint,
                slot=slot,
                side=hand.config.type,
                commanded_deg=end - start,
                before=before,
                after=after,
                angle_error_samples={
                    s: errors_before[s] + errors_after[s]
                    for s in range(AUTO_ENC_NUM_JOINTS)
                },
                polarity=polarity,
                slot_to_joint=slot_to_joint,
                response_limit_deg=response_limit_deg,
                magnitude_tolerance=magnitude_tolerance,
                crosstalk_limit_deg=crosstalk_limit_deg,
            )
            sweeps.append(sweep)
            _emit(
                progress_callback,
                "joint_swept",
                joint=joint,
                slot=slot,
                commanded_deg=sweep.commanded_deg,
                measured_deg=sweep.measured_deg,
                ok=sweep.ok,
                messages=list(sweep.messages),
            )

            hand.set_joint_positions(
                {joint: home[joint]}, num_steps=NUM_STEPS, step_size=STEP_SIZE
            )
    except BaseException:
        # Cleanup failures are reported, not raised, so the original error
        # propagates instead of being masked.
        try:
            _restore_hand()
        except Exception as e:
            _emit(progress_callback, "cleanup_failed", error=str(e))
            logger.warning("cleanup after aborted encoder mapping sweep failed: %s", e)
        raise

    _restore_hand()
    if aborted:
        _emit(progress_callback, "sweep_aborted")
        return None

    _emit(progress_callback, "sweep_done", failed=[s.joint for s in sweeps if not s.ok])
    return EncoderMappingResult(
        side=hand.config.type, sweeps=sweeps, thresholds=thresholds
    )


def _judge_sweep(
    *,
    joint: str,
    slot: int,
    side: str,
    commanded_deg: float,
    before: Dict[int, int | None],
    after: Dict[int, int | None],
    angle_error_samples: Dict[int, int],
    polarity: Dict[str, int],
    slot_to_joint: Dict[int, str],
    response_limit_deg: float,
    magnitude_tolerance: float,
    crosstalk_limit_deg: float,
) -> JointSweep:
    """Reduce one joint's before/after slot counts to measurements and verdicts.

    Each slot's travel is expressed in its own joint's degrees, so a slot
    belonging to another joint reads in units that joint's limits are in. A
    slot this hand does not configure has no polarity and is reported with
    the raw count direction.
    """
    slot_deltas: Dict[int, float] = {}
    for s in range(AUTO_ENC_NUM_JOINTS):
        if before.get(s) is None or after.get(s) is None:
            continue
        counts = signed_count_delta(before[s], after[s])
        sign = polarity.get(slot_to_joint.get(s, ""), 1)
        slot_deltas[s] = counts * ENCODER_LSB_DEG * sign

    measured_deg = slot_deltas.get(slot)
    slot_responded = (
        measured_deg is not None and abs(measured_deg) >= response_limit_deg
    )
    sign_ok = slot_responded and (measured_deg > 0) == (commanded_deg > 0)
    magnitude_ok = slot_responded and (
        abs(abs(measured_deg) - abs(commanded_deg))
        <= magnitude_tolerance * abs(commanded_deg)
    )

    crosstalk = [
        SlotCrosstalk(slot=s, joint=slot_to_joint.get(s), delta_deg=delta)
        for s, delta in sorted(slot_deltas.items())
        if s != slot and abs(delta) > crosstalk_limit_deg
    ]

    messages: List[str] = []
    if measured_deg is None:
        messages.append(
            f"Encoder slot {slot} ({joint}) returned no parity-valid count."
        )
    elif not slot_responded:
        messages.append(
            f"Joint {joint} was driven {commanded_deg:+.1f}° but encoder slot "
            f"{slot} moved {measured_deg:+.2f}° (response limit "
            f"{response_limit_deg:.1f}°)."
        )
    else:
        if not sign_ok:
            messages.append(
                f"Joint {joint} moved {commanded_deg:+.1f}° at the motor but "
                f"{measured_deg:+.1f}° at encoder slot {slot}; the polarity "
                f"table for a {side} hand expects the same direction."
            )
        if not magnitude_ok:
            messages.append(
                f"Joint {joint} was driven {commanded_deg:+.1f}° but encoder "
                f"slot {slot} moved {measured_deg:+.1f}° (tolerance "
                f"{magnitude_tolerance:.0%})."
            )
    for other in crosstalk:
        assigned = other.joint or "no configured joint"
        messages.append(
            f"Driving {joint} moved slot {other.slot} ({assigned}) by "
            f"{other.delta_deg:+.1f}°. Slot {other.slot} is assigned to {assigned}."
        )

    return JointSweep(
        joint=joint,
        slot=slot,
        commanded_deg=float(commanded_deg),
        measured_deg=measured_deg,
        polarity=int(polarity.get(joint, 1)),
        slot_deltas=slot_deltas,
        angle_error_samples=angle_error_samples,
        slot_responded=slot_responded,
        sign_ok=sign_ok,
        magnitude_ok=magnitude_ok,
        crosstalk=crosstalk,
        messages=messages,
    )
