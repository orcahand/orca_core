# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Waypoint refinement of the joint→motor map, run inside the calibration routine.

After a joint's endpoint calibration completes, the hand commands settled
interior waypoints and measures them with the joint encoder. A linear fit
``measured = a·commanded + b`` is folded back into the motor limits and
joint-to-motor ratio, correcting both the offset and the scale that the
endpoint-only calibration gets wrong (the motor limits are read after a
torque release, so the endpoint map inherits the tendon settle-back bias).
The runtime map keeps its exact form — only the calibrated numbers improve.

Like the rest of :mod:`orca_core.maintenance` this is interaction-free:
cancellation comes in through ``should_stop()``.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Dict, Optional, Protocol, Sequence, Tuple, TYPE_CHECKING

import numpy as np

from ..calibration import JointEncoderCal
from ..constants import FLEX, JOINTS
from ..hardware.joint_encoder_client import JointEncoderCalibrationError
from ..hardware.sensing.encoder_protocol import encoder_to_joint_angle
from ..hardware.sensing.types import EncoderReading

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand


# Fractions are of the joint ROM; the sweep visits them moving away from the
# just-pressed hardstop, then re-samples any that lie on the way back to
# neutral, so the fit mixes both approach directions and the out-vs-back
# difference doubles as a hysteresis diagnostic. A pose counts as settled when
# the decoded angle's peak-to-peak over the trailing window is below the
# tolerance; the window mean is the measurement, so the wait is exactly as long
# as the mechanics need. The scale/offset bounds reject fits that cannot be a
# plausible map correction (wiring or polarity faults, a stalled motor); the
# endpoint calibration is kept in that case.
WAYPOINT_FIT_FRACTIONS = (0.25, 0.5, 0.75)
WAYPOINT_SETTLE_TOL_DEG = 0.15
WAYPOINT_SETTLE_WINDOW_S = 0.15
WAYPOINT_SETTLE_TIMEOUT_S = 2.5
WAYPOINT_SETTLE_POLL_S = 0.005
WAYPOINT_SETTLE_MIN_SAMPLES = 5
WAYPOINT_FIT_MIN_POINTS = 3
WAYPOINT_FIT_SCALE_BOUNDS = (0.75, 1.25)
WAYPOINT_FIT_OFFSET_MAX_DEG = 10.0
WAYPOINT_FIT_RESIDUAL_WARN_DEG = 0.75


class _ReadsLatestEncoderFrame(Protocol):
    def get_latest(self) -> EncoderReading | None: ...


def wait_for_settled_angles(
    client: _ReadsLatestEncoderFrame,
    slots: np.ndarray,
    anchors: np.ndarray,
    polarities: np.ndarray,
    anchor_angles: np.ndarray,
    tol_deg: float | None = None,
    window_s: float | None = None,
    timeout_s: float | None = None,
    poll_period_s: float | None = None,
    min_samples: int | None = None,
) -> np.ndarray:
    """Wait until every joint's decoded angle is stable, then return the
    per-joint mean over the stable window (degrees).

    A joint is settled when the peak-to-peak of its decoded angle over the
    trailing ``window_s`` is below ``tol_deg`` with at least ``min_samples``
    distinct frames. The same window doubles as the measurement, so the call
    blocks exactly as long as the mechanics need. Joints still moving at
    ``timeout_s`` return ``NaN`` so the caller can skip that sample.
    ``None`` parameters resolve to the module ``WAYPOINT_SETTLE_*`` constants
    at call time.

    Raises:
        JointEncoderCalibrationError: no encoder frames arrived at all.
    """
    tol = WAYPOINT_SETTLE_TOL_DEG if tol_deg is None else float(tol_deg)
    window = WAYPOINT_SETTLE_WINDOW_S if window_s is None else float(window_s)
    timeout = WAYPOINT_SETTLE_TIMEOUT_S if timeout_s is None else float(timeout_s)
    poll = WAYPOINT_SETTLE_POLL_S if poll_period_s is None else float(poll_period_s)
    min_n = WAYPOINT_SETTLE_MIN_SAMPLES if min_samples is None else int(min_samples)

    samples: deque[tuple[float, np.ndarray]] = deque()
    last_ts: float | None = None
    deadline = time.monotonic() + timeout

    def _window_is_full() -> bool:
        return (
            len(samples) >= min_n
            and samples[-1][0] - samples[0][0] >= window * 0.8
        )

    while True:
        reading = client.get_latest()
        now = time.monotonic()
        if reading is not None and reading.timestamp != last_ts:
            last_ts = reading.timestamp
            counts = np.asarray(reading.raw_counts)[slots]
            angles = np.asarray(
                encoder_to_joint_angle(counts, anchors, polarities, anchor_angles),
                dtype=np.float64,
            )
            samples.append((now, angles))
            while samples and samples[0][0] < now - window:
                samples.popleft()
            if _window_is_full():
                stack = np.stack([a for _, a in samples])
                if np.all(np.ptp(stack, axis=0) <= tol):
                    return stack.mean(axis=0)
        if now > deadline:
            if not samples:
                raise JointEncoderCalibrationError(
                    f"no encoder frames arrived within {timeout}s while "
                    "waiting for a settled pose"
                )
            stack = np.stack([a for _, a in samples])
            means = stack.mean(axis=0)
            if _window_is_full():
                means[np.ptp(stack, axis=0) > tol] = np.nan
            else:
                means[:] = np.nan
            return means
        time.sleep(poll)


def fit_linear_joint_map(
    points: Sequence[Tuple[float, float]],
) -> Tuple[float, float, np.ndarray]:
    """Least-squares fit of ``measured = a·commanded + b``.

    Returns ``(a, b, residuals)`` with residuals in measurement order.

    Raises:
        ValueError: fewer than two points, or the commanded angles span less
            than one degree (no slope information).
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 2 or pts.shape[0] < 2:
        raise ValueError("need at least two (commanded, measured) points")
    cmd, meas = pts[:, 0], pts[:, 1]
    if float(np.ptp(cmd)) < 1.0:
        raise ValueError("commanded angles span less than 1°; cannot fit a slope")
    design = np.vstack([cmd, np.ones_like(cmd)]).T
    (a, b), *_ = np.linalg.lstsq(design, meas, rcond=None)
    residuals = meas - (a * cmd + b)
    return float(a), float(b), residuals


def fold_linear_correction(
    a: float,
    b: float,
    lower: float,
    ratio: float,
    rom_lo: float,
    rom_up: float,
    inverted: bool,
) -> Tuple[float, float, float]:
    """Fold a measured linear response ``measured = a·commanded + b`` back
    into the joint→motor map parameters.

    The corrected map commands the motor position at which the joint
    encoder reads the requested angle: ``motor'(θ) = map((θ − b) / a)``,
    re-expressed in the map's own ``(lower, upper, ratio)`` parameters so
    the runtime mapping code is untouched. Returns
    ``(lower', upper', ratio')`` with ``upper'`` recomputed to keep the
    ``ratio = (upper − lower) / (rom_up − rom_lo)`` invariant.

    Raises:
        ValueError: non-positive scale ``a`` (a map correction cannot flip
            or collapse the direction of motion).
    """
    if a <= 0:
        raise ValueError(f"scale must be positive, got a={a}")
    new_ratio = ratio / a
    if inverted:
        new_lower = lower + ((a - 1.0) * rom_up + b) * ratio / a
    else:
        new_lower = lower + ((1.0 - a) * rom_lo - b) * ratio / a
    new_upper = new_lower + new_ratio * (rom_up - rom_lo)
    return float(new_lower), float(new_upper), float(new_ratio)


def run_waypoint_fit_for_step(
    hand: "OrcaHand",
    *,
    step,
    calibrated_joints: Dict[str, float],
    motor_limits: Dict[int, list],
    joint_to_motor_ratios: Dict[int, float],
    joint_encoder_calibration: Dict[str, JointEncoderCal],
    joint_encoder_client,
    should_stop: Optional[callable] = None,
) -> None:
    """Refine the joint→motor map of the joints completed in ``step`` from
    settled interior waypoints measured by the joint encoders.

    Commands each completed joint through interior waypoints on its way back
    to neutral, waits for the encoder to settle at each (see
    :func:`wait_for_settled_angles`), fits ``measured = a·commanded + b`` and
    folds the correction into ``motor_limits`` / ``joint_to_motor_ratios`` in
    place. Waypoints the return leg crosses again are re-sampled from the
    opposite direction, making the fit direction-balanced; the out-vs-back
    difference is printed as a per-joint hysteresis diagnostic. A fit failing
    the sanity bounds leaves the endpoint calibration untouched. No motor
    reads are issued, so the encoder stream is not contended.
    """
    from ..hardware.sensing.constants import (
        JOINT_ENCODER_POLARITY,
        JOINT_TO_ENCODER_SLOT,
    )

    encoder_backed = set(hand._encoder_backed_joints())
    joints = [
        j
        for j in step[JOINTS]
        if j in calibrated_joints
        and j in encoder_backed
        and j in joint_encoder_calibration
    ]
    if not joints:
        return

    slots = np.array([JOINT_TO_ENCODER_SLOT[j] for j in joints], dtype=np.int64)
    anchors = np.array(
        [joint_encoder_calibration[j].enc_at_anchor_count for j in joints],
        dtype=np.int64,
    )
    polarities = np.array(
        [JOINT_ENCODER_POLARITY[j] for j in joints], dtype=np.int64
    )
    anchor_angles = np.array(
        [hand.config.joint_roms_dict[j][1] for j in joints], dtype=np.float64
    )

    schedules = {j: _waypoint_schedule(hand, j, step[JOINTS][j]) for j in joints}
    num_poses = max(len(s) for s in schedules.values())
    for schedule in schedules.values():
        # Pad with the final (neutral, no-sample) pose so all joints in
        # the step move in lockstep.
        schedule.extend([schedule[-1]] * (num_poses - len(schedule)))

    samples: Dict[str, list] = {j: [] for j in joints}
    for pose_idx in range(num_poses):
        if should_stop is not None and should_stop():
            print(
                "Calibration stop requested; keeping endpoint calibration "
                "for this step (waypoint fit skipped)."
            )
            return
        command = {
            hand.config.joint_to_motor_map[j]: _waypoint_motor_pos(
                hand, j, schedules[j][pose_idx][0], motor_limits, joint_to_motor_ratios
            )
            for j in joints
        }
        hand._set_motor_pos(command)
        try:
            measured = wait_for_settled_angles(
                joint_encoder_client, slots, anchors, polarities, anchor_angles
            )
        except JointEncoderCalibrationError as e:
            print(
                f"\033[93mWARNING: waypoint fit aborted, keeping endpoint "
                f"calibration: {e}\033[0m"
            )
            return
        for i, j in enumerate(joints):
            angle, record = schedules[j][pose_idx]
            if not record:
                continue
            if np.isfinite(measured[i]):
                samples[j].append((angle, float(measured[i])))
            else:
                print(
                    f"\033[93mWARNING: joint {j} did not settle at waypoint "
                    f"{angle:.1f}°; sample skipped\033[0m"
                )

    for j in joints:
        _fold_waypoint_fit(hand, j, samples[j], motor_limits, joint_to_motor_ratios)


def _waypoint_schedule(hand: "OrcaHand", joint: str, direction: str) -> list:
    """Ordered ``(angle_deg, record_sample)`` poses for one joint: the
    interior waypoints swept away from the just-pressed hardstop, then
    any waypoint the way back to neutral re-crosses (opposite-direction
    samples), ending at neutral."""
    rom_lo, rom_up = hand.config.joint_roms_dict[joint]
    span = rom_up - rom_lo
    waypoints = [rom_lo + f * span for f in WAYPOINT_FIT_FRACTIONS]
    start = rom_up if direction == FLEX else rom_lo
    outbound = sorted(waypoints, key=lambda w: abs(w - start))
    neutral = float(np.clip(0.0, rom_lo, rom_up))
    last = outbound[-1]
    toward_neutral = np.sign(neutral - last)
    reversal = [
        w
        for w in waypoints
        if toward_neutral != 0
        and (w - last) * toward_neutral > 1e-9
        and (w - neutral) * toward_neutral <= 1e-9
    ]
    reversal.sort(key=lambda w: abs(w - last))
    poses = [(w, True) for w in outbound] + [(w, True) for w in reversal]
    if not (reversal and abs(reversal[-1] - neutral) < 1e-9):
        poses.append((neutral, False))
    return poses


def _waypoint_motor_pos(
    hand: "OrcaHand",
    joint: str,
    angle_deg: float,
    motor_limits: Dict[int, list],
    joint_to_motor_ratios: Dict[int, float],
) -> float:
    """Joint→motor mapping using the in-flight calibration values (the
    instance calibration is not updated until the step persists)."""
    motor_id = hand.config.joint_to_motor_map[joint]
    lower = motor_limits[motor_id][0]
    ratio = joint_to_motor_ratios[motor_id]
    rom_lo, rom_up = hand.config.joint_roms_dict[joint]
    if hand.config.joint_inversion_dict.get(joint, False):
        base = lower + (rom_up - angle_deg) * ratio
    else:
        base = lower + (angle_deg - rom_lo) * ratio
    return base + (hand._wrap_offsets_dict or {}).get(motor_id, 0.0)


def _fold_waypoint_fit(
    hand: "OrcaHand",
    joint: str,
    points: list,
    motor_limits: Dict[int, list],
    joint_to_motor_ratios: Dict[int, float],
) -> None:
    """Fit the collected waypoint samples for one joint and, when the fit
    passes the sanity bounds, fold it into the in-flight calibration
    dicts. Any failure keeps the endpoint calibration and warns."""
    motor_id = hand.config.joint_to_motor_map[joint]
    if len(points) < WAYPOINT_FIT_MIN_POINTS:
        print(
            f"\033[93mWARNING: joint {joint} has only {len(points)} settled "
            f"waypoint(s); keeping endpoint calibration\033[0m"
        )
        return

    by_angle: Dict[float, list] = {}
    for cmd, meas in points:
        by_angle.setdefault(round(cmd, 6), []).append(meas)
    for cmd, values in by_angle.items():
        if len(values) == 2:
            print(
                f"Joint {joint} hysteresis at {cmd:.1f}°: "
                f"{values[0] - values[1]:+.2f}° (outbound − return)"
            )

    try:
        a, b, residuals = fit_linear_joint_map(points)
    except ValueError as e:
        print(
            f"\033[93mWARNING: joint {joint} waypoint fit failed ({e}); "
            f"keeping endpoint calibration\033[0m"
        )
        return

    scale_lo, scale_hi = WAYPOINT_FIT_SCALE_BOUNDS
    if not (scale_lo <= a <= scale_hi) or abs(b) > WAYPOINT_FIT_OFFSET_MAX_DEG:
        print(
            f"\033[93mWARNING: joint {joint} waypoint fit out of bounds "
            f"(a={a:.3f}, b={b:+.2f}°); keeping endpoint calibration\033[0m"
        )
        return

    max_residual = float(np.max(np.abs(residuals)))
    if max_residual > WAYPOINT_FIT_RESIDUAL_WARN_DEG:
        print(
            f"\033[93mWARNING: joint {joint} waypoint fit residual "
            f"{max_residual:.2f}° suggests nonlinearity beyond the linear "
            f"map; folding the linear part anyway\033[0m"
        )

    rom_lo, rom_up = hand.config.joint_roms_dict[joint]
    inverted = hand.config.joint_inversion_dict.get(joint, False)
    new_lower, new_upper, new_ratio = fold_linear_correction(
        a,
        b,
        motor_limits[motor_id][0],
        joint_to_motor_ratios[motor_id],
        rom_lo,
        rom_up,
        inverted,
    )
    motor_limits[motor_id] = [new_lower, new_upper]
    joint_to_motor_ratios[motor_id] = new_ratio
    print(
        f"Joint {joint} waypoint fit applied: a={a:.3f}, b={b:+.2f}°, "
        f"max residual {max_residual:.2f}°"
    )
