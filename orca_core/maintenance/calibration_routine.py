# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Joint calibration routine for an ORCA hand.

Drives each joint to its mechanical limits following the model's
``calibration_sequence``, records motor limits and joint-to-motor ratios,
and persists partial progress to ``calibration.yaml`` after every step.

Once a joint has both of its limits, the travel between them is checked
against that joint's ``joint_motor_travel`` baseline in ``config.yaml``. A
joint that falls short of it stalled before its hardstop - an over-tensioned
tendon does this - and is re-driven at a higher current before its limits are
committed. See :func:`_resolve_short_travel`.

Like the other maintenance routines this is interaction-free: progress is
reported through ``progress_callback({"event": ..., ...})`` and cooperative
cancellation through ``should_stop()``, so a terminal, a GUI, and a web
front-end all drive the same procedure. The usual entry point is
:meth:`OrcaHand.calibrate`, which wires both to its background-task plumbing;
call :func:`run_calibration` directly to supply your own.

The routine operates on the hand through its public API plus a few protected
seams (``_set_motor_pos``, ``_compute_wrap_offsets_dict``,
``_clear_wrap_offset``, ``_encoder_backed_joints``) and commits partial
results to ``hand.calibration`` per step, so an interrupted run never loses
the work already done.
"""

from __future__ import annotations

import logging
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional, TYPE_CHECKING

import numpy as np

from ..calibration import (
    CalibrationResult,
    JointEncoderCal,
    joint_encoder_calibration_to_yaml,
)
from ..constants import (
    CALIBRATED,
    CURRENT_BASED_POSITION,
    DRIVE_STEP_TIMEOUT_S,
    EXTEND,
    FLEX,
    JOINT_ENCODER_CALIBRATION,
    JOINT_ROMS_MEASURED,
    JOINT_TO_MOTOR_RATIOS,
    JOINTS,
    MIN_MOTOR_TRAVEL_RAD,
    MIN_TRAVEL_FRACTION,
    MOTOR_LIMITS_DICT,
    MOTOR_TRAVEL_MEASURED,
    NUM_STEPS,
    STEP,
    STEP_SIZE,
    TINY_SLEEP,
    WRIST,
    WRIST_CALIBRATED,
)
from ..hardware.joint_encoder_client import (
    JointEncoderCalibrationError,
    sample_anchor_count_from_client,
)
from ..hardware.sensing.constants import ENCODER_COUNTS_PER_REV, ENCODER_LSB_DEG
from ..utils.utils import read_yaml, write_yaml_atomic
from .motor_reads import read_motor_pos_checked
from .motor_travel import motor_travel_deg, travel_deviation

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]
# Blocking: returns one of MANUAL_ACTIONS once the operator has answered.
ManualPromptCallback = Callable[[dict], str]

MANUAL_RECORD = "record"
MANUAL_SKIP = "skip"
MANUAL_ABORT = "abort"
MANUAL_ACTIONS = (MANUAL_RECORD, MANUAL_SKIP, MANUAL_ABORT)

# An anchor commits only when the flex- and extend-hardstop samples differ by
# a meaningful fraction of the ROM span: a dead or unwired slot reads a
# constant (which passes the chip parity check) and must not be anchored.
MIN_ANCHOR_SWEEP_FRACTION = 0.25
MIN_ANCHOR_SWEEP_COUNTS = 300

# The encoder-measured hardstop span replaces the config nominal in the
# joint↔motor map, so the config ROM stays its sanity envelope: a measured
# endpoint this far off nominal is a build, hardstop, or sensor fault rather
# than unit-to-unit tolerance, and the nominal is kept instead. A clamped
# value would be neither measured nor nominal, and would enter the map
# silently.
MEASURED_ROM_WARN_TOL_DEG = 4.0
MEASURED_ROM_REJECT_TOL_DEG = 8.0


def _count_delta(a: int, b: int) -> int:
    """Wrap-aware distance between two 14-bit encoder counts."""
    d = abs(a - b) % ENCODER_COUNTS_PER_REV
    return min(d, ENCODER_COUNTS_PER_REV - d)


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the routine."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("calibration progress callback failed")


def run_calibration(
    hand: "OrcaHand",
    *,
    force_wrist: bool = False,
    joints: list[str] | None = None,
    joint_encoder_client=None,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
    persist: bool = True,
    manual: bool = False,
    prompt_callback: Optional[ManualPromptCallback] = None,
) -> CalibrationResult | None:
    """Run the calibration routine on ``hand`` and return the result.

    Returns ``None`` on early exit (``should_stop`` triggered). On any exit
    without a result — abort or exception — torque is released and the
    configured current limit restored so the hand doesn't strain against a
    hardstop. Partial per-step progress is committed to ``hand.calibration``
    (and, when ``persist`` is true, ``calibration.yaml``) either way; applying
    the *final* result to the hand is the caller's choice.

    Args:
        hand: A connected :class:`~orca_core.OrcaHand`.
        force_wrist: Recalibrate the wrist even if already calibrated.
        joints: Restrict to calibration steps touching these joint names.
            Joints not visited keep their previously-persisted values.
        joint_encoder_client: With ``hand.config.joint_feedback_enabled``
            and an encoder client, the encoder pass also runs and writes a
            ``joint_encoder_calibration:`` block.
        progress_callback: Optional ``callable(dict)`` invoked with
            structured progress events (``calibration_started``,
            ``step_started``, ``limit_recorded``, ``joint_calibrated``,
            ``encoder_anchor_recorded``, ``encoder_anchor_failed``,
            ``offset_calibration_failed``, ``torque_release_failed``,
            ``wrist_skipped``, ``motor_faulted``, ``sweep_no_motion``,
            ``travel_checked``,
            ``travel_baseline_missing``, ``travel_excess``,
            ``travel_retry_started``, ``travel_retry_succeeded``,
            ``travel_retry_exhausted``, ``travel_retry_disabled``,
            ``travel_retry_unavailable``,
            ``step_done``, ``calibration_done``, ``calibration_aborted``,
            ``cleanup_failed``). Called from the calibrating thread; must be
            fast and non-blocking. Exceptions raised by the callback are
            swallowed.
        should_stop: Optional ``callable() -> bool`` polled between motor
            commands; return ``True`` to abort cooperatively.
        persist: When ``False``, nothing is written to ``calibration.yaml``;
            results are still committed to ``hand.calibration`` in memory.
        manual: Capture each hardstop from a pose the operator sets by hand
            instead of driving the motors onto it. Torque stays off for the
            whole run and ``prompt_callback`` is required.
        prompt_callback: Blocking ``callable(dict) -> str`` used only in
            manual mode. Called with ``{"action": "capture_limit", "joint":
            ..., "motor": ..., "direction": "flex"|"extend"}`` once the
            operator should be holding that joint on that hardstop; return
            ``"record"``, ``"skip"`` or ``"abort"``.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731
    if manual and prompt_callback is None:
        raise ValueError(
            "manual calibration needs a prompt_callback to ask the operator "
            "when each joint is at its hardstop"
        )

    result = None
    try:
        result = _drive_calibration(
            hand,
            force_wrist=force_wrist,
            joints=joints,
            joint_encoder_client=joint_encoder_client,
            progress_callback=progress_callback,
            should_stop=should_stop,
            persist=persist,
            manual=manual,
            prompt_callback=prompt_callback,
        )
    finally:
        if result is None:
            _release_after_abort(hand, progress_callback)
    return result


def _release_after_abort(
    hand: "OrcaHand", progress_callback: Optional[ProgressCallback] = None
) -> None:
    """Release torque and restore the configured current limit so the hand
    doesn't strain against a hardstop after an abnormal exit."""
    try:
        hand.set_max_current(hand.config.max_current)
        hand.disable_torque()
    except Exception as e:
        _emit(progress_callback, "cleanup_failed", error=str(e))
        logger.warning("cleanup after aborted calibration failed: %s", e)


def _build_calibration_result(
    motor_limits: Dict[int, list],
    joint_to_motor_ratios: Dict[int, float],
    wrist_calibrated: bool,
    joint_encoder_calibration_dict: Dict[str, JointEncoderCal] | None = None,
    joint_roms_measured_dict: Dict[str, list] | None = None,
    motor_travel_measured_dict: Dict[str, float] | None = None,
) -> CalibrationResult:
    calibrated = all(
        limits[0] is not None and limits[1] is not None
        for limits in motor_limits.values()
    ) and all(
        ratio is not None and ratio != 0.0
        for ratio in joint_to_motor_ratios.values()
    )

    return CalibrationResult(
        motor_limits_dict={
            motor_id: list(limits) for motor_id, limits in motor_limits.items()
        },
        joint_to_motor_ratios_dict=dict(joint_to_motor_ratios),
        calibrated=calibrated,
        wrist_calibrated=wrist_calibrated,
        joint_encoder_calibration_dict=dict(joint_encoder_calibration_dict or {}),
        joint_roms_measured_dict={
            joint: list(rom)
            for joint, rom in (joint_roms_measured_dict or {}).items()
        },
        motor_travel_measured_dict=dict(motor_travel_measured_dict or {}),
    )


def persist_calibration(
    calibration_path: str, *, result: CalibrationResult, include_encoder: bool
) -> None:
    """Persist ``result`` to ``calibration.yaml`` in one atomic replace.

    The whole document is rewritten at once so an interrupted write can never
    leave the file truncated or with mismatched keys (e.g. new ratios paired
    with old limits). Keys not owned by the calibration routine are preserved.
    """
    doc = read_yaml(calibration_path) or {}
    doc[JOINT_TO_MOTOR_RATIOS] = dict(result.joint_to_motor_ratios_dict)
    doc[MOTOR_LIMITS_DICT] = {
        motor_id: list(limits)
        for motor_id, limits in result.motor_limits_dict.items()
    }
    doc[WRIST_CALIBRATED] = result.wrist_calibrated
    doc[CALIBRATED] = result.calibrated
    doc[MOTOR_TRAVEL_MEASURED] = {
        joint: round(float(travel), 2)
        for joint, travel in result.motor_travel_measured_dict.items()
        if travel is not None
    }
    if include_encoder:
        doc[JOINT_ENCODER_CALIBRATION] = joint_encoder_calibration_to_yaml(
            result.joint_encoder_calibration_dict
        )
        doc[JOINT_ROMS_MEASURED] = {
            joint: list(rom)
            for joint, rom in result.joint_roms_measured_dict.items()
        }
    write_yaml_atomic(calibration_path, doc)


def _skips_torque_release(hand: "OrcaHand", motor_id: int) -> bool:
    """Whether this motor's limit is taken while it presses the hardstop.

    Only a multi-turn wrist qualifies: it travels far beyond a turn, so a
    released re-read means nothing. On a single-turn family the wrist is an
    ordinary servo, and skipping the release (and the offset calibration that
    follows it) would leave its two limits in a different coordinate frame
    from every finger motor's.
    """
    return (
        hand.config.motor_to_joint_dict[motor_id] == WRIST
        and hand.motor_client.supports_multi_turn
    )


@dataclass
class _DriveState:
    """Motor-side state carried across every drive within one calibration run.

    ``pending_limits`` holds each motor's in-flight ``[lower, upper]`` pair;
    a bound commits to the run's results only once both are captured, so a
    partial or re-driven step never leaves half-updated limits behind. The
    offset sets track the one-time frame shifts families that need them
    ("requires_offset_calibration") have already been given, so a re-drive
    does not shift a motor's frame a second time.
    """

    pending_limits: Dict[int, list]
    motors_with_initial_offset: set = field(default_factory=set)
    motors_with_final_offset: set = field(default_factory=set)


@dataclass
class _EncoderPass:
    """Live state of the joint-encoder anchor pass, when one is running."""

    client: object
    joints_to_anchor: set
    pending_anchors: Dict[str, int]
    joint_encoder_calibration: Dict[str, JointEncoderCal]
    joint_roms_measured: Dict[str, list]


def _nominal_step_current(hand: "OrcaHand") -> Callable[[str], float]:
    """The current each joint is normally driven onto its hardstop with."""

    def current_for(joint: str) -> float:
        return (
            hand.config.calibration_current
            if joint != WRIST
            else hand.config.wrist_calibration_current
        )

    return current_for


def _motor_temperature(hand: "OrcaHand", motor_id: int) -> float | None:
    """This motor's present temperature in °C, or ``None`` if unreadable."""
    try:
        temps = hand.get_motor_temp(as_dict=True)
    except Exception:
        return None
    value = temps.get(motor_id)
    return None if value is None else float(value)


def _latched_fault(hand: "OrcaHand", motor_id: int) -> list[str] | None:
    """Names of the motor's latched Hardware Error Status bits, if any.

    A latched motor still answers the bus and still acknowledges torque-enable
    writes — it simply never energizes. Driving it therefore looks exactly
    like a joint that is already on its hardstop, which is how a dead motor
    ends up recorded as a zero-travel limit pair. Returns ``None`` when the
    register cannot be read (not every family exposes one), so an unreadable
    register never blocks a calibration.
    """
    client = hand.motor_client
    read = getattr(client, "read_hardware_error", None)
    if read is None:
        return None
    try:
        value = read(motor_id)
        if value is None:  # one retry: a garbled reply is not a fault
            value = read(motor_id)
    except Exception:
        return None
    return client.decode_hardware_error(value) or None


def _report_motionless_sweeps(
    hand: "OrcaHand",
    *,
    step_joints: Dict[str, str],
    directions: Dict[int, int],
    start_positions,
    progress_callback: Optional[ProgressCallback],
) -> None:
    """Flag any motor that finished this sweep where it started it.

    The sweep calls a hardstop when the position stops changing, which a motor
    that never moved satisfies on its first few reads — the step then completes
    in milliseconds and its "limit" is wherever the joint happened to be
    resting. Saying so per direction is the point: waiting for both bounds
    hides the first half of the problem behind a step that looked fine.
    """
    if start_positions is None or not directions:
        return
    try:
        end_positions = read_motor_pos_checked(hand)
    except Exception:
        return

    motor_to_joint = hand.config.motor_to_joint_dict
    for motor_id in directions:
        idx = hand.config.motor_id_to_idx_dict[motor_id]
        moved = abs(float(end_positions[idx]) - float(start_positions[idx]))
        if moved >= MIN_MOTOR_TRAVEL_RAD:
            continue
        joint = motor_to_joint.get(motor_id, "?")
        _emit(
            progress_callback,
            "sweep_no_motion",
            joint=joint,
            motor=motor_id,
            direction=step_joints.get(joint),
            moved_deg=math.degrees(moved),
        )
        logger.error(
            "motor %d (%s) finished its %s sweep %.2f deg from where it "
            "started: it never moved, so the position recorded as its "
            "hardstop is just its resting pose. Check that the tendon is "
            "connected and that nothing is blocking the joint.",
            motor_id, joint, step_joints.get(joint, "?"), math.degrees(moved),
        )


def _drive_step(
    hand: "OrcaHand",
    *,
    step_joints: Dict[str, str],
    current_for: Callable[[str], float],
    state: _DriveState,
    encoder_pass: Optional[_EncoderPass],
    progress_callback: Optional[ProgressCallback],
    should_stop: ShouldStop,
) -> Dict[int, int] | None:
    """Drive one step's joints onto their hardstops and record their limits.

    Torques each joint's motor, walks it in the step's direction until its
    position stops changing, samples encoder anchors while the motors still
    press their hardstops, then releases torque per motor to read the
    tension-free limit into ``state.pending_limits``.

    Returns the per-motor direction signs, or ``None`` if ``should_stop`` fired
    mid-drive — the joints are then at an arbitrary pose and nothing captured
    this step may be committed.
    """
    hand.disable_torque()

    if should_stop():
        return None

    pending_limits = state.pending_limits
    directions: Dict[int, int] = {}
    motor_reached_limit: Dict[int, bool] = {}
    position_buffers: Dict[int, deque] = {}

    for joint, direction in step_joints.items():
        motor_id = hand.config.joint_to_motor_map[joint]
        # A latched motor still acknowledges the torque-enable write below but
        # never energizes, so its resting pose would be recorded as both
        # hardstops — a dead motor read as a mechanical fact about the joint.
        faults = _latched_fault(hand, motor_id)
        if faults:
            temperature = _motor_temperature(hand, motor_id)
            _emit(
                progress_callback,
                "motor_faulted",
                joint=joint,
                motor=motor_id,
                flags=faults,
                temperature_c=temperature,
            )
            logger.error(
                "motor %d (joint %s) has latched %s and will not energize "
                "until it is rebooted or power-cycled%s. Skipping the joint: "
                "driving it would record its resting pose as both hardstops. "
                "Let it cool first — a motor that is still hot re-latches "
                "immediately.",
                motor_id, joint, " + ".join(faults),
                "" if temperature is None else f" (currently {temperature:.0f} °C)",
            )
            continue

        if motor_id in hand.enable_torque(motor_ids=[motor_id]):
            _emit(
                progress_callback,
                "torque_enable_failed",
                joint=joint,
                motor=motor_id,
            )
            logger.error(
                "motor %d (joint %s) did not acknowledge torque enable; "
                "skipping it this step rather than recording limits from a "
                "motor that cannot move. A latched hardware error needs a "
                "power cycle to clear.",
                motor_id, joint,
            )
            continue
        logger.debug("torque enabled for motor %d (joint %s)", motor_id, joint)

        if should_stop():
            return None

        hand.set_max_current(current_for(joint))

        sign = 1 if direction == FLEX else -1
        if hand.config.joint_inversion_dict.get(joint, False):
            sign = -sign

        if (
            hand.motor_client.requires_offset_calibration
            and motor_id not in state.motors_with_initial_offset
        ):
            # A failed offset command leaves the motor frame un-shifted;
            # driving on would record limits in the wrong frame.
            if not hand.motor_client.calibrate_offset(motor_id, upper=(sign < 0)):
                _emit(
                    progress_callback,
                    "offset_calibration_failed",
                    motor=motor_id,
                    joint=joint,
                )
                logger.warning(
                    "offset calibration failed for motor %d; joint %s "
                    "skipped this step",
                    motor_id,
                    joint,
                )
                hand.disable_torque([motor_id])
                continue
            state.motors_with_initial_offset.add(motor_id)

        directions[motor_id] = sign
        position_buffers[motor_id] = deque(maxlen=hand.config.calibration_num_stable)
        motor_reached_limit[motor_id] = False

    try:
        start_positions = read_motor_pos_checked(hand)
    except Exception:
        start_positions = None

    deadline = time.monotonic() + DRIVE_STEP_TIMEOUT_S
    while not all(motor_reached_limit.values()) and not should_stop():
        if time.monotonic() > deadline:
            # A motor that never stabilises (a disconnected tendon spins
            # freely) would otherwise drive until someone stops it.
            for motor_id, reached in motor_reached_limit.items():
                if reached:
                    continue
                _emit(
                    progress_callback,
                    "drive_step_timeout",
                    motor=motor_id,
                    joint=hand.config.motor_to_joint_dict[motor_id],
                )
                logger.error(
                    "motor %d (%s) never settled onto a hardstop within %.0fs; "
                    "giving up on this direction. Its limit is not recorded.",
                    motor_id, hand.config.motor_to_joint_dict[motor_id],
                    DRIVE_STEP_TIMEOUT_S,
                )
            break

        desired_increment = {
            motor_id: directions[motor_id] * hand.config.calibration_step_size
            for motor_id, reached_limit in motor_reached_limit.items()
            if not reached_limit
        }

        hand._set_motor_pos(desired_increment, rel_to_current=True)
        time.sleep(hand.config.calibration_step_period)
        with hand._motor_lock:
            curr_pos = hand.motor_client.read_position_velocity_current().position
            read_ok = hand.motor_client.last_read_ok

        # A failed bulk read returns the stale cache; feeding it into the
        # stability buffers would fake a hardstop detection. Skip and retry.
        if not read_ok:
            continue

        for motor_id in desired_increment.keys():
            idx = hand.config.motor_id_to_idx_dict[motor_id]
            position_buffers[motor_id].append(curr_pos[idx])

            if len(
                position_buffers[motor_id]
            ) == hand.config.calibration_num_stable and np.allclose(
                position_buffers[motor_id],
                position_buffers[motor_id][0],
                atol=hand.config.calibration_threshold,
            ):
                motor_reached_limit[motor_id] = True
                # A multi-turn wrist's limit comes from the stable-position
                # buffer (no torque release); every other motor is re-read
                # after the release.
                if _skips_torque_release(hand, motor_id):
                    avg_limit = float(np.mean(position_buffers[motor_id]))
                    bound = 1 if directions[motor_id] == 1 else 0
                    pending_limits[motor_id][bound] = avg_limit
                    _emit(
                        progress_callback,
                        "limit_recorded",
                        motor=motor_id,
                        joint=hand.config.motor_to_joint_dict[motor_id],
                        limit=avg_limit,
                        bound="upper" if bound else "lower",
                    )
                    logger.info(
                        "motor %d (%s) reached the limit at %.4f rad",
                        motor_id,
                        hand.config.motor_to_joint_dict[motor_id],
                        avg_limit,
                    )

    if should_stop():
        return None

    _report_motionless_sweeps(
        hand,
        step_joints=step_joints,
        directions=directions,
        start_positions=start_positions,
        progress_callback=progress_callback,
    )

    # Motors are still pressing their hardstops with calibration current:
    # sample encoder anchors before the release so counts match that pose.
    if encoder_pass is not None:
        _run_joint_encoder_pass_for_step(
            hand,
            step_joints=step_joints,
            directions=directions,
            encoder_pass=encoder_pass,
            progress_callback=progress_callback,
        )

    # Release torque so tendon tension doesn't bias the reading, then
    # capture the relaxed motor position as the limit and re-engage.
    for motor_id in directions.keys():
        if not motor_reached_limit.get(motor_id):
            continue
        if _skips_torque_release(hand, motor_id):
            continue
        idx = hand.config.motor_id_to_idx_dict[motor_id]

        # A failed torque release leaves the tendon tensioned, so a limit
        # read now would be biased. A None return reports no failed motors.
        failed_release = hand.disable_torque([motor_id])
        if failed_release:
            _emit(
                progress_callback,
                "torque_release_failed",
                motor=motor_id,
                joint=hand.config.motor_to_joint_dict[motor_id],
            )
            logger.warning(
                "torque release failed for motor %d; limit not recorded",
                motor_id,
            )
            continue
        time.sleep(TINY_SLEEP)
        avg_limit = float(read_motor_pos_checked(hand)[idx])

        if (
            hand.motor_client.requires_offset_calibration
            and motor_id not in state.motors_with_final_offset
        ):
            is_positive = directions[motor_id] > 0
            # A failed offset command leaves the motor frame un-shifted;
            # recording the limit would persist a wrong-frame value.
            if not hand.motor_client.calibrate_offset(motor_id, upper=is_positive):
                _emit(
                    progress_callback,
                    "offset_calibration_failed",
                    motor=motor_id,
                    joint=hand.config.motor_to_joint_dict[motor_id],
                )
                logger.warning(
                    "offset calibration failed for motor %d; limit not recorded",
                    motor_id,
                )
                hand.enable_torque([motor_id])
                continue
            time.sleep(TINY_SLEEP)
            avg_limit = float(read_motor_pos_checked(hand)[idx])
            state.motors_with_final_offset.add(motor_id)

        bound = 1 if directions[motor_id] == 1 else 0
        pending_limits[motor_id][bound] = avg_limit
        _emit(
            progress_callback,
            "limit_recorded",
            motor=motor_id,
            joint=hand.config.motor_to_joint_dict[motor_id],
            limit=avg_limit,
            bound="upper" if bound else "lower",
        )
        logger.info(
            "motor %d (%s) reached the limit at %.4f rad",
            motor_id,
            hand.config.motor_to_joint_dict[motor_id],
            avg_limit,
        )

        hand.enable_torque([motor_id])

    return directions


def _manual_prompt(
    prompt_callback: Optional[ManualPromptCallback], **payload
) -> str:
    """Ask the operator what to do with this hardstop; default to skipping."""
    if prompt_callback is None:
        raise ValueError(
            "manual calibration needs a prompt_callback to ask the operator "
            "when each joint is at its hardstop"
        )
    try:
        answer = prompt_callback({"action": "capture_limit", **payload})
    except Exception:
        logger.exception("manual calibration prompt failed")
        return MANUAL_ABORT
    answer = str(answer).strip().lower()
    return answer if answer in MANUAL_ACTIONS else MANUAL_SKIP


def _capture_step_manually(
    hand: "OrcaHand",
    *,
    step_joints: Dict[str, str],
    state: _DriveState,
    encoder_pass: Optional[_EncoderPass],
    progress_callback: Optional[ProgressCallback],
    prompt_callback: Optional[ManualPromptCallback],
    should_stop: ShouldStop,
) -> Dict[int, int] | None:
    """Record each joint's hardstop from a pose the operator sets by hand.

    Torque stays off for the whole step, so the reading is already
    tension-free and needs no release: the operator holds the joint on its
    hardstop and confirms, and the motor position at that moment becomes the
    limit. This is the path for a joint whose motor cannot drive itself onto
    the stop — a stalling tendon, or a motor latched off by a hardware error.

    Returns the per-motor direction signs, or ``None`` if the operator aborted
    or ``should_stop`` fired.
    """
    hand.disable_torque()

    if should_stop():
        return None

    directions: Dict[int, int] = {}

    for joint, direction in step_joints.items():
        motor_id = hand.config.joint_to_motor_map[joint]
        sign = 1 if direction == FLEX else -1
        if hand.config.joint_inversion_dict.get(joint, False):
            sign = -sign

        _emit(
            progress_callback,
            "manual_capture_started",
            joint=joint,
            motor=motor_id,
            direction=direction,
        )

        answer = _manual_prompt(
            prompt_callback,
            joint=joint,
            motor=motor_id,
            direction=direction,
        )
        if answer == MANUAL_ABORT or should_stop():
            return None
        if answer == MANUAL_SKIP:
            _emit(
                progress_callback,
                "manual_capture_skipped",
                joint=joint,
                motor=motor_id,
                direction=direction,
            )
            logger.info("manual calibration: %s %s skipped", joint, direction)
            continue

        position = float(
            read_motor_pos_checked(hand)[hand.config.motor_id_to_idx_dict[motor_id]]
        )
        bound = 1 if sign == 1 else 0
        state.pending_limits[motor_id][bound] = position
        directions[motor_id] = sign

        _emit(
            progress_callback,
            "limit_recorded",
            motor=motor_id,
            joint=joint,
            limit=position,
            bound="upper" if bound else "lower",
        )
        logger.info(
            "motor %d (%s) hand-held limit recorded at %.4f rad",
            motor_id, joint, position,
        )

        # The operator is still holding this joint on its stop; the anchor has
        # to be sampled now, not after they let go of it.
        if encoder_pass is not None:
            _run_joint_encoder_pass_for_step(
                hand,
                step_joints={joint: direction},
                directions={motor_id: sign},
                encoder_pass=encoder_pass,
                progress_callback=progress_callback,
            )

    return directions


def _drive_calibration(
    hand: "OrcaHand",
    *,
    force_wrist: bool,
    joints: list[str] | None,
    joint_encoder_client,
    progress_callback: Optional[ProgressCallback],
    should_stop: ShouldStop,
    persist: bool,
    manual: bool = False,
    prompt_callback: Optional[ManualPromptCallback] = None,
) -> CalibrationResult | None:
    """Execute the calibration routine and return a :class:`~orca_core.CalibrationResult`.

    Drives each joint through its mechanical limits following ``calibration_sequence``
    from ``config.yaml``, records motor positions at each limit, and persists
    the resulting motor limits and joint-to-motor ratios to ``calibration.yaml``
    after every step. Returns ``None`` on early exit (stop requested).

    Per-step sequencing: drive the step's motors onto their hardstops, sample
    encoder anchors while still under torque, then release torque per motor to
    read the tension-free motor-side limit before committing the step.

    Wrist calibration logic:
    - Wrist is calibrated independently of fingers (tracked by `wrist_calibrated` in calibration file).
    - Wrist steps write `wrist_calibration_current`, though the wrist motor's
      multi_turn_position mode ignores current caps (PWM-limited torque).
    - If already calibrated (and calibration run is not forcing), skip wrist
      steps — unless the encoder pass is active and the wrist still lacks an
      encoder anchor, in which case the wrist steps run to capture it.
    - If missing from the sequence, wrist flex/extend steps are appended.
    - If force_wrist=True, always include wrist in calibration steps.
    - `wrist_calibrated` turns true only once both wrist limits are captured;
      a persisted flag without stored wrist limits is treated as uncalibrated.
    """
    wrist_in_sequence = any(
        "wrist" in step[JOINTS] for step in hand.config.calibration_sequence
    )
    calibration_sequence = list(hand.config.calibration_sequence)

    encoder_pass_active = (
        getattr(hand.config, "joint_feedback_enabled", False)
        and joint_encoder_client is not None
    )

    # A calibrated flag without recorded wrist limits is inconsistent; treat
    # the wrist as uncalibrated so it is driven again rather than skipped.
    wrist_motor_id = hand.config.joint_to_motor_map.get(WRIST)
    prior_wrist_limits = hand.calibration.motor_limits_dict.get(
        wrist_motor_id, [None, None]
    )
    wrist_calibrated = (
        hand.calibration.wrist_calibrated and None not in prior_wrist_limits
    )

    # An encoder-backed wrist without an anchor still needs its steps: the
    # anchor is sampled at the wrist FLEX hardstop during the sweep. A
    # ``joints`` restriction that excludes the wrist keeps the plain skip.
    wrist_anchor_needed = (
        encoder_pass_active
        and (joints is None or WRIST in joints)
        and WRIST in set(hand._encoder_backed_joints())
        and WRIST not in hand.calibration.joint_encoder_calibration_dict
    )
    if wrist_anchor_needed and wrist_calibrated and not force_wrist:
        logger.info(
            "wrist is motor-calibrated but missing its encoder anchor; "
            "running wrist steps to capture it"
        )

    if wrist_calibrated and not force_wrist and not wrist_anchor_needed:
        if wrist_in_sequence:
            _emit(progress_callback, "wrist_skipped")
            logger.warning(
                "wrist already calibrated; skipping wrist steps "
                "(force_wrist=True overrides)"
            )
        calibration_sequence = [
            step for step in calibration_sequence if WRIST not in step[JOINTS]
        ]
    elif not wrist_in_sequence:
        calibration_sequence.append(
            {STEP: len(calibration_sequence) + 1, JOINTS: {WRIST: FLEX}}
        )
        calibration_sequence.append(
            {STEP: len(calibration_sequence) + 1, JOINTS: {WRIST: EXTEND}}
        )

    if joints is not None:
        joints_set = set(joints)
        filtered = []
        for step in calibration_sequence:
            step_joints = {
                j: d for j, d in step[JOINTS].items() if j in joints_set
            }
            if step_joints:
                filtered.append({STEP: step[STEP], JOINTS: step_joints})
        logger.info(
            "calibrating %d joint(s) across %d step(s) (out of %d total)",
            len(joints_set),
            len(filtered),
            len(calibration_sequence),
        )
        calibration_sequence = filtered

    # motor_limits commits only once both directions land for a joint;
    # pending_limits holds the in-flight half so partial runs keep old values.
    motor_limits = {
        motor_id: list(limits)
        for motor_id, limits in hand.calibration.motor_limits_dict.items()
    }
    pending_limits: Dict[int, list] = {
        motor_id: [None, None] for motor_id in hand.config.motor_ids
    }
    joint_to_motor_ratios = dict(hand.calibration.joint_to_motor_ratios_dict)

    joint_encoder_calibration: Dict[str, JointEncoderCal] = dict(
        hand.calibration.joint_encoder_calibration_dict
    )
    joint_roms_measured: Dict[str, list] = {
        joint: list(rom)
        for joint, rom in hand.calibration.joint_roms_measured_dict.items()
    }
    # Existing anchors stay until their replacement is sampled, so an aborted
    # or failed run never loses anchors of joints it did not re-anchor.
    joints_to_anchor: set[str] = set()
    pending_anchors: Dict[str, int] = {}
    if encoder_pass_active:
        joints_to_anchor = {
            joint
            for step in calibration_sequence
            for joint, direction in step[JOINTS].items()
            if direction == FLEX
        }

    hand._compute_wrap_offsets_dict()

    for step in calibration_sequence:
        for joint in step[JOINTS].keys():
            motor_id = hand.config.joint_to_motor_map[joint]
            hand._clear_wrap_offset(motor_id)

    drive_state = _DriveState(pending_limits=pending_limits)
    encoder_pass = (
        _EncoderPass(
            client=joint_encoder_client,
            joints_to_anchor=joints_to_anchor,
            pending_anchors=pending_anchors,
            joint_encoder_calibration=joint_encoder_calibration,
            joint_roms_measured=joint_roms_measured,
        )
        if encoder_pass_active
        else None
    )

    motor_travel_measured: Dict[str, float] = dict(
        hand.calibration.motor_travel_measured_dict
    )
    boosted_joints: Dict[str, float] = {}

    calibrated_joints: dict = {}

    if manual:
        # Nothing is driven, and the hand has to stay limp enough to move by
        # hand for the whole run.
        hand.disable_torque()
    else:
        hand.set_control_mode(CURRENT_BASED_POSITION)
        hand.set_max_current(hand.config.calibration_current)

    _emit(
        progress_callback,
        "calibration_started",
        steps=len(calibration_sequence),
        joints=sorted({j for step in calibration_sequence for j in step[JOINTS]}),
    )

    for step_index, step in enumerate(calibration_sequence):
        if should_stop():
            _emit(progress_callback, "calibration_aborted")
            return None

        _emit(
            progress_callback,
            "step_started",
            index=step_index,
            total=len(calibration_sequence),
            joints={j: d for j, d in step[JOINTS].items()},
        )

        directions = (
            _capture_step_manually(
                hand,
                step_joints=step[JOINTS],
                state=drive_state,
                encoder_pass=encoder_pass,
                progress_callback=progress_callback,
                prompt_callback=prompt_callback,
                should_stop=should_stop,
            )
            if manual
            else _drive_step(
                hand,
                step_joints=step[JOINTS],
                current_for=_nominal_step_current(hand),
                state=drive_state,
                encoder_pass=encoder_pass,
                progress_callback=progress_callback,
                should_stop=should_stop,
            )
        )
        # Mid-drive stop: joints are not at their hardstops, so bail before
        # anything captured from an arbitrary pose is committed.
        if directions is None:
            _emit(progress_callback, "calibration_aborted")
            return None

        calibrated_joints = {}
        for joint in step[JOINTS].keys():
            motor_id = hand.config.joint_to_motor_map[joint]
            if (
                pending_limits[motor_id][0] is None
                or pending_limits[motor_id][1] is None
            ):
                continue

            # Both bounds are in, so the joint's motor travel is known and can
            # be checked against its baseline before the limits are committed.
            if not _resolve_short_travel(
                hand,
                joint,
                motor_id,
                state=drive_state,
                encoder_pass=encoder_pass,
                motor_travel_measured=motor_travel_measured,
                boosted_joints=boosted_joints,
                progress_callback=progress_callback,
                should_stop=should_stop,
                manual=manual,
            ):
                _emit(progress_callback, "calibration_aborted")
                return None

            candidate = list(pending_limits[motor_id])
            delta_motor = candidate[1] - candidate[0]
            # Two limits this close together are not two hardstops — the motor
            # never turned (a latched hardware error refusing torque does
            # this). Committing the pair would write a ~zero joint-to-motor
            # ratio, which makes the joint uncommandable, and would leave the
            # wrap detector comparing live positions against a single point:
            # the corruption then survives into every later run. Keep whatever
            # calibration the joint already had and say so.
            if abs(delta_motor) < MIN_MOTOR_TRAVEL_RAD:
                _emit(
                    progress_callback,
                    "limits_rejected",
                    joint=joint,
                    motor=motor_id,
                    travel_deg=motor_travel_deg(candidate),
                    reason="degenerate",
                )
                logger.error(
                    "joint %s (motor %d) swept only %.2f deg of motor travel: "
                    "its two limits are the same point, so the motor did not "
                    "turn. Refusing to record it — check that the motor "
                    "accepts torque (a latched hardware error needs a power "
                    "cycle) and that the tendon is connected.",
                    joint, motor_id, motor_travel_deg(candidate) or 0.0,
                )
                motor_travel_measured.pop(joint, None)
                continue

            motor_limits[motor_id] = candidate
            # The encoder pass for this step has already run, so a joint whose
            # travel was measured this run fits its ratio against the measured
            # span — putting the map's degrees in the encoders' own units. The
            # rest fall back to the config nominal.
            rom = joint_roms_measured.get(joint) or hand.config.joint_roms_dict[joint]
            delta_joint = rom[1] - rom[0]
            joint_to_motor_ratios[motor_id] = float(delta_motor / delta_joint)
            logger.info("joint calibrated: %s", joint)
            _emit(
                progress_callback,
                "joint_calibrated",
                joint=joint,
                ratio=joint_to_motor_ratios[motor_id],
            )
            calibrated_joints[joint] = 0.0

        if WRIST in calibrated_joints:
            wrist_calibrated = True
        hand.calibration = _build_calibration_result(
            motor_limits=motor_limits,
            joint_to_motor_ratios=joint_to_motor_ratios,
            wrist_calibrated=wrist_calibrated,
            joint_encoder_calibration_dict=joint_encoder_calibration,
            joint_roms_measured_dict=joint_roms_measured,
            motor_travel_measured_dict=motor_travel_measured,
        )
        # Persist partial progress after every step so an interrupted run
        # never loses the work already done.
        if persist:
            persist_calibration(
                hand.config.calibration_path,
                result=hand.calibration,
                include_encoder=encoder_pass_active,
            )

        if calibrated_joints and not manual:
            hand.set_joint_positions(
                calibrated_joints, num_steps=NUM_STEPS, step_size=STEP_SIZE
            )

        _emit(progress_callback, "step_done", index=step_index, total=len(calibration_sequence))

        # TODO(fracapuano): Is this necessary?
        time.sleep(0.1)

    # A pending anchor whose joint saw no extend step this run could not be
    # sweep-validated; commit it so such sequences keep their anchors.
    for joint, count in list(pending_anchors.items()):
        _commit_anchor(
            hand,
            joint,
            count,
            joint_encoder_calibration=joint_encoder_calibration,
            joints_to_anchor=joints_to_anchor,
            progress_callback=progress_callback,
        )
    pending_anchors.clear()

    final_result = _build_calibration_result(
        motor_limits=motor_limits,
        joint_to_motor_ratios=joint_to_motor_ratios,
        wrist_calibrated=wrist_calibrated,
        joint_encoder_calibration_dict=joint_encoder_calibration,
        joint_roms_measured_dict=joint_roms_measured,
        motor_travel_measured_dict=motor_travel_measured,
    )
    hand.calibration = final_result
    if persist:
        persist_calibration(
            hand.config.calibration_path,
            result=final_result,
            include_encoder=encoder_pass_active,
        )

    if calibrated_joints:
        hand.set_joint_positions(
            calibrated_joints, num_steps=NUM_STEPS, step_size=STEP_SIZE
        )

    hand.set_max_current(hand.config.max_current)

    if boosted_joints:
        logger.warning(
            "%d joint(s) needed a higher-current re-drive to reach their motor "
            "travel baseline: %s. That is a symptom of over-tensioned tendons; "
            "re-run tensioning if it persists.",
            len(boosted_joints),
            ", ".join(
                f"{joint} @ {current:.0f} mA"
                for joint, current in sorted(boosted_joints.items())
            ),
        )

    _emit(
        progress_callback,
        "calibration_done",
        boosted_joints=dict(boosted_joints),
        motor_travel_deg=dict(motor_travel_measured),
    )
    return final_result


def _redrive_joint(
    hand: "OrcaHand",
    joint: str,
    motor_id: int,
    *,
    current: float,
    state: _DriveState,
    encoder_pass: Optional[_EncoderPass],
    progress_callback: Optional[ProgressCallback],
    should_stop: ShouldStop,
) -> bool:
    """Re-drive one joint onto both hardstops at ``current`` and re-take its limits.

    The elevated current is scoped to this re-drive: the nominal calibration
    current is restored before returning, however the drive ended. Returns
    ``False`` if ``should_stop`` fired, leaving the joint's limits incomplete.
    """
    state.pending_limits[motor_id] = [None, None]

    # The stored anchor and measured ROM were sampled at the hardstops this
    # re-drive is about to replace, so they are re-sampled alongside them. A
    # re-drive that fails to commit a new anchor keeps the previous one, as
    # everywhere else in this routine.
    if encoder_pass is not None and joint in set(hand._encoder_backed_joints()):
        encoder_pass.pending_anchors.pop(joint, None)
        encoder_pass.joints_to_anchor.add(joint)

    current_for = lambda _joint: current  # noqa: E731
    try:
        for direction in (FLEX, EXTEND):
            if _drive_step(
                hand,
                step_joints={joint: direction},
                current_for=current_for,
                state=state,
                encoder_pass=encoder_pass,
                progress_callback=progress_callback,
                should_stop=should_stop,
            ) is None:
                return False
    finally:
        hand.set_max_current(hand.config.calibration_current)
    return True


def _resolve_short_travel(
    hand: "OrcaHand",
    joint: str,
    motor_id: int,
    *,
    state: _DriveState,
    encoder_pass: Optional[_EncoderPass],
    motor_travel_measured: Dict[str, float],
    boosted_joints: Dict[str, float],
    progress_callback: Optional[ProgressCallback],
    should_stop: ShouldStop,
    manual: bool = False,
) -> bool:
    """Check the joint's motor travel against its baseline, re-driving if short.

    An over-tensioned tendon makes the drive stall before the hardstop, so the
    joint's two limits come out closer together than the spool geometry allows
    and every angle derived from them is wrong. When the travel falls below
    ``joint_motor_travel[joint]`` by more than ``calibration_travel_margin``,
    the joint is re-driven at a higher current — ramped over
    ``calibration_travel_retries`` attempts — and the best travel achieved is
    kept. Travel *above* the baseline is reported but not re-driven: more
    current cannot shorten a span, and the cause is a slipped tendon or a stale
    baseline rather than tension.

    A joint with no configured baseline is measured and left alone, as is a
    motor whose torque the current cap does not govern. Returns ``False`` if
    ``should_stop`` fired during a re-drive.
    """
    config = hand.config
    measured = motor_travel_deg(state.pending_limits[motor_id])
    motor_travel_measured[joint] = measured
    expected = config.expected_motor_travel_deg(joint)

    if expected is None:
        _emit(
            progress_callback,
            "travel_baseline_missing",
            joint=joint,
            motor=motor_id,
            travel_deg=measured,
        )
        logger.info(
            "joint %s (motor %d) motor travel %.2f deg; no joint_motor_travel "
            "baseline configured, so it was not checked",
            joint, motor_id, measured,
        )
        return True

    margin = config.calibration_travel_margin
    lower, upper = expected * (1 - margin), expected * (1 + margin)
    _emit(
        progress_callback,
        "travel_checked",
        joint=joint,
        motor=motor_id,
        travel_deg=measured,
        expected_deg=expected,
        deviation=travel_deviation(measured, expected),
        bounds_deg=[lower, upper],
        within_margin=lower <= measured <= upper,
    )

    if measured > upper:
        _emit(
            progress_callback,
            "travel_excess",
            joint=joint,
            motor=motor_id,
            travel_deg=measured,
            expected_deg=expected,
            deviation=travel_deviation(measured, expected),
        )
        logger.warning(
            "joint %s (motor %d) travelled %.2f deg, %+.0f%% past the %.2f deg "
            "baseline. Raising the current cannot shorten a span: check the "
            "tendon for slip and the joint_motor_travel baseline for staleness.",
            joint, motor_id, measured,
            100 * travel_deviation(measured, expected), expected,
        )
        return True

    if measured >= lower:
        logger.info(
            "joint %s (motor %d) motor travel %.2f deg (%+.0f%% of the %.2f deg "
            "baseline), within the %.0f%% margin",
            joint, motor_id, measured,
            100 * travel_deviation(measured, expected), expected, 100 * margin,
        )
        return True

    logger.warning(
        "joint %s (motor %d) travelled only %.2f deg of the %.2f deg baseline "
        "(%.0f%% short, margin %.0f%%); it stopped before its hardstop, which "
        "an over-tensioned tendon does.",
        joint, motor_id, measured, expected,
        -100 * travel_deviation(measured, expected), 100 * margin,
    )

    # A multi-turn wrist is PWM-limited: its mode ignores the current cap, so
    # a re-drive would repeat the same drive with the same torque.
    if _skips_torque_release(hand, motor_id):
        _emit(
            progress_callback,
            "travel_retry_unavailable",
            joint=joint,
            motor=motor_id,
            travel_deg=measured,
            expected_deg=expected,
        )
        logger.error(
            "joint %s (motor %d) travelled only %.2f deg of its %.2f deg "
            "baseline. Its control mode ignores the current cap, so raising "
            "the current cannot help: check the hardstop and the tendon.",
            joint, motor_id, measured, expected,
        )
        return True

    # A joint at a few percent of its baseline did not travel and then stall;
    # it never left its starting point. More current cannot free a motor that
    # is blocked, unpowered, or disconnected from its tendon — it just drives
    # a stalled motor harder, which is how the calibration current cooks a
    # motor. Report and leave it to the operator.
    if measured < expected * MIN_TRAVEL_FRACTION:
        _emit(
            progress_callback,
            "travel_retry_skipped",
            joint=joint,
            motor=motor_id,
            travel_deg=measured,
            expected_deg=expected,
            floor_deg=expected * MIN_TRAVEL_FRACTION,
        )
        logger.error(
            "joint %s (motor %d) travelled %.2f deg of its %.2f deg baseline "
            "(under the %.0f%% floor): it did not move at all, so no "
            "higher-current re-drive was attempted. Check that the motor "
            "accepts torque, that the tendon is connected, and that nothing "
            "is blocking the joint.",
            joint, motor_id, measured, expected, 100 * MIN_TRAVEL_FRACTION,
        )
        return True

    if manual or config.calibration_travel_retries < 1:
        _emit(
            progress_callback,
            "travel_retry_disabled",
            joint=joint,
            motor=motor_id,
            travel_deg=measured,
            expected_deg=expected,
            reason="manual" if manual else "retries_disabled",
        )
        logger.error(
            "joint %s (motor %d) is calibrated over a shortened range: %s, so "
            "no higher-current re-drive was attempted.",
            joint, motor_id,
            "this is a hand-held calibration and nothing is driven"
            if manual else "calibration_travel_retries is 0",
        )
        return True

    # The re-drive is capped at calibration_max_current. With no headroom it
    # repeats the pass that already stalled, so say so here rather than leaving
    # the operator to wonder why the boost changed nothing.
    if config.calibration_retry_current_resolved > config.calibration_max_current:
        logger.warning(
            "the re-drive for %s is capped at calibration_max_current (%d mA): "
            "it would otherwise use %d mA, so it repeats the pass that already "
            "stalled. Raise calibration_max_current or lower "
            "calibration_current (%d mA) to give it headroom.",
            joint, config.calibration_max_current,
            config.calibration_retry_current_resolved,
            config.calibration_current,
        )

    best_limits = list(state.pending_limits[motor_id])
    best_travel = measured

    for attempt in range(1, config.calibration_travel_retries + 1):
        current = config.retry_current_for_attempt(attempt)
        _emit(
            progress_callback,
            "travel_retry_started",
            joint=joint,
            motor=motor_id,
            attempt=attempt,
            attempts=config.calibration_travel_retries,
            current=current,
            travel_deg=best_travel,
            expected_deg=expected,
        )
        logger.warning(
            "joint %s (motor %d) re-drive %d/%d at %.0f mA (nominal %.0f mA)",
            joint, motor_id, attempt, config.calibration_travel_retries,
            current, config.calibration_current,
        )

        if not _redrive_joint(
            hand,
            joint,
            motor_id,
            current=current,
            state=state,
            encoder_pass=encoder_pass,
            progress_callback=progress_callback,
            should_stop=should_stop,
        ):
            # Aborted mid-re-drive: restore the best limits seen so the run's
            # partial results stay usable rather than half-captured.
            state.pending_limits[motor_id] = best_limits
            motor_travel_measured[joint] = best_travel
            return False

        retried = motor_travel_deg(state.pending_limits[motor_id])
        if retried is None:
            # The re-drive lost a bound (a failed release or offset command);
            # the previous attempt's limits are the better record.
            state.pending_limits[motor_id] = list(best_limits)
            continue
        if retried > best_travel:
            best_travel, best_limits = retried, list(state.pending_limits[motor_id])

        if retried >= lower:
            motor_travel_measured[joint] = retried
            boosted_joints[joint] = current
            _emit(
                progress_callback,
                "travel_retry_succeeded",
                joint=joint,
                motor=motor_id,
                attempt=attempt,
                current=current,
                travel_deg=retried,
                expected_deg=expected,
                deviation=travel_deviation(retried, expected),
            )
            logger.warning(
                "joint %s (motor %d) reached %.2f deg of its %.2f deg baseline "
                "at %.0f mA (was %.2f deg at %.0f mA); calibrated from the "
                "re-driven limits",
                joint, motor_id, retried, expected, current,
                measured, config.calibration_current,
            )
            return True

        logger.warning(
            "joint %s (motor %d) still only %.2f deg of %.2f deg after re-drive "
            "%d/%d at %.0f mA",
            joint, motor_id, retried, expected, attempt,
            config.calibration_travel_retries, current,
        )

    state.pending_limits[motor_id] = list(best_limits)
    motor_travel_measured[joint] = best_travel
    if best_travel > measured:
        boosted_joints[joint] = config.retry_current_for_attempt(
            config.calibration_travel_retries
        )
    _emit(
        progress_callback,
        "travel_retry_exhausted",
        joint=joint,
        motor=motor_id,
        attempts=config.calibration_travel_retries,
        travel_deg=best_travel,
        expected_deg=expected,
        deviation=travel_deviation(best_travel, expected),
    )
    logger.error(
        "joint %s (motor %d) never reached its %.2f deg baseline: best %.2f deg "
        "after %d re-drive(s) up to %.0f mA. The joint is calibrated over a "
        "shortened range - slacken the tendon, check the hardstop, or update "
        "the joint_motor_travel baseline.",
        joint, motor_id, expected, best_travel,
        config.calibration_travel_retries,
        config.retry_current_for_attempt(config.calibration_travel_retries),
    )
    return True


def _commit_anchor(
    hand: "OrcaHand",
    joint: str,
    anchor_count: int,
    *,
    joint_encoder_calibration: Dict[str, JointEncoderCal],
    joints_to_anchor: set,
    progress_callback: Optional[ProgressCallback],
) -> None:
    joint_encoder_calibration[joint] = JointEncoderCal(
        enc_at_anchor_count=anchor_count,
    )
    joints_to_anchor.discard(joint)
    anchor_angle_deg = float(hand.config.joint_roms_dict[joint][1])
    _emit(
        progress_callback,
        "encoder_anchor_recorded",
        joint=joint,
        anchor_count=anchor_count,
        anchor_angle_deg=anchor_angle_deg,
    )
    logger.info(
        "joint %s encoder anchor sampled: anchor_count=%d (at ROM upper %.2f deg)",
        joint,
        anchor_count,
        anchor_angle_deg,
    )


def _commit_measured_rom(
    hand: "OrcaHand",
    joint: str,
    span_deg: float,
    *,
    flex_count: int,
    extend_count: int,
    joint_roms_measured: Dict[str, list],
    progress_callback: Optional[ProgressCallback],
) -> None:
    """Record the ROM implied by an encoder-measured hardstop-to-hardstop span.

    The anchor pose — the flex hardstop, held to be the config ROM upper — is
    the fixed reference of the encoder frame, so the measured span places the
    lower endpoint and the upper carries through unchanged. A span implying a
    lower endpoint too far from nominal is rejected, leaving the joint on its
    config ROM.

    Both events carry the raw magnet counts sampled at the two hardstops
    (``flex_count``/``extend_count``) so successive calibrations can be
    compared for magnet drift.
    """
    rom_lower, rom_upper = hand.config.joint_roms_dict[joint]
    measured_lower = rom_upper - span_deg
    deviation = measured_lower - rom_lower

    if abs(deviation) > MEASURED_ROM_REJECT_TOL_DEG:
        joint_roms_measured.pop(joint, None)
        _emit(
            progress_callback,
            "measured_rom_rejected",
            joint=joint,
            span_deg=span_deg,
            deviation_deg=deviation,
            flex_count=flex_count,
            extend_count=extend_count,
        )
        logger.warning(
            "joint %s measured span %.2f deg puts its lower hardstop %+.2f deg "
            "from the configured %.2f deg, beyond the %.1f deg limit; keeping "
            "the configured ROM. Check the hardstop, the encoder wiring, and "
            "the configured ROM.",
            joint, span_deg, deviation, rom_lower, MEASURED_ROM_REJECT_TOL_DEG,
        )
        return

    if abs(deviation) > MEASURED_ROM_WARN_TOL_DEG:
        logger.warning(
            "joint %s measured span %.2f deg puts its lower hardstop %+.2f deg "
            "from the configured %.2f deg",
            joint, span_deg, deviation, rom_lower,
        )

    joint_roms_measured[joint] = [float(measured_lower), float(rom_upper)]
    _emit(
        progress_callback,
        "measured_rom_recorded",
        joint=joint,
        rom=[float(measured_lower), float(rom_upper)],
        deviation_deg=deviation,
        flex_count=flex_count,
        extend_count=extend_count,
    )
    logger.info(
        "joint %s measured ROM: [%.2f, %.2f] deg (span %.2f, %+.2f deg vs config)",
        joint, measured_lower, rom_upper, span_deg, deviation,
    )


def _run_joint_encoder_pass_for_step(
    hand: "OrcaHand",
    *,
    step_joints: Dict[str, str],
    directions: Dict[int, int],
    encoder_pass: _EncoderPass,
    progress_callback: Optional[ProgressCallback] = None,
) -> None:
    """Sample and sweep-validate anchor counts for the joints in a step.

    A FLEX-direction joint still awaiting an anchor is sampled at its flex
    hardstop into ``pending_anchors``; its later EXTEND step samples the
    opposite hardstop and commits the anchor only when the two counts differ
    by a meaningful fraction of the ROM span. A dead or unwired slot reads a
    constant and is rejected — including any previously stored anchor for it.
    The caller must hold the step's motors torqued at their hardstops. A
    failed sample keeps the previously persisted anchor.

    The same pair of hardstop samples measures the joint's actual travel, so a
    committed anchor also commits a measured ROM (see
    :func:`_commit_measured_rom`).
    """
    from ..hardware.sensing.constants import JOINT_TO_ENCODER_SLOT

    joints_to_anchor = encoder_pass.joints_to_anchor
    pending_anchors = encoder_pass.pending_anchors
    joint_encoder_calibration = encoder_pass.joint_encoder_calibration
    joint_roms_measured = encoder_pass.joint_roms_measured
    joint_encoder_client = encoder_pass.client

    encoder_backed = set(hand._encoder_backed_joints())

    for joint, direction in step_joints.items():
        if joint not in encoder_backed:
            continue
        slot = JOINT_TO_ENCODER_SLOT[joint]
        motor_id = hand.config.joint_to_motor_map.get(joint)
        if motor_id is None or motor_id not in directions:
            continue

        if direction == FLEX and joint in joints_to_anchor:
            try:
                pending_anchors[joint] = int(
                    sample_anchor_count_from_client(joint_encoder_client, slot=slot)
                )
            except JointEncoderCalibrationError as e:
                _emit(
                    progress_callback,
                    "encoder_anchor_failed",
                    joint=joint,
                    error=str(e),
                )
                logger.warning(
                    "encoder anchor sample failed for joint %s: %s", joint, e
                )
        elif direction == EXTEND and joint in pending_anchors:
            try:
                extend_count = int(
                    sample_anchor_count_from_client(joint_encoder_client, slot=slot)
                )
            except JointEncoderCalibrationError as e:
                pending_anchors.pop(joint)
                _emit(
                    progress_callback,
                    "encoder_anchor_failed",
                    joint=joint,
                    error=str(e),
                )
                logger.warning(
                    "encoder sweep check failed for joint %s: %s", joint, e
                )
                continue

            rom_lower, rom_upper = hand.config.joint_roms_dict[joint]
            expected = (rom_upper - rom_lower) * ENCODER_COUNTS_PER_REV / 360.0
            threshold = max(
                MIN_ANCHOR_SWEEP_COUNTS, MIN_ANCHOR_SWEEP_FRACTION * expected
            )
            delta = _count_delta(pending_anchors[joint], extend_count)
            if delta < threshold:
                pending_anchors.pop(joint)
                # A slot proven dead now also invalidates any stored anchor and
                # the measured ROM derived from the same samples.
                joint_encoder_calibration.pop(joint, None)
                joint_roms_measured.pop(joint, None)
                _emit(
                    progress_callback,
                    "encoder_anchor_failed",
                    joint=joint,
                    error=(
                        f"slot {slot} did not track the sweep ({delta} of "
                        f"~{int(expected)} expected counts) — encoder dead or "
                        "unwired; joint stays open-loop"
                    ),
                )
                logger.warning(
                    "joint %s encoder slot %d did not track the calibration "
                    "sweep (%d of ~%d expected counts); no anchor recorded",
                    joint, slot, delta, int(expected),
                )
                continue

            flex_count = pending_anchors[joint]
            _commit_anchor(
                hand,
                joint,
                pending_anchors.pop(joint),
                joint_encoder_calibration=joint_encoder_calibration,
                joints_to_anchor=joints_to_anchor,
                progress_callback=progress_callback,
            )
            # The two hardstop samples that validated the anchor also measure
            # the joint's actual travel. The span is wrap-aware and unsigned,
            # so it is independent of the slot's mounting polarity.
            _commit_measured_rom(
                hand,
                joint,
                delta * ENCODER_LSB_DEG,
                flex_count=flex_count,
                extend_count=extend_count,
                joint_roms_measured=joint_roms_measured,
                progress_callback=progress_callback,
            )
