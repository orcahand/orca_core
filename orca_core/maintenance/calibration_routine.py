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
import time
from collections import deque
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
    EXTEND,
    FLEX,
    JOINT_ENCODER_CALIBRATION,
    JOINT_ROMS_MEASURED,
    JOINT_TO_MOTOR_RATIOS,
    JOINTS,
    MOTOR_LIMITS_DICT,
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

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

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
            ``wrist_skipped``, ``step_done``, ``calibration_done``,
            ``calibration_aborted``,
            ``cleanup_failed``). Called from the calibrating thread; must be
            fast and non-blocking. Exceptions raised by the callback are
            swallowed.
        should_stop: Optional ``callable() -> bool`` polled between motor
            commands; return ``True`` to abort cooperatively.
        persist: When ``False``, nothing is written to ``calibration.yaml``;
            results are still committed to ``hand.calibration`` in memory.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

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


def _drive_calibration(
    hand: "OrcaHand",
    *,
    force_wrist: bool,
    joints: list[str] | None,
    joint_encoder_client,
    progress_callback: Optional[ProgressCallback],
    should_stop: ShouldStop,
    persist: bool,
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

    motors_with_initial_offset = set()
    motors_with_final_offset = set()

    calibrated_joints: dict = {}

    hand.set_control_mode(CURRENT_BASED_POSITION)
    hand.set_max_current(hand.config.calibration_current)

    _emit(
        progress_callback,
        "calibration_started",
        steps=len(calibration_sequence),
        joints=sorted({j for step in calibration_sequence for j in step[JOINTS]}),
    )

    for step_index, step in enumerate(calibration_sequence):
        hand.disable_torque()

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

        desired_increment, motor_reached_limit, directions = {}, {}, {}
        position_buffers, calibrated_joints, position_logs, current_log = (
            {},
            {},
            {},
            {},
        )

        for joint, direction in step[JOINTS].items():
            motor_id = hand.config.joint_to_motor_map[joint]
            hand.enable_torque(motor_ids=[motor_id])
            logger.debug("torque enabled for motor %d (joint %s)", motor_id, joint)

            if should_stop():
                _emit(progress_callback, "calibration_aborted")
                return None

            hand.set_max_current(
                hand.config.calibration_current
                if joint != WRIST
                else hand.config.wrist_calibration_current
            )

            sign = 1 if direction == FLEX else -1
            if hand.config.joint_inversion_dict.get(joint, False):
                sign = -sign

            if (
                hand.motor_client.requires_offset_calibration
                and motor_id not in motors_with_initial_offset
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
                motors_with_initial_offset.add(motor_id)

            directions[motor_id] = sign
            position_buffers[motor_id] = deque(
                maxlen=hand.config.calibration_num_stable
            )
            position_logs[motor_id] = []
            current_log[motor_id] = []
            motor_reached_limit[motor_id] = False

        while not all(motor_reached_limit.values()) and not should_stop():
            desired_increment = {}
            for motor_id, reached_limit in motor_reached_limit.items():
                if not reached_limit:
                    desired_increment[motor_id] = (
                        directions[motor_id] * hand.config.calibration_step_size
                    )

            hand._set_motor_pos(desired_increment, rel_to_current=True)
            time.sleep(hand.config.calibration_step_period)
            with hand._motor_lock:
                state = hand.motor_client.read_position_velocity_current()
                read_ok = hand.motor_client.last_read_ok
            curr_pos, curr_current = state.position, state.current

            # A failed bulk read returns the stale cache; feeding it into the
            # stability buffers would fake a hardstop detection. Skip and retry.
            if not read_ok:
                continue

            for motor_id in desired_increment.keys():
                if not motor_reached_limit[motor_id]:
                    idx = hand.config.motor_id_to_idx_dict[motor_id]
                    position_buffers[motor_id].append(curr_pos[idx])
                    position_logs[motor_id].append(float(curr_pos[idx]))
                    current_log[motor_id].append(float(curr_current[idx]))

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

        # Mid-drive stop: joints are not at their hardstops, so bail before the
        # anchor pass and limit capture persist values from an arbitrary pose.
        if should_stop():
            _emit(progress_callback, "calibration_aborted")
            return None

        # Motors are still pressing their hardstops with calibration current:
        # sample encoder anchors before the release so counts match that pose.
        if encoder_pass_active:
            _run_joint_encoder_pass_for_step(
                hand,
                step=step,
                directions=directions,
                joints_to_anchor=joints_to_anchor,
                pending_anchors=pending_anchors,
                joint_encoder_calibration=joint_encoder_calibration,
                joint_roms_measured=joint_roms_measured,
                joint_encoder_client=joint_encoder_client,
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
                and motor_id not in motors_with_final_offset
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
                motors_with_final_offset.add(motor_id)

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

        for joint in step[JOINTS].keys():
            motor_id = hand.config.joint_to_motor_map[joint]
            if (
                pending_limits[motor_id][0] is None
                or pending_limits[motor_id][1] is None
            ):
                continue

            motor_limits[motor_id] = list(pending_limits[motor_id])
            delta_motor = motor_limits[motor_id][1] - motor_limits[motor_id][0]
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
        )
        # Persist partial progress after every step so an interrupted run
        # never loses the work already done.
        if persist:
            persist_calibration(
                hand.config.calibration_path,
                result=hand.calibration,
                include_encoder=encoder_pass_active,
            )

        if calibrated_joints:
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

    _emit(progress_callback, "calibration_done")
    return final_result


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
    joint_roms_measured: Dict[str, list],
    progress_callback: Optional[ProgressCallback],
) -> None:
    """Record the ROM implied by an encoder-measured hardstop-to-hardstop span.

    The anchor pose — the flex hardstop, held to be the config ROM upper — is
    the fixed reference of the encoder frame, so the measured span places the
    lower endpoint and the upper carries through unchanged. A span implying a
    lower endpoint too far from nominal is rejected, leaving the joint on its
    config ROM.
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
    )
    logger.info(
        "joint %s measured ROM: [%.2f, %.2f] deg (span %.2f, %+.2f deg vs config)",
        joint, measured_lower, rom_upper, span_deg, deviation,
    )


def _run_joint_encoder_pass_for_step(
    hand: "OrcaHand",
    *,
    step,
    directions: Dict[int, int],
    joints_to_anchor: set,
    pending_anchors: Dict[str, int],
    joint_encoder_calibration: Dict[str, JointEncoderCal],
    joint_roms_measured: Dict[str, list],
    joint_encoder_client,
    progress_callback: Optional[ProgressCallback] = None,
) -> None:
    """Sample and sweep-validate anchor counts for the joints in ``step``.

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

    encoder_backed = set(hand._encoder_backed_joints())

    for joint, direction in step[JOINTS].items():
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
                joint_roms_measured=joint_roms_measured,
                progress_callback=progress_callback,
            )
