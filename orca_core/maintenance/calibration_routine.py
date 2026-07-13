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
from ..utils.utils import update_yaml

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]


def run_calibration(
    hand: "OrcaHand",
    *,
    force_wrist: bool = False,
    joints: list[str] | None = None,
    joint_encoder_client=None,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> CalibrationResult | None:
    """Run the calibration routine on ``hand`` and return the result.

    Returns ``None`` on early exit (``should_stop`` triggered). On any exit
    without a result — abort or exception — torque is released and the
    configured current limit restored so the hand doesn't strain against a
    hardstop. Partial per-step progress is committed to ``hand.calibration``
    and ``calibration.yaml`` either way; applying the *final* result to the
    hand is the caller's choice.

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
            ``step_started``, ``joint_calibrated``, ``step_done``,
            ``calibration_done``, ``calibration_aborted``). Called from the
            calibrating thread; must be fast and non-blocking. Exceptions
            raised by the callback are swallowed.
        should_stop: Optional ``callable() -> bool`` polled between motor
            commands; return ``True`` to abort cooperatively.
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
        )
    finally:
        if result is None:
            _release_after_abort(hand)
    return result


def _release_after_abort(hand: "OrcaHand") -> None:
    """Release torque and restore the configured current limit so the hand
    doesn't strain against a hardstop after an abnormal exit."""
    try:
        hand.set_max_current(hand.config.max_current)
        hand.disable_torque()
    except Exception as e:
        print(
            f"\033[91mWarning: cleanup after aborted calibration failed: {e}\033[0m"
        )


def _build_calibration_result(
    motor_limits: Dict[int, list],
    joint_to_motor_ratios: Dict[int, float],
    wrist_calibrated: bool,
    joint_encoder_calibration_dict: Dict[str, JointEncoderCal] | None = None,
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
    )


def _drive_calibration(
    hand: "OrcaHand",
    *,
    force_wrist: bool,
    joints: list[str] | None,
    joint_encoder_client,
    progress_callback: Optional[ProgressCallback],
    should_stop: ShouldStop,
) -> CalibrationResult | None:
    """Execute the calibration routine and return a :class:`~orca_core.CalibrationResult`.

    Drives each joint through its mechanical limits following ``calibration_sequence``
    from ``config.yaml``, records motor positions at each limit, and persists
    the resulting motor limits and joint-to-motor ratios to ``calibration.yaml``
    after every step. Returns ``None`` on early exit (stop requested).

    Wrist calibration logic:
    - Wrist is calibrated independently of fingers (tracked by `wrist_calibrated` in calibration file).
    - Uses a higher calibration current.
    - If already calibrated (and calibration run is not forcing), skip wrist steps.
    - If missing from sequence, is calibrated.
    - If force_wrist=True, always include wrist in calibration steps.
    """

    def _emit(event: str, **payload) -> None:
        if progress_callback is None:
            return
        try:
            progress_callback({"event": event, **payload})
        except Exception as e:
            print(f"Warning: calibration progress callback failed: {e}")

    wrist_in_sequence = any(
        "wrist" in step[JOINTS] for step in hand.config.calibration_sequence
    )
    calibration_sequence = list(hand.config.calibration_sequence)

    if hand.wrist_calibrated and not force_wrist:
        if wrist_in_sequence:
            print(
                "WARNING: Wrist is already calibrated. Skipping wrist calibration. Use --force-wrist to override."
            )
        calibration_sequence = [
            step for step in calibration_sequence if WRIST not in step[JOINTS]
        ]
    elif not wrist_in_sequence:
        # Adds wrist to calibration sequence
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
        print(
            f"Calibrating {len(joints_set)} joint(s) across {len(filtered)} "
            f"step(s) (out of {len(calibration_sequence)} total)."
        )
        calibration_sequence = filtered

    # motor_limits is committed only once both flex and extend land for a
    # joint; pending_limits holds the in-flight half so a single-direction
    # run does not erase prior good values.
    motor_limits = {
        motor_id: list(limits)
        for motor_id, limits in hand.calibration.motor_limits_dict.items()
    }
    pending_limits: Dict[int, list] = {
        motor_id: [None, None] for motor_id in hand.config.motor_ids
    }
    joint_to_motor_ratios = dict(hand.calibration.joint_to_motor_ratios_dict)

    encoder_pass_active = (
        getattr(hand.config, "joint_feedback_enabled", False)
        and joint_encoder_client is not None
    )
    joint_encoder_calibration: Dict[str, JointEncoderCal] = dict(
        hand.calibration.joint_encoder_calibration_dict
    )
    if encoder_pass_active:
        for step in calibration_sequence:
            for joint in step[JOINTS]:
                joint_encoder_calibration.pop(joint, None)

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
        "calibration_started",
        steps=len(calibration_sequence),
        joints=sorted({j for step in calibration_sequence for j in step[JOINTS]}),
    )

    for step_index, step in enumerate(calibration_sequence):
        hand.disable_torque()

        if should_stop():
            _emit("calibration_aborted")
            return None

        _emit(
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
            hand.enable_torque(motor_ids=[hand.config.joint_to_motor_map[joint]])
            print(
                "Enabling torque for the following motor: ",
                hand.config.joint_to_motor_map[joint],
            )

            if should_stop():
                _emit("calibration_aborted")
                return None

            hand.set_max_current(
                hand.config.calibration_current
                if joint != WRIST
                else hand.config.wrist_calibration_current
            )

            motor_id = hand.config.joint_to_motor_map[joint]
            sign = 1 if direction == FLEX else -1
            if hand.config.joint_inversion_dict.get(joint, False):
                sign = -sign

            directions[motor_id] = sign
            position_buffers[motor_id] = deque(
                maxlen=hand.config.calibration_num_stable
            )
            position_logs[motor_id] = []
            current_log[motor_id] = []
            motor_reached_limit[motor_id] = False

            if (
                hand.motor_client.requires_offset_calibration
                and motor_id not in motors_with_initial_offset
            ):
                hand.motor_client.calibrate_offset(motor_id, upper=(sign < 0))
                motors_with_initial_offset.add(motor_id)

        while not all(motor_reached_limit.values()) and not should_stop():
            desired_increment = {}
            for motor_id, reached_limit in motor_reached_limit.items():
                if not reached_limit:
                    desired_increment[motor_id] = (
                        directions[motor_id] * hand.config.calibration_step_size
                    )

            hand._set_motor_pos(desired_increment, rel_to_current=True)
            time.sleep(hand.config.calibration_step_period)
            curr_pos = hand.get_motor_pos()
            curr_current = hand.get_motor_current()

            # A failed bulk read returns the stale cache; feeding it into
            # the stability buffers would fake a "motor stopped moving"
            # hardstop detection. Skip this sample and try again.
            if not hand.motor_client.last_read_ok:
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
                        # Wrist limit is read from the stable-position
                        # buffer (no torque release). Non-wrist motors
                        # are kept under torque for the encoder anchor
                        # pass below; their motor-side limit is read
                        # post-release after that.
                        if WRIST in hand.config.motor_to_joint_dict[motor_id]:
                            avg_limit = float(np.mean(position_buffers[motor_id]))
                            print(
                                f"Motor {motor_id} corresponding to joint {hand.config.motor_to_joint_dict[motor_id]} reached the limit at {avg_limit} rad."
                            )
                            if directions[motor_id] == 1:
                                pending_limits[motor_id][1] = avg_limit
                            if directions[motor_id] == -1:
                                pending_limits[motor_id][0] = avg_limit

        # Stop requested mid-drive: the joints are NOT at their hardstops.
        # Bail out before the encoder-anchor pass and limit capture would
        # sample — and persist — values taken at an arbitrary pose.
        if should_stop():
            _emit("calibration_aborted")
            return None

        # All motors are pressing their hardstops with calibration current;
        # joints are firmly at the mechanical limit. Sample encoder anchors
        # NOW, before the torque release, so the anchor count corresponds
        # to the actual hardstop pose.
        if encoder_pass_active:
            _run_joint_encoder_pass_for_step(
                hand,
                step=step,
                directions=directions,
                joint_encoder_calibration=joint_encoder_calibration,
                joint_encoder_client=joint_encoder_client,
            )

        # Motor-side limit capture: release torque so tendon tension
        # doesn't bias the motor encoder reading at the hardstop, then
        # read the relaxed motor position, run any motor-type-specific
        # offset calibration, and re-enable torque.
        for motor_id in directions.keys():
            if motor_id not in motor_reached_limit or not motor_reached_limit[motor_id]:
                continue
            if WRIST in hand.config.motor_to_joint_dict[motor_id]:
                continue
            idx = hand.config.motor_id_to_idx_dict[motor_id]

            hand.disable_torque([motor_id])
            time.sleep(TINY_SLEEP)
            avg_limit = float(hand.get_motor_pos()[idx])
            print(
                f"Motor {motor_id} corresponding to joint {hand.config.motor_to_joint_dict[motor_id]} reached the limit at {avg_limit} rad."
            )
            if directions[motor_id] == 1:
                pending_limits[motor_id][1] = avg_limit
            if directions[motor_id] == -1:
                pending_limits[motor_id][0] = avg_limit

            if (
                hand.motor_client.requires_offset_calibration
                and motor_id not in motors_with_final_offset
            ):
                is_positive = directions[motor_id] > 0
                hand.motor_client.calibrate_offset(
                    motor_id, upper=is_positive
                )
                time.sleep(TINY_SLEEP)
                new_limit = float(hand.get_motor_pos()[idx])
                pending_limits[motor_id][1 if is_positive else 0] = new_limit
                print(
                    f"  (Offset adjusted: limit now at {new_limit} rad)"
                )
                motors_with_final_offset.add(motor_id)

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
            delta_joint = (
                hand.config.joint_roms_dict[joint][1]
                - hand.config.joint_roms_dict[joint][0]
            )
            joint_to_motor_ratios[motor_id] = float(delta_motor / delta_joint)
            print("Joint calibrated: ", joint)
            _emit(
                "joint_calibrated",
                joint=joint,
                ratio=joint_to_motor_ratios[motor_id],
            )
            calibrated_joints[joint] = 0.0

        # Persist partial progress after every step so an interrupted run
        # never loses the work already done.
        update_yaml(
            hand.config.calibration_path,
            JOINT_TO_MOTOR_RATIOS,
            joint_to_motor_ratios,
        )
        update_yaml(hand.config.calibration_path, MOTOR_LIMITS_DICT, motor_limits)

        step_wrist_calibrated = hand.calibration.wrist_calibrated or (
            WRIST in calibrated_joints
        )
        hand.calibration = _build_calibration_result(
            motor_limits=motor_limits,
            joint_to_motor_ratios=joint_to_motor_ratios,
            wrist_calibrated=step_wrist_calibrated,
            joint_encoder_calibration_dict=joint_encoder_calibration,
        )
        update_yaml(
            hand.config.calibration_path,
            WRIST_CALIBRATED,
            hand.calibration.wrist_calibrated,
        )
        update_yaml(
            hand.config.calibration_path,
            CALIBRATED,
            hand.calibration.calibrated,
        )
        if encoder_pass_active:
            update_yaml(
                hand.config.calibration_path,
                JOINT_ENCODER_CALIBRATION,
                joint_encoder_calibration_to_yaml(joint_encoder_calibration),
            )

        if calibrated_joints:
            hand.set_joint_positions(
                calibrated_joints, num_steps=NUM_STEPS, step_size=STEP_SIZE
            )

        _emit("step_done", index=step_index, total=len(calibration_sequence))

        # TODO(fracapuano): Is this necessary?
        time.sleep(0.1)

    new_wrist_calibrated = hand.calibration.wrist_calibrated
    if any(WRIST in step[JOINTS] for step in calibration_sequence):
        new_wrist_calibrated = True
        update_yaml(hand.config.calibration_path, WRIST_CALIBRATED, True)

    final_result = _build_calibration_result(
        motor_limits=motor_limits,
        joint_to_motor_ratios=joint_to_motor_ratios,
        wrist_calibrated=new_wrist_calibrated,
        joint_encoder_calibration_dict=joint_encoder_calibration,
    )
    hand.calibration = final_result
    update_yaml(hand.config.calibration_path, CALIBRATED, final_result.calibrated)

    if calibrated_joints:
        hand.set_joint_positions(
            calibrated_joints, num_steps=NUM_STEPS, step_size=TINY_SLEEP
        )

    hand.set_max_current(hand.config.max_current)

    _emit("calibration_done")
    return final_result


def _run_joint_encoder_pass_for_step(
    hand: "OrcaHand",
    *,
    step,
    directions: Dict[int, int],
    joint_encoder_calibration: Dict[str, JointEncoderCal],
    joint_encoder_client,
) -> None:
    """Sample anchor count for each not-yet-anchored joint in ``step``
    whose direction is FLEX (i.e. driven to its max ROM). Caller must
    hold motors torque-enabled at the FLEX hardstop. Joints not in
    ``_encoder_backed_joints()`` (no protocol slot, no motor, or not
    configured as sensed), joints already present in
    ``joint_encoder_calibration``, and EXTEND-direction steps are skipped.
    """
    from ..hardware.sensing.constants import JOINT_TO_ENCODER_SLOT

    encoder_backed = set(hand._encoder_backed_joints())

    for joint, direction in step[JOINTS].items():
        if direction != FLEX:
            continue
        if joint not in encoder_backed:
            continue
        if joint in joint_encoder_calibration:
            continue

        slot = JOINT_TO_ENCODER_SLOT[joint]
        if hand.config.joint_to_motor_map.get(joint) is None:
            continue

        try:
            anchor_count = sample_anchor_count_from_client(
                joint_encoder_client, slot=slot
            )
        except JointEncoderCalibrationError as e:
            print(
                f"\033[93mWARNING: encoder anchor sample failed for joint {joint}: {e}\033[0m"
            )
            continue

        joint_encoder_calibration[joint] = JointEncoderCal(
            enc_at_anchor_count=int(anchor_count),
        )
        anchor_angle_deg = float(hand.config.joint_roms_dict[joint][1])
        print(
            f"Joint {joint} encoder anchor sampled: "
            f"anchor_count={anchor_count} (at ROM upper {anchor_angle_deg:.2f}°)"
        )
