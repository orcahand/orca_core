# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import dataclasses
import logging
import math
import os
import threading
import time
from threading import RLock
from typing import Dict, List, Union

import numpy as np

from .base_hand import BaseHand
from .calibration import CalibrationResult
from .hand_config import OrcaHandConfig
from .hardware.motor_factory import create_motor_client
from .hardware.motor_client import MotorClient
from .hardware.motor_resolution import persist_resolved_driver, trial_probe
from .maintenance.calibration_routine import run_calibration
from .maintenance.tensioning import run_jitter, run_tension
from .utils.utils import (
    auto_detect_port,
    find_single_usb_serial_port,
    get_and_choose_port,
    read_yaml,
    update_yaml,
)

from .constants import (
    CALIBRATED,
    MODE_MAP,
    WRIST_MODE_VALUE,
    CURRENT_BASED_POSITION,
    CURRENT,
    WRIST,
    NUM_STEPS,
    POSITION,
    STEP_SIZE,
)

from .joint_position import OrcaJointPositions

# Motor-rad per joint-deg used to synthesise mock motor calibration. Sized so
# the widest bundled joint ROM stays inside the mock motors' simulated travel.
MOCK_JOINT_TO_MOTOR_RATIO = 0.007

logger = logging.getLogger(__name__)


class OrcaHand(BaseHand):
    """ORCA hand class.

    Extends :class:`~orca_core.BaseHand` with a full lifecycle for a physical
    hand: connection management, torque control, multi-mode motor control,
    (automatic) calibration, and background task execution.

    The recommended usage pattern is:

    >>> from orca_core import OrcaHand, OrcaJointPositions
    >>> hand = OrcaHand()
    >>> hand.connect()
    >>> hand.init_joints()  # enables torque, calibrates if needed
    >>> hand.set_joint_positions(OrcaJointPositions({"index_mcp": 0.5}))
    >>> hand.disconnect()

    Args:
        config_path: Path to a ``config.yaml`` file. Defaults to the bundled
            model when ``None``.
    """

    config_cls = OrcaHandConfig

    # Whether calibration state reaches calibration.yaml (the calibrate()
    # default and the stale-flag demote). Mock classes flip it off.
    _persist_calibration = True

    def __init__(
        self,
        config_path: str | None = None,
        calibration_path: str | None = None,
        model_version: str | None = None,
        model_name: str | None = None,
        config: OrcaHandConfig | None = None,
    ):
        super().__init__(
            config_path=config_path,
            config=config,
            calibration_path=calibration_path,
            model_version=model_version,
            model_name=model_name,
        )

        self._wrap_offsets_dict: Dict[int, float] = None
        self._motor_client: MotorClient = None
        self._motor_lock: RLock = RLock()
        self._uncalibrated_warned: set = set()

        self._task_thread: threading.Thread = None
        self._task_stop_event = threading.Event()
        self._lock = threading.Lock()
        self._current_task = None

        self.calibration = CalibrationResult.from_calibration_path(
            self.config.calibration_path, self.config.motor_ids
        )
        self._sanity_check()
        self.is_calibrated(verbose=True)

    def __del__(self):
        # Best-effort release of every link the subclass opened; a hand whose
        # __init__ raised can lack the attributes disconnect() touches.
        try:
            self.disconnect()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Calibration state — thin views onto self.calibration
    # ------------------------------------------------------------------

    @property
    def calibration(self) -> CalibrationResult:
        """The current :class:`~orca_core.calibration.CalibrationResult`."""
        return self._calibration

    @calibration.setter
    def calibration(self, value: CalibrationResult) -> None:
        self._calibration = value
        # Re-arm the warn-once set so a recalibration re-reports motors that
        # are still missing calibration data.
        self._uncalibrated_warned.clear()

    @property
    def motor_limits_dict(self) -> Dict[int, list]:
        return self.calibration.motor_limits_dict

    @property
    def joint_to_motor_ratios_dict(self) -> Dict[int, float]:
        return self.calibration.joint_to_motor_ratios_dict

    @property
    def calibrated(self) -> bool:
        return self.calibration.calibrated

    @property
    def wrist_calibrated(self) -> bool:
        return self.calibration.wrist_calibrated

    @property
    def motor_client(self) -> MotorClient | None:
        """The connected motor client, or ``None`` while disconnected.

        Advanced use only: reads and writes that race the hand's own bus
        traffic must go through the hand's lock-fenced methods instead.
        """
        return self._motor_client

    def _create_motor_client(self) -> MotorClient:
        return create_motor_client(
            self.config.motor_type,
            self.config.motor_ids,
            self.config.port,
            self.config.baudrate,
        )

    def _trial_probe(self, port: str) -> "tuple[str | None, int | None]":
        """Probe ``port`` for a responding (motor_type, baudrate) combination."""
        return trial_probe(self.config, port)

    def _resolve_motor_driver(self, port: str) -> bool:
        """Resolve and verify ``motor_type``/``baudrate`` for ``port``.

        Values pinned in yaml fix that probe axis; with both pinned the probe
        still verifies the single combination against the bus. Returns False
        when no motor family responded.
        """
        motor_type, baudrate = self._trial_probe(port)
        if motor_type is None or baudrate is None:
            return False
        self.config = dataclasses.replace(
            self.config, motor_type=motor_type, baudrate=baudrate
        )
        return True

    def _persist_resolved_driver(self, existing: "OrcaHandConfig") -> None:
        """Write driver fields the connect resolved (vs ``existing``) to config.yaml."""
        persist_resolved_driver(existing, self.config)

    def _connect_on_port(self, port: str, base_config: "OrcaHandConfig" = None) -> None:
        """Resolve the motor driver for ``port`` and open the client on it.

        Resolution starts from ``base_config`` (the yaml-pinned values), so a
        failed probe on an earlier port never leaks motor_type/baudrate into
        this attempt. Raises on failure so callers can run their recovery
        cascade.
        """
        self.config = dataclasses.replace(base_config or self.config, port=port)
        if not self._resolve_motor_driver(port):
            raise ConnectionError(
                f"no motor responded on {port} (check power and wiring)"
            )
        self._motor_client = self._create_motor_client()
        with self._motor_lock:
            self._motor_client.connect()

    def _try_port(
        self, port: str, base_config: "OrcaHandConfig"
    ) -> "Exception | None":
        """Attempt a connect on ``port``, returning the error on failure.

        On failure the opened client is closed and the config restored to
        ``base_config``, so every attempt in the cascade starts clean and a
        failed one can never leak an open (advisory-locked) port.
        """
        try:
            self._connect_on_port(port, base_config)
            self._persist_resolved_driver(base_config)
            return None
        except Exception as e:
            self._discard_motor_client()
            self.config = base_config
            return e

    def _discard_motor_client(self) -> None:
        """Drop the motor client, best-effort closing it first so a failed
        connect can't leak an open (and advisory-locked) serial port."""
        client, self._motor_client = self._motor_client, None
        if client is None:
            return
        try:
            client.disconnect()
        except Exception:
            pass

    def connect(
        self, interactive: bool = True, engage_feedback: bool = True
    ) -> tuple[bool, str]:
        """Open connection to the motor bus.

        Resolves (port, motor_type, baudrate) at connect time: the values in
        ``config.yaml`` win when present; a missing port is auto-detected via
        USB vendor ID (interactive picker as a last resort) and a missing
        motor_type/baudrate is found by pinging each candidate family from
        :data:`~orca_core.constants.MOTOR_BAUD_RATES`. Resolved values are
        persisted back to ``config.yaml``.

        Idempotent: calling ``connect()`` on an already-connected hand is a
        no-op that returns success. Call :meth:`disconnect` first to force a
        fresh connection.

        Args:
            interactive: When ``False``, skip the terminal port picker that
                otherwise runs as a last resort, so headless callers (servers,
                GUIs) get a clean failure instead of a blocking prompt.
            engage_feedback: Accepted for signature parity with the
                joint-feedback hands; this hand has no loop to engage, so the
                value is ignored.

        Returns:
            A ``(success, message)`` tuple where *success* is ``True`` on a
            successful connection.
        """
        if self.is_connected():
            return True, "Already connected"

        existing_config = self.config

        # ``port: auto`` keeps the tracked config hardware-agnostic; if no
        # unique adapter is found the normal recovery cascade below runs.
        first_port = self.config.port
        if first_port == "auto":
            detected = auto_detect_port(self.config.motor_type) or find_single_usb_serial_port()
            if detected is not None:
                first_port = detected

        error = self._try_port(first_port, existing_config)
        if error is None:
            return True, (
                f"Connection successful ({self.config.motor_type} @ "
                f"{self.config.port}, {self.config.baudrate} baud)"
            )
        logger.warning("Connection failed on %s: %s", first_port, error)

        chosen_port = auto_detect_port(self.config.motor_type)
        if chosen_port and chosen_port != first_port:
            if self._try_port(chosen_port, existing_config) is None:
                return (
                    True,
                    f"Connection successful with auto-detected port {chosen_port}",
                )

        if not interactive:
            return False, f"Connection failed on {first_port}: {str(error)}"
        print("Please select a port from available devices:")
        chosen_port = get_and_choose_port()
        if chosen_port is None:
            return False, "Connection failed: No port selected"

        error = self._try_port(chosen_port, existing_config)
        if error is None:
            return True, f"Connection successful with port {chosen_port}"
        return False, f"Connection failed with selected port: {str(error)}"

    def disconnect(self) -> tuple[bool, str]:
        """Disable torque (best-effort) and close the serial connection.

        The client is always closed and discarded, even when the torque
        disable fails, so :meth:`is_connected` reports ``False`` afterwards
        and a fresh :meth:`connect` starts from a clean state. Idempotent:
        calling it on an already-disconnected hand succeeds without touching
        the bus.

        Returns:
            A ``(success, message)`` tuple; *success* is ``False`` when the
            torque disable did not complete (the port is closed regardless).
        """
        if not self.is_connected():
            return True, "Disconnected successfully"
        failure = None
        with self._motor_lock:
            try:
                failed_ids = self.disable_torque()
                if failed_ids:
                    failure = (
                        f"torque disable was not acknowledged by motor IDs {failed_ids}"
                    )
                time.sleep(0.1)
            except Exception as e:
                failure = f"torque disable failed: {e}"
            finally:
                self._discard_motor_client()
        if failure is not None:
            return False, f"Disconnected, but {failure}"
        return True, "Disconnected successfully"

    def is_connected(self) -> bool:
        """Return ``True`` if the motor client is connected.

        Returns:
            Connection status as a boolean.
        """
        return self._motor_client is not None and self._motor_client.is_connected

    def enable_torque(self, motor_ids: List[int] = None) -> List[int]:
        """Enable torque on the specified motors.

        Args:
            motor_ids: List of motor IDs to enable. Defaults to all motors.

        Returns:
            The motor IDs that did not acknowledge the change after the
            client's retries; an empty list means every motor acked.
            Failures are also logged, so best-effort callers may ignore
            the return value.
        """
        motor_ids = self.config.motor_ids if motor_ids is None else motor_ids

        with self._motor_lock:
            failed_ids = list(self._motor_client.set_torque_enabled(motor_ids, True))
        if failed_ids:
            logger.warning(
                "Torque enable not acknowledged by motor IDs: %s", failed_ids
            )
        return failed_ids

    def disable_torque(self, motor_ids: List[int] = None) -> List[int]:
        """Disable torque on the specified motors.

        Args:
            motor_ids: List of motor IDs to disable. Defaults to all motors.

        Returns:
            The motor IDs that did not acknowledge the change after the
            client's retries; an empty list means every motor acked.
            Failures are also logged, so best-effort callers may ignore
            the return value.
        """
        motor_ids = self.config.motor_ids if motor_ids is None else motor_ids

        with self._motor_lock:
            failed_ids = list(self._motor_client.set_torque_enabled(motor_ids, False))
        if failed_ids:
            logger.warning(
                "Torque disable not acknowledged by motor IDs: %s", failed_ids
            )
        return failed_ids

    def set_max_current(self, current: Union[float, List[float]]):
        """Set the maximum allowable current for the motors.

        Args:
            current: Either a single float applied to all motors, or a list of
                per-motor current values (mA). If a list, its length must match
                the number of configured motors.

        Raises:
            ValueError: If *current* is a list with the wrong length.
        """
        if isinstance(current, list):
            if len(current) != len(self.config.motor_ids):
                raise ValueError(
                    "Number of currents do not match the number of motors."
                )

            with self._motor_lock:
                self._motor_client.write_desired_current(self.config.motor_ids, current)
            return

        with self._motor_lock:
            self._motor_client.write_desired_current(
                self.config.motor_ids, current * np.ones(len(self.config.motor_ids))
            )

    def set_control_mode(self, mode: str, motor_ids: List[int] = None):
        """Switch the operating mode of the specified motors.

        The wrist motor is always kept in ``multi_turn_position`` mode (4) when
        *mode* would otherwise be ``current_based_position`` (5) or
        ``current`` (0), because those modes are incompatible with the wrist
        joint's range of motion.

        Args:
            mode: One of ``"current"``, ``"velocity"``, ``"position"``,
                ``"multi_turn_position"``, or ``"current_based_position"``.
            motor_ids: Motors to reconfigure. Defaults to all motors.

        Raises:
            ValueError: If *mode* is not recognised or *motor_ids* contains
                unknown IDs.
        """
        mode_value = MODE_MAP.get(mode)
        if mode_value is None:
            raise ValueError("Invalid control mode.")

        # Holds the lock for the whole torque-off/mode-write/torque-on sequence
        # so it can't interleave with other bus traffic.
        with self._motor_lock:
            if motor_ids is None:
                motor_ids = self.config.motor_ids
            elif not all(motor_id in self.config.motor_ids for motor_id in motor_ids):
                raise ValueError("Invalid motor IDs.")

            if mode_value in (MODE_MAP[CURRENT_BASED_POSITION], MODE_MAP[CURRENT]):
                wrist_motor_id = self.config.joint_to_motor_map.get("wrist")
                if wrist_motor_id is not None:
                    motor_ids_without_wrist = [
                        motor_id for motor_id in motor_ids if motor_id != wrist_motor_id
                    ]
                    self._motor_client.set_operating_mode(
                        motor_ids_without_wrist, mode_value
                    )

                    if wrist_motor_id in motor_ids:
                        self._motor_client.set_operating_mode(
                            [wrist_motor_id], WRIST_MODE_VALUE
                        )

                    return

            self._motor_client.set_operating_mode(motor_ids, mode_value)

    def get_motor_pos(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Read raw motor positions from the bus.

        Args:
            as_dict: When ``True`` returns a ``dict`` keyed by motor ID.
                Defaults to ``False`` (returns an array ordered by
                :attr:`motor_ids`).

        Returns:
            Motor positions in radians as an array or dict.
        """
        with self._motor_lock:
            motor_pos = self._motor_client.read_position_velocity_current().position

            if as_dict:
                return {
                    motor_id: pos
                    for motor_id, pos in zip(self.config.motor_ids, motor_pos)
                }

            return motor_pos

    def get_motor_current(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Read the present current drawn by each motor.

        Args:
            as_dict: When ``True`` returns a ``dict`` keyed by motor ID.

        Returns:
            Motor currents (mA) as an array, or dict.
        """
        with self._motor_lock:
            motor_current = self._motor_client.read_position_velocity_current().current

            if as_dict:
                return {
                    motor_id: current
                    for motor_id, current in zip(self.config.motor_ids, motor_current)
                }

            return motor_current

    def wait_for_motion(self, timeout: float = 5.0) -> None:
        """Block until all motors have settled at their commanded position.

        No-op for motor types fast enough that callers don't need to wait
        (e.g., Dynamixel). Feetech polls a per-motor moving flag.

        Args:
            timeout: Max seconds to wait.

        Raises:
            MotionTimeoutError: If motors fail to settle within ``timeout``.
        """
        if not self._motor_client.waits_for_motion:
            return
        with self._motor_lock:
            self._motor_client.wait_for_motion_complete(timeout=timeout)

    def get_motor_temp(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Read the present temperature of each motor.

        Args:
            as_dict: When ``True`` returns a ``dict`` keyed by motor ID.

        Returns:
            Motor temperatures in °C as an array or dict.
        """
        with self._motor_lock:
            motor_temp = self._motor_client.read_temperature()

            if as_dict:
                return {
                    motor_id: temp
                    for motor_id, temp in zip(self.config.motor_ids, motor_temp)
                }

            return motor_temp

    def _get_joint_positions(self) -> OrcaJointPositions:
        motor_pos = self.get_motor_pos()
        return OrcaJointPositions.from_dict(self._motor_to_joint_pos(motor_pos))

    def _set_joint_positions(self, joint_pos: OrcaJointPositions) -> bool:
        motor_pos = self._joint_to_motor_pos(joint_pos.as_dict())
        self._set_motor_pos(motor_pos)
        return True

    def write_motor_pos(self, motor_ids: List[int], positions) -> None:
        """Write motor-position targets for an explicit subset of motors.

        Thin wrapper used by hot-path callers (e.g. the joint-loop thread)
        that already have an aligned ``(motor_ids, positions)`` pair and
        want to bypass the dict/list/array normalisation done by
        :meth:`_set_motor_pos`. Acquires the motor lock.
        """
        with self._motor_lock:
            self._motor_client.write_desired_pos(motor_ids, positions)

    def init_joints(self, force_calibrate: bool = False, move_to_neutral: bool = True):
        """Prepare the hand for operation.

        Enables torque, sets the configured control mode and current limit,
        runs calibration if needed, computes wrap offsets, and optionally
        moves to the neutral position.

        Args:
            force_calibrate: Force a fresh calibration even if the hand is
                already calibrated (default ``False``).
            move_to_neutral: Move to the configured neutral pose at the end
                of initialization (default ``True``). Set to ``False`` when
                the caller will immediately command a different pose.
        """
        self.enable_torque()
        self.set_control_mode(self.config.control_mode)
        self.set_max_current(self.config.max_current)

        if not self.calibrated or force_calibrate:
            self.calibrate()

        self._compute_wrap_offsets_dict()

        if move_to_neutral:
            control_mode = self.config.control_mode
            self.set_control_mode(POSITION)  # neutral position is given in POSITION mode
            self.set_joint_positions(
                OrcaJointPositions.from_dict(self.config.neutral_position),
                num_steps=NUM_STEPS
            )
            self.set_control_mode(control_mode)

    def is_calibrated(
        self, verbose: bool = False, use_joint_feedback: bool | None = None
    ) -> bool:
        """Check whether all joints have been fully calibrated.

        Args:
            verbose: When ``True``, prints a warning for each uncalibrated
                motor instead of returning early.
            use_joint_feedback: When ``True``, also require every encoder-backed
                joint to have a :class:`~orca_core.calibration.JointEncoderCal`
                entry. ``None`` defers to ``self.config.joint_feedback_enabled``.
        """
        if use_joint_feedback is None:
            use_joint_feedback = bool(
                getattr(self.config, "joint_feedback_enabled", False)
            )

        overall_calibrated = True
        uncalibrated_messages = []
        motors_with_warnings = set()

        for motor_id, limits in self.motor_limits_dict.items():
            if any(limit is None for limit in limits):
                overall_calibrated = False
                if not verbose:
                    return False
                joint_name = self.config.motor_to_joint_dict.get(motor_id, "Unknown")
                uncalibrated_messages.append(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing motor limits).\033[0m"
                )
                motors_with_warnings.add(motor_id)

        for motor_id, ratio in self.calibration.joint_to_motor_ratios_dict.items():
            if ratio is None or ratio == 0.0:
                overall_calibrated = False
                if not verbose:
                    return False
                if motor_id not in motors_with_warnings:
                    joint_name = self.config.motor_to_joint_dict.get(
                        motor_id, "Unknown"
                    )
                    uncalibrated_messages.append(
                        f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m"
                    )
                    motors_with_warnings.add(motor_id)

        if use_joint_feedback:
            encoder_dict = self.calibration.joint_encoder_calibration_dict
            for joint in self._encoder_backed_joints():
                if joint not in encoder_dict:
                    overall_calibrated = False
                    if not verbose:
                        return False
                    uncalibrated_messages.append(
                        f"\033[93mWarning: Joint {joint} is missing a joint-encoder calibration entry.\033[0m"
                    )

        if verbose:
            for msg in uncalibrated_messages:
                print(msg)

        return overall_calibrated

    @property
    def encoder_backed_joints(self) -> list[str]:
        """Names of the joints whose angle this hand reads from a joint encoder.

        A joint qualifies when it has an encoder slot in the wire protocol, a
        driving motor on this hand, and an entry in
        ``config.joint_encoder_joints`` (the ``["all"]`` sentinel selects
        every slotted, motor-driven joint; the wrist never qualifies). Empty
        when the config field is unset. Available before ``connect()``.
        """
        return self._encoder_backed_joints()

    def _encoder_backed_joints(self) -> list[str]:
        """Joints with a protocol slot, a driving motor on this hand, and an
        entry in ``config.joint_encoder_joints``. Returns ``[]`` when the
        config field is ``None``. The sentinel ``["all"]`` selects every
        slotted, motor-driven joint. Wrist is always excluded.
        """
        from .hardware.sensing.constants import (
            ENCODER_JOINTS_ALL,
            JOINT_TO_ENCODER_SLOT,
        )

        configured = self.config.joint_encoder_joints
        if configured is None:
            return []

        available = [
            joint
            for joint in JOINT_TO_ENCODER_SLOT
            if joint != WRIST and joint in self.config.joint_to_motor_map
        ]
        if any(str(j).lower() == ENCODER_JOINTS_ALL for j in configured):
            return available

        configured_set = set(configured)
        return [joint for joint in available if joint in configured_set]

    def _raw_to_joint_angle(self, raw_counts: np.ndarray) -> Dict[str, float]:
        """Convert raw encoder counts ``(AUTO_ENC_NUM_JOINTS,)`` into joint
        angles in degrees, keyed by joint name. Joints without a
        :class:`~orca_core.calibration.JointEncoderCal` entry are omitted.
        Raises ``ValueError`` when this hand's side has no validated encoder
        polarity table, so wrong-signed angles are never returned.
        """
        from .hardware.sensing.constants import (
            AUTO_ENC_NUM_JOINTS,
            JOINT_TO_ENCODER_SLOT,
            joint_encoder_polarity_for_side,
        )
        from .hardware.sensing.encoder_protocol import encoder_to_joint_angle

        raw_counts = np.asarray(raw_counts)
        if raw_counts.shape != (AUTO_ENC_NUM_JOINTS,):
            raise ValueError(
                f"raw_counts must have shape ({AUTO_ENC_NUM_JOINTS},), "
                f"got {raw_counts.shape}"
            )

        encoder_dict = self.calibration.joint_encoder_calibration_dict
        if not encoder_dict:
            return {}

        joints = [j for j in encoder_dict if j in JOINT_TO_ENCODER_SLOT]
        if not joints:
            return {}

        polarity_table = joint_encoder_polarity_for_side(self.config.type)
        slots = np.array([JOINT_TO_ENCODER_SLOT[j] for j in joints], dtype=np.int64)
        anchors = np.array([encoder_dict[j].enc_at_anchor_count for j in joints], dtype=np.int64)
        polarities = np.array([polarity_table[j] for j in joints], dtype=np.int64)
        anchor_angles = np.array(
            [self.config.joint_roms_dict[j][1] for j in joints], dtype=np.float64
        )

        slot_counts = raw_counts[slots]
        angles = encoder_to_joint_angle(slot_counts, anchors, polarities, anchor_angles)
        return {joint: float(angle) for joint, angle in zip(joints, angles)}

    def calibrate(
        self,
        blocking: bool = True,
        force_wrist: bool = False,
        joints: list[str] | None = None,
        joint_encoder_client=None,
        progress_callback=None,
        persist: bool | None = None,
    ):
        """Run the joint calibration routine.

        Drives each joint to its mechanical limits per ``calibration_sequence``
        and persists motor limits + joint-to-motor ratios to
        ``calibration.yaml`` after every step.

        Args:
            blocking: When ``True``, run to completion before returning.
            force_wrist: Recalibrate the wrist even if already calibrated.
            joints: Restrict to calibration steps touching these joint names.
                Joints not visited keep their previously-persisted values.
            joint_encoder_client: With ``self.config.joint_feedback_enabled``
                and an encoder client, the encoder pass also runs and writes a
                ``joint_encoder_calibration:`` block.
            progress_callback: Optional ``callable(dict)`` invoked with
                structured progress events: ``calibration_started``,
                ``step_started``, ``limit_recorded``, ``joint_calibrated``,
                ``encoder_anchor_recorded``, ``encoder_anchor_failed``,
                ``offset_calibration_failed``, ``wrist_skipped``,
                ``step_done``, ``calibration_done``, ``calibration_aborted``,
                and ``cleanup_failed``. Called from the calibrating thread;
                must be fast and non-blocking. Exceptions raised by the
                callback are swallowed.
            persist: Whether results are written to ``calibration.yaml``
                (in-memory ``self.calibration`` updates either way). ``None``
                (default) defers to the class: real hands persist, ``Mock*``
                hands don't. Pass ``True`` on a mock to deliberately write a
                synthetic calibration file.
        """
        if persist is None:
            persist = self._persist_calibration
        if blocking:
            self._task_stop_event.clear()
            self._calibrate_and_apply(
                force_wrist=force_wrist,
                joints=joints,
                joint_encoder_client=joint_encoder_client,
                progress_callback=progress_callback,
                persist=persist,
            )
        else:
            self._start_task(
                self._calibrate_and_apply,
                force_wrist=force_wrist,
                joints=joints,
                joint_encoder_client=joint_encoder_client,
                progress_callback=progress_callback,
                persist=persist,
            )

    def _calibrate_and_apply(self, **kwargs):
        """Run the calibration routine and apply a completed result."""
        result = run_calibration(
            self, should_stop=self._task_stop_event.is_set, **kwargs
        )
        if result is not None:
            self.calibration = result

    def set_neutral_position(self, num_steps: int = NUM_STEPS, step_size: float = STEP_SIZE):
        control_mode = self.config.control_mode
        self.set_control_mode(POSITION)
        super().set_neutral_position(num_steps, step_size)
        self.set_control_mode(control_mode)
    
    def _read_motor_pos_for_offsets(self, retries: int = 5, retry_interval: float = 0.05):
        """Read motor positions for wrap-offset detection, rejecting a read the
        bus never actually answered.

        A dropped status packet leaves ``read_position_velocity_current`` returning its
        zero-initialised cache — every motor reads ``0.0``, below its lower
        limit — and the caller would bake a spurious ``-2π`` wrap offset into
        all 17 motors, corrupting the joint→motor mapping for the whole
        session. Retry while the reader reports the read failed; raise loudly
        rather than proceed on stale cache.
        """
        for _ in range(retries):
            with self._motor_lock:
                motor_pos = self.get_motor_pos()
                read_ok = self._motor_client.last_read_ok
            if read_ok:
                return motor_pos
            time.sleep(retry_interval)
        raise RuntimeError(
            "motor position read failed while computing wrap offsets: the motor "
            "bus returned no status packets. Retry; if it persists, power-cycle "
            "the board."
        )

    def _compute_wrap_offsets_dict(self):
        """Detect per-motor encoder wrap-arounds and store correction offsets.

        Reads current motor positions and checks whether any motor has drifted
        more than 1/4pi beyond its configured limits, which indicates the
        rotary encoder has wrapped around a full revolution. For each such motor
        a ±2pi offset is recorded in `_wrap_offsets_dict` so callers can shift
        the raw reading back into the expected operating range. Motors with no
        configured limits, or whose positions are within tolerance, receive an
        offset of 0.
        """
        motor_pos = self._read_motor_pos_for_offsets()

        lower_limit = np.array(
            [self.motor_limits_dict[motor_id][0] for motor_id in self.config.motor_ids]
        )
        higher_limit = np.array(
            [self.motor_limits_dict[motor_id][1] for motor_id in self.config.motor_ids]
        )

        offsets = {}
        for idx, motor_id in enumerate(self.config.motor_ids):
            if lower_limit[idx] is None or higher_limit[idx] is None:
                offsets[motor_id] = 0.0
                continue

            if motor_pos[idx] < lower_limit[idx] - 0.25 * np.pi:
                print(
                    f"Motor ID {motor_id} is out of bounds: {lower_limit[idx]} < {motor_pos[idx]} < {higher_limit[idx]}"
                )
                offsets[motor_id] = -2 * np.pi
            elif motor_pos[idx] > higher_limit[idx] + 0.25 * np.pi:
                print(
                    f"Motor ID {motor_id} is out of bounds: {lower_limit[idx]} < {motor_pos[idx]} < {higher_limit[idx]}"
                )
                offsets[motor_id] = +2 * np.pi
            else:
                offsets[motor_id] = 0.0

        print(f"Offsets: {offsets}")
        self._wrap_offsets_dict = offsets

    def _clear_wrap_offset(self, motor_id: int) -> None:
        """Zero one motor's wrap offset so its raw position is read unshifted."""
        self._wrap_offsets_dict[motor_id] = 0.0

    def _set_motor_pos(
        self, desired_pos: Union[dict, np.ndarray, list], rel_to_current: bool = False
    ):
        with self._motor_lock:
            if (
                rel_to_current
            ):  # TODO(fracapuano): split in two methods for delta-set or absolute-set
                current_positions = self.get_motor_pos()
                # A stale cache read here would command a violent jump toward
                # its (possibly zero) values; refuse a relative move on it.
                if not self._motor_client.last_read_ok:
                    print(
                        "\033[93mWarning: motor position read failed; skipping "
                        "relative position command (stale base).\033[0m"
                    )
                    return

            motor_ids_to_write = []
            positions_to_write = []

            if isinstance(desired_pos, dict):
                for motor_id, pos_val in desired_pos.items():
                    if motor_id not in self.config.motor_ids:
                        print(
                            f"Warning: Motor ID {motor_id} in desired_pos dict is not in self.config.motor_ids. Skipping."
                        )
                        continue
                    if pos_val is None or math.isnan(pos_val):
                        continue

                    pos_to_write = float(pos_val)
                    if rel_to_current:
                        pos_to_write += current_positions[
                            self.config.motor_id_to_idx_dict[motor_id]
                        ]

                    motor_ids_to_write.append(motor_id)
                    positions_to_write.append(pos_to_write)

                if not motor_ids_to_write:
                    return
                positions_to_write = np.array(positions_to_write, dtype=float)

            elif isinstance(desired_pos, (np.ndarray, list)):
                if len(desired_pos) != len(self.config.motor_ids):
                    raise ValueError(
                        f"Length of desired_pos (list/ndarray) ({len(desired_pos)}) must match the number of configured motor_ids ({len(self.config.motor_ids)})."
                    )

                for idx, pos_val in enumerate(desired_pos):
                    if pos_val is None or math.isnan(pos_val):
                        continue

                    motor_ids_to_write.append(self.config.motor_ids[idx])
                    if rel_to_current:
                        positions_to_write.append(
                            float(pos_val) + current_positions[idx]
                        )
                    else:
                        positions_to_write.append(float(pos_val))

                if not motor_ids_to_write:
                    print(
                        "\033[93mWarning: All positions in desired_pos (list/array) were None. No motor commands sent.\033[0m"
                    )
                    return

                positions_to_write = np.array(positions_to_write, dtype=float)

            else:
                raise ValueError("desired_pos must be a dict, np.ndarray, or list.")

            self._motor_client.write_desired_pos(motor_ids_to_write, positions_to_write)

    def _warn_uncalibrated(self, motor_id: int, joint_name: str, missing: str) -> None:
        """Warn once per motor about missing calibration data (reads can run at loop rate)."""
        if motor_id in self._uncalibrated_warned:
            return
        self._uncalibrated_warned.add(motor_id)
        logger.warning(
            "Motor ID %s (Joint: %s) has not been fully calibrated (missing %s).",
            motor_id, joint_name, missing,
        )

    def _motor_to_joint_pos(self, motor_pos: np.ndarray) -> dict:
        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()

        joint_pos = {}
        for idx, pos in enumerate(motor_pos):
            motor_id = self.config.motor_ids[idx]
            joint_name = self.config.motor_to_joint_dict.get(motor_id)
            if any(limit is None for limit in self.motor_limits_dict[motor_id]):
                joint_pos[joint_name] = None
                self._warn_uncalibrated(motor_id, joint_name, "motor limits")
            elif self.calibration.joint_to_motor_ratios_dict[motor_id] == 0:
                joint_pos[joint_name] = None
                self._warn_uncalibrated(motor_id, joint_name, "joint-to-motor ratio")
            else:
                wrapped_pos = pos - self._wrap_offsets_dict.get(motor_id, 0.0)
                if self.config.joint_inversion_dict.get(joint_name, False):
                    joint_pos[joint_name] = (
                        self.config.joint_roms_dict[joint_name][1]
                        - (wrapped_pos - self.motor_limits_dict[motor_id][0])
                        / self.calibration.joint_to_motor_ratios_dict[motor_id]
                    )
                else:
                    joint_pos[joint_name] = (
                        self.config.joint_roms_dict[joint_name][0]
                        + (wrapped_pos - self.motor_limits_dict[motor_id][0])
                        / self.calibration.joint_to_motor_ratios_dict[motor_id]
                    )
        return joint_pos

    def _joint_to_motor_pos(self, joint_pos: dict) -> np.ndarray:
        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()

        motor_pos = [None] * len(self.config.motor_ids)

        for joint_name, pos in joint_pos.items():
            motor_id = self.config.joint_to_motor_map.get(joint_name)
            if motor_id is None:
                continue

            if pos is None:
                motor_pos[self.config.motor_id_to_idx_dict[motor_id]] = None
                continue

            if (
                self.motor_limits_dict[motor_id][0] is None
                or self.motor_limits_dict[motor_id][1] is None
                or self.calibration.joint_to_motor_ratios_dict[motor_id] == 0
            ):
                motor_pos[self.config.motor_id_to_idx_dict[motor_id]] = None
                self._warn_uncalibrated(motor_id, joint_name, "joint-to-motor ratio")
                continue

            if self.config.joint_inversion_dict.get(joint_name, False):
                motor_pos[self.config.motor_id_to_idx_dict[motor_id]] = (
                    self.motor_limits_dict[motor_id][0]
                    + (self.config.joint_roms_dict[joint_name][1] - pos)
                    * self.calibration.joint_to_motor_ratios_dict[motor_id]
                )
            else:
                motor_pos[self.config.motor_id_to_idx_dict[motor_id]] = (
                    self.motor_limits_dict[motor_id][0]
                    + (pos - self.config.joint_roms_dict[joint_name][0])
                    * self.calibration.joint_to_motor_ratios_dict[motor_id]
                )

            motor_pos[self.config.motor_id_to_idx_dict[motor_id]] += (
                self._wrap_offsets_dict.get(motor_id, 0.0)
            )

        return motor_pos

    def _sanity_check(self):
        """Demote the calibrated flag when any motor limit is missing.

        The on-disk flag is corrected only on classes that persist
        calibration, and only when ``calibration.yaml`` exists and still
        claims ``calibrated: true``; constructing a hand never creates or
        rewrites the file otherwise, and mock hands never write at all.
        """
        if not any(
            any(limit is None for limit in limits)
            for limits in self.motor_limits_dict.values()
        ):
            return
        self.calibration = dataclasses.replace(self.calibration, calibrated=False)
        if not self._persist_calibration:
            return
        calibration_path = self.config.calibration_path
        if os.path.exists(calibration_path) and (
            (read_yaml(calibration_path) or {}).get(CALIBRATED)
        ):
            update_yaml(calibration_path, CALIBRATED, False)

    def tension(
        self,
        move_motors: bool = True,
        blocking: bool = True,
        progress_callback=None,
    ):
        """Hold motors under current to allow manual tendon tensioning.

        Optionally pre-conditions the tendons with a short back-and-forth
        motion before entering the hold phase. Torque is disabled automatically
        on exit.

        Args:
            move_motors: When ``True``, execute a short flexion/extension cycle
                before holding (default ``True``).
            blocking: When ``True`` (default) blocks until the user interrupts
                with Ctrl-C. When ``False`` runs in a background thread.
            progress_callback: Optional ``callable(dict)`` invoked with
                structured progress events: ``phase`` (with ``phase`` one of
                winding/ramp/holding/released), ``winding_progress``, and
                ``cleanup_failed``. Called from the tensioning thread; must
                be fast and non-blocking. Exceptions raised by the callback
                are swallowed.
        """
        if blocking:
            self._task_stop_event.clear()
            self._tension(move_motors, progress_callback=progress_callback)
        else:
            self._start_task(
                self._tension, move_motors, progress_callback=progress_callback
            )

    def jitter(
        self,
        motor_ids: List[int] = None,
        amplitude: float = 5.0,
        frequency: float = 10.0,
        duration: float = 3.0,
        include_wrist: bool = False,
        blocking: bool = True,
    ):
        """Apply a sinusoidal jitter to the motors for tendon seating.

        All motors oscillate around their current position with a sine wave.
        Amplitude is capped at 10° for safety.

        Args:
            motor_ids: Motors to jitter. Defaults to all non-wrist motors (or
                all motors when *include_wrist* is ``True``).
            amplitude: Peak amplitude in degrees; motors swing ±amplitude
                around their start position (default ``5.0``, max ``10.0``).
            frequency: Oscillation frequency in Hz (default ``10.0``).
            duration: Total jitter duration in seconds (default ``3.0``).
            include_wrist: Include the wrist motor when *motor_ids* is
                ``None`` (default ``False``).
            blocking: When ``True`` (default) blocks until jitter completes.
                When ``False`` runs in a background thread.

        Raises:
            ValueError: If *amplitude* exceeds 10°.
        """
        if blocking:
            self._task_stop_event.clear()
            self._jitter(motor_ids, amplitude, frequency, duration, include_wrist)
        else:
            self._start_task(
                self._jitter, motor_ids, amplitude, frequency, duration, include_wrist
            )

    def _jitter(
        self,
        motor_ids: List[int] = None,
        amplitude: float = 5.0,
        frequency: float = 10.0,
        duration: float = 3.0,
        include_wrist: bool = False,
    ):
        run_jitter(
            self,
            motor_ids=motor_ids,
            amplitude=amplitude,
            frequency=frequency,
            duration=duration,
            include_wrist=include_wrist,
            should_stop=self._task_stop_event.is_set,
        )

    def _tension(self, move_motors: bool = True, progress_callback=None):
        run_tension(
            self,
            move_motors=move_motors,
            progress_callback=progress_callback,
            should_stop=self._task_stop_event.is_set,
        )

    def _run_task(self, task_fn, *args, **kwargs):
        with self._lock:
            self._task_stop_event.clear()
            self._current_task = task_fn.__name__
            try:
                task_fn(*args, **kwargs)
            finally:
                self._current_task = None

    def _start_task(self, task_fn, *args, **kwargs):
        if self._task_thread and self._task_thread.is_alive():
            print(f"Task '{self._current_task}' is already running.")
            return

        self._task_thread = threading.Thread(
            target=self._run_task, args=(task_fn,) + args, kwargs=kwargs
        )
        self._task_thread.start()

    @property
    def task_running(self) -> bool:
        """Whether a background task (calibration, tensioning, jitter) is running."""
        return self._task_thread is not None and self._task_thread.is_alive()

    def stop_task(self, timeout: float | None = None) -> bool:
        """Ask a running background task to stop and wait for it to wind down.

        Args:
            timeout: Seconds to wait for the task to finish. ``None`` waits
                indefinitely; pass a bound when the caller must stay
                responsive (e.g. serving a request).

        Returns:
            ``True`` once no task is running, ``False`` if one is still
            running when *timeout* elapsed.
        """
        if not self.task_running:
            return True
        self._task_stop_event.set()
        self._task_thread.join(timeout)
        stopped = not self._task_thread.is_alive()
        if stopped:
            logger.info("task stopped")
        else:
            logger.warning("task did not stop within %ss", timeout)
        return stopped


class MockMotorResolutionMixin:
    """Swaps the motor bus for an in-memory mock on ``Mock*`` hand classes.

    Supplies the mock motor client and skips connect-time port/driver
    resolution and yaml persistence: mock motors don't sit on a real bus, so
    there is nothing to detect or probe (``port: auto`` must not handshake
    real USB devices) and no auto-detected values worth writing back to
    config.yaml.

    It also synthesises the motor calibration a mock can't measure, so the
    bundled models are usable out of the box (see
    :meth:`_install_mock_calibration`). Mock-derived calibration never
    reaches disk: ``_persist_calibration`` defaults ``calibrate()`` to
    in-memory-only so synthesized values can't overwrite a real hand's
    ``calibration.yaml`` (pass ``persist=True`` to opt in deliberately).
    """

    _persist_calibration = False

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._install_mock_calibration()

    def _install_mock_calibration(self) -> None:
        """Fill in motor calibration that a mock has no way to measure.

        ``calibration.yaml`` is a per-hand artifact that a fresh checkout or a
        wheel install doesn't carry, and mock motors have no hardstops to
        sweep. Limits and ratios are derived from the config ROMs so
        joint-motor conversion round-trips exactly. Entries that are already
        calibrated are left untouched. When every motor ends up with limits
        and a ratio, the in-memory ``calibrated`` flag is set so
        ``init_joints()`` doesn't launch a calibration run against simulated
        hardstops; the on-disk file is never modified.
        """
        motor_limits = dict(self.calibration.motor_limits_dict)
        ratios = dict(self.calibration.joint_to_motor_ratios_dict)
        roms = self.config.joint_roms_dict
        changed = False

        for joint, motor_id in self.config.joint_to_motor_map.items():
            rom = roms.get(joint)
            if rom is None:
                continue
            limits = motor_limits.get(motor_id)
            if not limits or any(limit is None for limit in limits):
                # Lower limit of 0 matches the mock motors' rest position, so
                # wrap-offset detection doesn't read them as below-limit.
                span = abs(float(rom[1]) - float(rom[0])) * MOCK_JOINT_TO_MOTOR_RATIO
                motor_limits[motor_id] = [0.0, span]
                changed = True
            if not ratios.get(motor_id):
                ratios[motor_id] = MOCK_JOINT_TO_MOTOR_RATIO
                changed = True

        calibrated = all(
            limits[0] is not None and limits[1] is not None
            for limits in motor_limits.values()
        ) and all(
            ratio is not None and ratio != 0.0 for ratio in ratios.values()
        )

        if changed or calibrated != self.calibration.calibrated:
            self.calibration = dataclasses.replace(
                self.calibration,
                motor_limits_dict=motor_limits,
                joint_to_motor_ratios_dict=ratios,
                calibrated=calibrated,
            )

    def connect(self, interactive: bool = True, **kwargs) -> tuple[bool, str]:
        if self.config.port == "auto":
            self.config = dataclasses.replace(self.config, port="mock")
        return super().connect(interactive, **kwargs)

    def _create_motor_client(self) -> MotorClient:
        from .hardware.mock_dynamixel_client import MockDynamixelClient

        return MockDynamixelClient(
            self.config.motor_ids, self.config.port, self.config.baudrate
        )

    def _resolve_motor_driver(self, port: str) -> bool:
        if self.config.motor_type is None or self.config.baudrate is None:
            self.config = dataclasses.replace(
                self.config,
                motor_type=self.config.motor_type or "dynamixel",
                baudrate=self.config.baudrate or 1_000_000,
            )
        return True

    def _persist_resolved_driver(self, existing) -> None:
        pass


class MockOrcaHand(MockMotorResolutionMixin, OrcaHand):
    """Drop-in :class:`OrcaHand` backed by an in-memory mock motor client,
    for testing and prototyping.

    All methods behave identically to :class:`OrcaHand` but no serial
    port is opened and motor state is simulated in memory.
    """
