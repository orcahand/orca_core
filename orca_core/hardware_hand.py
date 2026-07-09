# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import dataclasses
import math
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
    update_yaml,
)

from .constants import (
    MODE_MAP,
    WRIST_MODE_VALUE,
    CURRENT_BASED_POSITION,
    CURRENT,
    WRIST,
    STEPS_TO_NEUTRAL,
    POSITION,
    STEP_SIZE_NEUTRAL,
)

from .joint_position import OrcaJointPositions


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
        self.disconnect()

    # ------------------------------------------------------------------
    # Calibration state — thin views onto self.calibration
    # ------------------------------------------------------------------

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
        """The connected motor client, or ``None`` before ``connect()``.

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
        """Fill in ``motor_type``/``baudrate`` for ``port`` when not pinned in yaml.

        Returns False when a probe was needed but no motor family responded.
        """
        if self.config.motor_type is not None and self.config.baudrate is not None:
            return True
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

    def connect(self, interactive: bool = True) -> tuple[bool, str]:
        """Open connection to the motor bus.

        Resolves (port, motor_type, baudrate) at connect time: the values in
        ``config.yaml`` win when present; a missing port is auto-detected via
        USB vendor ID (interactive picker as a last resort) and a missing
        motor_type/baudrate is found by pinging each candidate family from
        :data:`~orca_core.constants.MOTOR_BAUD_RATES`. Resolved values are
        persisted back to ``config.yaml``.

        Args:
            interactive: When ``False``, skip the terminal port picker that
                otherwise runs as a last resort, so headless callers (servers,
                GUIs) get a clean failure instead of a blocking prompt.

        Returns:
            A ``(success, message)`` tuple where *success* is ``True`` on a
            successful connection.
        """
        existing_config = self.config

        # ``port: auto`` keeps the tracked config hardware-agnostic; if no
        # unique adapter is found the normal recovery cascade below runs.
        first_port = self.config.port
        if first_port == "auto":
            detected = auto_detect_port(self.config.motor_type) or find_single_usb_serial_port()
            if detected is not None:
                first_port = detected

        try:
            self._connect_on_port(first_port, existing_config)
            self._persist_resolved_driver(existing_config)
            return True, (
                f"Connection successful ({self.config.motor_type} @ "
                f"{self.config.port}, {self.config.baudrate} baud)"
            )

        except Exception as e:
            self._motor_client = None
            print(f"Connection failed on {first_port}: {str(e)}")

            chosen_port = auto_detect_port(self.config.motor_type)
            if chosen_port and chosen_port != first_port:
                try:
                    self._connect_on_port(chosen_port, existing_config)
                    self._persist_resolved_driver(existing_config)
                    return (
                        True,
                        f"Connection successful with auto-detected port {chosen_port}",
                    )

                except Exception:
                    self._motor_client = None

            if not interactive:
                return False, "Connection failed: No port selected"
            print("Please select a port from available devices:")
            chosen_port = get_and_choose_port()
            if chosen_port is None:
                return False, "Connection failed: No port selected"

            try:
                self._connect_on_port(chosen_port, existing_config)
                self._persist_resolved_driver(existing_config)
                return True, f"Connection successful with port {chosen_port}"
            except Exception as e2:
                self._motor_client = None
                return False, f"Connection failed with selected port: {str(e2)}"

    def disconnect(self) -> tuple[bool, str]:
        """Disable torque and close the serial connection.

        Safe to call even when the hand is already disconnected.

        Returns:
            A ``(success, message)`` tuple.
        """
        try:
            if self._motor_client is None:
                return True, "Disconnected successfully"
            with self._motor_lock:
                self.disable_torque()
                time.sleep(0.1)
                self._motor_client.disconnect()
            return True, "Disconnected successfully"
        except Exception as e:
            return False, f"Disconnection failed: {str(e)}"

    def is_connected(self) -> bool:
        """Return ``True`` if the motor client is connected.

        Returns:
            Connection status as a boolean.
        """
        return self._motor_client is not None and self._motor_client.is_connected

    def enable_torque(self, motor_ids: List[int] = None):
        """Enable torque on the specified motors.

        Args:
            motor_ids: List of motor IDs to enable. Defaults to all motors.
        """
        motor_ids = self.config.motor_ids if motor_ids is None else motor_ids

        with self._motor_lock:
            self._motor_client.set_torque_enabled(motor_ids, True)

    def disable_torque(self, motor_ids: List[int] = None):
        """Disable torque on the specified motors.

        Args:
            motor_ids: List of motor IDs to disable. Defaults to all motors.
        """
        motor_ids = self.config.motor_ids if motor_ids is None else motor_ids

        with self._motor_lock:
            self._motor_client.set_torque_enabled(motor_ids, False)

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

    def init_joints(self, force_calibrate: bool = False):
        """Prepare the hand for operation.

        Enables torque, sets the configured control mode and current limit,
        runs calibration if needed, computes wrap offsets, and moves to the
        neutral position.

        Args:
            calibrate: Force a fresh calibration even if the hand is already
                calibrated (default ``False``).
        """
        self.enable_torque()
        self.set_control_mode(self.config.control_mode)
        self.set_max_current(self.config.max_current)

        if not self.calibrated or force_calibrate:
            self.calibrate()

        self._compute_wrap_offsets_dict()
        control_mode = self.config.control_mode
        self.set_control_mode(POSITION)  # neutral position is given in POSITION mode
        self.set_joint_positions(
            OrcaJointPositions.from_dict(self.config.neutral_position),
            num_steps=STEPS_TO_NEUTRAL
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
        """
        from .hardware.sensing.constants import (
            AUTO_ENC_NUM_JOINTS,
            JOINT_ENCODER_POLARITY,
            JOINT_TO_ENCODER_SLOT,
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

        slots = np.array([JOINT_TO_ENCODER_SLOT[j] for j in joints], dtype=np.int64)
        anchors = np.array([encoder_dict[j].enc_at_anchor_count for j in joints], dtype=np.int64)
        polarities = np.array([JOINT_ENCODER_POLARITY[j] for j in joints], dtype=np.int64)
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
                structured progress events (``calibration_started``,
                ``step_started``, ``joint_calibrated``, ``step_done``,
                ``calibration_done``, ``calibration_aborted``). Called from
                the calibrating thread; must be fast and non-blocking.
                Exceptions raised by the callback are swallowed.
        """
        if blocking:
            self._task_stop_event.clear()
            self._calibrate_and_apply(
                force_wrist=force_wrist,
                joints=joints,
                joint_encoder_client=joint_encoder_client,
                progress_callback=progress_callback,
            )
        else:
            self._start_task(
                self._calibrate_and_apply,
                force_wrist=force_wrist,
                joints=joints,
                joint_encoder_client=joint_encoder_client,
                progress_callback=progress_callback,
            )

    def _calibrate_and_apply(self, **kwargs):
        """Run the calibration routine and apply a completed result."""
        result = run_calibration(
            self, should_stop=self._task_stop_event.is_set, **kwargs
        )
        if result is not None:
            self.calibration = result

    def set_neutral_position(self, num_steps: int = STEPS_TO_NEUTRAL, step_size: float = STEP_SIZE_NEUTRAL):
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
            motor_pos = self.get_motor_pos()
            if self._motor_client.last_read_ok:
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

    def _motor_to_joint_pos(self, motor_pos: np.ndarray) -> dict:
        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()

        joint_pos = {}
        for idx, pos in enumerate(motor_pos):
            motor_id = self.config.motor_ids[idx]
            joint_name = self.config.motor_to_joint_dict.get(motor_id)
            if any(limit is None for limit in self.motor_limits_dict[motor_id]):
                joint_pos[joint_name] = None
                print(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing motor limits).\033[0m"
                )
            elif self.calibration.joint_to_motor_ratios_dict[motor_id] == 0:
                joint_pos[joint_name] = None
                print(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m"
                )
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
                print(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m"
                )
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
        for motor_limit in self.motor_limits_dict.values():
            if any(limit is None for limit in motor_limit):
                self.calibration = dataclasses.replace(
                    self.calibration, calibrated=False
                )
                update_yaml(self.config.calibration_path, "calibrated", False)

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
                structured progress events (``phase`` with
                winding/ramp/holding/released, ``winding_progress``). Called
                from the tensioning thread; must be fast and non-blocking.
                Exceptions raised by the callback are swallowed.
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
            amplitude: Peak-to-peak amplitude in degrees (default ``5.0``,
                max ``10.0``).
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

    def stop_task(self):
        """Stops a background task like calibration, tensioning or jittering."""
        if self._task_thread and self._task_thread.is_alive():
            self._task_stop_event.set()
            self._task_thread.join()
            print("Task stopped.")
        else:
            print("No running task to stop.")


class MockMotorResolutionMixin:
    """Swaps the motor bus for an in-memory mock on ``Mock*`` hand classes.

    Supplies the mock motor client and skips connect-time port/driver
    resolution and yaml persistence: mock motors don't sit on a real bus, so
    there is nothing to detect or probe (``port: auto`` must not handshake
    real USB devices) and no auto-detected values worth writing back to
    config.yaml.
    """

    def connect(self, interactive: bool = True) -> tuple[bool, str]:
        if self.config.port == "auto":
            self.config = dataclasses.replace(self.config, port="mock")
        return super().connect(interactive)

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


_MOVED_TO_SENSING = ("OrcaHandTouch", "MockOrcaHandTouch")


def __getattr__(name):
    if name in _MOVED_TO_SENSING:
        import warnings

        warnings.warn(
            f"importing {name} from orca_core.hardware_hand is deprecated; "
            "import it from orca_core (package root) or "
            "orca_core.hardware_hand_sensing instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        from . import hardware_hand_sensing

        return getattr(hardware_hand_sensing, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
