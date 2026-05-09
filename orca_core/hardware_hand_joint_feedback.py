# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Closed-loop joint-feedback variant of :class:`OrcaHand`.

Adds a host-side current-mode PID loop on the encoder-backed joints. The
wrist motor is not part of the loop; it stays in
``current_based_position`` and accepts targets through the inherited
synchronous motor-position path.
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional

from .constants import CURRENT, MODE_MAP, WRIST
from .control.constants import (
    DEFAULT_KD_MA_S_PER_RAD,
    DEFAULT_KI_MA_PER_RAD_S,
    DEFAULT_KP_MA_PER_RAD,
)
from .control.joint_loop import JointLoopThread
from .control.joint_pid import JointPIDController
from .hardware.hand_serial_link import HandSerialLink
from .hardware.joint_encoder_client import (
    EncodersNotAvailableError,
    JointEncoderClient,
)
from .hardware.motor_client import MotorClient
from .hardware.sensing.constants import LINK_DEFAULT_BAUDRATE
from .hardware.sensing.serial_discovery import resolve_sensing_ports
from .hardware_hand import OrcaHand


logger = logging.getLogger(__name__)


class OrcaHandJointFeedbackError(RuntimeError):
    """Raised when a joint-feedback connect precondition fails (no encoder
    port resolved, no encoder-backed joints, missing encoder calibration).
    """


class OrcaHandJointFeedback(OrcaHand):
    """ORCA hand with closed-loop joint feedback on the encoder-backed joints.

    ``connect()`` opens the motor bus, the encoder serial link, and starts a
    background ``JointLoopThread`` running a vectorised current-mode PID. The
    wrist stays in ``current_based_position`` and is driven through the
    inherited synchronous path.

    Connect-time preconditions raise: a missing encoder port, an absent
    ``joint_encoder_calibration`` block, or an encoder-stream timeout each
    surface as an exception. The motor bus is left open on failure; the
    caller invokes ``disconnect()`` to fully roll back.
    """

    def __init__(
        self,
        config_path: str | None = None,
        calibration_path: str | None = None,
        model_version: str | None = None,
        model_name: str | None = None,
        config=None,
    ):
        super().__init__(
            config_path=config_path,
            calibration_path=calibration_path,
            model_version=model_version,
            model_name=model_name,
            config=config,
        )
        self._encoder_link: Optional[HandSerialLink] = None
        self._encoder_client: Optional[JointEncoderClient] = None
        self._pid: Optional[JointPIDController] = None
        self._loop: Optional[JointLoopThread] = None
        self._prior_modes_snapshot: Dict[int, int] = {}

    def _create_encoder_link(self, port: str) -> HandSerialLink:
        return HandSerialLink(port=port, baudrate=LINK_DEFAULT_BAUDRATE)

    def _create_encoder_client(self, link: HandSerialLink) -> JointEncoderClient:
        return JointEncoderClient(link)

    def _encoder_motor_ids(self) -> List[int]:
        joint_to_motor = self.config.joint_to_motor_map
        return [joint_to_motor[j] for j in self._encoder_backed_joints()]

    def connect(self) -> tuple[bool, str]:
        success, msg = super().connect()
        if not success:
            return success, msg

        try:
            ports = resolve_sensing_ports(
                encoder_override=self.config.encoder_serial_port,
            )
            if ports.encoder is None:
                raise OrcaHandJointFeedbackError(
                    "No encoder serial port resolved "
                    f"(encoder_serial_port={self.config.encoder_serial_port!r})."
                )

            self._encoder_link = self._create_encoder_link(ports.encoder)
            self._encoder_link.connect()
            self._encoder_client = self._create_encoder_client(self._encoder_link)
            self._encoder_client.connect()
            self._encoder_client.start_encoder_stream()

            if not self.is_calibrated(use_joint_feedback=True):
                raise OrcaHandJointFeedbackError(
                    "Hand is missing joint-encoder calibration; "
                    "run calibration with use_joint_feedback enabled."
                )

            motor_ids = self._encoder_motor_ids()
            if not motor_ids:
                raise OrcaHandJointFeedbackError(
                    "No encoder-backed joints configured "
                    "(set joint_encoder_joints in config.yaml)."
                )

            with self._motor_lock:
                self._prior_modes_snapshot = self._motor_client.read_operating_modes(
                    motor_ids
                )
                self._motor_client.set_operating_mode_per_motor(
                    motor_ids, [MODE_MAP[CURRENT]] * len(motor_ids),
                )

            self._pid = JointPIDController(num_joints=len(motor_ids))
            i_max = float(self.config.max_current)
            self._pid.set_gains(
                Kp=DEFAULT_KP_MA_PER_RAD,
                Ki=DEFAULT_KI_MA_PER_RAD_S,
                Kd=DEFAULT_KD_MA_S_PER_RAD,
                i_clamp_mA=i_max,
                i_max_mA=i_max,
            )
            self._loop = JointLoopThread(self, self._encoder_client, self._pid)
            self._loop.start()
        except Exception:
            self._teardown_joint_feedback()
            raise

        return True, f"{msg} | Joint feedback loop running on {ports.encoder}"

    def disconnect(self) -> tuple[bool, str]:
        self._teardown_joint_feedback()
        return super().disconnect()

    def _set_joint_positions(self, joint_pos) -> bool:
        if self._loop is None:
            return super()._set_joint_positions(joint_pos)

        encoder_joints = set(self._encoder_backed_joints())
        loop_targets: Dict[str, float] = {}
        rest: Dict[str, float] = {}
        for joint, value in joint_pos.as_dict().items():
            if joint in encoder_joints:
                loop_targets[joint] = float(value)
            else:
                rest[joint] = float(value)

        if loop_targets:
            self._loop.set_target(loop_targets)
        if rest:
            from .joint_position import OrcaJointPositions
            super()._set_joint_positions(OrcaJointPositions(rest))
        return True

    def _get_joint_positions(self):
        from .joint_position import OrcaJointPositions

        if self._loop is None:
            return super()._get_joint_positions()

        motor_pos = self.get_motor_pos()
        joint_dict = self._motor_to_joint_pos(motor_pos)
        joint_dict.update(self._loop.get_measured_joints())
        return OrcaJointPositions.from_dict(joint_dict)

    def _teardown_joint_feedback(self) -> None:
        """Stop the loop, restore prior operating modes, and drop the
        encoder link/client. Tolerates partial connect state."""
        if self._loop is not None:
            try:
                self._loop.stop()
            except Exception:
                logger.exception("failed to stop joint loop thread")
            self._loop = None
        self._pid = None

        if self._prior_modes_snapshot and self._motor_client is not None:
            try:
                motor_ids = list(self._prior_modes_snapshot.keys())
                modes = [self._prior_modes_snapshot[mid] for mid in motor_ids]
                with self._motor_lock:
                    self._motor_client.set_operating_mode_per_motor(motor_ids, modes)
            except Exception:
                logger.exception("failed to restore prior operating modes")
            self._prior_modes_snapshot = {}

        if self._encoder_client is not None:
            try:
                self._encoder_client.stop_encoder_stream()
            except Exception:
                pass
            try:
                self._encoder_client.disconnect()
            except Exception:
                pass
            self._encoder_client = None

        if self._encoder_link is not None:
            try:
                self._encoder_link.disconnect()
            except Exception:
                pass
            self._encoder_link = None


class MockOrcaHandJointFeedback(OrcaHandJointFeedback):
    """Drop-in :class:`OrcaHandJointFeedback` with in-memory mock motor +
    encoder-link clients (no serial I/O). The encoder client itself is real
    so the demuxer + AA A9 handler path is exercised in tests.
    """

    def _create_motor_client(self) -> MotorClient:
        from .hardware.mock_dynamixel_client import MockDynamixelClient

        return MockDynamixelClient(
            self.config.motor_ids, self.config.port, self.config.baudrate
        )

    def _create_encoder_link(self, port: str) -> HandSerialLink:
        from .hardware.mock_hand_serial_link import MockHandSerialLink

        return MockHandSerialLink(port=port, baudrate=LINK_DEFAULT_BAUDRATE)
