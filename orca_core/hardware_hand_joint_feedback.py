# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Closed-loop joint-feedback variant of :class:`OrcaHand`.

``connect()`` opens the motor bus, the encoder serial link, and starts a
:class:`~orca_core.control.JointLoopThread` running a vectorised PI on
joint-encoder error. The motors stay in ``current_based_position``; the
host writes ``Goal_Position`` per cycle and the motor's internal position
PID handles the fast tracking against the motor encoder. The host trims
the residual offset between motor angle and joint angle.

The wrist motor is not part of the loop; it stays in
``current_based_position`` and accepts targets through the inherited
synchronous motor-position path.
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional

from .control.constants import (
    DEFAULT_CORRECTION_MAX_DEG,
    DEFAULT_I_CLAMP_DEG,
    DEFAULT_KI,
    DEFAULT_KP,
)
from .control.joint_controller import JointController
from .control.joint_loop import JointLoopThread
from .hardware.hand_serial_link import HandSerialLink
from .hardware.joint_encoder_client import JointEncoderClient
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

    ``connect()`` opens the motor bus, the encoder serial link, and starts
    the joint-loop thread. The wrist stays in ``current_based_position``
    and is driven through the inherited synchronous path.

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
        self._controller: Optional[JointController] = None
        self._loop: Optional[JointLoopThread] = None

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

            self._controller = JointController(num_joints=len(motor_ids))
            self._controller.set_gains(
                Kp=DEFAULT_KP,
                Ki=DEFAULT_KI,
                correction_max_deg=DEFAULT_CORRECTION_MAX_DEG,
                i_clamp_deg=DEFAULT_I_CLAMP_DEG,
            )
            self._loop = JointLoopThread(self, self._encoder_client, self._controller)
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
        """Stop the loop and drop the encoder link/client. Tolerates
        partial connect state so failure paths can re-enter cleanly."""
        if self._loop is not None:
            try:
                self._loop.stop()
            except Exception:
                logger.exception("failed to stop joint loop thread")
            self._loop = None
        self._controller = None

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
