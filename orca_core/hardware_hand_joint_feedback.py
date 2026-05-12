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
from .joint_position import OrcaJointPositions


logger = logging.getLogger(__name__)


class JointFeedbackConnectError(RuntimeError):
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
    surface as a :class:`JointFeedbackConnectError`. The motor bus opened
    by ``super().connect()`` is rolled back before the exception escapes,
    so a caller that catches the error sees the hand in the same state it
    started in.
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

    # ----- Construction seams (overridden by MockOrcaHandJointFeedback) ----

    def _create_encoder_link(self, port: str) -> HandSerialLink:
        return HandSerialLink(port=port, baudrate=LINK_DEFAULT_BAUDRATE)

    def _create_encoder_client(self, link: HandSerialLink) -> JointEncoderClient:
        return JointEncoderClient(link)

    # ----- Internal helpers ------------------------------------------------

    def _encoder_motor_ids(self) -> List[int]:
        joint_to_motor = self.config.joint_to_motor_map
        return [joint_to_motor[j] for j in self._encoder_backed_joints()]

    # ----- Lifecycle -------------------------------------------------------

    def connect(self) -> tuple[bool, str]:
        success, msg = super().connect()
        if not success:
            return success, msg

        try:
            ports = resolve_sensing_ports(
                encoder_override=self.config.encoder_serial_port,
            )
            if ports.encoder is None:
                raise JointFeedbackConnectError(
                    "No encoder serial port resolved "
                    f"(encoder_serial_port={self.config.encoder_serial_port!r})."
                )

            self._encoder_link = self._create_encoder_link(ports.encoder)
            self._encoder_link.connect()
            self._encoder_client = self._create_encoder_client(self._encoder_link)
            self._encoder_client.connect()
            self._encoder_client.start_encoder_stream()

            if not self.is_calibrated(use_joint_feedback=True):
                raise JointFeedbackConnectError(
                    "Hand is missing joint-encoder calibration; "
                    "run calibration with use_joint_feedback enabled."
                )

            motor_ids = self._encoder_motor_ids()
            if not motor_ids:
                raise JointFeedbackConnectError(
                    "No encoder-backed joints configured "
                    "(set joint_encoder_joints in config.yaml)."
                )

            # Wrap offsets feed the joint→motor mapping the loop runs every
            # cycle; populate them once here so the loop's snapshot is
            # deterministic.
            self._compute_wrap_offsets_dict()

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
            # The motor bus opened by super().connect() is part of the
            # session this connect attempt was supposed to set up; roll it
            # back so the caller doesn't inherit a half-connected hand.
            try:
                super().disconnect()
            except Exception:
                logger.exception("super().disconnect() failed during connect rollback")
            raise

        return True, f"{msg} | Joint feedback loop running on {ports.encoder}"

    def disconnect(self) -> tuple[bool, str]:
        self._teardown_joint_feedback()
        return super().disconnect()

    def _teardown_joint_feedback(self) -> None:
        """Stop the loop and drop the encoder link/client. Tolerates
        partial-connect state so failure paths can re-enter cleanly. Errors
        are logged (not swallowed) so a stuck teardown still surfaces."""
        if self._loop is not None:
            try:
                self._loop.stop()
            except Exception:
                logger.exception("failed to stop joint loop thread")
            self._loop = None
        self._controller = None

        if self._encoder_client is not None:
            try:
                self._encoder_client.disconnect()
            except Exception:
                logger.exception("failed to disconnect encoder client")
            self._encoder_client = None

        if self._encoder_link is not None:
            try:
                self._encoder_link.disconnect()
            except Exception:
                logger.exception("failed to disconnect encoder link")
            self._encoder_link = None

    # ----- Joint position routing ------------------------------------------

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
            super()._set_joint_positions(OrcaJointPositions(rest))
        return True

    def _get_joint_positions(self):
        if self._loop is None:
            return super()._get_joint_positions()

        # Start from the loop's encoder-measured angles, then patch in the
        # wrist via its own motor read — avoids the full _motor_to_joint_pos
        # pass that would (a) recompute the 16 encoder joints we're about
        # to overwrite and (b) spam calibration-warning prints every cycle.
        joint_dict: Dict[str, float] = dict(self._loop.get_measured_joints())
        wrist_joint = self._wrist_joint_name()
        if wrist_joint is not None:
            wrist_angle = self._wrist_joint_angle()
            if wrist_angle is not None:
                joint_dict[wrist_joint] = wrist_angle
        return OrcaJointPositions.from_dict(joint_dict)

    def _wrist_joint_name(self) -> Optional[str]:
        from .constants import WRIST

        if WRIST in self.config.joint_to_motor_map:
            return WRIST
        return None

    def _wrist_joint_angle(self) -> Optional[float]:
        """Read the wrist motor only and convert via the inherited motor→joint
        mapping. Returns ``None`` if the wrist isn't fully calibrated."""
        wrist_joint = self._wrist_joint_name()
        if wrist_joint is None:
            return None
        wrist_motor_id = self.config.joint_to_motor_map[wrist_joint]
        limits = self.motor_limits_dict.get(wrist_motor_id)
        ratio = self.calibration.joint_to_motor_ratios_dict.get(wrist_motor_id, 0.0)
        if limits is None or any(v is None for v in limits) or ratio == 0:
            return None
        motor_pos = self.get_motor_pos()
        idx = self.config.motor_id_to_idx_dict[wrist_motor_id]
        wrapped = motor_pos[idx] - (self._wrap_offsets_dict or {}).get(wrist_motor_id, 0.0)
        if self.config.joint_inversion_dict.get(wrist_joint, False):
            return self.config.joint_roms_dict[wrist_joint][1] - (wrapped - limits[0]) / ratio
        return self.config.joint_roms_dict[wrist_joint][0] + (wrapped - limits[0]) / ratio

    # ----- Public facade onto the loop + controller ------------------------

    def set_pid_gains(
        self,
        Kp,
        Ki,
        correction_max_deg: float,
        i_clamp_deg: Optional[float] = None,
    ) -> None:
        """Retune the outer-loop PI gains while the loop is running.

        ``i_clamp_deg`` defaults to ``correction_max_deg`` (the convention
        established during bring-up: anti-windup matches output clamp).
        Raises :class:`RuntimeError` when the joint loop isn't active.
        """
        if self._controller is None:
            raise RuntimeError("joint loop not running; call connect() first")
        clamp = correction_max_deg if i_clamp_deg is None else i_clamp_deg
        self._controller.set_gains(
            Kp=Kp,
            Ki=Ki,
            correction_max_deg=correction_max_deg,
            i_clamp_deg=clamp,
        )

    def get_measured_joints(self) -> Dict[str, float]:
        """Encoder-measured joint angles in degrees, per encoder-backed joint."""
        if self._loop is None:
            raise RuntimeError("joint loop not running; call connect() first")
        return self._loop.get_measured_joints()

    def get_loop_correction(self) -> Dict[str, float]:
        """Per-joint PI trim correction in degrees from the last cycle."""
        if self._loop is None:
            raise RuntimeError("joint loop not running; call connect() first")
        return self._loop.get_correction()

    def get_loop_stats(self) -> Dict[str, float]:
        """Diagnostic counters from the joint-loop thread (cycles_ok,
        cycles_overrun, e_stops, last_dt_s, fallback_active, …)."""
        if self._loop is None:
            raise RuntimeError("joint loop not running; call connect() first")
        return self._loop.get_stats()


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
