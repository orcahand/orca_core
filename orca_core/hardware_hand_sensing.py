# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Sensing-equipped variants of :class:`~orca_core.OrcaHand`.

One class per sensing capability, plus their combination:

- :class:`OrcaHandTouch` — tactile fingertip sensing.
- :class:`OrcaHandJointFeedback` — closed-loop joint control from the
  joint-angle encoders.
- :class:`OrcaHandFull` — both, sharing one serial link when the two
  streams arrive on the same port.

Every variant follows the same shape so combiners (and future capabilities)
can be written by symmetry: construction seams for its link/client
(overridden by the ``Mock*`` classes at the bottom), an
attach-onto-open-link / open-own-port / teardown trio that lets a combiner
decide how serial links are shared, and ``connect()``/``disconnect()``
extending the motor-only lifecycle.
"""

from __future__ import annotations

import dataclasses
import logging
from contextlib import contextmanager
from typing import Dict, List, Optional

import numpy as np

from .control.constants import (
    DEFAULT_CORRECTION_MAX_DEG,
    DEFAULT_I_CLAMP_DEG,
    DEFAULT_KI,
    DEFAULT_KP,
)
from .control.joint_controller import JointController
from .control.joint_loop import JointLoopThread
from .hand_config import OrcaHandTouchConfig
from .hardware.hand_serial_link import HandSerialLink
from .hardware.joint_encoder_client import JointEncoderClient
from .hardware.sensing.serial_discovery import baud_for_port, resolve_sensing_ports
from .hardware.sensing.taxel_geometry import TaxelGeometry
from .hardware.sensing.types import ResultantReading, TactileReading, TaxelData, TaxelReading
from .hardware.tactile_client import TactileStreamStats, TactileClient, TactileSensorConfiguration
from .hardware_hand import MockMotorResolutionMixin, OrcaHand
from .joint_position import OrcaJointPositions
from .kinematics import HandKinematics, Transform
from .kinematics import frames as tactile_frames


logger = logging.getLogger(__name__)


class OrcaHandTouch(OrcaHand):
    """ORCA hand with integrated tactile sensing.

    ``connect()`` opens both the motor bus and the sensor serial link;
    ``disconnect()`` tears down both.
    """

    config_cls = OrcaHandTouchConfig

    def __init__(
        self,
        config_path: str | None = None,
        calibration_path: str | None = None,
        model_version: str | None = None,
        model_name: str | None = None,
        config: OrcaHandTouchConfig | None = None,
    ):
        super().__init__(
            config_path=config_path,
            calibration_path=calibration_path,
            model_version=model_version,
            model_name=model_name,
            config=config,
        )
        self._tactile_link: HandSerialLink | None = None
        self._tactile_client: TactileClient | None = None
        self._base_pose = Transform.identity()

    def _create_tactile_link(self, port: str, baudrate: int) -> HandSerialLink:
        return HandSerialLink(port=port, baudrate=baudrate)

    def _create_tactile_client(self, link: HandSerialLink) -> TactileClient:
        return TactileClient(link, finger_to_sensor_id=self.config.finger_to_sensor_id)

    def _attach_tactile_client(self, link: HandSerialLink) -> None:
        """Connect a tactile client onto an already-open ``link``.

        Split out of :meth:`_open_tactile_on_port` so a hand that also runs
        joint feedback can attach tactile onto the shared encoder link
        instead of opening a second port. The caller owns ``link`` teardown,
        so this does not set ``self._tactile_link``.
        """
        client = self._create_tactile_client(link)
        client.connect()
        self._tactile_client = client

    def _open_tactile_on_port(self, port: str, baudrate: int) -> None:
        """Open a link on ``port`` at ``baudrate`` and connect a tactile client."""
        link = self._create_tactile_link(port, baudrate)
        link.connect()
        self._tactile_link = link
        self._attach_tactile_client(link)

    def _teardown_tactile(self) -> None:
        """Disconnect and drop the tactile client + link, tolerating partial state."""
        if self._tactile_client is not None:
            try:
                self._tactile_client.disconnect()
            except Exception:
                pass
            self._tactile_client = None
        if self._tactile_link is not None:
            try:
                self._tactile_link.disconnect()
            except Exception:
                pass
            self._tactile_link = None

    def _connect_sensor_with_fallback(self) -> tuple[bool, str]:
        """Open the sensor link, resolving the configured ``sensors.port`` against
        live discovery.

        ``"auto"`` discovers the port; an explicit path is tried first and then
        falls back to auto-discovery if it fails to open.
        """
        configured = self.config.sensor_port
        baud_override = self.config.sensor_baudrate

        candidates: list[tuple[str, int | None]] = []
        resolved = resolve_sensing_ports(
            tactile_override=configured, encoder_override="disabled",
            tactile_baud_override=baud_override,
        )
        if resolved.tactile:
            candidates.append((resolved.tactile, resolved.tactile_baudrate))
        if configured not in ("auto", "disabled"):
            auto = resolve_sensing_ports(
                tactile_override="auto", encoder_override="disabled",
                tactile_baud_override=baud_override,
            )
            if auto.tactile and auto.tactile not in [p for p, _ in candidates]:
                candidates.append((auto.tactile, auto.tactile_baudrate))

        for port, baud in candidates:
            # An explicit port skips discovery, so detect its baud directly.
            if baud is None:
                baud = baud_for_port(port)
            try:
                self._open_tactile_on_port(port, baud)
                if port != configured:
                    self.config = dataclasses.replace(self.config, sensor_port=port)
                return True, f"Sensor connected on {port} @ {baud}"
            except Exception as e:
                print(f"Sensor connection failed on {port}: {e}")
                self._teardown_tactile()

        return False, (
            "Sensor connection failed: no usable port (set sensors.port in config.yaml "
            "or check that the sensor adapter is plugged in)"
        )

    def connect(self, interactive: bool = True) -> tuple[bool, str]:
        success, msg = super().connect(interactive)
        if not success:
            return success, msg

        sensor_ok, sensor_msg = self._connect_sensor_with_fallback()
        if not sensor_ok:
            return False, f"{msg} | {sensor_msg}"
        return True, f"{msg} | {sensor_msg}"

    def connect_sensors_only(self) -> tuple[bool, str]:
        """Connect only the tactile sensor, skipping the motor bus.

        Useful for sensor bring-up and testing on a hand whose motors are not
        powered. After this call, tactile methods work; motor-control methods
        will fail because the motor client is not initialised.
        """
        return self._connect_sensor_with_fallback()

    def disconnect(self) -> None:
        # Tear motors down before sensors: motor disable_torque needs the bus
        # responsive, while the tactile teardown only blocks on its own port.
        super().disconnect()
        self._teardown_tactile()

    def _require_tactile_client(self) -> TactileClient:
        if self._tactile_client is None:
            raise RuntimeError(
                "Tactile sensor is not connected. Call connect() or "
                "connect_sensors_only() first."
            )
        return self._tactile_client

    def get_tactile_forces(self) -> ResultantReading | None:
        """Return the latest resultant ``ResultantReading``, or ``None`` if no
        sensor is connected or no frame has arrived yet."""
        if self._tactile_client is None:
            return None
        return self._tactile_client.get_latest_forces()

    def get_tactile_taxels(self) -> TaxelReading | None:
        """Return the latest per-taxel ``TaxelReading``, or ``None`` if no
        sensor is connected or no frame has arrived yet."""
        if self._tactile_client is None:
            return None
        return self._tactile_client.get_latest_taxels()

    def get_tactile_data(self) -> TactileReading | None:
        """Return resultant and per-taxel forces from the same frame, or
        ``None`` if no sensor is connected or no frame has arrived yet."""
        if self._tactile_client is None:
            return None
        return self._tactile_client.get_latest()

    def start_tactile_stream(
        self, resultant: bool = True, taxels: bool = False, min_sensors: int = 1
    ) -> None:
        self._require_tactile_client().start_stream(
            resultant=resultant, taxels=taxels, min_sensors=min_sensors,
        )

    def stop_tactile_stream(self) -> None:
        self._require_tactile_client().stop_stream()

    def zero_tactile_sensors(self, num_samples: int = 100) -> dict:
        """Capture current readings as zero baseline and return offsets."""
        return self._require_tactile_client().capture_taxel_offsets(num_samples=num_samples)

    def clear_tactile_zero(self) -> None:
        self._require_tactile_client().clear_taxel_offsets()

    def get_tactile_configuration(self) -> TactileSensorConfiguration | None:
        if self._tactile_client is None:
            return None
        return self._tactile_client.get_tactile_configuration()

    def get_tactile_stats(self) -> TactileStreamStats:
        """Return ``TactileStreamStats`` for the running auto-stream."""
        return self._require_tactile_client().get_stats()

    def get_taxel_geometry(self) -> Dict[str, TaxelGeometry]:
        """Return static per-taxel positions ``{finger: TaxelGeometry}`` for connected fingers.

        Positions are fixed sensor geometry in the sensor frame (meters) and
        row ``i`` aligns with taxel ``i`` from :meth:`get_tactile_taxels`, so
        the two join by index::

            geom = hand.get_taxel_geometry()["index"].positions   # (n, 3)
            forces = hand.get_tactile_taxels().as_array("index")  # (n, 3)
            combined = np.hstack([geom, forces])                  # (n, 6)

        Empty if the tactile sensor is not connected/configured.
        """
        if self._tactile_client is None:
            return {}
        return self._tactile_client.get_taxel_geometry()

    @property
    def kinematics(self) -> HandKinematics:
        """Packaged forward kinematics for this hand model."""
        if "thumb_cmc" not in self.config.joint_ids:
            raise NotImplementedError(
                "packaged kinematics are only available for v2 hand models"
            )
        if self.config.type not in ("left", "right"):
            raise ValueError("config must declare a hand type ('left' or 'right')")
        return HandKinematics.load(self.config.type)

    def set_base_pose(self, pose: Transform | np.ndarray) -> None:
        """Set the hand's pose in the world frame (``T_world_base``).

        Used by ``frame="world"`` lookups; identity until set. Pass e.g. the
        robot arm's end-effector pose whenever it moves.
        """
        self._base_pose = pose if isinstance(pose, Transform) else Transform(pose)

    def get_base_pose(self) -> Transform:
        return self._base_pose

    def _resolve_joint_pos(self, joint_pos) -> dict:
        if joint_pos is None:
            if self._motor_client is None:
                raise RuntimeError(
                    "joint angles unavailable (motor bus not connected); "
                    "pass joint_pos explicitly"
                )
            return dict(self.get_joint_position().data)
        if isinstance(joint_pos, OrcaJointPositions):
            return dict(joint_pos.data)
        return dict(joint_pos)

    def get_sensor_transforms(
        self,
        frame: str = tactile_frames.FINGERTIP,
        joint_pos: OrcaJointPositions | Dict[str, float] | None = None,
    ) -> Dict[str, Transform]:
        """Return ``{finger: T_frame_sensor}`` mapping sensor-frame data into ``frame``.

        ``frame`` is one of ``orca_core.kinematics.frames``: ``"sensor"``
        (identity), ``"fingertip"`` (static mount pose), ``"palm"``/``"base"``
        (forward kinematics), or ``"world"`` (``base`` composed with
        :meth:`set_base_pose`). Frames beyond ``fingertip`` need joint angles
        (degrees): pass ``joint_pos`` or the hand's current joint positions
        are used.
        """
        if frame == tactile_frames.SENSOR:
            from .kinematics import FINGERS

            return {finger: Transform.identity() for finger in FINGERS}
        kin = self.kinematics
        if frame == tactile_frames.FINGERTIP:
            return kin.sensor_mounts
        if frame in (tactile_frames.PALM, tactile_frames.BASE):
            return kin.sensor_poses(self._resolve_joint_pos(joint_pos), in_frame=frame)
        if frame == tactile_frames.WORLD:
            poses = kin.sensor_poses(
                self._resolve_joint_pos(joint_pos), in_frame=tactile_frames.BASE
            )
            return {finger: self._base_pose @ pose for finger, pose in poses.items()}
        raise ValueError(f"unknown frame {frame!r}; expected one of {tactile_frames.FRAMES}")

    def get_taxel_data(
        self,
        frame: str = tactile_frames.SENSOR,
        joint_pos: OrcaJointPositions | Dict[str, float] | None = None,
    ) -> Dict[str, TaxelData] | None:
        """Return joined per-taxel positions and forces per finger, in ``frame``.

        Positions (meters) come from the static sensor geometry and forces
        (Newtons) from the latest tactile stream frame; row ``i`` of both
        describes the same taxel. Positions get the full rigid transform into
        ``frame``; forces, being free vectors, are only rotated. See
        :meth:`get_sensor_transforms` for the available frames and how joint
        angles are sourced. Returns ``None`` if no stream frame has arrived
        yet; fingers whose geometry does not match the streamed taxel count
        are skipped.
        """
        if frame not in tactile_frames.FRAMES:
            raise ValueError(
                f"unknown frame {frame!r}; expected one of {tactile_frames.FRAMES}"
            )
        reading = self.get_tactile_taxels()
        if reading is None:
            return None
        geometry = self.get_taxel_geometry()
        is_sensor_frame = frame == tactile_frames.SENSOR
        transforms = None if is_sensor_frame else self.get_sensor_transforms(frame, joint_pos)

        data: Dict[str, TaxelData] = {}
        for finger in reading.fingers:
            if finger not in geometry:
                continue
            if transforms is not None and finger not in transforms:
                continue
            positions = geometry[finger].positions
            forces = reading.as_array(finger)
            if len(positions) != len(forces):
                continue
            if is_sensor_frame:
                transformed_positions, transformed_forces = positions, forces
            else:
                transform = transforms[finger]
                transformed_positions = transform.apply_to_points(positions)
                transformed_forces = transform.apply_to_vectors(forces)
            data[finger] = TaxelData(
                finger=finger,
                frame=frame,
                positions=transformed_positions,
                forces=transformed_forces,
                timestamp=reading.timestamp,
            )
        return data


class JointFeedbackConnectError(RuntimeError):
    """Raised when a joint-feedback connect precondition fails (no encoder
    port resolved, no encoder-backed joints, missing encoder calibration).
    """


class OrcaHandJointFeedback(OrcaHand):
    """ORCA hand with closed-loop joint feedback on the encoder-backed joints.

    ``connect()`` opens the motor bus, the encoder serial link, and starts a
    :class:`~orca_core.control.JointLoopThread` running a vectorised PI on
    joint-encoder error. The motors stay in ``current_based_position``: the
    host writes ``Goal_Position`` per cycle and the motor's internal position
    PID handles the fast tracking against the motor encoder, while the host
    trims the residual offset between motor angle and joint angle. The wrist
    is not part of the loop and is driven through the inherited synchronous
    path.

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
        return HandSerialLink(port=port, baudrate=self.config.encoder_baudrate)

    def _create_encoder_client(self, link: HandSerialLink) -> JointEncoderClient:
        return JointEncoderClient(link)

    def _attach_encoders(self, link: HandSerialLink) -> None:
        """Start the encoder client and joint loop on an already-open ``link``.

        Split out of :meth:`connect` so a hand that also has tactile sensing
        can share one :class:`HandSerialLink` between both streams.
        Raises :class:`JointFeedbackConnectError` on missing
        encoder calibration or no encoder-backed joints; the caller owns
        rollback. Does not own ``link`` teardown — the caller closes it.
        """
        self._encoder_client = self._create_encoder_client(link)
        self._encoder_client.connect()
        self._encoder_client.start_stream()

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

    # ----- Internal helpers ------------------------------------------------

    def _encoder_motor_ids(self) -> List[int]:
        joint_to_motor = self.config.joint_to_motor_map
        return [joint_to_motor[j] for j in self._encoder_backed_joints()]

    # ----- Lifecycle -------------------------------------------------------

    def connect(self, interactive: bool = True) -> tuple[bool, str]:
        success, msg = super().connect(interactive)
        if not success:
            return success, msg

        try:
            # Tactile isn't consumed here, so skip its discovery probing entirely.
            ports = resolve_sensing_ports(
                tactile_override="disabled",
                encoder_override=self.config.encoder_serial_port,
            )
            if ports.encoder is None:
                raise JointFeedbackConnectError(
                    "No encoder serial port resolved "
                    f"(encoder_serial_port={self.config.encoder_serial_port!r})."
                )

            self._encoder_link = self._create_encoder_link(ports.encoder)
            self._encoder_link.connect()
            self._attach_encoders(self._encoder_link)
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

    # ----- Torque toggles (fence loop writes off the bus) ------------------

    @contextmanager
    def _loop_writes_paused(self):
        """Pause the joint loop's motor writes for the duration of a round-trip
        motor op. The loop's 100 Hz sync_writes otherwise interleave with the
        op's status-packet reads on the shared motor CDC and intermittently
        stall them into "no status packet" timeouts. Encoder decoding keeps
        running, so ``get_measured_joints`` stays live throughout."""
        loop = self._loop
        if loop is None:
            yield
            return
        loop.pause_writes()
        try:
            yield
        finally:
            loop.resume_writes()

    def disable_torque(self, motor_ids: Optional[List[int]] = None):
        with self._loop_writes_paused():
            super().disable_torque(motor_ids)

    def enable_torque(self, motor_ids: Optional[List[int]] = None):
        with self._loop_writes_paused():
            super().enable_torque(motor_ids)

    def set_control_mode(self, mode: str, motor_ids: Optional[List[int]] = None):
        # Mode changes are torque-off/mode-write/torque-on round-trip
        # sequences; fence the loop's write stream like the torque toggles.
        with self._loop_writes_paused():
            super().set_control_mode(mode, motor_ids)

    def calibrate(self, *args, **kwargs):
        """Refuse to calibrate while the joint loop is running (it would fight for the same motors)."""
        if self._loop is not None:
            raise RuntimeError(
                "calibrate() while the joint-feedback loop is running is not "
                "supported: the 100 Hz loop and the calibration routine would "
                "command the same motors. Connect without engaging feedback "
                "(e.g. scripts/calibrate.py) and retry."
            )
        return super().calibrate(*args, **kwargs)

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

    def rebase_loop(self) -> None:
        """Re-anchor the running loop to the current pose (target=measured,
        fresh feed-forward bias, integral reset). Call after the motors have
        moved out from under the loop — e.g. torque was disabled to hand-pose
        the hand — so re-enabling torque doesn't lurch."""
        if self._loop is None:
            raise RuntimeError("joint loop not running; call connect() first")
        self._loop.rebase()

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


class OrcaHandFull(OrcaHandTouch, OrcaHandJointFeedback):
    """ORCA hand with both tactile sensing and closed-loop joint feedback.

    Combines :class:`OrcaHandTouch` and :class:`OrcaHandJointFeedback` without
    duplicating either: each capability is inherited, and this class only owns
    the ``connect``/``disconnect`` orchestration that decides how the two
    sensing streams share serial links.

    When both streams arrive on the same port, they ride **one** shared
    :class:`HandSerialLink` and are routed by frame type. When the tactile
    sensor is on its own port, each stream gets its own link at its own baud.
    """

    config_cls = OrcaHandTouchConfig

    def connect(self, interactive: bool = True) -> tuple[bool, str]:
        # Motor bus only — bypass the single-sensor connect() chains so this
        # class fully controls how the tactile and encoder links are opened.
        success, msg = OrcaHand.connect(self, interactive)
        if not success:
            return success, msg

        try:
            ports = resolve_sensing_ports(
                tactile_override=self.config.sensor_port,
                encoder_override=self.config.encoder_serial_port,
                tactile_baud_override=self.config.sensor_baudrate,
            )
            if ports.encoder is None:
                raise JointFeedbackConnectError(
                    "No encoder serial port resolved "
                    f"(encoder_serial_port={self.config.encoder_serial_port!r})."
                )
            if ports.tactile is None:
                raise RuntimeError(
                    "No tactile sensor port resolved "
                    f"(sensors.port={self.config.sensor_port!r})."
                )

            self._encoder_link = self._create_encoder_link(ports.encoder)
            self._encoder_link.connect()
            self._attach_encoders(self._encoder_link)

            if ports.shared:
                # One link carries both streams; attach the tactile client onto
                # the encoder link (leaves _tactile_link None, so its teardown
                # won't double-close the shared link).
                self._attach_tactile_client(self._encoder_link)
                tactile_where = f"shared link {ports.encoder}"
            else:
                # Tactile is on its own port: use the detected baud, or detect
                # it directly when the port was given explicitly.
                tactile_baud = ports.tactile_baudrate
                if tactile_baud is None:
                    tactile_baud = baud_for_port(ports.tactile)
                self._open_tactile_on_port(ports.tactile, tactile_baud)
                tactile_where = f"{ports.tactile} @ {tactile_baud}"
        except Exception:
            self._teardown_tactile()
            self._teardown_joint_feedback()
            try:
                OrcaHand.disconnect(self)
            except Exception:
                logger.exception("motor disconnect failed during connect rollback")
            raise

        return True, (
            f"{msg} | Joint feedback loop running on {ports.encoder} "
            f"| Tactile on {tactile_where}"
        )

    def disconnect(self) -> tuple[bool, str]:
        # Tactile first: its stop_stream() needs the link alive. Joint-feedback
        # teardown then stops the loop that drives the motor bus and closes the
        # shared (or dedicated encoder) link; a shared link is closed once
        # because tactile left _tactile_link None. Motors go down last, once
        # nothing is writing to the bus anymore.
        self._teardown_tactile()
        self._teardown_joint_feedback()
        return OrcaHand.disconnect(self)


class MockOrcaHandTouch(MockMotorResolutionMixin, OrcaHandTouch):
    """Drop-in :class:`OrcaHandTouch` with in-memory mock motor + sensor clients (no serial I/O)."""

    def _create_tactile_link(self, port: str, baudrate: int) -> HandSerialLink:
        from .hardware.mock_hand_serial_link import MockHandSerialLink

        return MockHandSerialLink(port=port, baudrate=baudrate)


class MockOrcaHandJointFeedback(MockMotorResolutionMixin, OrcaHandJointFeedback):
    """Drop-in :class:`OrcaHandJointFeedback` with in-memory mock motor +
    encoder-link clients (no serial I/O). The encoder client itself is real
    so the demuxer + AA A9 handler path is exercised in tests.
    """

    def _create_encoder_link(self, port: str) -> HandSerialLink:
        from .hardware.mock_hand_serial_link import MockHandSerialLink

        return MockHandSerialLink(port=port, baudrate=self.config.encoder_baudrate)


class MockOrcaHandFull(MockOrcaHandTouch, MockOrcaHandJointFeedback, OrcaHandFull):
    """Drop-in :class:`OrcaHandFull` with in-memory mock motor + link clients.

    The mock bases supply the in-memory motor client and mock serial links;
    :class:`OrcaHandFull` supplies the shared-link connect/disconnect logic.
    """
