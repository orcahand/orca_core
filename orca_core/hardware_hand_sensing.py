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
import threading
from contextlib import contextmanager
from typing import Dict, List, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .hardware.mock_hand_serial_link import MockHandSerialLink

import numpy as np

from .calibration import JointEncoderCal
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
from .hardware.joint_encoder_client import JointEncoderClient, JointFeedbackConnectError
from .hardware.sensing.constants import JOINT_ENCODER_POLARITY_BY_SIDE
from .hardware.sensing.serial_discovery import baud_for_port, resolve_sensing_ports
from .hardware.sensing.taxel_geometry import TaxelGeometry
from .hardware.sensing.types import (
    LinkHealth,
    ResultantReading,
    TactileReading,
    TaxelData,
    TaxelReading,
)
from .hardware.tactile_client import TactileStreamStats, TactileClient, TactileSensorConfiguration
from .hardware_hand import MockMotorResolutionMixin, OrcaHand
from .joint_position import OrcaJointPositions
from .kinematics import HandKinematics, Transform
from .kinematics import frames as tactile_frames


logger = logging.getLogger(__name__)

_LOOP_ALREADY_RUNNING_MSG = (
    "Already connected with the joint-feedback loop running; call disconnect() "
    "before reconnecting with engage_feedback=False for open-loop control"
)


def _link_health(link: Optional[HandSerialLink]) -> Optional[LinkHealth]:
    """Snapshot ``link``'s health, or ``None`` when there is no link."""
    if link is None:
        return None
    return LinkHealth(
        connected=link.is_connected,
        port_dead=link.is_port_dead,
        port_error=link.port_error,
        stats=link.get_link_stats(),
    )


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
            except Exception as e:
                logger.warning("Sensor connection failed on %s: %s", port, e)
                self._teardown_tactile()
                continue
            # Opening proves nothing on its own — any serial device accepts a
            # connection. Require the configuration read to have answered.
            if self._tactile_client.get_tactile_configuration() is None:
                logger.warning(
                    "Port %s opened but the sensor did not respond; "
                    "trying next candidate", port,
                )
                self._teardown_tactile()
                continue
            if port != configured:
                self.config = dataclasses.replace(self.config, sensor_port=port)
            return True, f"Sensor connected on {port} @ {baud}"

        return False, (
            "Sensor connection failed: no usable port (set sensors.port in config.yaml "
            "or check that the sensor adapter is plugged in)"
        )

    def connect(
        self, interactive: bool = True, engage_feedback: bool = True
    ) -> tuple[bool, str]:
        """Open the motor bus and the tactile sensor link.

        ``engage_feedback`` is accepted for signature parity with the
        joint-feedback hands; this hand has no loop to engage.
        """
        # Idempotent like the motor-only connect: never re-attach over (and
        # orphan) a live tactile link.
        if self.is_connected() and self._tactile_client is not None:
            return True, "Already connected"
        success, msg = super().connect(interactive)
        if not success:
            return success, msg
        if self._tactile_client is not None:
            return True, f"{msg} | Sensor already connected"

        sensor_ok, sensor_msg = self._connect_sensor_with_fallback()
        if not sensor_ok:
            # Roll the motor bus back so the caller doesn't inherit a
            # half-connected hand.
            try:
                super().disconnect()
            except Exception:
                logger.exception("motor disconnect failed during connect rollback")
            return False, f"{msg} | {sensor_msg}"
        return True, f"{msg} | {sensor_msg}"

    def connect_sensors_only(self) -> tuple[bool, str]:
        """Connect only the tactile sensor, skipping the motor bus.

        Useful for sensor bring-up and testing on a hand whose motors are not
        powered. After this call, tactile methods work; motor-control methods
        will fail because the motor client is not initialised.
        """
        return self._connect_sensor_with_fallback()

    def disconnect(self) -> tuple[bool, str]:
        # Tear motors down before sensors: motor disable_torque needs the bus
        # responsive, while the tactile teardown only blocks on its own port.
        result = super().disconnect()
        self._teardown_tactile()
        return result

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

    def zero_tactile_sensors(
        self, num_samples: int = 100, timeout_s: float | None = None
    ) -> dict:
        """Capture current readings as zero baseline and return offsets.

        ``timeout_s`` bounds the wait for stream frames; ``None`` derives a
        deadline from ``num_samples``.
        """
        return self._require_tactile_client().capture_taxel_offsets(
            num_samples=num_samples, timeout_s=timeout_s
        )

    def clear_tactile_zero(self) -> None:
        self._require_tactile_client().clear_taxel_offsets()

    def get_tactile_configuration(self) -> TactileSensorConfiguration | None:
        if self._tactile_client is None:
            return None
        return self._tactile_client.get_tactile_configuration()

    def get_tactile_stats(self) -> TactileStreamStats:
        """Return ``TactileStreamStats`` for the running auto-stream."""
        return self._require_tactile_client().get_stats()

    def get_tactile_link_health(self) -> LinkHealth | None:
        """Health snapshot of the serial link carrying the tactile stream, or
        ``None`` when no tactile link is open. ``port_dead`` reports a hard
        port failure (e.g. USB unplug) the stream cannot recover from."""
        return _link_health(self._tactile_link)

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

    Connect-time preconditions raise: an unsupported hand side (closed-loop
    control is validated on right-hand assemblies only), a missing encoder
    port, an absent ``joint_encoder_calibration`` block, or an encoder-stream
    timeout each surface as a :class:`JointFeedbackConnectError`. The motor
    bus opened by ``super().connect()`` is rolled back before the exception
    escapes, so a caller that catches the error sees the hand in the same
    state it started in. ``connect(engage_feedback=False)`` skips the loop
    (and its preconditions) entirely and opens the motor bus alone.
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
        self._loop_skipped_joints: List[str] = []
        self._estop_fallback_logged = False
        self._init_skip_calibrate = False

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

        backed = self._encoder_backed_joints()
        if not backed:
            raise JointFeedbackConnectError(
                "No encoder-backed joints configured "
                "(set joint_encoder_joints in config.yaml)."
            )

        # Close the loop only on fully calibrated joints; the rest stay on
        # open-loop motor control rather than blocking the whole feedback tier.
        ready = self._loop_ready_joints()
        if not ready:
            raise JointFeedbackConnectError(
                "No encoder-backed joint is fully calibrated (each needs "
                "motor limits, a joint-to-motor ratio, and a joint-encoder "
                "calibration anchor); run calibration with "
                "use_joint_feedback enabled."
            )
        self._loop_skipped_joints = [j for j in backed if j not in ready]
        if self._loop_skipped_joints:
            logger.warning(
                "joint(s) %s are missing motor or encoder calibration — "
                "running them open-loop (motor control only) until "
                "recalibrated", ", ".join(self._loop_skipped_joints),
            )

        # Wrap offsets feed the loop's per-cycle joint→motor mapping; populate
        # them once here so the loop's snapshot is deterministic.
        self._compute_wrap_offsets_dict()

        self._controller = JointController(num_joints=len(ready))
        self._controller.set_gains(
            Kp=DEFAULT_KP,
            Ki=DEFAULT_KI,
            correction_max_deg=DEFAULT_CORRECTION_MAX_DEG,
            i_clamp_deg=DEFAULT_I_CLAMP_DEG,
        )
        self._loop = JointLoopThread(
            self, self._encoder_client, self._controller, joints=ready
        )
        self._estop_fallback_logged = False
        self._loop.start()

    # ----- Internal helpers ------------------------------------------------

    def _encoder_motor_ids(self) -> List[int]:
        joint_to_motor = self.config.joint_to_motor_map
        return [joint_to_motor[j] for j in self._encoder_backed_joints()]

    def _require_validated_feedback_side(self) -> None:
        """Closed-loop control needs the per-side encoder polarity table;
        refuse sides (e.g. left-hand assemblies) without a validated one."""
        side = self.config.type
        if side not in JOINT_ENCODER_POLARITY_BY_SIDE:
            alternative = (
                f"load a feedback-free model such as orcahand-{side} / "
                f"orcahand-touch-{side} (load_hand(..., engage_feedback=False) "
                "does the same)"
                if side in ("left", "right")
                else "set 'type:' in config.yaml to a validated side"
            )
            raise JointFeedbackConnectError(
                f"Closed-loop joint feedback is unvalidated for {side!r} hand "
                "assemblies: no encoder polarity table exists for that side. "
                "Call connect(engage_feedback=False) to drive this hand "
                f"open-loop (tactile still works), or {alternative}."
            )

    def _loop_ready_joints(self) -> List[str]:
        """Encoder-backed joints calibrated well enough to close the loop on:
        motor limits + nonzero joint-to-motor ratio + an encoder anchor."""
        encoder_cal = self.calibration.joint_encoder_calibration_dict
        ratios = self.calibration.joint_to_motor_ratios_dict
        ready: List[str] = []
        for joint in self._encoder_backed_joints():
            if joint not in encoder_cal:
                continue
            motor_id = self.config.joint_to_motor_map.get(joint)
            limits = self.motor_limits_dict.get(motor_id)
            if not limits or any(limit is None for limit in limits):
                continue
            if not ratios.get(motor_id):
                continue
            ready.append(joint)
        return ready

    @property
    def loop_joint_names(self) -> Optional[List[str]]:
        """Joints the running loop closes on, or ``None`` when no loop runs.
        Encoder-backed joints absent from this list are on open-loop motor
        control (see ``loop_skipped_joints``)."""
        return list(self._loop.joint_names) if self._loop is not None else None

    @property
    def loop_skipped_joints(self) -> List[str]:
        """Encoder-backed joints excluded from the loop at connect because
        their motor or encoder calibration is incomplete."""
        return list(self._loop_skipped_joints)

    # ----- Lifecycle -------------------------------------------------------

    def connect(
        self, interactive: bool = True, engage_feedback: bool = True
    ) -> tuple[bool, str]:
        """Open the motor bus and start the closed-loop joint feedback.

        ``engage_feedback=False`` opens the motor bus only — no encoder
        link, no loop, and no validated-side gate — so the hand can be
        driven open-loop (e.g. for maintenance or on hand sides the loop
        does not support yet). It fails on a hand whose loop already runs
        rather than reporting an open-loop hand that isn't one.
        """
        if not engage_feedback:
            if self._loop is not None:
                return False, _LOOP_ALREADY_RUNNING_MSG
            return super().connect(interactive)
        # Idempotent like the motor-only connect: never orphan a running
        # loop and encoder link by re-attaching over them.
        if self.is_connected() and self._loop is not None:
            return True, "Already connected"
        self._require_validated_feedback_side()
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
            # Roll back the motor bus super().connect() opened so the caller
            # doesn't inherit a half-connected hand.
            try:
                super().disconnect()
            except Exception:
                logger.exception("super().disconnect() failed during connect rollback")
            raise

        return True, f"{msg} | Joint feedback loop running on {ports.encoder}" + (
            f" (motor-only: {', '.join(self._loop_skipped_joints)})"
            if self._loop_skipped_joints else ""
        )

    def disconnect(self) -> tuple[bool, str]:
        self._teardown_joint_feedback()
        return super().disconnect()

    def _teardown_joint_feedback(self) -> None:
        """Stop the loop and drop the encoder link/client. Tolerates
        partial-connect state so failure paths can re-enter cleanly. Errors
        are logged (not swallowed) so a stuck teardown still surfaces."""
        if self._loop is not None:
            try:
                if not self._loop.stop():
                    logger.warning(
                        "joint loop thread did not stop within its join timeout; "
                        "it may still be issuing motor writes"
                    )
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

    def disable_torque(self, motor_ids: Optional[List[int]] = None) -> List[int]:
        with self._loop_writes_paused():
            return super().disable_torque(motor_ids)

    def enable_torque(self, motor_ids: Optional[List[int]] = None) -> List[int]:
        with self._loop_writes_paused():
            return super().enable_torque(motor_ids)

    def set_control_mode(self, mode: str, motor_ids: Optional[List[int]] = None):
        # Mode changes are torque-off/mode-write/torque-on round-trip
        # sequences; fence the loop's write stream like the torque toggles.
        with self._loop_writes_paused():
            super().set_control_mode(mode, motor_ids)

    def _refuse_routine_while_loop_runs(self, routine: str) -> None:
        if self._loop is not None:
            raise RuntimeError(
                f"{routine}() while the joint-feedback loop is running is not "
                f"supported: the 100 Hz loop and the {routine} routine would "
                "command the same motors. Connect without engaging feedback "
                "(e.g. via the maintenance scripts) and retry."
            )

    def calibrate(self, *args, **kwargs):
        """Refuse to calibrate while the joint loop is running (it would
        fight for the same motors); :meth:`init_joints` instead skips the
        calibrate step, keeping connect-admitted joints open-loop."""
        if self._loop is not None and self._init_skip_calibrate:
            logger.warning(
                "skipping calibration while the joint-feedback loop runs; "
                "joint(s) %s stay open-loop until recalibrated",
                ", ".join(self._loop_skipped_joints) or "none",
            )
            return None
        self._refuse_routine_while_loop_runs("calibrate")
        return super().calibrate(*args, **kwargs)

    def tension(self, *args, **kwargs):
        """Refuse to run tendon tensioning while the joint loop is running
        (it would fight for the same motors)."""
        self._refuse_routine_while_loop_runs("tension")
        return super().tension(*args, **kwargs)

    def jitter(self, *args, **kwargs):
        """Refuse to run tendon-seating jitter while the joint loop is
        running (it would fight for the same motors)."""
        self._refuse_routine_while_loop_runs("jitter")
        return super().jitter(*args, **kwargs)

    def init_joints(self, force_calibrate: bool = False, move_to_neutral: bool = True):
        """Prepare the hand for operation (see :meth:`OrcaHand.init_joints`).

        With the joint loop running, calibration cannot run: joints
        ``connect()`` admitted as skipped stay open-loop instead of failing
        mid-sequence, and ``force_calibrate=True`` is refused up front
        (before torque or mode changes).
        """
        if self._loop is not None and force_calibrate:
            raise RuntimeError(
                "force_calibrate requires connecting without the "
                "joint-feedback loop: calibration cannot run while the "
                "100 Hz loop commands the motors."
            )
        self._init_skip_calibrate = self._loop is not None
        try:
            return super().init_joints(
                force_calibrate=force_calibrate, move_to_neutral=move_to_neutral
            )
        finally:
            self._init_skip_calibrate = False

    # ----- Joint position routing ------------------------------------------

    def _loop_engaged(self) -> bool:
        """True while the joint loop is live. After the watchdog e-stop the
        loop thread stops writing motors and refreshing measurements, so
        joint I/O falls back to the inherited open-loop path (logged once)."""
        if self._loop is None:
            return False
        if not self._loop.get_stats()["fallback_active"]:
            return True
        if not self._estop_fallback_logged:
            logger.error(
                "joint loop e-stopped; joint commands and reads fall back to "
                "the open-loop motor path"
            )
            self._estop_fallback_logged = True
        return False

    def _set_joint_positions(self, joint_pos) -> bool:
        if not self._loop_engaged():
            return super()._set_joint_positions(joint_pos)

        # Route by the loop's actual joint set: joints skipped at connect
        # (incomplete calibration) must take the open-loop motor path.
        encoder_joints = set(self._loop.joint_names)
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
        if not self._loop_engaged():
            return super()._get_joint_positions()

        # Motor-path angles cover the wrist and joints the loop doesn't
        # measure (e.g. skipped at connect); encoder angles win where present.
        joint_dict: Dict[str, float] = super()._get_joint_positions().as_dict()
        joint_dict.update(self._loop.get_measured_joints())
        return OrcaJointPositions.from_dict(joint_dict)

    # ----- Public facade onto the loop + controller ------------------------

    def set_pid_gains(
        self,
        Kp,
        Ki,
        correction_max_deg: float,
        i_clamp_deg: Optional[float] = None,
    ) -> None:
        """Retune the outer-loop PI gains while the loop is running.

        ``i_clamp_deg`` defaults to ``correction_max_deg`` (the anti-windup
        clamp matches the output clamp).
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

    def _require_live_loop(self) -> None:
        if self._loop is None:
            raise RuntimeError("joint loop not running; call connect() first")
        if self._loop.get_stats()["fallback_active"]:
            raise RuntimeError(
                "joint loop e-stopped; its measurements are frozen. Use "
                "get_joint_position() for live angles (see get_loop_stats())."
            )

    def get_measured_joints(self) -> Dict[str, float]:
        """Encoder-measured joint angles in degrees, per loop-controlled joint.

        Raises :class:`RuntimeError` when no loop is running or after the
        watchdog e-stop (the loop's last measurement is frozen); use
        :meth:`get_joint_position` for live open-loop angles.
        """
        self._require_live_loop()
        return self._loop.get_measured_joints()

    def get_loop_correction(self) -> Dict[str, float]:
        """Per-joint PI trim correction in degrees from the last cycle.

        Raises :class:`RuntimeError` when no loop is running or after the
        watchdog e-stop (the loop's last correction is frozen).
        """
        self._require_live_loop()
        return self._loop.get_correction()

    def get_encoder_link_health(self) -> LinkHealth | None:
        """Health snapshot of the joint-encoder serial link, or ``None`` when
        no encoder link is open. ``port_dead`` reports a hard port failure
        (e.g. USB unplug) the stream cannot recover from."""
        return _link_health(self._encoder_link)

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

    def connect(
        self, interactive: bool = True, engage_feedback: bool = True
    ) -> tuple[bool, str]:
        """Connect motors, tactile sensing, and the joint-feedback loop.

        ``engage_feedback=False`` connects motors + tactile only — no
        encoder link, no loop, and no validated-side gate — so e.g. a
        left-hand assembly still gets open-loop control with tactile. It
        fails on a hand whose loop already runs rather than reporting an
        open-loop hand that isn't one.
        """
        if not engage_feedback:
            return self._connect_without_feedback(interactive)

        # Idempotent like the motor-only connect: never orphan the running
        # loop or the live links by re-attaching over them.
        if (
            self.is_connected()
            and self._loop is not None
            and self._tactile_client is not None
        ):
            return True, "Already connected"
        self._require_validated_feedback_side()
        # Motor bus only — bypass the single-sensor connect() chains so this
        # class fully controls how the tactile and encoder links are opened.
        success, msg = OrcaHand.connect(self, interactive)
        if not success:
            # A failed motor connect must not cost a live sensors-only
            # tactile session (the documented sensors-first bring-up).
            return success, msg

        try:
            # A sensors-only tactile attach can't be kept: the topology is
            # re-resolved here (tactile may need the shared encoder link).
            self._teardown_tactile()
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
                # One link carries both streams: attach tactile onto the encoder
                # link, leaving _tactile_link None so teardown closes it once.
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

        loop_note = (
            f" (motor-only: {', '.join(self._loop_skipped_joints)})"
            if self._loop_skipped_joints else ""
        )
        return True, (
            f"{msg} | Joint feedback loop running on {ports.encoder}{loop_note} "
            f"| Tactile on {tactile_where}"
        )

    def _connect_without_feedback(self, interactive: bool) -> tuple[bool, str]:
        """Motor bus + tactile only, mirroring :meth:`OrcaHandTouch.connect`
        but never touching the encoder link or loop."""
        if self._loop is not None:
            return False, _LOOP_ALREADY_RUNNING_MSG
        if self.is_connected() and self._tactile_client is not None:
            return True, "Already connected"
        success, msg = OrcaHand.connect(self, interactive)
        if not success:
            return success, msg
        if self._tactile_client is not None:
            return True, f"{msg} | Sensor already connected"

        sensor_ok, sensor_msg = self._connect_sensor_with_fallback()
        if not sensor_ok:
            # Roll the motor bus back so the caller doesn't inherit a
            # half-connected hand.
            try:
                OrcaHand.disconnect(self)
            except Exception:
                logger.exception("motor disconnect failed during connect rollback")
            return False, f"{msg} | {sensor_msg}"
        return True, f"{msg} | {sensor_msg}"

    def get_tactile_link_health(self) -> LinkHealth | None:
        """Health of the link carrying the tactile stream — the shared
        encoder link when both streams ride one port."""
        if self._tactile_link is None and self._tactile_client is not None:
            return _link_health(self._encoder_link)
        return super().get_tactile_link_health()

    def disconnect(self) -> tuple[bool, str]:
        """Tear down in dependency order: tactile first (its ``stop_stream()``
        needs the link alive), then joint feedback (stopping the loop that
        drives the motor bus and closing the shared or dedicated encoder link
        — closed once, since tactile leaves ``_tactile_link`` unset when
        shared), motors last, once nothing writes to the bus anymore."""
        self._teardown_tactile()
        self._teardown_joint_feedback()
        return OrcaHand.disconnect(self)


class _MockEncoderFramePump:
    """Daemon thread feeding a fixed AA A9 frame to a mock link so the
    encoder stream starts and its freshness watchdog stays satisfied."""

    _PERIOD_S = 0.005

    def __init__(self, link, frame: bytes):
        self._link = link
        self._frame = frame
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="MockEncoderFramePump", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.wait(self._PERIOD_S):
            try:
                self._link.feed_bytes(self._frame)
            except Exception:
                return

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)


class MockOrcaHandTouch(MockMotorResolutionMixin, OrcaHandTouch):
    """Drop-in :class:`OrcaHandTouch` with in-memory mock motor + sensor
    clients: no serial I/O, no port discovery, and register reads served
    from an in-memory sensor state (all fingers connected).
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Mock sensors don't sit on a real bus: 'auto' must not enumerate
        # or probe real serial ports.
        replacements = {}
        if self.config.sensor_port == "auto":
            replacements["sensor_port"] = "mock"
        if self.config.sensor_baudrate == "auto":
            from .hardware.sensing.constants import DEFAULT_SENSOR_BAUDRATE

            replacements["sensor_baudrate"] = DEFAULT_SENSOR_BAUDRATE
        if replacements:
            self.config = dataclasses.replace(self.config, **replacements)

    def _create_tactile_link(self, port: str, baudrate: int) -> HandSerialLink:
        from .hardware.mock_hand_serial_link import MockHandSerialLink

        return MockHandSerialLink(port=port, baudrate=baudrate)

    def _attach_tactile_client(self, link: HandSerialLink) -> None:
        from .hardware.mock_hand_serial_link import MockHandSerialLink
        from .hardware.sensing.tactile_mock import TactileMockState, install_tactile_mock

        # Serve register reads from an in-memory sensor state so the client's
        # configuration read succeeds; a provider a test installed is kept.
        if isinstance(link, MockHandSerialLink) and link.response_provider is None:
            install_tactile_mock(
                link,
                TactileMockState(
                    finger_to_sensor_id=dict(self.config.finger_to_sensor_id)
                ),
            )
        super()._attach_tactile_client(link)

    def _connect_sensor_with_fallback(self) -> tuple[bool, str]:
        # Nothing to discover or fall back to on a mock: open the in-memory
        # link on the configured (pinned) port directly.
        port = self.config.sensor_port
        baudrate = self.config.sensor_baudrate
        try:
            self._open_tactile_on_port(port, int(baudrate))
        except Exception as e:
            self._teardown_tactile()
            return False, f"Sensor connection failed on {port}: {e}"
        return True, f"Sensor connected on {port} @ {baudrate}"

    @property
    def tactile_mock_link(self) -> "MockHandSerialLink":
        """The in-memory link behind the mock tactile client — feed it
        synthetic frames via :mod:`orca_core.hardware.sensing.tactile_mock`."""
        if self._tactile_link is None:
            raise RuntimeError(
                "tactile mock link not open; call connect() or "
                "connect_sensors_only() first"
            )
        return self._tactile_link


class MockOrcaHandJointFeedback(MockMotorResolutionMixin, OrcaHandJointFeedback):
    """Drop-in :class:`OrcaHandJointFeedback` with in-memory mock motor +
    encoder-link clients: no serial I/O and no port discovery. The encoder
    client itself is real so the demuxer + AA A9 handler path is exercised;
    a built-in pump keeps the mock link fed with encoder frames (override
    :meth:`_mock_encoder_frame` to shape the readings).
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._encoder_pump: Optional[_MockEncoderFramePump] = None
        if self.config.encoder_serial_port == "auto":
            self.config = dataclasses.replace(
                self.config, encoder_serial_port="mock"
            )

    def _install_mock_calibration(self) -> None:
        """Anchor the encoders on top of the synthesised motor calibration:
        mock encoders have no physical anchor pose to sample."""
        super()._install_mock_calibration()
        encoder_cal = dict(self.calibration.joint_encoder_calibration_dict)
        missing = [j for j in self._encoder_backed_joints() if j not in encoder_cal]
        if not missing:
            return
        encoder_cal.update({j: JointEncoderCal(enc_at_anchor_count=0) for j in missing})
        self.calibration = dataclasses.replace(
            self.calibration, joint_encoder_calibration_dict=encoder_cal
        )

    def _mock_encoder_frame(self) -> bytes:
        """One well-formed AA A9 frame (all counts zero) for the pump."""
        from .hardware.sensing.constants import (
            AUTO_ENC_NUM_JOINTS,
            PROTOCOL_HEADER_AUTO_ENC,
            PROTOCOL_RESERVED,
        )
        from .hardware.sensing.framing import calculate_checksum

        payload = bytes(1 + 2 * AUTO_ENC_NUM_JOINTS)
        body = (
            PROTOCOL_HEADER_AUTO_ENC
            + bytes([PROTOCOL_RESERVED])
            + len(payload).to_bytes(2, "little")
            + payload
        )
        return body + bytes([calculate_checksum(body)])

    def _create_encoder_link(self, port: str) -> HandSerialLink:
        from .hardware.mock_hand_serial_link import MockHandSerialLink

        link = MockHandSerialLink(port=port, baudrate=self.config.encoder_baudrate)
        if self._encoder_pump is not None:
            self._encoder_pump.stop()
        self._encoder_pump = _MockEncoderFramePump(link, self._mock_encoder_frame())
        return link

    def _teardown_joint_feedback(self) -> None:
        if self._encoder_pump is not None:
            self._encoder_pump.stop()
            self._encoder_pump = None
        super()._teardown_joint_feedback()


class MockOrcaHandFull(MockOrcaHandTouch, MockOrcaHandJointFeedback, OrcaHandFull):
    """Drop-in :class:`OrcaHandFull` with in-memory mock motor + link clients.

    The mock bases supply the in-memory motor client and mock serial links;
    :class:`OrcaHandFull` supplies the shared-link connect/disconnect logic.
    """

    @property
    def tactile_mock_link(self) -> "MockHandSerialLink":
        """The mock link carrying the tactile stream — the shared encoder
        link when both streams ride one port."""
        if self._tactile_link is None and self._encoder_link is not None:
            return self._encoder_link
        return super().tactile_mock_link
