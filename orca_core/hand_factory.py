# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Factory that selects the right hand class — and, by default, the right model.

A hand's capabilities are declared in ``config.yaml``: ``joint_encoder_joints``
plus ``use_joint_feedback`` decide whether the closed-loop joint-feedback
controller engages, and a ``sensors`` block declares tactile sensing.
:func:`load_hand` reads those once and returns the matching concrete class so
callers (and scripts) don't have to hand-pick hand classes.

| feedback | tactile | class                    |
|----------|---------|--------------------------|
| yes       | no      | ``OrcaHandJointFeedback``|
| yes       | yes     | ``OrcaHandFull``         |
| no      | no      | ``OrcaHand``             |
| no      | yes     | ``OrcaHandTouch``        |

Called with no config at all, :func:`load_hand` first runs
:func:`detect_hand`, which probes the connected hardware — the hand's side
from the controller board's identity reply, joint encoders from a live encoder
stream, tactile from a sensor register reply — and picks the bundled model
that matches, so ``load_hand()`` on a plugged-in hand just works. Hands
that report no side are treated as right-handed; a config selects the
model explicitly whenever detection can't.
"""

from __future__ import annotations

import dataclasses
import logging
from dataclasses import dataclass
from typing import Optional

from .hand_config import (
    OrcaHandConfig,
    OrcaHandTouchConfig,
    _resolve_config_path,
)
from .hardware.sensing.constants import (
    DEFAULT_ENCODER_BAUDRATE,
    JOINT_ENCODER_POLARITY_BY_SIDE,
)
from .hardware.sensing.serial_discovery import (
    OrcaBoardInfo,
    _tactile_responds_at,
    detect_encoder_stream,
    find_tactile_port,
    oh_board_ports,
    probe_orca_info,
)
from .hardware_hand import MockOrcaHand, OrcaHand
from .hardware_hand_sensing import (
    MockOrcaHandFull,
    MockOrcaHandJointFeedback,
    MockOrcaHandTouch,
    OrcaHandFull,
    OrcaHandJointFeedback,
    OrcaHandTouch,
)
from .utils.utils import read_yaml


logger = logging.getLogger(__name__)


# (feedback, tactile, mock) -> hand class
_CLASS_MATRIX = {
    (True, False, False): OrcaHandJointFeedback,
    (True, True, False): OrcaHandFull,
    (False, False, False): OrcaHand,
    (False, True, False): OrcaHandTouch,
    (True, False, True): MockOrcaHandJointFeedback,
    (True, True, True): MockOrcaHandFull,
    (False, False, True): MockOrcaHand,
    (False, True, True): MockOrcaHandTouch,
}

# (tactile, encoders) -> bundled v2 model, formatted with the detected side
_MODEL_BY_CAPS = {
    (False, False): "orcahand-{side}",
    (True, False): "orcahand-touch-{side}",
    (False, True): "orcahand-joint-{side}",
    (True, True): "orcahand-full-{side}",
}

_OH_PROBE_PASSES = 3
"""Passes over the controller board's CDCs (the probe is racy on macOS
composite CDC devices; see serial_discovery.ORCA_ID_PROBE_ATTEMPTS)."""


@dataclass(frozen=True)
class HandDetection:
    """What :func:`detect_hand` found plugged in.

    ``model_name`` is the bundled v2 model matching the detected side and
    sensing capabilities; the port fields carry what was discovered so the
    hand can connect without re-probing. ``identity`` is ``None`` for hands
    whose board doesn't report one.
    """

    model_name: str
    side: str
    has_tactile: bool
    has_encoders: bool
    motor_port: Optional[str] = None
    sensing_port: Optional[str] = None
    tactile_port: Optional[str] = None
    identity: Optional[OrcaBoardInfo] = None


def detect_hand() -> HandDetection:
    """Probe the connected hardware and name the bundled model that matches.

    The hand's side comes from the controller board's identity reply; joint
    encoders are confirmed by a live encoder stream on the sensing CDC and
    tactile by a sensor register reply (on the shared CDC or a dedicated
    adapter). Any question the hardware doesn't answer falls back
    conservatively: no side means right, no reply means the capability is
    absent — so with nothing plugged in this returns the plain right-hand
    model with all ports unset.
    """
    motor_port: Optional[str] = None
    sensing_port: Optional[str] = None
    identity: Optional[OrcaBoardInfo] = None

    candidates = oh_board_ports()
    for _ in range(_OH_PROBE_PASSES):
        for port in candidates:
            if port in (motor_port, sensing_port):
                continue
            info = probe_orca_info(port)
            if info is None:
                continue
            if info.role == "motor" and motor_port is None:
                motor_port = port
            elif info.role == "sensor" and sensing_port is None:
                sensing_port = port
            if identity is None or (identity.side is None and info.side):
                identity = info
        if motor_port is not None and sensing_port is not None:
            break

    has_encoders = sensing_port is not None and detect_encoder_stream(sensing_port)

    tactile_port = find_tactile_port()
    has_tactile = tactile_port is not None
    if not has_tactile and sensing_port is not None:
        has_tactile = _tactile_responds_at(sensing_port, DEFAULT_ENCODER_BAUDRATE)

    side = identity.side if identity is not None and identity.side else "right"
    model_name = _MODEL_BY_CAPS[(has_tactile, has_encoders)].format(side=side)

    return HandDetection(
        model_name=model_name,
        side=side,
        has_tactile=has_tactile,
        has_encoders=has_encoders,
        motor_port=motor_port,
        sensing_port=sensing_port,
        tactile_port=tactile_port,
        identity=identity,
    )


def _pin_detected_ports(config, detection: HandDetection):
    """Point the config at the ports detection already found, so connect()
    doesn't have to re-discover them. Fields with nothing detected keep
    their configured (typically ``auto``) values."""
    if detection.motor_port is not None:
        config = dataclasses.replace(config, port=detection.motor_port)
    if detection.has_encoders and detection.sensing_port is not None:
        config = dataclasses.replace(
            config, encoder_serial_port=detection.sensing_port
        )
    if detection.has_tactile and isinstance(config, OrcaHandTouchConfig):
        sensor_port = detection.tactile_port or detection.sensing_port
        if sensor_port is not None:
            config = dataclasses.replace(config, sensor_port=sensor_port)
    return config


def load_hand(
    config_path: str | None = None,
    calibration_path: str | None = None,
    model_version: str | None = None,
    model_name: str | None = None,
    mock: bool = False,
    engage_feedback: bool = True,
) -> OrcaHand:
    """Construct the hand class that matches a model's declared capabilities.

    With no config selection at all (no ``config_path``, ``model_name``, or
    ``model_version``, and ``mock=False``), the connected hardware is probed
    via :func:`detect_hand` and the matching bundled model is used — plug the
    hand in and ``load_hand()`` just works. Passing any of those arguments
    skips detection entirely.

    Args:
        config_path: Path to a ``config.yaml``. ``None`` autodetects the
            model from hardware (or uses the bundled default when
            ``model_version`` / ``model_name`` narrow it explicitly).
        calibration_path: Companion ``calibration.yaml``; defaults to the
            sibling of ``config.yaml``.
        model_version, model_name: Select a packaged model when
            ``config_path`` is ``None``.
        mock: Return the in-memory ``Mock*`` variant (no serial I/O,
            no hardware detection).
        engage_feedback: When ``False``, return the motor-only class even if
            the config enables joint feedback. The config still carries the
            encoder declaration so calibration's encoder pass runs.

    Returns:
        A constructed (not yet connected) hand instance.
    """
    detection = None
    if config_path is None and model_name is None and model_version is None and not mock:
        detection = detect_hand()
        model_name = detection.model_name

    resolved_config_path = _resolve_config_path(
        config_path,
        model_version=model_version,
        model_name=model_name,
    )
    raw = read_yaml(resolved_config_path) or {}
    tactile = "sensors" in raw

    config_cls = OrcaHandTouchConfig if tactile else OrcaHandConfig
    config = config_cls.from_config_path(
        config_path=resolved_config_path,
        calibration_path=calibration_path,
    )
    if detection is not None:
        config = _pin_detected_ports(config, detection)

    feedback = engage_feedback and config.joint_feedback_enabled
    if feedback and config.type not in JOINT_ENCODER_POLARITY_BY_SIDE:
        fallback_model = ("orcahand-touch-" if tactile else "orcahand-") + str(config.type)
        alternative = (
            f"use the {fallback_model} model"
            if config.type in ("left", "right")
            else "set 'type:' in config.yaml to a validated side"
        )
        logger.warning(
            "closed-loop joint feedback is unvalidated for %r hand assemblies; "
            "connect() will refuse to engage the loop. Pass "
            "engage_feedback=False (to load_hand or connect) for open-loop "
            "control, or %s.",
            config.type, alternative,
        )

    hand_cls = _CLASS_MATRIX[(bool(feedback), tactile, bool(mock))]
    return hand_cls(config=config)
