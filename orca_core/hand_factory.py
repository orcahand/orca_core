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
:func:`detect_hand` and uses the bundled model it names, so ``load_hand()`` on
a plugged-in hand just works. Detection reads the hand's side and sensing
config straight off the controller board's identity reply — the board knows
what it was built with, so that declaration wins over inference. Probing the
sensors (encoder stream, tactile register) then serves two purposes: it
supplies the capabilities for boards too old or too fresh to declare a
``CFG``, and it flags declared sensors that aren't answering. Hands that
report no side are treated as right-handed; a config selects the model
explicitly whenever detection can't.
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
    port_in_use,
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

# Provisioned sensing-config code -> (tactile, encoders). This is the hand's own
# declaration of what it was built with, stored in the controller board's flash
# and reported as ``CFG`` by ``ORCA_INFO?``; codes are assigned by provisioning
# (orca_firmware/oh_board/provision.py --config).
SENSING_CONFIG_CAPS = {
    1000: (False, False),  # motors only
    1500: (False, True),   # joint encoders
    2000: (True, False),   # tactile
    2500: (True, True),    # tactile + joint encoders
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
    whose board doesn't report one. ``busy_ports`` lists controller-board
    CDCs another process holds: those stay silent under probing, so anything
    behind them is missing from the rest of this result.

    ``has_tactile``/``has_encoders`` are what the hand *is*: the provisioned
    ``declared_config`` when the board reports one, else what probing found.
    ``probed_tactile``/``probed_encoders`` are what actually answered, so the
    two disagreeing means declared hardware isn't responding —
    ``missing_capabilities`` names those, and ``undeclared_capabilities``
    names sensors that answered without being provisioned.
    """

    model_name: str
    side: str
    has_tactile: bool
    has_encoders: bool
    motor_port: Optional[str] = None
    sensing_port: Optional[str] = None
    tactile_port: Optional[str] = None
    identity: Optional[OrcaBoardInfo] = None
    busy_ports: tuple[str, ...] = ()
    declared_config: Optional[int] = None
    probed_tactile: bool = False
    probed_encoders: bool = False
    missing_capabilities: tuple[str, ...] = ()
    undeclared_capabilities: tuple[str, ...] = ()


def detect_hand() -> HandDetection:
    """Probe the connected hardware and name the bundled model that matches.

    The hand's side and sensing capabilities come from the controller board's
    identity reply: a provisioned ``CFG`` code states what the hand was built
    with, and it is trusted over probing because it is ground truth rather
    than inference. Probing still runs, to catch declared hardware that isn't
    responding (see ``missing_capabilities``).

    Boards that report no ``CFG`` — unprovisioned, or firmware predating the
    field — fall back to probing: joint encoders from a live encoder stream on
    the sensing CDC, tactile from a sensor register reply (on the shared CDC or
    a dedicated adapter). Anything the hardware doesn't answer falls back
    conservatively: no side means right, no reply means the capability is
    absent — so with nothing plugged in this returns the plain right-hand
    model with all ports unset.

    A CDC another process already holds is silent under probing and so reads
    as absent; those ports are reported in ``busy_ports`` so callers can tell
    an incomplete result from a genuinely simpler hand.
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
            # Both CDCs of one board report the same identity, so this only
            # matters when one answers a fuller line than the other.
            if identity is None or (
                (identity.side is None and info.side)
                or (identity.config is None and info.config)
            ):
                identity = info
        if motor_port is not None and sensing_port is not None:
            break

    probed_encoders = sensing_port is not None and detect_encoder_stream(sensing_port)

    tactile_port = find_tactile_port()
    probed_tactile = tactile_port is not None
    if not probed_tactile and sensing_port is not None:
        probed_tactile = _tactile_responds_at(sensing_port, DEFAULT_ENCODER_BAUDRATE)

    declared_config = identity.config if identity is not None else None
    declared = SENSING_CONFIG_CAPS.get(declared_config)
    if declared_config is not None and declared is None:
        logger.warning(
            "controller board reports an unrecognised sensing config CFG=%s "
            "(known: %s); falling back to probing for this hand's sensors. "
            "Re-provision the board, or update SENSING_CONFIG_CAPS.",
            declared_config, ", ".join(str(c) for c in sorted(SENSING_CONFIG_CAPS)),
        )
    has_tactile, has_encoders = (
        declared if declared is not None else (probed_tactile, probed_encoders)
    )

    probed = {"tactile": probed_tactile, "encoders": probed_encoders}
    declared_caps = {"tactile": has_tactile, "encoders": has_encoders}
    missing = tuple(n for n, on in declared_caps.items() if on and not probed[n])
    undeclared = tuple(n for n, on in probed.items() if on and not declared_caps[n])

    side = identity.side if identity is not None and identity.side else "right"
    model_name = _MODEL_BY_CAPS[(has_tactile, has_encoders)].format(side=side)

    busy_ports = tuple(
        port
        for port in candidates
        if port not in (motor_port, sensing_port) and port_in_use(port)
    )

    return HandDetection(
        model_name=model_name,
        side=side,
        has_tactile=has_tactile,
        has_encoders=has_encoders,
        motor_port=motor_port,
        sensing_port=sensing_port,
        tactile_port=tactile_port,
        identity=identity,
        busy_ports=busy_ports,
        declared_config=declared_config,
        probed_tactile=probed_tactile,
        probed_encoders=probed_encoders,
        missing_capabilities=missing,
        undeclared_capabilities=undeclared,
    )


def _warn_on_capability_mismatch(detection: HandDetection) -> None:
    """Say so when the hand's provisioned config and its live sensors disagree.

    Declared-but-silent sensors are a fault to surface, not a simpler hand:
    the model is still loaded from the declaration, so ``connect()`` fails on
    the real problem instead of quietly handing back a sensorless hand.
    """
    if detection.missing_capabilities:
        logger.warning(
            "board %s declares CFG=%s (%s) but %s did not respond on %s. The "
            "%r model is loaded as declared, so connect() will fail until this "
            "is fixed — check the sensing cable and power-cycle the hand, or "
            "run scripts/check_sensors.py. Load a motor-only model explicitly "
            "to work around it.",
            (detection.identity and detection.identity.hand_id) or "?",
            detection.declared_config,
            _describe_caps(detection.has_tactile, detection.has_encoders),
            " and ".join(detection.missing_capabilities),
            detection.sensing_port or "any port",
            detection.model_name,
        )
    if detection.undeclared_capabilities:
        logger.warning(
            "board %s answers for %s, which its CFG=%s does not declare, so "
            "that sensing stays unused. The board is provisioned wrong: "
            "re-run provisioning with the code matching this hand.",
            (detection.identity and detection.identity.hand_id) or "?",
            " and ".join(detection.undeclared_capabilities),
            detection.declared_config,
        )


def _describe_caps(tactile: bool, encoders: bool) -> str:
    present = [n for n, on in (("tactile", tactile), ("encoders", encoders)) if on]
    return " + ".join(present) if present else "motors only"


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
    engage_sensors: bool = True,
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
        engage_sensors: When ``False``, return a class that does not open the
            tactile link even if the config declares sensors. Tactile and
            encoders can share one CDC, so a caller that opens its own reader
            on the sensing port must not have the hand open it too.

    Returns:
        A constructed (not yet connected) hand instance.
    """
    detection = None
    if config_path is None and model_name is None and model_version is None and not mock:
        detection = detect_hand()
        model_name = detection.model_name
        if detection.busy_ports:
            logger.warning(
                "controller-board port(s) %s are held by another process, so "
                "anything behind them went undetected and %r may understate "
                "this hand. Close the other client (a running UI, script or "
                "serial monitor), or name the model explicitly.",
                ", ".join(detection.busy_ports), model_name,
            )
        _warn_on_capability_mismatch(detection)

    resolved_config_path = _resolve_config_path(
        config_path,
        model_version=model_version,
        model_name=model_name,
    )
    raw = read_yaml(resolved_config_path) or {}
    declares_tactile = "sensors" in raw
    # The config keeps its sensor declaration either way; only the class that
    # would open the link is withheld, mirroring engage_feedback.
    tactile = declares_tactile and engage_sensors

    config_cls = OrcaHandTouchConfig if declares_tactile else OrcaHandConfig
    config = config_cls.from_config_path(
        config_path=resolved_config_path,
        calibration_path=calibration_path,
    )
    if detection is not None:
        config = _pin_detected_ports(config, detection)

    feedback = engage_feedback and config.joint_feedback_enabled
    if feedback and config.type not in JOINT_ENCODER_POLARITY_BY_SIDE:
        logger.warning(
            "closed-loop joint feedback is unavailable for %r hand assemblies; "
            "connect() will refuse to engage the loop. Pass "
            "engage_feedback=False (to load_hand or connect) for open-loop "
            "control, or set 'type:' in config.yaml to one of %s.",
            config.type, sorted(JOINT_ENCODER_POLARITY_BY_SIDE),
        )

    hand_cls = _CLASS_MATRIX[(bool(feedback), tactile, bool(mock))]
    return hand_cls(config=config)
