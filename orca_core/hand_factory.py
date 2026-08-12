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
    BoardProbe,
    OrcaBoardInfo,
    _tactile_responds_at,
    detect_encoder_stream,
    find_tactile_port,
    probe_boards,
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



@dataclass(frozen=True)
class HandDetection:
    """What :func:`detect_hand` found plugged in.

    ``model_name`` is the bundled v2 model matching the detected side and
    sensing capabilities; the port fields carry what was discovered so the
    hand can connect without re-probing. ``identity`` is ``None`` for hands
    whose board doesn't report one. ``busy_ports`` lists controller-board
    CDCs another process holds: those stay silent under probing, so anything
    behind them is missing from the rest of this result.
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
    hand_id: Optional[str] = None
    """Stable name for this hand: its assigned serial when provisioned, else
    the board ID. ``None`` only when the board reports neither."""
    board_id: Optional[str] = None
    usb_serial: Optional[str] = None
    side_source: Optional[str] = None
    """Where :attr:`side` came from — ``"board"`` when the hardware said so,
    ``"default"`` when nothing did and right-handed was assumed, ``None``
    when the detection was built by hand and says nothing either way."""


def _detection_from_board(
    board: BoardProbe, tactile_port: Optional[str]
) -> HandDetection:
    """Describe one probed board as the bundled model that matches it."""
    has_encoders = (
        board.sensing_port is not None and detect_encoder_stream(board.sensing_port)
    )

    has_tactile = tactile_port is not None
    if not has_tactile and board.sensing_port is not None:
        has_tactile = _tactile_responds_at(board.sensing_port, DEFAULT_ENCODER_BAUDRATE)

    identity = board.identity
    side = identity.side if identity is not None and identity.side else None
    model_name = _MODEL_BY_CAPS[(has_tactile, has_encoders)].format(
        side=side or "right"
    )

    return HandDetection(
        model_name=model_name,
        side=side or "right",
        has_tactile=has_tactile,
        has_encoders=has_encoders,
        motor_port=board.motor_port,
        sensing_port=board.sensing_port,
        tactile_port=tactile_port,
        identity=identity,
        busy_ports=board.busy_ports,
        hand_id=board.hand_id,
        board_id=board.board_id,
        usb_serial=board.usb_serial,
        side_source="board" if side else "default",
    )


def detect_hands() -> "list[HandDetection]":
    """Describe every hand plugged in, one per controller board.

    Boards are grouped from USB descriptors before anything is opened, so
    each hand's motor and sensing CDCs always come from the same physical
    board — the pairing does not depend on enumeration order, and a port
    another client holds still groups correctly.

    Results are ordered by ``hand_id``, which is stable across replugs
    (``comports()`` order is not). Returns an empty list when nothing is
    attached.
    """
    # The controller board shares its vendor ID with the bare module it is
    # built on, so a board that answered on neither CDC is a spare on the
    # bench, not a hand. One held by another client is silent for that reason
    # alone and stays.
    boards = [
        board for board in probe_boards()
        if board.motor_port or board.sensing_port or board.busy_ports
    ]

    # A dedicated tactile adapter is a separate USB device with nothing tying
    # it to a board, so it can only be attributed when there is one hand.
    adapter_port = find_tactile_port()
    if adapter_port is not None and len(boards) > 1:
        logger.warning(
            "a dedicated tactile adapter is attached (%s) but %d hands are "
            "plugged in, so it cannot be attributed to one of them. Tactile "
            "is reported only for hands whose sensing CDC answers.",
            adapter_port, len(boards),
        )
        adapter_port = None

    # A tactile adapter with no controller board behind it is a pre-identity
    # touch hand, whose motor bus is a plain USB adapter found at connect.
    if not boards:
        if adapter_port is None:
            return []
        return [_detection_from_board(BoardProbe(ports=()), adapter_port)]

    detections = [_detection_from_board(board, adapter_port) for board in boards]
    # Ordered by board ID because it comes from the USB descriptor: a hand
    # that is connected holds its ports and cannot be probed, so anything
    # taken from a reply would reorder the list the moment a hand engages.
    detections.sort(key=lambda d: (d.board_id is None, d.board_id or ""))

    unnamed = [d for d in detections if d.side_source == "default"]
    if len(detections) > 1 and unnamed:
        logger.warning(
            "%d of %d attached hands report no side and default to "
            "right-handed (%s). A hand modelled as the wrong side gets the "
            "wrong joint map and encoder polarities; name its model "
            "explicitly.",
            len(unnamed), len(detections),
            ", ".join(d.hand_id or "unidentified" for d in unnamed),
        )
    return detections


def detect_hand() -> HandDetection:
    """Probe the connected hardware and name the bundled model that matches.

    The hand's side comes from the controller board's identity reply; joint
    encoders are confirmed by a live encoder stream on the sensing CDC and
    tactile by a sensor register reply (on the shared CDC or a dedicated
    adapter). Any question the hardware doesn't answer falls back
    conservatively: no side means right, no reply means the capability is
    absent — so with nothing plugged in this returns the plain right-hand
    model with all ports unset.

    A CDC another process already holds is silent under probing and so reads
    as absent; those ports are reported in ``busy_ports`` so callers can tell
    an incomplete result from a genuinely simpler hand.

    Describes a single hand. Use :func:`detect_hands` when more than one may
    be attached.
    """
    detections = detect_hands()
    if not detections:
        return HandDetection(
            model_name=_MODEL_BY_CAPS[(False, False)].format(side="right"),
            side="right",
            has_tactile=False,
            has_encoders=False,
            side_source="default",
        )
    if len(detections) > 1:
        logger.warning(
            "%d hands are plugged in (%s); describing only %s. Use "
            "detect_hands() to see them all.",
            len(detections),
            ", ".join(d.hand_id or "unidentified" for d in detections),
            detections[0].hand_id or detections[0].model_name,
        )
    return detections[0]


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
        if detection.busy_ports:
            logger.warning(
                "controller-board port(s) %s are held by another process, so "
                "anything behind them went undetected and %r may understate "
                "this hand. Close the other client (a running UI, script or "
                "serial monitor), or name the model explicitly.",
                ", ".join(detection.busy_ports), model_name,
            )

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
        logger.warning(
            "closed-loop joint feedback is unavailable for %r hand assemblies; "
            "connect() will refuse to engage the loop. Pass "
            "engage_feedback=False (to load_hand or connect) for open-loop "
            "control, or set 'type:' in config.yaml to one of %s.",
            config.type, sorted(JOINT_ENCODER_POLARITY_BY_SIDE),
        )

    hand_cls = _CLASS_MATRIX[(bool(feedback), tactile, bool(mock))]
    return hand_cls(config=config)
