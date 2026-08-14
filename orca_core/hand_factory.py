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

from . import hand_store
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
    owned_ports: tuple[str, ...] = ()
    """Ports this process is already driving, usually another hand it
    connected. Silent under probing for that reason, not a foreign client."""
    declared_config: Optional[int] = None
    probed_tactile: bool = False
    probed_encoders: bool = False
    missing_capabilities: tuple[str, ...] = ()
    undeclared_capabilities: tuple[str, ...] = ()
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
    """Describe one probed board as the bundled model that matches it.

    The board's provisioned ``CFG`` states what the hand was built with and
    wins over probing; probing still runs, so declared hardware that isn't
    answering is reported rather than mistaken for a simpler hand.
    """
    probed_encoders = (
        board.sensing_port is not None and detect_encoder_stream(board.sensing_port)
    )

    probed_tactile = tactile_port is not None
    if not probed_tactile and board.sensing_port is not None:
        probed_tactile = _tactile_responds_at(
            board.sensing_port, DEFAULT_ENCODER_BAUDRATE
        )

    identity = board.identity
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
        owned_ports=board.owned_ports,
        declared_config=declared_config,
        probed_tactile=probed_tactile,
        probed_encoders=probed_encoders,
        missing_capabilities=missing,
        undeclared_capabilities=undeclared,
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
        if board.motor_port or board.sensing_port
        or board.busy_ports or board.owned_ports
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


class HandSelectionError(LookupError):
    """Base for the ways picking a hand can fail.

    Carries the detections behind the failure, so a front-end can render its
    own message — naming a CLI flag rather than a Python call — without
    probing the bus a second time and racing whatever connected in between.
    """

    def __init__(
        self,
        message: str,
        detections: "tuple[HandDetection, ...]" = (),
        summary: "str | None" = None,
    ):
        super().__init__(message)
        self.detections = tuple(detections)
        self.summary = summary or message
        """The failure without the advice, so a front-end can append its own.
        The message names a Python call; a CLI has a flag instead."""


class HandNotFoundError(HandSelectionError):
    """Raised when no attached hand matches what the caller asked for.
    :attr:`detections` holds every hand that *is* attached."""


class AmbiguousHandError(HandSelectionError):
    """Raised when more than one attached hand matches, so there is no
    single right answer and picking one could drive the wrong arm.
    :attr:`detections` holds the hands that matched."""


class HandPortUnresolvedError(HandSelectionError):
    """Raised when an attached hand was detected but the bus it must be driven
    through could not be named. Connecting anyway would mean searching the
    whole bus, which with a second hand attached lands on the neighbour.
    :attr:`detections` holds the hand that could not be resolved."""


@dataclass(frozen=True)
class HandSelector:
    """Which hand a caller means, when more than one is attached.

    Every field given must match. ``side`` is the loosest and only tells two
    hands apart when they face opposite ways; ``hand_id`` is exact.
    """

    hand_id: Optional[str] = None
    board_id: Optional[str] = None
    side: Optional[str] = None
    motor_port: Optional[str] = None

    def matches(self, detection: HandDetection) -> bool:
        for field in ("hand_id", "board_id", "side", "motor_port"):
            wanted = getattr(self, field)
            if wanted is not None and getattr(detection, field) != wanted:
                return False
        return True

    def describe(self) -> str:
        given = {
            field: getattr(self, field)
            for field in ("hand_id", "board_id", "side", "motor_port")
            if getattr(self, field) is not None
        }
        if not given:
            return "any hand"
        return ", ".join(f"{k}={v!r}" for k, v in given.items())


def _describe_hand(detection: HandDetection) -> str:
    name = detection.hand_id or "unidentified"
    return f"{name} ({detection.side} {detection.model_name})"


def select_hand(
    detections: "list[HandDetection]", selector: Optional[HandSelector] = None
) -> HandDetection:
    """Pick the one hand a selector names.

    Raises rather than guessing: with two hands attached, returning either
    hands the caller an arm it did not ask for, and ``comports()`` ordering
    is not reproducible across replugs anyway.
    """
    selector = selector or HandSelector()
    matches = [d for d in detections if selector.matches(d)]

    if len(matches) == 1:
        return matches[0]
    if not matches:
        if not detections:
            raise HandNotFoundError(
                "no hand is attached. Check power and the USB cable, or pass "
                "config_path/model_name to work offline."
            )
        raise HandNotFoundError(
            f"no attached hand matches {selector.describe()}. Attached: "
            + "; ".join(_describe_hand(d) for d in detections),
            detections,
        )
    summary = (
        f"{len(matches)} attached hands match {selector.describe()}: "
        + "; ".join(_describe_hand(d) for d in matches)
        + "."
    )
    raise AmbiguousHandError(
        summary + " Name one with load_hand(select=HandSelector(hand_id=...)), "
        "or use load_hands() to get them all.",
        matches,
        summary,
    )


def _pinned_port(
    config_path: str | None, model_version: str | None, model_name: str | None
) -> Optional[str]:
    """The motor port a named config pins, or ``None`` for ``auto``.

    Only consulted when the caller named a config: with no model named the
    model comes from detection, so there is no configured port to honour.
    """
    if config_path is None and model_version is None and model_name is None:
        return None
    resolved = _resolve_config_path(
        config_path, model_version=model_version, model_name=model_name
    )
    port = (read_yaml(resolved) or {}).get("port")
    return port if port and port != "auto" else None


def _unresolved_reason(detection: HandDetection) -> str:
    """Why a detected hand stayed silent on a CDC, and what to do about it."""
    if detection.busy_ports:
        return (
            f"{', '.join(detection.busy_ports)} is held by another program; "
            "close it and retry"
        )
    if detection.owned_ports:
        return (
            f"{', '.join(detection.owned_ports)} is already connected in this "
            "process; disconnect that hand first"
        )
    return "the board did not answer; check its power and USB cable"


def _pin_detected_ports(config, detection: HandDetection, attached: int):
    """Point the config at the ports detection already found, so connect()
    doesn't have to re-discover them.

    A port detection could not name is left at its configured ``auto`` while
    one hand is attached, where searching the bus can only find that hand
    again and recovers a probe the board missed. With a neighbour attached the
    same search returns *its* bus, so the field is refused or disabled instead.
    """
    board_seen = bool(
        detection.sensing_port or detection.busy_ports or detection.owned_ports
    )
    if config.port != "auto":
        # Pinned by whoever wrote the config. Normally it is also the selector
        # that chose this hand, so the two agree; when a caller's own selector
        # picked a different hand, driving either one crosses the pair.
        if detection.motor_port is not None and detection.motor_port != config.port:
            raise ValueError(
                f"config pins port {config.port!r} but the selected hand "
                f"{_describe_hand(detection)} is on {detection.motor_port!r}. "
                f"Driving that hand through the pinned port would run one "
                f"hand's motors under the other's calibration; drop the pin or "
                f"select the hand it belongs to."
            )
    elif detection.motor_port is not None:
        config = dataclasses.replace(config, port=detection.motor_port)
    elif board_seen and attached > 1:
        # No "disabled" sentinel exists for the motor bus, and no workflow
        # survives an unknown one, so refusing is the only fail-safe outcome.
        raise HandPortUnresolvedError(
            f"hand {_describe_hand(detection)} is attached but its motor bus "
            f"could not be resolved: {_unresolved_reason(detection)}. Refusing "
            f"to search the bus for it — with {attached} hands attached, that "
            f"search returns the neighbour's motor bus.",
            (detection,),
        )
    elif board_seen:
        logger.info(
            "hand %s did not name its motor bus (%s); resolving it at connect, "
            "which is unambiguous while it is the only hand attached.",
            detection.hand_id or "unidentified", _unresolved_reason(detection),
        )

    wants_tactile = detection.has_tactile and isinstance(config, OrcaHandTouchConfig)
    sensor_port = detection.tactile_port or detection.sensing_port
    if detection.has_encoders and detection.sensing_port is not None:
        config = dataclasses.replace(
            config, encoder_serial_port=detection.sensing_port
        )
    if wants_tactile and sensor_port is not None:
        config = dataclasses.replace(config, sensor_port=sensor_port)

    unresolved = [
        name
        for name, declared, port in (
            ("joint encoders", detection.has_encoders, detection.sensing_port),
            ("tactile", wants_tactile, sensor_port),
        )
        if declared and port is None
    ]
    if unresolved and attached > 1:
        # Disabled rather than refused: open-loop maintenance on a hand whose
        # sensing link is unreachable is still worth allowing.
        if detection.has_encoders and detection.sensing_port is None:
            config = dataclasses.replace(config, encoder_serial_port="disabled")
        if wants_tactile and sensor_port is None:
            config = dataclasses.replace(config, sensor_port="disabled")
        logger.warning(
            "hand %s declares %s but its sensing port could not be resolved: "
            "%s. That link is disabled rather than auto-discovered, which with "
            "another hand attached would open the neighbour's sensing CDC; "
            "connect() fails on it until the port is free.",
            detection.hand_id or "unidentified", " and ".join(unresolved),
            _unresolved_reason(detection),
        )
    return config


def _bind_hand_store(config, detection: HandDetection):
    """Point a detected hand's calibration at its own directory.

    Two hands of one model resolve to the same packaged model, so without
    this the second to calibrate overwrites the first. An explicit
    ``calibration_path`` always wins over the store.
    """
    identity = detection.identity
    hand_store.record_identity(detection.hand_id, {
        "serial": identity.serial if identity is not None else None,
        "board_id": detection.board_id,
        "usb_serial": detection.usb_serial,
        "side": detection.side,
        "model": detection.model_name,
        "declared_config": detection.declared_config,
    })
    return dataclasses.replace(
        config,
        calibration_path=hand_store.resolve_calibration_path(
            detection.hand_id, config.calibration_path, board_id=detection.board_id
        ),
    )


def load_hands(
    mock: bool = False,
    engage_feedback: bool = True,
    engage_sensors: bool = True,
    select: Optional[HandSelector] = None,
) -> "list[OrcaHand]":
    """Construct every attached hand, one per controller board.

    Each hand gets its own model, its own ports, and its own calibration, so
    several can be driven from one program. Ordered as :func:`detect_hands`
    orders them, which is stable across replugs. Returns an empty list when
    nothing is attached; pass ``select`` to build only the hands that match.

    A hand that cannot name the bus it must be driven through is skipped with
    a warning rather than failing the call: this returns the hands it can
    build, and one arm being held by another program is no reason to withhold
    the other. Use :func:`load_hand` to ask for one hand and be told why it
    is unavailable.

    Args:
        select: Restrict to the hands a selector matches. Unlike
            :func:`load_hand` this never raises on several matches — that is
            the point — but it does raise :class:`HandNotFoundError` when a
            selector matches nothing.
    """
    detections = detect_hands()
    # Counted before ``select`` narrows the list: what a port search can stray
    # onto is what is attached, not what this caller asked for.
    attached = len(detections)
    if select is not None:
        matched = [d for d in detections if select.matches(d)]
        if not matched:
            # Reuse the one place that phrases this well.
            select_hand(detections, select)
        detections = matched

    hands = []
    for detection in detections:
        try:
            hands.append(_build_hand(
                detection,
                config_path=None,
                calibration_path=None,
                model_version=None,
                model_name=None,
                mock=mock,
                engage_feedback=engage_feedback,
                engage_sensors=engage_sensors,
                attached=attached,
            ))
        except HandPortUnresolvedError as e:
            logger.warning("%s Skipping it; the other hands still load.", e)
    return hands


def load_hand(
    config_path: str | None = None,
    calibration_path: str | None = None,
    model_version: str | None = None,
    model_name: str | None = None,
    mock: bool = False,
    engage_feedback: bool = True,
    engage_sensors: bool = True,
    select: Optional[HandSelector] = None,
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
        select: Which hand to load when more than one is attached. Without
            it, two attached hands raise :class:`AmbiguousHandError` rather
            than picking one — with two arms on the bench, guessing drives
            the wrong one.

    Returns:
        A constructed (not yet connected) hand instance.

    Raises:
        AmbiguousHandError: several attached hands match, so there is no
            single right answer. Name one, or use :func:`load_hands`.
        HandNotFoundError: ``select`` matches nothing attached.
        HandPortUnresolvedError: the chosen hand is attached but did not name
            its motor bus, and another hand is attached for a port search to
            stray onto.
    """
    if select is not None and mock:
        raise ValueError(
            "select= names a physical hand to detect; mock=True has none. Drop "
            "one of the two."
        )

    detection = None
    attached = 0
    if not mock:
        # A config that pins a port names a hand as surely as a selector does,
        # so it picks the hand whose bus that is — otherwise the store and the
        # calibration could come from a different hand than the motors.
        if select is None:
            pinned = _pinned_port(config_path, model_version, model_name)
            if pinned is not None:
                select = HandSelector(motor_port=pinned)
        detections = detect_hands()
        attached = len(detections)
        if detections or select is not None:
            # A selector that matches nothing is an error; asking for no
            # particular hand with none attached is not, and still yields the
            # bundled default so a config can be inspected off the bench.
            detection = select_hand(detections, select)
    return _build_hand(
        detection,
        config_path=config_path,
        calibration_path=calibration_path,
        model_version=model_version,
        model_name=model_name,
        mock=mock,
        engage_feedback=engage_feedback,
        engage_sensors=engage_sensors,
        attached=attached,
    )


def _build_hand(
    detection: Optional[HandDetection],
    *,
    config_path: str | None,
    calibration_path: str | None,
    model_version: str | None,
    model_name: str | None,
    mock: bool,
    engage_feedback: bool,
    engage_sensors: bool,
    attached: int = 0,
) -> OrcaHand:
    """Construct the hand class a resolved detection (or an explicit model)
    calls for. Detection has already happened; nothing here probes.

    ``attached`` is how many hands that detection found, which decides whether
    a port it could not name may still be searched for at connect."""
    named_model = (
        config_path is not None or model_name is not None or model_version is not None
    )
    # Detection says *which hand* this is; it only says which model when the
    # caller named none. The warnings below all read "may understate this
    # hand", which means nothing once a model is pinned.
    if detection is not None and not named_model:
        model_name = detection.model_name
        if detection.busy_ports:
            logger.warning(
                "controller-board port(s) %s are held by another process, so "
                "anything behind them went undetected and %r may understate "
                "this hand. Close the other client (a running UI, script or "
                "serial monitor), or name the model explicitly.",
                ", ".join(detection.busy_ports), model_name,
            )
        if detection.owned_ports:
            logger.warning(
                "controller-board port(s) %s are already connected in this "
                "process, so they could not be probed and %r may understate "
                "this hand. Detect before connecting, or name the model "
                "explicitly.",
                ", ".join(detection.owned_ports), model_name,
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
        config = _pin_detected_ports(config, detection, attached)
        if calibration_path is None and detection.hand_id:
            config = _bind_hand_store(config, detection)

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
