# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Confirm the hand on the bench is the hand the plan thinks it is.

Two questions, both asked before anything is driven: does the board's own
declaration of what it was built with match what actually answers, and is
exactly the configured set of motors on the bus.

This runs before the hand is connected. The motor scan opens the port itself,
so it must not race a client already holding it.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple, TYPE_CHECKING

from ..hand_factory import HandDetection, _describe_caps, detect_hand
from ..hardware.motor_factory import create_motor_client

if TYPE_CHECKING:
    from ..hand_config import OrcaHandConfig
    from ..hardware.motor_client import MotorClient

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

def configured_id_range(config: "OrcaHandConfig") -> Tuple[int, int]:
    """The inclusive ID span this hand's own motors occupy.

    A motor that was never assigned answers on its family's factory default,
    which sits inside this span, so scanning wider buys only the case of a
    motor assigned *outside* the configured set — a setup typo. A bench
    hunting one passes an explicit wider range.
    """
    ids = config.motor_ids
    return (min(ids), max(ids)) if ids else (1, 1)


class PortsBusyError(RuntimeError):
    """A controller-board CDC is held by another process.

    Infrastructure, not a bad hand: anything behind that port reads as absent
    under probing, so a result taken now would understate the hand.
    """


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the routine."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("identity inventory progress callback failed")


@dataclass(frozen=True)
class IdentityInventoryResult:
    """Who the hand says it is, and what is actually on its motor bus.

    Implements :class:`~orca_core.verification.StepResult`.
    """

    model_name: str
    side: str
    hand_id: str | None
    serial: str | None
    board_id: str | None
    hw_version: int | None
    fw_version: int | None
    declared_config: int | None
    declared_capabilities: str
    detected_capabilities: str
    expected_motor_ids: List[int]
    found_motor_ids: List[int]
    missing_motor_ids: List[int]
    unexpected_motor_ids: List[int]
    duplicate_motor_ids: List[int]
    motor_models: Dict[int, str]
    thresholds: Dict[str, float] = field(default_factory=dict)
    messages: List[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return not self.messages

    def measurements(self) -> Dict[str, float]:
        """Identity is text, so the only scalars are the bus census."""
        return {
            "motors_expected": float(len(self.expected_motor_ids)),
            "motors_found": float(len(self.found_motor_ids)),
            "motors_missing": float(len(self.missing_motor_ids)),
            "motors_unexpected": float(len(self.unexpected_motor_ids)),
        }


def run_identity_inventory(
    config: "OrcaHandConfig",
    *,
    detection: HandDetection | None = None,
    motor_client: "MotorClient | None" = None,
    id_range: Tuple[int, int] | None = None,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> IdentityInventoryResult | None:
    """Read the board's identity and take a census of the motor bus.

    Run this on a **disconnected** hand: the scan opens the motor port itself,
    and a client already holding it would garble both. Returns ``None`` on
    early exit (``should_stop`` triggered).

    Args:
        config: The model config the plan expects this hand to be.
        detection: A :func:`~orca_core.detect_hand` result to score. Defaults
            to probing now.
        motor_client: An unconnected client to scan with. Defaults to one
            built for ``config.motor_type`` on the detected motor port.
        id_range: Inclusive motor-ID range to ping. Defaults to the span this
            hand's configured motors occupy; widen it to hunt a motor assigned
            outside that set.
        progress_callback: Optional ``callable(dict)`` invoked with structured
            progress events (``identity_started``, ``identity_read``,
            ``scan_started``, ``scan_done``, ``identity_aborted``,
            ``identity_done``). Must be fast and non-blocking; exceptions it
            raises are swallowed.
        should_stop: Optional ``callable() -> bool`` polled before the scan.

    Raises:
        PortsBusyError: If another process holds a controller-board CDC, so
            capabilities behind it cannot be detected.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    if id_range is None:
        id_range = configured_id_range(config)

    _emit(progress_callback, "identity_started")
    if detection is None:
        detection = detect_hand()

    if detection.busy_ports:
        raise PortsBusyError(
            f"Port {', '.join(detection.busy_ports)} is held by another "
            f"process. Capabilities behind it cannot be detected. Close other "
            f"software using the hand and re-run."
        )

    identity = detection.identity
    _emit(
        progress_callback,
        "identity_read",
        model_name=detection.model_name,
        side=detection.side,
        hand_id=identity.hand_id if identity else None,
    )

    if should_stop():
        _emit(progress_callback, "identity_aborted")
        return None

    port = detection.motor_port or config.port
    baudrate = config.baudrate
    _emit(progress_callback, "scan_started", port=port, baudrate=baudrate)
    found = _scan(
        config, motor_client=motor_client, port=port, baudrate=baudrate,
        id_range=id_range,
    )
    _emit(progress_callback, "scan_done", found=sorted({e["id"] for e in found}))

    result = _judge(config, detection=detection, found=found)
    _emit(
        progress_callback,
        "identity_done",
        passed=result.passed,
        messages=list(result.messages),
    )
    return result


def _scan(
    config: "OrcaHandConfig",
    *,
    motor_client: "MotorClient | None",
    port: str | None,
    baudrate: int | None,
    id_range: Tuple[int, int],
) -> List[dict]:
    """Ping ``id_range`` at the configured baud, returning the raw scan rows."""
    if motor_client is None:
        if not port or not config.motor_type:
            logger.warning(
                "no motor port or motor type resolved; skipping the bus scan"
            )
            return []
        motor_client = create_motor_client(config.motor_type, [], port, baudrate)
    return motor_client.scan_for_motors(
        port=port, id_range=id_range,
        baud_rates=[baudrate] if baudrate else None,
    )


def _judge(
    config: "OrcaHandConfig", *, detection: HandDetection, found: List[dict]
) -> IdentityInventoryResult:
    """Score the identity and the bus census against what the config expects."""
    identity = detection.identity
    found_ids = [int(entry["id"]) for entry in found]
    unique_found = set(found_ids)
    expected = set(config.motor_ids)
    duplicates = sorted({mid for mid in found_ids if found_ids.count(mid) > 1})

    declared_caps = _describe_caps(detection.has_tactile, detection.has_encoders)
    detected_caps = _describe_caps(
        detection.probed_tactile, detection.probed_encoders
    )

    messages: List[str] = []
    if identity is None or identity.hand_id is None:
        messages.append(
            "The controller board reported no identity, so this hand cannot be "
            "recorded against a serial."
        )
    if detection.missing_capabilities or detection.undeclared_capabilities:
        messages.append(
            f"The board reports config {detection.declared_config} "
            f"({declared_caps}) but only {detected_caps} were detected."
        )

    missing = sorted(expected - unique_found)
    unexpected = sorted(unique_found - expected)
    if missing:
        messages.append(
            f"Expected motor IDs {sorted(expected)}, found {sorted(unique_found)}. "
            f"Missing: {missing}. Motors ship at a factory-default ID and must "
            f"be assigned before they can share a bus (see the motor-chain "
            f"setup step)."
        )
    if unexpected:
        messages.append(
            f"Motor IDs {unexpected} answered but are not in this hand's "
            f"configured set {sorted(expected)}."
        )
    if duplicates:
        messages.append(f"Motor IDs {duplicates} answered more than once.")

    return IdentityInventoryResult(
        model_name=detection.model_name,
        side=detection.side,
        hand_id=identity.hand_id if identity else None,
        serial=identity.serial if identity else None,
        board_id=identity.board_id if identity else None,
        hw_version=identity.hw_version if identity else None,
        fw_version=identity.fw_version if identity else None,
        declared_config=detection.declared_config,
        declared_capabilities=declared_caps,
        detected_capabilities=detected_caps,
        expected_motor_ids=sorted(expected),
        found_motor_ids=sorted(unique_found),
        missing_motor_ids=missing,
        unexpected_motor_ids=unexpected,
        duplicate_motor_ids=duplicates,
        motor_models={
            int(entry["id"]): str(entry.get("model_name", "")) for entry in found
        },
        messages=messages,
    )
