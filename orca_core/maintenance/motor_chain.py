# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Assign IDs and baud rates to the motors of a fresh ORCA hand.

Motors ship at a factory-default ID and baud rate. Building a hand means
plugging them in one at a time and re-programming each to its place in the
chain, because two motors at the same default ID cannot share a bus.

Every routine here is interaction-free. Progress is reported through
``progress_callback({"event": ..., ...})`` and human action is requested
through a blocking ``prompt_callback({"action": ..., ...})``, so a terminal,
a GUI, and a web front-end can all drive the same operation.
"""
from __future__ import annotations

import contextlib
import logging
import os
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from ..constants import FINGER, MOTOR_MODELS, SUPPORTED_MOTOR_TYPES, WRIST
from ..hardware.motor_client import MotorClient
from ..hardware.motor_factory import create_motor_client, motor_client_class
from ..utils.utils import auto_detect_port

logger = logging.getLogger(__name__)

# Timings local to chain assembly: USB device-node polling, re-scan for a freshly
# plugged motor, and settle time after a motor's ID is rewritten.
PORT_POLL_INTERVAL_S = 0.3
PORT_SETTLE_S = 0.5
MOTOR_POLL_INTERVAL_S = 1.0
ID_CHANGE_SETTLE_S = 0.5

ProgressCallback = Callable[[dict], None]
PromptCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]


class MotorChainError(RuntimeError):
    """A motor-chain operation could not complete."""


class MotorChainAborted(MotorChainError):
    """The bus holds motors that do not form a valid prefix of the target chain.

    Carries the offending IDs so a caller can tell the user which motors to
    reset to factory defaults.
    """

    def __init__(self, message: str, invalid_ids: list[int]):
        super().__init__(message)
        self.invalid_ids = invalid_ids


@dataclass(frozen=True)
class MotorChainPlan:
    """What the chain should look like once configured.

    Family-specific facts (factory defaults, hot-plug rules) are read from the
    motor client rather than stored here, so a new motor family needs no change
    to this module.
    """

    motor_type: str
    port: str
    target_baud: int
    finger_ids: list[int]
    wrist_id: Optional[int]

    @property
    def client_cls(self) -> type[MotorClient]:
        return motor_client_class(self.motor_type)

    @property
    def default_id(self) -> int:
        return self.client_cls.factory_default_id

    @property
    def default_baud(self) -> int:
        return self.client_cls.factory_default_baudrate

    @property
    def requires_unpowered_hotplug(self) -> bool:
        return self.client_cls.requires_unpowered_hotplug

    @property
    def finger_model(self) -> str:
        return MOTOR_MODELS[self.motor_type][FINGER]

    @property
    def wrist_model(self) -> str:
        return MOTOR_MODELS[self.motor_type][WRIST]

    @property
    def all_target_ids(self) -> list[int]:
        """Configuration order: descending IDs, the order motors daisy-chain from the board."""
        ids = self.finger_ids + ([self.wrist_id] if self.wrist_id is not None else [])
        return sorted(ids, reverse=True)

    @property
    def total_motors(self) -> int:
        return len(self.all_target_ids)

    def model_for(self, target_id: int) -> str:
        """Model expected at ``target_id``: the wrist motor differs from the fingers."""
        if self.wrist_id is not None and target_id == self.wrist_id:
            return self.wrist_model
        return self.finger_model


@dataclass(frozen=True)
class ChainScan:
    """Result of scanning the bus for motors already at their target settings."""

    valid_ids: list[int]
    invalid_ids: list[int]
    motors_by_id: dict[int, dict] = field(default_factory=dict)


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the operation."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("motor-chain progress callback failed")


def _check_stop(should_stop: Optional[ShouldStop]) -> None:
    if should_stop is not None and should_stop():
        raise MotorChainError("motor-chain operation cancelled")


def valid_baudrates(motor_type: str) -> list[int]:
    """Baud rates the given motor family accepts, highest first."""
    return motor_client_class(motor_type).supported_baudrates()


def resolve_port(port: Optional[str], motor_type: Optional[str] = None) -> Optional[str]:
    """Return ``port`` if it exists, else an auto-detected one, else ``None``.

    Never prompts. A front-end that wants to offer the user a choice should
    call this first and fall back to its own picker.
    """
    if port and os.path.exists(port):
        return port
    try:
        detected = auto_detect_port(motor_type)
    except Exception:
        return None
    return detected if detected and os.path.exists(detected) else None


def _deregister_client(client: MotorClient) -> None:
    """Remove a client from its family's exit-time cleanup registry.

    Clients that bypass ``disconnect()`` must never reach the atexit
    torque-disable handler: it would write to a closed (or never-opened) port.
    """
    if hasattr(client, "_connected"):
        client._connected = False  # a stale flag would let disconnect() touch the closed port
    getattr(client, "OPEN_CLIENTS", set()).discard(client)


@contextlib.contextmanager
def _config_session(motor_type: str, motor_ids: list[int], port: str, baudrate: int):
    """Connect a client, yield it, then close the port directly.

    Bypasses the normal disconnect-with-torque-disable: the caller may have
    just changed the motor's ID or baud rate, so the motor no longer answers
    on the settings this client was opened with.
    """
    client = create_motor_client(motor_type, motor_ids, port, baudrate)
    client.connect()
    try:
        yield client
    finally:
        try:
            client.port_handler.closePort()
        except Exception:
            logger.debug("closing port after config session failed", exc_info=True)
        _deregister_client(client)


def scan_motors(motor_type: str, port: str, baudrate: int, id_range: tuple[int, int]) -> list[dict]:
    """Ping ``id_range`` at ``baudrate``. Returns dicts with id, model_name, baud_rate."""
    client = create_motor_client(motor_type, [], port, baudrate)
    try:
        return client.scan_for_motors(port=port, id_range=id_range, baud_rates=[baudrate])
    finally:
        # The scan client never connects; deregister it so polling cannot grow the registry.
        _deregister_client(client)


def detect_motor_type(port: str, progress_callback: Optional[ProgressCallback] = None) -> Optional[str]:
    """Identify the motor family on ``port`` by probing each family's factory defaults.

    Families ship at distinct (ID, baud) pairs, so a motor answering one
    family's defaults identifies the family.
    """
    for motor_type in SUPPORTED_MOTOR_TYPES:
        client_cls = motor_client_class(motor_type)
        baud, default_id = client_cls.factory_default_baudrate, client_cls.factory_default_id
        _emit(progress_callback, "probing_motor_type", motor_type=motor_type,
              motor_id=default_id, baudrate=baud)
        try:
            if scan_motors(motor_type, port, baud, (default_id, default_id)):
                _emit(progress_callback, "motor_type_detected", motor_type=motor_type)
                return motor_type
        except Exception as exc:
            _emit(progress_callback, "probe_failed", motor_type=motor_type, error=str(exc))
    return None


def plan_motor_chain(config: dict, port: str, motor_type: str) -> MotorChainPlan:
    """Build the target chain layout from a hand ``config.yaml`` mapping.

    The wrist motor ID comes from ``joint_to_motor_map`` (sign stripped, as in
    the hand config); the fingers take the remaining IDs. Motors are configured
    from the highest ID down, the order they are daisy-chained from the board.
    """
    motor_ids = config.get("motor_ids") or list(range(17, 0, -1))
    joint_map = config.get("joint_to_motor_map") or {}
    wrist_id = abs(int(joint_map["wrist"])) if "wrist" in joint_map else None
    if wrist_id is not None and wrist_id not in motor_ids:
        raise MotorChainError(f"wrist motor {wrist_id} is not one of motor_ids")
    finger_ids = sorted((mid for mid in motor_ids if mid != wrist_id), reverse=True)
    if not finger_ids:
        raise MotorChainError("config declares no finger motors")
    return MotorChainPlan(
        motor_type=motor_type,
        port=port,
        target_baud=config.get("baudrate") or 1_000_000,
        finger_ids=finger_ids,
        wrist_id=wrist_id,
    )


def wait_for_port(
    port: str,
    *,
    present: bool,
    timeout: Optional[float] = None,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> None:
    """Block until ``port`` appears (``present=True``) or disappears.

    Waits indefinitely by default — the operator may be mid-assembly — and can
    always be cancelled via ``should_stop``. Pass ``timeout`` to raise
    :class:`MotorChainError` instead of waiting forever.
    """
    _emit(progress_callback, "waiting_for_port", port=port, present=present)
    deadline = None if timeout is None else time.monotonic() + timeout
    while os.path.exists(port) != present:
        _check_stop(should_stop)
        if deadline is not None and time.monotonic() > deadline:
            raise MotorChainError(
                f"timed out after {timeout:.0f}s waiting for {port} to "
                f"{'appear' if present else 'disappear'}"
            )
        time.sleep(PORT_POLL_INTERVAL_S)
    if present:
        time.sleep(PORT_SETTLE_S)  # let the OS finish bringing the device up
    _emit(progress_callback, "port_ready", port=port, present=present)


def _prompt(prompt_callback: Optional[PromptCallback], action: str, **payload) -> None:
    if prompt_callback is None:
        raise MotorChainError(
            f"this operation needs to ask the operator to {action!r}, "
            "but no prompt_callback was supplied"
        )
    prompt_callback({"action": action, **payload})


def _await_motor_connection(
    plan: MotorChainPlan,
    message: str,
    prompt_callback: Optional[PromptCallback],
    progress_callback: Optional[ProgressCallback],
    should_stop: Optional[ShouldStop],
    **prompt_payload,
) -> None:
    """Ask the operator to connect the next motor, and wait until they have.

    A hot-pluggable family needs no prompt: the caller has already announced the
    motor through the ``step_started`` event, and simply polls the bus until it
    appears. This keeps chain assembly headless for those families.

    A family that cannot be hot-plugged onto a live bus
    (:attr:`MotorClient.requires_unpowered_hotplug`) is power-cycled around the
    prompt — USB out, motor connected, USB back in — and therefore requires a
    ``prompt_callback``: the sequence cannot proceed unattended.
    """
    if not plan.requires_unpowered_hotplug:
        return

    wait_for_port(plan.port, present=False, progress_callback=progress_callback,
                  should_stop=should_stop)
    _prompt(prompt_callback, "connect_motor", message=message, **prompt_payload)
    wait_for_port(plan.port, present=True, progress_callback=progress_callback,
                  should_stop=should_stop)


def find_default_motor(plan: MotorChainPlan, expected_model: str) -> Optional[dict]:
    """Return the factory-default motor on the bus, or ``None`` if absent.

    Raises :class:`MotorChainError` when a default motor is present but is the
    wrong model — continuing would program a wrist motor into a finger slot.
    """
    motors = scan_motors(plan.motor_type, plan.port, plan.default_baud,
                         (plan.default_id, plan.default_id))
    if not motors:
        return None
    motor = motors[0]
    if expected_model not in motor["model_name"]:
        raise MotorChainError(
            f"wrong motor type on the bus: found {motor['model_name']}, "
            f"expected {expected_model}"
        )
    return motor


def configure_default_motor(
    plan: MotorChainPlan,
    target_id: int,
    progress_callback: Optional[ProgressCallback] = None,
) -> None:
    """Re-program the factory-default motor: baud rate first, then ID.

    Baud-first keeps a failure recoverable: an interruption leaves the motor at
    (default ID, target baud), which the resume prescan reports as invalid and
    ``reset_all_motors`` can revert. ID-first would strand it at a combination
    no scan visits. Afterwards the default ID is probed again: a motor still
    answering there means a second default motor joined the bus mid-step, so
    the step is refused rather than risking a duplicate ``target_id``.
    """
    if plan.target_baud != plan.default_baud:
        with _config_session(plan.motor_type, [plan.default_id], plan.port, plan.default_baud) as client:
            if not client.change_motor_baudrate(plan.default_id, plan.target_baud):
                raise MotorChainError(
                    f"failed to set the factory-default motor's baud to {plan.target_baud}"
                )
        time.sleep(ID_CHANGE_SETTLE_S)

    with _config_session(plan.motor_type, [plan.default_id], plan.port, plan.target_baud) as client:
        if not client.change_motor_id(plan.default_id, target_id):
            raise MotorChainError(f"failed to change motor ID to {target_id}")
    time.sleep(ID_CHANGE_SETTLE_S)

    _refuse_leftover_default_motor(plan, target_id, progress_callback)


def _refuse_leftover_default_motor(
    plan: MotorChainPlan,
    target_id: int,
    progress_callback: Optional[ProgressCallback],
) -> None:
    """Fail the step when a factory-default motor still answers after programming.

    Duplicate motors at the target ID are invisible to ``verify_chain`` (both
    answer as one), so the leftover default motor is the only observable symptom.
    """
    if target_id == plan.default_id:
        return  # the programmed motor legitimately still holds the default ID
    bauds = [plan.default_baud]
    if plan.target_baud != plan.default_baud:
        bauds.append(plan.target_baud)
    for baud in bauds:
        if scan_motors(plan.motor_type, plan.port, baud, (plan.default_id, plan.default_id)):
            _emit(progress_callback, "duplicate_default_motor", target_id=target_id,
                  baudrate=baud)
            raise MotorChainError(
                f"a factory-default motor still answers after programming motor "
                f"{target_id}: a second motor joined the bus during this step "
                f"(likely the next one plugged in early). Leave it connected and "
                f"rerun — the run resumes and configures it as the next motor"
            )


def scan_configured_motors(plan: MotorChainPlan) -> ChainScan:
    """Find motors already at their target ID and baud.

    A resumed build must see a *contiguous* prefix of the chain order (IDs
    descending, each slot holding its expected model). Anything else means a
    motor was mis-programmed, so the IDs are returned as ``invalid_ids``
    rather than silently skipped.
    """
    motors = scan_motors(plan.motor_type, plan.port, plan.target_baud, (1, plan.total_motors))

    # With target_baud == default_baud, a fresh motor at the default ID is
    # indistinguishable from a configured wrist; only a matching model name survives.
    if plan.target_baud == plan.default_baud:
        motors = [
            m for m in motors
            if m["id"] != plan.default_id or plan.wrist_model in m["model_name"]
        ]
    if not motors:
        return ChainScan(valid_ids=[], invalid_ids=[], motors_by_id={})

    motor_by_id = {m["id"]: m for m in motors}
    valid: list[int] = []
    for expected_id in plan.all_target_ids:
        found = motor_by_id.get(expected_id)
        if found is None or plan.model_for(expected_id) not in found["model_name"]:
            break
        valid.append(expected_id)

    invalid = sorted(mid for mid in motor_by_id if mid not in valid)
    return ChainScan(valid_ids=valid, invalid_ids=invalid, motors_by_id=motor_by_id)


def verify_chain(plan: MotorChainPlan, configured_ids: list[int]) -> None:
    """Re-scan and confirm exactly ``configured_ids`` answer at the target baud."""
    if not configured_ids:
        return
    motors = scan_motors(plan.motor_type, plan.port, plan.target_baud, (1, plan.total_motors))
    found = sorted(m["id"] for m in motors)
    expected = sorted(configured_ids)
    if found != expected:
        raise MotorChainError(
            f"motor verification failed: expected {expected}, found {found}. "
            "Likely a duplicate default ID on the bus, or a motor lost power."
        )


def change_motor_baudrate_only(plan: MotorChainPlan, motor_id: int, current_baud: int,
                               new_baud: int) -> bool:
    """Change one motor's baud rate, leaving its ID alone.

    Returns ``False`` when the motor is already at ``new_baud``.
    """
    if current_baud == new_baud:
        return False
    with _config_session(plan.motor_type, [motor_id], plan.port, current_baud) as client:
        if not client.change_motor_baudrate(motor_id, new_baud):
            raise MotorChainError(f"failed to change baud rate for motor {motor_id}")
    return True


def reset_motor_to_factory(plan: MotorChainPlan, motor_id: int, current_baud: int) -> None:
    """Revert one configured motor to its factory ID and baud rate."""
    if current_baud != plan.default_baud:
        with _config_session(plan.motor_type, [motor_id], plan.port, current_baud) as client:
            if not client.change_motor_baudrate(motor_id, plan.default_baud):
                raise MotorChainError(f"failed to change baud rate for motor {motor_id}")
        time.sleep(ID_CHANGE_SETTLE_S)
    with _config_session(plan.motor_type, [motor_id], plan.port, plan.default_baud) as client:
        if not client.change_motor_id(motor_id, plan.default_id):
            raise MotorChainError(f"failed to change ID for motor {motor_id}")


def configure_motor_chain(
    plan: MotorChainPlan,
    progress_callback: Optional[ProgressCallback] = None,
    prompt_callback: Optional[PromptCallback] = None,
    should_stop: Optional[ShouldStop] = None,
    poll_interval: float = MOTOR_POLL_INTERVAL_S,
) -> list[int]:
    """Program every motor in ``plan``, resuming from whatever is already done.

    Emits ``chain_started``, ``prescan_done``, ``step_started``,
    ``awaiting_motor``, ``wrong_motor_detected``, ``motor_found``,
    ``duplicate_default_motor``, ``motor_configured``, ``chain_verified``,
    ``chain_done``. Asks the operator
    to connect each motor via ``prompt_callback`` (required for Feetech, whose
    bus must be unpowered while plugging).

    Returns the configured motor IDs. Raises :class:`MotorChainAborted` when
    the bus already holds mis-programmed motors.
    """
    _emit(progress_callback, "chain_started", motor_type=plan.motor_type, port=plan.port,
          total_motors=plan.total_motors, target_baud=plan.target_baud,
          finger_ids=plan.finger_ids, wrist_id=plan.wrist_id)

    scan = scan_configured_motors(plan)
    if scan.invalid_ids:
        raise MotorChainAborted(
            f"motors {scan.invalid_ids} are not part of a valid chain prefix; reset them "
            f"to factory defaults (ID={plan.default_id}, baud={plan.default_baud}) and retry",
            invalid_ids=scan.invalid_ids,
        )
    _emit(progress_callback, "prescan_done", already_configured=scan.valid_ids,
          motors=[scan.motors_by_id[i] for i in sorted(scan.valid_ids, reverse=True)])

    configured = list(scan.valid_ids)
    remaining = [mid for mid in plan.all_target_ids if mid not in scan.valid_ids]
    if not remaining:
        _emit(progress_callback, "chain_done", configured_ids=configured)
        return configured

    highest_id = max(plan.all_target_ids)
    for offset, target_id in enumerate(remaining):
        _check_stop(should_stop)
        step = len(scan.valid_ids) + offset + 1
        expected_model = plan.model_for(target_id)
        attaches_to = "board" if target_id == highest_id else f"motor {target_id + 1}"
        message = f"Connect a {expected_model} (ID {target_id}) to the {attaches_to}"

        _emit(progress_callback, "step_started", step=step, total=plan.total_motors,
              target_id=target_id, expected_model=expected_model,
              attaches_to=attaches_to, message=message)

        _await_motor_connection(plan, message, prompt_callback, progress_callback,
                                should_stop, target_id=target_id,
                                expected_model=expected_model)

        _emit(progress_callback, "awaiting_motor", target_id=target_id,
              expected_model=expected_model)
        # A wrong-model motor is an operator mistake, not a fatal state: report
        # it once and keep polling so they can swap it without losing the run.
        wrong_model_reported = False
        while True:
            _check_stop(should_stop)
            try:
                motor = find_default_motor(plan, expected_model)
            except MotorChainError as exc:
                if not wrong_model_reported:
                    _emit(progress_callback, "wrong_motor_detected",
                          target_id=target_id, expected_model=expected_model,
                          error=str(exc))
                    wrong_model_reported = True
                time.sleep(poll_interval)
                continue
            if motor is not None:
                break
            wrong_model_reported = False
            time.sleep(poll_interval)

        _emit(progress_callback, "motor_found", target_id=target_id, motor=motor)
        configure_default_motor(plan, target_id, progress_callback=progress_callback)
        configured.append(target_id)
        _emit(progress_callback, "motor_configured", target_id=target_id,
              baudrate=plan.target_baud, configured_ids=list(configured))

        verify_chain(plan, configured)
        _emit(progress_callback, "chain_verified", configured_ids=list(configured))

    _emit(progress_callback, "chain_done", configured_ids=configured)
    return configured


def _each_motor_on_bus(
    plan: MotorChainPlan,
    action: Callable[[dict], None],
    connect_message: str,
    progress_callback: Optional[ProgressCallback],
    prompt_callback: Optional[PromptCallback],
    should_stop: Optional[ShouldStop],
) -> list[dict]:
    """Apply ``action`` to every motor currently on the bus. One pass, no looping.

    Families needing an unpowered bus are power-cycled first, so this pass acts
    on whatever the operator just plugged in. Motors are scanned at the chain's
    target baud. An action returning ``False`` had nothing to do; such motors
    still count as acted on but are not re-announced, so a repeated pass stays
    quiet about motors already in the requested state.
    """
    _await_motor_connection(plan, connect_message, prompt_callback,
                            progress_callback, should_stop)

    motors = scan_motors(plan.motor_type, plan.port, plan.target_baud, (1, plan.total_motors))
    acted: list[dict] = []
    for motor in motors:
        _check_stop(should_stop)
        try:
            changed = action(motor)
            acted.append(motor)
            if changed is not False:
                _emit(progress_callback, "motor_updated", motor=motor)
        except MotorChainError as exc:
            _emit(progress_callback, "motor_update_failed", motor=motor, error=str(exc))
    return acted


def reset_all_motors(
    plan: MotorChainPlan,
    progress_callback: Optional[ProgressCallback] = None,
    prompt_callback: Optional[PromptCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> list[dict]:
    """Reset every motor currently on the bus to factory defaults. One pass."""
    return _each_motor_on_bus(
        plan,
        lambda m: reset_motor_to_factory(plan, m["id"], m["baud_rate"]),
        "Connect the motor(s) you want to reset",
        progress_callback, prompt_callback, should_stop,
    )


def change_all_baudrates(
    plan: MotorChainPlan,
    new_baud: int,
    progress_callback: Optional[ProgressCallback] = None,
    prompt_callback: Optional[PromptCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> list[dict]:
    """Set every motor on the bus to ``new_baud``, leaving IDs alone. One pass."""
    if new_baud not in valid_baudrates(plan.motor_type):
        raise MotorChainError(
            f"invalid baud rate {new_baud} for {plan.motor_type}; "
            f"valid: {valid_baudrates(plan.motor_type)}"
        )
    return _each_motor_on_bus(
        plan,
        lambda m: change_motor_baudrate_only(plan, m["id"], m["baud_rate"], new_baud),
        "Connect the motor(s) whose baudrate you want to change",
        progress_callback, prompt_callback, should_stop,
    )
