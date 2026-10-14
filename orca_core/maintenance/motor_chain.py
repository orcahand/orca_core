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
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from ..constants import (
    FINGER,
    MOTOR_BAUD_RATES,
    MOTOR_MODELS,
    SUPPORTED_MOTOR_TYPES,
    WRIST,
)
from ..hardware.motor_client import MotorClient
from ..hardware.motor_factory import create_motor_client, motor_client_class
from ..utils.utils import auto_detect_port, serial_port_exists

logger = logging.getLogger(__name__)

# Timings local to chain assembly: USB device-node polling, re-scan for a freshly
# plugged motor, and settle time after a motor's ID is rewritten.
PORT_POLL_INTERVAL_S = 0.3
PORT_SETTLE_S = 0.5
MOTOR_POLL_INTERVAL_S = 1.0
ID_CHANGE_SETTLE_S = 0.5
# While waiting for a motor the bus is surveyed for motors that are not at the
# factory defaults, in tiers that widen with the time spent waiting: 1 = the
# chain's IDs at the chain's rate (the default rate is polled anyway); 2, after
# OTHER_RATES_AFTER_S = also the chain's IDs at every other rate the driver can
# set; 3, after ESCALATE_AFTER_S = also every ID up to SWEEP_ID_RANGE at the
# chain's rate. Hands only ever use the factory and chain rates, so the others
# are a last resort. Liveness is reported every HEARTBEAT_INTERVAL_S. Sweeps
# stop at ID 40: a motor from any hand carries a lower ID, and each absent ID
# costs a reply timeout.
CENSUS_EVERY_POLLS = 5
OTHER_RATES_AFTER_S = 30.0
ESCALATE_AFTER_S = 60.0
SWEEP_ID_RANGE = (1, 40)
HEARTBEAT_INTERVAL_S = 10.0
# Bus errors (a serial hiccup, a port the OS held) are retried this many times
# in a row before the run gives up on them.
BUS_ERROR_ATTEMPTS = 3
# EEPROM writes are confirmed by pinging the new setting; scans are repeated
# before a mismatch counts, so one dropped reply does not end a run.
WRITE_ATTEMPTS = 3
VERIFY_ATTEMPTS = 3

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

    def __init__(self, message: str, invalid_ids: list[int],
                 invalid_motors: Optional[list[dict]] = None):
        super().__init__(message)
        self.invalid_ids = invalid_ids
        # The offending motors as scanned (id, model_name, baud_rate), so a
        # front-end can reset them without scanning again.
        self.invalid_motors = list(invalid_motors or [])


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

    def __post_init__(self):
        if self.target_baud not in valid_baudrates(self.motor_type):
            raise MotorChainError(
                f"invalid baud rate {self.target_baud} for {self.motor_type}; "
                f"valid: {valid_baudrates(self.motor_type)}"
            )

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
    def return_delay_time_us(self) -> Optional[int]:
        return self.client_cls.return_delay_time_us

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
    if serial_port_exists(port):
        return port
    try:
        detected = auto_detect_port(motor_type)
    except Exception:
        return None
    return detected if serial_port_exists(detected) else None


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


def detect_motor_type(
    port: str,
    progress_callback: Optional[ProgressCallback] = None,
    chain_config: Optional[dict] = None,
) -> Optional[str]:
    """Identify the motor family on ``port``.

    Families ship at distinct (ID, baud) pairs, so a motor answering one
    family's factory defaults identifies the family. When nothing does and
    ``chain_config`` (a hand ``config.yaml`` mapping) is given, the chain that
    config describes is probed instead — its ``motor_ids`` at its pinned
    ``baudrate`` or the family's chain rates — so a resumed build, whose
    motors have all moved off the defaults, still finds its family.
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
    if chain_config is None:
        return None
    motor_ids = chain_config.get("motor_ids") or list(range(1, 18))
    id_range = (min(motor_ids), max(motor_ids))
    pinned = chain_config.get("baudrate")
    for motor_type in SUPPORTED_MOTOR_TYPES:
        rates = [pinned] if pinned in valid_baudrates(motor_type) else MOTOR_BAUD_RATES[motor_type]
        for baud in rates:
            _emit(progress_callback, "probing_motor_type", motor_type=motor_type,
                  motor_id=id_range, baudrate=baud)
            try:
                if scan_motors(motor_type, port, baud, id_range):
                    _emit(progress_callback, "motor_type_detected", motor_type=motor_type,
                          configured=True)
                    return motor_type
            except Exception as exc:
                _emit(progress_callback, "probe_failed", motor_type=motor_type, error=str(exc))
    return None


def plan_motor_chain(config: dict, port: str, motor_type: str) -> MotorChainPlan:
    """Build the target chain layout from a hand ``config.yaml`` mapping.

    The wrist motor ID comes from ``joint_to_motor_map`` (sign stripped, as in
    the hand config); the fingers take the remaining IDs. Motors are configured
    from the highest ID down, the order they are daisy-chained from the board.
    An unpinned ``baudrate`` falls back to the family's preferred rate.
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
        target_baud=config.get("baudrate") or MOTOR_BAUD_RATES[motor_type][0],
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
    while serial_port_exists(port) != present:
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


def _model_is_unrecognised(plan: MotorChainPlan, motor: dict) -> bool:
    """Whether the client reported the bare family label or "Unknown(n)",
    meaning the model number is outside its lookup table."""
    model_name = str(motor.get("model_name", "")).strip().lower()
    return model_name == plan.motor_type.lower() or model_name.startswith("unknown")


def _model_matches(plan: MotorChainPlan, motor: dict, expected_model: str) -> bool:
    """Whether ``motor`` is the expected model, treating unknown as a match."""
    return _model_is_unrecognised(plan, motor) or expected_model in motor["model_name"]


def find_default_motor(
    plan: MotorChainPlan,
    expected_model: Optional[str],
    progress_callback: Optional[ProgressCallback] = None,
    baud: Optional[int] = None,
) -> Optional[dict]:
    """Return the motor at the factory-default ID on the bus, or ``None``.

    Probes at the factory-default baud unless ``baud`` says otherwise (the
    target baud finds a motor whose programming stopped after the baud write).
    Raises :class:`MotorChainError` when a default motor is present but is a
    known wrong model — continuing would program a wrist motor into a finger
    slot. ``expected_model=None`` skips the check entirely.
    """
    motors = scan_motors(plan.motor_type, plan.port,
                         plan.default_baud if baud is None else baud,
                         (plan.default_id, plan.default_id))
    if not motors:
        return None
    motor = motors[0]
    if expected_model is None:
        return motor
    if _model_is_unrecognised(plan, motor):
        _emit(progress_callback, "unrecognised_motor_model",
              expected_model=expected_model,
              model_number=motor.get("model_number"),
              model_name=motor["model_name"])
        return motor
    if expected_model not in motor["model_name"]:
        raise MotorChainError(
            f"wrong motor type on the bus: found {motor['model_name']}, "
            f"expected {expected_model}"
        )
    return motor


def _default_answers(plan: MotorChainPlan, baud: int) -> bool:
    return bool(scan_motors(plan.motor_type, plan.port, baud, (plan.default_id, plan.default_id)))


def _id_answers(plan: MotorChainPlan, motor_id: int, baud: int) -> bool:
    return bool(scan_motors(plan.motor_type, plan.port, baud, (motor_id, motor_id)))


def configure_default_motor(
    plan: MotorChainPlan,
    target_id: int,
    progress_callback: Optional[ProgressCallback] = None,
    current_baud: Optional[int] = None,
) -> None:
    """Re-program the motor at the factory-default ID: baud rate, return delay, then ID.

    ``current_baud`` is where the motor answers now: the factory default for a
    fresh motor, the target baud for one whose earlier programming stopped after
    the baud write (the baud step is then skipped).

    Baud-first keeps a failure recoverable: an interruption leaves the motor at
    (default ID, target baud), which the chain's bus survey adopts and finishes.
    ID-first would strand it at a combination no scan visits. The return delay
    (for families that set one) goes just before the ID, so a motor that reaches
    its target ID always carries it.

    Every EEPROM write is confirmed by pinging the new setting and repeated up to
    ``WRITE_ATTEMPTS`` times (``write_retry``): a write the motor acknowledged but
    did not take is retried, not mistaken for a second motor. A factory-default
    motor still answering at the default baud afterwards *is* a second motor that
    joined mid-step (``duplicate_default_motor``), and the step is refused rather
    than risking a duplicate ``target_id``.
    """
    current = plan.default_baud if current_baud is None else current_baud
    if current != plan.target_baud:
        for attempt in range(1, WRITE_ATTEMPTS + 1):
            with _config_session(plan.motor_type, [plan.default_id], plan.port, current) as client:
                acknowledged = client.change_motor_baudrate(plan.default_id, plan.target_baud)
            time.sleep(ID_CHANGE_SETTLE_S)
            if _default_answers(plan, plan.target_baud):
                break
            if attempt == WRITE_ATTEMPTS:
                raise MotorChainError(
                    f"the factory-default motor did not take the baud rate "
                    f"{plan.target_baud} after {WRITE_ATTEMPTS} attempts (it "
                    f"{'acknowledged' if acknowledged else 'did not acknowledge'} the "
                    "last write); check its connector and power, then rerun"
                )
            _emit(progress_callback, "write_retry", target_id=target_id, setting="baud",
                  attempt=attempt, acknowledged=bool(acknowledged))
            if not _default_answers(plan, current):
                # Not at the old rate either: give it a moment, it may be rebooting.
                time.sleep(ID_CHANGE_SETTLE_S)

    for attempt in range(1, WRITE_ATTEMPTS + 1):
        with _config_session(plan.motor_type, [plan.default_id], plan.port, plan.target_baud) as client:
            delay_us = plan.return_delay_time_us
            if delay_us is not None and not client.change_return_delay_time(plan.default_id, delay_us):
                raise MotorChainError(
                    f"failed to set the factory-default motor's return delay to {delay_us} us"
                )
            acknowledged = client.change_motor_id(plan.default_id, target_id)
        time.sleep(ID_CHANGE_SETTLE_S)
        if target_id == plan.default_id:
            return  # the programmed motor legitimately still holds the default ID
        _refuse_leftover_default_motor(plan, target_id, progress_callback)
        target_answers = _id_answers(plan, target_id, plan.target_baud)
        default_answers = _default_answers(plan, plan.target_baud)
        if target_answers and not default_answers:
            return
        if target_answers and default_answers:
            _emit(progress_callback, "duplicate_default_motor", target_id=target_id,
                  baudrate=plan.target_baud)
            raise MotorChainError(
                f"a factory-default motor still answers after programming motor "
                f"{target_id}: a second motor joined the bus during this step "
                f"(likely the next one plugged in early). Leave it connected and "
                f"rerun — the run resumes and configures it as the next motor"
            )
        if attempt == WRITE_ATTEMPTS:
            if default_answers:
                raise MotorChainError(
                    f"the motor acknowledged ID {target_id} but kept answering as ID "
                    f"{plan.default_id} after {WRITE_ATTEMPTS} attempts; it may be "
                    "write-protected or faulty. Swap it and rerun"
                )
            raise MotorChainError(
                f"the motor stopped answering after the ID write to {target_id} "
                f"({WRITE_ATTEMPTS} attempts); check its connector and power, then "
                "rerun — the run resumes from the bus as it is"
            )
        _emit(progress_callback, "write_retry", target_id=target_id, setting="id",
              attempt=attempt, acknowledged=bool(acknowledged),
              still_default=default_answers)
        if not default_answers:
            time.sleep(ID_CHANGE_SETTLE_S)  # silent for the moment; let it come back


def _refuse_leftover_default_motor(
    plan: MotorChainPlan,
    target_id: int,
    progress_callback: Optional[ProgressCallback],
) -> None:
    """Fail the step when a factory-default motor answers at the factory baud
    after the ID write.

    The programmed motor was moved to the target baud first, so whatever still
    answers at the factory baud is a second motor. Duplicates at the target ID
    are invisible to ``verify_chain`` (both answer as one), so this is the only
    observable symptom.
    """
    if plan.target_baud == plan.default_baud or not _default_answers(plan, plan.default_baud):
        return
    _emit(progress_callback, "duplicate_default_motor", target_id=target_id,
          baudrate=plan.default_baud)
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
            if m["id"] != plan.default_id or _model_matches(plan, m, plan.wrist_model)
        ]
    if not motors:
        return ChainScan(valid_ids=[], invalid_ids=[], motors_by_id={})

    motor_by_id = {m["id"]: m for m in motors}
    valid: list[int] = []
    for expected_id in plan.all_target_ids:
        found = motor_by_id.get(expected_id)
        if found is None or not _model_matches(plan, found, plan.model_for(expected_id)):
            break
        valid.append(expected_id)

    invalid = sorted(mid for mid in motor_by_id if mid not in valid)
    return ChainScan(valid_ids=valid, invalid_ids=invalid, motors_by_id=motor_by_id)


def verify_chain(
    plan: MotorChainPlan,
    configured_ids: list[int],
    progress_callback: Optional[ProgressCallback] = None,
    tolerated_ids: "set[int] | frozenset[int]" = frozenset(),
) -> None:
    """Re-scan and confirm exactly ``configured_ids`` answer at the target baud.

    ``tolerated_ids`` may answer as well: motors this run validated earlier
    that sit behind a link which has just been repaired. A mismatch is
    re-scanned up to ``VERIFY_ATTEMPTS`` times (``verify_retry``) before it
    counts: one dropped reply must not end a run.
    """
    if not configured_ids:
        return
    expected = sorted(configured_ids)
    for attempt in range(1, VERIFY_ATTEMPTS + 1):
        motors = scan_motors(plan.motor_type, plan.port, plan.target_baud, (1, plan.total_motors))
        found = sorted(m["id"] for m in motors if m["id"] in expected or m["id"] not in tolerated_ids)
        if found == expected:
            return
        missing = [i for i in expected if i not in found]
        extra = [i for i in found if i not in expected]
        if attempt < VERIFY_ATTEMPTS:
            _emit(progress_callback, "verify_retry", attempt=attempt, missing=missing, extra=extra)
            time.sleep(ID_CHANGE_SETTLE_S)
    raise MotorChainError(
        f"motor verification failed after {VERIFY_ATTEMPTS} scans: expected {expected}, "
        f"found {found}." + (f" Not answering: {missing} — check their connectors and power."
                            if missing else "")
        + (f" Unexpected: {extra} — a motor not part of this chain is on the bus."
           if extra else "")
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


def set_return_delay_only(plan: MotorChainPlan, motor_id: int, baud: int) -> None:
    """Write the family's Return Delay Time into one configured motor."""
    delay_us = plan.return_delay_time_us
    with _config_session(plan.motor_type, [motor_id], plan.port, baud) as client:
        if not client.change_return_delay_time(motor_id, delay_us):
            raise MotorChainError(f"failed to set the return delay of motor {motor_id}")


def reset_motor_to_factory(plan: MotorChainPlan, motor_id: int, current_baud: int) -> bool:
    """Revert the motor(s) answering as ``motor_id`` at ``current_baud`` to the
    factory ID and baud rate. Returns ``False`` when already there.

    Each write is judged by what the bus says afterwards, not by the
    acknowledgement: several motors sharing an ID all take a write addressed
    to it while their replies collide, so a "failed" write may well have
    landed. A motor that still answers where it was did not take it.
    """
    if motor_id == plan.default_id and current_baud == plan.default_baud:
        return False
    if current_baud != plan.default_baud:
        with _config_session(plan.motor_type, [motor_id], plan.port, current_baud) as client:
            acked = client.change_motor_baudrate(motor_id, plan.default_baud)
        time.sleep(ID_CHANGE_SETTLE_S)
        if _id_answers(plan, motor_id, current_baud):
            raise MotorChainError(
                f"motor {motor_id} did not take the baud rate {plan.default_baud}"
                + ("" if acked else " (and did not acknowledge the write)"))
    if motor_id != plan.default_id:
        with _config_session(plan.motor_type, [motor_id], plan.port, plan.default_baud) as client:
            acked = client.change_motor_id(motor_id, plan.default_id)
        time.sleep(ID_CHANGE_SETTLE_S)
        if _id_answers(plan, motor_id, plan.default_baud):
            raise MotorChainError(
                f"motor {motor_id} did not take ID {plan.default_id}"
                + ("" if acked else " (and did not acknowledge the write)"))
    return True


def _reset_stray(plan: MotorChainPlan, motor: dict,
                 progress_callback: Optional[ProgressCallback]) -> bool:
    """Revert a motor that is on the bus but not part of the chain to the
    factory defaults, so the next poll finds and programs it. Failures are
    reported, not raised: the operator can still unplug it."""
    found = dict(motor)  # as scanned; the bus entry itself changes under the reset
    try:
        reset_motor_to_factory(plan, found["id"], found["baud_rate"])
    except MotorChainError as exc:
        _emit(progress_callback, "stray_motor_reset_failed", motor=found, error=str(exc))
        return False
    time.sleep(ID_CHANGE_SETTLE_S)
    _emit(progress_callback, "stray_motor_reset", motor=found)
    return True


def _bauds_to_survey(plan: MotorChainPlan, state: dict) -> list[int]:
    """The family's other baud rates worth pinging on this bus: those the
    client's serial driver can set, minus rates the bus has shown to mirror.
    A USB bridge that does not honour a rate leaves the bus where it was, so a
    scan "at" that rate answers with the chain's own motors (``phantom_baud``);
    such rates are remembered for the run in ``state``.
    """
    return [b for b in plan.client_cls.scannable_baudrates()
            if b not in (plan.default_baud, plan.target_baud) and b not in state["skip_bauds"]]


def _is_phantom(plan: MotorChainPlan, motor: dict, at_target: list[dict]) -> bool:
    """A motor "at" another baud whose ID also answers at the chain's baud is
    the same motor seen through a bridge that did not switch rate.

    A genuine stray that happens to carry a configured motor's ID reads the
    same way and is left alone: resetting a mirrored chain motor would wreck
    the chain, leaving a stray only delays it until the operator unplugs or
    resets it (the heartbeat says what was checked).
    """
    return any(m["id"] == motor["id"] for m in at_target) or (
        motor["id"] == plan.default_id and _default_answers(plan, plan.default_baud))


def _census(plan: MotorChainPlan, configured: list[int], target_id: int,
            progress_callback: Optional[ProgressCallback], tier: int = 1,
            state: Optional[dict] = None) -> dict:
    """Survey the bus for anything that is not the chain so far.

    A motor at the target baud with an ID the chain has not configured, and any
    motor at another baud, is a stray: reset to factory defaults in place so the
    poll finds and programs it — one per survey, so two strays never meet at
    the default ID. Two exceptions: the motor at the factory-default ID at the
    default or target baud is left to the poll, and a right-model motor already
    carrying ``target_id`` — the slot being waited for — is one that came back
    (reconnected after it dropped off), reported as ``returned`` so the caller
    re-reads the chain. A motor "at" another baud whose ID also answers at the
    chain's baud is the chain seen through a bridge that ignored that rate; the
    rate is reported (``phantom_baud``) and skipped for the rest of the run.
    ``tier`` widens the survey (see the tier constants). Returns the IDs seen at
    the target baud, the configured IDs that did not answer, and what was
    checked.
    """
    state = {"skip_bauds": set()} if state is None else state
    chain_ids = (1, plan.total_motors)
    at_target = scan_motors(plan.motor_type, plan.port, plan.target_baud, chain_ids)
    checked = [f"IDs 1-{plan.total_motors} @ {plan.target_baud}"]
    if tier >= 3:
        at_target += scan_motors(plan.motor_type, plan.port, plan.target_baud,
                                 (plan.total_motors + 1, SWEEP_ID_RANGE[1]))
        checked[0] = f"IDs 1-{SWEEP_ID_RANGE[1]} @ {plan.target_baud}"
    present = sorted(m["id"] for m in at_target)
    returned = False
    strays: list[dict] = []
    for m in at_target:
        if m["id"] == plan.default_id or m["id"] in configured:
            continue
        if m["id"] == target_id and _model_matches(plan, m, plan.model_for(target_id)):
            returned = True
            continue
        strays.append(m)
    other_bauds = _bauds_to_survey(plan, state) if tier >= 2 else []
    id_range = chain_ids
    for baud in other_bauds:
        found = scan_motors(plan.motor_type, plan.port, baud, id_range)
        phantoms = [m for m in found if _is_phantom(plan, m, at_target)]
        if phantoms:
            state["skip_bauds"].add(baud)
            _emit(progress_callback, "phantom_baud", baudrate=baud,
                  motor_ids=[m["id"] for m in phantoms])
            continue
        strays.extend(found)
    if strays and not returned:
        _reset_stray(plan, strays[0], progress_callback)
    if other_bauds:
        checked.append(f"IDs 1-{plan.total_motors} @ {len(other_bauds)} other baud rate(s)")
    return {"present": present, "returned": returned, "checked": checked,
            "missing": [i for i in configured if i not in present]}


def _census_tier(waited_s: float) -> int:
    if waited_s >= ESCALATE_AFTER_S:
        return 3
    return 2 if waited_s >= OTHER_RATES_AFTER_S else 1


def survey_bus(port: str, progress_callback: Optional[ProgressCallback] = None,
               id_range: tuple[int, int] = SWEEP_ID_RANGE) -> list[dict]:
    """Everything on ``port``: every family, every scannable baud, every ID in
    ``id_range``. A diagnostic for a bus that does not behave. Emits
    ``survey_progress`` per (family, baud) with what answered. Returns dicts
    with ``motor_type``, ``id``, ``model_name``, ``baud_rate``.
    """
    found: list[dict] = []
    for motor_type in SUPPORTED_MOTOR_TYPES:
        for baud in motor_client_class(motor_type).scannable_baudrates():
            try:
                motors = scan_motors(motor_type, port, baud, id_range)
            except Exception as exc:
                _emit(progress_callback, "probe_failed", motor_type=motor_type, error=str(exc))
                continue
            for m in motors:
                found.append({"motor_type": motor_type, **m})
            _emit(progress_callback, "survey_progress", motor_type=motor_type, baudrate=baud,
                  motors=motors)
    return found


def _wait_for_default_motor(
    plan: MotorChainPlan,
    target_id: int,
    expected_model: str,
    configured: list[int],
    progress_callback: Optional[ProgressCallback],
    should_stop: Optional[ShouldStop],
    poll_interval: float,
    state: Optional[dict] = None,
) -> Optional[tuple[dict, int]]:
    """Poll until a motor at the factory-default ID is on the bus.

    Returns ``(motor, baud)``: the factory baud for a fresh motor, the target
    baud for one adopted mid-programming (``half_programmed_motor_adopted``).
    Every ``CENSUS_EVERY_POLLS`` empty polls the bus is surveyed and strays are
    reset, in tiers that widen with the time waited (other rates after
    ``OTHER_RATES_AFTER_S``, every ID after ``ESCALATE_AFTER_S``);
    ``still_waiting`` reports liveness every ``HEARTBEAT_INTERVAL_S`` with what
    the last survey checked. A bus error (``bus_error``) is retried
    ``BUS_ERROR_ATTEMPTS`` times in a row before it ends the run.
    Returns ``None`` whenever the chain must be re-read from the bus before
    carrying on: the port went away and came back (``port_lost``), a motor
    already carrying this slot's ID answered again (``configured_motor_returned``),
    or configured motors stopped answering on two surveys in a row
    (``configured_motor_missing`` on the first). A wrong model is reported once
    (``wrong_motor_detected``) and polling continues so the operator can swap it
    without losing the run.
    """
    started = last_beat = time.monotonic()
    polls = 0
    wrong_model_reported = False
    missing_before: list[int] = []
    checked: list[str] = []
    bus_errors = 0
    while True:
        _check_stop(should_stop)
        if not serial_port_exists(plan.port):
            _emit(progress_callback, "port_lost", port=plan.port, target_id=target_id)
            wait_for_port(plan.port, present=True, progress_callback=progress_callback,
                          should_stop=should_stop)
            return None
        try:
            for baud in dict.fromkeys((plan.default_baud, plan.target_baud)):
                motor = find_default_motor(plan, expected_model, progress_callback, baud=baud)
                if motor is not None:
                    if baud != plan.default_baud:
                        _emit(progress_callback, "half_programmed_motor_adopted",
                              target_id=target_id, motor=motor)
                    return motor, baud
            wrong_model_reported = False
            polls += 1
            if polls % CENSUS_EVERY_POLLS == 0:
                seen = _census(plan, configured, target_id, progress_callback,
                               tier=_census_tier(time.monotonic() - started), state=state)
                checked = seen["checked"]
                if seen["returned"]:
                    _emit(progress_callback, "configured_motor_returned", target_id=target_id)
                    return None
                if seen["missing"]:
                    if seen["missing"] == missing_before:
                        return None
                    _emit(progress_callback, "configured_motor_missing", missing=seen["missing"])
                missing_before = seen["missing"]
            bus_errors = 0
        except MotorChainError as exc:
            if not wrong_model_reported:
                _emit(progress_callback, "wrong_motor_detected", target_id=target_id,
                      expected_model=expected_model, error=str(exc))
                wrong_model_reported = True
            time.sleep(poll_interval)
            continue
        except Exception as exc:
            # A serial hiccup mid-scan: say so and look again, up to a point.
            bus_errors += 1
            _emit(progress_callback, "bus_error", target_id=target_id, attempt=bus_errors,
                  error=str(exc))
            if bus_errors >= BUS_ERROR_ATTEMPTS:
                raise MotorChainError(
                    f"the bus kept failing while waiting for motor {target_id}: {exc}"
                ) from exc
            time.sleep(ID_CHANGE_SETTLE_S)
            continue
        now = time.monotonic()
        if now - last_beat >= HEARTBEAT_INTERVAL_S:
            last_beat = now
            waited = now - started
            _emit(progress_callback, "still_waiting", target_id=target_id,
                  expected_model=expected_model, elapsed_s=round(waited),
                  polls=polls, checked=list(checked),
                  other_rates=waited >= OTHER_RATES_AFTER_S,
                  escalated=waited >= ESCALATE_AFTER_S)
        time.sleep(poll_interval)


def _settle_prescan(
    plan: MotorChainPlan,
    validated: set[int],
    expected: list[int],
    first_scan: bool,
    progress_callback: Optional[ProgressCallback],
) -> ChainScan:
    """Read the chain off the bus and deal with anything that is not a prefix.

    ``validated`` holds every ID this run has seen in the chain; ``expected``
    the prefix as of the previous read. A motor at the factory-default ID is
    not invalid: it is fresh, or stopped mid-programming, and the poll adopts
    it. A motor of the wrong model for its slot, or one at an ID this run never
    validated, is a stray and is reset. A validated motor that no longer
    answers shrinks the chain (``chain_shortened``): the run goes back to that
    slot and waits for the motor to return or a fresh one to take its place.
    Validated motors stranded behind the gap are left alone and rejoin when it
    closes. On the first scan a right-model motor beyond the prefix is
    ambiguous — a leftover from another hand, or a configured motor behind one
    that does not answer — so the run aborts and names it rather than guess.
    """
    for attempt in range(VERIFY_ATTEMPTS):
        scan = scan_configured_motors(plan)
        if all(i in scan.motors_by_id for i in expected) or attempt == VERIFY_ATTEMPTS - 1:
            break
        time.sleep(ID_CHANGE_SETTLE_S)
    missing = [i for i in expected if i not in scan.motors_by_id]
    stranded = [i for i in scan.invalid_ids if i in validated]
    if missing:
        _emit(progress_callback, "chain_shortened", missing=missing, stranded=stranded,
              configured_ids=list(scan.valid_ids))
    ambiguous: list[dict] = []
    for mid in scan.invalid_ids:
        m = scan.motors_by_id[mid]
        if mid == plan.default_id or mid in validated:
            continue
        if _model_matches(plan, m, plan.model_for(mid)) and first_scan:
            ambiguous.append(m)
            continue
        _reset_stray(plan, m, progress_callback)
    if ambiguous:
        ids = [m["id"] for m in ambiguous]
        raise MotorChainAborted(
            f"motors {ids} answer at the target baud but the chain before them is "
            f"incomplete. Either a motor before them is not answering (check its "
            f"connector and power) or they are leftovers from another hand: reset "
            f"them to factory defaults (ID={plan.default_id}, baud={plan.default_baud}) "
            "and retry",
            invalid_ids=ids, invalid_motors=ambiguous,
        )
    return scan


def configure_motor_chain(
    plan: MotorChainPlan,
    progress_callback: Optional[ProgressCallback] = None,
    prompt_callback: Optional[PromptCallback] = None,
    should_stop: Optional[ShouldStop] = None,
    poll_interval: float = MOTOR_POLL_INTERVAL_S,
) -> list[int]:
    """Program every motor in ``plan``, resuming from whatever is already done.

    The chain is re-read from the bus before every step, so the run follows
    the bus rather than its own bookkeeping: a motor stranded mid-programming
    is adopted and finished, a motor on the bus that is not part of the chain
    is reset to factory defaults and then programmed, a configured motor that
    drops off hands its slot back to the run (which waits for it to return or
    for a fresh motor), and a port that goes away is waited for and the chain
    re-read when it returns.

    Emits ``chain_started``, ``prescan_done``, ``step_started``,
    ``awaiting_motor``, ``still_waiting``, ``wrong_motor_detected``,
    ``unrecognised_motor_model``, ``half_programmed_motor_adopted``,
    ``stray_motor_reset``, ``stray_motor_reset_failed``,
    ``configured_motor_missing``, ``configured_motor_returned``,
    ``chain_shortened``, ``phantom_baud``, ``port_lost``, ``bus_error``,
    ``motor_found``, ``write_retry``, ``duplicate_default_motor``,
    ``motor_configured``, ``verify_retry``, ``verify_failed``,
    ``chain_verified``, ``chain_done``. Asks the operator to connect each motor
    via ``prompt_callback`` (required for Feetech, whose bus must be unpowered
    while plugging).

    Returns the configured motor IDs. Raises :class:`MotorChainAborted` when
    the bus holds motors it cannot place without guessing.
    """
    _emit(progress_callback, "chain_started", motor_type=plan.motor_type, port=plan.port,
          total_motors=plan.total_motors, target_baud=plan.target_baud,
          finger_ids=plan.finger_ids, wrist_id=plan.wrist_id)

    highest_id = max(plan.all_target_ids)
    validated: set[int] = set()
    configured: list[int] = []
    bus_state = {"skip_bauds": set()}
    first_scan = True
    announced: Optional[int] = None
    bus_errors = 0
    while True:
        _check_stop(should_stop)
        try:
            scan = _settle_prescan(plan, validated, configured, first_scan, progress_callback)
        except MotorChainError:
            raise
        except Exception as exc:
            bus_errors += 1
            _emit(progress_callback, "bus_error", target_id=None, attempt=bus_errors, error=str(exc))
            if bus_errors >= BUS_ERROR_ATTEMPTS:
                raise MotorChainError(f"the bus kept failing while reading the chain: {exc}") from exc
            time.sleep(ID_CHANGE_SETTLE_S)
            continue
        bus_errors = 0
        if first_scan:
            _emit(progress_callback, "prescan_done", already_configured=scan.valid_ids,
                  motors=[scan.motors_by_id[i] for i in sorted(scan.valid_ids, reverse=True)])
            first_scan = False
        configured = list(scan.valid_ids)
        validated.update(configured)
        remaining = [mid for mid in plan.all_target_ids if mid not in configured]
        if not remaining:
            _emit(progress_callback, "chain_done", configured_ids=configured)
            return configured

        target_id = remaining[0]
        expected_model = plan.model_for(target_id)
        attaches_to = "board" if target_id == highest_id else f"motor {target_id + 1}"
        message = f"Connect a {expected_model} (ID {target_id}) to the {attaches_to}"
        if announced != target_id:
            announced = target_id
            _emit(progress_callback, "step_started", step=len(configured) + 1,
                  total=plan.total_motors, target_id=target_id,
                  expected_model=expected_model, attaches_to=attaches_to, message=message)
            _await_motor_connection(plan, message, prompt_callback, progress_callback,
                                    should_stop, target_id=target_id,
                                    expected_model=expected_model)

        _emit(progress_callback, "awaiting_motor", target_id=target_id,
              expected_model=expected_model)
        found = _wait_for_default_motor(plan, target_id, expected_model, configured,
                                        progress_callback, should_stop, poll_interval,
                                        state=bus_state)
        if found is None:
            continue  # the port came back: re-read the chain before touching anything
        motor, at_baud = found
        _emit(progress_callback, "motor_found", target_id=target_id, motor=motor)
        configure_default_motor(plan, target_id, progress_callback=progress_callback,
                                current_baud=at_baud)
        configured.append(target_id)
        validated.add(target_id)
        _emit(progress_callback, "motor_configured", target_id=target_id,
              baudrate=plan.target_baud, configured_ids=list(configured))

        try:
            verify_chain(plan, configured, progress_callback, tolerated_ids=validated)
        except MotorChainError as exc:
            # The bus changed under us; the next read of the chain says how.
            _emit(progress_callback, "verify_failed", configured_ids=list(configured),
                  error=str(exc))
            continue
        _emit(progress_callback, "chain_verified", configured_ids=list(configured))


def _each_motor_on_bus(
    plan: MotorChainPlan,
    action: Callable[[dict], None],
    connect_message: str,
    progress_callback: Optional[ProgressCallback],
    prompt_callback: Optional[PromptCallback],
    should_stop: Optional[ShouldStop],
    bauds: Optional[list[int]] = None,
    id_range: Optional[tuple[int, int]] = None,
) -> list[dict]:
    """Apply ``action`` to every motor currently on the bus. One pass, no looping.

    Families needing an unpowered bus are power-cycled first, so this pass acts
    on whatever the operator just plugged in. Motors are scanned at the chain's
    target baud over the chain's IDs unless ``bauds`` / ``id_range`` widen it.
    An action returning ``False`` had nothing to do; such motors still count as
    acted on but are not re-announced, so a repeated pass stays quiet about
    motors already in the requested state.
    """
    _await_motor_connection(plan, connect_message, prompt_callback,
                            progress_callback, should_stop)

    motors: list[dict] = []
    for baud in bauds or [plan.target_baud]:
        _check_stop(should_stop)
        motors += scan_motors(plan.motor_type, plan.port, baud, id_range or (1, plan.total_motors))
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
    all_rates: bool = False,
) -> list[dict]:
    """Reset every motor on the bus to factory defaults, whatever ID or baud
    it carries. One pass; the caller loops so motors plugged in later are
    taken too.

    The chain's rate and the factory rate are swept over ``SWEEP_ID_RANGE``,
    the factory rate last so a motor moved there by this pass is seen once;
    hands use no other rate, so the rest join only with ``all_rates`` (the
    caller's last resort when nothing has turned up for a while). Motors
    already at the defaults are counted in the result but not re-announced.
    """
    rates = plan.client_cls.scannable_baudrates() if all_rates else [plan.target_baud]
    bauds = [b for b in rates if b != plan.default_baud]
    bauds.append(plan.default_baud)
    return _each_motor_on_bus(
        plan,
        lambda m: reset_motor_to_factory(plan, m["id"], m["baud_rate"]),
        "Connect the motor(s) you want to reset",
        progress_callback, prompt_callback, should_stop,
        bauds=bauds, id_range=SWEEP_ID_RANGE,
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


def set_all_return_delays(
    plan: MotorChainPlan,
    progress_callback: Optional[ProgressCallback] = None,
    prompt_callback: Optional[PromptCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> list[dict]:
    """Write the family's Return Delay Time into every motor on the bus. One pass.

    For hands whose motors were configured before chain assembly set it: the
    delay lives in each motor's EEPROM, and re-running the chain skips motors
    already at their target ID. Raises :class:`MotorChainError` for a family
    that declares no delay (Feetech), before touching the bus.
    """
    if plan.return_delay_time_us is None:
        raise MotorChainError(f"{plan.motor_type} motors have no return delay to set")
    return _each_motor_on_bus(
        plan,
        lambda m: set_return_delay_only(plan, m["id"], m["baud_rate"]),
        "Connect the motor(s) whose return delay you want to set",
        progress_callback, prompt_callback, should_stop,
    )
