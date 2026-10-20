"""Per-motor fault accounting and the policy that acts on it.

Pure logic: nothing here touches a bus. A driver feeds observations in and
carries out the actions that come back, so the same policy serves a script,
the console, and the test suite.

Two conditions are handled, chosen because both families raise them and both
matter mechanically:

``excess_load``
    A stalled or over-tensioned tendon. One family raises it as two separate
    bits (overcurrent and overload) and stops the motor for only one of them;
    the other raises one bit and stops. Whether the motor actually stopped is
    read from its stop mask rather than assumed, and only a motor that
    stopped is recovered - there is nothing to fix on one that is still
    driving.

``thermal``
    Overheating, which is in both families' shipped stop masks. Recovery
    waits: a motor rebooted while still hot re-latches on the next command,
    so the reboot is deferred until it has cooled a configurable margin below
    its own temperature limit.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Set

# Conditions this module acts on. Others are recorded and reported only.
EXCESS_LOAD = "excess_load"
THERMAL = "thermal"


@dataclass(frozen=True)
class FaultPolicyConfig:
    """Thresholds, all per motor and all counting reboots."""

    # Reboots of one motor before its repetition is called out prominently.
    report_after_reboots: int = 3
    # Load reboots of one motor before it is left latched instead of
    # recovered. Counted separately from thermal: a joint that keeps stalling
    # is a mechanical fault, and recovering it again only hides the cause.
    giveup_after_reboots: int = 5
    # Thermal recoveries before a motor is left latched. ``None`` means never
    # give up, which is the default: a motor that has cooled can work again,
    # and the wait is self-limiting because it does not come back until it
    # has cooled.
    thermal_giveup_after_reboots: Optional[int] = None
    # How far below its temperature limit a motor must cool before the
    # thermal latch is cleared.
    thermal_clear_margin_c: float = 10.0
    # Floor on how often one motor may be rebooted. A motor that faults the
    # instant torque returns would otherwise spend the bus on restarts, and
    # each one costs a third of a second to nearly a second depending on
    # family.
    min_reboot_interval_s: float = 2.0


@dataclass(frozen=True)
class FaultEvent:
    """One observed fault on one motor.

    ``stopped`` says whether the motor's own stop mask covered the bits that
    were raised, which is the difference between "it quit" and "it carried on
    and told us".

    Deliberately no current reading. The sampled value at the moment a fault
    is noticed understates the spike that caused it, badly enough to mislead
    anyone reading it as "the current this motor faults at". A per-direction
    running average would mean something, and calibration already records
    one; a single sample does not.
    """

    motor_id: int
    joint: Optional[str]
    conditions: Set[str]
    flag_names: Set[str]
    raw: int
    stopped: bool
    at: float               # wall clock, for reporting
    monotonic: float        # for durations and rate limits
    temperature_c: Optional[float] = None
    cleared_at: Optional[float] = None
    duration_s: Optional[float] = None


@dataclass
class MotorFaultRecord:
    """Everything known about one motor's faults this session."""

    motor_id: int
    events: List[FaultEvent] = field(default_factory=list)
    # Reboots this policy performed. Drives both thresholds.
    reboots: int = 0
    last_reboot_monotonic: Optional[float] = None
    # Raw error byte last observed, so a persisting latch is one event rather
    # than one per poll.
    last_raw: int = 0
    # Set while a thermal latch is waiting on the motor to cool.
    cooling: bool = False
    # Thermal latches cleared after cooling. Not gated by the reboot
    # thresholds - those count load recoveries - but counted so a motor that
    # keeps cooking itself is visible.
    thermal_recoveries: int = 0
    # Whether torque was on when the fault stopped the motor, so recovery
    # restores what the operator had rather than energizing a relaxed hand.
    torque_was_on: bool = False
    given_up: bool = False
    thermal_given_up: bool = False

    def count(self, condition: str, *, stopped_only: bool = False) -> int:
        return sum(1 for event in self.events
                   if condition in event.conditions
                   and (event.stopped or not stopped_only))


# Actions a driver carries out. Combinable: a single observation can return
# several, in the order they should be performed.
LOG = "log"
NOTIFY = "notify"
DISABLE_TORQUE = "disable_torque"
ENABLE_TORQUE = "enable_torque"
REBOOT = "reboot"


@dataclass(frozen=True)
class FaultAction:
    """One thing to do to one motor, and why."""

    kind: str
    motor_id: int
    reason: str
    # Set on notify: a condition the operator has to be shown, not just logged.
    prominent: bool = False


@dataclass(frozen=True)
class MotorObservation:
    """What a driver knows about one motor on one tick."""

    motor_id: int
    raw_error: Optional[int]
    stop_mask: Optional[int]
    joint: Optional[str] = None
    temperature_c: Optional[float] = None
    temperature_limit_c: Optional[float] = None
    torque_enabled: bool = False


class MotorFaultPolicy:
    """Decides what to do about each motor's faults, and remembers.

    Counters are per motor and last for the life of the policy, which a
    driver recreates per session, so "three reboots" means three this run.
    """

    def __init__(self, client, config: FaultPolicyConfig | None = None):
        self._client = client
        self.config = config or FaultPolicyConfig()
        self.records: Dict[int, MotorFaultRecord] = {}

    def record(self, motor_id: int) -> MotorFaultRecord:
        return self.records.setdefault(int(motor_id),
                                       MotorFaultRecord(motor_id=int(motor_id)))

    def observe(self, observations, now: float | None = None) -> List[FaultAction]:
        """Fold one tick's readings in and return the actions to perform."""
        monotonic = time.monotonic() if now is None else now
        actions: List[FaultAction] = []
        for observation in observations:
            actions.extend(self._observe_one(observation, monotonic))
        return actions

    def _observe_one(self, obs: MotorObservation,
                     monotonic: float) -> List[FaultAction]:
        # A motor that did not answer tells us nothing; leave its state alone
        # rather than reading silence as recovery, and do not invent a record
        # for a motor we have never heard from.
        if obs.raw_error is None:
            return []

        record = self.record(obs.motor_id)
        raw = int(obs.raw_error)

        actions: List[FaultAction] = []

        # Cooling takes priority: the motor is already stopped and the only
        # question is whether it is cool enough to bring back.
        if record.cooling:
            return self._resume_if_cool(obs, record, monotonic)

        appeared = raw & ~record.last_raw
        record.last_raw = raw
        if not appeared:
            if not raw:
                self._close_open_events(record, monotonic)
            return actions

        stopped, flagged = self._client.split_faults(appeared, obs.stop_mask)
        event = FaultEvent(
            motor_id=int(obs.motor_id), joint=obs.joint,
            conditions=stopped | flagged,
            flag_names=set(self._client.decode_hardware_error(appeared) or []),
            raw=appeared, stopped=bool(stopped),
            at=time.time(), monotonic=monotonic,
            temperature_c=obs.temperature_c,
        )
        record.events.append(event)
        actions.append(FaultAction(
            LOG, obs.motor_id,
            f"{'stopped by' if stopped else 'reported'} "
            f"{', '.join(sorted(event.flag_names)) or 'unknown'}"))

        if not stopped:
            # Nothing to recover: the motor is still driving and said so.
            return actions

        record.torque_was_on = obs.torque_enabled or record.torque_was_on

        if THERMAL in stopped:
            record.cooling = True
            actions.append(FaultAction(
                NOTIFY, obs.motor_id,
                "overheated and stopped; holding until it cools "
                f"{self.config.thermal_clear_margin_c:.0f} degC below its limit",
                prominent=True))
            # Both families ship overheating in their stop mask, so the
            # motor has normally de-energized itself by now. If it still
            # reports torque on - a cleared mask bit, or a read that raced
            # the stop - take it off ourselves rather than leave a hot motor
            # driving.
            if obs.torque_enabled:
                actions.append(FaultAction(
                    DISABLE_TORQUE, obs.motor_id,
                    "still energized after an overheat stop"))
            return actions

        if EXCESS_LOAD in stopped:
            actions.extend(self._recover_load(obs, record, monotonic))
        return actions

    def _recover_load(self, obs: MotorObservation, record: MotorFaultRecord,
                      monotonic: float) -> List[FaultAction]:
        config = self.config
        if record.reboots >= config.giveup_after_reboots:
            if not record.given_up:
                record.given_up = True
            return [FaultAction(
                NOTIFY, obs.motor_id,
                f"left latched after {record.reboots} reboots: it keeps "
                "overloading, so recovering it again would only hide the "
                "mechanical cause", prominent=True)]

        last = record.last_reboot_monotonic
        if last is not None and monotonic - last < config.min_reboot_interval_s:
            return [FaultAction(
                LOG, obs.motor_id,
                f"faulted again within {monotonic - last:.1f}s; waiting out "
                f"the {config.min_reboot_interval_s:.0f}s reboot floor")]

        record.reboots += 1
        record.last_reboot_monotonic = monotonic
        actions = [FaultAction(REBOOT, obs.motor_id,
                               f"overloaded and stopped (reboot {record.reboots})")]
        if record.torque_was_on:
            actions.append(FaultAction(ENABLE_TORQUE, obs.motor_id,
                                       "torque was on before the fault"))
        if record.reboots >= config.report_after_reboots:
            actions.append(FaultAction(
                NOTIFY, obs.motor_id,
                f"has overloaded and been rebooted {record.reboots} times: "
                "something is mechanically wrong with this joint",
                prominent=True))
        return actions

    def _resume_if_cool(self, obs: MotorObservation, record: MotorFaultRecord,
                        monotonic: float) -> List[FaultAction]:
        limit = obs.temperature_limit_c
        temperature = obs.temperature_c
        if limit is None or temperature is None:
            return []
        if temperature > limit - self.config.thermal_clear_margin_c:
            return []

        # Same floor as a load recovery: if clearing the latch does not take,
        # this must not become a restart loop.
        last = record.last_reboot_monotonic
        if last is not None and monotonic - last < self.config.min_reboot_interval_s:
            return []

        allowed = self.config.thermal_giveup_after_reboots
        if allowed is not None and record.thermal_recoveries >= allowed:
            if record.thermal_given_up:
                return []
            record.thermal_given_up = True
            return [FaultAction(
                NOTIFY, obs.motor_id,
                f"left latched after {record.thermal_recoveries} thermal "
                "recoveries: it keeps overheating, so whatever is heating it "
                "has not been dealt with", prominent=True)]

        record.cooling = False
        record.thermal_recoveries += 1
        record.last_reboot_monotonic = monotonic
        record.last_raw = 0
        self._close_open_events(record, monotonic)
        actions = [FaultAction(
            REBOOT, obs.motor_id,
            f"cooled to {temperature:.0f} degC, {limit - temperature:.0f} degC "
            "below its limit; clearing the thermal latch")]
        if record.torque_was_on:
            actions.append(FaultAction(ENABLE_TORQUE, obs.motor_id,
                                       "torque was on before it overheated"))
        actions.append(FaultAction(NOTIFY, obs.motor_id,
                                   "recovered after cooling"))
        return actions

    @staticmethod
    def _close_open_events(record: MotorFaultRecord, monotonic: float) -> None:
        """Stamp how long each still-open fault lasted."""
        for index, event in enumerate(record.events):
            if event.cleared_at is None:
                record.events[index] = replace(
                    event, cleared_at=time.time(),
                    duration_s=monotonic - event.monotonic)
