# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Measure every motor on a cold, stationary hand.

Nothing is commanded and nothing moves, which is the point: this is the
clean-bus baseline taken before any load, and it is what a returned hand's
"no status packet" report gets compared against. A bus that already drops
reads with the motors still is a bus problem, not a duty-cycle problem.

Error bytes are reported by name but never interpreted into a mechanical
cause — the byte says a fault is latched, not what broke it.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, TYPE_CHECKING

from .step_result import flat_measurements

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

BULK_READS = 200
"""Bulk reads taken on the stationary bus, as the test plan specifies."""

MAX_FAILED_READS = 0
"""Dropped status packets tolerated on a hand that is not moving."""

MAX_TEMPERATURE_C = 40.0
"""Provisional. A motor that has not been driven should sit near ambient;
this leaves room for a warm room and a hand that ran recently."""

TEMPERATURE_SPREAD_LIMIT_C = 8.0
"""Provisional, and the more meaningful of the two: absolute temperature is
ambient-dependent, so the outlier motor is the signal rather than the peak."""


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the routine."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("motor health progress callback failed")


@dataclass(frozen=True)
class MotorHealth:
    """One motor's cold reading.

    ``error_byte`` is ``None`` when the motor did not answer, which is not
    the same claim as a clean ``0x00`` and is judged as a fault.
    ``voltage_v`` is ``None`` on a motor family that cannot report it.
    """

    motor: int
    joint: str
    error_byte: int | None
    faults: List[str] = field(default_factory=list)
    temperature_c: float | None = None
    voltage_v: float | None = None
    messages: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.error_byte == 0 and not self.messages


@dataclass(frozen=True)
class MotorHealthResult:
    """Every motor's cold reading plus the bus's dropped-read count.

    Implements :class:`~orca_core.verification.StepResult`.
    """

    motors: List[MotorHealth]
    bulk_reads: int
    failed_reads: int
    temperature_spread_c: float | None
    thresholds: Dict[str, float]
    step_messages: List[str] = field(default_factory=list)

    @property
    def failed(self) -> List[MotorHealth]:
        return [motor for motor in self.motors if not motor.ok]

    @property
    def passed(self) -> bool:
        return not self.failed and not self.step_messages

    @property
    def messages(self) -> List[str]:
        return [m for motor in self.motors for m in motor.messages] + list(
            self.step_messages
        )

    def measurements(self) -> Dict[str, float]:
        """Per joint: its error byte, temperature and supply voltage. Then the
        bus-wide counts, which belong to no single motor."""
        measurements = {
            **flat_measurements(
                "hardware_error_byte", {m.joint: m.error_byte for m in self.motors}
            ),
            **flat_measurements(
                "temperature_c", {m.joint: m.temperature_c for m in self.motors}
            ),
            **flat_measurements(
                "supply_voltage_v", {m.joint: m.voltage_v for m in self.motors}
            ),
            "bulk_reads": float(self.bulk_reads),
            "failed_bulk_reads": float(self.failed_reads),
        }
        if self.temperature_spread_c is not None:
            measurements["temperature_spread_c"] = float(self.temperature_spread_c)
        return measurements


def run_motor_health(
    hand: "OrcaHand",
    *,
    bulk_reads: int = BULK_READS,
    max_failed_reads: int = MAX_FAILED_READS,
    max_temperature_c: float = MAX_TEMPERATURE_C,
    temperature_spread_limit_c: float = TEMPERATURE_SPREAD_LIMIT_C,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> MotorHealthResult | None:
    """Read every motor's error byte, temperature and voltage, then hammer the bus.

    Read-only: no motor is commanded and torque is left exactly as it was
    found, so this is safe to run on a hand in any state. Returns ``None`` on
    early exit (``should_stop`` triggered).

    Args:
        hand: A connected :class:`~orca_core.OrcaHand`.
        bulk_reads: Position/velocity/current reads taken on the still bus.
        max_failed_reads: Dropped status packets tolerated across those reads.
        max_temperature_c: Above this a motor is not cold.
        temperature_spread_limit_c: Largest gap tolerated between the hottest
            and coldest motor.
        progress_callback: Optional ``callable(dict)`` invoked with structured
            progress events (``health_started``, ``motor_read``,
            ``bus_check_started``, ``bus_check_done``, ``health_aborted``,
            ``health_done``). Must be fast and non-blocking; exceptions it
            raises are swallowed.
        should_stop: Optional ``callable() -> bool`` polled between motors and
            during the bulk-read loop.

    Raises:
        NotImplementedError: If this motor family cannot report hardware
            errors, which is the measurement this step exists for. Supply
            voltage is optional and simply reads as absent.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    motor_to_joint = hand.config.motor_to_joint_dict
    thresholds = {
        "bulk_reads": float(bulk_reads),
        "max_failed_reads": float(max_failed_reads),
        "max_temperature_c": float(max_temperature_c),
        "temperature_spread_limit_c": float(temperature_spread_limit_c),
    }

    _emit(progress_callback, "health_started", motors=list(hand.config.motor_ids))

    errors = hand.get_motor_errors()
    temperatures = hand.get_motor_temp(as_dict=True)
    voltages = _read_voltages(hand)

    motors: List[MotorHealth] = []
    for motor_id in hand.config.motor_ids:
        if should_stop():
            _emit(progress_callback, "health_aborted")
            return None
        error_byte = errors.get(motor_id)
        health = _judge_motor(
            motor_id=motor_id,
            joint=motor_to_joint.get(motor_id, f"motor_{motor_id}"),
            error_byte=error_byte,
            temperature_c=_as_float(temperatures.get(motor_id)),
            voltage_v=_as_float(voltages.get(motor_id)),
            faults=(
                hand.motor_client.decode_hardware_error(error_byte)
                if error_byte
                else []
            ),
            max_temperature_c=max_temperature_c,
        )
        motors.append(health)
        _emit(
            progress_callback,
            "motor_read",
            motor=health.motor,
            joint=health.joint,
            error_byte=health.error_byte,
            temperature_c=health.temperature_c,
            ok=health.ok,
        )

    _emit(progress_callback, "bus_check_started", reads=bulk_reads)
    taken, failed_reads = _count_failed_bulk_reads(
        hand, reads=bulk_reads, should_stop=should_stop
    )
    _emit(progress_callback, "bus_check_done", reads=taken, failed=failed_reads)
    if taken < bulk_reads:
        _emit(progress_callback, "health_aborted")
        return None

    result = MotorHealthResult(
        motors=motors,
        bulk_reads=taken,
        failed_reads=failed_reads,
        temperature_spread_c=_spread(motors),
        thresholds=thresholds,
        step_messages=_step_messages(
            motors=motors,
            bulk_reads=taken,
            failed_reads=failed_reads,
            max_failed_reads=max_failed_reads,
            temperature_spread_limit_c=temperature_spread_limit_c,
        ),
    )
    _emit(
        progress_callback,
        "health_done",
        failed=[m.joint for m in result.failed],
        failed_reads=failed_reads,
    )
    return result


def _as_float(value) -> float | None:
    return None if value is None else float(value)


def _read_voltages(hand: "OrcaHand") -> Dict[int, float]:
    """Supply voltage per motor, empty on a family that cannot report it.

    Voltage is a confounder recorded alongside the measurements, not a gate,
    so a family without it does not fail the step.
    """
    try:
        return hand.get_motor_voltage(as_dict=True)
    except NotImplementedError:
        logger.info("motor family cannot report supply voltage; not recorded")
        return {}


def _spread(motors: List[MotorHealth]) -> float | None:
    temperatures = [m.temperature_c for m in motors if m.temperature_c is not None]
    if len(temperatures) < 2:
        return None
    return round(max(temperatures) - min(temperatures), 2)


def _judge_motor(
    *,
    motor_id: int,
    joint: str,
    error_byte: int | None,
    temperature_c: float | None,
    voltage_v: float | None,
    faults: List[str],
    max_temperature_c: float,
) -> MotorHealth:
    """Reduce one motor's readings to a verdict.

    The error byte's named bits are reported; what set them is not guessed.
    """
    messages: List[str] = []
    if error_byte is None:
        messages.append(
            f"Motor {motor_id} ({joint}) returned no status packet, so its "
            f"hardware error state is unknown."
        )
    elif error_byte:
        named = ", ".join(faults) if faults else "no named bits"
        messages.append(
            f"Motor {motor_id} ({joint}) reports hardware error "
            f"0x{error_byte:02X} ({named})."
        )

    if temperature_c is not None and temperature_c > max_temperature_c:
        messages.append(
            f"Motor {motor_id} ({joint}) is at {temperature_c:.1f}°C before "
            f"being driven (limit {max_temperature_c:.1f}°C)."
        )

    return MotorHealth(
        motor=motor_id,
        joint=joint,
        error_byte=error_byte,
        faults=list(faults),
        temperature_c=temperature_c,
        voltage_v=voltage_v,
        messages=messages,
    )


def _step_messages(
    *,
    motors: List[MotorHealth],
    bulk_reads: int,
    failed_reads: int,
    max_failed_reads: int,
    temperature_spread_limit_c: float,
) -> List[str]:
    """Text for what belongs to the bus rather than to one motor."""
    messages: List[str] = []
    if failed_reads > max_failed_reads:
        messages.append(
            f"{failed_reads} of {bulk_reads} bulk reads returned no status "
            f"packet on a stationary hand (limit {max_failed_reads})."
        )

    spread = _spread(motors)
    if spread is not None and spread > temperature_spread_limit_c:
        hottest = max(motors, key=lambda m: m.temperature_c or float("-inf"))
        coldest = min(motors, key=lambda m: m.temperature_c or float("inf"))
        messages.append(
            f"Motor temperatures span {spread:.1f}°C on a cold hand (limit "
            f"{temperature_spread_limit_c:.1f}°C): {hottest.joint} at "
            f"{hottest.temperature_c:.1f}°C against {coldest.joint} at "
            f"{coldest.temperature_c:.1f}°C."
        )
    return messages


def _count_failed_bulk_reads(
    hand: "OrcaHand", *, reads: int, should_stop: ShouldStop
) -> tuple[int, int]:
    """Take ``reads`` bulk reads, counting the ones the bus never answered.

    The read and its freshness flag are taken under one lock hold: the flag
    is client-global and qualifies only the most recent read, so any
    interleaved read would rebind it to different data.

    Returns:
        ``(reads taken, reads that failed)``.
    """
    failed = 0
    for taken in range(reads):
        if should_stop():
            return taken, failed
        with hand._motor_lock:
            hand.motor_client.read_position_velocity_current()
            if not hand.motor_client.last_read_ok:
                failed += 1
    return reads, failed
