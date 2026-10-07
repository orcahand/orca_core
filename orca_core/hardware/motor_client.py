# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Abstract base class for motor communication clients."""

import logging
import math
from dataclasses import dataclass
from abc import ABC, abstractmethod
from typing import ClassVar, NamedTuple, Optional, Sequence
import numpy as np

from ..constants import CONTROL_MODES

logger = logging.getLogger(__name__)


class MotorError(Exception):
    """Raised when a motor operation cannot be completed."""


class MotionTimeoutError(MotorError):
    """Raised when motors fail to settle within the requested timeout."""


class MotorRead(NamedTuple):
    """A single snapshot of position / velocity / current for all motors.

    Each field is a 1-D numpy array indexed by the motor order configured
    on the client. NamedTuple so callers can also unpack as
    ``position, velocity, current = client.read_position_velocity_current()``.

    A snapshot does not carry its own freshness; that is reported
    out-of-band via :attr:`MotorClient.last_read_ok`, whose coupling rules
    are documented there.
    """
    position: np.ndarray
    velocity: np.ndarray
    current: np.ndarray


@dataclass(frozen=True)
class ServoGains:
    """One motor's internal position-PID and feedforward gains.

    Feedforward acts on the *desired trajectory*, not on error: ``ff_1st``
    scales its velocity and ``ff_2nd`` its acceleration, supplying the command
    a move needs before any error has accumulated. Both are therefore near
    useless while Profile Velocity and Acceleration are zero -- a Goal
    Position write is then a step, and a step has no trajectory to
    differentiate.

    ``None`` means "leave this one alone" on a write, and "not reported" on a
    read.
    """

    kp: "int | None" = None
    ki: "int | None" = None
    kd: "int | None" = None
    ff_1st: "int | None" = None
    """Velocity feedforward: cancels the lag of tracking a moving target."""
    ff_2nd: "int | None" = None
    """Acceleration feedforward: acts at profile corners and reversals."""

    def __post_init__(self):
        for name in ("kp", "ki", "kd", "ff_1st", "ff_2nd"):
            value = getattr(self, name)
            if value is None:
                continue
            if isinstance(value, bool) or int(value) != value or value < 0:
                raise ValueError(f"{name} must be a non-negative integer, got {value!r}")


@dataclass(frozen=True)
class ServoProfile:
    """One motor's trajectory-shaping limits, in SI units.

    The servo generates its own trajectory toward Goal Position: acceleration
    ramps up to the velocity cap, giving a trapezoid. ``0.0`` disables a
    limit -- no cap, or instantaneous acceleration -- which is the factory
    state and makes a Goal Position write a step.

    This is a slew-rate limiter, not a filter: motion that stays under the cap
    passes through untouched. With a *streamed* target it therefore behaves as
    a rate limit on the command, which is a genuine safety property for teleop
    and an unwanted lag inside a tuned closed loop. On encoder hands it also
    fights the outer PI, which integrates the error the profile is holding.

    A profile is also what makes the feedforward gains in :class:`ServoGains`
    do anything: they act on this trajectory's derivatives, and a step has
    none. ``None`` means "leave this one alone".
    """

    velocity_rad_s: "float | None" = None
    """Speed cap. 0.0 = uncapped."""
    acceleration_rad_s2: "float | None" = None
    """Ramp rate toward the cap. 0.0 = instantaneous."""

    def __post_init__(self):
        for name in ("velocity_rad_s", "acceleration_rad_s2"):
            value = getattr(self, name)
            if value is None:
                continue
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and >= 0, got {value!r}")


def _warn_servo_registers_unavailable(client, what: str, requested: dict) -> None:
    if requested:
        logger.warning(
            "%s does not expose servo %s; the request for motor(s) %s was "
            "ignored.", type(client).__name__, what, sorted(requested),
        )


class MotorClient(ABC):
    """Abstract base class for motor communication clients.

    This defines the interface that all motor clients (Dynamixel, Feetech, etc.)
    must implement to work with OrcaHand.

    Subclasses describe their motor family through the class attributes below,
    so callers can stay family-agnostic: adding a new family means adding a
    client, not branching on ``motor_type`` at every call site.
    """

    # Subclasses set this to True when ``wait_for_motion_complete`` actually
    # blocks; callers can use it to skip locking around no-op waits.
    waits_for_motion: bool = False

    # How close to its goal a motor must be to count as arrived, in radians.
    # A moving flag alone is not enough: a servo stalled against a load
    # reports stopped while short of its target, and a caller that advanced
    # on that would desynchronise a chain mid-motion.
    arrival_tolerance_rad: ClassVar[float] = 0.02

    def _flush_input_buffer(self):
        """Discards stale RX bytes so a late reply can't be misread as the next response."""
        ser = getattr(getattr(self, "port_handler", None), "ser", None)
        if ser is None or not hasattr(ser, "reset_input_buffer"):
            return
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        # A new motor family gets its own registry; subclasses of an existing
        # family share theirs, so that family's exit cleanup still sees them.
        if not any("OPEN_CLIENTS" in base.__dict__ for base in cls.__mro__):
            cls.OPEN_CLIENTS = set()

    @classmethod
    def cleanup_open_clients(cls):
        """Disconnect every open client of this family at interpreter exit.

        Each client is handled independently so one failing client cannot
        prevent torque-disable of the others.
        """
        for client in list(cls.OPEN_CLIENTS):
            try:
                if client.port_handler.is_using:
                    logging.warning("Forcing %s to close.", cls.__name__)
                client.port_handler.is_using = False
                client.disconnect()
            except Exception:
                logging.exception(
                    "Exit cleanup failed for client on %s",
                    getattr(client, "port_name", "<unknown>"),
                )

    # ----- Motor-family description ----------------------------------------

    motor_type: ClassVar[str] = ""
    """The family name this client drives, e.g. ``"dynamixel"``."""

    factory_default_id: ClassVar[int] = 1
    """Motor ID a factory-fresh motor of this family answers on."""

    factory_default_baudrate: ClassVar[int] = 0
    """Baud rate a factory-fresh motor of this family answers at."""

    baud_rate_map: ClassVar[dict] = {}
    """Baud rate in bps → the register value this family encodes it as."""

    return_delay_time_us: ClassVar[Optional[int]] = None
    """Return Delay Time chain assembly programs into each motor, in µs.

    ``None`` leaves the factory value untouched. A family that sets it must
    implement :meth:`change_return_delay_time`.
    """

    requires_unpowered_hotplug: ClassVar[bool] = False
    """Whether the bus must be de-powered before a motor is plugged in.

    Families that latch their ID on power-up cannot be hot-plugged onto a live
    bus; chain assembly power-cycles the adapter between motors when this is set.
    """

    supports_multi_turn: ClassVar[bool] = True
    """Whether this family can travel beyond one turn (multi-turn position mode)."""

    supported_modes: ClassVar[frozenset[str]] = frozenset(CONTROL_MODES)
    """Control-mode names from :data:`~orca_core.constants.CONTROL_MODES` this family accepts."""

    position_range_rad: ClassVar["tuple[float, float] | None"] = None
    """Commandable position span in radians; ``None`` means unbounded/wrapping."""

    profile_velocity_max_rad_s: ClassVar["float | None"] = None
    """Largest speed the trajectory register can express. ``None`` where the
    family exposes no profile.

    A register width, not a reachable speed. Both families accept the register
    maximum and read it back unchanged while the motor turns no faster than its
    own no-load speed, which is two orders of magnitude lower. Offer this to an
    operator and they are offered a speed nothing can reach; use
    :meth:`read_profile_limits` for the ceiling that actually binds.
    """

    profile_acceleration_max_rad_s2: ClassVar["float | None"] = None
    """Largest acceleration the trajectory register can express.

    A register width, with the same caveat as the velocity above. Note the two
    families are nothing alike here: one holds acceleration in four bytes and
    the other in one, so the same figure is a rounding error of one family's
    range and a quarter of the other's.
    """

    no_load_speed_rad_s: ClassVar["float | None"] = None
    """Datasheet speed with nothing on the output shaft.

    The ceiling for a family whose protocol publishes none, so that
    :meth:`read_profile_limits` can answer with a speed the motor can reach
    instead of one the register merely stores. ``None`` where the limit is
    read from the motor itself.
    """

    default_profile_velocity_fraction: ClassVar[float] = 0.5
    """Share of a motor's own ceiling used as its default speed cap.

    A fraction rather than a figure because the ceiling is per motor: the
    same hand holds motors that top out at different speeds, and a default
    that is deliberate on one of them would be arbitrary on the next.
    """

    default_profile_acceleration_rad_s2: ClassVar[float] = 10.0
    """Default ramp, in rad/s^2.

    Not a fraction of anything: acceleration is unbounded on one family and
    capped at 38.6 on the other, so a share of the range would mean two
    unrelated things. A ramp time is the quantity that transfers -- this
    reaches a typical default cruise speed in about a third of a second.
    """

    profile_ceiling_source: ClassVar[str] = (
        "the motor's own no-load speed; the protocol exposes no limit register, "
        "so nothing rejects a faster request"
    )
    """Where :meth:`read_profile_limits` gets its numbers, in words.

    Worth reporting because the answer differs in kind: one family publishes
    its ceiling in a register the firmware enforces, the other leaves it as a
    datasheet fact that only physics applies.
    """

    servo_gain_max: ClassVar["int | None"] = None
    """Largest value the servo's position-PID registers accept.

    A register width, not a tuning recommendation: the X-series holds gains in
    two bytes capped at 16383, an HLS servo in one byte capped at 254. A
    front-end that assumes one family's width offers values the other refuses.
    ``None`` where the family exposes no gains at all.
    """

    max_operating_temp_c: ClassVar[float] = 70.0
    """Maximum rated operating temperature in degrees Celsius (XC330/XC430, HLS3930/HLS3915)."""

    current_scale_ma: ClassVar[float]
    """mA per raw unit of this family's goal-current register."""

    max_current_ma: ClassVar[float]
    """Largest value this family's goal-current register can express, in mA."""

    default_max_current_ma: ClassVar[int]
    """Goal-current limit in mA for a config whose ``max_current`` is ``default``."""

    default_calibration_current_ma: ClassVar[int]
    """Calibration drive current in mA for a config whose ``calibration_current`` is ``default``."""

    hardware_error_bits: ClassVar["tuple[tuple[int, str], ...]"] = (
        (0x01, "input_voltage"),
        (0x04, "overheating"),
        (0x08, "motor_encoder"),
        (0x10, "electrical_shock"),
        (0x20, "overload"),
    )
    """Hardware Error Status bits, as ``(mask, name)``.

    Every bit in the motor's Shutdown mask latches torque off until the motor
    is rebooted: it keeps answering the bus and acknowledges torque-enable
    writes, but never energizes. Callers that drive a motor and infer
    something from whether it moved must check this first, or they will read
    "did not move" as a mechanical fact.
    """

    @classmethod
    def decode_hardware_error(cls, value: "int | None") -> "list[str] | None":
        """Names of the latched error bits in ``value``; ``None`` if unread."""
        if value is None:
            return None
        return [name for bit, name in cls.hardware_error_bits if value & bit]

    @classmethod
    def servo_limits(cls) -> dict:
        """What each tunable accepts, in the units it is set in.

        A front-end cannot infer these: register widths and units differ by
        family, and the same number means different things — a velocity of
        zero removes the cap, and on one family that is written as the
        register maximum because a literal zero stops the motor. Reported so
        the operator can be shown the real range and what its ends mean,
        rather than a bound someone guessed.

        ``None`` for a tunable the family does not expose.
        """
        return {
            "gain": {
                "min": 0,
                "max": cls.servo_gain_max,
                "unit": "",
                "zero_means": "no contribution from this term",
            },
            "velocity_rad_s": {
                "min": 0.0,
                "max": cls.profile_velocity_max_rad_s,
                "unit": "rad/s",
                "zero_means": "no speed cap",
                "register_width_only": True,
                "ceiling_source": cls.profile_ceiling_source,
            },
            "acceleration_rad_s2": {
                "min": 0.0,
                "max": cls.profile_acceleration_max_rad_s2,
                "unit": "rad/s^2",
                "zero_means": "maximum acceleration",
                "register_width_only": True,
                "ceiling_source": cls.profile_ceiling_source,
            },
        }

    def read_profile_limits(
        self, motor_ids: "Sequence[int]"
    ) -> "dict[int, ServoProfile]":
        """The fastest profile each motor will actually honour.

        Not the same question as how wide the register is, and the difference
        is not small: a motor whose firmware caps it at seven radians a second
        still stores, and reads back, a request for seven hundred. Nothing in
        the protocol objects, so only this tells a caller what the hardware
        will do.

        Reported per motor because it is a property of the motor and not of
        the family — a wrist and a finger joint on one chain answer differently.
        A zero means that axis is unbounded.
        """
        velocity = (self.no_load_speed_rad_s
                    or self.profile_velocity_max_rad_s or 0.0)
        return {
            int(mid): ServoProfile(
                velocity_rad_s=velocity,
                acceleration_rad_s2=self.profile_acceleration_max_rad_s2 or 0.0,
            )
            for mid in motor_ids
        }

    def default_profile(
        self, motor_ids: "Sequence[int]"
    ) -> "dict[int, ServoProfile]":
        """A starting trajectory profile, derived per motor from its ceiling.

        Half speed, which is deliberate on any motor without needing a figure
        chosen for one of them, and a fixed ramp. Both are a starting point an
        operator is expected to adjust, not a limit.
        """
        return {
            mid: ServoProfile(
                velocity_rad_s=round(
                    limits.velocity_rad_s * self.default_profile_velocity_fraction, 3),
                acceleration_rad_s2=self.default_profile_acceleration_rad_s2,
            )
            for mid, limits in self.read_profile_limits(motor_ids).items()
        }

    config_registers: "ClassVar[tuple]" = ()
    """Operator-editable configuration registers this family exposes.

    A tuple of :class:`~orca_core.hardware.config_registers.ConfigRegister`.
    Empty where a family exposes none. What a family lacks is absent rather
    than present-and-disabled, so a caller renders what the connected motor
    actually has.
    """

    def read_config_register(self, motor_id: int, key: str) -> "int | None":
        """Raw value of one declared configuration register, or None.

        None means the motor did not answer, which is not the same as a zero.
        """
        raise NotImplementedError

    def write_config_register(self, motor_id: int, key: str,
                              value: int) -> "int | None":
        """Write one declared register and read it back; returns what stuck.

        The read-back is the point. A sync write carries no acknowledgement,
        and a chain here was found holding gains a write had never landed on,
        so a caller that trusts the write alone will eventually be wrong
        without knowing it. The returned value is what the motor now reports,
        which the caller compares against what it asked for.
        """
        raise NotImplementedError

    def transport_baud_rates(self) -> "tuple[int, ...] | None":
        """Bus rates the transport between host and motors can actually carry.

        ``None`` -- the default, and the answer for a plain USB-TTL adapter --
        means the transport imposes no limit, so ``baud_rate_map`` is the only
        bound. A bridge that only retunes its wire for certain rates returns
        those: moving a motor to a rate outside the list strands it, because the
        host has no way to follow it there.

        A caller offering a bus-wide baud change must intersect this with
        ``baud_rate_map`` rather than offering the family's map raw.
        """
        return None

    def read_servo_gains(
        self, motor_ids: "Sequence[int]"
    ) -> "dict[int, ServoGains | None]":
        """The servo's own position-PID and feedforward gains, per motor.

        Distinct from the host outer-loop PI in ``control/constants.py``:
        these live inside the servo and close the loop the host trims. A
        family that does not expose them reports ``None`` for every motor.
        """
        return {int(mid): None for mid in motor_ids}

    def write_servo_gains(self, gains: "dict[int, ServoGains]") -> None:
        """Set the servo position-PID and feedforward gains, per motor.

        These are RAM registers, so a reboot clears them; clients that
        implement this must remember what was written and restore it the way
        the current ceiling is restored. Fields left ``None`` are untouched.
        """
        _warn_servo_registers_unavailable(self, "gains", gains)

    def read_servo_profile(
        self, motor_ids: "Sequence[int]"
    ) -> "dict[int, ServoProfile | None]":
        """Per-motor trajectory limits. ``None`` where unsupported."""
        return {int(mid): None for mid in motor_ids}

    def write_servo_profile(self, profiles: "dict[int, ServoProfile]") -> None:
        """Set per-motor trajectory limits; ``None`` fields are untouched.

        RAM registers, so implementations must remember what they wrote and
        replay it after a reboot, as they do for the current ceiling.
        """
        _warn_servo_registers_unavailable(self, "trajectory limits", profiles)

    def read_hardware_errors(
        self, motor_ids: Sequence[int]
    ) -> "dict[int, int | None]":
        """Latched Hardware Error Status for several motors at once.

        The bus is half-duplex, so every read blocks commands for its whole
        round trip; a family that can fetch one register from many motors in a
        single transaction should override this to do so. The default falls
        back to one round trip per motor.
        """
        return {int(mid): self.read_hardware_error(mid) for mid in motor_ids}

    def take_hardware_alerts(self) -> "dict[int, int]":
        """Motors seen carrying a latched fault since the last call, and clear.

        Status packets already carry each motor's error byte, so a family that
        can see it should report it here rather than paying for a second read.
        Recovery is the caller's decision: a reboot holds the bus for a third
        of a second, and the moment a motor faults is the worst time to
        restart it. Families that cannot see it report nothing.
        """
        return {}

    @classmethod
    def supported_baudrates(cls) -> list[int]:
        """Baud rates this family accepts, highest first."""
        return sorted(cls.baud_rate_map, reverse=True)

    @staticmethod
    def probe(port: str, baudrate: int, motor_ids: Sequence[int]) -> bool:
        """Return True if motors of this family answer on ``port`` at ``baudrate``.

        Used by connect-time driver resolution; must not enable torque or
        change motor state. Clients that cannot probe report False.
        """
        return False

    # ----- Provisioning (assigning IDs and baud rates) ----------------------
    #
    # Optional: only clients that can re-program motors implement these. They
    # are what orca_core.maintenance.motor_chain drives during hand assembly.

    def scan_for_motors(self, port: str, id_range: tuple, baud_rates: "list | None" = None) -> list:
        """Ping ``id_range`` at each of ``baud_rates``.

        Returns:
            One dict per motor found, with ``id``, ``baud_rate`` and
            ``model_name``.
        """
        raise NotImplementedError(f"{type(self).__name__} cannot scan for motors")

    def change_motor_id(self, current_id: int, new_id: int) -> bool:
        """Re-program a motor's ID. Returns True on success."""
        raise NotImplementedError(f"{type(self).__name__} cannot change motor IDs")

    def change_motor_baudrate(self, motor_id: int, new_baud_rate: int) -> bool:
        """Re-program a motor's baud rate. Returns True on success."""
        raise NotImplementedError(f"{type(self).__name__} cannot change motor baud rates")

    def change_return_delay_time(self, motor_id: int, delay_us: int) -> bool:
        """Re-program a motor's Return Delay Time. Returns True on success."""
        raise NotImplementedError(f"{type(self).__name__} cannot change the return delay time")

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Returns True if the client is connected to the motors."""
        ...

    @abstractmethod
    def connect(self) -> None:
        """Connects to the motors."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnects from the motors."""
        ...

    @abstractmethod
    def set_torque_enabled(
        self,
        motor_ids: Sequence[int],
        enabled: bool,
        retries: int = 3,
        retry_interval: float = 0.25
    ) -> "list[int]":
        """Sets whether torque is enabled for the specified motors.

        Unacked motors are reported, never raised: implementations must
        return the failing IDs (and log them) so callers decide whether the
        toggle was best-effort or must be acted on.

        Args:
            motor_ids: The motor IDs to configure.
            enabled: Whether to engage or disengage the motors.
            retries: The number of times to retry after the first attempt.
                0 means a single attempt; <0 retries forever.
            retry_interval: The number of seconds to wait between retries.

        Returns:
            The motor IDs that could not be set; an empty list means every
            motor acknowledged the change.
        """
        ...

    @abstractmethod
    def set_operating_mode(self, motor_ids: Sequence[int], mode: int) -> None:
        """Sets the operating mode for the specified motors.

        Mode changes require torque off; implementations must not write mode
        registers for motors whose torque-disable was not acknowledged —
        those motors are logged and skipped.

        Args:
            motor_ids: The motor IDs to configure.
            mode: The operating mode value:
                0: current control mode
                1: velocity control mode
                3: position control mode
                4: multi-turn position control mode
                5: current-based position control mode
        """
        ...

    @abstractmethod
    def read_position_velocity_current(self) -> MotorRead:
        """Read the current position, velocity, and current for all motors.

        Returns:
            A :class:`MotorRead` snapshot. Positions are in radians,
            velocities in rad/s, currents in mA.
        """
        ...

    @property
    def last_read_ok(self) -> bool:
        """Whether the most recent :meth:`read_position_velocity_current`
        returned fresh data for every motor.

        A failed bus read keeps returning the stale cache, so callers must
        discard or retry samples taken while this is ``False``. A motor's own
        latched fault flags (overload, overheat) say nothing about freshness:
        they are logged and the data they arrived with stands, so a routine
        that stalls the motors on purpose still reads them.

        The flag is client-global mutable state, overwritten by every read
        from any thread: it qualifies only the single most recent read on
        this client, never a specific :class:`MotorRead` snapshot. Check it
        immediately after the read it describes, before any other read can
        run — i.e. under the same lock that serialized that read. Any
        interleaved read (another thread, or a second read of your own)
        rebinds the flag to different data.

        Clients with no failure signal report ``True``; their reads are
        authoritative.
        """
        return True

    @abstractmethod
    def read_temperature(self) -> np.ndarray:
        """Reads the current temperature for all motors.

        Returns:
            An array of temperatures in degrees Celsius.
        """
        ...

    @abstractmethod
    def read_hardware_error(self, motor_id: int) -> "int | None":
        """Read one motor's latched hardware-error byte.

        Returns:
            The raw error byte (0 when the motor answered without a fault), or
            ``None`` when the motor did not answer. Doubles as a per-motor
            liveness probe after a bus-wide read has failed.
        """
        ...

    @abstractmethod
    def write_desired_pos(
        self,
        motor_ids: Sequence[int],
        positions: np.ndarray
    ) -> None:
        """Writes desired positions to the specified motors.

        Args:
            motor_ids: The motor IDs to write to.
            positions: The desired positions in radians.
        """
        ...

    @abstractmethod
    def write_desired_current(
        self,
        motor_ids: Sequence[int],
        currents: np.ndarray
    ) -> None:
        """Set each motor's goal-current limit, in mA.

        Values are quantized to the register and clamped to the motor's
        ceiling (see :meth:`read_current_limits`); motors without a current
        register are skipped. Negative or non-finite values raise
        ``ValueError`` before anything is written.

        Args:
            motor_ids: The motor IDs to write to.
            currents: The desired current limits in mA.
        """
        ...

    def read_current_limits(self) -> "dict[int, float | None]":
        """Per-motor ceiling for the goal current, in mA.

        ``None`` marks a motor with no current register. Families with a
        per-motor EEPROM limit read it from the bus here, so cache the result
        instead of calling this in a loop.
        """
        return {motor_id: self._current_ceiling_ma(motor_id) for motor_id in self.motor_ids}

    def _current_ceiling_ma(self, motor_id: int) -> "float | None":
        """Cached ceiling for one motor: the family maximum unless a subclass knows better."""
        return self.max_current_ma

    def _goal_current_plan(
        self, motor_ids: Sequence[int], currents_ma: Sequence[float]
    ) -> "dict[int, int]":
        """Goal-current register values for ``currents_ma``, validated and clamped.

        Raises ``ValueError`` before touching anything when a value is negative
        or non-finite. Motors without a current register are left out; clamped
        and zero-quantized requests are logged once per call.
        """
        if len(motor_ids) != len(currents_ma):
            raise ValueError('motor_ids and currents must have the same length')
        values = [float(value) for value in currents_ma]
        if any(not math.isfinite(value) or value < 0 for value in values):
            raise ValueError('current limits must be non-negative finite values')

        plan: dict[int, int] = {}
        clamped, zeroed, skipped = [], [], []
        for motor_id, value in zip(motor_ids, values):
            ceiling = self._current_ceiling_ma(motor_id)
            if ceiling is None:
                skipped.append(motor_id)
                continue
            if value > ceiling:
                clamped.append(f'{motor_id}: {value:.0f} -> {ceiling:.0f} mA')
                value = ceiling
            raw = int(value / self.current_scale_ma)
            if raw == 0 and value > 0:
                zeroed.append(motor_id)
            plan[motor_id] = raw

        if skipped:
            logging.debug('No current register on motors %s; limit not applied', skipped)
        if clamped:
            logging.warning('Current limit clamped to the motor ceiling: %s', ', '.join(clamped))
        if zeroed:
            logging.warning(
                'Current limit under one register unit (%.1f mA) on motors %s; they will not move',
                self.current_scale_ma, zeroed)
        return plan

    def write_profile_velocity(
        self,
        motor_ids: Sequence[int],
        profile_velocity: np.ndarray
    ) -> None:
        """Writes the motion-profile velocity limit, in this family's raw speed units.

        Args:
            motor_ids: The motor IDs to write to.
            profile_velocity: The per-motor speed limits.
        """
        raise NotImplementedError(
            f"{type(self).__name__} cannot write a profile velocity")

    def read_status_is_done_moving(self) -> bool:
        """Returns True once every motor reports it has stopped moving."""
        raise NotImplementedError(
            f"{type(self).__name__} cannot report moving status")

    def check_connected(self) -> None:
        """Raises ``OSError`` unless the client is connected.

        Connects first when the client was built with ``lazy_connect``.
        """
        raise NotImplementedError(
            f"{type(self).__name__} does not implement check_connected")

    def wait_for_motion_complete(self, timeout: float = 5.0) -> None:
        """Block until *every* motor has settled at its commanded position.

        A recorded motion is implicitly synchronous: letting one motor start
        its next leg because it arrived first, while others are still
        travelling, is not a faster version of the same move, it is a
        different one. So this waits for the whole chain or raises.

        Settled means two things at once. The motor's moving flag is clear,
        and it is within :attr:`arrival_tolerance_rad` of its goal. The flag
        alone is not enough — a servo stalled against a load reports stopped
        short of its target.

        A family whose motion is effectively instantaneous may leave this a
        no-op and ``waits_for_motion`` False, but note that setting a
        trajectory profile makes a goal write a ramp rather than a step, so
        that is a property of how the family is being driven and not of the
        family itself.

        Raises:
            MotionTimeoutError: if any motor is still unsettled at ``timeout``.
        """

    @property
    def requires_offset_calibration(self) -> bool:
        """Returns True if this motor type needs offset calibration during joint calibration."""
        return False

    def calibrate_offset(self, motor_id: int, upper: bool = True) -> bool:
        """Set current physical position to read as upper or lower bound.

        Used during calibration to shift the position coordinate system,
        ensuring the motor's full range fits within valid bounds.

        Args:
            motor_id: Motor to calibrate.
            upper: If True, set to upper bound. If False, set to lower bound.

        Returns:
            True if the motor acknowledged the offset command. ``False``
            means the position frame was NOT shifted; callers must check
            and must not persist limits derived from an unshifted frame.
        """
        # Base implementation: no-op for motors that don't need this
        return True
