# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Abstract base class for motor communication clients."""

import logging
from abc import ABC, abstractmethod
from typing import ClassVar, NamedTuple, Sequence
import numpy as np


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

    requires_unpowered_hotplug: ClassVar[bool] = False
    """Whether the bus must be de-powered before a motor is plugged in.

    Families that latch their ID on power-up cannot be hot-plugged onto a live
    bus; chain assembly power-cycles the adapter between motors when this is set.
    """

    @classmethod
    def supported_baudrates(cls) -> list[int]:
        """Baud rates this family accepts, highest first."""
        return sorted(cls.baud_rate_map, reverse=True)

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
        discard or retry samples taken while this is ``False``.

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
        """Writes desired currents (torque limits) to the specified motors.

        Args:
            motor_ids: The motor IDs to write to.
            currents: The desired currents in mA.
        """
        ...

    def wait_for_motion_complete(self, timeout: float = 5.0) -> None:
        """Block until all motors finish their commanded motion.

        Default implementation is a no-op for motor families that respond
        fast enough that callers don't need to wait (e.g., Dynamixel, mock).
        Subclasses that actually block (e.g., Feetech) must set
        ``waits_for_motion = True`` and override this to poll a per-motor
        moving flag, raising ``MotionTimeoutError`` on timeout.
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
