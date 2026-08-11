"""Typed containers for tactile sensor and joint encoder readings, plus the
serial-link health snapshot. All reading timestamps are ``time.monotonic()``
receive times: comparable across the tactile and encoder streams, not
wall-clock."""

import time
from dataclasses import dataclass

import numpy as np

from orca_core.constants import FingerName
from orca_core.hardware.hand_serial_link import LinkStats


ForceVector = list[float]
"""A 3-axis force ``[fx, fy, fz]`` in Newtons. fx/fy are signed (shear), fz
is unsigned (normal). Mutable so callers can apply zeroing offsets in place."""

FingerTaxels = list[ForceVector]
"""Per-taxel forces on one finger: ``[[fx, fy, fz], ...]``."""

ResultantForces = dict[FingerName, ForceVector]
"""``{finger: [fx, fy, fz]}`` resultant force per finger."""

TaxelForces = dict[FingerName, FingerTaxels]
"""``{finger: [[fx, fy, fz], ...]}`` per-taxel forces per finger."""


@dataclass(frozen=True)
class ResultantReading:
    """Resultant force per finger from a single auto-stream frame.

    Supports dict-style access: ``reading["thumb"]`` returns ``[fx, fy, fz]``.
    ``timestamp`` is the ``time.monotonic()`` frame receive time.
    """

    forces: ResultantForces
    timestamp: float | None = None

    def __getitem__(self, finger: FingerName) -> ForceVector:
        return self.forces[finger]

    def __contains__(self, finger: FingerName) -> bool:
        return finger in self.forces

    @property
    def fingers(self) -> list[FingerName]:
        return list(self.forces.keys())

    def as_array(self) -> np.ndarray:
        """Return an ``(n_fingers, 3)`` array, rows in finger-name order."""
        return np.array([self.forces[f] for f in sorted(self.forces)])


@dataclass(frozen=True)
class TaxelReading:
    """Per-taxel forces from a single auto-stream frame.

    Supports dict-style access: ``reading["thumb"]`` returns
    ``[[fx, fy, fz], ...]`` for every taxel on that finger.
    ``timestamp`` is the ``time.monotonic()`` frame receive time.
    """

    taxels: TaxelForces
    timestamp: float | None = None

    def __getitem__(self, finger: FingerName) -> FingerTaxels:
        return self.taxels[finger]

    def __contains__(self, finger: FingerName) -> bool:
        return finger in self.taxels

    @property
    def fingers(self) -> list[FingerName]:
        return list(self.taxels.keys())

    def as_array(self, finger: FingerName) -> np.ndarray:
        """Return an ``(n_taxels, 3)`` array for *finger*."""
        return np.array(self.taxels[finger])


@dataclass(frozen=True)
class TactileReading:
    """Atomic snapshot of resultant + per-taxel forces from a single frame.

    Either field may be ``None`` if the matching stream mode is disabled.
    Use this when you need forces and taxels guaranteed to come from the
    same frame (one lock acquisition, one ``time.monotonic()`` timestamp).
    """

    forces: ResultantReading | None
    taxels: TaxelReading | None
    timestamp: float | None = None


@dataclass(frozen=True)
class TactileZeroBaseline:
    """Per-taxel rest statistics from the frames a zeroing capture averaged.

    ``means`` are the raw pre-zero readings that were applied as offsets, and
    ``std`` the spread of the same frames — a noisy taxel is a bad connection.
    ``max_abs_fz`` is the largest ``|mean fz|`` on each finger: a standing
    force at rest, consuming part of the sensor's usable range.
    """

    num_samples: int
    means: TaxelForces
    std: TaxelForces
    max_abs_fz: dict[FingerName, float]


@dataclass(frozen=True)
class EncoderReading:
    """Decoded encoder auto-stream frame plus a ``time.monotonic()`` receive timestamp.

    ``raw_counts`` is the u16 per slot with the parity + err bits still on it.
    ``parity_ok`` is the AS5048A even-parity check over the word as received
    and ``angle_error`` is bit 14; use these rather than re-deriving them from
    ``raw_counts``, which is smoothed on a ``get_latest()`` reading.
    """

    raw_counts: np.ndarray
    parity_ok: np.ndarray
    angle_error: np.ndarray
    error_byte: int
    timestamp: float

    @property
    def freshness_ms(self) -> float:
        return (time.monotonic() - self.timestamp) * 1000.0


@dataclass(frozen=True)
class TaxelData:
    """Per-taxel positions and forces for one finger, expressed in a common frame.

    Row ``i`` of ``positions`` (meters) and ``forces`` (Newtons) describe the
    same taxel. ``frame`` names the coordinate frame both are expressed in
    (see ``orca_core.kinematics.frames``). ``timestamp`` is the
    ``time.monotonic()`` receive time of the source tactile frame.
    """

    finger: str
    frame: str
    positions: np.ndarray
    forces: np.ndarray
    timestamp: float | None = None

    @property
    def num_taxels(self) -> int:
        return len(self.positions)


@dataclass(frozen=True)
class LinkHealth:
    """Health snapshot of one hand serial link.

    ``port_dead`` latches ``True`` after a hard port failure (e.g. USB
    unplug) — the link cannot recover and must be reconnected —
    with ``port_error`` describing the failure. ``stats`` are the link's
    demuxer counters at snapshot time.
    """

    connected: bool
    port_dead: bool
    port_error: str | None
    stats: LinkStats
