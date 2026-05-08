"""Typed containers for tactile sensor and joint encoder readings."""

import time
from dataclasses import dataclass

import numpy as np

from orca_core.constants import FingerName


FingerForces = list[float]
"""Resultant 3-axis force on one finger: ``[fx, fy, fz]`` in Newtons."""

FingerTaxels = list[list[float]]
"""Per-taxel 3-axis forces on one finger: ``[[fx, fy, fz], ...]`` in Newtons."""


@dataclass(frozen=True)
class ResultantReading:
    """Resultant force per finger from a single auto-stream frame.

    Supports dict-style access: ``reading["thumb"]`` returns ``[fx, fy, fz]``.
    """

    forces: dict[FingerName, FingerForces]
    timestamp: float | None = None

    def __getitem__(self, finger: FingerName) -> FingerForces:
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
    """

    taxels: dict[FingerName, FingerTaxels]
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
    same frame (one lock acquisition, one timestamp).
    """

    forces: ResultantReading | None
    taxels: TaxelReading | None
    timestamp: float | None = None


@dataclass(frozen=True)
class EncoderReading:
    """Decoded encoder auto-stream frame plus a ``time.monotonic()`` receive timestamp.

    ``raw_counts`` is the unmodified u16 from the wire (parity + err bits
    not stripped). ``parity_ok`` is the result of the AS5048A even-parity
    check across the full 16-bit word. ``angle_error`` is bit 14 (the chip's own error flag).
    """

    raw_counts: np.ndarray
    parity_ok: np.ndarray
    angle_error: np.ndarray
    err_byte: int
    timestamp: float

    @property
    def freshness_ms(self) -> float:
        return (time.monotonic() - self.timestamp) * 1000.0
