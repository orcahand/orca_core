"""Typed containers for tactile sensor readings."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from orca_core.hardware.sensing.constants import FingerName


@dataclass(frozen=True)
class ResultantReading:
    """Resultant force per finger from a single auto-stream frame.

    Supports dict-style access: ``reading["thumb"]`` returns ``[fx, fy, fz]``.
    """

    forces: dict[str, list[float]]
    timestamp: float | None = None

    def __getitem__(self, finger: FingerName) -> list[float]:
        return self.forces[finger]

    def __contains__(self, finger: FingerName) -> bool:
        return finger in self.forces

    @property
    def fingers(self) -> list[str]:
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

    taxels: dict[str, list[list[float]]]
    timestamp: float | None = None

    def __getitem__(self, finger: FingerName) -> list[list[float]]:
        return self.taxels[finger]

    def __contains__(self, finger: FingerName) -> bool:
        return finger in self.taxels

    @property
    def fingers(self) -> list[str]:
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
