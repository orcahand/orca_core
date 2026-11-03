"""Typed containers for tactile sensor and joint encoder readings, plus the
serial-link health snapshot. All reading timestamps are ``time.monotonic()``
receive times: comparable across the tactile and encoder streams, not
wall-clock."""

import time
import warnings
from dataclasses import dataclass
from functools import cached_property

import numpy as np

from orca_core.constants import FINGER_NAMES, FingerName
from orca_core.hardware.hand_serial_link import LinkStats
from orca_core.hardware.sensing.constants import (
    DEFAULT_TAXEL_COUNTS,
    FINGERTIP_ERROR_GONE,
    FINGERTIP_ERROR_RESULTANT_READ,
    FINGERTIP_ERROR_TAXEL_READ,
)


ForceVector = list[float]
"""A 3-axis force ``[fx, fy, fz]`` in Newtons. fx/fy are signed (shear), fz
is unsigned (normal). Mutable so callers can apply zeroing offsets in place."""

FingerTaxels = list[ForceVector]
"""Per-taxel forces on one finger: ``[[fx, fy, fz], ...]``, each in its own
taxel's local axes (fz along the taxel's outward surface normal, fx/fy tangent
to the surface). ``TaxelGeometry.rotate_forces`` maps them into the sensor frame."""

def _taxel_slices() -> dict[FingerName, slice]:
    slices, start = {}, 0
    for finger in FINGER_NAMES:
        slices[finger] = slice(start, start + DEFAULT_TAXEL_COUNTS[finger])
        start += DEFAULT_TAXEL_COUNTS[finger]
    return slices


TAXEL_SLICES: dict[FingerName, slice] = _taxel_slices()
"""Each finger's rows in ``TaxelReading.array``, thumb → pinky."""

NUM_TAXELS = TAXEL_SLICES[FINGER_NAMES[-1]].stop
"""Rows in ``TaxelReading.array``: every taxel on a full hand."""

ResultantForces = dict[FingerName, ForceVector]
"""``{finger: [fx, fy, fz]}`` resultant force per finger."""

TaxelForces = dict[FingerName, FingerTaxels]
"""``{finger: [[fx, fy, fz], ...]}`` per-taxel forces per finger."""


@dataclass(frozen=True)
class ResultantReading:
    """Resultant force per finger from a single auto-stream frame.

    Supports dict-style access: ``reading["thumb"]`` returns ``[fx, fy, fz]``.
    ``timestamp`` is the ``time.monotonic()`` frame receive time. ``seq``
    numbers the frames the host received since the stream started: a
    repeat means the same frame, a jump means frames were skipped or dropped.

    The sensor reports fz up to 25.5 N. ``saturated`` names the fingers whose
    fz was at that ceiling before zeroing: their real force is at least that.
    """

    forces: ResultantForces
    timestamp: float | None = None
    seq: int | None = None
    saturated: frozenset[FingerName] = frozenset()

    def __getitem__(self, finger: FingerName) -> ForceVector:
        return self.forces[finger]

    def __contains__(self, finger: FingerName) -> bool:
        return finger in self.forces

    @property
    def fingers(self) -> list[FingerName]:
        return list(self.forces.keys())

    def as_array(self) -> np.ndarray:
        """Return an ``(n_fingers, 3)`` array, rows in finger-name order.

        Deprecated: rows shift when a finger is absent. Use :attr:`array`.
        """
        warnings.warn(
            "ResultantReading.as_array() is deprecated: its rows are alphabetical and "
            "shift when a finger is absent. Use .array (fixed rows thumb → pinky).",
            DeprecationWarning,
            stacklevel=2,
        )
        return np.array([self.forces[f] for f in sorted(self.forces)])

    @cached_property
    def array(self) -> np.ndarray:
        """Read-only ``(5, 3)`` forces, rows thumb → pinky, NaN for an absent finger."""
        out = np.full((len(FINGER_NAMES), 3), np.nan)
        for row, finger in enumerate(FINGER_NAMES):
            if finger in self.forces:
                out[row] = self.forces[finger]
        out.setflags(write=False)
        return out


@dataclass(frozen=True)
class TaxelReading:
    """Per-taxel forces from a single auto-stream frame.

    Supports dict-style access: ``reading["thumb"]`` returns
    ``[[fx, fy, fz], ...]`` for every taxel on that finger, each in that
    taxel's local axes (see ``FingerTaxels``).
    ``timestamp`` and ``seq`` as on :class:`ResultantReading`.
    """

    taxels: TaxelForces
    timestamp: float | None = None
    seq: int | None = None

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

    @cached_property
    def array(self) -> np.ndarray:
        """Read-only ``(NUM_TAXELS, 3)`` forces for the whole hand.

        Finger rows sit at ``TAXEL_SLICES[finger]``, thumb → pinky. A finger
        that is absent, or reports a taxel count other than its sensor
        model's, reads NaN.
        """
        out = np.full((NUM_TAXELS, 3), np.nan)
        for finger, rows in TAXEL_SLICES.items():
            values = self.taxels.get(finger)
            if values is not None and len(values) == rows.stop - rows.start:
                out[rows] = values
        out.setflags(write=False)
        return out


@dataclass(frozen=True)
class TactileReading:
    """Atomic snapshot of resultant + per-taxel forces from a single frame.

    Either field may be ``None`` if the matching stream mode is disabled.
    Use this when you need forces and taxels guaranteed to come from the
    same frame (one lock acquisition, one ``time.monotonic()`` timestamp,
    one ``seq``).
    """

    forces: ResultantReading | None
    taxels: TaxelReading | None
    timestamp: float | None = None
    seq: int | None = None


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
    same taxel. Forces are rotated out of each taxel's local axes, so all rows
    share one set of axes. ``frame`` names the coordinate frame both are expressed in
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


@dataclass(frozen=True)
class SensingBoardVersion:
    """What register 0x0000 says about the board carrying the tactile stream.

    ``kind`` is ``"connector"`` for ORCA connector firmware 1.0 and later
    (``release`` is ``(major, minor)``, ``None`` if the string doesn't parse),
    ``"connector_legacy"`` for connector firmware before 1.0 (it reads zeros),
    ``"paxini"`` for a Paxini comm board's own string, ``"unknown"`` otherwise.
    """

    kind: str
    text: str = ""
    release: tuple[int, int] | None = None
    commit: str = ""
    dirty: bool = False

    def at_least(self, release: tuple[int, int]) -> bool:
        """True on ORCA connector firmware of ``release`` or newer."""
        return self.kind == "connector" and self.release is not None and self.release >= release


@dataclass(frozen=True)
class FingertipHealth:
    """One fingertip's error report and bus counters, from connector firmware 1.0+.

    ``error_code`` is the live report (0 = fine); the counters run since the
    board last booted, and errors count only while the fingertip is connected.
    """

    error_code: int
    polls_ok: int
    timeouts: int
    bad_frames: int
    disconnects: int

    @property
    def gone(self) -> bool:
        """The fingertip was connected and stopped answering."""
        return bool(self.error_code & FINGERTIP_ERROR_GONE)

    @property
    def resultant_read_failed(self) -> bool:
        return bool(self.error_code & FINGERTIP_ERROR_RESULTANT_READ)

    @property
    def taxel_read_failed(self) -> bool:
        return bool(self.error_code & FINGERTIP_ERROR_TAXEL_READ)
