"""Wrap-aware low-pass smoothing for the joint-encoder count stream.

The stream's noise spans its full bandwidth, so a consumer polling at display
rates cannot filter it without aliasing. Only the stream client sees every
frame, so the smoothing belongs there.
"""
import math

import numpy as np

from orca_core.hardware.sensing.constants import (
    AUTO_ENC_ANGLE_MASK,
    ENCODER_COUNTS_PER_REV,
)

_HALF_REV = ENCODER_COUNTS_PER_REV // 2
_FLAG_BITS = ~AUTO_ENC_ANGLE_MASK & 0xFFFF


class EncoderCountFilter:
    """Per-slot one-pole low-pass over the 14-bit angle field.

    Filters the shortest signed distance between samples, so it is correct
    across the 16383->0 seam. The coefficient comes from each frame's own
    arrival interval, holding the cutoff under a jittery frame rate and
    snapping to the new sample after a long gap. State is kept in fractional
    counts; only the returned word is rounded.
    """

    def __init__(self, cutoff_hz: float):
        self._state: np.ndarray | None = None
        self._last_timestamp: float = 0.0
        self.cutoff_hz = cutoff_hz

    @property
    def cutoff_hz(self) -> float:
        return self._cutoff_hz

    @cutoff_hz.setter
    def cutoff_hz(self, value: float) -> None:
        value = float(value)
        if not value > 0.0 or not math.isfinite(value):
            raise ValueError(f"cutoff_hz must be finite and positive, got {value}")
        self._cutoff_hz = value

    def reset(self) -> None:
        """Drop the state so the next sample passes through and reseeds it."""
        self._state = None

    def update(
        self,
        raw_counts: np.ndarray,
        timestamp: float,
        valid: np.ndarray | None = None,
    ) -> np.ndarray:
        """Fold one frame in and return the smoothed u16 words.

        Parity and angle-error bits pass through untouched. Slots ``False``
        in ``valid`` hold their state instead of ingesting the sample.
        """
        raw_counts = np.asarray(raw_counts)
        angle = raw_counts.astype(np.int64) & AUTO_ENC_ANGLE_MASK

        if self._state is None or self._state.shape != angle.shape:
            self._state = angle.astype(np.float64)
        else:
            dt = max(timestamp - self._last_timestamp, 0.0)
            alpha = 1.0 - math.exp(-dt * 2.0 * math.pi * self._cutoff_hz)
            delta = ((angle - self._state + _HALF_REV) % ENCODER_COUNTS_PER_REV) - _HALF_REV
            step = alpha * delta
            if valid is not None:
                step = np.where(valid, step, 0.0)
            self._state = (self._state + step) % ENCODER_COUNTS_PER_REV
        self._last_timestamp = timestamp

        smoothed = np.rint(self._state).astype(np.int64) % ENCODER_COUNTS_PER_REV
        return (smoothed | (raw_counts.astype(np.int64) & _FLAG_BITS)).astype(np.uint16)
