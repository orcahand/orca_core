"""Fixed-capacity row buffer between a producer on a timing-critical thread and
a consumer that writes to disk.

One producer and one consumer only. The producer fills a row and then advertises
it, so the consumer never reads a half-written row. Neither side takes a lock:
the producer must not block, and a lock shared with a drain that is inside a
file write would do exactly that.

An overflow drops the row being offered rather than overwriting an unread one,
so what survives is a contiguous run and the loss shows up as a gap in the
producer's own sequence column.
"""

from __future__ import annotations

import numpy as np


class SampleRing:
    """Ring of fixed-width float64 rows."""

    def __init__(self, capacity: int, num_columns: int):
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        if num_columns <= 0:
            raise ValueError("num_columns must be positive")
        self._buffer = np.zeros((capacity, num_columns), dtype=np.float64)
        self._capacity = int(capacity)
        self._num_columns = int(num_columns)
        self._written = 0
        self._read = 0
        self._dropped = 0

    @property
    def capacity(self) -> int:
        return self._capacity

    @property
    def num_columns(self) -> int:
        return self._num_columns

    @property
    def dropped(self) -> int:
        """Rows the producer offered while the ring was full."""
        return self._dropped

    @property
    def pending(self) -> int:
        return self._written - self._read

    def push(self, row: np.ndarray) -> bool:
        """Offer one row. Returns False if the ring was full and it was lost."""
        if self._written - self._read >= self._capacity:
            self._dropped += 1
            return False
        self._buffer[self._written % self._capacity, :] = row
        self._written += 1
        return True

    def drain(self) -> np.ndarray:
        """Take every row advertised so far as a copy, oldest first."""
        written = self._written
        available = written - self._read
        if available <= 0:
            return np.empty((0, self._num_columns), dtype=np.float64)

        start = self._read % self._capacity
        end = start + available
        if end <= self._capacity:
            out = self._buffer[start:end].copy()
        else:
            split = self._capacity - start
            out = np.empty((available, self._num_columns), dtype=np.float64)
            out[:split] = self._buffer[start:]
            out[split:] = self._buffer[: available - split]
        self._read = written
        return out
