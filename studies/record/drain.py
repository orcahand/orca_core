"""Background drain from rings to sinks.

Keeps file I/O off the threads that produce rows. One drain serves any number of
ring/sink pairs, so a run with a loop table, a frame table and a link table
still has exactly one writer thread.
"""

from __future__ import annotations

import logging
import threading
from typing import List, Tuple

from .ring import SampleRing
from .sink import RowSink


logger = logging.getLogger(__name__)

DEFAULT_DRAIN_INTERVAL_S = 0.1


class Drain:
    """Moves rows from rings to sinks on its own thread."""

    def __init__(self, interval_s: float = DEFAULT_DRAIN_INTERVAL_S):
        self._pairs: List[Tuple[SampleRing, RowSink]] = []
        self._interval_s = float(interval_s)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._write_failures = 0

    @property
    def write_failures(self) -> int:
        return self._write_failures

    def add(self, ring: SampleRing, sink: RowSink) -> None:
        """Register a table. Legal while draining: the list is replaced rather
        than mutated, so a sweep already under way finishes on the list it
        started with and picks the new table up next time round."""
        self._pairs = self._pairs + [(ring, sink)]

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="StudyDrain", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 5.0) -> bool:
        """Stop draining, take whatever is left, and close every sink.

        Returns whether the thread exited cleanly. The final sweep runs either
        way, so a hung thread costs the rows still in flight, not the file.
        """
        if self._thread is not None:
            self._stop.set()
            self._thread.join(timeout=timeout)
            clean = not self._thread.is_alive()
            self._thread = None
        else:
            clean = True
        self._sweep()
        for _, sink in self._pairs:
            sink.close()
        return clean

    def _run(self) -> None:
        while not self._stop.wait(self._interval_s):
            self._sweep()

    def _sweep(self) -> None:
        for ring, sink in self._pairs:  # snapshot; add() replaces the list
            rows = ring.drain()
            if rows.size == 0:
                continue
            try:
                sink.write_rows(rows)
            except Exception:
                # Losing a batch must not take the drain down: the run is still
                # producing, and a truncated table is recoverable where a dead
                # writer is not.
                self._write_failures += 1
                logger.exception("sink write failed; %d rows lost", rows.shape[0])
