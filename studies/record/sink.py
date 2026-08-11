"""Row sinks: where drained rows go.

The reader keys on the column names recorded in the manifest, not on the file
format, so swapping a sink changes nothing downstream.
"""

from __future__ import annotations

import csv
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Sequence

import numpy as np

from .schema import CODE_TO_PHASE, PHASE_COLUMN


class RowSink(ABC):
    """Destination for batches of rows in a fixed column order."""

    @abstractmethod
    def write_rows(self, rows: np.ndarray) -> None:
        """Append rows. Called from the drain thread, never the producer."""

    @abstractmethod
    def close(self) -> None:
        """Flush and release. Idempotent."""


class CsvRowSink(RowSink):
    """Streaming CSV, one file per table.

    Phase codes are written as their names: the cost is a dict lookup per row,
    and it buys a file that can be read with ``head`` during a bench session
    without decoding anything.
    """

    def __init__(self, path: Path | str, columns: Sequence[str]):
        self._path = Path(path)
        self._columns = list(columns)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self._path.open("w", newline="")
        self._writer = csv.writer(self._handle)
        self._writer.writerow(self._columns)
        self._handle.flush()
        self._phase_index = (
            self._columns.index(PHASE_COLUMN) if PHASE_COLUMN in self._columns else None
        )
        self._rows_written = 0
        self._closed = False

    @property
    def path(self) -> Path:
        return self._path

    @property
    def rows_written(self) -> int:
        return self._rows_written

    def write_rows(self, rows: np.ndarray) -> None:
        if self._closed:
            raise RuntimeError(f"sink for {self._path.name} is closed")
        if rows.size == 0:
            return
        if rows.shape[1] != len(self._columns):
            raise ValueError(
                f"expected {len(self._columns)} columns, got {rows.shape[1]}"
            )
        phase_index = self._phase_index
        for row in rows:
            values = row.tolist()
            if phase_index is not None:
                values[phase_index] = CODE_TO_PHASE.get(int(values[phase_index]), "?")
            self._writer.writerow(values)
        self._rows_written += rows.shape[0]
        self._handle.flush()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._handle.close()


class ListRowSink(RowSink):
    """In-memory sink. Used by tests that assert on what a recorder produced."""

    def __init__(self, columns: Sequence[str]):
        self.columns = list(columns)
        self.batches: list[np.ndarray] = []
        self._closed = False

    @property
    def rows(self) -> np.ndarray:
        if not self.batches:
            return np.empty((0, len(self.columns)), dtype=np.float64)
        return np.vstack(self.batches)

    @property
    def rows_written(self) -> int:
        return int(sum(batch.shape[0] for batch in self.batches))

    def write_rows(self, rows: np.ndarray) -> None:
        if self._closed:
            raise RuntimeError("sink is closed")
        if rows.size:
            self.batches.append(np.array(rows, copy=True))

    def close(self) -> None:
        self._closed = True
