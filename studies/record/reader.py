"""Reading a recorded dataset back.

Tables are read by the column names in the manifest, never by position, so a
table that gains a column stays readable.

Joining two tables recorded at different rates is the one place a slow value
meets a fast row, and it is deliberately awkward: every joined column arrives
with the age of the sample it came from, and there is no interpolation. A value
sampled every few hundred milliseconds carries no information about the instants
between, and a reader that quietly invented one would put a straight line
through a gap and let a fit be run on it.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Dict, Sequence

import numpy as np

from .schema import PHASE_COLUMN, PHASE_TO_CODE


class Table:
    """One recorded table: named columns over rows."""

    def __init__(self, columns: Sequence[str], data: np.ndarray):
        self.columns = list(columns)
        self.data = data
        self._index = {name: i for i, name in enumerate(self.columns)}

    def __len__(self) -> int:
        return int(self.data.shape[0])

    def __contains__(self, column: str) -> bool:
        return column in self._index

    def __getitem__(self, column: str) -> np.ndarray:
        try:
            return self.data[:, self._index[column]]
        except KeyError:
            raise KeyError(
                f"no column {column!r}; have {', '.join(self.columns)}"
            ) from None

    def joints(self, prefix: str) -> list[str]:
        """Joint names present under a per-joint prefix, in column order."""
        start = f"{prefix}_"
        return [c[len(start):] for c in self.columns if c.startswith(start)]

    def block(self, prefix: str) -> np.ndarray:
        """All columns of one per-joint prefix as an (n_rows, n_joints) array."""
        names = [c for c in self.columns if c.startswith(f"{prefix}_")]
        if not names:
            raise KeyError(f"no columns with prefix {prefix!r}")
        return np.column_stack([self[name] for name in names])

    def where(self, column: str, value) -> "Table":
        return Table(self.columns, self.data[self[column] == value])


def read_table(path: Path | str) -> Table:
    """Read one CSV table. Phase names are converted back to their codes."""
    path = Path(path)
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        columns = next(reader)
        rows = [
            [
                PHASE_TO_CODE.get(value, -1)
                if column == PHASE_COLUMN
                else (float(value) if value != "" else np.nan)
                for column, value in zip(columns, row)
            ]
            for row in reader
        ]
    data = (
        np.array(rows, dtype=np.float64)
        if rows
        else np.empty((0, len(columns)), dtype=np.float64)
    )
    return Table(columns, data)


class Dataset:
    """A recorded run: its manifest and its tables."""

    def __init__(self, directory: Path | str):
        self.directory = Path(directory)
        manifest_path = self.directory / "meta.json"
        if not manifest_path.exists():
            raise FileNotFoundError(f"no meta.json in {self.directory}")
        self.meta = json.loads(manifest_path.read_text())
        self._tables: Dict[str, Table] = {}

    @property
    def status(self) -> str:
        return self.meta.get("status", "unknown")

    @property
    def truncated(self) -> bool:
        """Whether the run never closed its manifest, so the tables stop wherever
        the process died."""
        return self.status not in ("ok", "aborted")

    def table(self, name: str) -> Table:
        if name not in self._tables:
            self._tables[name] = read_table(self.directory / f"{name}.csv")
        return self._tables[name]

    def events(self) -> list[dict]:
        path = self.directory / "events.jsonl"
        if not path.exists():
            return []
        return [
            json.loads(line)
            for line in path.read_text().splitlines()
            if line.strip()
        ]


def join_asof(
    base: Table,
    other: Table,
    columns: Sequence[str],
    *,
    on: str = "t",
    tolerance_s: float,
) -> Table:
    """Attach columns from a slower table to a faster one.

    Each base row takes the most recent ``other`` row at or before its own
    timestamp — never a later one, which would carry information backwards in
    time. Every attached column is accompanied by ``<column>_age_s``, and rows
    whose nearest sample is older than ``tolerance_s`` get NaN rather than a
    stale value passed off as current.
    """
    if tolerance_s <= 0:
        raise ValueError("tolerance_s must be positive")
    missing = [c for c in columns if c not in other]
    if missing:
        raise KeyError(f"columns not in the joined table: {missing}")

    base_t = base[on]
    other_t = other[on]
    # searchsorted on the right, minus one, is the last sample at or before t.
    position = np.searchsorted(other_t, base_t, side="right") - 1
    valid = position >= 0
    safe = np.where(valid, position, 0)

    age = np.where(valid, base_t - other_t[safe], np.inf)
    usable = valid & (age <= tolerance_s)

    data = [base.data]
    names = list(base.columns)
    for column in columns:
        values = np.where(usable, other[column][safe], np.nan)
        data.append(values[:, None])
        data.append(np.where(usable, age, np.nan)[:, None])
        names.extend([column, f"{column}_age_s"])
    return Table(names, np.hstack(data))
