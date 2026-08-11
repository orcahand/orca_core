"""A recording session: one directory, one manifest, one drain.

The manifest is written as soon as the directory exists, marked ``running``, and
rewritten at the end with how the run actually finished. A dataset still marked
``running`` was cut short — the process died, the machine slept, someone pulled
the cable — and the reader can say so instead of presenting a truncated table as
a complete one.
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

from .drain import Drain
from .sink import CsvRowSink


logger = logging.getLogger(__name__)

STATUS_RUNNING = "running"
STATUS_OK = "ok"
STATUS_ABORTED = "aborted"
STATUS_CRASHED = "crashed"


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    """Replace a JSON file in one step, so a reader never sees a partial one."""
    handle, temp_path = tempfile.mkstemp(dir=str(path.parent), suffix=".tmp")
    try:
        with os.fdopen(handle, "w") as file:
            json.dump(payload, file, indent=2, sort_keys=True, default=str)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temp_path, path)
    except BaseException:
        Path(temp_path).unlink(missing_ok=True)
        raise


class RecordingSession:
    """Owns a dataset directory, its tables, and the thread that fills them.

    Use as a context manager: leaving normally closes the manifest as ``ok``,
    leaving via an exception closes it as ``crashed``, and an explicit
    :meth:`abort` closes it as ``aborted``.
    """

    def __init__(
        self,
        directory: Path | str,
        *,
        experiment: str,
        metadata: Optional[Dict[str, Any]] = None,
        drain_interval_s: Optional[float] = None,
    ):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.experiment = experiment
        self.meta: Dict[str, Any] = dict(metadata or {})
        self.meta.update(
            experiment=experiment,
            status=STATUS_RUNNING,
            started_wall=time.time(),
            started_mono=time.monotonic(),
            tables={},
        )
        self._drain = (
            Drain(drain_interval_s) if drain_interval_s is not None else Drain()
        )
        self._sinks: Dict[str, CsvRowSink] = {}
        self._sources: Dict[str, Any] = {}
        self._events_path = self.directory / "events.jsonl"
        self._events = self._events_path.open("w")
        self._closed = False
        self._flush_manifest()

    @property
    def manifest_path(self) -> Path:
        return self.directory / "meta.json"

    def add_table(self, name: str, ring, columns: Sequence[str], source=None) -> None:
        """Register a table. ``source`` is the recorder whose drop counters are
        folded into the manifest at close, so a lossy run says so.

        Registration completes or leaves nothing behind: a table half-added and
        then abandoned would make the manifest disagree with the sinks, and
        ``close`` would fail on the mismatch instead of on the original problem.
        """
        if self._closed:
            raise RuntimeError("session is closed")
        if name in self._sinks:
            raise ValueError(f"table {name!r} already registered")
        sink = CsvRowSink(self.directory / f"{name}.csv", columns)
        try:
            self._drain.add(ring, sink)
        except BaseException:
            sink.close()
            raise
        self._sinks[name] = sink
        self._sources[name] = source
        self.meta["tables"][name] = {"columns": list(columns)}

    def record_event(self, kind: str, **payload: Any) -> None:
        """Append one timestamped state change.

        Anything that changes mid-run and cannot be a column belongs here — a
        gain that was retuned, a waypoint that was reached, an abort. The
        manifest is written once and cannot carry them.
        """
        self._events.write(
            json.dumps(
                {"t": time.monotonic(), "wall": time.time(), "event": kind, **payload},
                default=str,
            )
            + "\n"
        )
        self._events.flush()

    def start(self) -> "RecordingSession":
        self._drain.start()
        return self

    def close(self, status: str = STATUS_OK, reason: Optional[str] = None) -> None:
        """Stop draining, close the tables, and finalise the manifest."""
        if self._closed:
            return
        self._closed = True
        drained_cleanly = self._drain.stop()
        self._events.close()

        self.meta.update(
            status=status,
            ended_wall=time.time(),
            ended_mono=time.monotonic(),
            drain_clean=drained_cleanly,
            drain_write_failures=self._drain.write_failures,
        )
        if reason is not None:
            self.meta["status_reason"] = reason
        for name, sink in self._sinks.items():
            entry = self.meta["tables"][name]
            entry["rows"] = sink.rows_written
            entry["file"] = sink.path.name
            source = self._sources.get(name)
            if source is not None:
                entry["dropped"] = getattr(source, "dropped", 0)
                for counter in ("stale", "unpublished", "unknown_phases"):
                    if hasattr(source, counter):
                        entry[counter] = getattr(source, counter)
        self._flush_manifest()

    def abort(self, reason: str) -> None:
        self.record_event("abort", reason=reason)
        self.close(status=STATUS_ABORTED, reason=reason)

    def _flush_manifest(self) -> None:
        _write_json_atomic(self.manifest_path, self.meta)

    def __enter__(self) -> "RecordingSession":
        return self.start()

    def __exit__(self, exc_type, exc, traceback) -> bool:
        if exc_type is None:
            self.close(STATUS_OK)
        elif self._closed:
            pass
        else:
            self.close(STATUS_CRASHED, reason=f"{exc_type.__name__}: {exc}")
        return False
