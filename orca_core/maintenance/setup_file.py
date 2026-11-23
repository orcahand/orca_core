# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""``setup.yaml``: what the setup routines recorded about a hand, in one file
in the hand's own folder (see :mod:`orca_core.hand_store`).

One top-level section per kind of record (``slack_tests``, ``tension_sweeps``,
``travel_baselines``, ``tension_progress``, ``last_tension``). Each write re-reads
the file and replaces it whole while holding a lock on ``setup.yaml.lock`` beside
it, so two writers, in one process or several, never lose each other's sections.

The file keeps only the latest record of each kind, plus the one marked as the
reference (the reference sweep, the active travel baseline). Every other record
moves to ``archive/<section>/<name>.yaml`` as it drops out, one file per record,
and every sweep's full data is archived as it is saved; :func:`read_archive`
reads them back.

The console reads sections on every hand-info refresh, so a parse is kept until
the file changes on disk.
"""

from __future__ import annotations

import contextlib
import copy
import errno
import logging
import os
import threading
import time
from typing import Any, Callable, Iterator, Optional


from .. import hand_store

logger = logging.getLogger(__name__)

SETUP_FILE = "setup.yaml"
LOCK_SUFFIX = ".lock"
LOCK_TIMEOUT_S = 10.0
_LOCK_POLL_S = 0.05
# What a non-blocking lock attempt raises while another process holds the lock.
_CONTENDED = {errno.EACCES, errno.EAGAIN, errno.EWOULDBLOCK, errno.EDEADLK}

# Guards the parse cache; held only briefly, never while waiting for another process.
_lock = threading.RLock()
_writers: dict[str, "_Writer"] = {}
_parsed: dict[str, tuple[tuple[int, int, int], dict]] = {}
"""Per path: the (inode, mtime_ns, size) it was parsed at, and its content. Every
write replaces the file, so the inode changes even where mtime is coarse. Callers
hold ``_lock`` and never hand out or mutate the cached dict itself."""


def _setup_dir(config) -> str:
    """The folder holding ``config``'s hand's setup records: its hand folder."""
    folder = getattr(config, "hand_dir", "") or ""
    # Never a relative folder: that would be wherever the process happens to run.
    if not folder or not os.path.isabs(folder):
        raise ValueError(
            "this config belongs to no hand folder yet: connect the hand (or pass a "
            "hand's config) before reading or writing its setup records")
    return folder


def setup_path(config) -> str:
    return os.path.join(_setup_dir(config), SETUP_FILE)


def read_archive(config, section: str, name: Any = None) -> Any:
    """Archived records of ``section`` for ``config``'s hand: ``{name: record}``,
    or the one record called ``name`` (``None`` if there is none). Sections with
    two kinds of record archive them as ``travel_baselines-snapshots`` and
    ``travel_baselines-baselines``."""
    return hand_store.read_archive(_setup_dir(config), section, name)


def _newest(records: dict) -> Optional[str]:
    if not records:
        return None
    return max(records, key=lambda key: (str((records[key] or {}).get("recorded_at") or ""), str(key)))


def _keep_latest(
    value: Any,
    collection: str,
    *,
    pinned: tuple = (),
    also=None,
    archive_all: bool = False,
) -> "tuple[Any, list]":
    """``value`` with ``value[collection]`` cut to its newest record plus the
    ``pinned`` names (and ``also(records)``), and the records to archive."""
    if not isinstance(value, dict) or not isinstance(value.get(collection), dict):
        return value, []
    records = value[collection]
    keep = {_newest(records), *pinned}
    if also is not None:
        keep.add(also(records))
    kept = {k: r for k, r in records.items() if k in keep}
    archive = list(records.items()) if archive_all else [
        (k, r) for k, r in records.items() if k not in keep]
    return {**value, collection: kept}, archive


def _newest_full_snapshot(records: dict) -> Optional[str]:
    return _newest({k: r for k, r in records.items() if not (r or {}).get("partial")})


def _prune(name: str, value: Any) -> "tuple[Any, list]":
    """``value`` as setup.yaml keeps it, and ``[(archive section, name, record)]``."""
    if name == "slack_tests":
        value, out = _keep_latest(value, "tests")
        return value, [(name, k, r) for k, r in out]
    if name == "tension_sweeps":
        reference = value.get("reference") if isinstance(value, dict) else None
        value, out = _keep_latest(value, "sweeps", pinned=(reference,), archive_all=True)
        return value, [(name, k, r) for k, r in out]
    if name == "travel_baselines":
        active = value.get("active") if isinstance(value, dict) else None
        value, baselines = _keep_latest(value, "baselines", pinned=(active,))
        value, snapshots = _keep_latest(value, "snapshots", also=_newest_full_snapshot)
        return value, ([(f"{name}-baselines", k, r) for k, r in baselines]
                       + [(f"{name}-snapshots", k, r) for k, r in snapshots])
    return value, []


class _Writer:
    """One path's writers in this process: a thread lock and how deeply it is held."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.depth = 0


def _try_lock(fd: int) -> bool:
    """True once locked, False while another process holds the lock; raises
    OSError when the file system cannot lock at all."""
    os.lseek(fd, 0, os.SEEK_SET)
    try:
        if os.name == "nt":
            import msvcrt

            msvcrt.locking(fd, msvcrt.LK_NBLCK, 1)
        else:
            import fcntl

            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as error:
        if error.errno in _CONTENDED:
            return False
        raise
    return True


def _unlock(fd: int) -> None:
    os.lseek(fd, 0, os.SEEK_SET)
    if os.name == "nt":
        import msvcrt

        msvcrt.locking(fd, msvcrt.LK_UNLCK, 1)
    else:
        import fcntl

        fcntl.flock(fd, fcntl.LOCK_UN)


def _acquire(path: str) -> Optional[int]:
    """The locked descriptor of ``path``'s sidecar lock file, or None when it
    cannot be opened or locked here (the write then goes ahead unlocked)."""
    lock_path = os.path.realpath(path) + LOCK_SUFFIX
    try:
        # Read-only: locking needs no write access, so a lock file another user created still works.
        fd = os.open(lock_path, os.O_RDONLY | os.O_CREAT, 0o666)
    except OSError as error:
        logger.warning("cannot open %s (%s); saving without the cross-process lock", lock_path, error)
        return None
    deadline = time.monotonic() + LOCK_TIMEOUT_S
    try:
        while not _try_lock(fd):
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"another process kept {path} locked for {LOCK_TIMEOUT_S:.0f} s; "
                    "close the other program writing this hand's setup records and retry"
                )
            time.sleep(_LOCK_POLL_S)
    except TimeoutError:
        os.close(fd)
        raise
    except OSError as error:
        os.close(fd)
        logger.warning("cannot lock %s (%s); saving without the cross-process lock", lock_path, error)
        return None
    return fd


@contextlib.contextmanager
def _locked(path: str) -> Iterator[None]:
    """Hold ``path``'s writer lock in this process and an advisory lock on its
    sidecar lock file across processes; raises TimeoutError rather than wait forever,
    and PermissionError for a path inside the installed package, before any lock file."""
    hand_store._refuse_packaged(path)
    with _lock:
        writer = _writers.setdefault(path, _Writer())
    with writer.lock:
        if writer.depth:
            yield
            return
        fd = _acquire(path)
        writer.depth = 1
        try:
            yield
        finally:
            writer.depth = 0
            if fd is not None:
                _unlock(fd)
                os.close(fd)


def _read(path: str, fresh: bool = False) -> dict:
    """``path`` parsed, from the cache unless the file changed or ``fresh`` is set."""
    try:
        stat = os.stat(path)
    except FileNotFoundError:
        _parsed.pop(path, None)
        return {}
    version = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
    cached = _parsed.get(path)
    if not fresh and cached is not None and cached[0] == version:
        return cached[1]
    data = hand_store.read_doc(path, "setup")
    _parsed[path] = (version, data)
    return data


def _store(path: str, data: dict, name: str, value: Any) -> Any:
    """Write ``value`` as section ``name``, archiving what drops out of it
    first so a crash between the two never loses a record. Returns what was kept."""
    data = dict(data)
    if value is None:
        data.pop(name, None)
    else:
        value, archive = _prune(name, value)
        folder = os.path.dirname(path)
        for section, record_name, record in archive:
            hand_store.archive_record(folder, section, record_name, record)
        data[name] = value
    hand_store.write_doc(path, data)
    return value


def read_section(config, name: str) -> Any:
    """The ``name`` section, or None when it has never been written."""
    with _lock:
        return copy.deepcopy(_read(setup_path(config)).get(name))


def write_section(config, name: str, value: Any) -> None:
    """Replace the ``name`` section; ``None`` removes it."""
    path = setup_path(config)
    with _locked(path), _lock:
        _store(path, _read(path, fresh=True), name, value)


def update_section(config, name: str, fn: Callable[[Any], Any]) -> Any:
    """Read the section, pass it to ``fn`` and store what ``fn`` returns, as
    one step under the lock; returns the stored value."""
    path = setup_path(config)
    with _locked(path):
        with _lock:
            data = _read(path, fresh=True)
        value = fn(copy.deepcopy(data.get(name)))
        with _lock:
            return _store(path, data, name, value)
