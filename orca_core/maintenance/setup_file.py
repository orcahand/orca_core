# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""``setup.yaml``: everything the setup routines record about a hand, in one
file beside its ``calibration.yaml``.

One top-level section per kind of record (``slack_tests``, ``sweeps``,
``travel_baselines``, ``tension_progress``, ``last_tension``); records inside
are keyed by the hand they were made on where that matters. Each write re-reads
the file and replaces it whole while holding a lock on ``setup.yaml.lock`` beside
it, so two writers, in one process or several, never lose each other's sections.

A section missing from ``setup.yaml`` is read from the separate file it used
to live in (``legacy``), so existing data keeps working; the next write of
that section moves it into ``setup.yaml``. The old file is left in place.

The file grows with every recorded sweep, and the console reads sections on
every hand-info refresh, so a parse is kept until the file changes on disk.
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

import yaml

from ..utils.utils import read_yaml, write_yaml_atomic

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
# libyaml's loader where PyYAML was built with it: the same result, several times faster.
_LOADER = getattr(yaml, "CSafeLoader", yaml.SafeLoader)
_parsed: dict[str, tuple[tuple[int, int, int], dict]] = {}
"""Per path: the (inode, mtime_ns, size) it was parsed at, and its content. Every
write replaces the file, so the inode changes even where mtime is coarse. Callers
hold ``_lock`` and never hand out or mutate the cached dict itself."""


def setup_path(config) -> str:
    return os.path.join(os.path.dirname(config.calibration_path), SETUP_FILE)


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
    sidecar lock file across processes; raises TimeoutError rather than wait forever."""
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
    with open(path) as file:
        data = yaml.load(file, Loader=_LOADER)
    data = data if isinstance(data, dict) else {}
    _parsed[path] = (version, data)
    return data


def _section(config, data: dict, name: str, legacy: Optional[str]) -> Any:
    if name in data:
        return copy.deepcopy(data[name])
    if legacy:
        old = os.path.join(os.path.dirname(config.calibration_path), legacy)
        if os.path.exists(old):
            return read_yaml(old)
    return None


def _store(path: str, data: dict, name: str, value: Any) -> None:
    data = dict(data)
    if value is None:
        data.pop(name, None)
    else:
        data[name] = value
    write_yaml_atomic(path, data)


def read_section(config, name: str, legacy: Optional[str] = None) -> Any:
    """The ``name`` section, or the content of the ``legacy`` file beside it
    while the section has never been written; None when neither exists."""
    with _lock:
        return _section(config, _read(setup_path(config)), name, legacy)


def write_section(config, name: str, value: Any) -> None:
    """Replace the ``name`` section; ``None`` removes it."""
    path = setup_path(config)
    with _locked(path), _lock:
        _store(path, _read(path, fresh=True), name, value)


def update_section(
    config, name: str, fn: Callable[[Any], Any], legacy: Optional[str] = None
) -> Any:
    """Read the section, pass it to ``fn`` and store what ``fn`` returns, as
    one step under the lock; returns the stored value."""
    path = setup_path(config)
    with _locked(path):
        with _lock:
            data = _read(path, fresh=True)
        value = fn(_section(config, data, name, legacy))
        with _lock:
            _store(path, data, name, value)
        return value
