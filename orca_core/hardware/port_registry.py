# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Which serial ports this process's clients are holding.

The advisory file lock a motor client takes catches a second *process*
opening the same bus. It says nothing about two clients inside one process:
with several hands driven from one program, the second open succeeds and both
clients then interleave transactions on one bus, reading each other's
replies. This registry closes that case, and unlike a lock it can name the
owner.

Claims are weak, so a client dropped without disconnecting releases its port
instead of stranding it for the life of the process. That does mean an owner
has to be weakref-able, which every client class is.
"""

import os
import threading
from weakref import WeakValueDictionary


class PortAlreadyClaimed(OSError):
    """Raised when another client in this process already holds the port."""


_lock = threading.Lock()
_owners: "WeakValueDictionary[str, object]" = WeakValueDictionary()


def _key(port: str) -> str:
    """Normalize a device path so two names for one device collide.

    Linux ships stable ``/dev/serial/by-id/...`` symlinks alongside the
    ``/dev/ttyACM*`` they point at, and a config may pin either.
    """
    return os.path.realpath(port)


def claim(port: str, owner: object) -> None:
    """Record ``owner`` as holding ``port`` for as long as it is open.

    Raises:
        PortAlreadyClaimed: another live client in this process holds it.
    """
    key = _key(port)
    with _lock:
        current = _owners.get(key)
        if current is not None and current is not owner:
            raise PortAlreadyClaimed(
                f"{port} is already open in this process by "
                f"{type(current).__name__}. Two clients on one bus corrupt "
                f"each other's transactions; disconnect that one first, or "
                f"give this hand its own port."
            )
        _owners[key] = owner


def release(port: str, owner: object) -> None:
    """Drop ``owner``'s claim on ``port``. A claim it no longer holds is
    left alone, so releasing twice is harmless."""
    key = _key(port)
    with _lock:
        if _owners.get(key) is owner:
            del _owners[key]


def owner_of(port: str) -> object:
    """The client holding ``port`` in this process, or ``None``."""
    with _lock:
        return _owners.get(_key(port))


def claimed_ports() -> "frozenset[str]":
    """Every port currently held by a client in this process."""
    with _lock:
        return frozenset(_owners.keys())


def clear() -> None:
    """Drop every claim without touching the clients that made them.

    For tearing a process's hardware layer back down to a known state; it
    does not close anything, so a client still driving a bus keeps driving it.
    """
    with _lock:
        _owners.clear()
