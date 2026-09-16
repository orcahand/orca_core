"""Plain helpers shared across test modules.

These live here rather than in conftest.py because pytest loads conftest as a
plugin, not as a library, so importing from it is discouraged.
"""

import time
from types import SimpleNamespace


def wait_until(predicate, timeout: float = 1.0) -> None:
    """Spin on ``predicate`` until true; lets async link dispatch settle."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError(f"predicate not satisfied within {timeout}s")


def fake_serial_port(device: str, vid: int):
    """Build a fake serial.tools.list_ports.ListPortInfo-like object."""
    return SimpleNamespace(device=device, vid=vid, description="fake")
