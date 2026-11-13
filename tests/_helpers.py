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


def copy_model_config(src, dst) -> None:
    """Write ``src``'s effective config (its ``shared.yaml`` merged in) to ``dst``:
    a standalone copy that loads the same with no ``shared.yaml`` beside it."""
    from pathlib import Path

    import yaml

    from orca_core.hand_config import load_config

    Path(dst).write_text(yaml.safe_dump(load_config(str(src)), sort_keys=False), encoding="utf-8")
