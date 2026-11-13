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
    from orca_core import write_standalone_config

    write_standalone_config(str(dst), config_path=str(src))


def hand_calibration_file(config_dir):
    """The calibration file a mock hand built from ``config_dir/config.yaml`` uses:
    its hand folder's, never one beside the config. The folder is created so a
    test can seed the file before building the hand."""
    import os
    from pathlib import Path

    from orca_core import hand_store

    name = os.path.basename(os.path.abspath(str(config_dir)))
    folder = hand_store._mock_hand_dir(hand_store._mock_hand_id(name))
    folder.mkdir(parents=True, exist_ok=True)
    return Path(folder) / hand_store.CALIBRATION_FILE
