"""Where a camera rig keeps its own files on this computer::

    <ORCA_HOME>/rigs/<rig id>/              (default ~/.orca, orca_core.hand_store.orca_home)
      rig.yaml                          the board's USB serial, for a board with a rig ID
      calibration.yaml                  the solver's lens, camera, axis and tower (steps 1a, 1b)
      turntable_calibration.yaml        the motor calibration
      sessions/<id>/                    the photo sessions of the rig steps (orca-ui)
      runs/<hand id>/<YYYYMMDD-HHMMSS>/   one hand measured on the rig (hand_study)

The rig id is the rig board's identity serial when that is a rig ID (``rig-...``), else the
board's USB serial. The rig's static config (``config.yaml``, ``turntable.yaml``) stays in
the repo (rig_config).
"""
from __future__ import annotations

import re
from pathlib import Path

from orca_core.hand_store import orca_home
from orca_core.utils.utils import read_yaml, write_yaml_atomic

RIGS_DIR = "rigs"
RIG_FILE = "rig.yaml"
_ID_RE = re.compile(r"^[0-9A-Za-z._-]{1,128}$")


class RigIdError(ValueError):
    pass


def is_rig_identity(serial) -> bool:
    """The identity serial of a board provisioned as a rig (``rig-<n>``)."""
    return isinstance(serial, str) and serial.lower().startswith("rig")


def rigs_root() -> Path:
    return orca_home() / RIGS_DIR


def rig_dir(rig_id: str) -> Path:
    """The folder of the rig filed under ``rig_id`` (not created)."""
    if not isinstance(rig_id, str) or not _ID_RE.match(rig_id) or not any(c.isalnum() for c in rig_id):
        raise RigIdError(f"{rig_id!r} cannot be a rig id: 1-128 characters of [0-9A-Za-z._-]")
    return rigs_root() / rig_id


def rig_id(usb_serial: str | None = None, identity: str | None = None) -> str:
    """The rig's id: its board's rig ID; else the rig whose rig.yaml names this USB serial (a board
    with a rig ID, seen before); else the USB serial itself."""
    if is_rig_identity(identity):
        return str(identity)
    if not usb_serial:
        raise RigIdError("the rig has no id: pin its board's usb_serial in turntable.yaml, or connect the rig")
    for path in sorted(rigs_root().glob(f"*/{RIG_FILE}")):
        if (read_yaml(path) or {}).get("usb_serial") == usb_serial:
            return path.parent.name
    return str(usb_serial)


def remember(usb_serial: str | None, identity: str | None) -> str:
    """The id of a rig board seen now. A board with a rig ID gets its USB serial recorded in its
    folder, so the rig is found from the USB serial pinned in turntable.yaml as well."""
    rid = rig_id(usb_serial, identity)
    if is_rig_identity(identity) and usb_serial:
        path = rig_dir(rid) / RIG_FILE
        if (read_yaml(path) or {}).get("usb_serial") != usb_serial:
            path.parent.mkdir(parents=True, exist_ok=True)
            write_yaml_atomic(path, {"id": rid, "usb_serial": usb_serial})
    return rid
