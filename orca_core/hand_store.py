# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Where each physical hand's own files live on this computer.

Everything measured on, or recorded about, one physical hand is kept in one
folder per hand, outside every installed package::

    $ORCA_HOME/                     (default ~/.orca)
      hands/<hand id>/
        hand.yaml                   identity: id, aliases seen, model, nickname, first/last seen
        calibration.yaml            measurements of the last calibration
        calibration.previous.yaml   the calibration before it
        joint_ranges.yaml           joint ranges measured on this hand (optional)
        setup.yaml                  the latest setup record of each kind
        board.bin                   the board's calibration store as last read
        archive/<section>/<name>.yaml   older setup records, one file per record

A hand's id is its board's serial, else the board's own id, else, for a board
whose firmware is too old to report either, ``usb-<its USB serial number>``. A
hand with none of these (a plain USB adapter) is filed under ``noboard-<model>``,
so every such hand of one model shares a folder. Mock hands get a folder in a
temporary directory and never touch ``ORCA_HOME``.

Every file written here carries ``schema:``, which :func:`read_doc` checks,
and every write replaces the file atomically.
"""

from __future__ import annotations

import atexit
import copy
import logging
import os
import re
import shutil
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from .utils.utils import is_packaged_model_path, read_yaml, write_yaml_atomic

logger = logging.getLogger(__name__)

ORCA_HOME_ENV = "ORCA_HOME"
SCHEMA = 1
SCHEMA_KEY = "schema"

HANDS_DIR = "hands"
HAND_FILE = "hand.yaml"
CALIBRATION_FILE = "calibration.yaml"
PREVIOUS_CALIBRATION_FILE = "calibration.previous.yaml"
JOINT_RANGES_FILE = "joint_ranges.yaml"
SETUP_FILE = "setup.yaml"
BOARD_STORE_FILE = "board.bin"
ARCHIVE_DIR = "archive"

NO_BOARD_PREFIX = "noboard-"
USB_PREFIX = "usb-"
MOCK_PREFIX = "mock-"

# Per-hand files an older orca_core kept beside a config. Found beside a loaded
# config, they are reported once (see legacy_files) and never read, moved or deleted.
LEGACY_FILES = (
    "calibration.yaml",
    "setup.yaml",
    "local_config.yaml",
    "last_tension.yaml",
    "tension_progress.yaml",
    "slack_tests.yaml",
    "tension_sweeps.yaml",
    "travel_baselines.yaml",
)

_ID_RE = re.compile(r"^[0-9A-Za-z._-]{1,64}$")
_NAME_RE = re.compile(r"[^0-9A-Za-z._-]+")


class HandStoreError(ValueError):
    """A hand id or record name that cannot name a file here."""


# --- locations ----------------------------------------------------------------


_resolved_homes: Dict[str, Path] = {}


def orca_home() -> Path:
    """The root of everything orca keeps for this user: ``$ORCA_HOME``, else ``~/.orca``.

    ``~`` is expanded, and a relative ``$ORCA_HOME`` is made absolute against the
    folder this process first read it in (with a warning), so a later change of
    working folder never moves the hands' files.
    """
    configured = os.environ.get(ORCA_HOME_ENV)
    if not configured:
        return Path.home() / ".orca"
    home = _resolved_homes.get(configured)
    if home is None:
        home = Path(configured).expanduser()
        if not home.is_absolute():
            home = home.absolute()
            logger.warning("%s=%s is a relative path; using %s", ORCA_HOME_ENV, configured, home)
        _resolved_homes[configured] = home
    return home


def _hands_dir() -> Path:
    return orca_home() / HANDS_DIR


def valid_hand_id(hand_id: Optional[str]) -> bool:
    """1-64 characters of ``[0-9A-Za-z._-]`` with at least one alphanumeric,
    so an id is always one plain folder name."""
    return (
        isinstance(hand_id, str)
        and bool(_ID_RE.match(hand_id))
        and any(c.isalnum() for c in hand_id)
        and hand_id not in (".", "..")
    )


def _check_hand_id(hand_id: str) -> str:
    if not valid_hand_id(hand_id):
        raise HandStoreError(
            f"{hand_id!r} cannot be a hand id: 1-64 characters of [0-9A-Za-z._-], "
            "at least one of them alphanumeric"
        )
    return hand_id


def hand_dir(hand_id: str) -> Path:
    """The folder of the hand filed under ``hand_id`` (not created)."""
    return _hands_dir() / _check_hand_id(hand_id)


def _slug(name: str) -> str:
    slug = _NAME_RE.sub("-", str(name)).strip("-.") or "unknown"
    return slug[:48]


def no_board_hand_id(model_name: str) -> str:
    """The id every hand of ``model_name`` that reports no identity shares."""
    return NO_BOARD_PREFIX + _slug(model_name)


def _mock_hand_id(model_name: str) -> str:
    return MOCK_PREFIX + _slug(model_name)


def resolve_hand_id(serial: Optional[str], board_id: Optional[str],
                    usb_serial: Optional[str] = None) -> Optional[str]:
    """The id a board names its hand by: the serial, else the board id, else
    (firmware too old to report either) ``usb-<the board's USB serial number>``."""
    usb = USB_PREFIX + str(usb_serial) if usb_serial else None
    for candidate in (serial, board_id, usb):
        if candidate and valid_hand_id(str(candidate)):
            return str(candidate)
    return None


_mock_root: Optional[Path] = None


def _mock_hands_dir() -> Path:
    """A temporary folder for this process's mock hands, removed at exit.

    One per process, so the fresh mock hand an operation builds finds what the
    session's mock hand recorded; never under ``ORCA_HOME``.
    """
    global _mock_root
    if _mock_root is None or not _mock_root.is_dir():
        _mock_root = Path(tempfile.mkdtemp(prefix="orca-mock-hands-"))
        atexit.register(shutil.rmtree, str(_mock_root), True)
    return _mock_root


def _mock_hand_dir(hand_id: str) -> Path:
    return _mock_hands_dir() / _check_hand_id(hand_id)


# --- documents ------------------------------------------------------------------

def schema_of(doc: Dict[str, Any]) -> int:
    """The schema ``doc`` was written with; 0 for a document without one."""
    try:
        return int(doc.get(SCHEMA_KEY, 0) or 0)
    except (TypeError, ValueError, OverflowError):
        return 0


def _set_aside(path: os.PathLike | str, problem: str) -> None:
    """Rename an unreadable ``path`` to ``<name>.corrupt-<UTC time>`` so it reads
    as absent from now on, and say where it went."""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    aside = f"{path}.corrupt-{stamp}"
    try:
        os.replace(path, aside)
    except OSError as exc:
        logger.warning("%s cannot be read (%s) and cannot be moved aside (%s); "
                       "ignoring it", path, problem, exc)
        return
    logger.warning("%s cannot be read (%s); moved it to %s and carrying on without it",
                   path, problem, aside)


def read_doc(path: os.PathLike | str, kind: str) -> Dict[str, Any]:
    """The ``kind`` document at ``path``; ``{}`` when absent.

    A document that is not a YAML mapping is moved aside (see :func:`_set_aside`)
    and reads as absent. A document from a newer orca_core than this one is
    returned as it is, with a warning: its readers here may miss what it added,
    and :func:`write_doc` keeps its schema and the keys a caller passes back.
    """
    try:
        raw = read_yaml(str(path)) or {}
    except (yaml.YAMLError, UnicodeDecodeError):
        _set_aside(path, "not valid YAML")
        return {}
    if not isinstance(raw, dict):
        _set_aside(path, "not a mapping")
        return {}
    version = schema_of(raw)
    if version > SCHEMA:
        logger.warning(
            "%s was written with schema %d by a newer orca_core (this one reads %d); "
            "what that version added is not understood here", path, version, SCHEMA)
    # A schema change adds its step here: bring a ``kind`` document of ``version`` forward.
    return raw


def _refuse_packaged(path: os.PathLike | str) -> None:
    """Raises PermissionError for ``path`` inside the installed package's models:
    what one hand records never goes where every hand of a model reads."""
    if is_packaged_model_path(path):
        raise PermissionError(
            f"refusing to write {path}: a hand's files belong in its own folder, "
            "never inside the installed package")


def write_doc(path: os.PathLike | str, doc: Dict[str, Any]) -> Path:
    """Replace ``path`` with ``doc`` (``schema:`` first), atomically, creating its
    folder. A schema newer than this orca_core's, read with ``doc``, is kept.

    Raises:
        PermissionError: ``path`` is inside the installed package.
    """
    _refuse_packaged(path)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    body = {SCHEMA_KEY: max(SCHEMA, schema_of(doc))}
    body.update({k: v for k, v in doc.items() if k != SCHEMA_KEY})
    write_yaml_atomic(str(path), body)
    return path


def now_iso() -> str:
    """The current UTC time, to the second, as every stamp here is written."""
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


# --- hand.yaml --------------------------------------------------------------------


def read_hand(folder: os.PathLike | str) -> Dict[str, Any]:
    """The ``hand.yaml`` in ``folder``; ``{}`` when there is none."""
    return read_doc(Path(folder) / HAND_FILE, "hand")


def _seen(previous: Any, value: Optional[str]) -> List[str]:
    seen = [str(v) for v in previous] if isinstance(previous, list) else []
    if value and str(value) not in seen:
        seen.append(str(value))
    return seen


def record_contact(
    folder: os.PathLike | str,
    hand_id: str,
    *,
    model: Optional[str] = None,
    serial: Optional[str] = None,
    board_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Create or update ``hand.yaml`` for a hand just seen; returns what was written.

    Every serial and board id the hand has answered to is kept under ``aliases``,
    so a hand re-filed under a better name still says what it was called before.
    """
    folder = Path(folder)
    doc = read_hand(folder)
    stamp = now_iso()
    aliases = doc.get("aliases") if isinstance(doc.get("aliases"), dict) else {}
    doc.update({
        "id": hand_id,
        "aliases": {
            "serial": _seen(aliases.get("serial"), serial),
            "board_id": _seen(aliases.get("board_id"), board_id),
        },
        "model": model or doc.get("model"),
        "nickname": doc.get("nickname"),
        "first_seen": doc.get("first_seen") or stamp,
        "last_seen": stamp,
    })
    write_doc(folder / HAND_FILE, doc)
    return doc


def set_nickname(folder: os.PathLike | str, nickname: Optional[str]) -> Dict[str, Any]:
    """Give the hand in ``folder`` a display name (``None`` or blank clears it)."""
    folder = Path(folder)
    doc = read_hand(folder)
    nickname = (nickname or "").strip() or None
    if nickname is not None and (len(nickname) > 64 or any(ord(c) < 32 for c in nickname)):
        raise HandStoreError("a nickname is up to 64 printable characters")
    doc["nickname"] = nickname
    write_doc(folder / HAND_FILE, doc)
    return doc


def update_hand(folder: os.PathLike | str, **values: Any) -> Dict[str, Any]:
    """Set keys of the ``hand.yaml`` in ``folder``, keeping the rest."""
    folder = Path(folder)
    doc = read_hand(folder)
    doc.update(values)
    write_doc(folder / HAND_FILE, doc)
    return doc


def known_hands() -> List[Dict[str, Any]]:
    """Every hand this computer has a folder for, as its ``hand.yaml`` (plus ``folder``)."""
    root = _hands_dir()
    try:
        entries = sorted(p for p in root.iterdir() if p.is_dir() and valid_hand_id(p.name))
    except OSError:
        return []
    hands = []
    for entry in entries:
        doc = read_hand(entry)
        doc.setdefault("id", entry.name)
        doc["folder"] = str(entry)
        hands.append(doc)
    return hands


def open_hand_folder(
    hand_id: str, *, serial: Optional[str] = None, board_id: Optional[str] = None
) -> "tuple[Path, bool]":
    """The folder for ``hand_id``, and whether this call created it.

    A hand first seen before its board had a serial is filed under its board
    id. Once the board reports a serial, that folder is renamed to the serial,
    which follows the hand even if its board is replaced. Should both folders
    exist, the serial's is used and the other is left as it is.
    """
    folder = hand_dir(hand_id)
    if folder.is_dir():
        return folder, False
    older = resolve_hand_id(None, board_id) if serial and hand_id == serial else None
    if older and older != hand_id and hand_dir(older).is_dir():
        try:
            os.rename(hand_dir(older), folder)
            logger.info("hand folder %s renamed to %s (the board now reports a serial)",
                        hand_dir(older), folder)
            return folder, False
        except OSError as exc:
            logger.warning("could not rename %s to %s (%s); starting a new folder",
                           hand_dir(older), folder, exc)
    folder.mkdir(parents=True, exist_ok=True)
    return folder, True


def find_legacy_files(directory: Optional[os.PathLike | str]) -> List[str]:
    """Per-hand files an older orca_core kept beside a model's config."""
    if not directory:
        return []
    directory = Path(directory)
    return [str(directory / name) for name in LEGACY_FILES if (directory / name).is_file()]


# --- archive --------------------------------------------------------------------


def _record_name(name: Any) -> str:
    slug = _NAME_RE.sub("_", str(name)).strip("._") or "record"
    return slug[:96]


def archive_record(folder: os.PathLike | str, section: str, name: Any, record: Any) -> Path:
    """Keep one record under ``archive/<section>/<name>.yaml``.

    Never overwrites a different record: a name already taken by other content
    gets a numbered suffix. Writing the identical record again is a no-op.
    """
    target_dir = Path(folder) / ARCHIVE_DIR / _record_name(section)
    base = _record_name(name)
    doc = {"name": str(name), "record": record}
    for index in range(1000):
        candidate = target_dir / (f"{base}.yaml" if index == 0 else f"{base}-{index + 1}.yaml")
        if not candidate.exists():
            return write_doc(candidate, doc)
        existing = read_doc(candidate, "archive")
        if existing.get("name") == str(name) and existing.get("record") == record:
            return candidate
    raise HandStoreError(f"too many archived records named {name!r} in {section}")


# Per archived file: the (inode, mtime_ns, size) it was parsed at, and its content.
_archived: Dict[str, tuple] = {}


def _read_archived(path: Path) -> Dict[str, Any]:
    """The archived record at ``path``, parsed again only once the file changed."""
    try:
        stat = path.stat()
    except OSError:
        return {}
    version = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
    cached = _archived.get(str(path))
    if cached is None or cached[0] != version:
        cached = (version, read_doc(path, "archive"))
        _archived[str(path)] = cached
    return cached[1]


def read_archive(folder: os.PathLike | str, section: str, name: Any = None) -> Any:
    """Archived records of ``section``: ``{name: record}`` for all of them, or
    the one record called ``name`` (``None`` when there is none). Where a name
    was archived more than once, the last one written wins."""
    target_dir = Path(folder) / ARCHIVE_DIR / _record_name(section)

    def written_order(path: Path):
        stem, _, number = path.stem.rpartition("-")
        index = int(number) if stem and number.isdigit() else 1
        return path.stat().st_mtime_ns, index

    try:
        paths = sorted(target_dir.glob("*.yaml"), key=written_order)
    except OSError:
        paths = []
    records: Dict[str, Any] = {}
    for path in paths:
        doc = _read_archived(path)
        if "name" in doc:
            records[str(doc["name"])] = doc.get("record")
    if name is not None:
        records = {str(name): records.get(str(name))}
    records = copy.deepcopy(records)  # the cache's own records stay as read
    return records if name is None else records[str(name)]
