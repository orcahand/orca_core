# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Per-hand state, kept outside the packaged models.

A model directory in the wheel describes what an ``orcahand-full-right``
*is*; it is read-only and shared by every hand of that model. Calibration is
the opposite: it belongs to one physical hand, and writing it beside the
model means two hands of a kind overwrite each other, a wheel upgrade
deletes it, and a read-only install cannot record it at all.

So each hand gets a directory under ``${ORCA_HOME:-~/.orca}/hands/``, keyed
by the identity its controller board reports::

    ~/.orca/hands/ser-0001/
        identity.yaml     what this hand is, and when it was last seen
        calibration.yaml  this hand's calibration

Hands that predate the store keep working: a calibration sitting beside the
model is still read when the store has none, and is copied in the first time
this hand writes one.
"""

from __future__ import annotations

import hashlib
import logging
import os

from .utils.utils import read_yaml, write_yaml_atomic


logger = logging.getLogger(__name__)

ORCA_HOME_ENV = "ORCA_HOME"
"""Overrides where per-hand state lives. Useful to keep a rig's hands out of
the invoking user's home directory."""

_DEFAULT_HOME = "~/.orca"


def orca_home() -> str:
    """The directory holding per-hand state."""
    return os.path.abspath(
        os.path.expanduser(os.environ.get(ORCA_HOME_ENV) or _DEFAULT_HOME)
    )


def hands_root() -> str:
    """The directory holding one subdirectory per known hand."""
    return os.path.join(orca_home(), "hands")


def _safe_id(hand_id: str) -> str:
    """A hand id reduced to something usable as a directory name.

    Ids come from a board's own reply, so they are not trusted to be a safe
    path component. Rewriting one is lossy — ``ser 1`` and ``ser/1`` reduce
    alike — so a rewritten id carries a digest of the original and only ids
    that survive untouched keep naming their directory exactly as they read.
    """
    stripped = hand_id.strip()
    cleaned = "".join(
        c if c.isalnum() or c in "-_." else "_" for c in stripped
    ).strip("._")
    if not cleaned:
        raise ValueError(f"hand id {hand_id!r} has no usable characters")
    if cleaned != stripped:
        digest = hashlib.blake2s(stripped.encode("utf-8"), digest_size=4).hexdigest()
        return f"{cleaned}-{digest}"
    return cleaned


def _write(path: str, data: dict) -> None:
    """Atomically write ``data``, creating the hand's directory if needed."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    write_yaml_atomic(path, data)


def hand_dir(hand_id: str) -> str:
    """This hand's directory. Not created until something is written."""
    return os.path.join(hands_root(), _safe_id(hand_id))


def calibration_path(hand_id: str) -> str:
    """Where this hand's calibration belongs, whether or not it exists yet."""
    return os.path.join(hand_dir(hand_id), "calibration.yaml")


def _stored_identities() -> "list[tuple[str, dict]]":
    """Every hand directory in the store, with what it recorded about itself."""
    root = hands_root()
    if not os.path.isdir(root):
        return []
    found = []
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name, "identity.yaml")
        if os.path.isfile(path):
            found.append((name, read_yaml(path) or {}))
    return found


def _adopted_by(source: str) -> "str | None":
    """The store directory already claiming ``source`` as its origin."""
    for name, identity in _stored_identities():
        if identity.get("adopted_from") == source:
            return name
    return None


def _adopt(source: str, hand_id: str, reason: str) -> "str | None":
    """Copy ``source`` into ``hand_id``'s store slot and record where it came
    from. Returns the new path, or ``None`` if the store could not be written.
    """
    data = read_yaml(source)
    if not data:
        return None
    stored = calibration_path(hand_id)
    try:
        _write(stored, data)
    except OSError as e:
        logger.warning(
            "could not copy %s into the hand store (%s); using it in place.",
            source, e,
        )
        return None
    logger.warning(
        "hand %s adopted the calibration at %s (%s); it now lives at %s. "
        "No other hand will adopt that file.",
        hand_id, source, reason, stored,
    )
    record_identity(hand_id, {"adopted_from": os.path.realpath(source)})
    return stored


def _same_board_calibration(hand_id: str, board_id: "str | None") -> "str | None":
    """A calibration this same board recorded under an earlier hand id.

    A board reports its board id until provisioning gives it a serial, at
    which point its hand id changes and its calibration would otherwise be
    orphaned.
    """
    if not board_id:
        return None
    here = _safe_id(hand_id)
    candidates = [
        name for name, identity in _stored_identities()
        if identity.get("board_id") == board_id and name != here
        and os.path.isfile(os.path.join(hands_root(), name, "calibration.yaml"))
    ]
    if len(candidates) > 1:
        logger.warning(
            "%d store directories (%s) record board %s and hold a calibration, "
            "so none can be attributed to hand %s. Delete the stale ones.",
            len(candidates), ", ".join(candidates), board_id, hand_id,
        )
        return None
    if not candidates:
        return None
    return os.path.join(hands_root(), candidates[0], "calibration.yaml")


def resolve_calibration_path(
    hand_id: str, packaged_fallback: str, board_id: "str | None" = None
) -> str:
    """The calibration path to use for ``hand_id``.

    The store wins. Behind it, in order: a calibration this same board wrote
    under an earlier hand id, then one sitting beside the packaged model.
    Both are copied into this hand's slot rather than used in place, and both
    are claimed — a calibration measured on one hand must never be handed to a
    second, which would report itself calibrated and skip its hardstop sweep.
    """
    stored = calibration_path(hand_id)
    if os.path.exists(stored):
        return stored

    earlier = _same_board_calibration(hand_id, board_id)
    if earlier is not None:
        return _adopt(earlier, hand_id, f"same board {board_id}") or earlier

    if not os.path.exists(packaged_fallback):
        return stored
    claimed_by = _adopted_by(os.path.realpath(packaged_fallback))
    if claimed_by is not None:
        logger.warning(
            "the calibration at %s was measured on hand %s, so hand %s does "
            "not inherit it and starts uncalibrated.",
            packaged_fallback, claimed_by, hand_id,
        )
        return stored
    return _adopt(
        packaged_fallback, hand_id, "it predates the per-hand store"
    ) or packaged_fallback


def record_identity(hand_id: str, fields: dict) -> None:
    """Note what this hand is, so its directory is identifiable by hand.

    Best-effort: a rig with a read-only home still calibrates, it just keeps
    no record of which hand the directory belongs to.
    """
    path = os.path.join(hand_dir(hand_id), "identity.yaml")
    existing = (read_yaml(path) or {}) if os.path.exists(path) else {}
    merged = {**existing, **{k: v for k, v in fields.items() if v is not None}}
    if merged == existing:
        return
    try:
        _write(path, merged)
    except OSError as e:
        logger.debug("could not record identity for %s: %s", hand_id, e)


def known_hands() -> "list[str]":
    """Every hand this machine has stored state for, whether attached or not."""
    root = hands_root()
    if not os.path.isdir(root):
        return []
    return sorted(
        name for name in os.listdir(root)
        if os.path.isdir(os.path.join(root, name))
    )
