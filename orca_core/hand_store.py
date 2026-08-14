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
    path component.
    """
    cleaned = "".join(
        c if c.isalnum() or c in "-_." else "_" for c in hand_id.strip()
    ).strip("._")
    if not cleaned:
        raise ValueError(f"hand id {hand_id!r} has no usable characters")
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


def resolve_calibration_path(hand_id: str, packaged_fallback: str) -> str:
    """The calibration path to use for ``hand_id``.

    The store wins. Before it holds anything, a calibration next to the
    packaged model is adopted so a hand calibrated before the store keeps its
    limits; it is copied into the store rather than written back in place.
    """
    stored = calibration_path(hand_id)
    if os.path.exists(stored):
        return stored

    legacy = read_yaml(packaged_fallback) if os.path.exists(packaged_fallback) else None
    if legacy:
        logger.info(
            "adopting the calibration beside %s for hand %s; it now lives at %s.",
            os.path.dirname(packaged_fallback), hand_id, stored,
        )
        try:
            _write(stored, legacy)
            return stored
        except OSError as e:
            logger.warning(
                "could not copy %s into the hand store (%s); using it in place.",
                packaged_fallback, e,
            )
            return packaged_fallback
    return stored


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
