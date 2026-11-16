# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""``calibration.yaml``: what a calibration measured, and which one it was.

The file holds measurements only: motor limits, measured travel and drive
currents, encoder anchors and measured spans, plus whatever other routines keep
there (tactile zero). The joint-to-motor ratio is derived when the file is
loaded, from the limits and the joint ranges in force then, so a calibration
taken under other ranges still maps correctly. Each joint's ``fingerprint``
says which motor and directions its limits were taken against.

``sequence`` counts calibrations of this hand on this computer and
``calibrated_at`` says when the last one ran. A new calibration first moves the
file it replaces to ``calibration.previous.yaml``, the one step of undo.
"""

from __future__ import annotations

import logging
import os
from typing import Any, Dict, Mapping, Optional

from .calibration import FINGERPRINT, CalibrationResult, joint_encoder_calibration_to_yaml
from .constants import (
    CALIBRATED,
    DRIVE_CURRENT_MEASURED,
    JOINT_ENCODER_CALIBRATION,
    JOINT_ROMS_MEASURED,
    JOINT_TO_MOTOR_RATIOS,
    MOTOR_LIMITS_DICT,
    MOTOR_TRAVEL_MEASURED,
    WRIST_CALIBRATED,
)
from .hand_store import (
    CALIBRATION_FILE,
    PREVIOUS_CALIBRATION_FILE,
    SCHEMA,
    SCHEMA_KEY,
    _refuse_packaged,
    now_iso,
    read_doc,
    schema_of,
    write_doc,
)
from .utils.utils import write_text_atomic

logger = logging.getLogger(__name__)

SEQUENCE = "sequence"
CALIBRATED_AT = "calibrated_at"
SOURCE = "source"
# Set while a calibration run is still writing; left set by a run that stopped.
IN_PROGRESS = "in_progress"
KIND = "calibration"
ROM_FRAME = "rom_frame"
# What one calibration is: replaced whole when another calibration is adopted.
CALIBRATION_KEYS = frozenset({
    MOTOR_LIMITS_DICT, WRIST_CALIBRATED, CALIBRATED, MOTOR_TRAVEL_MEASURED,
    DRIVE_CURRENT_MEASURED, JOINT_ENCODER_CALIBRATION, JOINT_ROMS_MEASURED,
    JOINT_TO_MOTOR_RATIOS, FINGERPRINT, SEQUENCE, CALIBRATED_AT, SOURCE, IN_PROGRESS,
    ROM_FRAME,
})
# The tactile zero: the results of one zero run on this computer, kept or replaced whole.
TACTILE_ZERO_KEYS = ("sensor_offsets", "resultant_offsets", "taxel_noise_gates")


def _not_calibration(doc: Mapping[str, Any]) -> Dict[str, Any]:
    """What other routines keep in the file (the tactile zero): this computer's,
    so restoring or adopting a calibration never changes it."""
    return {k: v for k, v in doc.items() if k not in CALIBRATION_KEYS and k != SCHEMA_KEY}


def _kept_here(path: str) -> Dict[str, Any]:
    """:func:`_not_calibration` of the file at ``path``. From a newer orca_core's
    file only the tactile zero, whose format does not change with the schema."""
    if not os.path.exists(path):
        return {}
    doc = _read(path)
    if schema_of(doc) > SCHEMA:
        return {k: doc[k] for k in TACTILE_ZERO_KEYS if k in doc}
    return _not_calibration(doc)


def previous_path(path: str) -> str:
    """Where the calibration before the one at ``path`` is kept."""
    folder, name = os.path.split(path)
    if name == CALIBRATION_FILE:
        return os.path.join(folder, PREVIOUS_CALIBRATION_FILE)
    stem, ext = os.path.splitext(name)
    return os.path.join(folder, f"{stem}.previous{ext or '.yaml'}")


def swap_path(path: str) -> str:
    """Where the current calibration waits while :func:`restore_previous` swaps."""
    stem, ext = os.path.splitext(path)
    return f"{stem}.swap{ext or '.yaml'}"


def recover_swap(path: str) -> None:
    """Finish or undo a :func:`restore_previous` that was interrupted, so the
    pair of files is always one of its two whole states."""
    swap = swap_path(path)
    if not os.path.exists(swap):
        return
    if not os.path.exists(path):
        os.replace(swap, path)                      # stopped after step 1: undo it
    elif not os.path.exists(previous_path(path)):
        os.replace(swap, previous_path(path))       # stopped after step 2: finish it


class NewerCalibrationError(RuntimeError):
    """The calibration file was written by a newer orca_core; only a new
    calibration replaces it."""


def _read(path: str) -> Dict[str, Any]:
    try:
        recover_swap(path)
    except OSError:
        pass
    return read_doc(path, KIND)


def read_document(path: Optional[str]) -> Dict[str, Any]:
    """The calibration document at ``path``; ``{}`` when there is none, or when
    a newer orca_core wrote it: what it holds is never used to drive motors."""
    if not path:
        return {}
    doc = _read(path)
    if schema_of(doc) > SCHEMA:
        logger.warning(
            "%s was written by a newer orca_core, so this one does not use it: the hand "
            "reads as uncalibrated until it is calibrated again, which keeps that file as "
            "%s", path, os.path.basename(previous_path(path)))
        return {}
    return doc


def check_writable(path: str) -> None:
    """Refuse to change a calibration file a newer orca_core wrote, other than
    by replacing it with a new calibration.

    Raises:
        NewerCalibrationError: it was written by a newer orca_core.
    """
    if os.path.exists(path) and schema_of(_read(path)) > SCHEMA:
        raise NewerCalibrationError(
            f"{path} was written by a newer orca_core and is left as it is; calibrate "
            "the hand to replace it (it is then kept as the previous calibration)")


def previous_stamp(path: str) -> Optional[Dict[str, Any]]:
    """:func:`stamp_of` the calibration kept before the one at ``path``, ``None``
    when none is kept."""
    previous = previous_path(path)
    return stamp_of(_read(previous)) if os.path.exists(previous) else None


def stamp_of(doc: Mapping[str, Any]) -> Dict[str, Any]:
    """``{"sequence", "calibrated_at"}`` of a calibration document."""
    try:
        sequence = int(doc.get(SEQUENCE) or 0)
    except (TypeError, ValueError, OverflowError):
        sequence = 0
    calibrated_at = doc.get(CALIBRATED_AT)
    return {"sequence": sequence, "calibrated_at": str(calibrated_at) if calibrated_at else None}


def measurements(result: CalibrationResult, *, include_encoder: bool) -> Dict[str, Any]:
    """The parts of ``result`` a calibration document stores."""
    doc: Dict[str, Any] = {
        MOTOR_LIMITS_DICT: {
            motor_id: list(limits) for motor_id, limits in result.motor_limits_dict.items()
        },
        WRIST_CALIBRATED: result.wrist_calibrated,
        CALIBRATED: result.calibrated,
        MOTOR_TRAVEL_MEASURED: {
            joint: round(float(travel), 2)
            for joint, travel in result.motor_travel_measured_dict.items()
            if travel is not None
        },
        DRIVE_CURRENT_MEASURED: {
            joint: {direction: round(float(current), 1)
                    for direction, current in sorted(per_direction.items())}
            for joint, per_direction in sorted(result.drive_current_measured_dict.items())
        },
    }
    if include_encoder:
        doc[JOINT_ENCODER_CALIBRATION] = joint_encoder_calibration_to_yaml(
            result.joint_encoder_calibration_dict
        )
        doc[JOINT_ROMS_MEASURED] = {
            joint: list(rom) for joint, rom in result.joint_roms_measured_dict.items()
        }
    return doc


def save(
    path: str,
    result: CalibrationResult,
    *,
    include_encoder: bool,
    fingerprint: Optional[Mapping[str, str]] = None,
    start_run: bool = False,
    stamp: bool = False,
    in_progress: Optional[bool] = None,
) -> Dict[str, Any]:
    """Write ``result``'s measurements to ``path`` in one atomic replace.

    Keys this module does not own are kept. ``start_run`` marks the first
    write of a new calibration: the file it replaces becomes
    ``calibration.previous.yaml`` and ``sequence`` goes up by one, except a
    run that stopped before it finished, which is replaced and not kept.
    ``stamp`` (implied by ``start_run``) sets ``calibrated_at`` to now.
    ``in_progress`` sets or clears the mark of a run still writing (``None``
    leaves it). Returns the document written.

    Raises:
        NewerCalibrationError: a newer orca_core wrote ``path`` and this is
            not the start of a new calibration.
    """
    _refuse_packaged(path)
    if not start_run:
        check_writable(path)
    doc = read_document(path)
    if start_run:
        _keep_as_previous(path, doc)
        doc[SEQUENCE] = stamp_of(doc)["sequence"] + 1
        doc.pop(SOURCE, None)
    if start_run or stamp:
        doc[CALIBRATED_AT] = now_iso()
    doc.pop(JOINT_TO_MOTOR_RATIOS, None)
    doc.update(measurements(result, include_encoder=include_encoder))
    if fingerprint is not None:
        doc[FINGERPRINT] = dict(sorted(fingerprint.items()))
    if in_progress:
        doc[IN_PROGRESS] = True
    elif in_progress is not None:
        doc.pop(IN_PROGRESS, None)
    write_doc(path, doc)
    return doc


def _keep_as_previous(path: str, current: Mapping[str, Any]) -> None:
    """Copy the calibration at ``path`` (read as ``current``) to the previous
    one, unless it is a run that stopped before it finished: the last
    complete calibration stays the previous one then."""
    if os.path.exists(path) and not current.get(IN_PROGRESS):
        with open(path, encoding="utf-8") as file:
            write_text_atomic(previous_path(path), file.read())


def adopt(
    path: str, document: Mapping[str, Any], *, source: str, keep_sequence: bool = False
) -> Dict[str, Any]:
    """Make ``document`` (a whole calibration from elsewhere, e.g. a hand's board)
    the calibration at ``path``, keeping the one it replaces as previous.

    Its ``calibrated_at`` is kept, and its ratios dropped. Keys of the file
    here that a calibration does not own (a tactile zero) are kept. The
    document's tactile zero is taken, whole, only when there is no file here
    yet (first contact): never after a Reset Zero, never mixed with one here.
    ``sequence`` counts on from the higher of the two, or is kept as it is
    with ``keep_sequence`` (a first copy, where there is nothing here to count
    on from).
    """
    _refuse_packaged(path)
    current = read_document(path)
    # Checked after the read, which finishes a restore that was cut short.
    first_contact = not os.path.exists(path)
    doc = {k: v for k, v in document.items()
           if k not in (JOINT_TO_MOTOR_RATIOS, SCHEMA_KEY, *TACTILE_ZERO_KEYS)}
    if first_contact:
        doc.update({k: document[k] for k in TACTILE_ZERO_KEYS if k in document})
    doc.update(_kept_here(path))
    _keep_as_previous(path, current)
    theirs = stamp_of(document)["sequence"]
    doc[SEQUENCE] = theirs if keep_sequence else max(stamp_of(current)["sequence"], theirs) + 1
    doc[SOURCE] = source
    write_doc(path, doc)
    return doc


def restore_previous(path: str) -> Dict[str, Any]:
    """Swap the calibration at ``path`` with its previous one, so restoring
    twice gets back where it started. Keys a calibration does not own (a
    tactile zero) stay as they were in force. Returns the stamp now in force.

    Raises:
        FileNotFoundError: there is no previous calibration to restore.
    """
    recover_swap(path)
    previous = previous_path(path)
    if not os.path.exists(previous):
        raise FileNotFoundError(f"no previous calibration kept beside {path}")
    kept = _kept_here(path)
    # Three renames, each atomic: every state in between is whole, and
    # recover_swap puts an interrupted one right on the next read.
    if os.path.exists(path):
        os.replace(path, swap_path(path))
    os.replace(previous, path)
    if os.path.exists(swap_path(path)):
        os.replace(swap_path(path), previous)
    restored = read_document(path)
    if restored and _not_calibration(restored) != kept:
        # The zero in force before the swap stays in force, in one atomic replace.
        write_doc(path, {**{k: v for k, v in restored.items() if k in CALIBRATION_KEYS}, **kept})
    return stamp_of(read_document(path))
