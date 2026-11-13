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
    now_iso,
    read_doc,
    write_doc,
)
from .utils.utils import write_text_atomic

SEQUENCE = "sequence"
CALIBRATED_AT = "calibrated_at"
SOURCE = "source"
KIND = "calibration"


def previous_path(path: str) -> str:
    """Where the calibration before the one at ``path`` is kept."""
    folder, name = os.path.split(path)
    if name == CALIBRATION_FILE:
        return os.path.join(folder, PREVIOUS_CALIBRATION_FILE)
    stem, ext = os.path.splitext(name)
    return os.path.join(folder, f"{stem}.previous{ext or '.yaml'}")


def read_document(path: Optional[str]) -> Dict[str, Any]:
    """The calibration document at ``path``, migrated; ``{}`` when there is none."""
    if not path:
        return {}
    return read_doc(path, KIND)


def stamp_of(doc: Mapping[str, Any]) -> Dict[str, Any]:
    """``{"sequence", "calibrated_at"}`` of a calibration document."""
    try:
        sequence = int(doc.get(SEQUENCE) or 0)
    except (TypeError, ValueError):
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
) -> Dict[str, Any]:
    """Write ``result``'s measurements to ``path`` in one atomic replace.

    Keys this module does not own are kept. ``start_run`` marks the first
    write of a new calibration: the file it replaces becomes
    ``calibration.previous.yaml`` and ``sequence`` goes up by one. ``stamp``
    (implied by ``start_run``) sets ``calibrated_at`` to now. Returns the
    document written.
    """
    doc = read_document(path)
    if start_run:
        if os.path.exists(path):
            with open(path, encoding="utf-8") as file:
                write_text_atomic(previous_path(path), file.read())
        doc[SEQUENCE] = stamp_of(doc)["sequence"] + 1
        doc.pop(SOURCE, None)
    if start_run or stamp:
        doc[CALIBRATED_AT] = now_iso()
    doc.pop(JOINT_TO_MOTOR_RATIOS, None)
    doc.update(measurements(result, include_encoder=include_encoder))
    if fingerprint is not None:
        doc[FINGERPRINT] = dict(sorted(fingerprint.items()))
    write_doc(path, doc)
    return doc


def adopt(
    path: str, document: Mapping[str, Any], *, source: str, keep_sequence: bool = False
) -> Dict[str, Any]:
    """Make ``document`` (a whole calibration from elsewhere, e.g. a hand's board)
    the calibration at ``path``, keeping the one it replaces as previous.

    Its ``calibrated_at`` is kept, and its ratios dropped. ``sequence`` counts
    on from the higher of the two, or is kept as it is with ``keep_sequence``
    (a first copy, where there is nothing here to count on from).
    """
    current = read_document(path)
    doc = {k: v for k, v in document.items() if k != JOINT_TO_MOTOR_RATIOS}
    if os.path.exists(path):
        with open(path, encoding="utf-8") as file:
            write_text_atomic(previous_path(path), file.read())
    theirs = stamp_of(document)["sequence"]
    doc[SEQUENCE] = theirs if keep_sequence else max(stamp_of(current)["sequence"], theirs) + 1
    doc[SOURCE] = source
    write_doc(path, doc)
    return doc


def restore_previous(path: str) -> Dict[str, Any]:
    """Swap the calibration at ``path`` with its previous one, so restoring
    twice gets back where it started. Returns the stamp now in force.

    Raises:
        FileNotFoundError: there is no previous calibration to restore.
    """
    previous = previous_path(path)
    if not os.path.exists(previous):
        raise FileNotFoundError(f"no previous calibration kept beside {path}")
    with open(previous, encoding="utf-8") as file:
        restored = file.read()
    current = None
    if os.path.exists(path):
        with open(path, encoding="utf-8") as file:
            current = file.read()
    write_text_atomic(path, restored)
    if current is None:
        os.remove(previous)
    else:
        write_text_atomic(previous, current)
    return stamp_of(read_document(path))
