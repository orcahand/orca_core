"""Column layouts for the recorded tables.

Every table is a flat float64 matrix whose columns are named once, here, and
carried in the dataset manifest. Joint order is part of that manifest and is
never inferred from a config at read time: a dataset stays readable after the
hand it came from has been reconfigured.

One rule governs the layout: a column lives in exactly one table, at that
table's own cadence. Nothing copies a slow value onto a fast row. Combining
tables is the reader's job, and it stamps the age of what it joined.
"""

from __future__ import annotations

from typing import Sequence

from orca_core.control.joint_loop import LOOP_PHASES
from orca_core.hardware.sensing.constants import AUTO_ENC_NUM_JOINTS


PHASE_TO_CODE = {name: index for index, name in enumerate(LOOP_PHASES)}
CODE_TO_PHASE = {index: name for name, index in PHASE_TO_CODE.items()}

# Stored as a code so the row stays a homogeneous float array; the sink writes
# the name so the file reads without a lookup table.
PHASE_COLUMN = "phase"

SAMPLE_SCALARS = (
    "cycle",
    "t",
    "dt_s",
    PHASE_COLUMN,
    "freshness_ms",
    "same_frame",
    "integral_frozen",
    "clamped",
    "error_byte",
    "frame_t",
)

SAMPLE_JOINT_PREFIXES = ("raw", "meas", "tgt", "corr", "ierr", "mcmd")

FRAME_SCALARS = ("seq", "t", "frame_t", "error_byte")

LINK_STAT_COLUMNS = (
    "t",
    "frames_ok",
    "frames_bad_effective_length",
    "frames_routed_enc",
    "frames_bad_lrc_enc",
    "frames_dropped_no_handler_enc",
    "handler_errors_enc",
    "bad_header_resyncs",
    "response_queue_dropped",
    "responses_received",
    "responses_implausible_length",
)


def sample_columns(joint_names: Sequence[str]) -> list[str]:
    """Columns of the loop table for a given joint order."""
    columns = list(SAMPLE_SCALARS)
    for prefix in SAMPLE_JOINT_PREFIXES:
        columns.extend(f"{prefix}_{joint}" for joint in joint_names)
    return columns


def frame_columns(num_slots: int = AUTO_ENC_NUM_JOINTS) -> list[str]:
    """Columns of the encoder-frame table.

    Counts are named by encoder slot rather than by joint: a frame carries every
    slot on the board, including ones no joint is mapped to, and reducing it to
    the loop's joint set at capture time would discard evidence about the slots
    that are misbehaving.
    """
    return list(FRAME_SCALARS) + [f"raw_s{slot:02d}" for slot in range(num_slots)]
