"""Records one row per control cycle from a ``JointLoopThread`` observer.

Runs on the loop thread inside the cycle budget, so it does the least it can:
pack a float row and hand it to a ring. Nothing here allocates per cycle,
formats a string, or touches a file.
"""

from __future__ import annotations

import numpy as np

from orca_core.control.joint_loop import LoopSample
from orca_core.hardware.sensing.constants import JOINT_TO_ENCODER_SLOT

from .ring import SampleRing
from .schema import PHASE_TO_CODE, SAMPLE_SCALARS, sample_columns


# Five minutes at 100 Hz. Sized per run where a run is longer; the ring reports
# what it dropped either way.
DEFAULT_LOOP_CAPACITY = 32_000

_UNKNOWN_PHASE_CODE = -1


class LoopRecorder:
    """Observer that packs :class:`LoopSample` into rows.

    Install with ``loop.attach_observer(recorder)`` and drain the ring
    elsewhere. Samples offered after the ring fills are counted, not blocked
    on, so recording can never stall the loop.
    """

    def __init__(self, joint_names, capacity: int = DEFAULT_LOOP_CAPACITY):
        self.joint_names = list(joint_names)
        self.columns = sample_columns(self.joint_names)
        self._num_joints = len(self.joint_names)
        # The loop hands over every slot on the board; the table holds one raw
        # column per controlled joint, in this recorder's joint order.
        self._slots = np.array(
            [JOINT_TO_ENCODER_SLOT[joint] for joint in self.joint_names],
            dtype=np.int64,
        )
        self._num_scalars = len(SAMPLE_SCALARS)
        self.ring = SampleRing(capacity, len(self.columns))
        # Reused: push() copies into the ring, so the scratch row never escapes.
        self._row = np.empty(len(self.columns), dtype=np.float64)
        self._offsets = {
            name: self._num_scalars + index * self._num_joints
            for index, name in enumerate(
                ("raw", "meas", "tgt", "corr", "ierr", "mcmd")
            )
        }
        self._unknown_phases = 0

    @property
    def dropped(self) -> int:
        return self.ring.dropped

    @property
    def unknown_phases(self) -> int:
        """Samples whose phase the schema did not know. Nonzero means the loop
        grew an outcome this table cannot name."""
        return self._unknown_phases

    def __call__(self, sample: LoopSample) -> None:
        row = self._row
        row[: self._num_scalars] = (
            sample.cycle,
            sample.t,
            sample.dt_s,
            self._phase_code(sample.phase),
            sample.freshness_ms,
            sample.same_frame,
            sample.integral_frozen,
            sample.clamped,
            sample.error_byte,
            sample.frame_t,
        )
        self._pack("raw", sample.raw_counts, row, slots=True)
        self._pack("meas", sample.measured_deg, row)
        self._pack("tgt", sample.target_deg, row)
        self._pack("corr", sample.correction_deg, row)
        self._pack("ierr", sample.ierr_deg_s, row)
        self._pack("mcmd", sample.motor_cmd_rad, row)
        self.ring.push(row)

    def _phase_code(self, phase: str) -> int:
        code = PHASE_TO_CODE.get(phase)
        if code is None:
            self._unknown_phases += 1
            return _UNKNOWN_PHASE_CODE
        return code

    def _pack(self, name, values, row, *, slots: bool = False) -> None:
        """Write one per-joint block, or NaN where the cycle produced nothing.

        NaN rather than zero: a path that never computed a value must not be
        readable as having computed zero.
        """
        start = self._offsets[name]
        end = start + self._num_joints
        if values is None:
            row[start:end] = np.nan
            return
        row[start:end] = values[self._slots] if slots else values
