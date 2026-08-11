"""Records one row per encoder frame, at the rate the board emits them.

The control loop consumes whatever frame is latest when its cycle comes round,
which is a fraction of what arrives. This tier keeps the rest. It is the only
record that resolves motion faster than the loop rate: a disturbance near half
the loop rate appears in the loop's own record as a steady offset rather than as
motion, so a measurement of anything oscillating has to read this table.

Attaching is by subclassing the encoder client, which is the seam
``OrcaHandJointFeedback._create_encoder_client`` exists for. Registering a
second handler for the same frame type would not work: the link keys handlers by
type byte and a second registration replaces the first, which would leave the
control loop with no encoder feed at all.
"""

from __future__ import annotations

import time

import numpy as np

from orca_core.hardware.joint_encoder_client import JointEncoderClient
from orca_core.hardware.sensing.constants import AUTO_ENC_NUM_JOINTS

from .ring import SampleRing
from .schema import FRAME_SCALARS, frame_columns


# Two minutes at 500 Hz.
DEFAULT_FRAME_CAPACITY = 60_000


class FrameRecorder:
    """Packs encoder frames into rows. Fed by :class:`RecordingJointEncoderClient`."""

    def __init__(
        self,
        capacity: int = DEFAULT_FRAME_CAPACITY,
        num_slots: int = AUTO_ENC_NUM_JOINTS,
    ):
        self.columns = frame_columns(num_slots)
        self._num_slots = int(num_slots)
        self._num_scalars = len(FRAME_SCALARS)
        self.ring = SampleRing(capacity, len(self.columns))
        self._row = np.empty(len(self.columns), dtype=np.float64)
        self._seq = 0
        self._unpublished = 0
        self._stale = 0
        self._last_frame_t = float("nan")

    @property
    def dropped(self) -> int:
        return self.ring.dropped

    @property
    def unpublished(self) -> int:
        """Frames that arrived while the client was not publishing readings.

        Nonzero means ``start_stream()`` was not called for part of the run and
        that stretch of the table is missing.
        """
        return self._unpublished

    @property
    def stale(self) -> int:
        """Arrivals that left the client's reading unchanged.

        A frame the decoder rejects never becomes a reading, so what is on offer
        afterwards is still the previous one. Counting those keeps a rejected
        frame from entering the table a second time as if it were new, and the
        count is itself the rejection rate on the delivery thread.
        """
        return self._stale

    def record(self, received_at: float, reading) -> None:
        """Append one frame.

        ``received_at`` is taken before the frame is decoded and ``frame_t``
        after, so the gap between them is the decode cost on the delivery
        thread, measured rather than assumed.
        """
        if reading is None:
            self._unpublished += 1
            return
        if reading.timestamp == self._last_frame_t:
            self._stale += 1
            return
        self._last_frame_t = reading.timestamp
        row = self._row
        self._seq += 1
        row[: self._num_scalars] = (
            self._seq,
            received_at,
            reading.timestamp,
            reading.error_byte,
        )
        row[self._num_scalars :] = reading.raw_counts
        self.ring.push(row)


class RecordingJointEncoderClient(JointEncoderClient):
    """Encoder client that hands every frame to a :class:`FrameRecorder`.

    Behaves exactly like its base class for every other consumer: the frame is
    decoded once, by the base handler, and the recorder reads the result.
    """

    def __init__(self, link, recorder: FrameRecorder):
        super().__init__(link)
        self._recorder = recorder

    def _on_encoder_frame(self, frame_bytes: bytes) -> None:
        received_at = time.monotonic()
        super()._on_encoder_frame(frame_bytes)
        self._recorder.record(received_at, self.get_latest())
