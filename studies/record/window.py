"""A short live view of the joints, at the rate frames arrive.

Everything else here writes to disk and is read afterwards. This is the one
structure a running experiment reads back from, because two of its decisions
cannot wait for the file: whether a joint has started oscillating, and whether
it is about to hurt itself.

Both have to be answered at frame rate rather than at loop rate. An oscillation
near half the loop rate shows up in the loop's own record as a steady offset
instead of as motion, so a monitor fed from there would report a joint that is
ringing hard as a joint sitting quietly off target.

Angles are decoded the way the control loop decodes them, from the same anchor
counts and the same polarity table, with one difference: a joint the encoder
chip flags is recorded as NaN. The loop holds the previous value there, which is
right for control and wrong for measurement — it would put a flat, plausible
stretch into a record of how the joint moved.
"""

from __future__ import annotations

import numpy as np

from orca_core.hardware.sensing.constants import (
    JOINT_TO_ENCODER_SLOT,
    joint_encoder_polarity_for_side,
)
from orca_core.hardware.sensing.encoder_protocol import encoder_to_joint_angle


# Ten seconds at the board's emit rate. The producer must not lap a reader
# mid-copy, so this is sized well above the longest window anything reads.
DEFAULT_WINDOW_CAPACITY = 5_000


class JointAngleWindow:
    """The last few seconds of joint angles, decoded as frames arrive.

    Written on the delivery thread, read on whichever thread is running the
    experiment, and neither takes a lock: the writer must not block behind a
    reader, and the reader only ever looks at rows the writer has finished with.
    That holds as long as the capacity is much larger than the window being
    read, which is what makes the sizing above part of the contract rather than
    a tuning choice.
    """

    def __init__(
        self,
        joint_names,
        anchor_counts,
        polarities,
        anchor_angles_deg,
        capacity: int = DEFAULT_WINDOW_CAPACITY,
    ):
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self.joint_names = list(joint_names)
        if not self.joint_names:
            raise ValueError("a window needs at least one joint")
        self._index = {name: i for i, name in enumerate(self.joint_names)}
        self._slots = np.array(
            [JOINT_TO_ENCODER_SLOT[name] for name in self.joint_names], dtype=np.int64
        )
        self._anchors = np.asarray(anchor_counts, dtype=np.int64)
        self._polarities = np.asarray(polarities, dtype=np.int64)
        self._anchor_angles = np.asarray(anchor_angles_deg, dtype=np.float64)

        self._capacity = int(capacity)
        self._t = np.zeros(self._capacity, dtype=np.float64)
        self._angles = np.zeros((self._capacity, len(self.joint_names)), dtype=np.float64)
        self._written = 0
        self._flagged = 0
        self._last_frame_t = float("nan")

    @classmethod
    def for_hand(cls, hand, joints=None, capacity: int = DEFAULT_WINDOW_CAPACITY):
        """Build a window over ``joints`` using the hand's own calibration.

        The anchor angle is the config ROM upper, which is where the encoder
        frame is pinned; taking it from anywhere else would put this window in a
        different frame from the loop it is watching.
        """
        encoder_cal = hand.calibration.joint_encoder_calibration_dict
        names = [
            joint
            for joint in (joints if joints is not None else hand._encoder_backed_joints())
            if joint in encoder_cal
        ]
        if not names:
            raise ValueError("none of the requested joints have an encoder anchor")
        polarity = joint_encoder_polarity_for_side(hand.config.type)
        config_roms = hand.config.joint_roms_dict
        return cls(
            names,
            [encoder_cal[joint].enc_at_anchor_count for joint in names],
            [polarity[joint] for joint in names],
            [config_roms[joint][1] for joint in names],
            capacity=capacity,
        )

    @property
    def written(self) -> int:
        """Frames appended since construction."""
        return self._written

    @property
    def flagged(self) -> int:
        """Joint-samples the encoder chip flagged, and this window stored as NaN."""
        return self._flagged

    def record(self, received_at: float, reading) -> None:
        """Append one frame. Same signature as the recorders, so one encoder
        client can feed both."""
        if reading is None or reading.timestamp == self._last_frame_t:
            return
        self._last_frame_t = reading.timestamp

        raw = np.asarray(reading.raw_counts)[self._slots]
        angles = encoder_to_joint_angle(
            raw, self._anchors, self._polarities, self._anchor_angles
        )
        invalid = (
            ~np.asarray(reading.parity_ok)[self._slots]
            | np.asarray(reading.angle_error)[self._slots]
        )
        if np.any(invalid):
            self._flagged += int(np.count_nonzero(invalid))
            angles = np.where(invalid, np.nan, angles)

        slot = self._written % self._capacity
        self._t[slot] = received_at
        self._angles[slot] = angles
        self._written += 1

    def recent(self, seconds: float, now: float | None = None):
        """The last ``seconds`` of the record: arrival times and angles.

        Angles come back as ``(rows, joints)`` in this window's joint order.
        Returns whatever is there, which after a short dwell is less than asked
        for and after a stalled stream is nothing at all — a caller deciding
        anything from it has to look at how much it got.
        """
        times, angles = self._tail()
        if times.size == 0:
            return times, angles
        end = times[-1] if now is None else float(now)
        keep = times >= end - float(seconds)
        return times[keep], angles[keep]

    def latest(self):
        """The most recent decoded angles, or ``None`` if nothing has arrived."""
        if self._written == 0:
            return None
        row = self._angles[(self._written - 1) % self._capacity].copy()
        return {name: float(row[i]) for i, name in enumerate(self.joint_names)}

    def column(self, joint: str, seconds: float, now: float | None = None):
        """One joint's recent angles, with the times they arrived at."""
        times, angles = self.recent(seconds, now=now)
        return times, angles[:, self._index[joint]]

    def _tail(self):
        written = self._written
        if written == 0:
            return (
                np.empty(0, dtype=np.float64),
                np.empty((0, len(self.joint_names)), dtype=np.float64),
            )
        if written <= self._capacity:
            return self._t[:written].copy(), self._angles[:written].copy()
        start = written % self._capacity
        times = np.concatenate((self._t[start:], self._t[:start]))
        angles = np.concatenate((self._angles[start:], self._angles[:start]))
        return times, angles
