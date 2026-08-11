"""Tests for the checks that refuse to start a measurement."""

from __future__ import annotations

import numpy as np
import pytest

from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware.sensing.types import EncoderReading

from studies.preconditions import (
    MAX_PLAUSIBLE_JUMP_COUNTS,
    PreconditionError,
    check_encoder_health,
    require_closed_loop,
    require_healthy_encoders,
    require_no_motion_profile,
    require_open_loop,
)


class FakeEncoderStream:
    """Encoder client that plays a scripted sequence of frames."""

    def __init__(self, readings):
        self._readings = list(readings)
        self._index = -1

    def get_latest(self):
        if self._index + 1 < len(self._readings):
            self._index += 1
        if self._index < 0:
            return None
        return self._readings[self._index]


def reading(counts, *, timestamp, parity_bad=(), angle_error=()):
    parity_ok = np.ones(AUTO_ENC_NUM_JOINTS, dtype=bool)
    parity_ok[list(parity_bad)] = False
    errors = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool)
    errors[list(angle_error)] = True
    return EncoderReading(
        raw_counts=np.asarray(counts, dtype=np.uint16),
        parity_ok=parity_ok,
        angle_error=errors,
        error_byte=0,
        timestamp=timestamp,
    )


def drifting_frames(n, *, parity_bad=(), angle_error=(), start=1000, step=3):
    return [
        reading(
            np.full(AUTO_ENC_NUM_JOINTS, start + i * step, dtype=np.uint16),
            timestamp=100.0 + i * 0.002,
            parity_bad=parity_bad,
            angle_error=angle_error,
        )
        for i in range(n)
    ]


def test_healthy_stream_passes():
    report = check_encoder_health(FakeEncoderStream(drifting_frames(20)), num_frames=20)

    assert report.frames_sampled == 20
    assert report.unhealthy == []


def test_a_slot_flagging_every_frame_is_reported():
    slot = JOINT_TO_ENCODER_SLOT["middle_pip"]
    report = check_encoder_health(
        FakeEncoderStream(drifting_frames(20, angle_error=[slot])), num_frames=20
    )

    faults = {s.joint for s in report.unhealthy}
    assert "middle_pip" in faults
    assert report.slots[slot].flagged_fraction == 1.0


def test_parity_failures_are_reported_separately_from_the_chip_flag():
    slot = JOINT_TO_ENCODER_SLOT["index_mcp"]
    report = check_encoder_health(
        FakeEncoderStream(drifting_frames(20, parity_bad=[slot])), num_frames=20
    )

    assert report.slots[slot].parity_bad == 20
    assert report.slots[slot].angle_error == 0
    assert "parity 20" in report.slots[slot].complaint


def test_an_occasional_flag_is_tolerated():
    """A glitch is not a condition; refusing on one frame would stop every run."""
    slot = JOINT_TO_ENCODER_SLOT["index_mcp"]
    frames = drifting_frames(200)
    frames[7] = reading(
        np.asarray(frames[7].raw_counts), timestamp=frames[7].timestamp, angle_error=[slot]
    )

    report = check_encoder_health(FakeEncoderStream(frames), num_frames=200)
    assert report.unhealthy == []


def test_a_jump_no_joint_could_make_is_caught():
    """Corruption can satisfy a single-bit parity check; the angle it decodes to
    cannot be reached in one frame interval."""
    slot = JOINT_TO_ENCODER_SLOT["ring_mcp"]
    frames = drifting_frames(20)
    counts = np.asarray(frames[10].raw_counts).copy()
    counts[slot] = counts[slot] + MAX_PLAUSIBLE_JUMP_COUNTS + 50
    frames[10] = reading(counts, timestamp=frames[10].timestamp)

    report = check_encoder_health(FakeEncoderStream(frames), num_frames=20)
    # A one-frame spike is two implausible transitions: into it and back out.
    assert report.slots[slot].impossible_jumps == 2
    assert "implausible" in report.slots[slot].complaint
    assert not report.slots[slot].healthy


def test_a_count_wrapping_past_zero_is_not_a_jump():
    frames = [
        reading(
            np.full(AUTO_ENC_NUM_JOINTS, (16380 + i * 3) % 16384, dtype=np.uint16),
            timestamp=100.0 + i * 0.002,
        )
        for i in range(20)
    ]

    report = check_encoder_health(FakeEncoderStream(frames), num_frames=20)
    assert report.unhealthy == []


def test_a_slot_that_never_moves_is_noticed():
    frames = [
        reading(np.full(AUTO_ENC_NUM_JOINTS, 900, dtype=np.uint16), timestamp=100.0 + i * 0.002)
        for i in range(20)
    ]

    report = check_encoder_health(FakeEncoderStream(frames), num_frames=20)
    assert {s.joint for s in report.constant_slots}


def test_repeated_reads_of_one_frame_are_not_counted_twice():
    single = drifting_frames(1)

    with pytest.raises(PreconditionError, match="1/5"):
        check_encoder_health(
            FakeEncoderStream(single), num_frames=5, timeout_s=0.05, poll_s=0.001
        )


def test_a_dead_stream_says_what_to_check():
    with pytest.raises(PreconditionError, match="link is down"):
        check_encoder_health(
            FakeEncoderStream([]), num_frames=5, timeout_s=0.05, poll_s=0.001
        )


def test_require_healthy_encoders_names_the_joint_and_the_remedy():
    slot = JOINT_TO_ENCODER_SLOT["wrist"]
    stream = FakeEncoderStream(drifting_frames(20, angle_error=[slot]))

    with pytest.raises(PreconditionError) as excinfo:
        require_healthy_encoders(stream, num_frames=20)

    message = str(excinfo.value)
    assert "wrist" in message
    assert "Power-cycle" in message


def test_require_healthy_encoders_ignores_joints_the_run_does_not_use():
    slot = JOINT_TO_ENCODER_SLOT["wrist"]
    stream = FakeEncoderStream(drifting_frames(20, angle_error=[slot]))

    report = require_healthy_encoders(stream, joints=["index_mcp"], num_frames=20)
    assert report.frames_sampled == 20


def test_health_report_serialises_for_the_manifest():
    report = check_encoder_health(FakeEncoderStream(drifting_frames(10)), num_frames=10)
    payload = report.as_dict()

    assert payload["frames_sampled"] == 10
    assert len(payload["slots"]) == AUTO_ENC_NUM_JOINTS


class FakeHand:
    def __init__(self, loop=None):
        self._loop = loop


class FakeLoop:
    def __init__(self, fallback_active=False):
        self._fallback = fallback_active

    def get_stats(self):
        return {"fallback_active": self._fallback}


def test_require_open_loop_rejects_a_running_loop():
    with pytest.raises(PreconditionError, match="same motors"):
        require_open_loop(FakeHand(loop=FakeLoop()))


def test_require_open_loop_accepts_a_hand_without_one():
    require_open_loop(FakeHand())


def test_require_closed_loop_rejects_an_estopped_loop():
    with pytest.raises(PreconditionError, match="e-stopped"):
        require_closed_loop(FakeHand(loop=FakeLoop(fallback_active=True)))


def test_require_closed_loop_rejects_a_hand_without_one():
    with pytest.raises(PreconditionError, match="no joint-feedback loop"):
        require_closed_loop(FakeHand())


class FakeMotorHand:
    def __init__(self, velocities):
        self._velocities = velocities
        self._motor_client = self
        self.config = self

    @property
    def motor_ids(self):
        return list(self._velocities)

    def read_profile_velocity(self, motor_ids):
        return [self._velocities[mid] for mid in motor_ids]


def test_a_shaped_motion_profile_blocks_a_timing_measurement():
    with pytest.raises(PreconditionError, match="profile velocity"):
        require_no_motion_profile(FakeMotorHand({1: 0, 2: 40}))


def test_cleared_profiles_are_returned_for_the_manifest():
    assert require_no_motion_profile(FakeMotorHand({1: 0, 2: 0})) == {1: 0, 2: 0}


def test_a_motor_client_without_the_register_is_not_an_error():
    """Not every backend exposes it; a missing reader is not a failed check."""

    class Bare:
        _motor_client = object()

    assert require_no_motion_profile(Bare()) == {}
