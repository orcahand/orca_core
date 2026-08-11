"""The live window: what a running experiment is allowed to believe from it."""

from __future__ import annotations

import os
import shutil

import numpy as np
import pytest

from orca_core.hardware.sensing.constants import ENCODER_LSB_DEG, JOINT_TO_ENCODER_SLOT

from studies.record import JointAngleWindow

from tests._loop_helpers import encoder_reading_from_joint_angles, make_calibrated_hand


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


@pytest.fixture
def hand(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    hand = make_calibrated_hand(str(config_path))
    yield hand
    hand.disconnect()


def feed(window, hand, angles_by_joint, t_start=0.0, dt=0.002):
    """Push one frame per angle, as the delivery thread would."""
    for i, angles in enumerate(angles_by_joint):
        reading = encoder_reading_from_joint_angles(hand, angles)
        # Distinct frame timestamps: a repeat is a re-read, not a new frame.
        reading = reading.__class__(**{**vars(reading), "timestamp": t_start + i * dt})
        window.record(t_start + i * dt, reading)


def test_decoded_angles_match_what_was_encoded(hand):
    joint = "index_mcp"
    window = JointAngleWindow.for_hand(hand, [joint])

    feed(window, hand, [{joint: angle} for angle in (0.0, 5.0, 12.5)])

    _, angles = window.column(joint, 10.0)
    assert angles == pytest.approx([0.0, 5.0, 12.5], abs=ENCODER_LSB_DEG / 2)


def test_the_window_keeps_only_what_it_is_asked_for(hand):
    joint = "index_mcp"
    window = JointAngleWindow.for_hand(hand, [joint])

    feed(window, hand, [{joint: float(i)} for i in range(100)], dt=0.01)

    times, angles = window.column(joint, 0.2)
    assert times[-1] - times[0] == pytest.approx(0.2)
    assert angles[-1] == pytest.approx(99.0, abs=ENCODER_LSB_DEG)


def test_a_full_window_keeps_the_newest_frames(hand):
    """The oldest rows are the ones an experiment has already read; a window
    that dropped the newest would hide the thing it is watching for."""
    joint = "index_mcp"
    window = JointAngleWindow.for_hand(hand, [joint], capacity=50)

    feed(window, hand, [{joint: float(i % 30)} for i in range(200)], dt=0.002)

    times, angles = window.column(joint, 10.0)
    assert times.size == 50
    assert angles[-1] == pytest.approx(199 % 30, abs=ENCODER_LSB_DEG)
    assert window.written == 200


def test_a_flagged_joint_is_recorded_as_nothing_rather_than_as_a_number(hand):
    """The control loop holds a flagged joint's previous angle, which keeps it
    controllable; a record that did the same would show a joint holding still."""
    joint = "index_mcp"
    window = JointAngleWindow.for_hand(hand, [joint])
    slot = JOINT_TO_ENCODER_SLOT[joint]

    good = encoder_reading_from_joint_angles(hand, {joint: 4.0})
    flagged = encoder_reading_from_joint_angles(hand, {joint: 4.0})
    flagged.angle_error[slot] = True
    flagged = flagged.__class__(**{**vars(flagged), "timestamp": 1.0})

    window.record(0.0, good)
    window.record(1.0, flagged)

    _, angles = window.column(joint, 10.0)
    assert angles[0] == pytest.approx(4.0, abs=ENCODER_LSB_DEG)
    assert np.isnan(angles[1])
    assert window.flagged == 1


def test_a_repeated_frame_is_not_recorded_twice(hand):
    """Reading the client again between frames offers the same reading back;
    counting it would report a stalled stream as a live one."""
    joint = "index_mcp"
    window = JointAngleWindow.for_hand(hand, [joint])
    reading = encoder_reading_from_joint_angles(hand, {joint: 1.0})

    window.record(0.0, reading)
    window.record(0.1, reading)

    assert window.written == 1


def test_an_empty_window_reports_nothing_rather_than_a_number(hand):
    window = JointAngleWindow.for_hand(hand, ["index_mcp"])

    times, angles = window.column("index_mcp", 1.0)
    assert times.size == 0
    assert angles.size == 0
    assert window.latest() is None


def test_it_covers_every_encoder_backed_joint_by_default(hand):
    window = JointAngleWindow.for_hand(hand)

    assert set(window.joint_names) == set(hand._encoder_backed_joints())
