"""Tests for the synthetic joint.

The plant is the reference every later fit is checked against, so its own
behaviour is pinned here: it must overshoot when told to, ring at the frequency
it was given, hold still inside its slack, and start late when delayed.
"""

from __future__ import annotations

import numpy as np
import pytest

from studies.plant import PlantJoint, simulate_step


def peak_count(angles, threshold):
    """Local maxima above a threshold — one per cycle of a ringing response."""
    return int(
        np.sum(
            (angles[1:-1] > angles[:-2])
            & (angles[1:-1] > angles[2:])
            & (angles[1:-1] > threshold)
        )
    )


def test_a_damped_joint_settles_on_its_command():
    times, angles = simulate_step(PlantJoint(damping_ratio=1.0), 10.0, duration_s=2.0)

    assert angles[-1] == pytest.approx(10.0, abs=0.05)


def test_a_damped_joint_does_not_overshoot():
    _, angles = simulate_step(PlantJoint(damping_ratio=1.0), 10.0, duration_s=2.0)

    assert angles.max() <= 10.05


def test_an_underdamped_joint_overshoots():
    """The behaviour a first-order model cannot express."""
    _, angles = simulate_step(PlantJoint(damping_ratio=0.15), 10.0, duration_s=2.0)

    assert angles.max() > 11.0


def test_an_underdamped_joint_rings_at_its_natural_frequency():
    frequency = 12.0
    times, angles = simulate_step(
        PlantJoint(natural_frequency_hz=frequency, damping_ratio=0.05),
        10.0,
        duration_s=1.0,
        dt_s=0.001,
    )
    moving = times >= 0
    peaks = peak_count(angles[moving], threshold=10.0)

    # Roughly one peak per cycle over the recorded second.
    assert abs(peaks - frequency) <= 2


def test_gain_scales_what_is_measured_against_what_was_commanded():
    """A ratio the map got wrong shows up here, not as a modelling error."""
    _, angles = simulate_step(
        PlantJoint(damping_ratio=1.0, gain=0.8), 10.0, duration_s=2.0
    )

    assert angles[-1] == pytest.approx(8.0, abs=0.05)


def test_dead_time_delays_the_start_of_motion():
    dt = 0.002
    times, angles = simulate_step(
        PlantJoint(damping_ratio=1.0, dead_time_s=0.02),
        10.0,
        duration_s=0.5,
        dt_s=dt,
    )
    moved = times[np.argmax(np.abs(angles) > 0.01)]

    assert moved == pytest.approx(0.02, abs=3 * dt)


def test_backlash_leaves_a_reversal_unanswered_until_the_slack_is_taken_up():
    joint = PlantJoint(damping_ratio=1.0, backlash_deg=4.0)
    for _ in range(2000):
        joint.step(10.0, 0.002)
    settled = joint.angle_deg

    for _ in range(50):
        joint.step(9.0, 0.002)

    assert joint.angle_deg == pytest.approx(settled, abs=0.2)


def test_the_joint_rests_at_the_edge_of_its_slack():
    """The driver reaches the command; the joint trails it by half the slack."""
    joint = PlantJoint(damping_ratio=1.0, backlash_deg=4.0)
    for _ in range(2000):
        joint.step(10.0, 0.002)

    assert joint.driver_deg == pytest.approx(10.0, abs=0.05)
    assert joint.angle_deg == pytest.approx(8.0, abs=0.05)


def test_backlash_is_taken_up_once_the_command_moves_far_enough():
    joint = PlantJoint(damping_ratio=1.0, backlash_deg=4.0)
    for _ in range(2000):
        joint.step(10.0, 0.002)
    for _ in range(2000):
        joint.step(0.0, 0.002)

    assert joint.angle_deg == pytest.approx(2.0, abs=0.1)


def test_backlash_makes_a_reversal_look_like_extra_dead_time():
    """The reason a step measurement pairs a preload with its test step: measured
    one way only, this is indistinguishable from transport delay."""
    dt = 0.002
    joint = PlantJoint(damping_ratio=1.0, backlash_deg=3.0)
    for _ in range(2000):
        joint.step(10.0, dt)

    start = joint.angle_deg
    elapsed = 0.0
    while abs(joint.angle_deg - start) < 0.05 and elapsed < 0.5:
        joint.step(0.0, dt)
        elapsed += dt

    assert elapsed > 5 * dt


def test_reset_clears_the_delay_line_and_the_slack():
    joint = PlantJoint(damping_ratio=1.0, backlash_deg=2.0, dead_time_s=0.01)
    for _ in range(100):
        joint.step(10.0, 0.002)

    joint.reset()
    assert joint.angle_deg == 0.0
    assert joint.driver_deg == 0.0

    for _ in range(2000):
        joint.step(5.0, 0.002)
    assert joint.angle_deg == pytest.approx(4.0, abs=0.1)


def test_the_response_does_not_depend_on_the_sample_rate():
    """A fit run on 2 ms samples must describe the same joint as one run on 1 ms."""
    coarse = simulate_step(PlantJoint(damping_ratio=0.6), 10.0, duration_s=1.0, dt_s=0.002)
    fine = simulate_step(PlantJoint(damping_ratio=0.6), 10.0, duration_s=1.0, dt_s=0.0005)

    assert coarse[1][-1] == pytest.approx(fine[1][-1], abs=0.05)
    assert coarse[1].max() == pytest.approx(fine[1].max(), abs=0.2)


def test_a_settle_window_precedes_the_step():
    times, angles = simulate_step(
        PlantJoint(), 10.0, duration_s=0.1, dt_s=0.002, settle_s=0.05
    )

    assert times[0] == pytest.approx(-0.05)
    assert np.all(angles[times < 0] == 0.0)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"natural_frequency_hz": 0.0},
        {"damping_ratio": -0.1},
        {"backlash_deg": -1.0},
        {"dead_time_s": -0.01},
    ],
)
def test_impossible_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):
        PlantJoint(**kwargs)


def test_a_non_advancing_step_is_rejected():
    with pytest.raises(ValueError, match="dt_s"):
        PlantJoint().step(1.0, 0.0)
