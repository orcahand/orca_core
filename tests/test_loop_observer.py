"""Tests for the ``JointLoopThread`` observer hook: coverage of every cycle
outcome, the same-frame flag, non-interference with the loop's own counters,
and containment of a misbehaving observer.
"""

from __future__ import annotations

import ast
import dataclasses as dc
import inspect
import os
import shutil
import textwrap
import threading
import time

import numpy as np
import pytest

from orca_core.control import JointController, JointLoopThread
from orca_core.control.constants import (
    WATCHDOG_HOLD_BASE_MS,
    WATCHDOG_HOLD_MS,
    WATCHDOG_STOP_LOOP_MS,
)
from orca_core.control.joint_loop import (
    LOOP_PHASES,
    OBSERVER_MAX_CONSECUTIVE_FAILURES,
    LoopSample,
)

from tests._loop_helpers import (
    StaticEncoderSource,
    encoder_reading_from_joint_angles,
    make_calibrated_hand,
)


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


@pytest.fixture
def calibrated_hand(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    hand = make_calibrated_hand(str(config_path))
    yield hand
    hand.disconnect()


class FrozenEncoderSource:
    """Encoder client that hands back the identical reading object every call,
    timestamp included, so consecutive cycles see one unchanged frame."""

    def __init__(self, reading):
        self._reading = reading

    def get_latest(self):
        return self._reading


def _make_loop(hand, encoder_source, *, Kp=0.1, Ki=1.0):
    controller = JointController(num_joints=len(hand._encoder_backed_joints()))
    controller.set_gains(Kp=Kp, Ki=Ki, correction_max_deg=15.0)
    return JointLoopThread(hand, encoder_source, controller, target_hz=100)


def _reading_at_zero(hand):
    return encoder_reading_from_joint_angles(
        hand, {j: 0.0 for j in hand._encoder_backed_joints()}
    )


def _source_at_zero(hand, freshness_ms=0.0):
    return StaticEncoderSource(_reading_at_zero(hand), freshness_ms=freshness_ms)


class Collector:
    """Observer that copies every sample's arrays so they stay readable after
    the cycle that produced them has returned."""

    def __init__(self):
        self.samples = []

    def __call__(self, sample: LoopSample) -> None:
        self.samples.append(
            sample._replace(
                **{
                    field: (None if value is None else np.array(value, copy=True))
                    for field, value in zip(sample._fields, sample)
                    if isinstance(value, np.ndarray)
                }
            )
        )

    def phases(self):
        return [s.phase for s in self.samples]


# ----- Emission-site coverage ---------------------------------------------


def test_emit_sites_match_step_once_exit_paths():
    """Every way out of ``step_once`` emits, and nothing emits twice.

    A tail-only hook would record the healthy cycles and silently drop the
    degraded ones, so the count is pinned rather than left to review. Counting
    ``return`` nodes alone would miss the fall-through, hence the emit-call
    count is the assertion.
    """
    source = textwrap.dedent(inspect.getsource(JointLoopThread.step_once))
    tree = ast.parse(source)

    emits = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "_emit"
    ]
    returns = [node for node in ast.walk(tree) if isinstance(node, ast.Return)]

    assert len(returns) == 5, "exit paths changed; revisit the emit sites"
    assert len(emits) == len(returns) + 1, (
        "step_once must emit once on each return plus once on the fall-through"
    )
    phases = {
        node.args[0].value for node in emits if isinstance(node.args[0], ast.Constant)
    }
    assert phases <= set(LOOP_PHASES)


def test_run_loop_emits_on_a_caught_exception():
    source = textwrap.dedent(inspect.getsource(JointLoopThread._run_loop))
    assert '_emit("exception"' in source


def test_phase_ok_carries_every_field(calibrated_hand):
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()
    collector = Collector()
    loop.attach_observer(collector)

    loop.step_once(dt=0.01)

    (sample,) = collector.samples
    assert sample.phase == "ok"
    assert sample.cycle == 1
    assert sample.dt_s == 0.01
    for field in (
        "raw_counts",
        "measured_deg",
        "target_deg",
        "correction_deg",
        "ierr_deg_s",
        "motor_cmd_rad",
    ):
        assert getattr(sample, field) is not None, field
    assert len(sample.measured_deg) == len(loop.joint_names)
    assert not sample.clamped


@pytest.mark.parametrize(
    "freshness_ms, phase",
    [
        (WATCHDOG_HOLD_BASE_MS + 10, "hold_base"),
        (WATCHDOG_STOP_LOOP_MS + 10, "estop"),
    ],
)
def test_watchdog_tiers_emit_their_phase(calibrated_hand, freshness_ms, phase):
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()
    loop._encoder_client.freshness_ms = freshness_ms
    collector = Collector()
    loop.attach_observer(collector)

    loop.step_once(dt=0.01)

    assert collector.phases() == [phase]


def test_hold_base_emits_raw_counts_but_not_decoded_angles(calibrated_hand):
    """The hold-base path never decodes, so the sample must not imply it did."""
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()
    loop._encoder_client.freshness_ms = WATCHDOG_HOLD_BASE_MS + 10
    collector = Collector()
    loop.attach_observer(collector)

    loop.step_once(dt=0.01)

    (sample,) = collector.samples
    assert sample.raw_counts is not None
    assert sample.measured_deg is None
    assert sample.motor_cmd_rad is not None


def test_hold_tier_reports_a_frozen_integrator(calibrated_hand):
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()
    loop._encoder_client.freshness_ms = WATCHDOG_HOLD_MS + 10
    collector = Collector()
    loop.attach_observer(collector)

    loop.step_once(dt=0.01)

    (sample,) = collector.samples
    assert sample.phase == "ok"
    assert sample.integral_frozen


def test_no_reading_and_fallback_phases(calibrated_hand):
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()
    collector = Collector()
    loop.attach_observer(collector)

    loop._encoder_client.set_reading(None)
    loop.step_once(dt=0.01)

    loop._stats["fallback_active"] = True
    loop.step_once(dt=0.01)

    assert collector.phases() == ["no_reading", "fallback"]
    assert [s.cycle for s in collector.samples] == [1, 2]


def test_paused_phase_still_reports_measurements(calibrated_hand):
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()
    collector = Collector()
    loop.attach_observer(collector)

    loop.pause_writes()
    loop.step_once(dt=0.01)

    (sample,) = collector.samples
    assert sample.phase == "paused"
    assert sample.measured_deg is not None
    assert sample.motor_cmd_rad is None


# ----- Sample content ------------------------------------------------------


def test_same_frame_marks_a_reused_measurement(calibrated_hand):
    """A cycle that re-reads the previous frame is flagged, so a consumer can
    tell a fresh measurement from a repeated one."""
    loop = _make_loop(calibrated_hand, FrozenEncoderSource(_reading_at_zero(calibrated_hand)))
    loop.prime_for_step()
    collector = Collector()
    loop.attach_observer(collector)

    loop.step_once(dt=0.01)
    loop.step_once(dt=0.01)

    assert [s.same_frame for s in collector.samples] == [False, True]


def test_distinct_frames_are_not_flagged_as_same(calibrated_hand):
    source = _source_at_zero(calibrated_hand)
    loop = _make_loop(calibrated_hand, source)
    loop.prime_for_step()
    collector = Collector()
    loop.attach_observer(collector)

    loop.step_once(dt=0.01)
    loop.step_once(dt=0.01)

    assert [s.same_frame for s in collector.samples] == [False, False]


def test_dt_is_reported_before_the_controller_clamp(calibrated_hand):
    """A cycle long enough to be clamped must be visible as such."""
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()
    collector = Collector()
    loop.attach_observer(collector)

    loop.step_once(dt=0.4)

    (sample,) = collector.samples
    assert sample.dt_s == 0.4


def test_cycle_counter_covers_dropped_samples(calibrated_hand):
    """The counter advances on cycles the observer never sees, so a gap in the
    recorded sequence is unambiguous loss."""
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()

    loop.step_once(dt=0.01)
    collector = Collector()
    loop.attach_observer(collector)
    loop.step_once(dt=0.01)

    assert [s.cycle for s in collector.samples] == [2]


def test_ierr_tracks_the_controller(calibrated_hand):
    hand = calibrated_hand
    joints = hand._encoder_backed_joints()
    source = _source_at_zero(hand)
    loop = _make_loop(hand, source, Ki=5.0)
    loop.prime_for_step()
    collector = Collector()
    loop.attach_observer(collector)

    loop.set_target({j: 5.0 for j in loop.joint_names})
    for _ in range(3):
        loop.step_once(dt=0.01)

    banked = [float(np.max(np.abs(s.ierr_deg_s))) for s in collector.samples]
    assert banked == sorted(banked)
    assert banked[-1] > 0.0
    assert len(joints) == len(collector.samples[0].ierr_deg_s)


# ----- Non-interference ----------------------------------------------------


def test_observer_does_not_change_loop_counters(calibrated_hand):
    """Attaching must not perturb the loop's own bookkeeping."""

    def run(observer):
        hand_loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
        hand_loop.prime_for_step()
        if observer is not None:
            hand_loop.attach_observer(observer)
        for _ in range(5):
            hand_loop.step_once(dt=0.01)
        stats = hand_loop.get_stats()
        return {k: v for k, v in stats.items() if k != "last_dt_s"}

    assert run(None) == run(Collector())


def test_detach_stops_delivery(calibrated_hand):
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()
    collector = Collector()
    loop.attach_observer(collector)

    loop.step_once(dt=0.01)
    loop.detach_observer()
    loop.step_once(dt=0.01)

    assert len(collector.samples) == 1


def test_detach_is_idempotent(calibrated_hand):
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()
    loop.detach_observer()
    loop.detach_observer()
    loop.step_once(dt=0.01)


def test_attach_replaces_the_previous_observer(calibrated_hand):
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()
    first, second = Collector(), Collector()

    loop.attach_observer(first)
    loop.step_once(dt=0.01)
    loop.attach_observer(second)
    loop.step_once(dt=0.01)

    assert len(first.samples) == 1
    assert len(second.samples) == 1


def test_attach_rejects_none(calibrated_hand):
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    with pytest.raises(ValueError, match="detach_observer"):
        loop.attach_observer(None)


# ----- Containment ---------------------------------------------------------


def test_a_raising_observer_is_detached_and_the_loop_survives(calibrated_hand, caplog):
    calls = []

    def broken(sample):
        calls.append(sample.cycle)
        raise RuntimeError("observer is broken")

    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()
    loop.attach_observer(broken)

    for _ in range(OBSERVER_MAX_CONSECUTIVE_FAILURES + 3):
        loop.step_once(dt=0.01)

    assert len(calls) == OBSERVER_MAX_CONSECUTIVE_FAILURES
    assert loop.get_stats()["cycles_exception"] == 0
    assert loop.get_stats()["cycles_ok"] == OBSERVER_MAX_CONSECUTIVE_FAILURES + 3


def test_intermittent_failures_do_not_detach(calibrated_hand):
    """The counter is consecutive, so an occasional failure is tolerated."""
    calls = []

    def flaky(sample):
        calls.append(sample.cycle)
        if sample.cycle % 2 == 0:
            raise RuntimeError("transient")

    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    loop.prime_for_step()
    loop.attach_observer(flaky)

    for _ in range(20):
        loop.step_once(dt=0.01)

    assert len(calls) == 20


def test_running_thread_delivers_samples(calibrated_hand):
    """End to end on the real thread, not by calling step_once directly."""
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand))
    collector = Collector()
    seen = threading.Event()

    def observe(sample):
        collector(sample)
        if len(collector.samples) >= 5:
            seen.set()

    loop.start()
    try:
        loop.attach_observer(observe)
        assert seen.wait(timeout=2.0), "no samples arrived from the loop thread"
    finally:
        loop.detach_observer()
        loop.stop()

    assert all(s.phase == "ok" for s in collector.samples)
    cycles = [s.cycle for s in collector.samples]
    assert cycles == sorted(cycles)


# ----- Integrator reset ----------------------------------------------------


def test_reset_integral_discharges_banked_demand(calibrated_hand):
    """Stored integral survives a gain change, so retuning mid-run inherits the
    previous tuning's demand unless it is cleared."""
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand), Ki=5.0)
    loop.prime_for_step()
    collector = Collector()
    loop.attach_observer(collector)

    loop.set_target({j: 5.0 for j in loop.joint_names})
    for _ in range(20):
        loop.step_once(dt=0.01)
    assert float(np.max(np.abs(collector.samples[-1].ierr_deg_s))) > 0.0

    loop.reset_integral()
    loop.step_once(dt=0.01)

    banked = float(np.max(np.abs(collector.samples[-1].ierr_deg_s)))
    assert banked == pytest.approx(0.0, abs=0.06)


def test_reset_integral_leaves_the_target_and_bias_alone(calibrated_hand):
    """Re-anchoring would re-read the motors and re-latch both; this must not."""
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand), Ki=5.0)
    loop.prime_for_step()
    loop.set_target({j: 4.0 for j in loop.joint_names})
    for _ in range(10):
        loop.step_once(dt=0.01)

    target_before = loop._target_deg.copy()
    bias_before = loop._motor_bias.copy()
    loop.reset_integral()

    assert np.array_equal(loop._target_deg, target_before)
    assert np.array_equal(loop._motor_bias, bias_before)


def test_reset_integral_unfreezes_a_frozen_integrator(calibrated_hand):
    loop = _make_loop(calibrated_hand, _source_at_zero(calibrated_hand), Ki=5.0)
    loop.prime_for_step()
    loop._controller.freeze_integral()

    loop.reset_integral()
    assert not loop._controller.integral_frozen
