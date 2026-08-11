"""Tests for the capture path: ring accounting, sinks, the recorders that feed
them, and the reader that takes a dataset back.
"""

from __future__ import annotations

import json
import threading
import time

import numpy as np
import pytest

from orca_core.control.joint_loop import LoopSample
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware.sensing.types import EncoderReading

from studies.record import (
    CsvRowSink,
    Drain,
    FrameRecorder,
    ListRowSink,
    LoopRecorder,
    SampleRing,
    Table,
    join_asof,
    read_table,
)
from studies.record.schema import PHASE_TO_CODE, frame_columns, sample_columns


JOINTS = ["index_mcp", "index_pip", "thumb_cmc"]


def make_sample(cycle=1, phase="ok", num_joints=len(JOINTS), **overrides):
    fields = dict(
        cycle=cycle,
        t=100.0 + cycle * 0.01,
        dt_s=0.01,
        phase=phase,
        freshness_ms=2.0,
        same_frame=False,
        integral_frozen=False,
        clamped=False,
        error_byte=0,
        frame_t=99.998,
        raw_counts=np.arange(AUTO_ENC_NUM_JOINTS, dtype=np.uint16),
        measured_deg=np.full(num_joints, 1.5),
        target_deg=np.full(num_joints, 2.0),
        correction_deg=np.full(num_joints, 0.25),
        ierr_deg_s=np.full(num_joints, 0.05),
        motor_cmd_rad=np.full(num_joints, 0.3),
    )
    fields.update(overrides)
    return LoopSample(**fields)


# ----- Ring ----------------------------------------------------------------


def test_ring_round_trips_rows_in_order():
    ring = SampleRing(capacity=8, num_columns=3)
    for i in range(5):
        assert ring.push(np.array([i, i * 2, i * 3], dtype=np.float64))

    rows = ring.drain()
    assert rows.shape == (5, 3)
    assert list(rows[:, 0]) == [0, 1, 2, 3, 4]
    assert ring.drain().shape == (0, 3)


def test_ring_wraps_across_the_capacity_boundary():
    ring = SampleRing(capacity=4, num_columns=1)
    for i in range(3):
        ring.push(np.array([i], dtype=np.float64))
    ring.drain()
    for i in range(3, 7):
        ring.push(np.array([i], dtype=np.float64))

    assert list(ring.drain()[:, 0]) == [3, 4, 5, 6]


def test_ring_overflow_drops_the_newest_and_counts_it():
    """What survives must be a contiguous run, so an overrun reads as a gap in
    the producer's sequence rather than as scrambled history."""
    ring = SampleRing(capacity=4, num_columns=1)
    accepted = [ring.push(np.array([i], dtype=np.float64)) for i in range(10)]

    assert accepted == [True] * 4 + [False] * 6
    assert ring.dropped == 6
    assert list(ring.drain()[:, 0]) == [0, 1, 2, 3]


def test_ring_recovers_capacity_after_a_drain():
    ring = SampleRing(capacity=2, num_columns=1)
    ring.push(np.zeros(1))
    ring.push(np.zeros(1))
    assert not ring.push(np.zeros(1))
    ring.drain()
    assert ring.push(np.zeros(1))


def test_ring_rejects_a_useless_shape():
    with pytest.raises(ValueError):
        SampleRing(capacity=0, num_columns=1)
    with pytest.raises(ValueError):
        SampleRing(capacity=1, num_columns=0)


# ----- Loop recorder -------------------------------------------------------


def test_loop_recorder_packs_every_field():
    recorder = LoopRecorder(JOINTS, capacity=16)
    recorder(make_sample())

    rows = recorder.ring.drain()
    table = Table(recorder.columns, rows)
    assert len(table) == 1
    assert table["cycle"][0] == 1
    assert table["phase"][0] == PHASE_TO_CODE["ok"]
    assert table["dt_s"][0] == 0.01
    assert list(table.block("meas")[0]) == [1.5, 1.5, 1.5]
    assert list(table.block("corr")[0]) == [0.25, 0.25, 0.25]


def test_loop_recorder_takes_raw_counts_for_its_own_joints():
    """The sample carries the whole board; the table holds the joints it names."""
    recorder = LoopRecorder(JOINTS, capacity=16)
    recorder(make_sample())

    table = Table(recorder.columns, recorder.ring.drain())
    expected = [JOINT_TO_ENCODER_SLOT[joint] for joint in JOINTS]
    assert list(table.block("raw")[0]) == expected


def test_loop_recorder_writes_nan_where_a_cycle_computed_nothing():
    """A path that never produced a value must not read as having produced zero."""
    recorder = LoopRecorder(JOINTS, capacity=16)
    recorder(
        make_sample(
            phase="hold_base",
            measured_deg=None,
            correction_deg=None,
        )
    )

    table = Table(recorder.columns, recorder.ring.drain())
    assert np.all(np.isnan(table.block("meas")[0]))
    assert np.all(np.isnan(table.block("corr")[0]))
    assert not np.any(np.isnan(table.block("tgt")[0]))


def test_loop_recorder_column_order_matches_joint_order():
    recorder = LoopRecorder(JOINTS, capacity=4)
    assert recorder.columns == sample_columns(JOINTS)
    assert Table(recorder.columns, recorder.ring.drain()).joints("meas") == JOINTS


def test_loop_recorder_counts_a_phase_the_schema_cannot_name():
    recorder = LoopRecorder(JOINTS, capacity=4)
    recorder(make_sample(phase="something_new"))

    assert recorder.unknown_phases == 1
    assert Table(recorder.columns, recorder.ring.drain())["phase"][0] == -1


def test_loop_recorder_never_blocks_when_full():
    recorder = LoopRecorder(JOINTS, capacity=2)
    for cycle in range(1, 8):
        recorder(make_sample(cycle=cycle))

    assert recorder.dropped == 5
    assert list(Table(recorder.columns, recorder.ring.drain())["cycle"]) == [1, 2]


def test_loop_recorder_does_not_retain_the_sample_arrays():
    """Arrays are the loop's working buffers; the row must be a copy."""
    recorder = LoopRecorder(JOINTS, capacity=4)
    measured = np.full(len(JOINTS), 1.5)
    recorder(make_sample(measured_deg=measured))
    measured[:] = 99.0

    table = Table(recorder.columns, recorder.ring.drain())
    assert list(table.block("meas")[0]) == [1.5, 1.5, 1.5]


# ----- Frame recorder ------------------------------------------------------


def make_reading(timestamp, counts=None, error_byte=0):
    raw = (
        np.arange(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
        if counts is None
        else np.asarray(counts, dtype=np.uint16)
    )
    return EncoderReading(
        raw_counts=raw,
        parity_ok=np.ones(AUTO_ENC_NUM_JOINTS, dtype=bool),
        angle_error=np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool),
        error_byte=error_byte,
        timestamp=timestamp,
    )


def test_frame_recorder_numbers_frames_and_keeps_every_slot():
    recorder = FrameRecorder(capacity=16)
    recorder.record(10.0, make_reading(9.999))
    recorder.record(10.002, make_reading(10.001))

    table = Table(recorder.columns, recorder.ring.drain())
    assert list(table["seq"]) == [1, 2]
    assert table.columns == frame_columns()
    assert table["raw_s16"][0] == AUTO_ENC_NUM_JOINTS - 1


def test_frame_recorder_keeps_both_timestamps():
    """The gap between arrival and the decoded frame's own stamp is the decode
    cost on the delivery thread."""
    recorder = FrameRecorder(capacity=4)
    recorder.record(10.0, make_reading(10.0004))

    table = Table(recorder.columns, recorder.ring.drain())
    assert table["t"][0] == 10.0
    assert table["frame_t"][0] == pytest.approx(10.0004)


def test_frame_recorder_ignores_an_unchanged_reading():
    """A frame the decoder rejects leaves the previous reading on offer; it must
    not enter the table a second time."""
    recorder = FrameRecorder(capacity=8)
    reading = make_reading(10.0)
    recorder.record(10.0, reading)
    recorder.record(10.002, reading)
    recorder.record(10.004, make_reading(10.004))

    assert recorder.stale == 1
    assert list(Table(recorder.columns, recorder.ring.drain())["seq"]) == [1, 2]


def test_frame_recorder_counts_frames_arriving_before_publishing():
    recorder = FrameRecorder(capacity=4)
    recorder.record(10.0, None)

    assert recorder.unpublished == 1
    assert recorder.ring.drain().shape[0] == 0


# ----- Sinks and drain -----------------------------------------------------


def test_csv_sink_writes_phase_names(tmp_path):
    recorder = LoopRecorder(JOINTS, capacity=8)
    recorder(make_sample(phase="hold_base"))
    sink = CsvRowSink(tmp_path / "loop.csv", recorder.columns)
    sink.write_rows(recorder.ring.drain())
    sink.close()

    header, first = (tmp_path / "loop.csv").read_text().splitlines()[:2]
    assert header.split(",")[3] == "phase"
    assert first.split(",")[3] == "hold_base"


def test_csv_sink_round_trips_through_the_reader(tmp_path):
    recorder = LoopRecorder(JOINTS, capacity=8)
    for cycle in range(1, 4):
        recorder(make_sample(cycle=cycle))
    sink = CsvRowSink(tmp_path / "loop.csv", recorder.columns)
    sink.write_rows(recorder.ring.drain())
    sink.close()

    table = read_table(tmp_path / "loop.csv")
    assert table.columns == recorder.columns
    assert list(table["cycle"]) == [1, 2, 3]
    assert list(table["phase"]) == [PHASE_TO_CODE["ok"]] * 3


def test_csv_sink_round_trips_nan(tmp_path):
    recorder = LoopRecorder(JOINTS, capacity=8)
    recorder(make_sample(measured_deg=None))
    sink = CsvRowSink(tmp_path / "loop.csv", recorder.columns)
    sink.write_rows(recorder.ring.drain())
    sink.close()

    assert np.all(np.isnan(read_table(tmp_path / "loop.csv").block("meas")[0]))


def test_csv_sink_rejects_a_row_of_the_wrong_width(tmp_path):
    sink = CsvRowSink(tmp_path / "t.csv", ["a", "b"])
    with pytest.raises(ValueError, match="columns"):
        sink.write_rows(np.zeros((1, 3)))
    sink.close()


def test_sink_refuses_writes_after_close(tmp_path):
    sink = CsvRowSink(tmp_path / "t.csv", ["a"])
    sink.close()
    with pytest.raises(RuntimeError, match="closed"):
        sink.write_rows(np.zeros((1, 1)))


def test_a_slow_sink_never_delays_the_producer():
    """File I/O belongs to the drain thread; a producer inside a control cycle
    must not wait on it."""

    class SlowSink(ListRowSink):
        def write_rows(self, rows):
            time.sleep(0.05)
            super().write_rows(rows)

    recorder = LoopRecorder(JOINTS, capacity=4096)
    sink = SlowSink(recorder.columns)
    drain = Drain(interval_s=0.005)
    drain.add(recorder.ring, sink)
    drain.start()

    worst = 0.0
    for cycle in range(1, 201):
        start = time.perf_counter()
        recorder(make_sample(cycle=cycle))
        worst = max(worst, time.perf_counter() - start)
    drain.stop()

    assert worst < 0.005, f"a cycle waited {worst * 1e3:.1f} ms on the recorder"
    assert sink.rows_written == 200


def test_drain_takes_what_is_left_when_it_stops():
    recorder = LoopRecorder(JOINTS, capacity=64)
    sink = ListRowSink(recorder.columns)
    drain = Drain(interval_s=10.0)
    drain.add(recorder.ring, sink)
    drain.start()
    for cycle in range(1, 6):
        recorder(make_sample(cycle=cycle))

    assert drain.stop()
    assert sink.rows_written == 5


def test_drain_survives_a_failing_sink():
    class BrokenSink(ListRowSink):
        def write_rows(self, rows):
            raise IOError("disk went away")

    recorder = LoopRecorder(JOINTS, capacity=64)
    drain = Drain(interval_s=0.005)
    drain.add(recorder.ring, BrokenSink(recorder.columns))
    drain.start()
    for cycle in range(1, 4):
        recorder(make_sample(cycle=cycle))
    time.sleep(0.05)
    drain.stop()

    assert drain.write_failures >= 1


def test_a_table_added_while_draining_is_picked_up():
    drain = Drain(interval_s=0.005)
    drain.start()
    ring, sink = SampleRing(8, 1), ListRowSink(["a"])
    drain.add(ring, sink)
    ring.push(np.array([7.0]))
    time.sleep(0.05)
    drain.stop()

    assert list(sink.rows[:, 0]) == [7.0]


# ----- Joining -------------------------------------------------------------


def base_table(times):
    return Table(["t", "x"], np.column_stack([times, np.arange(len(times))]))


def slow_table(times, values):
    return Table(["t", "current"], np.column_stack([times, values]))


def test_join_asof_takes_the_most_recent_earlier_sample():
    joined = join_asof(
        base_table([1.0, 1.5, 2.0]),
        slow_table([0.9, 1.9], [10.0, 20.0]),
        ["current"],
        tolerance_s=1.0,
    )
    assert list(joined["current"]) == [10.0, 10.0, 20.0]


def test_join_asof_never_reaches_forward_in_time():
    joined = join_asof(
        base_table([1.0]),
        slow_table([2.0], [42.0]),
        ["current"],
        tolerance_s=10.0,
    )
    assert np.isnan(joined["current"][0])


def test_join_asof_reports_the_age_of_what_it_attached():
    joined = join_asof(
        base_table([2.0]),
        slow_table([1.4], [7.0]),
        ["current"],
        tolerance_s=1.0,
    )
    assert joined["current_age_s"][0] == pytest.approx(0.6)


def test_join_asof_drops_a_sample_older_than_the_tolerance():
    joined = join_asof(
        base_table([5.0]),
        slow_table([1.0], [7.0]),
        ["current"],
        tolerance_s=0.5,
    )
    assert np.isnan(joined["current"][0])
    assert np.isnan(joined["current_age_s"][0])


def test_join_asof_does_not_interpolate():
    """A value sampled every few hundred milliseconds says nothing about the
    instants between. Inventing one would let a fit run on a straight line."""
    import inspect

    assert "interpolate" not in inspect.signature(join_asof).parameters

    joined = join_asof(
        base_table([1.0, 1.5, 2.0]),
        slow_table([1.0, 2.0], [0.0, 100.0]),
        ["current"],
        tolerance_s=5.0,
    )
    assert list(joined["current"]) == [0.0, 0.0, 100.0]


def test_join_asof_rejects_an_absent_column():
    with pytest.raises(KeyError):
        join_asof(base_table([1.0]), slow_table([1.0], [1.0]), ["nope"], tolerance_s=1.0)


def test_join_asof_requires_a_tolerance():
    with pytest.raises(ValueError, match="tolerance"):
        join_asof(
            base_table([1.0]), slow_table([1.0], [1.0]), ["current"], tolerance_s=0.0
        )


# ----- Table ---------------------------------------------------------------


def test_table_names_the_columns_it_has_when_asked_for_one_it_lacks():
    table = Table(["t", "x"], np.zeros((1, 2)))
    with pytest.raises(KeyError, match="have t, x"):
        table["missing"]


def test_table_filters_by_column_value():
    recorder = LoopRecorder(JOINTS, capacity=8)
    recorder(make_sample(cycle=1, phase="ok"))
    recorder(make_sample(cycle=2, phase="no_reading"))
    table = Table(recorder.columns, recorder.ring.drain())

    assert list(table.where("phase", PHASE_TO_CODE["ok"])["cycle"]) == [1]
