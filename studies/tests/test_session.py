"""Tests for the recording session: manifest lifecycle, table registration,
event log, and what a dataset says about itself after each way of ending.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from studies.record import Dataset, LoopRecorder
from studies.record.session import (
    STATUS_ABORTED,
    STATUS_CRASHED,
    STATUS_OK,
    STATUS_RUNNING,
    RecordingSession,
)

from studies.tests.test_record import JOINTS, make_sample


def build(tmp_path, **kwargs):
    return RecordingSession(
        tmp_path / "run", experiment="timing", drain_interval_s=0.005, **kwargs
    )


def test_manifest_exists_and_says_running_before_the_run_ends(tmp_path):
    session = build(tmp_path)
    meta = json.loads(session.manifest_path.read_text())

    assert meta["status"] == STATUS_RUNNING
    assert meta["experiment"] == "timing"
    session.close()


def test_a_clean_run_closes_as_ok(tmp_path):
    with build(tmp_path):
        pass

    assert Dataset(tmp_path / "run").status == STATUS_OK
    assert not Dataset(tmp_path / "run").truncated


def test_an_exception_closes_the_manifest_as_crashed(tmp_path):
    with pytest.raises(ValueError):
        with build(tmp_path):
            raise ValueError("bench cable pulled")

    dataset = Dataset(tmp_path / "run")
    assert dataset.status == STATUS_CRASHED
    assert "bench cable pulled" in dataset.meta["status_reason"]
    assert dataset.truncated


def test_an_aborted_run_is_not_treated_as_truncated(tmp_path):
    session = build(tmp_path).start()
    session.abort("encoder freshness exceeded the limit")

    dataset = Dataset(tmp_path / "run")
    assert dataset.status == STATUS_ABORTED
    assert not dataset.truncated
    assert dataset.events()[-1]["reason"].startswith("encoder freshness")


def test_a_run_that_never_closed_reads_as_truncated(tmp_path):
    """The manifest is the only evidence a table stops early on purpose."""
    build(tmp_path)

    assert Dataset(tmp_path / "run").truncated


def test_tables_are_written_and_counted(tmp_path):
    recorder = LoopRecorder(JOINTS, capacity=64)
    with build(tmp_path) as session:
        session.add_table("loop", recorder.ring, recorder.columns, source=recorder)
        for cycle in range(1, 6):
            recorder(make_sample(cycle=cycle))

    dataset = Dataset(tmp_path / "run")
    assert dataset.meta["tables"]["loop"]["rows"] == 5
    assert dataset.meta["tables"]["loop"]["dropped"] == 0
    assert list(dataset.table("loop")["cycle"]) == [1, 2, 3, 4, 5]


def test_a_lossy_run_records_what_it_lost(tmp_path):
    """A dataset must not look complete when the ring overran."""
    recorder = LoopRecorder(JOINTS, capacity=2)
    session = build(tmp_path)
    session.add_table("loop", recorder.ring, recorder.columns, source=recorder)
    for cycle in range(1, 10):
        recorder(make_sample(cycle=cycle))
    session.close()

    assert Dataset(tmp_path / "run").meta["tables"]["loop"]["dropped"] == 7


def test_a_table_cannot_be_registered_twice(tmp_path):
    recorder = LoopRecorder(JOINTS, capacity=8)
    session = build(tmp_path)
    session.add_table("loop", recorder.ring, recorder.columns)
    with pytest.raises(ValueError, match="already registered"):
        session.add_table("loop", recorder.ring, recorder.columns)
    session.close()


def test_events_carry_state_the_manifest_cannot(tmp_path):
    """Gains change mid-run; the manifest is written once."""
    with build(tmp_path) as session:
        session.record_event("gain_change", joint="index_mcp", kp=0.5)
        session.record_event("gain_change", joint="index_mcp", kp=0.625)

    events = Dataset(tmp_path / "run").events()
    assert [e["kp"] for e in events] == [0.5, 0.625]
    assert all(e["event"] == "gain_change" for e in events)
    assert events[0]["t"] <= events[1]["t"]


def test_close_is_idempotent(tmp_path):
    session = build(tmp_path).start()
    session.close()
    session.close()

    assert Dataset(tmp_path / "run").status == STATUS_OK


def test_an_abort_inside_the_context_survives_a_later_exception(tmp_path):
    """An abort names the real reason; a teardown failure must not overwrite it."""
    with pytest.raises(RuntimeError):
        with build(tmp_path) as session:
            session.abort("joint left its travel")
            raise RuntimeError("teardown also failed")

    assert Dataset(tmp_path / "run").status == STATUS_ABORTED


def test_metadata_passed_in_is_kept(tmp_path):
    session = build(tmp_path, metadata={"hand": "orcahand-joint-right"})
    session.close()

    assert Dataset(tmp_path / "run").meta["hand"] == "orcahand-joint-right"


def test_dataset_requires_a_manifest(tmp_path):
    (tmp_path / "empty").mkdir()
    with pytest.raises(FileNotFoundError, match="meta.json"):
        Dataset(tmp_path / "empty")
