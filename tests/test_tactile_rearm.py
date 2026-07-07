"""Stream re-arm: a stale running stream is re-enabled from the read path."""
import time

import pytest

from orca_core.hardware.sensing.constants import (
    ADDR_AUTO_DATA_TYPE,
    ADDR_AUTO_ENABLE,
    REGISTER_ENABLE,
    TACTILE_STREAM_STALE_REARM_S,
)
from tests._tactile_helpers import feed_resultant_frame

FORCES = {f: [0.0, 0.0, 0.5] for f in ("thumb", "index", "middle", "ring", "pinky")}


def _start_stream(link, client, state):
    client.start_stream(resultant=True, taxels=False)
    feed_resultant_frame(link, FORCES, state.active_sensors)
    client.wait_for_first_frame()
    state.write_log.clear()


def _backdate_last_frame(client, seconds):
    with client._auto_lock:
        client._auto_latest_ts -= seconds


def _wait_for_rearm_thread(client, timeout=2.0):
    t = client._rearm_thread
    assert t is not None, "re-arm thread was never spawned"
    t.join(timeout)
    assert not t.is_alive()


def test_stale_stream_triggers_rearm(tactile_mock):
    link, client, state = tactile_mock
    _start_stream(link, client, state)

    _backdate_last_frame(client, TACTILE_STREAM_STALE_REARM_S + 1.0)
    client.get_latest_forces()
    _wait_for_rearm_thread(client)

    addrs = [a for a, _ in state.write_log]
    assert ADDR_AUTO_DATA_TYPE in addrs
    assert ADDR_AUTO_ENABLE in addrs
    enable_writes = [d for a, d in state.write_log if a == ADDR_AUTO_ENABLE]
    assert enable_writes[-1] == REGISTER_ENABLE
    assert client.get_stats().stream_rearms == 1


def test_rearm_rate_limited(tactile_mock):
    link, client, state = tactile_mock
    _start_stream(link, client, state)

    _backdate_last_frame(client, TACTILE_STREAM_STALE_REARM_S + 1.0)
    client.get_latest_forces()
    _wait_for_rearm_thread(client)
    n_writes = len(state.write_log)

    # Still stale, but inside the min-interval window: no second attempt.
    client.get_latest_forces()
    client.get_latest_taxels()
    client.get_latest()
    time.sleep(0.05)
    assert len(state.write_log) == n_writes
    assert client.get_stats().stream_rearms == 1


def test_fresh_stream_never_rearms(tactile_mock):
    link, client, state = tactile_mock
    _start_stream(link, client, state)

    for _ in range(20):
        client.get_latest_forces()
    time.sleep(0.05)
    assert state.write_log == []
    assert client.get_stats().stream_rearms == 0


def test_no_rearm_when_stream_not_started(tactile_mock):
    link, client, state = tactile_mock
    for _ in range(5):
        client.get_latest_forces()
    time.sleep(0.05)
    assert state.write_log == []


def test_no_rearm_after_stop_stream(tactile_mock):
    link, client, state = tactile_mock
    _start_stream(link, client, state)
    _backdate_last_frame(client, TACTILE_STREAM_STALE_REARM_S + 1.0)
    client.stop_stream()
    state.write_log.clear()  # stop_stream's own disable write

    client.get_latest_forces()
    time.sleep(0.05)
    assert state.write_log == []


def test_rearm_failure_is_swallowed(tactile_mock, monkeypatch):
    link, client, state = tactile_mock
    _start_stream(link, client, state)

    def boom(*a, **k):
        raise OSError("device unreachable")

    monkeypatch.setattr(client, "set_auto_data_type", boom)
    _backdate_last_frame(client, TACTILE_STREAM_STALE_REARM_S + 1.0)
    client.get_latest_forces()  # must not raise
    _wait_for_rearm_thread(client)
    assert client.get_stats().stream_rearms == 1


def test_stale_before_first_frame_uses_stream_start(tactile_mock):
    """A stream that starts and then never delivers a single frame must still
    re-arm, using the stream-start time as the staleness baseline."""
    link, client, state = tactile_mock
    client.start_stream(resultant=True, taxels=False)
    state.write_log.clear()
    with client._auto_lock:
        client._auto_started_ts -= TACTILE_STREAM_STALE_REARM_S + 1.0

    client.get_latest_forces()
    _wait_for_rearm_thread(client)
    assert any(a == ADDR_AUTO_ENABLE for a, _ in state.write_log)
