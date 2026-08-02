"""Stream re-arm: a stale running stream is re-enabled from the read path."""
import threading
import time

import pytest

from orca_core.hardware.sensing.constants import (
    ADDR_AUTO_DATA_TYPE,
    ADDR_AUTO_ENABLE,
    REGISTER_DISABLE,
    REGISTER_ENABLE,
    TACTILE_STREAM_STALE_REARM_S,
)
from orca_core.hardware.sensing.tactile_mock import feed_resultant_frame

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


def test_stop_during_inflight_rearm_leaves_device_disabled(tactile_mock, monkeypatch):
    """stop_stream() racing an already-spawned re-arm thread must end with
    the device stream disabled, never re-enabled behind the caller's back."""
    link, client, state = tactile_mock
    _start_stream(link, client, state)

    entered = threading.Event()
    release = threading.Event()
    orig_set_type = client.set_auto_data_type

    def gated(*args, **kwargs):
        entered.set()
        assert release.wait(timeout=2.0)
        return orig_set_type(*args, **kwargs)

    monkeypatch.setattr(client, "set_auto_data_type", gated)
    _backdate_last_frame(client, TACTILE_STREAM_STALE_REARM_S + 1.0)
    client.get_latest_forces()  # spawns the re-arm thread
    assert entered.wait(timeout=2.0)  # re-arm passed its guard, writes pending

    stopper = threading.Thread(target=client.stop_stream, daemon=True)
    stopper.start()
    time.sleep(0.05)  # let stop_stream reach its device-disable step
    release.set()
    stopper.join(timeout=2.0)
    assert not stopper.is_alive()
    _wait_for_rearm_thread(client)

    enable_writes = [d for a, d in state.write_log if a == ADDR_AUTO_ENABLE]
    assert enable_writes[-1] == REGISTER_DISABLE


def test_rearm_spawned_before_stop_makes_no_writes(tactile_mock):
    """A re-arm thread that reaches its guard after stop_stream() must abort."""
    link, client, state = tactile_mock
    _start_stream(link, client, state)
    generation = client._stream_generation
    client.stop_stream()
    state.write_log.clear()

    client._rearm_stream(3.0, True, False, generation)
    assert state.write_log == []


def test_rearm_for_previous_stream_generation_is_ignored(tactile_mock):
    """A stale re-arm surviving a stop/start cycle must not write with the
    old stream's modes."""
    link, client, state = tactile_mock
    _start_stream(link, client, state)
    stale_generation = client._stream_generation
    client.stop_stream()
    _start_stream(link, client, state)
    state.write_log.clear()

    client._rearm_stream(3.0, True, False, stale_generation)
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
