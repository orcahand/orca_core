"""Fingertips joining or leaving mid-stream: the client re-reads the layout and recovers."""
import itertools
import threading
import time
from pathlib import Path

import pytest

from orca_core.hardware.sensing.constants import TACTILE_STREAM_STALE_REARM_S
from orca_core.hardware.sensing.tactile_mock import feed_combined_frame, feed_resultant_frame
from orca_core.hardware.sensing.tactile_protocol import build_auto_frame
from tests._helpers import wait_until

# 20 AA 56 frames from a real hand (combined mode): 6 with index/middle/ring,
# then the thumb joins (967 B), then the pinky (1126 B, 13 frames).
FIXTURE = Path(__file__).parent / "fixtures" / "tactile_late_join.bin"
IMR = ["index", "middle", "ring"]
ALL = ["thumb", "index", "middle", "ring", "pinky"]
FORCES = {f: [0.0, 0.0, 0.5] for f in ALL}


def _split_frames(raw: bytes) -> list[bytes]:
    frames, pos = [], 0
    while pos < len(raw):
        n = 5 + int.from_bytes(raw[pos + 3:pos + 5], "little") + 1
        frames.append(raw[pos:pos + n])
        pos += n
    return frames


def _latest_fingers(client) -> set[str]:
    reading = client.get_latest()
    return set(reading.forces.forces) if reading else set()


def _gate(monkeypatch, client, name):
    """Hold ``client.<name>`` mid-call until released."""
    entered, release = threading.Event(), threading.Event()
    orig = getattr(client, name)

    def gated(*args, **kwargs):
        result = orig(*args, **kwargs)
        entered.set()
        assert release.wait(timeout=2.0)
        return result

    monkeypatch.setattr(client, name, gated)
    return entered, release


def test_late_joining_fingertips_recover_without_restart(tactile_mock):
    link, client, state = tactile_mock
    frames = _split_frames(FIXTURE.read_bytes())
    assert [len(f) for f in frames[5:8]] == [808, 967, 1126]

    state.connected_fingers = list(IMR)
    client.start_stream(resultant=True, taxels=True)
    for frame in frames[:6]:
        link.feed_bytes(frame)
    wait_until(lambda: client.get_stats().frames_ok == 6)
    stale_ts = client.get_latest().timestamp

    state.connected_fingers = ["thumb"] + IMR
    link.feed_bytes(frames[6])
    wait_until(lambda: client.get_stats().frames_bad_payload_size >= 1)
    # Nothing new is published while the layout is unknown.
    assert client.get_latest().timestamp == stale_ts
    assert _latest_fingers(client) == set(IMR)

    state.connected_fingers = list(ALL)
    full_frames = itertools.cycle(frames[7:])
    wait_until(lambda: _latest_fingers(client) == set(ALL)
               or link.feed_bytes(next(full_frames)), timeout=2.0)
    reading = client.get_latest()
    assert {f: len(t) for f, t in reading.taxels.taxels.items()} == {
        "thumb": 51, "index": 87, "middle": 87, "ring": 87, "pinky": 51,
    }
    stats = client.get_stats()
    assert stats.layout_changes >= 1
    assert stats.stream_rearms == 0


def test_dropped_fingertip_shrinks_the_layout(tactile_mock):
    link, client, state = tactile_mock
    client.start_stream(resultant=True, taxels=False)
    feed_resultant_frame(link, FORCES, ALL)
    client.wait_for_first_frame()

    state.connected_fingers = IMR + ["pinky"]
    wait_until(lambda: _latest_fingers(client) == set(state.connected_fingers)
               or feed_resultant_frame(link, FORCES, state.active_sensors))
    assert client.get_tactile_configuration().active_sensors == state.active_sensors
    assert client.get_stats().layout_changes == 1


def test_mask_that_changes_again_during_a_reread_is_followed(tactile_mock, monkeypatch):
    link, client, state = tactile_mock
    state.connected_fingers = list(IMR)
    client.start_stream(resultant=True, taxels=False)
    entered, release = _gate(monkeypatch, client, "read_connected_sensors")

    state.connected_fingers = ["thumb"] + IMR
    feed_resultant_frame(link, FORCES, state.active_sensors)
    assert entered.wait(timeout=2.0)  # the re-read has the thumb-only mask

    state.connected_fingers = list(ALL)
    feed_resultant_frame(link, FORCES, state.active_sensors)  # dropped, re-read busy
    release.set()

    wait_until(lambda: _latest_fingers(client) == set(ALL)
               or feed_resultant_frame(link, FORCES, state.active_sensors))
    assert client.get_stats().layout_changes == 2


def test_stop_stream_during_a_reread_discards_it(tactile_mock, monkeypatch):
    link, client, state = tactile_mock
    client.start_stream(resultant=True, taxels=False)
    before = client.get_tactile_configuration()
    entered, release = _gate(monkeypatch, client, "read_connected_sensors")

    state.connected_fingers = list(IMR)
    feed_resultant_frame(link, FORCES, IMR)
    assert entered.wait(timeout=2.0)
    client.stop_stream()
    state.write_log.clear()
    release.set()
    client._reread_thread.join(timeout=2.0)

    assert client.get_tactile_configuration() is before
    assert client.get_stats().layout_changes == 0
    assert client.get_latest() is None
    assert state.write_log == []


@pytest.mark.parametrize("board", ["unchanged", "unreachable"])
def test_reread_that_finds_nothing_backs_off(tactile_mock, monkeypatch, board):
    """A mismatch that isn't a layout change must not poll the board every frame."""
    link, client, state = tactile_mock
    client.start_stream(resultant=True, taxels=False)
    reads = []
    orig = client.read_connected_sensors

    def read():
        reads.append(1)
        if board == "unreachable":
            raise RuntimeError("hand serial link disconnected")
        return orig()

    monkeypatch.setattr(client, "read_connected_sensors", read)
    errors = []
    monkeypatch.setattr(threading, "excepthook", errors.append)

    link.feed_bytes(build_auto_frame(b"\x00" * 7))
    wait_until(lambda: reads and not client._reread_thread.is_alive())
    for _ in range(5):
        link.feed_bytes(build_auto_frame(b"\x00" * 7))
    wait_until(lambda: client.get_stats().frames_bad_payload_size == 6)

    assert len(reads) == 1
    assert client.get_stats().layout_changes == 0
    assert errors == []


def test_watchdog_rearm_rereads_the_layout(tactile_mock):
    link, client, state = tactile_mock
    client.start_stream(resultant=True, taxels=False)
    feed_resultant_frame(link, FORCES, ALL)
    client.wait_for_first_frame()

    state.connected_fingers = list(IMR)
    with client._auto_lock:
        client._auto_latest_ts -= TACTILE_STREAM_STALE_REARM_S + 1.0
    client.get_latest_forces()
    client._rearm_thread.join(timeout=2.0)

    assert client.get_tactile_configuration().active_sensors == IMR
    assert client.get_stats().layout_changes == 1


def test_zeroing_restarts_when_a_fingertip_drops_out(tactile_mock):
    link, client, state = tactile_mock
    client.start_stream(resultant=True, taxels=True)
    taxels = {f: [[0.0, 0.0, 0.2]] * n for f, n in state.taxel_counts.items()}
    result, errors = {}, []

    def capture():
        try:
            result["offsets"] = client.capture_taxel_offsets(num_samples=5, timeout_s=5.0)
        except Exception as e:  # noqa: BLE001 - surfaced by the main thread
            errors.append(e)

    capturing = threading.Thread(target=capture)
    capturing.start()
    for _ in range(3):
        feed_combined_frame(link, FORCES, taxels, ALL)
        time.sleep(0.03)
    state.connected_fingers = list(IMR)
    while capturing.is_alive():
        feed_combined_frame(link, FORCES, taxels, state.active_sensors)
        time.sleep(0.03)

    assert errors == []
    assert set(result["offsets"]) == set(IMR)
    assert set(client.resultant_offsets) == set(IMR)
