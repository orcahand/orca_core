"""Integration tests for TactileClient on MockHandSerialLink.

Drives a real ``TactileClient`` through a mock serial link with
fixture-built wire frames. Pure protocol codec tests live in
``test_tactile_protocol.py``.
"""

import threading
import time

import pytest

from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import (
    ADDR_AUTO_ENABLE,
    DEFAULT_TAXEL_COUNTS,
    REGISTER_DISABLE,
    TACTILE_REGISTER_ATTEMPTS,
)
from orca_core.hardware.sensing.tactile_mock import (
    feed_combined_frame,
    feed_resultant_frame,
    feed_taxels_frame,
)
from orca_core.hardware.sensing.tactile_protocol import compute_distal_module_index
from orca_core.hardware.tactile_client import TactileClient, TactileSensorConfiguration

from tests._helpers import wait_until

ALL_FINGERS = ["thumb", "index", "middle", "ring", "pinky"]


def _make_config(
    connected_fingers: list[str],
    taxel_counts: dict[str, int] | None = None,
    finger_to_sensor_id: dict[str, int] | None = None,
) -> TactileSensorConfiguration:
    if taxel_counts is None:
        taxel_counts = DEFAULT_TAXEL_COUNTS
    if finger_to_sensor_id is None:
        finger_to_sensor_id = {"thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4}

    connected = {f: (f in connected_fingers) for f in ALL_FINGERS}
    num_taxels = {f: taxel_counts.get(f, 0) for f in connected_fingers}
    module_indices = {f: compute_distal_module_index(finger_to_sensor_id[f]) for f in connected_fingers}

    return TactileSensorConfiguration(
        connected=connected,
        num_taxels=num_taxels,
        module_indices=module_indices,
        finger_to_sensor_id=finger_to_sensor_id,
    )


# ---------------------------------------------------------------------------
# Resultant forces — round-trip per finger
# ---------------------------------------------------------------------------

FORCE_VECTORS = {
    "thumb": [1.5, -2.0, 3.0],
    "index": [0.1, 0.2, 0.3],
    "middle": [-1.0, 0.5, 10.0],
    "ring": [4.0, -4.0, 4.0],
    "pinky": [0.0, 0.0, 0.5],
}


@pytest.mark.parametrize("finger", ALL_FINGERS)
def test_resultant_round_trip_per_finger(tactile_mock, finger):
    link, client, state = tactile_mock
    client.start_stream(resultant=True, taxels=False)
    feed_resultant_frame(link, FORCE_VECTORS, state.active_sensors)
    client.wait_for_first_frame()
    reading = client.get_latest_forces()
    client.stop_stream()

    assert reading is not None
    assert reading.timestamp is not None
    assert reading[finger] == FORCE_VECTORS[finger]


@pytest.mark.parametrize(
    "subset",
    [
        ["thumb"],
        ["thumb", "pinky"],
        ["index", "middle", "ring"],
        ALL_FINGERS,
    ],
)
def test_only_connected_sensors_returned(tactile_mock_factory, subset):
    link, client, state = tactile_mock_factory(subset)
    forces = {f: [1.0, 0.0, 0.5] for f in subset}
    client.start_stream(resultant=True, taxels=False)
    feed_resultant_frame(link, forces, state.active_sensors)
    client.wait_for_first_frame()
    reading = client.get_latest_forces()
    client.stop_stream()

    assert set(reading.fingers) == set(subset)


# ---------------------------------------------------------------------------
# Combined mode
# ---------------------------------------------------------------------------

def test_combined_mode_returns_both_streams(tactile_mock):
    link, client, state = tactile_mock
    forces = {f: [1.0, 0.0, 0.5] for f in ALL_FINGERS}
    taxels = {
        f: [[1.0, 1.0, 1.0] for _ in range(DEFAULT_TAXEL_COUNTS[f])]
        for f in ALL_FINGERS
    }
    client.start_stream(resultant=True, taxels=True)
    feed_combined_frame(link, forces, taxels, state.active_sensors)
    client.wait_for_first_frame()
    reading = client.get_latest()
    client.stop_stream()

    assert reading is not None
    assert reading.timestamp is not None
    assert set(reading.forces.fingers) == set(ALL_FINGERS)
    assert set(reading.taxels.fingers) == set(ALL_FINGERS)
    for finger in ALL_FINGERS:
        assert reading.forces[finger] == [1.0, 0.0, 0.5]
        assert len(reading.taxels[finger]) == DEFAULT_TAXEL_COUNTS[finger]


# ---------------------------------------------------------------------------
# Custom data via direct frame feed
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("kind", ["resultant", "taxel"])
def test_custom_provider_is_used(tactile_mock_factory, kind):
    if kind == "resultant":
        marker = [4.2, -3.0, 20.0]
        link, client, state = tactile_mock_factory(["thumb"])
        client.start_stream(resultant=True, taxels=False)
        feed_resultant_frame(link, {"thumb": marker}, state.active_sensors)
        client.wait_for_first_frame()
        reading = client.get_latest_forces()
        client.stop_stream()
        assert reading["thumb"] == marker
    else:
        marker = [[9.9, -8.8, 7.7]]
        link, client, state = tactile_mock_factory(
            ["thumb"], taxel_counts={"thumb": 1},
        )
        client.start_stream(resultant=False, taxels=True)
        feed_taxels_frame(link, {"thumb": marker}, state.active_sensors)
        client.wait_for_first_frame()
        reading = client.get_latest_taxels()
        client.stop_stream()
        assert reading["thumb"] == marker


# ---------------------------------------------------------------------------
# Offset logic — synchronous register-read path
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("finger", ALL_FINGERS)
def test_resultant_offsets_applied(tactile_mock_factory, finger):
    _, client, state = tactile_mock_factory([finger])
    state.resultant_forces = {finger: [5.0, 3.0, 10.0]}
    client.set_taxel_offsets({finger: [[1.0, 0.5, 2.0]]})

    result = client.read_resultant_force()
    assert result[finger] == [4.0, 2.5, 8.0]


@pytest.mark.parametrize("finger", ALL_FINGERS)
def test_fz_clamped_to_zero(tactile_mock_factory, finger):
    _, client, state = tactile_mock_factory([finger])
    state.resultant_forces = {finger: [0.0, 0.0, 1.0]}
    client.set_taxel_offsets({finger: [[0.0, 0.0, 5.0]]})

    result = client.read_resultant_force()
    assert result[finger][2] == 0.0


def test_clear_offsets(tactile_mock_factory):
    _, client, state = tactile_mock_factory(["thumb"])
    state.resultant_forces = {"thumb": [5.0, 3.0, 10.0]}
    client.set_taxel_offsets({"thumb": [[1.0, 0.5, 2.0]]})
    client.clear_taxel_offsets()

    result = client.read_resultant_force()
    assert result["thumb"] == [5.0, 3.0, 10.0]


# ---------------------------------------------------------------------------
# Offset logic — auto-stream path
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("finger", ALL_FINGERS)
def test_stream_offsets_applied(tactile_mock_factory, finger):
    link, client, state = tactile_mock_factory([finger])
    client.set_taxel_offsets({finger: [[1.0, 0.5, 2.0]]})
    client.start_stream(resultant=True, taxels=False)
    feed_resultant_frame(link, {finger: [5.0, 3.0, 10.0]}, state.active_sensors)
    client.wait_for_first_frame()
    reading = client.get_latest_forces()
    client.stop_stream()

    assert reading[finger] == [4.0, 2.5, 8.0]


def test_taxel_offsets_applied_in_stream(tactile_mock_factory):
    taxels = [[2.0, 1.0, 5.0], [3.0, 2.0, 8.0]]
    offsets = [[0.5, 0.5, 1.0], [1.0, 1.0, 2.0]]
    link, client, state = tactile_mock_factory(
        ["thumb"], taxel_counts={"thumb": 2},
    )
    client.set_taxel_offsets({"thumb": offsets})
    client.start_stream(resultant=False, taxels=True)
    feed_taxels_frame(link, {"thumb": taxels}, state.active_sensors)
    client.wait_for_first_frame()
    reading = client.get_latest_taxels()
    client.stop_stream()

    assert reading["thumb"] == [[1.5, 0.5, 4.0], [2.0, 1.0, 6.0]]


# ---------------------------------------------------------------------------
# Offset capture — deadline behaviour
# ---------------------------------------------------------------------------

def _start_taxel_pump(link, taxels, active_sensors, period_s=0.005):
    """Background thread feeding taxel frames at a steady rate."""
    stop = threading.Event()

    def _run():
        while not stop.is_set():
            feed_taxels_frame(link, taxels, active_sensors)
            time.sleep(period_s)

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    return stop, thread


def test_capture_taxel_offsets_averages_and_applies(tactile_mock_factory):
    link, client, state = tactile_mock_factory(["thumb"], taxel_counts={"thumb": 1})
    client.start_stream(resultant=False, taxels=True)
    stop, thread = _start_taxel_pump(
        link, {"thumb": [[1.0, 2.0, 4.0]]}, state.active_sensors
    )
    try:
        offsets = client.capture_taxel_offsets(num_samples=3)
        assert offsets == {"thumb": [[1.0, 2.0, 4.0]]}

        # Offsets are live: frames decoded after the capture come back zeroed.
        wait_until(
            lambda: client.get_latest_taxels()["thumb"] == [[0.0, 0.0, 0.0]]
        )
    finally:
        stop.set()
        thread.join(timeout=1.0)
        client.stop_stream()


def test_capture_taxel_offsets_times_out_when_stream_stalls(tactile_mock_factory):
    link, client, state = tactile_mock_factory(["thumb"], taxel_counts={"thumb": 1})
    client.start_stream(resultant=False, taxels=True)
    feed_taxels_frame(link, {"thumb": [[1.0, 1.0, 1.0]]}, state.active_sensors)
    client.wait_for_first_frame()

    with pytest.raises(TimeoutError, match="stalled during offset capture"):
        client.capture_taxel_offsets(num_samples=5, timeout_s=0.2)
    client.stop_stream()


def test_capture_taxel_offsets_times_out_when_stream_dead_at_entry(
    tactile_mock_factory,
):
    _, client, _ = tactile_mock_factory(["thumb"], taxel_counts={"thumb": 1})
    client.start_stream(resultant=False, taxels=True)

    with pytest.raises(TimeoutError, match="0/1"):
        client.capture_taxel_offsets(num_samples=1, timeout_s=0.2)
    client.stop_stream()


def test_failed_capture_restores_prior_offsets(tactile_mock_factory):
    link, client, state = tactile_mock_factory(["thumb"], taxel_counts={"thumb": 1})
    client.set_taxel_offsets({"thumb": [[0.5, 0.5, 0.5]]})
    client.start_stream(resultant=False, taxels=True)

    with pytest.raises(TimeoutError):
        client.capture_taxel_offsets(num_samples=2, timeout_s=0.2)

    # The pre-capture offsets still apply to frames after the failure.
    feed_taxels_frame(link, {"thumb": [[1.0, 1.0, 1.0]]}, state.active_sensors)
    client.wait_for_first_frame()
    reading = client.get_latest_taxels()
    client.stop_stream()
    assert reading["thumb"] == [[0.5, 0.5, 0.5]]


# ---------------------------------------------------------------------------
# Timestamps
# ---------------------------------------------------------------------------

def test_reading_timestamps_use_monotonic_clock(tactile_mock):
    """Tactile timestamps share EncoderReading's time.monotonic() base so
    staleness logic and cross-stream correlation survive wall-clock steps."""
    link, client, state = tactile_mock
    client.start_stream(resultant=True, taxels=False)
    feed_resultant_frame(link, FORCE_VECTORS, state.active_sensors)
    client.wait_for_first_frame()
    reading = client.get_latest_forces()
    client.stop_stream()

    assert reading.timestamp == pytest.approx(time.monotonic(), abs=5.0)


# ---------------------------------------------------------------------------
# Error paths
# ---------------------------------------------------------------------------

def test_read_before_connect_raises():
    link = MockHandSerialLink()
    link.connect()
    try:
        client = TactileClient(link)
        with pytest.raises(OSError, match="connect"):
            client.read_resultant_force()
    finally:
        link.disconnect()


def test_get_latest_forces_before_stream_returns_none(tactile_mock):
    _, client, _ = tactile_mock
    assert client.get_latest_forces() is None


# ---------------------------------------------------------------------------
# TactileSensorConfiguration ordering
# ---------------------------------------------------------------------------

def test_slot_order_default():
    config = _make_config(ALL_FINGERS)
    assert config.active_sensors == ["thumb", "index", "middle", "ring", "pinky"]


def test_slot_order_custom_mapping():
    custom_map = {"thumb": 1, "index": 3, "middle": 0, "ring": 2, "pinky": 4}
    config = _make_config(ALL_FINGERS, finger_to_sensor_id=custom_map)
    assert config.active_sensors == ["middle", "thumb", "ring", "index", "pinky"]


def test_slot_order_subset_preserves_order():
    config = _make_config(["pinky", "thumb"])
    assert config.active_sensors == ["thumb", "pinky"]


# ---------------------------------------------------------------------------
# Stream re-arm bounding (stop/disconnect must never wait behind retries)
# ---------------------------------------------------------------------------

def test_rearm_register_writes_use_a_single_attempt(tactile_mock, monkeypatch):
    """Re-arm writes must not retry: retries would hold the stream control
    lock and stall a concurrent stop_stream/disconnect for seconds."""
    link, client, state = tactile_mock
    client.start_stream(resultant=True, taxels=False)

    recorded = []
    orig = client._send_register_request

    def spy(request, response_timeout_s, attempts=TACTILE_REGISTER_ATTEMPTS):
        recorded.append(attempts)
        return orig(request, response_timeout_s, attempts)

    monkeypatch.setattr(client, "_send_register_request", spy)
    client._rearm_stream(3.0, True, False, client._stream_generation)

    assert recorded == [1, 1]


def test_stop_between_rearm_writes_skips_the_enable(tactile_mock, monkeypatch):
    """A stop_stream() landing between the re-arm's two register writes must
    win: the enable write is skipped and the device stays disabled."""
    link, client, state = tactile_mock
    client.start_stream(resultant=True, taxels=False)
    state.write_log.clear()

    wrote_type = threading.Event()
    release = threading.Event()
    orig_set_type = client.set_auto_data_type

    def gated(*args, **kwargs):
        result = orig_set_type(*args, **kwargs)
        wrote_type.set()
        assert release.wait(timeout=2.0)
        return result

    monkeypatch.setattr(client, "set_auto_data_type", gated)

    generation = client._stream_generation
    rearm = threading.Thread(
        target=client._rearm_stream, args=(3.0, True, False, generation), daemon=True
    )
    rearm.start()
    assert wrote_type.wait(timeout=2.0)

    # stop_stream bumps the generation immediately, then queues behind the
    # re-arm's control lock for its own disable write.
    stopper = threading.Thread(target=client.stop_stream, daemon=True)
    stopper.start()
    wait_until(lambda: client._stream_generation != generation)
    release.set()
    rearm.join(timeout=2.0)
    stopper.join(timeout=2.0)
    assert not rearm.is_alive() and not stopper.is_alive()

    enable_writes = [d for a, d in state.write_log if a == ADDR_AUTO_ENABLE]
    assert enable_writes == [REGISTER_DISABLE]
