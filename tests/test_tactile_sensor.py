"""Integration tests for TactileClient on MockHandSerialLink.

Drives a real ``TactileClient`` through a mock serial link with
fixture-built wire frames. Pure protocol codec tests live in
``test_tactile_protocol.py``.
"""

import pytest

from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import DEFAULT_TAXEL_COUNTS
from orca_core.hardware.sensing.tactile_protocol import compute_distal_module_index
from orca_core.hardware.tactile_client import TactileClient, TactileSensorConfiguration

from tests._tactile_helpers import (
    feed_combined_frame,
    feed_resultant_frame,
    feed_taxels_frame,
)

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
    client.start_auto_stream(resultant=True, taxels=False)
    feed_resultant_frame(link, FORCE_VECTORS, state.active_sensors)
    client.wait_for_first_tactile_frame()
    result, ts = client.get_auto_latest()
    client.stop_auto_stream()

    assert ts is not None
    assert result[finger] == FORCE_VECTORS[finger]


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
    client.start_auto_stream(resultant=True, taxels=False)
    feed_resultant_frame(link, forces, state.active_sensors)
    client.wait_for_first_tactile_frame()
    result, _ = client.get_auto_latest()
    client.stop_auto_stream()

    assert set(result.keys()) == set(subset)


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
    client.start_auto_stream(resultant=True, taxels=True)
    feed_combined_frame(link, forces, taxels, state.active_sensors)
    client.wait_for_first_tactile_frame()
    forces_out, taxels_out, ts = client.get_auto_latest_all()
    client.stop_auto_stream()

    assert ts is not None
    assert set(forces_out.keys()) == set(ALL_FINGERS)
    assert set(taxels_out.keys()) == set(ALL_FINGERS)
    for finger in ALL_FINGERS:
        assert forces_out[finger] == [1.0, 0.0, 0.5]
        assert len(taxels_out[finger]) == DEFAULT_TAXEL_COUNTS[finger]


# ---------------------------------------------------------------------------
# Custom data via direct frame feed
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("kind", ["resultant", "taxel"])
def test_custom_provider_is_used(tactile_mock_factory, kind):
    if kind == "resultant":
        marker = [4.2, -3.0, 20.0]
        link, client, state = tactile_mock_factory(["thumb"])
        client.start_auto_stream(resultant=True, taxels=False)
        feed_resultant_frame(link, {"thumb": marker}, state.active_sensors)
        client.wait_for_first_tactile_frame()
        result, _ = client.get_auto_latest()
        client.stop_auto_stream()
        assert result["thumb"] == marker
    else:
        marker = [[9.9, -8.8, 7.7]]
        link, client, state = tactile_mock_factory(
            ["thumb"], taxel_counts={"thumb": 1},
        )
        client.start_auto_stream(resultant=False, taxels=True)
        feed_taxels_frame(link, {"thumb": marker}, state.active_sensors)
        client.wait_for_first_tactile_frame()
        result, _ = client.get_auto_latest_taxels()
        client.stop_auto_stream()
        assert result["thumb"] == marker


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
    client.start_auto_stream(resultant=True, taxels=False)
    feed_resultant_frame(link, {finger: [5.0, 3.0, 10.0]}, state.active_sensors)
    client.wait_for_first_tactile_frame()
    result, _ = client.get_auto_latest()
    client.stop_auto_stream()

    assert result[finger] == [4.0, 2.5, 8.0]


def test_taxel_offsets_applied_in_stream(tactile_mock_factory):
    taxels = [[2.0, 1.0, 5.0], [3.0, 2.0, 8.0]]
    offsets = [[0.5, 0.5, 1.0], [1.0, 1.0, 2.0]]
    link, client, state = tactile_mock_factory(
        ["thumb"], taxel_counts={"thumb": 2},
    )
    client.set_taxel_offsets({"thumb": offsets})
    client.start_auto_stream(resultant=False, taxels=True)
    feed_taxels_frame(link, {"thumb": taxels}, state.active_sensors)
    client.wait_for_first_tactile_frame()
    result, _ = client.get_auto_latest_taxels()
    client.stop_auto_stream()

    assert result["thumb"] == [[1.5, 0.5, 4.0], [2.0, 1.0, 6.0]]


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


def test_get_auto_latest_before_stream_returns_none(tactile_mock):
    _, client, _ = tactile_mock
    result, ts = client.get_auto_latest()
    assert result is None
    assert ts is None


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
