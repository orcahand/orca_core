"""Tests for MockTactileClient integration and TactileSensorConfiguration contracts.

Validates the mock's lifecycle (connect → stream → read → stop), offset logic,
and configuration ordering. Pure protocol codec tests live in test_protocol.py.
"""

import pytest

from orca_core.hardware.tactile_client import TactileSensorConfiguration
from orca_core.hardware.mock_tactile_client import MockTactileClient
from orca_core.hardware.sensing.constants import DEFAULT_TAXEL_COUNTS
from orca_core.hardware.sensing.protocol import compute_distal_module_index

ALL_FINGERS = ["thumb", "index", "middle", "ring", "pinky"]


@pytest.fixture
def mock():
    """Connected MockTactileClient with all fingers, cleaned up on teardown."""
    client = MockTactileClient(connected_sensors=ALL_FINGERS)
    client.connect()
    yield client
    client.disconnect()


@pytest.fixture
def mock_factory():
    """Factory for connected mocks with custom finger subsets."""
    created = []

    def _make(connected_sensors, **kwargs):
        client = MockTactileClient(connected_sensors=connected_sensors, **kwargs)
        client.connect()
        created.append(client)
        return client

    yield _make
    for client in created:
        client.disconnect()


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
def test_resultant_round_trip_per_finger(mock, finger):
    mock.set_mock_forces(FORCE_VECTORS)
    mock.start_auto_stream(resultant=True, taxels=False)
    mock.wait_for_first_frame()
    result, ts = mock.get_auto_latest()
    mock.stop_auto_stream()

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
def test_only_connected_sensors_returned(mock_factory, subset):
    mock = mock_factory(subset)
    mock.start_auto_stream(resultant=True, taxels=False)
    mock.wait_for_first_frame()
    result, _ = mock.get_auto_latest()
    mock.stop_auto_stream()

    assert set(result.keys()) == set(subset)


# ---------------------------------------------------------------------------
# Combined mode
# ---------------------------------------------------------------------------

def test_combined_mode_returns_both_streams(mock):
    mock.set_mock_forces({f: [1.0, 0.0, 0.5] for f in ALL_FINGERS})
    mock.start_auto_stream(resultant=True, taxels=True)
    mock.wait_for_first_frame()
    forces, taxels, ts = mock.get_auto_latest_all()
    mock.stop_auto_stream()

    assert ts is not None
    assert set(forces.keys()) == set(ALL_FINGERS)
    assert set(taxels.keys()) == set(ALL_FINGERS)
    for finger in ALL_FINGERS:
        assert forces[finger] == [1.0, 0.0, 0.5]
        assert len(taxels[finger]) == DEFAULT_TAXEL_COUNTS[finger]


# ---------------------------------------------------------------------------
# Provider injection
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "kind",
    ["resultant", "taxel"],
)
def test_custom_provider_is_used(kind):
    if kind == "resultant":
        marker = [4.2, -3.0, 20.0]
        mock = MockTactileClient(
            connected_sensors=["thumb"],
            resultant_provider=lambda: {"thumb": marker},
        )
        mock.connect()
        mock.start_auto_stream(resultant=True, taxels=False)
        mock.wait_for_first_frame()
        result, _ = mock.get_auto_latest()
        mock.stop_auto_stream()
        mock.disconnect()
        assert result["thumb"] == marker
    else:
        marker = [[9.9, -8.8, 7.7]]
        mock = MockTactileClient(
            connected_sensors=["thumb"],
            taxel_counts={"thumb": 1},
            taxel_provider=lambda: {"thumb": marker},
        )
        mock.connect()
        mock.start_auto_stream(resultant=False, taxels=True)
        mock.wait_for_first_frame()
        result, _ = mock.get_auto_latest_taxels()
        mock.stop_auto_stream()
        mock.disconnect()
        assert result["thumb"] == marker


# ---------------------------------------------------------------------------
# Offset logic
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("finger", ALL_FINGERS)
def test_resultant_offsets_applied(mock_factory, finger):
    mock = mock_factory([finger])
    mock.set_mock_forces({finger: [5.0, 3.0, 10.0]})
    mock.set_taxel_offsets({finger: [[1.0, 0.5, 2.0]]})

    result = mock.read_resultant_force()
    assert result[finger] == [4.0, 2.5, 8.0]


@pytest.mark.parametrize("finger", ALL_FINGERS)
def test_fz_clamped_to_zero(mock_factory, finger):
    mock = mock_factory([finger])
    mock.set_mock_forces({finger: [0.0, 0.0, 1.0]})
    mock.set_taxel_offsets({finger: [[0.0, 0.0, 5.0]]})

    result = mock.read_resultant_force()
    assert result[finger][2] == 0.0


def test_clear_offsets(mock_factory):
    mock = mock_factory(["thumb"])
    mock.set_mock_forces({"thumb": [5.0, 3.0, 10.0]})
    mock.set_taxel_offsets({"thumb": [[1.0, 0.5, 2.0]]})
    mock.clear_taxel_offsets()

    result = mock.read_resultant_force()
    assert result["thumb"] == [5.0, 3.0, 10.0]


@pytest.mark.parametrize("finger", ALL_FINGERS)
def test_stream_offsets_applied(mock_factory, finger):
    mock = mock_factory([finger])
    mock.set_mock_forces({finger: [5.0, 3.0, 10.0]})
    mock.set_taxel_offsets({finger: [[1.0, 0.5, 2.0]]})
    mock.start_auto_stream(resultant=True, taxels=False)
    mock.wait_for_first_frame()
    result, _ = mock.get_auto_latest()
    mock.stop_auto_stream()

    assert result[finger] == [4.0, 2.5, 8.0]


def test_taxel_offsets_applied_in_stream(mock_factory):
    taxels = [[2.0, 1.0, 5.0], [3.0, 2.0, 8.0]]
    offsets = [[0.5, 0.5, 1.0], [1.0, 1.0, 2.0]]
    mock = mock_factory(
        ["thumb"],
        taxel_counts={"thumb": 2},
        taxel_provider=lambda: {"thumb": [list(t) for t in taxels]},
    )
    mock.set_taxel_offsets({"thumb": offsets})
    mock.start_auto_stream(resultant=False, taxels=True)
    mock.wait_for_first_frame()
    result, _ = mock.get_auto_latest_taxels()
    mock.stop_auto_stream()

    assert result["thumb"] == [[1.5, 0.5, 4.0], [2.0, 1.0, 6.0]]


# ---------------------------------------------------------------------------
# Error paths
# ---------------------------------------------------------------------------

def test_read_before_connect_raises():
    mock = MockTactileClient(connected_sensors=ALL_FINGERS)
    with pytest.raises(OSError, match="connect"):
        mock.read_resultant_force()


def test_get_auto_latest_before_stream_returns_none(mock):
    result, ts = mock.get_auto_latest()
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
