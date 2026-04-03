"""Tests for MockSensorClient integration and SensorConfiguration contracts.

Validates the mock's lifecycle (connect → stream → read → stop), offset logic,
dynamic reconfiguration, and configuration ordering. Pure protocol codec tests
live in test_protocol.py; taxel coordinate tests live in test_taxel_coordinates.py.
"""

import time

import pytest

from orca_core.hardware.sensor_client import SensorConfiguration
from orca_core.hardware.mock_sensor_client import MockSensorClient
from orca_core.hardware.sensing.constants import (
    DEFAULT_TAXEL_COUNTS,
    BYTES_PER_RESULTANT,
    BYTES_PER_TAXEL,
)
from orca_core.hardware.sensing.protocol import compute_distal_module_index

EXPECTED_TAXEL_COUNTS = DEFAULT_TAXEL_COUNTS

ALL_FINGERS = ["thumb", "index", "middle", "ring", "pinky"]

POLL_INTERVAL = 0.001
POLL_TIMEOUT = 2.0


def _poll_auto_latest(mock, timeout=POLL_TIMEOUT):
    """Poll until auto-stream produces a resultant frame."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        result, ts = mock.get_auto_latest()
        if result is not None:
            return result, ts
        time.sleep(POLL_INTERVAL)
    raise TimeoutError("No auto-stream frame received within timeout")


def _poll_auto_latest_taxels(mock, timeout=POLL_TIMEOUT):
    """Poll until auto-stream produces a taxel frame."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        result, ts = mock.get_auto_latest_taxels()
        if result is not None:
            return result, ts
        time.sleep(POLL_INTERVAL)
    raise TimeoutError("No auto-stream taxel frame received within timeout")


def _poll_auto_latest_all(mock, timeout=POLL_TIMEOUT):
    """Poll until auto-stream produces both resultant and taxel frames."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        forces, taxels, ts = mock.get_auto_latest_all()
        if forces is not None and taxels is not None:
            return forces, taxels, ts
        time.sleep(POLL_INTERVAL)
    raise TimeoutError("No combined auto-stream frame received within timeout")


def _make_config(
    connected_fingers: list[str],
    taxel_counts: dict[str, int] | None = None,
    finger_to_sensor_id: dict[str, int] | None = None,
) -> SensorConfiguration:
    """Build a SensorConfiguration for testing."""
    if taxel_counts is None:
        taxel_counts = EXPECTED_TAXEL_COUNTS
    if finger_to_sensor_id is None:
        finger_to_sensor_id = {"thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4}

    connected = {f: (f in connected_fingers) for f in ALL_FINGERS}
    num_taxels = {f: taxel_counts.get(f, 0) for f in connected_fingers}
    module_indices = {f: compute_distal_module_index(finger_to_sensor_id[f]) for f in connected_fingers}

    num_active = len(connected_fingers)
    expected_resultant = num_active * BYTES_PER_RESULTANT
    expected_taxels = sum(num_taxels[f] * BYTES_PER_TAXEL for f in connected_fingers)

    return SensorConfiguration(
        connected=connected,
        num_taxels=num_taxels,
        module_indices=module_indices,
        expected_payload_size_resultant=expected_resultant,
        expected_payload_size_taxels=expected_taxels,
        expected_payload_size_combined=expected_resultant + expected_taxels,
        timestamp=time.time(),
        finger_to_sensor_id=finger_to_sensor_id,
    )


# ---------------------------------------------------------------------------
# Mock client — resultant forces
# ---------------------------------------------------------------------------

class TestMockResultantForces:
    def test_values_round_trip(self):
        """Values set via set_mock_forces come back through the stream."""
        mock = MockSensorClient(connected_sensors=ALL_FINGERS)
        mock.connect()
        mock.set_mock_forces({
            "thumb": [1.5, -2.0, 3.0],
            "index": [0.0, 0.0, 0.0],
            "middle": [-1.0, 0.5, 10.0],
        })
        mock.start_auto_stream(resultant=True, taxels=False)
        result, ts = _poll_auto_latest(mock)
        mock.stop_auto_stream()
        mock.disconnect()

        assert ts is not None
        assert result["thumb"] == [1.5, -2.0, 3.0]
        assert result["index"] == [0.0, 0.0, 0.0]
        assert result["middle"] == [-1.0, 0.5, 10.0]
        # Fingers without explicit mock data get default [1.0, 1.0, 1.0]
        assert result["ring"] == [1.0, 1.0, 1.0]

    def test_subset_only_returns_connected(self):
        """Only connected sensors appear in output."""
        subset = ["thumb", "pinky"]
        mock = MockSensorClient(connected_sensors=subset)
        mock.connect()
        mock.set_mock_forces({"thumb": [1.0, 0.0, 0.0], "pinky": [0.0, 1.0, 0.0]})
        mock.start_auto_stream(resultant=True, taxels=False)
        result, _ = _poll_auto_latest(mock)
        mock.stop_auto_stream()
        mock.disconnect()

        assert set(result.keys()) == set(subset)
        assert result["thumb"] == [1.0, 0.0, 0.0]
        assert result["pinky"] == [0.0, 1.0, 0.0]


# ---------------------------------------------------------------------------
# Mock client — taxel data
# ---------------------------------------------------------------------------

class TestMockTaxelData:
    def test_taxel_counts_match_sensor_models(self):
        """Each finger returns the expected number of taxels."""
        mock = MockSensorClient(connected_sensors=ALL_FINGERS)
        mock.connect()
        mock.start_auto_stream(resultant=False, taxels=True)
        result, _ = _poll_auto_latest_taxels(mock)
        mock.stop_auto_stream()
        mock.disconnect()

        assert set(result.keys()) == set(ALL_FINGERS)
        for finger in ALL_FINGERS:
            assert len(result[finger]) == EXPECTED_TAXEL_COUNTS[finger], (
                f"{finger}: expected {EXPECTED_TAXEL_COUNTS[finger]} taxels, "
                f"got {len(result[finger])}"
            )


# ---------------------------------------------------------------------------
# Mock client — combined mode
# ---------------------------------------------------------------------------

class TestMockCombinedMode:
    def test_both_resultant_and_taxels_returned(self):
        mock = MockSensorClient(connected_sensors=ALL_FINGERS)
        mock.connect()
        mock.set_mock_forces({f: [1.0, 0.0, 0.5] for f in ALL_FINGERS})
        mock.start_auto_stream(resultant=True, taxels=True)
        forces, taxels, ts = _poll_auto_latest_all(mock)
        mock.stop_auto_stream()
        mock.disconnect()

        assert set(forces.keys()) == set(ALL_FINGERS)
        assert set(taxels.keys()) == set(ALL_FINGERS)
        for finger in ALL_FINGERS:
            assert forces[finger] == [1.0, 0.0, 0.5]
            assert len(taxels[finger]) == EXPECTED_TAXEL_COUNTS[finger]


# ---------------------------------------------------------------------------
# Mock client — provider injection
# ---------------------------------------------------------------------------

class TestProviderInjection:
    def test_custom_resultant_provider(self):
        call_count = 0
        def counting_provider():
            nonlocal call_count
            call_count += 1
            return {"thumb": [float(call_count), 0.0, 0.0]}

        mock = MockSensorClient(
            connected_sensors=["thumb"],
            resultant_provider=counting_provider,
        )
        mock.connect()
        mock.start_auto_stream(resultant=True, taxels=False)
        result, _ = _poll_auto_latest(mock)
        mock.stop_auto_stream()
        mock.disconnect()

        assert call_count > 0
        assert result["thumb"][0] > 0  # Provider was called at least once

    def test_custom_taxel_provider(self):
        marker = [[99.0, 88.0, 77.0]]
        mock = MockSensorClient(
            connected_sensors=["thumb"],
            taxel_provider=lambda: {"thumb": marker},
        )
        mock.connect()
        mock.start_auto_stream(resultant=False, taxels=True)
        result, _ = _poll_auto_latest_taxels(mock)
        mock.stop_auto_stream()
        mock.disconnect()

        assert result["thumb"] == marker


# ---------------------------------------------------------------------------
# Mock client — dynamic reconfiguration
# ---------------------------------------------------------------------------

class TestDynamicReconfiguration:
    def test_simulate_dropout_removes_sensor(self):
        mock = MockSensorClient(connected_sensors=ALL_FINGERS)
        mock.connect()
        assert mock._sensor_config.num_active_sensors == 5

        mock.simulate_dropout(["index", "ring"])
        assert mock._sensor_config.num_active_sensors == 3
        assert "index" not in mock._sensor_config.active_sensors
        assert "ring" not in mock._sensor_config.active_sensors

    def test_set_connected_sensors_updates_config(self):
        mock = MockSensorClient(connected_sensors=ALL_FINGERS)
        mock.connect()

        mock.set_connected_sensors(["thumb"])
        assert mock._sensor_config.active_sensors == ["thumb"]
        assert mock._sensor_config.num_active_sensors == 1

    def test_dropout_clears_mock_data(self):
        mock = MockSensorClient(connected_sensors=ALL_FINGERS)
        mock.connect()
        mock.set_mock_forces({"index": [5.0, 0.0, 0.0]})

        mock.simulate_dropout(["index"])
        # index mock data should be cleared
        assert "index" not in mock._mock_forces


# ---------------------------------------------------------------------------
# Offset logic
# ---------------------------------------------------------------------------

class TestOffsets:
    def test_resultant_offsets_applied(self):
        mock = MockSensorClient(connected_sensors=["thumb"])
        mock.connect()
        mock.set_mock_forces({"thumb": [5.0, 3.0, 10.0]})
        mock.set_taxel_offsets({"thumb": [[1.0, 0.5, 2.0]]})

        # Resultant offset = sum of taxel offsets
        result = mock.read_resultant_force()
        assert result["thumb"][0] == pytest.approx(4.0, abs=0.1)  # 5.0 - 1.0
        assert result["thumb"][1] == pytest.approx(2.5, abs=0.1)  # 3.0 - 0.5
        assert result["thumb"][2] == pytest.approx(8.0, abs=0.1)  # 10.0 - 2.0

    def test_fz_clamped_to_zero(self):
        """fz should never go negative after offset subtraction."""
        mock = MockSensorClient(connected_sensors=["thumb"])
        mock.connect()
        mock.set_mock_forces({"thumb": [0.0, 0.0, 1.0]})
        mock.set_taxel_offsets({"thumb": [[0.0, 0.0, 5.0]]})

        result = mock.read_resultant_force()
        assert result["thumb"][2] == 0.0  # clamped, not -4.0

    def test_clear_offsets(self):
        mock = MockSensorClient(connected_sensors=["thumb"])
        mock.connect()
        mock.set_mock_forces({"thumb": [5.0, 3.0, 10.0]})
        mock.set_taxel_offsets({"thumb": [[1.0, 0.5, 2.0]]})
        mock.clear_taxel_offsets()

        result = mock.read_resultant_force()
        assert result["thumb"] == [5.0, 3.0, 10.0]  # No offset applied

    def test_stream_offsets_applied(self):
        """Offsets should also apply to auto-stream data."""
        mock = MockSensorClient(connected_sensors=["thumb"])
        mock.connect()
        mock.set_mock_forces({"thumb": [5.0, 3.0, 10.0]})
        mock.set_taxel_offsets({"thumb": [[1.0, 0.5, 2.0]]})
        mock.start_auto_stream(resultant=True, taxels=False)
        result, _ = _poll_auto_latest(mock)
        mock.stop_auto_stream()
        mock.disconnect()

        assert result["thumb"][0] == pytest.approx(4.0, abs=0.1)
        assert result["thumb"][2] == pytest.approx(8.0, abs=0.1)

    def test_taxel_offsets_applied_in_stream(self):
        """Per-taxel offsets should apply to taxel auto-stream data."""
        taxels = [[2.0, 1.0, 5.0], [3.0, 2.0, 8.0]]
        offsets = [[0.5, 0.5, 1.0], [1.0, 1.0, 2.0]]
        mock = MockSensorClient(
            connected_sensors=["thumb"],
            taxel_provider=lambda: {"thumb": [list(t) for t in taxels]},
        )
        mock.connect()
        mock._sim_taxel_counts["thumb"] = 2
        mock._sensor_config = mock._get_configuration()
        mock.set_taxel_offsets({"thumb": offsets})
        mock.start_auto_stream(resultant=False, taxels=True)
        result, _ = _poll_auto_latest_taxels(mock)
        mock.stop_auto_stream()
        mock.disconnect()

        assert result["thumb"][0][0] == pytest.approx(1.5, abs=0.1)  # 2.0 - 0.5
        assert result["thumb"][0][2] == pytest.approx(4.0, abs=0.1)  # 5.0 - 1.0
        assert result["thumb"][1][0] == pytest.approx(2.0, abs=0.1)  # 3.0 - 1.0
        assert result["thumb"][1][2] == pytest.approx(6.0, abs=0.1)  # 8.0 - 2.0


# ---------------------------------------------------------------------------
# Error paths
# ---------------------------------------------------------------------------

class TestErrorPaths:
    def test_read_before_connect_raises(self):
        mock = MockSensorClient(connected_sensors=ALL_FINGERS)
        with pytest.raises(OSError, match="connect"):
            mock.read_resultant_force()

    def test_get_auto_latest_before_stream_returns_none(self):
        mock = MockSensorClient(connected_sensors=ALL_FINGERS)
        mock.connect()
        result, ts = mock.get_auto_latest()
        mock.disconnect()

        assert result is None
        assert ts is None


# ---------------------------------------------------------------------------
# SensorConfiguration ordering
# ---------------------------------------------------------------------------

class TestSensorConfigOrdering:
    def test_slot_order_default(self):
        config = _make_config(ALL_FINGERS)
        assert config.active_sensors == ["thumb", "index", "middle", "ring", "pinky"]

    def test_slot_order_custom_mapping(self):
        custom_map = {"thumb": 1, "index": 3, "middle": 0, "ring": 2, "pinky": 4}
        config = _make_config(ALL_FINGERS, finger_to_sensor_id=custom_map)
        # Sorted by sensor_id: middle(0), thumb(1), ring(2), index(3), pinky(4)
        assert config.active_sensors == ["middle", "thumb", "ring", "index", "pinky"]

    def test_subset_preserves_order(self):
        config = _make_config(["pinky", "thumb"])
        # thumb=0, pinky=4 → thumb first
        assert config.active_sensors == ["thumb", "pinky"]
