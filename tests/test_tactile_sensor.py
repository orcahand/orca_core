"""Tests for tactile sensor data contracts and parsing.

Validates byte formats, data shapes, payload sizes, and protocol details
without requiring hardware. Uses MockSensorClient and direct parser calls.
"""

import struct
import time

import pytest

from orca_core.hardware.sensing.sensor_client import (
    SensorClient,
    SensorConfiguration,
    calculate_checksum,
    FINGER_NAMES,
)
from orca_core.hardware.sensing.mock_sensor_client import MockSensorClient
from orca_core.hardware.sensing.taxel_coordinates import get_all_coordinates

EXPECTED_TAXEL_COUNTS = {
    "thumb": 51, "index": 87, "middle": 87, "ring": 87, "pinky": 51,
}

ALL_FINGERS = ["thumb", "index", "middle", "ring", "pinky"]


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
    module_indices = {f: finger_to_sensor_id[f] * 4 + 2 for f in connected_fingers}

    num_active = len(connected_fingers)
    expected_resultant = num_active * 6
    expected_taxels = sum(num_taxels[f] * 3 for f in connected_fingers)

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


def _sensor_client_instance() -> SensorClient:
    """Create a SensorClient without connecting (for calling parse methods)."""
    client = SensorClient.__new__(SensorClient)
    return client


# ---------------------------------------------------------------------------
# Test 1: Mock client — resultant forces shape
# ---------------------------------------------------------------------------

class TestMockResultantForces:
    def test_shape_all_fingers(self):
        mock = MockSensorClient(connected_sensors=ALL_FINGERS)
        mock.connect()
        forces = {f: [1.0, -0.5, 2.0] for f in ALL_FINGERS}
        mock.set_mock_forces(forces)
        mock.start_auto_stream(resultant=True, taxels=False)
        time.sleep(0.05)

        result, ts = mock.get_auto_latest()
        mock.stop_auto_stream()
        mock.disconnect()

        assert result is not None
        assert ts is not None
        assert set(result.keys()) == set(ALL_FINGERS)
        for finger in ALL_FINGERS:
            assert len(result[finger]) == 3
            assert all(isinstance(v, float) for v in result[finger])

    def test_values_match_no_noise(self):
        mock = MockSensorClient(connected_sensors=ALL_FINGERS)
        mock.set_noise_level(0.0)
        mock.connect()
        mock.set_mock_forces({"thumb": [1.5, -2.0, 3.0], "index": [0.0, 0.0, 0.0]})
        mock.start_auto_stream(resultant=True, taxels=False)
        time.sleep(0.05)

        result, _ = mock.get_auto_latest()
        mock.stop_auto_stream()
        mock.disconnect()

        assert result["thumb"] == [1.5, -2.0, 3.0]
        assert result["index"] == [0.0, 0.0, 0.0]

    def test_subset_of_fingers(self):
        subset = ["thumb", "pinky"]
        mock = MockSensorClient(connected_sensors=subset)
        mock.connect()
        mock.set_mock_forces({"thumb": [1.0, 0.0, 0.0], "pinky": [0.0, 1.0, 0.0]})
        mock.start_auto_stream(resultant=True, taxels=False)
        time.sleep(0.05)

        result, _ = mock.get_auto_latest()
        mock.stop_auto_stream()
        mock.disconnect()

        assert set(result.keys()) == set(subset)


# ---------------------------------------------------------------------------
# Test 2: Mock client — taxel data shape
# ---------------------------------------------------------------------------

class TestMockTaxelData:
    def test_shape_all_fingers(self):
        mock = MockSensorClient(connected_sensors=ALL_FINGERS)
        mock.connect()
        mock.start_auto_stream(resultant=False, taxels=True)
        time.sleep(0.05)

        result, ts = mock.get_auto_latest_taxels()
        mock.stop_auto_stream()
        mock.disconnect()

        assert result is not None
        assert set(result.keys()) == set(ALL_FINGERS)
        for finger in ALL_FINGERS:
            assert len(result[finger]) == EXPECTED_TAXEL_COUNTS[finger], (
                f"{finger}: expected {EXPECTED_TAXEL_COUNTS[finger]} taxels, "
                f"got {len(result[finger])}"
            )
            for taxel in result[finger]:
                assert len(taxel) == 3
                assert all(isinstance(v, float) for v in taxel)


# ---------------------------------------------------------------------------
# Test 3: Mock client — combined mode shape
# ---------------------------------------------------------------------------

class TestMockCombinedMode:
    def test_shape(self):
        mock = MockSensorClient(connected_sensors=ALL_FINGERS)
        mock.connect()
        mock.set_mock_forces({f: [1.0, 0.0, 0.5] for f in ALL_FINGERS})
        mock.start_auto_stream(resultant=True, taxels=True)
        time.sleep(0.05)

        forces, taxels, ts = mock.get_auto_latest_all()
        mock.stop_auto_stream()
        mock.disconnect()

        assert forces is not None
        assert taxels is not None
        assert set(forces.keys()) == set(ALL_FINGERS)
        assert set(taxels.keys()) == set(ALL_FINGERS)
        for finger in ALL_FINGERS:
            assert len(forces[finger]) == 3
            assert len(taxels[finger]) == EXPECTED_TAXEL_COUNTS[finger]


# ---------------------------------------------------------------------------
# Test 4: Payload size calculation
# ---------------------------------------------------------------------------

class TestPayloadSize:
    def test_all_sensors(self):
        config = _make_config(ALL_FINGERS)
        total_taxels = 51 + 87 + 87 + 87 + 51  # 363
        assert config.expected_payload_size_resultant == 5 * 6  # 30
        assert config.expected_payload_size_taxels == total_taxels * 3  # 1089
        assert config.expected_payload_size_combined == 30 + total_taxels * 3  # 1119

    def test_two_sensors(self):
        config = _make_config(["thumb", "index"])
        assert config.expected_payload_size_resultant == 2 * 6  # 12
        assert config.expected_payload_size_taxels == (51 + 87) * 3  # 414
        assert config.expected_payload_size_combined == 12 + 414  # 426

    def test_single_sensor(self):
        config = _make_config(["pinky"])
        assert config.expected_payload_size_resultant == 6
        assert config.expected_payload_size_taxels == 51 * 3  # 153
        assert config.expected_payload_size_combined == 6 + 153  # 159

    def test_no_sensors(self):
        config = _make_config([])
        assert config.expected_payload_size_resultant == 0
        assert config.expected_payload_size_taxels == 0
        assert config.expected_payload_size_combined == 0


# ---------------------------------------------------------------------------
# Test 5: Parse resultant compact — known bytes
# ---------------------------------------------------------------------------

class TestParseResultantCompact:
    def test_known_values(self):
        client = _sensor_client_instance()
        config = _make_config(["thumb"])

        # fx=100 (10.0N), fy=-50 (-5.0N), fz=200 (20.0N)
        data = struct.pack("<hhH", 100, -50, 200)
        result = client._parse_auto_stream_compact(data, config)

        assert result["thumb"] == [10.0, -5.0, 20.0]

    def test_two_sensors(self):
        client = _sensor_client_instance()
        config = _make_config(["thumb", "index"])

        data = struct.pack("<hhH", 10, 20, 30) + struct.pack("<hhH", -10, -20, 40)
        result = client._parse_auto_stream_compact(data, config)

        assert result["thumb"] == [1.0, 2.0, 3.0]
        assert result["index"] == [-1.0, -2.0, 4.0]

    def test_zero_forces(self):
        client = _sensor_client_instance()
        config = _make_config(["thumb"])

        data = struct.pack("<hhH", 0, 0, 0)
        result = client._parse_auto_stream_compact(data, config)

        assert result["thumb"] == [0.0, 0.0, 0.0]

    def test_wrong_size_raises(self):
        client = _sensor_client_instance()
        config = _make_config(["thumb"])

        with pytest.raises(ValueError, match="size mismatch"):
            client._parse_auto_stream_compact(b"\x00" * 5, config)

        with pytest.raises(ValueError, match="size mismatch"):
            client._parse_auto_stream_compact(b"\x00" * 7, config)


# ---------------------------------------------------------------------------
# Test 6: Parse taxels compact — known bytes
# ---------------------------------------------------------------------------

class TestParseTaxelsCompact:
    def test_known_values(self):
        client = _sensor_client_instance()
        # Use small taxel count for test
        config = _make_config(["thumb"], taxel_counts={"thumb": 2})

        # taxel 0: fx=10 (1.0N), fy=-5 (-0.5N), fz=20 (2.0N)
        # taxel 1: fx=0, fy=0, fz=50 (5.0N)
        data = struct.pack("bbB", 10, -5, 20) + struct.pack("bbB", 0, 0, 50)
        result = client._parse_taxels_compact(data, config)

        assert len(result["thumb"]) == 2
        assert result["thumb"][0] == [1.0, -0.5, 2.0]
        assert result["thumb"][1] == [0.0, 0.0, 5.0]

    def test_correct_taxel_count(self):
        client = _sensor_client_instance()
        config = _make_config(["thumb"], taxel_counts={"thumb": 51})

        data = b"\x00" * (51 * 3)
        result = client._parse_taxels_compact(data, config)

        assert len(result["thumb"]) == 51

    def test_wrong_size_raises(self):
        client = _sensor_client_instance()
        config = _make_config(["thumb"], taxel_counts={"thumb": 2})

        with pytest.raises(ValueError, match="size mismatch"):
            client._parse_taxels_compact(b"\x00" * 5, config)


# ---------------------------------------------------------------------------
# Test 7: Parse combined compact — known bytes
# ---------------------------------------------------------------------------

class TestParseCombinedCompact:
    def test_interleaved_format(self):
        client = _sensor_client_instance()
        config = _make_config(["thumb", "index"], taxel_counts={"thumb": 1, "index": 1})

        # thumb: resultant(6) + taxels(3) + index: resultant(6) + taxels(3)
        data = (
            struct.pack("<hhH", 100, 0, 50)  # thumb resultant
            + struct.pack("bbB", 10, 0, 5)    # thumb taxel
            + struct.pack("<hhH", 0, -100, 200)  # index resultant
            + struct.pack("bbB", 0, -10, 20)  # index taxel
        )
        forces, taxels = client._parse_combined_compact(data, config)

        assert forces["thumb"] == [10.0, 0.0, 5.0]
        assert forces["index"] == [0.0, -10.0, 20.0]
        assert len(taxels["thumb"]) == 1
        assert len(taxels["index"]) == 1
        assert taxels["thumb"][0] == [1.0, 0.0, 0.5]
        assert taxels["index"][0] == [0.0, -1.0, 2.0]

    def test_wrong_size_raises(self):
        client = _sensor_client_instance()
        config = _make_config(["thumb"], taxel_counts={"thumb": 1})

        with pytest.raises(ValueError, match="size mismatch"):
            client._parse_combined_compact(b"\x00" * 5, config)


# ---------------------------------------------------------------------------
# Test 8: Taxel coordinate counts match sensor models
# ---------------------------------------------------------------------------

class TestTaxelCoordinates:
    def test_counts_match_models(self):
        coords = get_all_coordinates()
        for finger, expected_count in EXPECTED_TAXEL_COUNTS.items():
            assert len(coords[finger]) == expected_count, (
                f"{finger}: expected {expected_count} coordinates, got {len(coords[finger])}"
            )

    def test_coordinate_structure(self):
        coords = get_all_coordinates()
        for finger, taxels in coords.items():
            for i, coord in enumerate(taxels):
                assert "x" in coord and "y" in coord and "z" in coord, (
                    f"{finger} taxel {i}: missing x/y/z keys"
                )
                assert all(isinstance(coord[k], float) for k in ("x", "y", "z"))


# ---------------------------------------------------------------------------
# Test 9: LRC checksum
# ---------------------------------------------------------------------------

class TestChecksum:
    def test_known_value(self):
        assert calculate_checksum(b"\x01\x02\x03") == 0xFA

    def test_round_trip(self):
        frame = b"\xAA\x55\x00\x03\x10\x00\x04\x00"
        checksum = calculate_checksum(frame)
        assert (sum(frame) + checksum) & 0xFF == 0

    def test_empty_frame(self):
        assert calculate_checksum(b"") == 0

    def test_single_byte(self):
        assert calculate_checksum(b"\x01") == 0xFF


# ---------------------------------------------------------------------------
# Test 10: SensorConfiguration active_sensors ordering
# ---------------------------------------------------------------------------

class TestSensorConfigOrdering:
    def test_slot_order_default(self):
        config = _make_config(ALL_FINGERS)
        # Default mapping: thumb=0, index=1, middle=2, ring=3, pinky=4
        assert config.active_sensors == ["thumb", "index", "middle", "ring", "pinky"]

    def test_slot_order_custom_mapping(self):
        custom_map = {"thumb": 1, "index": 3, "middle": 0, "ring": 2, "pinky": 4}
        config = _make_config(ALL_FINGERS, finger_to_sensor_id=custom_map)
        # Sorted by sensor_id: middle(0), thumb(1), ring(2), index(3), pinky(4)
        assert config.active_sensors == ["middle", "thumb", "ring", "index", "pinky"]

    def test_num_active_sensors(self):
        config = _make_config(["thumb", "middle", "pinky"])
        assert config.num_active_sensors == 3

    def test_subset_preserves_order(self):
        config = _make_config(["pinky", "thumb"])
        # thumb=0, pinky=4 → thumb first
        assert config.active_sensors == ["thumb", "pinky"]
