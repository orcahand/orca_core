"""Tests for static taxel geometry loading and its wiring into the tactile config."""

import numpy as np
import pytest

from orca_core.constants import FINGER_NAMES
from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import DEFAULT_TAXEL_COUNTS
from orca_core.hardware.sensing.taxel_geometry import (
    SENSOR_FRAME,
    load_all_taxel_geometry,
    load_taxel_geometry,
)
from orca_core.hardware.sensing.tactile_mock import (
    TactileMockState,
    feed_taxels_frame,
    install_tactile_mock,
)
from orca_core.hardware.tactile_client import TactileClient


def _make_client(connected_fingers=None):
    state = TactileMockState()
    if connected_fingers is not None:
        state.connected_fingers = list(connected_fingers)
    link = MockHandSerialLink()
    install_tactile_mock(link, state)
    link.connect()
    client = TactileClient(link, finger_to_sensor_id=state.finger_to_sensor_id)
    return client, link, state


class TestLoadGeometry:
    def test_counts_match_taxel_counts(self):
        for finger, expected in DEFAULT_TAXEL_COUNTS.items():
            geom = load_taxel_geometry(finger)
            assert geom.num_taxels == expected, (
                f"{finger}: expected {expected} positions, got {geom.num_taxels}"
            )

    def test_positions_shape_and_dtype(self):
        for geom in load_all_taxel_geometry().values():
            assert geom.positions.shape == (geom.num_taxels, 3)
            assert geom.positions.dtype == np.float64

    def test_frame_is_sensor(self):
        for geom in load_all_taxel_geometry().values():
            assert geom.frame == SENSOR_FRAME

    def test_positions_are_meters(self):
        # Sensor bodies are a few cm long; mm values would exceed this by 1000x.
        for geom in load_all_taxel_geometry().values():
            assert np.all(np.abs(geom.positions) < 0.05)
            assert np.max(np.abs(geom.positions)) > 0.005

    def test_known_first_taxel_converted(self):
        positions = load_taxel_geometry("index").positions
        np.testing.assert_allclose(
            positions[0], [4.03569563e-3, 28.08244307e-3, 3.72577291e-3]
        )

    def test_cache_returns_the_same_array_object(self):
        assert (
            load_taxel_geometry("index").positions
            is load_taxel_geometry("index").positions
        )

    def test_cached_positions_are_read_only(self):
        """Callers share the cached array, so it must not be writable."""
        with pytest.raises(ValueError):
            load_taxel_geometry("index").positions[0, 0] = 1.0


class TestConfigWiring:
    def test_geometry_present_for_connected_fingers(self):
        client, link, _ = _make_client()
        client.connect()
        geometry = client.get_taxel_geometry()
        assert set(geometry) == set(FINGER_NAMES)
        for finger in FINGER_NAMES:
            assert geometry[finger].num_taxels == DEFAULT_TAXEL_COUNTS[finger]
        link.disconnect()

    def test_only_connected_fingers_have_geometry(self):
        client, link, _ = _make_client(connected_fingers=["index", "thumb"])
        client.connect()
        assert set(client.get_taxel_geometry()) == {"index", "thumb"}
        link.disconnect()

    def test_geometry_count_matches_taxel_forces(self):
        client, link, state = _make_client(connected_fingers=["index"])
        client.connect()
        client.start_stream(resultant=False, taxels=True)
        taxels = {"index": [[0.1, 0.2, 0.3]] * DEFAULT_TAXEL_COUNTS["index"]}
        feed_taxels_frame(link, taxels, state.active_sensors)
        client.wait_for_first_frame()
        reading = client.get_latest_taxels()
        positions = client.get_taxel_geometry()["index"].positions
        assert len(positions) == len(reading["index"])
        link.disconnect()

    def test_empty_before_connect(self):
        client, link, _ = _make_client()
        assert client.get_taxel_geometry() == {}
        link.disconnect()
