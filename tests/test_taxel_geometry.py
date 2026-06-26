"""Tests for static taxel geometry loading and its wiring into the tactile config."""

import numpy as np
import pytest

from orca_core.hardware.mock_tactile_client import MockTactileClient
from orca_core.hardware.sensing.constants import DEFAULT_TAXEL_COUNTS, FINGER_NAMES
from orca_core.hardware.sensing.taxel_geometry import (
    FINGERTIP_LOCAL_FRAME,
    load_all_taxel_geometry,
    load_taxel_geometry,
)


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

    def test_frame_is_fingertip_local(self):
        for geom in load_all_taxel_geometry().values():
            assert geom.frame == FINGERTIP_LOCAL_FRAME

    def test_cache_returns_equal_data(self):
        assert np.array_equal(
            load_taxel_geometry("index").positions,
            load_taxel_geometry("index").positions,
        )


class TestConfigWiring:
    def test_geometry_present_for_connected_fingers(self):
        client = MockTactileClient()
        client.connect()
        geometry = client.get_taxel_geometry()
        assert set(geometry) == set(FINGER_NAMES)
        for finger in FINGER_NAMES:
            assert geometry[finger].num_taxels == DEFAULT_TAXEL_COUNTS[finger]

    def test_only_connected_fingers_have_geometry(self):
        client = MockTactileClient(connected_sensors=["index", "thumb"])
        client.connect()
        assert set(client.get_taxel_geometry()) == {"index", "thumb"}

    def test_geometry_count_matches_taxel_forces(self):
        client = MockTactileClient(connected_sensors=["index"])
        client.connect()
        client.start_auto_stream(resultant=False, taxels=True)
        try:
            client.wait_for_first_frame(timeout=2.0)
            taxels, _ = client.get_auto_latest_taxels()
        finally:
            client.stop_auto_stream()
        positions = client.get_taxel_geometry()["index"].positions
        assert len(positions) == len(taxels["index"])

    def test_empty_before_connect(self):
        assert MockTactileClient().get_taxel_geometry() == {}
