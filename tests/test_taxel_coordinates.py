"""Tests for taxel coordinate data integrity."""

from orca_core.hardware.sensing.constants import DEFAULT_TAXEL_COUNTS
from orca_core.hardware.sensing.taxel_coordinates import get_all_coordinates


class TestTaxelCoordinates:
    def test_counts_match_models(self):
        coords = get_all_coordinates()
        for finger, expected_count in DEFAULT_TAXEL_COUNTS.items():
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
