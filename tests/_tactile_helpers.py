"""Compatibility shim: the tactile mock helpers live in the package now."""

from orca_core.hardware.sensing.tactile_mock import (  # noqa: F401
    TactileMockState,
    feed_combined_frame,
    feed_resultant_frame,
    feed_taxels_frame,
    install_tactile_mock,
)
