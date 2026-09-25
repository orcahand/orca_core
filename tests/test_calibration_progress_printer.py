"""The terminal printer must render every event the calibration routine emits;
a GUI sees them all through the same callback, so the terminal must too."""
import inspect
import re

import pytest

from orca_core.maintenance import calibration_routine
from orca_core.utils.cli import print_calibration_progress

EMITTED = sorted(set(re.findall(
    r'_emit\(\s*progress_callback,\s*"(\w+)"', inspect.getsource(calibration_routine)
)))

SAMPLE = dict(
    joint="index_mcp", motor=3, direction="flex", bound="upper", limit=1.0, ratio=0.02,
    joints={"index_mcp": "flex"}, steps=2, index=0, total=2, error="boom",
    anchor_count=1234, anchor_angle_deg=60.0, moved_deg=1.2, flags=["overload"],
    temperature_c=52.0, travel_deg=30.0, expected_deg=120.0, floor_deg=12.0,
    deviation=-0.75, bounds_deg=(90.0, 150.0), within_margin=False, current=450.0,
    attempt=1, attempts=1, reason="degenerate", boosted_joints={}, motor_travel_deg={},
    rom=[-10.0, 90.0], span_deg=100.0, deviation_deg=-2.5, flex_count=100,
    extend_count=5000, stage=1, stages=2,
)

# Progress markers with no information of their own; a GUI uses them to
# advance a bar, the terminal already printed the step header.
SILENT = {"step_done"}


def test_the_routine_emits_something():
    assert len(EMITTED) > 20


@pytest.mark.parametrize("name", [n for n in EMITTED if n not in SILENT])
def test_every_emitted_event_is_printed(name, capsys):
    print_calibration_progress({"event": name, **SAMPLE})
    out = capsys.readouterr().out
    assert out.strip(), f"print_calibration_progress prints nothing for {name!r}"
