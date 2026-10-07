"""Each maintenance routine reports a failing progress callback under its own logger and message."""

import logging

import pytest

from orca_core.maintenance import calibration_routine, motor_chain, tensioning


@pytest.mark.parametrize(
    "module, message",
    [
        (calibration_routine, "calibration progress callback failed"),
        (tensioning, "tension progress callback failed"),
        (motor_chain, "motor-chain progress callback failed"),
    ],
    ids=["calibration", "tension", "motor-chain"],
)
def test_only_a_failing_progress_callback_is_logged_by_its_own_routine(module, message, caplog):
    def boom(event):
        raise RuntimeError("callback broke")

    with caplog.at_level(logging.DEBUG):
        module._emit(None, "phase", phase="winding")
        module._emit(boom, "phase", phase="winding")

    assert [(r.name, r.getMessage()) for r in caplog.records] == [(module.__name__, message)]
