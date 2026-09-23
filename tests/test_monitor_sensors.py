"""Tests for the stream resolution behind ``scripts/monitor_sensors.py``.

The script decides which sensor streams to open, on which ports, from either a
hand's ``config.yaml`` or an explicit/autodetected port. Everything here
exercises that decision; the Tkinter view itself is not covered.
"""

import importlib.util
import os
import sys
from unittest.mock import patch

import pytest

import orca_core
from orca_core import OrcaHandConfig, OrcaHandTouchConfig
from orca_core.hardware.sensing.constants import (
    DEFAULT_ENCODER_BAUDRATE,
    DEFAULT_SENSOR_BAUDRATE,
)
from orca_core.hardware.sensing.serial_discovery import SensingPorts

pytest.importorskip("tkinter", reason="monitor_sensors is a Tkinter script")

MODELS = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2")
SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts", "monitor_sensors.py",
)


def _load_script():
    """Import ``scripts/monitor_sensors.py``, which is not on the package path."""
    spec = importlib.util.spec_from_file_location("monitor_sensors", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


monitor = _load_script()
# The script imports the discovery helpers by name, so patch its own globals.
DISCOVERY = "monitor_sensors"


def _config(model_name):
    cls = OrcaHandTouchConfig if "touch" in model_name or "full" in model_name else OrcaHandConfig
    return cls.from_config_path(os.path.join(MODELS, model_name, "config.yaml"))


def _resolved(tactile=None, encoder=None, tactile_baud=None):
    return patch(
        f"{DISCOVERY}.resolve_sensing_ports",
        return_value=SensingPorts(tactile, encoder, tactile_baud),
    )


# ----- targets_from_config --------------------------------------------------


def test_touch_config_yields_tactile_only():
    """A config with a sensors block but no encoders monitors tactile alone."""
    with _resolved(tactile="/dev/t", tactile_baud=DEFAULT_SENSOR_BAUDRATE):
        targets = monitor.targets_from_config(_config("orcahand-touch-right"))
    assert targets.tactile == monitor.StreamTarget("/dev/t", DEFAULT_SENSOR_BAUDRATE)
    assert targets.encoder is None
    assert targets.finger_to_sensor_id == {
        "thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4,
    }


def test_joint_config_yields_encoder_only():
    """A config with joint encoders but no sensors block skips tactile."""
    with _resolved(encoder="/dev/e"):
        targets = monitor.targets_from_config(_config("orcahand-joint-right"))
    assert targets.tactile is None
    assert targets.encoder == monitor.StreamTarget("/dev/e", DEFAULT_ENCODER_BAUDRATE)
    assert targets.finger_to_sensor_id is None


def test_full_config_yields_both_streams():
    with _resolved(tactile="/dev/t", encoder="/dev/e",
                   tactile_baud=DEFAULT_SENSOR_BAUDRATE):
        targets = monitor.targets_from_config(_config("orcahand-full-right"))
    assert targets.tactile.port == "/dev/t"
    assert targets.encoder.port == "/dev/e"


def test_config_disables_the_streams_it_does_not_declare():
    """A stream the config is silent about is never probed for."""
    with patch(f"{DISCOVERY}.resolve_sensing_ports",
               return_value=SensingPorts(None, None)) as resolve:
        monitor.targets_from_config(_config("orcahand-touch-right"))
    assert resolve.call_args.kwargs["encoder_override"] == "disabled"


def test_explicit_tactile_port_gets_its_baud_detected():
    """Discovery is skipped for an explicit port, so the baud is probed."""
    with _resolved(tactile="/dev/t"), \
            patch(f"{DISCOVERY}.baud_for_port", return_value=DEFAULT_SENSOR_BAUDRATE) as probe:
        targets = monitor.targets_from_config(_config("orcahand-touch-right"))
    probe.assert_called_once_with("/dev/t")
    assert targets.tactile.baud == DEFAULT_SENSOR_BAUDRATE


def test_baud_override_wins_over_the_config():
    with _resolved(tactile="/dev/s", encoder="/dev/s",
                   tactile_baud=DEFAULT_SENSOR_BAUDRATE):
        targets = monitor.targets_from_config(_config("orcahand-full-right"), baud=115200)
    assert targets.tactile.baud == 115200
    assert targets.encoder.baud == 115200


def test_unresolved_port_drops_its_stream():
    with _resolved(tactile=None, encoder=None):
        targets = monitor.targets_from_config(_config("orcahand-full-right"))
    assert targets.tactile is None and targets.encoder is None


# ----- targets_from_port ----------------------------------------------------


def test_explicit_port_carries_both_streams():
    with patch(f"{DISCOVERY}.discover_sensing_ports") as discover:
        targets = monitor.targets_from_port("/dev/x", DEFAULT_ENCODER_BAUDRATE)
    discover.assert_not_called()
    assert targets.tactile == targets.encoder
    assert targets.tactile == monitor.StreamTarget("/dev/x", DEFAULT_ENCODER_BAUDRATE)


def test_autodetected_shared_port_uses_the_discovered_baud():
    shared = SensingPorts("/dev/s", "/dev/s", DEFAULT_ENCODER_BAUDRATE)
    with patch(f"{DISCOVERY}.discover_sensing_ports", return_value=shared):
        targets = monitor.targets_from_port(None, None)
    assert targets.encoder == monitor.StreamTarget("/dev/s", DEFAULT_ENCODER_BAUDRATE)


def test_nothing_plugged_in_exits():
    with patch(f"{DISCOVERY}.discover_sensing_ports",
               return_value=SensingPorts(None, None)):
        with pytest.raises(SystemExit):
            monitor.targets_from_port(None, None)


# ----- SensingLinks ---------------------------------------------------------


def test_shared_port_opens_one_link():
    """Two links on one device would steal each other's bytes."""
    target = monitor.StreamTarget("/dev/s", DEFAULT_ENCODER_BAUDRATE)
    links = monitor.SensingLinks(monitor.SensingTargets(target, target))
    assert links.tactile is links.encoder


def test_separate_ports_open_separate_links():
    links = monitor.SensingLinks(monitor.SensingTargets(
        monitor.StreamTarget("/dev/t", DEFAULT_SENSOR_BAUDRATE),
        monitor.StreamTarget("/dev/e", DEFAULT_ENCODER_BAUDRATE),
    ))
    assert links.tactile is not links.encoder
    assert links.tactile is not None and links.encoder is not None


def test_missing_stream_has_no_link():
    links = monitor.SensingLinks(monitor.SensingTargets(
        None, monitor.StreamTarget("/dev/e", DEFAULT_ENCODER_BAUDRATE),
    ))
    assert links.tactile is None
    assert links.encoder is not None


def test_shared_port_opens_at_the_encoder_baud(capsys):
    """The encoder stream is the baud-critical one, and the clash is reported."""
    links = monitor.SensingLinks(monitor.SensingTargets(
        monitor.StreamTarget("/dev/s", DEFAULT_SENSOR_BAUDRATE),
        monitor.StreamTarget("/dev/s", DEFAULT_ENCODER_BAUDRATE),
    ))
    assert links.tactile._baudrate == DEFAULT_ENCODER_BAUDRATE
    assert "WARNING" in capsys.readouterr().out


def _raiser(exc):
    """Stand-in for a link method that fails."""

    def fail():
        raise exc

    return fail


def _two_links():
    return monitor.SensingLinks(monitor.SensingTargets(
        monitor.StreamTarget("/dev/t", DEFAULT_SENSOR_BAUDRATE),
        monitor.StreamTarget("/dev/e", DEFAULT_ENCODER_BAUDRATE),
    ))


def test_a_failed_second_link_closes_the_first():
    """A half-open pair would hold a serial port with no UI to show for it."""
    links = _two_links()
    closed = []
    # The encoder link opens first; the tactile one then fails to.
    links.encoder.connect = lambda: None
    links.encoder.disconnect = lambda: closed.append("encoder")
    links.tactile.connect = _raiser(OSError("busy"))
    links.tactile.disconnect = lambda: closed.append("tactile")

    with pytest.raises(OSError):
        links.connect()
    assert "encoder" in closed


def test_one_failing_close_does_not_strand_the_other_link():
    links = _two_links()
    closed = []
    links.encoder.disconnect = _raiser(OSError("gone"))
    links.tactile.disconnect = lambda: closed.append("tactile")

    links.disconnect()
    assert closed == ["tactile"]


# ----- resolve_targets ------------------------------------------------------


def _args(**overrides):
    from argparse import Namespace

    return Namespace(
        **{"config_path": None, "port": None, "baud": None, **overrides}
    )


def test_config_and_port_together_are_rejected():
    with pytest.raises(SystemExit):
        monitor.resolve_targets(_args(config_path="x", port="/dev/x"))


def test_explicit_port_skips_discovery():
    with patch(f"{DISCOVERY}.discover_sensing_ports") as discover:
        targets = monitor.resolve_targets(
            _args(port="/dev/x", baud=DEFAULT_ENCODER_BAUDRATE))
    discover.assert_not_called()
    assert targets.encoder.port == "/dev/x"


def test_config_path_decides_the_ports():
    config_path = os.path.join(MODELS, "orcahand-touch-left", "config.yaml")
    with _resolved(tactile="/dev/t", tactile_baud=DEFAULT_SENSOR_BAUDRATE):
        targets = monitor.resolve_targets(_args(config_path=config_path))
    assert targets.tactile.port == "/dev/t"


def test_sensorless_config_exits():
    config_path = os.path.join(MODELS, "orcahand-right", "config.yaml")
    with pytest.raises(SystemExit, match="declares no sensors"):
        monitor.resolve_targets(_args(config_path=config_path))


def test_config_whose_ports_do_not_resolve_exits():
    config_path = os.path.join(MODELS, "orcahand-touch-left", "config.yaml")
    with _resolved():
        with pytest.raises(SystemExit, match="No sensor port resolved"):
            monitor.resolve_targets(_args(config_path=config_path))
