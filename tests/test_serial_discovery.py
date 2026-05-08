from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import patch

import pytest

from orca_core.constants import ORCA_ID_RESP_MOTOR, ORCA_ID_RESP_SENSOR
from orca_core.hardware.sensing.serial_discovery import (
    SensingPorts,
    discover_sensing_ports,
    find_tactile_port,
    resolve_sensing_ports,
)


PAXINI_VID, OH_VID = 0x28E9, 0x2F5D
PAXINI_PORT = "/dev/cu.paxini"
OH_MOTOR_PORT = "/dev/cu.oh_motor"
OH_SENSOR_PORT = "/dev/cu.oh_sensor"
DISCOVERY = "orca_core.hardware.sensing.serial_discovery"

paxini = SimpleNamespace(device=PAXINI_PORT, vid=PAXINI_VID)
motor = SimpleNamespace(device=OH_MOTOR_PORT, vid=OH_VID)
sensor = SimpleNamespace(device=OH_SENSOR_PORT, vid=OH_VID)
OH_RESPONSES = {OH_MOTOR_PORT: ORCA_ID_RESP_MOTOR, OH_SENSOR_PORT: ORCA_ID_RESP_SENSOR}
DISCOVERED = SensingPorts("/auto/t", "/auto/e")


@pytest.mark.parametrize("tactile,encoder,expected", [
    (PAXINI_PORT, PAXINI_PORT,    True),
    (PAXINI_PORT, OH_SENSOR_PORT, False),
    (PAXINI_PORT, None,           False),
    (None,        PAXINI_PORT,    False),
    (None,        None,           False),
])
def test_sensing_ports_shared(tactile, encoder, expected):
    assert SensingPorts(tactile, encoder).shared is expected


@pytest.mark.parametrize("ports,expected", [
    pytest.param([paxini],                                                       PAXINI_PORT, id="single"),
    pytest.param([motor],                                                        None,        id="absent"),
    pytest.param([paxini, SimpleNamespace(device="/dev/cu.p2", vid=PAXINI_VID)], None,        id="ambiguous"),
    pytest.param([],                                                             None,        id="empty"),
])
def test_find_tactile_port(ports, expected):
    with patch("serial.tools.list_ports.comports", return_value=ports):
        assert find_tactile_port() == expected


@pytest.mark.parametrize("ports,probes,expected", [
    pytest.param([],                      {},                                  SensingPorts(None, None),                     id="nothing"),
    pytest.param([paxini],                {},                                  SensingPorts(PAXINI_PORT, None),              id="paxini_only"),
    pytest.param([motor, sensor],         OH_RESPONSES,                        SensingPorts(OH_SENSOR_PORT, OH_SENSOR_PORT), id="oh_only_shared"),
    pytest.param([sensor, motor],         OH_RESPONSES,                        SensingPorts(OH_SENSOR_PORT, OH_SENSOR_PORT), id="oh_reversed_order"),
    pytest.param([motor, sensor, paxini], OH_RESPONSES,                        SensingPorts(PAXINI_PORT, OH_SENSOR_PORT),    id="paxini_plus_oh"),
    pytest.param([motor],                 {OH_MOTOR_PORT: ORCA_ID_RESP_MOTOR}, SensingPorts(None, None),                     id="motor_only"),
    pytest.param([motor, sensor],         {},                                  SensingPorts(None, None),                     id="oh_silent"),
    pytest.param([motor, paxini],         {OH_MOTOR_PORT: ORCA_ID_RESP_MOTOR}, SensingPorts(PAXINI_PORT, None),              id="paxini_with_motor"),
])
def test_discover_sensing_ports(ports, probes, expected):
    with patch("serial.tools.list_ports.comports", return_value=ports), \
            patch(f"{DISCOVERY}._probe_orca_id", side_effect=lambda p, *_a, **_kw: probes.get(p)):
        assert discover_sensing_ports() == expected


def test_discover_does_not_probe_paxini():
    """ORCA_ID? on a Paxini board would hit the wrong protocol."""
    with patch("serial.tools.list_ports.comports", return_value=[paxini]), \
            patch(f"{DISCOVERY}._probe_orca_id") as probe:
        discover_sensing_ports()
    probe.assert_not_called()


@pytest.mark.parametrize("tactile,encoder,expected,expect_discovery", [
    pytest.param("auto",     "auto",      DISCOVERED,                            True,  id="auto"),
    pytest.param("auto",     "/dev/y",    SensingPorts("/auto/t", "/dev/y"),     True,  id="auto+explicit"),
    pytest.param("auto",     "disabled",  SensingPorts("/auto/t", None),         True,  id="auto+disabled"),
    pytest.param("/dev/x",   "/dev/y",    SensingPorts("/dev/x", "/dev/y"),      False, id="explicit"),
    pytest.param("/dev/s",   "/dev/s",    SensingPorts("/dev/s", "/dev/s"),      False, id="same_explicit"),
    pytest.param("disabled", "/dev/y",    SensingPorts(None, "/dev/y"),          False, id="disabled+explicit"),
    pytest.param("disabled", "disabled",  SensingPorts(None, None),              False, id="all_disabled"),
])
def test_resolve_sensing_ports(tactile, encoder, expected, expect_discovery):
    with patch(f"{DISCOVERY}.discover_sensing_ports", return_value=DISCOVERED) as discover:
        assert resolve_sensing_ports(tactile, encoder) == expected
    assert discover.called is expect_discovery
