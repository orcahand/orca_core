"""A scan that never got its port must not read as an empty bus."""

import logging
from types import SimpleNamespace

from orca_core.hardware import dynamixel_client as dc


class _RefusingPortHandler:
    """Refuses to open ``refusals`` times, then opens."""

    refusals = 0
    attempts = 0

    def __init__(self, port):
        self.port = port

    def openPort(self):
        type(self).attempts += 1
        if type(self).attempts <= type(self).refusals:
            raise OSError("Resource busy")
        return True

    def setBaudRate(self, baud):
        return True

    def closePort(self):
        pass


class _PacketHandler:
    def __init__(self, protocol):
        pass

    def ping(self, port_handler, motor_id):
        return (1200, 0, 0) if motor_id == 17 else (0, -1, 0)


def _client(monkeypatch, refusals):
    _RefusingPortHandler.refusals = refusals
    _RefusingPortHandler.attempts = 0
    client = object.__new__(dc.DynamixelClient)
    client.dxl = SimpleNamespace(PortHandler=_RefusingPortHandler, PacketHandler=_PacketHandler,
                                 COMM_SUCCESS=0)
    monkeypatch.setattr(dc.time, "sleep", lambda _s: None)
    return client


def test_a_port_the_os_still_holds_is_retried(monkeypatch):
    client = _client(monkeypatch, refusals=1)
    motors = client.scan_for_motors("/dev/cu.usbmodemXXXX", (1, 17), [1_000_000])
    assert [m["id"] for m in motors] == [17]
    assert _RefusingPortHandler.attempts == 2


def test_a_port_that_stays_shut_is_logged_not_silent(monkeypatch, caplog):
    client = _client(monkeypatch, refusals=99)
    with caplog.at_level(logging.WARNING):
        motors = client.scan_for_motors("/dev/cu.usbmodemXXXX", (1, 17), [1_000_000])
    assert motors == []
    assert _RefusingPortHandler.attempts == dc.SCAN_PORT_OPEN_ATTEMPTS
    assert "could not open" in caplog.text
