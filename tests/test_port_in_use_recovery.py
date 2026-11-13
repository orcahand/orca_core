"""A serial exception mid-transaction must not leave the port refusing every later one.

Both vendor SDKs set ``port_handler.is_using`` when a packet is sent and clear
it only once the reply has been read. These tests drive the real packet
handlers against a fake port whose read raises once, then check that torque
can still be switched off and the port still closes.
"""

import pytest
import serial

from orca_core.hardware.dynamixel_client import DynamixelClient
from orca_core.hardware.feetech_client import FeetechClient, HLSPacketHandler

IDS = [1, 2]
TORQUE_ADDRESS = 64  # any register: the read only has to start a transaction


class FakeSerial:
    def reset_input_buffer(self):
        pass


class FakePort:
    """Duck-typed SDK port handler: records sent packets, never answers."""

    def __init__(self):
        self.is_open = True
        self.is_using = False
        self.ser = FakeSerial()
        self.baudrate = 1000000
        self.port_name = "/dev/cu.usbmodemXXXX"
        self.sent = []
        self.raise_next_read = False
        self.raise_next_write = False

    def clearPort(self):
        pass

    def writePort(self, packet):
        if self.raise_next_write:
            self.raise_next_write = False
            raise serial.SerialException("write failed: device disconnected")
        self.sent.append(bytes(packet))
        return len(packet)

    def readPort(self, length):
        if self.raise_next_read:
            self.raise_next_read = False
            raise serial.SerialException("device reports readiness to read but returned no data")
        return b""

    def setPacketTimeout(self, packet_length):
        pass

    def setPacketTimeoutMillis(self, msec):
        pass

    def isPacketTimeout(self):
        return True

    def getBaudRate(self):
        return self.baudrate

    def closePort(self):
        self.is_open = False


def _dynamixel():
    client = DynamixelClient(IDS, port="/dev/cu.usbmodemXXXX")
    client.port_handler = FakePort()

    def raw_read():
        return client.packet_handler.read1ByteTxRx(client.port_handler, IDS[0], TORQUE_ADDRESS)
    return client, raw_read


def _feetech():
    client = FeetechClient(IDS, port="/dev/cu.usbmodemXXXX")
    client.port_handler = FakePort()
    client.packet_handler = HLSPacketHandler(client.port_handler)
    client._connected = True

    def raw_read():
        return client.packet_handler.read1ByteTxRx(IDS[0], TORQUE_ADDRESS)
    return client, raw_read


@pytest.fixture(params=[_dynamixel, _feetech], ids=["dynamixel", "feetech"])
def stuck(request):
    """A client whose last transaction raised mid-reply, leaving the flag set."""
    client, raw_read = request.param()
    port = client.port_handler
    port.raise_next_read = True
    with client._bus_lock, pytest.raises(serial.SerialException):
        raw_read()
    assert port.is_using, "the SDK should leave its in-use flag set"
    client.OPEN_CLIENTS.add(client)
    yield client, port
    client.OPEN_CLIENTS.discard(client)


def test_torque_off_still_reaches_every_motor(stuck):
    client, port = stuck
    port.sent.clear()

    client.set_torque_enabled(IDS, False, retries=0)

    assert len(port.sent) == len(IDS)


def test_a_torque_write_that_raises_does_not_skip_the_rest(stuck):
    client, port = stuck
    port.sent.clear()
    port.raise_next_read = True

    failed = client.set_torque_enabled(IDS, False, retries=1)

    assert len(port.sent) == 2 * len(IDS)  # every motor, and the retry pass ran
    assert failed == IDS and not port.is_using


def test_disconnect_still_closes_and_deregisters(stuck):
    client, port = stuck

    client.disconnect()

    assert not port.is_open
    assert client not in client.OPEN_CLIENTS


def test_the_next_failed_transaction_releases_the_port(stuck):
    client, port = stuck

    client.read_hardware_error(IDS[0])

    assert not port.is_using


def test_a_sync_write_that_raised_does_not_resend_its_values():
    client, _ = _dynamixel()
    port = client.port_handler
    port.raise_next_write = True
    with pytest.raises(serial.SerialException):
        client.sync_write(IDS, [111, 222], TORQUE_ADDRESS, 1)
    client.set_torque_enabled(IDS, False, retries=0)  # reclaims the port
    port.sent.clear()

    client.sync_write(IDS, [7, 8], TORQUE_ADDRESS, 1)

    assert bytes([IDS[0], 7, IDS[1], 8]) in port.sent[-1]
