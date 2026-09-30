"""The Feetech position loop's own gains.

Kp, Kd and Ki live at 50-52 in SRAM, loaded from EEPROM 21-23 at power-up.
This family has no feedforward, which is the one place its gains do not line
up with the shared ServoGains shape.
"""

import pytest

from orca_core.hardware.feetech_registers import HLS
from orca_core.hardware.mock_feetech_client import MockFeetechClient
from orca_core.hardware.motor_client import MotorClient, ServoGains


@pytest.fixture()
def client():
    c = MockFeetechClient([1, 2])
    c.connect()
    return c


def test_the_family_reports_gains_at_all(client):
    """The base class reports None for a family that cannot do this, so a
    real reading is what distinguishes implemented from inherited."""
    gains = client.read_servo_gains([1, 2])

    assert set(gains) == {1, 2}
    assert all(g is not None for g in gains.values())


def test_a_write_merges_rather_than_replaces(client):
    """None means leave it alone, so tuning one term cannot silently zero
    the other two."""
    before = client.read_servo_gains([1])[1]

    client.write_servo_gains({1: ServoGains(kp=90)})
    after = client.read_servo_gains([1])[1]

    assert after.kp == 90
    assert after.kd == before.kd
    assert after.ki == before.ki


def test_one_motor_at_a_time(client):
    client.write_servo_gains({1: ServoGains(kp=90)})

    assert client.read_servo_gains([2])[2].kp != 90


def test_feedforward_is_refused_not_dropped(client):
    """A caller asking for feedforward is tuning against a term this family
    does not have; silently ignoring it would leave them tuning nothing."""
    for field in ("ff_1st", "ff_2nd"):
        with pytest.raises(ValueError) as caught:
            client.write_servo_gains({1: ServoGains(**{field: 5})})
        assert field in str(caught.value)


def test_reported_gains_carry_no_feedforward(client):
    """None means 'not reported'. Zero would claim the term exists and is
    switched off."""
    gains = client.read_servo_gains([1])[1]

    assert gains.ff_1st is None
    assert gains.ff_2nd is None


def test_the_registers_are_a_contiguous_block():
    """Read and written as one block, so the map must stay adjacent."""
    assert HLS.GAIN_BLOCK == HLS.KP
    assert (HLS.KD, HLS.KI) == (HLS.KP + 1, HLS.KP + 2)
    assert HLS.GAIN_BLOCK_LEN == 3


def test_every_client_answers_the_call():
    """Adding a capability means implementing it everywhere, mocks included."""
    for name in ("read_servo_gains", "write_servo_gains"):
        assert getattr(MockFeetechClient, name) is not getattr(MotorClient, name)


def test_a_gain_above_the_register_range_is_refused():
    from orca_core.hardware.feetech_client import FeetechClient

    client = FeetechClient.__new__(FeetechClient)
    with pytest.raises(ValueError) as caught:
        client.write_servo_gains({1: ServoGains(kp=HLS.GAIN_MAX + 1)})
    assert str(HLS.GAIN_MAX) in str(caught.value)
