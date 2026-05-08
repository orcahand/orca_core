import pytest

from orca_core.hardware.joint_encoder_client import JointEncoderClient
from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import (
    DEFAULT_FINGER_TO_SENSOR_ID,
    DEFAULT_TAXEL_COUNTS,
)
from orca_core.hardware.tactile_client import TactileClient

from tests._tactile_helpers import TactileMockState, install_tactile_mock


@pytest.fixture
def tactile_mock():
    """Default tactile setup: connected client on a mock link, all 5 fingers."""
    state = TactileMockState()
    link = MockHandSerialLink()
    install_tactile_mock(link, state)
    link.connect()
    client = TactileClient(link, finger_to_sensor_id=state.finger_to_sensor_id)
    client.connect()
    try:
        yield link, client, state
    finally:
        try:
            client.disconnect()
        finally:
            link.disconnect()


@pytest.fixture
def tactile_mock_factory():
    """Factory for tactile-mock setups with arbitrary finger subsets."""
    created: list[tuple[MockHandSerialLink, TactileClient]] = []

    def _make(
        connected_fingers,
        taxel_counts=None,
        finger_to_sensor_id=None,
    ):
        state = TactileMockState(
            connected_fingers=list(connected_fingers),
            taxel_counts=(
                dict(taxel_counts)
                if taxel_counts is not None
                else dict(DEFAULT_TAXEL_COUNTS)
            ),
            finger_to_sensor_id=(
                dict(finger_to_sensor_id)
                if finger_to_sensor_id is not None
                else dict(DEFAULT_FINGER_TO_SENSOR_ID)
            ),
        )
        link = MockHandSerialLink()
        install_tactile_mock(link, state)
        link.connect()
        client = TactileClient(link, finger_to_sensor_id=state.finger_to_sensor_id)
        client.connect()
        created.append((link, client))
        return link, client, state

    yield _make
    for link, client in created:
        try:
            client.disconnect()
        except Exception:
            pass
        try:
            link.disconnect()
        except Exception:
            pass


@pytest.fixture
def encoder_link_and_client():
    """Encoder client connected on a mock link; AA A9 handler registered."""
    link = MockHandSerialLink()
    link.connect()
    client = JointEncoderClient(link)
    client.connect()
    try:
        yield link, client
    finally:
        try:
            client.disconnect()
        finally:
            link.disconnect()
