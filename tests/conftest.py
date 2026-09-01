import threading
import time

import pytest

from orca_core.hardware.joint_encoder_client import JointEncoderClient
from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import (
    DEFAULT_FINGER_TO_SENSOR_ID,
    DEFAULT_TAXEL_COUNTS,
)
from orca_core.hardware.sensing.tactile_mock import (
    TactileMockState,
    install_tactile_mock,
)
from orca_core.hardware.tactile_client import TactileClient


class _VirtualClock:
    """Stand-in for the ``time`` module that runs paced routines at full speed.

    ``sleep`` advances a virtual offset instead of blocking, and ``time`` /
    ``monotonic`` add that offset back. A routine that paces itself with
    ``sleep`` and bounds a phase with ``time.time() - start < limit`` therefore
    observes exactly the durations it asked for while costing no wall-clock
    time — deterministically, instead of racing a real deadline.
    """

    def __init__(self):
        self._offset = 0.0
        self._lock = threading.Lock()

    def __getattr__(self, name):
        return getattr(time, name)

    def sleep(self, seconds):
        if seconds and seconds > 0:
            with self._lock:
                self._offset += seconds

    @property
    def offset(self) -> float:
        with self._lock:
            return self._offset

    def time(self) -> float:
        return time.time() + self.offset

    def monotonic(self) -> float:
        return time.monotonic() + self.offset


_VIRTUAL_CLOCK = _VirtualClock()


@pytest.fixture(scope="session")
def virtual_clock():
    """The clock the paced routines see; read it to assert on their durations."""
    return _VIRTUAL_CLOCK


@pytest.fixture(autouse=True, scope="session")
def _no_settle_sleeps():
    """Run hardware-paced routines on a virtual clock instead of real time.

    Swaps the ``time`` module reference inside the routines that pace themselves
    for real motors. Patching ``time.sleep`` itself would be global and would
    break tests that rely on real delays to open a race window.
    """
    from orca_core import base_hand
    from orca_core.maintenance import calibration_routine, tensioning

    fake = _VIRTUAL_CLOCK
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(base_hand, "time", fake)
        mp.setattr(calibration_routine, "time", fake)
        mp.setattr(tensioning, "time", fake)
        mp.setattr(calibration_routine, "TINY_SLEEP", 0.0)
        yield


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
    yield from _encoder_link_and_client()


@pytest.fixture
def encoder_link_and_client_unfiltered():
    """As ``encoder_link_and_client``, with ``get_latest()`` smoothing off."""
    yield from _encoder_link_and_client(filter_cutoff_hz=None)


def _encoder_link_and_client(**client_kwargs):
    link = MockHandSerialLink()
    link.connect()
    client = JointEncoderClient(link, **client_kwargs)
    client.connect()
    try:
        yield link, client
    finally:
        try:
            client.disconnect()
        finally:
            link.disconnect()


# ----- Serial-port and driver-resolution fixtures ---------------------------

@pytest.fixture
def patch_comports(monkeypatch):
    """Replace serial.tools.list_ports.comports() with a fixed port list."""
    def _set(ports):
        import serial.tools.list_ports as ltp
        monkeypatch.setattr(ltp, "comports", lambda: ports)
    return _set


@pytest.fixture
def mock_config_dir(tmp_path):
    """Writable copy of the packaged v2 config so connect() can persist to it."""
    import os
    import shutil

    import orca_core

    model_config = os.path.join(
        os.path.dirname(orca_core.__file__), "models", "v2", "orcahand-right", "config.yaml"
    )
    shutil.copy(model_config, tmp_path / "config.yaml")
    (tmp_path / "calibration.yaml").write_text("{}\n", encoding="utf-8")
    return tmp_path


@pytest.fixture
def mock_hand(mock_config_dir):
    """Bare MockOrcaHand (not connected) for unit-testing helper methods."""
    from orca_core import MockOrcaHand

    return MockOrcaHand(config_path=str(mock_config_dir / "config.yaml"))


@pytest.fixture
def connected_mock_hand(mock_config_dir):
    from orca_core import MockOrcaHand

    hand = MockOrcaHand(config_path=str(mock_config_dir / "config.yaml"))
    success, msg = hand.connect()
    assert success, f"Failed to connect mock hand: {msg}"
    try:
        yield hand
    finally:
        hand.stop_task()
        hand.disconnect()


@pytest.fixture
def initialized_mock_hand(connected_mock_hand):
    connected_mock_hand.init_joints(force_calibrate=True)
    return connected_mock_hand
