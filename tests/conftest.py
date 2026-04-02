import shutil
import sys
import types
from pathlib import Path

import pytest

from orca_core.hardware_hand import MockOrcaHand


REPO_ROOT = Path(__file__).resolve().parent.parent
# default hand model
MODEL_DIR = REPO_ROOT / "orca_core" / "models" / "v2" / "orcahand_right"
MODEL_CONFIG = MODEL_DIR / "config.yaml"


def _install_fake_dynamixel_sdk() -> types.ModuleType:
    """Stub out the Dynamixel SDK for testing purposes."""
    module = types.ModuleType("dynamixel_sdk")

    class PortHandler:
        def __init__(self, port: str):
            self.port_name = port
            self.is_using = False

    class PacketHandler:
        def __init__(self, protocol_version: float):
            self.protocol_version = protocol_version

    class GroupBulkRead:
        def __init__(self, port_handler: PortHandler, packet_handler: PacketHandler):
            self.port_handler = port_handler
            self.packet_handler = packet_handler

        def addParam(self, motor_id: int, address: int, size: int) -> bool:
            return True

        def txRxPacket(self) -> int:
            return 0

        def isAvailable(self, motor_id: int, address: int, size: int) -> bool:
            return True

        def getData(self, motor_id: int, address: int, size: int) -> int:
            return 0

    module.PortHandler = PortHandler
    module.PacketHandler = PacketHandler
    module.GroupBulkRead = GroupBulkRead
    return module


@pytest.fixture(scope="session", autouse=True)
def fake_dynamixel_sdk():
    """Overrides the runtime dynamixel_sdk import with a fake stub implementatation
    for testing purposes."""
    original = sys.modules.get("dynamixel_sdk")
    sys.modules["dynamixel_sdk"] = _install_fake_dynamixel_sdk()
    try:
        yield
    finally:
        if original is None:
            sys.modules.pop("dynamixel_sdk", None)
        else:
            sys.modules["dynamixel_sdk"] = original


@pytest.fixture
def mock_config_dir(tmp_path: Path) -> Path:
    shutil.copy(MODEL_CONFIG, tmp_path / "config.yaml")
    (tmp_path / "calibration.yaml").write_text("{}\n", encoding="utf-8")
    return tmp_path


@pytest.fixture
def connected_mock_hand(mock_config_dir: Path) -> MockOrcaHand:
    hand = MockOrcaHand(config_path=str(mock_config_dir / "config.yaml"))
    success, msg = hand.connect()
    assert success, f"Failed to connect mock hand: {msg}"
    try:
        yield hand
    finally:
        hand.stop_task()
        hand.disconnect()


@pytest.fixture
def initialized_mock_hand(connected_mock_hand: MockOrcaHand) -> MockOrcaHand:
    connected_mock_hand.init_joints(force_calibrate=True)
    return connected_mock_hand
