import sys
import types
from argparse import ArgumentParser
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MOCK_CONFIG_PATH = PROJECT_ROOT / "orca_core" / "models" / "v2" / "orcahand_right" / "config.yaml"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from orca_core import OrcaHand
from orca_core.hardware_hand import MockOrcaHand


def _install_fake_dynamixel_sdk() -> None:
    if "dynamixel_sdk" in sys.modules:
        return

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
    sys.modules["dynamixel_sdk"] = module


def add_hand_arguments(parser: ArgumentParser, *, mock_default: bool = False) -> None:
    parser.add_argument(
        "config_path",
        nargs="?",
        default=None,
        help="Path to config.yaml. Defaults to the bundled model when omitted.",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        default=mock_default,
        help="Use MockOrcaHand instead of a physical hand.",
    )


def create_hand(config_path: str | None, *, use_mock: bool):
    if use_mock:
        _install_fake_dynamixel_sdk()
        if config_path is None:
            config_path = str(DEFAULT_MOCK_CONFIG_PATH)
    hand_cls = MockOrcaHand if use_mock else OrcaHand
    return hand_cls(config_path=config_path)


def connect_hand(hand) -> None:
    success, message = hand.connect()
    print(f"connect() -> success={success}, message={message}")
    if not success:
        raise RuntimeError(message)


def shutdown_hand(hand) -> None:
    try:
        hand.stop_task()
    except Exception:
        pass
    try:
        success, message = hand.disconnect()
        print(f"disconnect() -> success={success}, message={message}")
    except Exception as exc:
        print(f"disconnect() failed: {exc}")


def prepare_output_dir(path: str | None, *, default_name: str = "replay_sequences") -> Path:
    output_dir = Path(path) if path is not None else PROJECT_ROOT / default_name
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def resolve_input_path(path: str, *, default_dir: str = "replay_sequences") -> Path:
    candidate = Path(path).expanduser()
    if candidate.is_absolute():
        return candidate

    if candidate.parent != Path("."):
        return (PROJECT_ROOT / candidate).resolve()

    return (PROJECT_ROOT / default_dir / candidate).resolve()
