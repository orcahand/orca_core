"""Wire-format frame builders and a stateful AA 55 responder for tactile
tests on top of ``MockHandSerialLink``.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from orca_core.constants import FINGER_NAMES
from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import (
    ADDR_AUTO_DATA_TYPE,
    ADDR_AUTO_ENABLE,
    ADDR_CONNECTED_SENSORS_LENGTH,
    ADDR_CONNECTED_SENSORS_START,
    ADDR_NUM_TAXELS_LENGTH,
    ADDR_NUM_TAXELS_START,
    ADDR_RESULTANT_FORCE_START,
    BYTES_PER_RESULTANT,
    DEFAULT_FINGER_TO_SENSOR_ID,
    DEFAULT_TAXEL_COUNTS,
    FUNC_CODE_READ,
    FUNC_CODE_WRITE,
    PROTOCOL_HEADER_AUTO,
    PROTOCOL_HEADER_RESPONSE,
    PROTOCOL_RESERVED,
    RESULTANT_BLOCK_SIZE,
    SLOT_CONNECTED_BIT_POSITIONS,
    SLOT_DISTAL_TAXEL_REGISTER_OFFSETS,
)
from orca_core.hardware.sensing.tactile_protocol import (
    ResultantForces,
    TaxelForces,
    _pack_resultant_for_mock,
    calculate_checksum,
    compute_distal_module_index,
    encode_combined_auto_for_mock,
    encode_resultant_auto_for_mock,
    encode_taxels_auto_for_mock,
)


@dataclass
class TactileMockState:
    """Mutable state behind the tactile mock's AA 55 responses.

    Tests set ``resultant_forces`` to control what ``read_resultant_force``
    returns; ``connected_fingers`` and ``taxel_counts`` shape the config
    read at ``connect()``.
    """
    connected_fingers: list[str] = field(default_factory=lambda: list(FINGER_NAMES))
    taxel_counts: dict[str, int] = field(default_factory=lambda: dict(DEFAULT_TAXEL_COUNTS))
    finger_to_sensor_id: dict[str, int] = field(
        default_factory=lambda: dict(DEFAULT_FINGER_TO_SENSOR_ID)
    )
    resultant_forces: ResultantForces = field(default_factory=dict)

    @property
    def active_sensors(self) -> list[str]:
        """Connected fingers in slot-id order (matches wire-frame order)."""
        active = [f for f in FINGER_NAMES if f in self.connected_fingers]
        active.sort(key=lambda f: self.finger_to_sensor_id[f])
        return active


def install_tactile_mock(link: MockHandSerialLink, state: TactileMockState) -> None:
    """Wire a response provider that serves AA 55 reads/writes from ``state``."""
    link.set_response_provider(lambda request: _respond_to_request(request, state))


# ---------------------------------------------------------------------------
# Auto-stream frame feeders (AA 56)
# ---------------------------------------------------------------------------

def feed_resultant_frame(
    link: MockHandSerialLink,
    forces: ResultantForces,
    active_sensors: list[str],
) -> None:
    valid = encode_resultant_auto_for_mock(forces, active_sensors)
    link.feed_bytes(_wrap_auto_frame(valid))


def feed_taxels_frame(
    link: MockHandSerialLink,
    taxels: TaxelForces,
    active_sensors: list[str],
) -> None:
    valid = encode_taxels_auto_for_mock(taxels, active_sensors)
    link.feed_bytes(_wrap_auto_frame(valid))


def feed_combined_frame(
    link: MockHandSerialLink,
    forces: ResultantForces,
    taxels: TaxelForces,
    active_sensors: list[str],
) -> None:
    valid = encode_combined_auto_for_mock(forces, taxels, active_sensors)
    link.feed_bytes(_wrap_auto_frame(valid))


def _wrap_auto_frame(valid_bytes: bytes, err_code: int = 0) -> bytes:
    """Wrap a payload with the AA 56 envelope: header + meta + err + LRC."""
    payload = bytes([err_code]) + valid_bytes
    eff_len = len(payload)
    body = (
        PROTOCOL_HEADER_AUTO
        + bytes([PROTOCOL_RESERVED])
        + eff_len.to_bytes(2, "little")
        + payload
    )
    return body + bytes([calculate_checksum(body)])


# ---------------------------------------------------------------------------
# Register-frame builders (AA 55)
# ---------------------------------------------------------------------------

def _respond_to_request(request: bytes, state: TactileMockState) -> bytes | None:
    if len(request) < 9:
        return None
    func = request[3]
    address = int.from_bytes(request[4:6], "little")
    count = int.from_bytes(request[6:8], "little")

    if func == FUNC_CODE_READ:
        data = _read_register_value(address, count, state)
        return _build_read_response(address, data)
    if func == FUNC_CODE_WRITE:
        return _build_write_response(address)
    return None


def _read_register_value(address: int, count: int, state: TactileMockState) -> bytes:
    if address == ADDR_CONNECTED_SENSORS_START:
        block = _encode_connected_sensors(state)
    elif address == ADDR_NUM_TAXELS_START:
        block = _encode_num_taxels(state)
    elif address == ADDR_RESULTANT_FORCE_START:
        block = _encode_resultant_register_block(state)
    elif address == ADDR_AUTO_DATA_TYPE:
        block = b"\x00"
    elif address == ADDR_AUTO_ENABLE:
        block = b"\x00"
    else:
        block = b"\x00" * count
    if len(block) >= count:
        return block[:count]
    return block + b"\x00" * (count - len(block))


def _encode_connected_sensors(state: TactileMockState) -> bytes:
    out = bytearray(ADDR_CONNECTED_SENSORS_LENGTH)
    for finger in state.connected_fingers:
        slot = state.finger_to_sensor_id[finger]
        byte_idx, bit_pos = SLOT_CONNECTED_BIT_POSITIONS[slot]
        out[byte_idx] |= 1 << bit_pos
    return bytes(out)


def _encode_num_taxels(state: TactileMockState) -> bytes:
    out = bytearray(ADDR_NUM_TAXELS_LENGTH)
    for finger in state.connected_fingers:
        slot = state.finger_to_sensor_id[finger]
        addr = SLOT_DISTAL_TAXEL_REGISTER_OFFSETS[slot]
        offset = addr - ADDR_NUM_TAXELS_START
        count = state.taxel_counts.get(finger, 0)
        out[offset:offset + 2] = count.to_bytes(2, "little")
    return bytes(out)


def _encode_resultant_register_block(state: TactileMockState) -> bytes:
    out = bytearray(RESULTANT_BLOCK_SIZE)
    for finger, force in state.resultant_forces.items():
        slot = state.finger_to_sensor_id[finger]
        module_idx = compute_distal_module_index(slot)
        offset = module_idx * BYTES_PER_RESULTANT
        out[offset:offset + BYTES_PER_RESULTANT] = _pack_resultant_for_mock(force)
    return bytes(out)


def _build_read_response(address: int, data: bytes) -> bytes:
    body = (
        PROTOCOL_HEADER_RESPONSE
        + bytes([PROTOCOL_RESERVED, FUNC_CODE_READ])
        + address.to_bytes(2, "little")
        + len(data).to_bytes(2, "little")
        + data
    )
    return body + bytes([calculate_checksum(body)])


def _build_write_response(address: int) -> bytes:
    payload = bytes([0x00])  # status byte: 0 = success
    body = (
        PROTOCOL_HEADER_RESPONSE
        + bytes([PROTOCOL_RESERVED, FUNC_CODE_WRITE])
        + address.to_bytes(2, "little")
        + len(payload).to_bytes(2, "little")
        + payload
    )
    return body + bytes([calculate_checksum(body)])
