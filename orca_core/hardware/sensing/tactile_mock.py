# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Wire-format frame builders and a stateful AA 55 responder for driving a
``MockHandSerialLink`` with synthetic tactile data (tests, demos, examples).
"""
from __future__ import annotations

from dataclasses import dataclass, field

from orca_core.constants import FINGER_NAMES
from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import (
    ADDR_AUTO_DATA_TYPE,
    ADDR_AUTO_ENABLE,
    ADDR_MODULE_ERRORS,
    ADDR_BOARD_STATS,
    ADDR_BOARD_STATS_LEGACY,
    ADDR_RESET,
    ADDR_SLOT_COUNTERS,
    ADDR_TOTAL_ERROR,
    ADDR_VERSION,
    NUMBERING_OP_ZERO,
    SLOT_IDENTITY_LENGTH_1_0,
    VERSION_PREFIX_ORCA,
    ZERO_KEY,
    ZERO_NO_REPLY,
    ADDR_EXT_ID,
    ADDR_NUMBERING_COMMAND,
    ADDR_NUMBERING_IDENTITY,
    ADDR_NUMBERING_RAW_REPLY,
    ADDR_NUMBERING_STATUS,
    ADDR_SLOT_IDENTITY_START,
    ADDR_CONNECTED_SENSORS_LENGTH,
    ADDR_CONNECTED_SENSORS_START,
    ADDR_NUM_TAXELS_LENGTH,
    ADDR_NUM_TAXELS_START,
    ADDR_RESULTANT_FORCE_START,
    BYTES_PER_RESULTANT,
    DEFAULT_FINGER_TO_SENSOR_ID,
    DEFAULT_TAXEL_COUNTS,
    EXT_CAP_FINGERTIP_NUMBERING,
    EXT_CAP_NUMBERING_SET_AT,
    EXT_CAP_SLOT_IDENTITY,
    EXT_ID_MAGIC,
    FINGER_SENSOR_PART,
    FUNC_CODE_READ,
    FUNC_CODE_WRITE,
    NUMBERING_MAX_MODULE,
    NUMBERING_OP_SCAN,
    NUMBERING_OP_SET,
    NUMBERING_OP_SET_AT,
    NUMBERING_SET_KEY,
    NUMBERING_STATUS_LENGTH,
    PROTOCOL_HEADER_RESPONSE,
    RESULTANT_BLOCK_SIZE,
    SLOT_CONNECTED_BIT_POSITIONS,
    SLOT_DISTAL_TAXEL_REGISTER_OFFSETS,
    SLOT_IDENTITY_LENGTH,
    SLOT_IDENTITY_STRIDE,
)
from orca_core.hardware.sensing.framing import calculate_checksum
from orca_core.hardware.sensing.types import ResultantForces, TaxelForces
from orca_core.hardware.sensing.tactile_protocol import (
    _pack_resultant_for_mock,
    build_auto_frame,
    build_register_frame,
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
    write_log: list[tuple[int, bytes]] = field(default_factory=list)
    """Every register write the mock ACKed, as (address, data) — lets tests
    assert on device-bound side effects like the stream re-arm."""
    numbering: FingertipBusMock = field(default_factory=lambda: FingertipBusMock())
    """The connector board's fingertip-numbering registers and the fingertips
    on its bus; tests renumber through it and then call ``power_cycle``."""
    firmware: bytes = b""
    """The version register (0x0000). Empty reads zeros, like connector firmware
    before 1.0; ``b"ORCA1.0-..."`` turns on sensor zero, the error report and
    32-byte slot names."""
    module_errors: bytes = bytes(56)
    slot_counters: bytes = bytes(80)
    zeroed: list[int] = field(default_factory=list)
    """Slot mask of every sensor zero the board ran."""
    reboots: int = 0
    frames_received: int = 0
    """The board's received-request counter (first stat), cleared by a reboot."""
    read_log: list[tuple[int, int]] = field(default_factory=list)
    """Every register read, as (address, count)."""

    @property
    def firmware_1_0(self) -> bool:
        return self.firmware.startswith(VERSION_PREFIX_ORCA.encode())

    @property
    def active_sensors(self) -> list[str]:
        """Connected fingers in slot-id order (matches wire-frame order)."""
        active = [f for f in FINGER_NAMES if f in self.connected_fingers]
        active.sort(key=lambda f: self.finger_to_sensor_id[f])
        return active


@dataclass
class FingertipBusMock:
    """Fingertips on the bus by module number, plus the board's numbering state.

    ``modules`` lists the module number of each physically connected fingertip
    (repeats model two tips sharing a number). ``None`` mirrors the mock's
    connected fingers. A renumbering is staged until ``power_cycle``;
    ``write_reply=False`` models a fingertip that takes it without answering,
    ``set_at=False`` firmware that renumbers only a lone fingertip.
    """
    supported: bool = True
    set_at: bool = True
    modules: list[int] | None = None
    parts: dict[int, str] = field(default_factory=dict)
    write_status: int = 0
    write_reply: bool = True
    pending: dict[int, int] = field(default_factory=dict)
    status: bytearray = field(default_factory=lambda: bytearray(NUMBERING_STATUS_LENGTH))
    identity: bytes = b""
    raw_reply: bytes = b""


def _bus_modules(state: TactileMockState) -> list[int]:
    if state.numbering.modules is not None:
        return list(state.numbering.modules)
    return sorted(state.finger_to_sensor_id[f] for f in state.connected_fingers)


def _part_for_module(state: TactileMockState, module: int) -> str:
    if module in state.numbering.parts:
        return state.numbering.parts[module]
    finger = next((f for f, s in state.finger_to_sensor_id.items() if s == module), None)
    return FINGER_SENSOR_PART.get(finger, "")


def _sensor_name(part: str) -> bytes:
    return f"{part} V1.0.0".encode() if part else b""


def power_cycle(state: TactileMockState) -> None:
    """Apply staged renumberings and rebuild the connected fingers from the bus."""
    bus = state.numbering
    modules = _bus_modules(state)
    parts = {m: _part_for_module(state, m) for m in modules}
    renumbered = [bus.pending.get(m, m) for m in modules]
    bus.parts = {bus.pending.get(m, m): parts[m] for m in modules}
    bus.modules = renumbered
    bus.pending = {}
    slot_to_finger = {s: f for f, s in state.finger_to_sensor_id.items()}
    unique = [m for m in renumbered if renumbered.count(m) == 1 and m in slot_to_finger]
    state.connected_fingers = [slot_to_finger[m] for m in sorted(unique)]


def _run_numbering_command(state: TactileMockState, data: bytes) -> int:
    bus = state.numbering
    if not bus.supported:
        return 0
    op = data[0] if data else 0
    if op == NUMBERING_OP_ZERO and state.firmware_1_0:
        return _run_zero(state, data)
    keyed = op in (NUMBERING_OP_SET, NUMBERING_OP_SET_AT)
    sizes = {NUMBERING_OP_SCAN: 4, NUMBERING_OP_SET: 4, NUMBERING_OP_SET_AT: 5 if bus.set_at else 0}
    if len(data) != sizes.get(op):
        return 1
    if keyed and (bytes(data[1:3]) != NUMBERING_SET_KEY or max(data[3:]) > NUMBERING_MAX_MODULE):
        return 1
    arg = data[3]

    modules = _bus_modules(state)
    addresses = sorted({m + 1 for m in modules if modules.count(m) == 1})
    garbled = sorted({m + 1 for m in modules if modules.count(m) > 1})
    status = bytearray(NUMBERING_STATUS_LENGTH)
    status[0] = 2
    status[2] = (bus.status[2] + 1) & 0xFF
    status[3] = len(addresses)
    status[4] = addresses[0] if addresses else 0
    status[5] = arg + 1 if keyed else 0
    status[7] = sum(1 << (a - 1) for a in addresses)
    status[10] = sum(1 << (a - 1) for a in garbled)
    bus.identity, bus.raw_reply = b"", b""

    if garbled:
        result = 12
    elif op == NUMBERING_OP_SET_AT:
        current = data[4]
        if current + 1 not in addresses:
            result = 10
        elif current == arg:
            result = 13
        elif arg + 1 in addresses:
            result = 18
        else:
            status[4] = current + 1
            result = _write_module(state, status, current, arg)
    elif not addresses:
        result = 10
    elif len(addresses) > 1:
        result = 11
    elif op == NUMBERING_OP_SCAN:
        bus.identity = _sensor_name(_part_for_module(state, addresses[0] - 1))
        result = 2
    elif addresses[0] - 1 == arg:
        result = 13
    else:
        result = _write_module(state, status, addresses[0] - 1, arg)
    status[1] = result
    status[8] = len(bus.identity)
    status[9] = len(bus.raw_reply)
    bus.status = status
    return 0


def _run_zero(state: TactileMockState, data: bytes) -> int:
    """Connector firmware 1.0's sensor zero: each slot in the mask answers or doesn't."""
    bus = state.numbering
    if len(data) != 4 or bytes(data[1:3]) != ZERO_KEY or not 0 < data[3] < 0x20:
        return 1
    modules = _bus_modules(state)
    status = bytearray(NUMBERING_STATUS_LENGTH)
    status[0], status[2] = 2, (bus.status[2] + 1) & 0xFF
    status[11:16] = bytes(0xFD for _ in range(5))
    for slot in range(5):
        if data[3] >> slot & 1:
            status[11 + slot] = 0 if modules.count(slot) == 1 else ZERO_NO_REPLY
    status[1] = 3 if all(status[11 + s] == 0 for s in range(5) if data[3] >> s & 1) else 19
    bus.status = status
    state.zeroed.append(data[3])
    return 0


def _write_module(state: TactileMockState, status: bytearray, current: int, new: int) -> int:
    """The board's module write to the fingertip at ``current``; returns the result code."""
    bus = state.numbering
    bus.identity = _sensor_name(_part_for_module(state, current))
    if not bus.write_reply:
        bus.pending[current] = new
        return 15
    status[6] = bus.write_status
    # Opaque to orca_core: the board's capture of the fingertip's reply.
    reply = bytes([0xAA, 0x55, current + 1, bus.write_status])
    bus.raw_reply = reply + bytes([calculate_checksum(reply)])
    if bus.write_status != 0:
        return 17
    bus.pending[current] = new
    return 1


def _numbering_block(state: TactileMockState, address: int) -> bytes | None:
    bus = state.numbering
    if not bus.supported:
        return None
    if address == ADDR_EXT_ID:
        caps = EXT_CAP_FINGERTIP_NUMBERING | EXT_CAP_SLOT_IDENTITY
        return EXT_ID_MAGIC + bytes([1, caps | (EXT_CAP_NUMBERING_SET_AT if bus.set_at else 0)])
    if address == ADDR_NUMBERING_STATUS:
        return bytes(bus.status)
    if address == ADDR_NUMBERING_IDENTITY:
        return bus.identity
    if address == ADDR_NUMBERING_RAW_REPLY:
        return bus.raw_reply
    offset = address - ADDR_SLOT_IDENTITY_START
    if 0 <= offset < SLOT_IDENTITY_STRIDE * len(DEFAULT_FINGER_TO_SENSOR_ID) and offset % SLOT_IDENTITY_STRIDE == 0:
        slot = offset // SLOT_IDENTITY_STRIDE
        modules = _bus_modules(state)
        if modules.count(slot) == 1:
            length = SLOT_IDENTITY_LENGTH_1_0 if state.firmware_1_0 else SLOT_IDENTITY_LENGTH
            return _sensor_name(_part_for_module(state, slot))[:length]
        return b""
    return None


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
    link.feed_bytes(build_auto_frame(valid))


def feed_taxels_frame(
    link: MockHandSerialLink,
    taxels: TaxelForces,
    active_sensors: list[str],
) -> None:
    valid = encode_taxels_auto_for_mock(taxels, active_sensors)
    link.feed_bytes(build_auto_frame(valid))


def feed_combined_frame(
    link: MockHandSerialLink,
    forces: ResultantForces,
    taxels: TaxelForces,
    active_sensors: list[str],
) -> None:
    valid = encode_combined_auto_for_mock(forces, taxels, active_sensors)
    link.feed_bytes(build_auto_frame(valid))


# ---------------------------------------------------------------------------
# Register-frame builders (AA 55)
# ---------------------------------------------------------------------------

def _respond_to_request(request: bytes, state: TactileMockState) -> bytes | None:
    if len(request) < 9:
        return None
    func = request[3]
    address = int.from_bytes(request[4:6], "little")
    count = int.from_bytes(request[6:8], "little")

    if func in (FUNC_CODE_READ, FUNC_CODE_WRITE):
        state.frames_received += 1
    if func == FUNC_CODE_READ:
        state.read_log.append((address, count))
        data = _read_register_value(address, count, state)
        return _build_read_response(address, data)
    if func == FUNC_CODE_WRITE:
        data = bytes(request[8:8 + count])
        state.write_log.append((address, data))
        status = _run_numbering_command(state, data) if address == ADDR_NUMBERING_COMMAND else 0
        if address == ADDR_RESET and data[:1] == b"\x01":
            state.reboots += 1
            state.frames_received = 0
        return _build_write_response(address, status)
    return None


def _read_register_value(address: int, count: int, state: TactileMockState) -> bytes:
    numbering = _numbering_block(state, address)
    if numbering is not None:
        block = numbering
    elif address == ADDR_VERSION:
        block = state.firmware
    elif state.firmware_1_0 and address == ADDR_TOTAL_ERROR:
        block = bytes([any(state.module_errors)])
    elif state.firmware_1_0 and address == ADDR_MODULE_ERRORS:
        block = state.module_errors
    elif address == (ADDR_BOARD_STATS if state.firmware_1_0 else ADDR_BOARD_STATS_LEGACY):
        block = state.frames_received.to_bytes(4, "little")
    elif state.firmware_1_0 and address == ADDR_SLOT_COUNTERS:
        block = state.slot_counters
    elif address == ADDR_CONNECTED_SENSORS_START:
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
    return build_register_frame(
        PROTOCOL_HEADER_RESPONSE, FUNC_CODE_READ, address, len(data), data,
    )


def _build_write_response(address: int, status: int = 0) -> bytes:
    payload = bytes([status])  # 0 = success
    return build_register_frame(
        PROTOCOL_HEADER_RESPONSE, FUNC_CODE_WRITE, address, len(payload), payload,
    )
