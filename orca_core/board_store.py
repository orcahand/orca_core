"""The hand's identity and calibration as its own board stores them.

A hand's calibration is a document whose shape follows the hand: how many
joints it drives, and which of the optional sections it has. So this encoding
carries its own structure — a joint-name table, length-tagged sections, an
explicit payload length — and nothing that reads it assumes a size or a joint
ordering. An older reader skips a section type it does not know.

Layout, little-endian throughout (the board is):

    header, 16 bytes
      0   4  magic b"OHC1"
      4   1  layout version
      5   1  section count
      6   2  flags: bit0 calibrated, bit1 wrist_calibrated
      8   4  sequence number, higher wins between the board's two slots
      12  2  payload length
      14  2  CRC-16/XMODEM over header[:14] + payload
    payload, one length-tagged section each
      0   1  section type
      1   1  record count
      2   2  body length
      4   .. body

Values are float64, not because the hardware needs it — a 14-bit encoder step
is ~4e-4 rad, some six orders coarser — but because space is not scarce here
and an exact round-trip means the file mirrored from the board equals what
calibration produced, with no digits lost in between.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

MAGIC = b"OHC1"
LAYOUT_VERSION = 1

_HEADER_FMT = "<4sBBHIHH"
HEADER_BYTES = struct.calcsize(_HEADER_FMT)
_SECTION_FMT = "<BBH"
_SECTION_BYTES = struct.calcsize(_SECTION_FMT)

# Mirrors __cal_store_slot_size in orca_firmware's
# variants/OpenRB-150/linker_scripts/gcc/flash_with_bootloader.ld, which is the
# authority for the board's flash map. A packed store must fit one slot.
SLOT_BYTES = 0x4000

FLAG_CALIBRATED = 1 << 0
FLAG_WRIST_CALIBRATED = 1 << 1

# Section types. Numbers are permanent: a reader skips what it does not know,
# so a retired type is never reused.
SEC_IDENTITY = 1
SEC_JOINT_NAMES = 2
SEC_RATIOS = 3
SEC_MOTOR_LIMITS = 4
SEC_TRAVEL = 5
SEC_ENCODER_ANCHORS = 6
SEC_ROMS_MEASURED = 7
SEC_DRIVE_CURRENTS = 8
SEC_ROM_FRAME = 9
# Tactile calibration, which a touch hand keeps in the same document. Stored as
# float32: the sensor reports each axis on a single-byte scale, so float32 is
# already several orders finer than the data (see orca_ui/hand/zeroing.py).
SEC_FINGER_NAMES = 10
SEC_TAXEL_GATES = 11
SEC_SENSOR_OFFSETS = 12
SEC_RESULTANT_OFFSETS = 13

_ROM_FRAMES = {1: "anchor", 2: "centered"}
_ROM_FRAME_CODES = {name: code for code, name in _ROM_FRAMES.items()}

_UNSET_U8 = 0xFF


class BoardStoreError(ValueError):
    """A blob that cannot be trusted: wrong magic, bad CRC, truncated."""


class BoardStoreTooLarge(BoardStoreError):
    """The packed store does not fit the slot it was destined for."""


def crc16(data: bytes, crc: int = 0) -> int:
    """CRC-16/XMODEM (poly 0x1021, init 0).

    The same one the SAM-BA bootloader's ``Z`` command returns, so a slot
    written through the bootloader can be verified without running firmware.
    """
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class BoardIdentity:
    """What provisioning assigns to a hand. ``None`` where unprovisioned."""

    side: Optional[str] = None          # "left" | "right"
    hw_version: Optional[int] = None
    config: Optional[int] = None        # sensing config code
    serial: Optional[str] = None

    @property
    def provisioned(self) -> bool:
        return self.side is not None


@dataclass(frozen=True)
class BoardStore:
    """What a slot held: the calibration document, plus who the hand is."""

    calibration: Dict[str, Any]
    identity: BoardIdentity
    sequence: int
    layout_version: int
    # Section types this reader did not understand, in the order seen.
    unknown_sections: Tuple[int, ...] = ()


def _finger_order(calibration: Mapping[str, Any]) -> List[str]:
    """Every fingertip the tactile sections mention, deterministically."""
    seen: Dict[str, None] = {}
    for key in ("taxel_noise_gates", "sensor_offsets", "resultant_offsets"):
        for finger in (calibration.get(key) or {}):
            seen.setdefault(str(finger), None)
    return sorted(seen)


def _name_table(type_code: int, names: Sequence[str]) -> bytes:
    body = b"".join(
        struct.pack("<B", len(n.encode("ascii"))) + n.encode("ascii") for n in names
    )
    return _section(type_code, len(names), body)


def _joint_order(calibration: Mapping[str, Any]) -> List[str]:
    """Every joint the document mentions, in a deterministic order."""
    seen: Dict[str, None] = {}
    for key in ("motor_travel_measured", "joint_encoder_calibration",
                "joint_roms_measured", "drive_current_measured"):
        for joint in (calibration.get(key) or {}):
            seen.setdefault(str(joint), None)
    return sorted(seen)


def _read_name_table(body: bytes) -> List[str]:
    names: List[str] = []
    cursor = 0
    while cursor < len(body):
        length = body[cursor]
        cursor += 1
        names.append(body[cursor:cursor + length].decode("ascii", "replace"))
        cursor += length
    return names


def _section(type_code: int, count: int, body: bytes) -> bytes:
    if count > 0xFF:
        raise BoardStoreError(f"section {type_code} has {count} records, max 255")
    if len(body) > 0xFFFF:
        raise BoardStoreError(f"section {type_code} body is {len(body)} bytes, max 65535")
    return struct.pack(_SECTION_FMT, type_code, count, len(body)) + body


def _pack_identity(identity: BoardIdentity) -> bytes:
    side = {"left": ord("L"), "right": ord("R")}.get(
        (identity.side or "").lower(), _UNSET_U8
    )
    serial = (identity.serial or "").encode("ascii")
    if len(serial) > 0xFF:
        raise BoardStoreError("the serial is longer than 255 bytes")
    body = struct.pack(
        "<BBHB",
        side,
        _UNSET_U8 if identity.hw_version is None else int(identity.hw_version),
        0 if identity.config is None else int(identity.config),
        len(serial),
    ) + serial
    return _section(SEC_IDENTITY, 1, body)


def _unpack_identity(body: bytes) -> BoardIdentity:
    side_raw, hw, config, serial_len = struct.unpack_from("<BBHB", body, 0)
    serial = body[struct.calcsize("<BBHB"):][:serial_len].decode("ascii", "replace")
    return BoardIdentity(
        side={ord("L"): "left", ord("R"): "right"}.get(side_raw),
        hw_version=None if hw == _UNSET_U8 else hw,
        config=config or None,
        serial=serial or None,
    )


def _pack_by_motor(type_code: int, mapping: Mapping[Any, Any], fmt: str,
                   values) -> Optional[bytes]:
    records = []
    for motor_id in sorted(mapping, key=int):
        payload = values(mapping[motor_id])
        if payload is None:
            continue
        records.append(struct.pack(fmt, int(motor_id), *payload))
    if not records:
        return None
    return _section(type_code, len(records), b"".join(records))


def _pack_by_joint(type_code: int, mapping: Mapping[str, Any], index: Mapping[str, int],
                   fmt: str, values) -> Optional[bytes]:
    records = []
    for joint in sorted(mapping):
        payload = values(mapping[joint])
        if payload is None:
            continue
        records.append(struct.pack(fmt, index[str(joint)], *payload))
    if not records:
        return None
    return _section(type_code, len(records), b"".join(records))


def _limits(value: Any) -> Optional[Tuple[float, float]]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    if value[0] is None or value[1] is None:
        return None
    return float(value[0]), float(value[1])


def pack(
    calibration: Mapping[str, Any],
    *,
    identity: BoardIdentity = BoardIdentity(),
    sequence: int = 1,
    joints: Optional[Sequence[str]] = None,
    slot_bytes: int = SLOT_BYTES,
) -> bytes:
    """Encode a calibration document and an identity into one slot's bytes.

    ``calibration`` is the mapping ``calibration.yaml`` holds. Sections the
    hand does not have are left out rather than zero-filled, so the result is
    as small as that hand's data.

    Raises:
        BoardStoreTooLarge: when the result would not fit ``slot_bytes``. The
            size is measured, never assumed: pack first, then compare.
    """
    names = [str(j) for j in (joints if joints is not None else _joint_order(calibration))]
    index = {name: i for i, name in enumerate(names)}

    sections: List[bytes] = [_pack_identity(identity)]

    if names:
        sections.append(_name_table(SEC_JOINT_NAMES, names))

    ratios = calibration.get("joint_to_motor_ratios") or {}
    packed = _pack_by_motor(SEC_RATIOS, ratios, "<Bd",
                            lambda v: None if v is None else (float(v),))
    if packed:
        sections.append(packed)

    packed = _pack_by_motor(SEC_MOTOR_LIMITS, calibration.get("motor_limits") or {},
                            "<Bdd", _limits)
    if packed:
        sections.append(packed)

    packed = _pack_by_joint(SEC_TRAVEL, calibration.get("motor_travel_measured") or {},
                            index, "<Bd", lambda v: None if v is None else (float(v),))
    if packed:
        sections.append(packed)

    packed = _pack_by_joint(
        SEC_ENCODER_ANCHORS, calibration.get("joint_encoder_calibration") or {}, index,
        "<BH",
        lambda v: None if not isinstance(v, Mapping) or v.get("enc_at_anchor_count") is None
        else (int(v["enc_at_anchor_count"]),),
    )
    if packed:
        sections.append(packed)

    packed = _pack_by_joint(SEC_ROMS_MEASURED, calibration.get("joint_roms_measured") or {},
                            index, "<Bdd", _limits)
    if packed:
        sections.append(packed)

    packed = _pack_by_joint(
        SEC_DRIVE_CURRENTS, calibration.get("drive_current_measured") or {}, index, "<Bdd",
        lambda v: None if not isinstance(v, Mapping)
        or v.get("flex") is None or v.get("extend") is None
        else (float(v["flex"]), float(v["extend"])),
    )
    if packed:
        sections.append(packed)

    fingers = _finger_order(calibration)
    if fingers:
        finger_index = {name: i for i, name in enumerate(fingers)}
        sections.append(_name_table(SEC_FINGER_NAMES, fingers))

        gates = calibration.get("taxel_noise_gates") or {}
        records = []
        for finger in sorted(gates):
            values = [float(v) for v in (gates[finger] or [])]
            records.append(struct.pack("<BH", finger_index[str(finger)], len(values))
                           + struct.pack(f"<{len(values)}f", *values))
        if records:
            sections.append(_section(SEC_TAXEL_GATES, len(records), b"".join(records)))

        offsets = calibration.get("sensor_offsets") or {}
        records = []
        for finger in sorted(offsets):
            rows = offsets[finger] or []
            flat = [float(v) for row in rows for v in row]
            records.append(struct.pack("<BH", finger_index[str(finger)], len(rows))
                           + struct.pack(f"<{len(flat)}f", *flat))
        if records:
            sections.append(_section(SEC_SENSOR_OFFSETS, len(records), b"".join(records)))

        resultants = calibration.get("resultant_offsets") or {}
        records = []
        for finger in sorted(resultants):
            values = [float(v) for v in (resultants[finger] or [])]
            records.append(struct.pack("<BH", finger_index[str(finger)], len(values))
                           + struct.pack(f"<{len(values)}f", *values))
        if records:
            sections.append(
                _section(SEC_RESULTANT_OFFSETS, len(records), b"".join(records))
            )

    frame = _ROM_FRAME_CODES.get(str(calibration.get("rom_frame") or ""))
    if frame:
        sections.append(_section(SEC_ROM_FRAME, 1, struct.pack("<B", frame)))

    flags = 0
    if calibration.get("calibrated"):
        flags |= FLAG_CALIBRATED
    if calibration.get("wrist_calibrated"):
        flags |= FLAG_WRIST_CALIBRATED

    payload = b"".join(sections)
    if len(payload) > 0xFFFF:
        raise BoardStoreTooLarge(
            f"the payload is {len(payload)} bytes; the length field holds 65535"
        )
    header = struct.pack(_HEADER_FMT, MAGIC, LAYOUT_VERSION, len(sections), flags,
                         int(sequence) & 0xFFFFFFFF, len(payload), 0)
    blob = header[:-2] + struct.pack("<H", crc16(header[:-2] + payload)) + payload
    if len(blob) > slot_bytes:
        raise BoardStoreTooLarge(
            f"the packed store is {len(blob)} bytes and the slot holds "
            f"{slot_bytes}; it needs {len(blob) - slot_bytes} bytes more room"
        )
    return blob


def packed_size(calibration: Mapping[str, Any], **kwargs: Any) -> int:
    """How many bytes this document takes, measured by packing it."""
    return len(pack(calibration, slot_bytes=1 << 30, **kwargs))


def unpack(blob: bytes) -> BoardStore:
    """Decode a slot's bytes. Raises ``BoardStoreError`` on anything untrusted.

    A blank slot (all 0xFF) and a slot written by a newer layout both raise,
    rather than being half-believed.
    """
    if len(blob) < HEADER_BYTES:
        raise BoardStoreError(f"{len(blob)} bytes is shorter than the {HEADER_BYTES}-byte header")
    magic, version, section_count, flags, sequence, payload_len, crc = struct.unpack_from(
        _HEADER_FMT, blob, 0
    )
    if magic != MAGIC:
        raise BoardStoreError(f"magic is {magic!r}, not {MAGIC!r}")
    if version > LAYOUT_VERSION:
        raise BoardStoreError(
            f"layout version {version} is newer than this reader's {LAYOUT_VERSION}"
        )
    end = HEADER_BYTES + payload_len
    if len(blob) < end:
        raise BoardStoreError(
            f"the header declares {payload_len} payload bytes, {len(blob) - HEADER_BYTES} present"
        )
    payload = blob[HEADER_BYTES:end]
    want = crc16(blob[:HEADER_BYTES - 2] + payload)
    if crc != want:
        raise BoardStoreError(f"CRC is 0x{crc:04X}, the bytes give 0x{want:04X}")

    calibration: Dict[str, Any] = {
        "calibrated": bool(flags & FLAG_CALIBRATED),
        "wrist_calibrated": bool(flags & FLAG_WRIST_CALIBRATED),
    }
    identity = BoardIdentity()
    names: List[str] = []
    fingers: List[str] = []
    unknown: List[int] = []
    pending: List[Tuple[int, int, bytes]] = []

    offset = 0
    for _ in range(section_count):
        if offset + _SECTION_BYTES > len(payload):
            raise BoardStoreError("a section header runs past the payload")
        type_code, count, body_len = struct.unpack_from(_SECTION_FMT, payload, offset)
        offset += _SECTION_BYTES
        if offset + body_len > len(payload):
            raise BoardStoreError(f"section {type_code} runs past the payload")
        body = payload[offset:offset + body_len]
        offset += body_len
        if type_code == SEC_IDENTITY:
            identity = _unpack_identity(body)
        elif type_code == SEC_JOINT_NAMES:
            names.extend(_read_name_table(body))
        elif type_code == SEC_FINGER_NAMES:
            fingers.extend(_read_name_table(body))
        elif type_code in (SEC_RATIOS, SEC_MOTOR_LIMITS, SEC_TRAVEL, SEC_ENCODER_ANCHORS,
                           SEC_ROMS_MEASURED, SEC_DRIVE_CURRENTS, SEC_ROM_FRAME,
                           SEC_TAXEL_GATES, SEC_SENSOR_OFFSETS, SEC_RESULTANT_OFFSETS):
            pending.append((type_code, count, body))
        else:
            unknown.append(type_code)

    def joint(idx: int) -> str:
        if idx >= len(names):
            raise BoardStoreError(f"joint index {idx} with only {len(names)} names stored")
        return names[idx]

    def finger(idx: int) -> str:
        if idx >= len(fingers):
            raise BoardStoreError(
                f"fingertip index {idx} with only {len(fingers)} names stored"
            )
        return fingers[idx]

    def per_finger(body: bytes, width: int):
        """Walk ``(finger, count, count*width floats)`` records."""
        cursor = 0
        while cursor < len(body):
            idx, count = struct.unpack_from("<BH", body, cursor)
            cursor += struct.calcsize("<BH")
            floats = count * width
            values = struct.unpack_from(f"<{floats}f", body, cursor)
            cursor += 4 * floats
            yield finger(idx), count, list(values)

    for type_code, count, body in pending:
        if type_code == SEC_RATIOS:
            calibration["joint_to_motor_ratios"] = {
                mid: value for mid, value in struct.iter_unpack("<Bd", body)
            }
        elif type_code == SEC_MOTOR_LIMITS:
            calibration["motor_limits"] = {
                mid: [lower, upper] for mid, lower, upper in struct.iter_unpack("<Bdd", body)
            }
        elif type_code == SEC_TRAVEL:
            calibration["motor_travel_measured"] = {
                joint(idx): value for idx, value in struct.iter_unpack("<Bd", body)
            }
        elif type_code == SEC_ENCODER_ANCHORS:
            calibration["joint_encoder_calibration"] = {
                joint(idx): {"enc_at_anchor_count": value}
                for idx, value in struct.iter_unpack("<BH", body)
            }
        elif type_code == SEC_ROMS_MEASURED:
            calibration["joint_roms_measured"] = {
                joint(idx): [lower, upper]
                for idx, lower, upper in struct.iter_unpack("<Bdd", body)
            }
        elif type_code == SEC_DRIVE_CURRENTS:
            calibration["drive_current_measured"] = {
                joint(idx): {"flex": flex, "extend": extend}
                for idx, flex, extend in struct.iter_unpack("<Bdd", body)
            }
        elif type_code == SEC_TAXEL_GATES:
            calibration["taxel_noise_gates"] = {
                name: values for name, _, values in per_finger(body, 1)
            }
        elif type_code == SEC_SENSOR_OFFSETS:
            calibration["sensor_offsets"] = {
                name: [values[i * 3:i * 3 + 3] for i in range(count)]
                for name, count, values in per_finger(body, 3)
            }
        elif type_code == SEC_RESULTANT_OFFSETS:
            calibration["resultant_offsets"] = {
                name: values for name, _, values in per_finger(body, 1)
            }
        elif type_code == SEC_ROM_FRAME:
            frame = _ROM_FRAMES.get(body[0] if body else 0)
            if frame:
                calibration["rom_frame"] = frame

    return BoardStore(
        calibration=calibration,
        identity=identity,
        sequence=sequence,
        layout_version=version,
        unknown_sections=tuple(unknown),
    )
