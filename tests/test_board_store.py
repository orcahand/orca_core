"""The on-board store's encoding: what it keeps, what it refuses, what it costs.

Sizes here are measured by packing, never asserted from a constant — a hand's
calibration is as big as that hand's data, and a new joint count or a new
section should fail this suite rather than a board.
"""

import struct

import pytest

from orca_core.board_store import (
    FLAG_CALIBRATED,
    FLAG_WRIST_CALIBRATED,
    HEADER_BYTES,
    LAYOUT_VERSION,
    MAGIC,
    SLOT_BYTES,
    BoardIdentity,
    BoardStoreError,
    BoardStoreTooLarge,
    crc16,
    pack,
    packed_size,
    unpack,
)
from orca_core.utils.utils import read_yaml

JOINTS = ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip", "index_abd", "index_mcp",
          "index_pip", "middle_abd", "middle_mcp", "middle_pip", "ring_abd", "ring_mcp",
          "ring_pip", "pinky_abd", "pinky_mcp", "pinky_pip", "wrist"]


def taxel_counts():
    """Taxels per fingertip, from the packaged geometry rather than a constant."""
    from orca_core.hardware.sensing.taxel_geometry import load_all_taxel_geometry

    return {finger: geo.num_taxels for finger, geo in load_all_taxel_geometry().items()}


def tactile_document(counts=None):
    """What a touch hand adds: a gate and a 3-vector offset for every taxel."""
    counts = taxel_counts() if counts is None else counts
    return {
        "taxel_noise_gates": {f: [0.1] * n for f, n in counts.items()},
        "sensor_offsets": {f: [[0.0, 0.0, 0.0]] * n for f, n in counts.items()},
        "resultant_offsets": {f: [0.0, 0.0, 0.0] for f in counts},
    }


def full_document(joints=JOINTS, motors=17):
    """Every section populated — the most a hand of this size can hold."""
    return {
        "joint_to_motor_ratios": {m: 0.0123456789012345 + m for m in range(1, motors + 1)},
        "motor_limits": {m: [-1.1570817232131958, 1.851668119430542]
                         for m in range(1, motors + 1)},
        "calibrated": True,
        "wrist_calibrated": True,
        "motor_travel_measured": {j: 131.66 for j in joints},
        "joint_encoder_calibration": {j: {"enc_at_anchor_count": 11655} for j in joints},
        "joint_roms_measured": {j: [-58.14208984375, 35.0] for j in joints},
        "drive_current_measured": {j: {"flex": 242.3, "extend": 152.1} for j in joints},
        "rom_frame": "anchor",
    }


class TestTheRoundTrip:
    def test_a_full_document_comes_back_intact(self):
        doc = full_document()
        got = unpack(pack(doc)).calibration
        assert got == doc

    def test_floats_come_back_bit_exact(self):
        """float64, so a mirrored file equals what calibration produced."""
        doc = {"joint_to_motor_ratios": {1: 0.03230279509178986},
               "motor_limits": {1: [4.491495609283447, 6.309262752532959]}}
        got = unpack(pack(doc)).calibration
        assert got["joint_to_motor_ratios"][1] == 0.03230279509178986
        assert got["motor_limits"][1] == [4.491495609283447, 6.309262752532959]

    def test_a_motors_only_hand_carries_no_empty_sections(self):
        doc = {"joint_to_motor_ratios": {1: 0.02}, "motor_limits": {1: [0.0, 1.0]},
               "calibrated": False, "wrist_calibrated": False}
        got = unpack(pack(doc)).calibration
        assert got == doc
        assert "joint_encoder_calibration" not in got
        assert packed_size(doc) < packed_size(full_document())

    def test_an_uncalibrated_motor_is_absent_rather_than_null(self):
        """Absent means uncalibrated, which is how the yaml loader reads it too."""
        doc = {"motor_limits": {1: [0.0, 1.0], 2: [None, None]}}
        assert unpack(pack(doc)).calibration["motor_limits"] == {1: [0.0, 1.0]}

    def test_the_flags_carry_both_calibration_bits(self):
        for calibrated in (False, True):
            for wrist in (False, True):
                doc = {"calibrated": calibrated, "wrist_calibrated": wrist}
                got = unpack(pack(doc)).calibration
                assert got["calibrated"] is calibrated
                assert got["wrist_calibrated"] is wrist

    def test_the_rom_frame_survives(self):
        for frame in ("anchor", "centered"):
            assert unpack(pack({"rom_frame": frame})).calibration["rom_frame"] == frame

    def test_an_unknown_rom_frame_is_dropped_not_guessed(self):
        assert "rom_frame" not in unpack(pack({"rom_frame": "sideways"})).calibration


class TestTheIdentity:
    def test_a_provisioned_identity_round_trips(self):
        identity = BoardIdentity(side="left", hw_version=21, config=2500, serial="ser-0000")
        got = unpack(pack({}, identity=identity)).identity
        assert got == identity
        assert got.provisioned

    def test_an_unprovisioned_board_reports_nothing_rather_than_defaults(self):
        got = unpack(pack({})).identity
        assert (got.side, got.hw_version, got.config, got.serial) == (None, None, None, None)
        assert not got.provisioned

    def test_the_longest_serial_fits(self):
        identity = BoardIdentity(side="right", serial="A" * 20)
        assert unpack(pack({}, identity=identity)).identity.serial == "A" * 20

    def test_the_sequence_number_is_what_picks_a_slot(self):
        assert unpack(pack({}, sequence=7)).sequence == 7
        assert unpack(pack({}, sequence=0xFFFFFFFF)).sequence == 0xFFFFFFFF


class TestWhatItRefuses:
    def test_a_blank_slot(self):
        with pytest.raises(BoardStoreError, match="magic"):
            unpack(b"\xff" * SLOT_BYTES)

    def test_a_flipped_bit_anywhere_in_the_payload(self):
        blob = bytearray(pack(full_document()))
        blob[HEADER_BYTES + 5] ^= 0x01
        with pytest.raises(BoardStoreError, match="CRC"):
            unpack(bytes(blob))

    def test_a_flipped_bit_in_the_header(self):
        """The CRC covers the header too, so a corrupt length is caught."""
        blob = bytearray(pack(full_document()))
        blob[8] ^= 0x01  # sequence number
        with pytest.raises(BoardStoreError, match="CRC"):
            unpack(bytes(blob))

    def test_a_truncated_blob(self):
        blob = pack(full_document())
        with pytest.raises(BoardStoreError):
            unpack(blob[:HEADER_BYTES + 10])

    def test_something_shorter_than_a_header(self):
        with pytest.raises(BoardStoreError, match="header"):
            unpack(b"OHC1")

    def test_a_layout_this_reader_does_not_know(self):
        blob = bytearray(pack({}))
        blob[4] = LAYOUT_VERSION + 1
        struct.pack_into("<H", blob, 14, 0)
        payload = bytes(blob[HEADER_BYTES:])
        struct.pack_into("<H", blob, 14, crc16(bytes(blob[:14]) + payload))
        with pytest.raises(BoardStoreError, match="newer"):
            unpack(bytes(blob))

    def test_a_joint_index_with_no_name_behind_it(self):
        doc = {"motor_travel_measured": {"index_mcp": 1.0}}
        blob = bytearray(pack(doc, joints=["index_mcp"]))
        # Point the travel record at a joint that was never named.
        body_at = blob.index(struct.pack("<Bd", 0, 1.0))
        blob[body_at] = 9
        payload = bytes(blob[HEADER_BYTES:])
        struct.pack_into("<H", blob, 14, crc16(bytes(blob[:14]) + payload))
        with pytest.raises(BoardStoreError, match="joint index"):
            unpack(bytes(blob))


class TestRoomToGrow:
    def test_an_unknown_section_is_skipped_and_reported(self):
        """An older reader must not choke on a section added later."""
        blob = bytearray(pack({"calibrated": True}))
        count = blob[5]
        extra = struct.pack("<BBH", 200, 1, 4) + b"\xde\xad\xbe\xef"
        payload = bytes(blob[HEADER_BYTES:]) + extra
        blob[5] = count + 1
        struct.pack_into("<H", blob, 12, len(payload))
        struct.pack_into("<H", blob, 14, crc16(bytes(blob[:14]) + payload))
        store = unpack(bytes(blob[:HEADER_BYTES]) + payload)
        assert store.unknown_sections == (200,)
        assert store.calibration["calibrated"] is True

    def test_a_document_too_big_is_refused_with_both_sizes(self):
        doc = full_document(joints=[f"j{i:02d}" for i in range(200)], motors=200)
        with pytest.raises(BoardStoreTooLarge) as caught:
            pack(doc, slot_bytes=512)
        message = str(caught.value)
        assert "512" in message and "more room" in message

    def test_the_size_is_measured_not_assumed(self):
        small = packed_size({"calibrated": True})
        large = packed_size(full_document())
        assert small < large < SLOT_BYTES


class TestTheTactileSections:
    """A touch hand keeps its tactile calibration in the same document."""

    def test_they_round_trip_with_their_shape(self):
        doc = tactile_document()
        got = unpack(pack(doc, slot_bytes=1 << 20)).calibration
        counts = taxel_counts()
        for finger, n in counts.items():
            assert len(got["taxel_noise_gates"][finger]) == n
            assert len(got["sensor_offsets"][finger]) == n
            assert all(len(row) == 3 for row in got["sensor_offsets"][finger])
            assert len(got["resultant_offsets"][finger]) == 3

    def test_they_are_float32_which_the_sensor_scale_more_than_covers(self):
        """Each axis is reported on a single-byte scale, so 1/255 is its quantum."""
        doc = {"taxel_noise_gates": {"thumb": [0.1, 0.25, 1.0]}}
        got = unpack(pack(doc)).calibration["taxel_noise_gates"]["thumb"]
        for original, stored in zip([0.1, 0.25, 1.0], got):
            assert abs(original - stored) < 1e-6 * max(1.0, abs(original))
        assert got != [0.1, 0.25, 1.0]  # not exact, and that is the trade

    def test_a_hand_with_no_fingertips_stores_no_tactile_sections(self):
        got = unpack(pack(full_document())).calibration
        assert "taxel_noise_gates" not in got
        assert "sensor_offsets" not in got


class TestEveryShippedModelFits:
    """The per-model check.

    A hand with a different joint count, a different taxel count, or a new
    section fails here rather than on a board. Tactile is included for every
    model: fingertips can be added to a hand, and the slot is one size.
    """

    @pytest.mark.parametrize("config_path", sorted(
        str(p) for p in __import__("pathlib").Path("orca_core/models").rglob("config.yaml")
    ))
    def test_its_worst_case_fits_one_slot(self, config_path):
        config = read_yaml(config_path) or {}
        motors = config.get("joint_to_motor_map") or {}
        joints = config.get("joint_ids") or list(motors)
        doc = full_document(joints=[str(j) for j in joints], motors=len(motors))
        doc.update(tactile_document())
        identity = BoardIdentity(side="right", hw_version=21, config=2500, serial="S" * 20)
        size = packed_size(doc, identity=identity)
        assert size <= SLOT_BYTES, (
            f"{config_path}: worst case packs to {size} B, slot holds {SLOT_BYTES}"
        )
        # Room to grow: a slot should not be most of the way full already.
        assert size < SLOT_BYTES * 0.75, (
            f"{config_path}: {size} B is over three quarters of a {SLOT_BYTES} B slot; "
            "enlarge __cal_store_size in the firmware's linker script"
        )
