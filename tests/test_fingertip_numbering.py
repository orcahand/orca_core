"""Fingertip numbering against the connector board's register block.

Command bytes, register addresses, status layout and result codes are written out
from the firmware spec (orca_firmware docs/paxini/MODULE_NUMBER_PROVISIONING.md),
because production and the mock board share the constants.
"""
import pytest

from orca_core.hardware import tactile_client
from orca_core.hardware.sensing import tactile_mock
from orca_core.maintenance import fingertip_numbering as fn

COMMAND_REGISTER = 0x6110
SCAN = b"\x01\x00\x00\x00"
NAME = b"PXSR-CMDDP18B V1.0.0"
# [state, result, seq, found_count, found_addr, new_addr, sensor_status, responders, ident_len, raw_len]
SCAN_OK_AT_3 = bytes([2, 2, 0, 1, 3, 0, 0, 0b100, len(NAME), 0]) + bytes(6)


def test_register_map_follows_the_firmware_spec():
    from orca_core.hardware.sensing import constants as c

    assert (c.ADDR_EXT_ID, c.EXT_ID_MAGIC, c.ADDR_NUMBERING_COMMAND, c.ADDR_NUMBERING_STATUS,
            c.ADDR_NUMBERING_IDENTITY, c.ADDR_NUMBERING_RAW_REPLY, c.ADDR_SLOT_IDENTITY_START,
            c.SLOT_IDENTITY_STRIDE, c.EXT_CAP_FINGERTIP_NUMBERING, c.EXT_CAP_SLOT_IDENTITY,
            c.EXT_CAP_NUMBERING_SET_AT) == (
        0x6100, b"OX", 0x6110, 0x6120, 0x6130, 0x6150, 0x6200, 0x20, 0x01, 0x02, 0x04)


def _single_tip(tactile_mock_factory, module=2, part="PXSR-CMDDP18B"):
    link, client, state = tactile_mock_factory(connected_fingers=[])
    state.numbering.modules = [module]
    state.numbering.parts = {module: part}
    return client, state


def _hand(tactile_mock_factory, modules):
    link, client, state = tactile_mock_factory(connected_fingers=[])
    state.numbering.modules = list(modules)
    return client, state


def _board_reports(monkeypatch, blocks, identity=b"", raw_reply=b""):
    """Make the board answer each op with a literal status block, keyed by op byte."""
    def run(state, data):
        bus = state.numbering
        block = bytearray(blocks[data[0]])
        block[2] = (bus.status[2] + (block[0] == 2)) & 0xFF  # seq counts finished commands
        bus.status, bus.identity, bus.raw_reply = block, identity, raw_reply
        return 0

    monkeypatch.setattr(tactile_mock, "_run_numbering_command", run)


def test_scan_reads_the_status_block_the_firmware_writes(tactile_mock_factory, monkeypatch):
    client, _ = _single_tip(tactile_mock_factory)
    _board_reports(monkeypatch, {0x01: SCAN_OK_AT_3}, identity=NAME)
    scan = fn.scan_fingertips(client)
    assert scan.result == "scan_ok"
    assert scan.addresses == (3,)
    assert scan.single_module == 2
    assert scan.part == "PXSR-CMDDP18B"


def test_scan_writes_nothing_but_the_scan_command(tactile_mock_factory):
    client, state = _single_tip(tactile_mock_factory)
    fn.scan_fingertips(client)
    assert state.write_log == [(COMMAND_REGISTER, SCAN)]
    assert state.numbering.pending == {}


def test_set_sends_the_keyed_command(tactile_mock_factory):
    client, state = _single_tip(tactile_mock_factory, module=2, part="PXSR-CMDDP18A")
    done = fn.set_fingertip_module(client, 0)
    assert (done.old_module, done.new_module) == (2, 0)
    assert state.write_log == [(COMMAND_REGISTER, SCAN), (COMMAND_REGISTER, b"\x02MN\x00")]


@pytest.mark.parametrize(
    "code, result",
    [(1, "set_ok"), (2, "scan_ok"), (10, "no_sensor"), (11, "multiple"), (12, "bus_error"),
     (13, "already"), (15, "no_reply"), (16, "bad_reply"), (17, "sensor_rejected"), (18, "taken")],
)
def test_result_codes_follow_the_firmware_spec(tactile_mock_factory, monkeypatch, code, result):
    client, _ = _single_tip(tactile_mock_factory)
    _board_reports(monkeypatch, {0x01: bytes([2, code]) + bytes(14)})
    assert fn.scan_fingertips(client).result == result


def test_scan_lists_every_responding_and_garbled_module(tactile_mock_factory, monkeypatch):
    client, _ = _single_tip(tactile_mock_factory)
    _board_reports(monkeypatch, {0x01: bytes([2, 12, 0, 2, 2, 0, 0, 0b1010, 0, 0, 0b100]) + bytes(5)})
    scan = fn.scan_fingertips(client)
    assert (scan.modules, scan.garbled_modules, scan.single_module) == ((1, 3), (2,), None)


def test_an_unknown_scan_result_is_a_numbering_error(tactile_mock_factory, monkeypatch):
    client, state = _single_tip(tactile_mock_factory)
    _board_reports(monkeypatch, {0x01: bytes([2, 14]) + bytes(14)})
    with pytest.raises(fn.FingertipNumberingError) as err:
        fn.set_fingertip_module(client, 0)
    assert err.value.result == "code_14"
    assert [d for _, d in state.write_log] == [SCAN]


@pytest.mark.parametrize(
    "modules, result",
    [([], "no_sensor"), ([1, 3], "multiple"), ([2, 2], "bus_error")],
)
def test_set_refuses_unless_exactly_one_fingertip(tactile_mock_factory, modules, result):
    link, client, state = tactile_mock_factory(connected_fingers=[])
    state.numbering.modules = modules
    with pytest.raises(fn.FingertipNumberingError) as err:
        fn.set_fingertip_module(client, 0)
    assert err.value.result == result
    assert state.numbering.pending == {}
    assert [d for _, d in state.write_log] == [SCAN]


def test_set_refuses_the_current_number(tactile_mock_factory):
    client, state = _single_tip(tactile_mock_factory, module=1)
    with pytest.raises(fn.FingertipNumberingError) as err:
        fn.set_fingertip_module(client, 1)
    assert err.value.result == "already"


def test_set_at_renumbers_the_named_fingertip_in_a_full_hand(tactile_mock_factory):
    client, state = _hand(tactile_mock_factory, [0, 1, 2, 3, 4])
    done = fn.set_fingertip_module(client, 5, current=1)
    assert (done.old_module, done.new_module) == (1, 5)
    assert state.write_log == [(COMMAND_REGISTER, SCAN), (COMMAND_REGISTER, b"\x03MN\x05\x01")]


def test_swapping_two_fingertips_through_the_spare_number(tactile_mock_factory):
    client, state = _hand(tactile_mock_factory, [0, 1, 2, 3, 4])
    state.numbering.parts = {0: "PXSR-CMDDP18C", 4: "PXSR-CMDDP18A"}
    for new, current in [(5, 0), (0, 4), (4, 5)]:
        fn.set_fingertip_module(client, new, current=current)
        tactile_mock.power_cycle(state)
    assert sorted(state.numbering.modules) == [0, 1, 2, 3, 4]
    assert (state.numbering.parts[0], state.numbering.parts[4]) == ("PXSR-CMDDP18A", "PXSR-CMDDP18C")
    assert state.write_log[-1] == (COMMAND_REGISTER, b"\x03MN\x04\x05")


@pytest.mark.parametrize(
    "modules, new, current, result",
    [([0, 1, 2], 1, 2, "taken"), ([0, 1], 5, 3, "no_sensor"), ([0, 2, 2], 5, 0, "bus_error")],
)
def test_set_at_refuses_before_writing(tactile_mock_factory, modules, new, current, result):
    client, state = _hand(tactile_mock_factory, modules)
    with pytest.raises(fn.FingertipNumberingError) as err:
        fn.set_fingertip_module(client, new, current=current)
    assert err.value.result == result
    assert [d for _, d in state.write_log] == [SCAN]


@pytest.mark.parametrize("modules, sent", [([1], [SCAN, b"\x02MN\x05"]), ([0, 1], [SCAN])])
def test_firmware_without_set_at_renumbers_only_a_lone_fingertip(tactile_mock_factory, modules, sent):
    client, state = _hand(tactile_mock_factory, modules)
    state.numbering.set_at = False
    try:
        fn.set_fingertip_module(client, 5, current=1)
    except fn.FingertipNumberingError as err:
        assert err.result == "multiple"
    assert [d for _, d in state.write_log] == sent


def test_set_without_a_reply_is_unconfirmed(tactile_mock_factory):
    client, state = _single_tip(tactile_mock_factory)
    state.numbering.write_reply = False
    done = fn.set_fingertip_module(client, 1)
    assert (done.old_module, done.new_module, done.acknowledged) == (2, 1, False)


def test_set_with_a_garbled_reply_is_unconfirmed_and_keeps_the_bytes(tactile_mock_factory, monkeypatch):
    client, _ = _single_tip(tactile_mock_factory)
    raw = b"\xaa\x55\x03\x7e\x99"
    bad_reply = bytes([2, 16, 0, 1, 3, 2, 0, 0b100, len(NAME), len(raw)]) + bytes(6)
    _board_reports(monkeypatch, {0x01: SCAN_OK_AT_3, 0x02: bad_reply}, identity=NAME, raw_reply=raw)
    done = fn.set_fingertip_module(client, 1)
    assert (done.old_module, done.acknowledged, done.raw_reply) == (2, False, raw)


def test_set_reports_a_rejection_by_the_fingertip(tactile_mock_factory):
    client, state = _single_tip(tactile_mock_factory)
    state.numbering.write_status = 1
    with pytest.raises(fn.FingertipNumberingError) as err:
        fn.set_fingertip_module(client, 4)
    assert err.value.result == "sensor_rejected"
    assert "aa 55" in str(err.value)
    assert state.numbering.pending == {}


def test_set_rejects_out_of_range_numbers(tactile_mock_factory):
    client, _ = _single_tip(tactile_mock_factory)
    with pytest.raises(ValueError):
        fn.set_fingertip_module(client, 6)


def test_old_firmware_is_detected_before_any_command(tactile_mock_factory):
    client, state = _single_tip(tactile_mock_factory)
    state.numbering.supported = False
    with pytest.raises(fn.FingertipNumberingUnsupported):
        fn.scan_fingertips(client)
    assert state.write_log == []


def _lose_set_acks(client, monkeypatch, deliver=True):
    write = client._write_register

    def lossy(address, data, **kw):
        if deliver or data[0] != 0x02:
            write(address, data, **kw)
        if data[0] == 0x02:
            raise IOError("no register response")

    monkeypatch.setattr(client, "_write_register", lossy)


def test_a_lost_ack_counts_when_the_board_ran_the_command(tactile_mock_factory, monkeypatch):
    client, state = _single_tip(tactile_mock_factory)
    _lose_set_acks(client, monkeypatch)
    done = fn.set_fingertip_module(client, 1)
    assert (done.old_module, done.new_module, done.acknowledged) == (2, 1, True)
    assert state.numbering.pending == {2: 1}


def test_a_lost_ack_while_the_board_is_busy_waits_for_it(tactile_mock_factory, monkeypatch, virtual_clock):
    client, state = _single_tip(tactile_mock_factory)
    monkeypatch.setattr(tactile_client, "time", virtual_clock)
    _board_reports(monkeypatch, {0x01: SCAN_OK_AT_3, 0x02: bytes([1, 0]) + bytes(14)})
    _lose_set_acks(client, monkeypatch)
    with pytest.raises(fn.FingertipNumberingError) as err:
        fn.set_fingertip_module(client, 1, timeout_s=0.5)
    assert err.value.result == "timeout"


def test_a_command_that_never_arrived_is_not_taken(tactile_mock_factory, monkeypatch):
    client, state = _single_tip(tactile_mock_factory)
    _lose_set_acks(client, monkeypatch, deliver=False)
    with pytest.raises(fn.FingertipNumberingError) as err:
        fn.set_fingertip_module(client, 1)
    assert err.value.result == "rejected"
    assert state.numbering.pending == {}


def test_board_refusing_the_command_raises(tactile_mock_factory, monkeypatch):
    client, state = _single_tip(tactile_mock_factory)
    monkeypatch.setattr(fn, "NUMBERING_SET_KEY", b"XX")
    with pytest.raises(fn.FingertipNumberingError) as err:
        fn.set_fingertip_module(client, 0)
    assert err.value.result == "rejected"


def test_a_board_that_never_finishes_times_out(tactile_mock_factory, monkeypatch, virtual_clock):
    client, state = _single_tip(tactile_mock_factory)
    monkeypatch.setattr(tactile_client, "time", virtual_clock)
    monkeypatch.setattr(tactile_mock, "_run_numbering_command", lambda state, data: 0)
    with pytest.raises(fn.FingertipNumberingError) as err:
        fn.scan_fingertips(client, timeout_s=0.5)
    assert err.value.result == "timeout"


def test_slot_identities_and_mismatches(tactile_mock_factory):
    link, client, state = tactile_mock_factory(connected_fingers=[])
    state.numbering.modules = [0, 1, 4]
    state.numbering.parts = {0: "PXSR-CMDDP18A", 1: "PXSR-CMDDP18B", 4: "PXSR-CMDDP18A"}
    identities = fn.read_slot_identities(client)
    assert fn.sensor_part(identities["thumb"]) == "PXSR-CMDDP18A"
    assert identities["middle"] == ""
    assert fn.identity_mismatches(identities) == {"pinky": ("PXSR-CMDDP18C", "PXSR-CMDDP18A")}
