"""Sensing-board firmware features: the version register, sensor zero, the
fingertip error report and reboot.

Register addresses, command bytes and status layout are written out from the
connector firmware (orca_firmware connector_board/firmware/src/commboard.h and
paxini_prov.h), because production and the mock board share the constants.
"""
import pytest

from orca_core import MockOrcaHandTouch, OrcaHandFull
from orca_core.hardware import tactile_client
from orca_core.hardware.sensing import tactile_mock
from orca_core.hardware.sensing.tactile_protocol import parse_board_version
from orca_core.hardware.sensing.types import SensingBoardVersion
from orca_core.hardware.tactile_client import TactileBoardError
from tests.test_touch_connect import TOUCH_CONFIG

FIRMWARE_1_0 = b"ORCA1.0-af7556\x00\x00"


@pytest.mark.parametrize("raw, kind, release, commit, dirty", [
    (bytes(16), "connector_legacy", None, "", False),
    (bytes(15), "connector_legacy", None, "", False),
    (b"ADAPTER_FW02.22", "paxini", None, "", False),
    (b"ADAPTER_FW02.22\x00", "paxini", None, "", False),
    (b"ORCA1.0-af7556\x00\x00", "connector", (1, 0), "af7556", False),
    (b"ORCA1.0+af7556\x00\x00", "connector", (1, 0), "af7556", True),
    (b"ORCA1.0-af7556\x00", "connector", (1, 0), "af7556", False),
    (b"ORCA12.34-0123ab", "connector", (12, 34), "0123ab", False),
    (b"ORCA\x00" + bytes(11), "connector", None, "", False),
    (b"ORCA1.x-af7556\x00\x00", "connector", None, "", False),
    (b"\xff\xfe" + bytes(14), "unknown", None, "", False),
    (b"AB\x07CD" + bytes(11), "unknown", None, "", False),
])
def test_version_register_tells_the_boards_apart(raw, kind, release, commit, dirty):
    v = parse_board_version(raw)
    assert (v.kind, v.release, v.commit, v.dirty) == (kind, release, commit, dirty)


def test_features_follow_the_release_number_not_an_exact_match():
    assert parse_board_version(b"ORCA7.42-0123ab").at_least((1, 0))
    assert parse_board_version(b"ORCA1.0-0123ab").at_least((1, 0))
    assert not parse_board_version(b"ORCA0.9-0123ab").at_least((1, 0))
    assert not parse_board_version(b"ORCA").at_least((1, 0))
    assert not SensingBoardVersion("paxini", "ORCA9.9 lookalike").at_least((1, 0))
    assert not parse_board_version(bytes(16)).at_least((1, 0))


def test_register_map_follows_the_firmware():
    from orca_core.hardware.sensing import constants as c

    assert (c.ADDR_VERSION, c.VERSION_LENGTH, c.ADDR_RESET, c.ADDR_TOTAL_ERROR, c.ADDR_MODULE_ERRORS,
            c.MODULE_ERRORS_LENGTH, c.ADDR_SLOT_COUNTERS, c.SLOT_COUNTER_STRIDE, c.NUMBERING_OP_ZERO,
            c.ZERO_KEY, c.SLOT_IDENTITY_LENGTH_1_0) == (
        0x0000, 16, 0x0022, 0x0023, 0x0700, 56, 0x6480, 16, 0x04, b"ZR", 32)


def test_field_boards_get_one_version_read_and_nothing_else(tactile_mock_factory):
    link, client, state = tactile_mock_factory(["index", "middle"])
    assert client.board_version.kind == "connector_legacy"
    assert not client.supports_sensor_zero
    assert [a for a, _ in state.read_log].count(0x0000) == 1
    assert {a for a, _ in state.read_log} == {0x0010, 0x0030, 0x0000}
    with pytest.raises(TactileBoardError) as err:
        client.zero_sensors()
    assert err.value.result == "unsupported"
    assert client.read_fingertip_health() is None
    assert state.write_log == []


def test_sensor_zero_goes_to_the_connected_fingertips(tactile_mock_factory):
    link, client, state = tactile_mock_factory(["index", "middle"], firmware=FIRMWARE_1_0)
    assert client.board_version.release == (1, 0)
    assert client.zero_sensors() == {"index": 0, "middle": 0}
    assert state.write_log == [(0x6110, bytes([0x04, ord("Z"), ord("R"), 0b00110]))]
    assert client.zero_sensors(["middle", "ring"]) == {"middle": 0}
    assert state.zeroed == [0b00110, 0b00100]


def test_sensor_zero_reports_a_fingertip_that_did_not_answer(tactile_mock_factory):
    link, client, state = tactile_mock_factory(["index", "middle"], firmware=FIRMWARE_1_0)
    state.numbering.modules = [1]
    assert client.zero_sensors() == {"index": 0, "middle": 0xFF}


def test_a_board_that_refuses_the_zero_is_not_taken_for_a_zero(tactile_mock_factory, monkeypatch):
    link, client, state = tactile_mock_factory(["index"], firmware=FIRMWARE_1_0)
    monkeypatch.setattr(tactile_mock, "_run_zero", lambda state, data: 1)
    with pytest.raises(TactileBoardError) as err:
        client.zero_sensors()
    assert err.value.result == "rejected"


def test_fingertip_health_decodes_the_error_report_and_counters(tactile_mock_factory):
    link, client, state = tactile_mock_factory(["index", "middle"], firmware=FIRMWARE_1_0)
    errors = bytearray(56)
    errors[2 * (2 * 4 + 2)] = 0x01           # middle (slot 2): distal module 10
    counters = bytearray(80)
    counters[32:48] = (1000).to_bytes(4, "little") + (3).to_bytes(4, "little") + bytes(4) + (1).to_bytes(4, "little")
    state.module_errors, state.slot_counters = bytes(errors), bytes(counters)
    health = client.read_fingertip_health()
    assert health["middle"].gone and not health["middle"].taxel_read_failed
    assert (health["middle"].polls_ok, health["middle"].timeouts, health["middle"].disconnects) == (1000, 3, 1)
    assert health["index"].error_code == 0 and health["index"].polls_ok == 0


@pytest.mark.parametrize("firmware", [b"", FIRMWARE_1_0])
def test_reboot_rearms_a_running_stream(tactile_mock_factory, monkeypatch, virtual_clock, firmware):
    monkeypatch.setattr(tactile_client, "time", virtual_clock)
    link, client, state = tactile_mock_factory(["index"], firmware=firmware)
    client.start_stream(resultant=True, taxels=True)
    state.write_log.clear()
    client.reboot_board()
    assert state.reboots == 1
    assert state.write_log == [(0x0022, b"\x01"), (0x0016, b"\x03"), (0x0017, b"\x01")]
    client.stop_stream()


@pytest.mark.parametrize("version", [b"ADAPTER_FW02.22", b"\xff\xfe" + bytes(14), None])
def test_reboot_is_refused_unless_the_board_is_ours(tactile_mock_factory, version):
    link, client, state = tactile_mock_factory(["index"])
    client._board_version = None if version is None else parse_board_version(version)
    with pytest.raises(TactileBoardError) as err:
        client.reboot_board()
    assert err.value.result == "unsupported"
    assert state.reboots == 0


def test_a_reboot_whose_ack_is_lost_still_counts(tactile_mock_factory, monkeypatch, virtual_clock):
    monkeypatch.setattr(tactile_client, "time", virtual_clock)
    link, client, state = tactile_mock_factory(["index"], firmware=FIRMWARE_1_0)
    write = client._write_register

    def ack_lost(address, value, **kwargs):
        write(address, value, **kwargs)
        raise IOError("no ACK")
    monkeypatch.setattr(client, "_write_register", ack_lost)
    client.reboot_board()
    assert state.reboots == 1


def test_a_reset_the_board_never_got_is_not_taken_for_a_reboot(tactile_mock_factory, monkeypatch, virtual_clock):
    monkeypatch.setattr(tactile_client, "time", virtual_clock)
    link, client, state = tactile_mock_factory(["index"], firmware=FIRMWARE_1_0)

    def lost(address, value, **kwargs):
        raise IOError("write lost")
    monkeypatch.setattr(client, "_write_register", lost)
    with pytest.raises(TactileBoardError) as err:
        client.reboot_board()
    assert err.value.result == "not_rebooted"


def test_a_zero_the_board_stops_part_way_is_an_error(tactile_mock_factory, monkeypatch):
    link, client, state = tactile_mock_factory(["index", "middle"], firmware=FIRMWARE_1_0)
    run_zero = tactile_mock._run_zero

    def bus_error(state, data):
        result = run_zero(state, data)
        state.numbering.status[1], state.numbering.status[13] = 12, 0xFD  # middle never reached
        return result
    monkeypatch.setattr(tactile_mock, "_run_zero", bus_error)
    with pytest.raises(TactileBoardError) as err:
        client.zero_sensors()
    assert err.value.result == "code_12"


def test_hand_zero_uses_the_sensor_zero_where_the_board_has_it(tactile_mock_factory):
    hand = MockOrcaHandTouch(config_path=TOUCH_CONFIG)
    ok, _ = hand.connect_sensors_only()
    assert ok
    try:
        for firmware, sensor_zero, expect in [
            (FIRMWARE_1_0, True, [0b00110]), (FIRMWARE_1_0, False, []), (b"", True, [])
        ]:
            link, client, state = tactile_mock_factory(["index", "middle"], firmware=firmware)
            client.capture_taxel_offsets = lambda **kwargs: {"captured": True}
            hand._tactile_client = client
            assert hand.zero_tactile_sensors(sensor_zero=sensor_zero) == {"captured": True}
            assert state.zeroed == expect
    finally:
        hand._tactile_client = None
        hand.disconnect()


def test_hand_zero_falls_back_to_the_host_baseline_when_the_sensor_zero_fails(tactile_mock_factory, monkeypatch):
    hand = MockOrcaHandTouch(config_path=TOUCH_CONFIG)
    ok, _ = hand.connect_sensors_only()
    assert ok
    try:
        link, client, state = tactile_mock_factory(["index"], firmware=FIRMWARE_1_0)
        monkeypatch.setattr(tactile_mock, "_run_zero", lambda state, data: 1)
        client.capture_taxel_offsets = lambda **kwargs: {"captured": True}
        hand._tactile_client = client
        assert hand.zero_tactile_sensors() == {"captured": True}
    finally:
        hand._tactile_client = None
        hand.disconnect()


def test_full_hand_refuses_a_reboot_while_its_joint_loop_runs():
    hand = OrcaHandFull.__new__(OrcaHandFull)
    hand._loop = object()
    with pytest.raises(RuntimeError, match="joint-feedback loop"):
        hand.reboot_tactile_board()
