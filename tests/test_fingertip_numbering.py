import pytest

from orca_core.hardware.sensing.constants import (
    ADDR_NUMBERING_COMMAND,
    NUMBERING_OP_SET,
    NUMBERING_SET_KEY,
)
from orca_core.hardware.sensing.tactile_mock import power_cycle
from orca_core.maintenance import fingertip_numbering as fn


def _single_tip(tactile_mock_factory, module=2, part="PXSR-CMDDP18B"):
    link, client, state = tactile_mock_factory(connected_fingers=[])
    state.numbering.modules = [module]
    state.numbering.parts = {module: part}
    return client, state


def test_scan_finds_the_one_connected_fingertip(tactile_mock_factory):
    client, _ = _single_tip(tactile_mock_factory, module=2)
    scan = fn.scan_fingertips(client)
    assert scan.result == "scan_ok"
    assert scan.addresses == (3,)
    assert scan.single_module == 2
    assert scan.part == "PXSR-CMDDP18B"


def test_scan_writes_nothing_but_the_scan_command(tactile_mock_factory):
    client, state = _single_tip(tactile_mock_factory)
    fn.scan_fingertips(client)
    assert [a for a, _ in state.write_log] == [ADDR_NUMBERING_COMMAND]
    assert state.numbering.pending == {}


def test_set_renumbers_after_a_power_cycle(tactile_mock_factory):
    client, state = _single_tip(tactile_mock_factory, module=2, part="PXSR-CMDDP18A")
    done = fn.set_fingertip_module(client, 0, expected_current=2)
    assert (done.old_module, done.new_module, done.power_cycle_required) == (2, 0, True)
    assert state.write_log[-1] == (
        ADDR_NUMBERING_COMMAND, bytes([NUMBERING_OP_SET, *NUMBERING_SET_KEY, 0])
    )
    assert fn.scan_fingertips(client).single_module == 2

    power_cycle(state)
    assert fn.scan_fingertips(client).single_module == 0
    assert state.connected_fingers == ["thumb"]


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
    assert all(w[1][0] != NUMBERING_OP_SET for w in state.write_log)


def test_set_refuses_the_current_number(tactile_mock_factory):
    client, state = _single_tip(tactile_mock_factory, module=1)
    with pytest.raises(fn.FingertipNumberingError) as err:
        fn.set_fingertip_module(client, 1)
    assert err.value.result == "already"


def test_set_checks_the_expected_current_number(tactile_mock_factory):
    client, state = _single_tip(tactile_mock_factory, module=3)
    with pytest.raises(fn.FingertipNumberingError) as err:
        fn.set_fingertip_module(client, 0, expected_current=2)
    assert err.value.result == "unexpected_module"
    assert state.numbering.pending == {}


def test_set_without_a_reply_is_unconfirmed_and_checked_after_the_power_cycle(tactile_mock_factory):
    client, state = _single_tip(tactile_mock_factory)
    state.numbering.write_reply = False
    done = fn.set_fingertip_module(client, 1)
    assert (done.old_module, done.new_module, done.acknowledged) == (2, 1, False)
    power_cycle(state)
    assert fn.scan_fingertips(client).single_module == 1


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


def test_board_refusing_the_command_raises(tactile_mock_factory, monkeypatch):
    client, state = _single_tip(tactile_mock_factory)
    monkeypatch.setattr(fn, "NUMBERING_SET_KEY", b"XX")
    with pytest.raises(fn.FingertipNumberingError) as err:
        fn.set_fingertip_module(client, 0)
    assert err.value.result == "rejected"


def test_a_board_that_never_finishes_times_out(tactile_mock_factory, monkeypatch, virtual_clock):
    client, state = _single_tip(tactile_mock_factory)
    monkeypatch.setattr(fn, "time", virtual_clock)
    monkeypatch.setattr(
        "orca_core.hardware.sensing.tactile_mock._run_numbering_command", lambda state, data: 0
    )
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


def test_default_module_follows_the_slot_map():
    assert [fn.default_module(f) for f in ("thumb", "index", "middle", "ring", "pinky")] == [0, 1, 2, 3, 4]
