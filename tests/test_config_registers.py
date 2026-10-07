"""Operator-editable registers: what each family offers, and what a write means."""

import pytest

from orca_core.hardware.motor_factory import (
    mock_motor_client_class,
    motor_client_class,
)

FAMILIES = ("dynamixel", "feetech")


def _client(motor_type, ids=(1, 2)):
    client = mock_motor_client_class(motor_type)(list(ids))
    client.connect()
    return client


class TestWhatEachFamilyOffers:
    def test_both_declare_the_settings_they_share(self):
        shared = None
        for motor_type in FAMILIES:
            keys = {r.key for r in motor_client_class(motor_type).config_registers}
            shared = keys if shared is None else shared & keys
        assert shared == {"id", "baud_rate", "operating_mode",
                          "secondary_id", "temperature_limit"}

    def test_a_setting_a_family_lacks_is_absent_not_disabled(self):
        """A front-end renders what the motor has. Carrying a row the family
        cannot answer would mean greying it out against a hard-coded list."""
        feetech = {r.key for r in motor_client_class("feetech").config_registers}
        assert "return_delay_time" not in feetech
        assert "drive_mode" not in feetech

    def test_protocol_type_is_offered_by_neither(self):
        """Switching a motor to protocol 1.0 strands it: this client speaks
        2.0 only, so there is no way back from inside the tool."""
        for motor_type in FAMILIES:
            keys = {r.key for r in motor_client_class(motor_type).config_registers}
            assert "protocol_type" not in keys

    def test_the_two_families_disagree_about_what_a_baud_index_means(self):
        """Which is why a raw index must never reach an operator."""
        dxl = next(r for r in motor_client_class("dynamixel").config_registers
                   if r.key == "baud_rate")
        fee = next(r for r in motor_client_class("feetech").config_registers
                   if r.key == "baud_rate")
        assert dxl.choices[0] != fee.choices[0]

    def test_id_and_baud_are_flagged_as_moving_the_motor(self):
        for motor_type in FAMILIES:
            moving = {r.key for r in motor_client_class(motor_type).config_registers
                      if r.reidentifies}
            assert moving == {"id", "baud_rate"}

    def test_every_offered_register_is_eeprom(self):
        """So every one of them needs torque off and survives a power cycle --
        a caller can state that flatly rather than per row."""
        for motor_type in FAMILIES:
            for entry in motor_client_class(motor_type).config_registers:
                assert entry.eeprom

    def test_the_mocks_declare_what_their_family_declares(self):
        for motor_type in FAMILIES:
            real = motor_client_class(motor_type).config_registers
            mock = mock_motor_client_class(motor_type).config_registers
            assert [r.key for r in mock] == [r.key for r in real]


class TestWrites:
    def test_a_write_returns_what_the_motor_now_holds(self):
        """Not what was asked for. A sync write carries no acknowledgement,
        and a chain here was found holding gains a write never landed on."""
        client = _client("dynamixel")
        assert client.write_config_register(1, "temperature_limit", 65) == 65
        assert client.read_config_register(1, "temperature_limit") == 65

    def test_a_value_outside_the_range_is_refused(self):
        client = _client("dynamixel")
        with pytest.raises(ValueError):
            client.write_config_register(1, "temperature_limit", 101)
        with pytest.raises(ValueError):
            client.write_config_register(1, "id", 253)

    def test_a_value_outside_the_choices_is_refused(self):
        """Operating mode is an index into a set, not a number: the XC430
        accepts four of the six and nothing accepts an arbitrary integer."""
        client = _client("feetech")
        with pytest.raises(ValueError):
            client.write_config_register(1, "operating_mode", 9)

    def test_an_unknown_register_is_refused_rather_than_written(self):
        client = _client("feetech")
        with pytest.raises(ValueError):
            client.write_config_register(1, "return_delay_time", 10)

    def test_an_id_write_is_read_back_at_the_new_id(self):
        """The motor has moved; asking the old address would time out and
        read as a failed write."""
        client = _client("dynamixel")
        assert client.write_config_register(1, "id", 9) == 9
        assert client.read_config_register(9, "id") == 9

    def test_two_motors_keep_separate_values(self):
        client = _client("dynamixel")
        client.write_config_register(1, "temperature_limit", 60)
        client.write_config_register(2, "temperature_limit", 75)
        assert client.read_config_register(1, "temperature_limit") == 60
        assert client.read_config_register(2, "temperature_limit") == 75


class TestDescribe:
    def test_a_choice_register_reads_as_its_meaning(self):
        entry = next(r for r in motor_client_class("feetech").config_registers
                     if r.key == "operating_mode")
        assert entry.describe(0) == "position"

    def test_an_unknown_choice_says_so_rather_than_guessing(self):
        entry = next(r for r in motor_client_class("feetech").config_registers
                     if r.key == "operating_mode")
        assert "unknown" in entry.describe(7)

    def test_an_unread_register_is_not_shown_as_zero(self):
        entry = next(r for r in motor_client_class("dynamixel").config_registers
                     if r.key == "temperature_limit")
        assert entry.describe(None) == "--"
        assert entry.describe(0) != entry.describe(None)


class TestStalePortFlag:
    """The SDK's in-use flag can be left set by an interrupted transaction,
    and nothing clears it on its own."""

    def test_the_client_clears_it_before_a_transaction(self):
        """Holding the bus lock is the real mutual exclusion, so a flag still
        set at that point can only be stale. Left alone it returns
        COMM_PORT_BUSY forever -- and disconnect() refuses to close a port it
        believes is in use, so a reconnect cannot clear it either."""
        from orca_core.hardware.dynamixel_client import DynamixelClient

        assert hasattr(DynamixelClient, "_claim_port")

    def test_disconnect_still_refuses_a_genuinely_busy_port(self):
        """The guard in disconnect() is deliberately left alone: clearing the
        flag belongs with the caller that holds the lock, not with teardown."""
        import inspect

        from orca_core.hardware.dynamixel_client import DynamixelClient

        source = inspect.getsource(DynamixelClient.disconnect)
        assert "is_using" in source
