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


def _unconnected_client():
    """A real client with a stand-in port handler, so teardown can be driven
    without hardware."""
    import threading

    from orca_core.hardware.dynamixel_client import DynamixelClient

    class FakePort:
        is_open = True
        is_using = False

        def __init__(self):
            self.closed = False

        def closePort(self):
            self.closed = True
            self.is_open = False

    client = DynamixelClient([1, 2])
    client.port_handler = FakePort()
    client.set_torque_enabled = lambda *a, **k: None
    DynamixelClient.OPEN_CLIENTS.add(client)
    return client, threading


class TestTeardownWithTheBusHeld:
    """A reconnect provoked by a bus transaction tears the client down while
    the bus is live. Which of the two teardowns is used decides whether a
    momentary condition becomes a permanent one."""

    def test_the_shared_teardown_gives_up_when_the_bus_looks_busy(self):
        """Why the new paths do not use it: the flag is read before the lock,
        so the answer may be a true one about another thread -- and the
        response is to return, leaving the port open and the client
        registered, with nothing scheduled to try again."""
        client, _ = _unconnected_client()
        client.port_handler.is_using = True

        client.disconnect()

        assert not client.port_handler.closed
        assert client in type(client).OPEN_CLIENTS
        type(client).OPEN_CLIENTS.discard(client)

    def test_the_fixed_order_closes_the_port_regardless(self):
        client, _ = _unconnected_client()
        client.port_handler.is_using = True

        client.disconnect_fixed_lock_order()

        assert client.port_handler.closed
        assert client not in type(client).OPEN_CLIENTS

    def test_it_waits_for_an_in_flight_transaction_instead_of_refusing(self):
        """The lock is the real mutual exclusion. Taking it first means the
        teardown queues behind a live exchange rather than reading its flag
        and declining, so no transaction is cut in half either."""
        client, threading = _unconnected_client()
        mid_transaction = threading.Event()
        released = threading.Event()
        closed_while_busy = []

        def transaction():
            with client._bus_lock:
                client.port_handler.is_using = True
                mid_transaction.set()
                released.wait(5)
                closed_while_busy.append(client.port_handler.closed)
                client.port_handler.is_using = False

        holder = threading.Thread(target=transaction)
        holder.start()
        assert mid_transaction.wait(5)

        teardown = threading.Thread(target=client.disconnect_fixed_lock_order)
        teardown.start()
        teardown.join(0.2)
        assert teardown.is_alive(), "teardown should be waiting on the bus lock"

        released.set()
        holder.join(5)
        teardown.join(5)

        assert closed_while_busy == [False]
        assert client.port_handler.closed
        assert client not in type(client).OPEN_CLIENTS

    def test_the_shared_teardown_is_left_exactly_as_it_was(self):
        """Changing the locking of a method every consumer calls is its own
        change. This branch only adds a path beside it."""
        import inspect

        from orca_core.hardware.dynamixel_client import DynamixelClient

        source = inspect.getsource(DynamixelClient.disconnect)
        assert "is_using" in source
        assert "cannot disconnect" in source

    def test_a_register_write_does_not_touch_the_in_use_flag(self):
        """The register paths hold the bus lock and leave the SDK's own
        bookkeeping to the SDK."""
        import inspect

        from orca_core.hardware.dynamixel_client import DynamixelClient

        for method in (DynamixelClient.read_config_register,
                       DynamixelClient.write_config_register):
            assert "is_using" not in inspect.getsource(method)


class TestEveryClientOffersTheFixedTeardown:
    """Both families read the in-use flag before taking the lock, so both get
    the twin, and the mocks carry it so a front-end exercising the path under
    test is exercising the same surface."""

    def test_real_and_mock_clients_all_have_it(self):
        from orca_core.hardware.motor_factory import (
            mock_motor_client_class,
            motor_client_class,
        )

        for motor_type in FAMILIES:
            for factory in (motor_client_class, mock_motor_client_class):
                assert hasattr(factory(motor_type), "disconnect_fixed_lock_order")

    def test_it_takes_the_lock_before_reading_any_flag(self):
        """The whole point. Reading first is what turns someone else's live
        transaction into a refusal to tear down."""
        import inspect

        from orca_core.hardware.motor_factory import motor_client_class

        for motor_type in FAMILIES:
            source = inspect.getsource(
                motor_client_class(motor_type).disconnect_fixed_lock_order)
            body = source[source.index('"""', source.index('"""') + 3):]
            assert "is_using" not in body
            assert body.index("_bus_lock") < body.index("closePort")


class TestUnits:
    """An operator works in the unit, never in register counts."""

    def test_return_delay_is_microseconds(self):
        from orca_core.hardware.motor_factory import motor_client_class

        entry = next(r for r in motor_client_class("dynamixel").config_registers
                     if r.key == "return_delay_time")
        assert entry.unit == "us"
        assert (entry.minimum, entry.maximum) == (0, 508)
        assert entry.to_raw(20) == 10
        assert entry.from_raw(10) == 20

    def test_a_write_in_microseconds_reads_back_in_microseconds(self):
        client = _client("dynamixel")
        assert client.write_config_register(1, "return_delay_time", 20) == 20
        assert client.read_config_register(1, "return_delay_time") == 20

    def test_a_value_the_register_cannot_hold_reports_what_it_became(self):
        """The register counts in twos, so 21 us is 20. Refusing it would be
        pedantic; saying nothing would be a lie."""
        client = _client("dynamixel")
        assert client.write_config_register(1, "return_delay_time", 21) == 20

    def test_the_bound_is_in_microseconds_not_register_counts(self):
        client = _client("dynamixel")
        with pytest.raises(ValueError):
            client.write_config_register(1, "return_delay_time", 509)
        assert client.write_config_register(1, "return_delay_time", 508) == 508

    def test_an_unscaled_register_is_unaffected(self):
        client = _client("dynamixel")
        assert client.write_config_register(1, "temperature_limit", 65) == 65


class TestWhatTheTransportCanCarry:
    """A bus-wide baud change is only safe to the rates the thing between host
    and motors will follow. That is a property of the transport, not of the
    motor family, so it cannot live in ``baud_rate_map``."""

    def test_no_limit_is_the_default(self):
        """So a plain adapter, a mock, and any third-party client all answer
        'the family's map is the only bound' without implementing anything."""
        from orca_core.hardware.motor_client import MotorClient
        from orca_core.hardware.motor_factory import mock_motor_client_class

        assert MotorClient.transport_baud_rates(object()) is None
        for motor_type in FAMILIES:
            assert _client(motor_type).transport_baud_rates() is None

    def test_both_real_clients_override_the_default(self):
        """They have a port to ask through, so they answer for real."""
        from orca_core.hardware.motor_client import MotorClient
        from orca_core.hardware.motor_factory import motor_client_class

        for motor_type in FAMILIES:
            assert (motor_client_class(motor_type).transport_baud_rates
                    is not MotorClient.transport_baud_rates)

    def test_the_board_cannot_carry_the_rates_probing_tools_open_at(self):
        """9600 and 115200 are the rates ModemManager and serial terminals pick
        by default. A board that followed them would retune a live bus, so it
        does not -- which also means a motor sent to one is stranded."""
        from orca_core.hardware.sensing.serial_discovery import (
            OH_BOARD_MOTOR_BAUD_RATES)

        assert 9600 not in OH_BOARD_MOTOR_BAUD_RATES
        assert 115200 not in OH_BOARD_MOTOR_BAUD_RATES
        assert 57600 in OH_BOARD_MOTOR_BAUD_RATES  # the factory default
        assert 1_000_000 in OH_BOARD_MOTOR_BAUD_RATES  # the hands' own rate

    def test_the_dynamixel_map_offers_three_rates_the_board_cannot_follow(self):
        """Which is what made a bus-wide change dangerous: the family map is a
        strict superset of what this transport can carry."""
        from orca_core.hardware.motor_factory import motor_client_class
        from orca_core.hardware.sensing.serial_discovery import (
            OH_BOARD_MOTOR_BAUD_RATES)

        family = set(motor_client_class("dynamixel").baud_rate_map)
        assert family - set(OH_BOARD_MOTOR_BAUD_RATES) == {
            9600, 115200, 10_500_000}

    def test_a_silent_link_reads_as_unconstrained(self):
        """A plain adapter does not answer the query, and the motors ignore it
        (no 0xFF header). Silence must mean 'no limit', not 'no rates'."""
        from orca_core.hardware.sensing.serial_discovery import (
            motor_baud_rates_over_link)

        class Silent:
            def reset_input_buffer(self): pass
            def write(self, data): pass
            def flush(self): pass
            def read(self, n): return b""

        assert motor_baud_rates_over_link(Silent(), timeout=0.01) is None

    def test_a_board_that_identifies_itself_reports_its_allowlist(self):
        from orca_core.hardware.sensing.serial_discovery import (
            OH_BOARD_MOTOR_BAUD_RATES,
            motor_baud_rates_over_link,
        )

        class Board:
            def reset_input_buffer(self): pass
            def write(self, data): pass
            def flush(self): pass
            def read(self, n): return b"ORCA:MOTOR\n"

        assert motor_baud_rates_over_link(Board()) == OH_BOARD_MOTOR_BAUD_RATES

    def test_an_unusable_link_does_not_raise(self):
        """The probe is incidental to whatever the caller was doing; it must not
        turn a closed port into an exception."""
        from orca_core.hardware.sensing.serial_discovery import (
            motor_baud_rates_over_link)

        assert motor_baud_rates_over_link(None) is None
