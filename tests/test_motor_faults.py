"""Fault policy: what each family's faults do, and what gets done about them."""

import pytest

from orca_core.hardware.mock_dynamixel_client import MockDynamixelClient
from orca_core.hardware.mock_feetech_client import MockFeetechClient
from orca_core.hardware.motor_faults import (
    DISABLE_TORQUE,
    ENABLE_TORQUE,
    HOLD_OFF,
    RELEASE,
    EXCESS_LOAD,
    FaultPolicyConfig,
    LOG,
    MotorFaultPolicy,
    MotorObservation,
    NOTIFY,
    REBOOT,
)

# What each family actually ships, read off hardware.
DXL_MASK = 52      # overheating + electrical shock + overload
FEETECH_MASK = 13  # voltage + temperature + current


def _client(cls):
    client = cls([1, 2], port="mock", baudrate=1000000)
    client.connect()
    return client


def _obs(raw, mask, **kwargs):
    base = dict(motor_id=1, raw_error=raw, stop_mask=mask, joint="index_pip",
                 temperature_c=40.0,
                temperature_limit_c=80.0, torque_enabled=True)
    base.update(kwargs)
    return MotorObservation(**base)


def _kinds(actions):
    return [action.kind for action in actions]


class TestWhetherTheMotorStopped:
    """The split between "it quit" and "it carried on and told us"."""

    def test_an_overload_that_stops_a_dynamixel_is_recovered(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            actions = policy.observe([_obs(0x20, DXL_MASK)], now=0.0)
            assert _kinds(actions) == [LOG, REBOOT, ENABLE_TORQUE]
            assert policy.record(1).reboots == 1
        finally:
            client.disconnect()

    def test_an_overload_that_does_not_stop_a_feetech_is_only_logged(self):
        """Feetech ships overload out of its unload mask, so the motor keeps
        driving. There is nothing to recover and a reboot would interrupt a
        working joint."""
        client = _client(MockFeetechClient)
        try:
            policy = MotorFaultPolicy(client)
            actions = policy.observe([_obs(0x20, FEETECH_MASK)], now=0.0)
            assert _kinds(actions) == [LOG]
            assert policy.record(1).reboots == 0
            # Still recorded, so a joint stalling repeatedly is visible.
            assert policy.record(1).count(EXCESS_LOAD) == 1
        finally:
            client.disconnect()

    def test_a_feetech_overcurrent_does_stop_it_and_is_recovered(self):
        """Same canonical condition as the overload above, opposite handling,
        decided by the register rather than the family."""
        client = _client(MockFeetechClient)
        try:
            policy = MotorFaultPolicy(client)
            actions = policy.observe([_obs(0x08, FEETECH_MASK)], now=0.0)
            assert _kinds(actions) == [LOG, REBOOT, ENABLE_TORQUE]
        finally:
            client.disconnect()

    def test_an_unreadable_mask_never_recovers_anything(self):
        """Under-reacting beats rebooting a motor that is still driving."""
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            actions = policy.observe([_obs(0x20, None)], now=0.0)
            assert _kinds(actions) == [LOG]
        finally:
            client.disconnect()


class TestEscalation:
    def _drive(self, policy, count, mask=DXL_MASK, start=0.0, step=10.0):
        seen = []
        now = start
        for _ in range(count):
            seen.append(policy.observe([_obs(0x20, mask)], now=now))
            policy.record(1).last_raw = 0  # the reboot cleared it
            now += step
        return seen

    def test_the_third_reboot_is_reported_prominently(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            rounds = self._drive(policy, 3)
            assert NOTIFY not in _kinds(rounds[0])
            assert NOTIFY not in _kinds(rounds[1])
            third = [a for a in rounds[2] if a.kind == NOTIFY]
            assert third and third[0].prominent
        finally:
            client.disconnect()

    def test_after_five_reboots_the_motor_is_left_latched(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            rounds = self._drive(policy, 7)
            assert policy.record(1).reboots == 5
            assert REBOOT not in _kinds(rounds[5])
            assert REBOOT not in _kinds(rounds[6])
            assert policy.record(1).given_up is True
        finally:
            client.disconnect()

    def test_both_thresholds_are_configurable(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(
                client, FaultPolicyConfig(report_after_reboots=1,
                                          giveup_after_reboots=2))
            rounds = self._drive(policy, 3)
            assert NOTIFY in _kinds(rounds[0])
            assert policy.record(1).reboots == 2
            assert REBOOT not in _kinds(rounds[2])
        finally:
            client.disconnect()

    def test_a_persisting_latch_is_one_fault_not_one_per_poll(self):
        """The flag stays set until cleared, and the sweep sees it every tick.
        Counting polls would exhaust the reboot budget in a second."""
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            policy.observe([_obs(0x20, DXL_MASK)], now=0.0)
            for tick in range(1, 20):
                policy.observe([_obs(0x20, DXL_MASK)], now=tick * 0.05)
            assert policy.record(1).reboots == 1
            assert len(policy.record(1).events) == 1
        finally:
            client.disconnect()

    def test_refaulting_inside_the_floor_waits(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            policy.observe([_obs(0x20, DXL_MASK)], now=0.0)
            policy.record(1).last_raw = 0
            actions = policy.observe([_obs(0x20, DXL_MASK)], now=0.5)
            assert REBOOT not in _kinds(actions)
            assert policy.record(1).reboots == 1
        finally:
            client.disconnect()

    def test_torque_is_not_restored_if_it_was_off(self):
        """A reboot leaves torque off deliberately; recovery restores what the
        operator had, so a relaxed hand is never silently energized."""
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            actions = policy.observe(
                [_obs(0x20, DXL_MASK, torque_enabled=False)], now=0.0)
            assert ENABLE_TORQUE not in _kinds(actions)
        finally:
            client.disconnect()


class TestThermal:
    def _cool_to(self, policy, temps, mask=DXL_MASK, raw=0x04, start=0.0):
        rounds = []
        now = start
        for temp in temps:
            rounds.append(policy.observe(
                [_obs(raw, mask, temperature_c=temp, torque_enabled=False)],
                now=now))
            now += 10.0
        return rounds

    def test_an_overheated_motor_waits_instead_of_rebooting_hot(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            rounds = self._cool_to(policy, [81.0, 75.0, 71.0])
            assert REBOOT not in _kinds(rounds[0])
            assert policy.record(1).cooling is True
            assert all(REBOOT not in _kinds(r) for r in rounds)
        finally:
            client.disconnect()

    def test_it_recovers_once_it_is_the_margin_below_its_limit(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            rounds = self._cool_to(policy, [81.0, 75.0, 69.0])
            assert _kinds(rounds[2])[:2] == [RELEASE, REBOOT]
            assert policy.record(1).cooling is False
            assert policy.record(1).thermal_recoveries == 1
        finally:
            client.disconnect()

    def test_the_margin_is_configurable(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(
                client, FaultPolicyConfig(thermal_clear_margin_c=2.0))
            rounds = self._cool_to(policy, [81.0, 79.0, 77.0])
            # 79 is still above 80 - 2, so only 77 clears it.
            assert REBOOT not in _kinds(rounds[1])
            assert REBOOT in _kinds(rounds[2])
        finally:
            client.disconnect()

    def test_a_hot_motor_is_clamped_to_zero_current(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            actions = policy.observe(
                [_obs(0x04, DXL_MASK, temperature_c=81.0, torque_enabled=True)],
                now=0.0)
            assert HOLD_OFF in _kinds(actions)
        finally:
            client.disconnect()

    def test_thermal_recovery_does_not_spend_the_load_budget(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            self._cool_to(policy, [81.0, 69.0])
            assert policy.record(1).reboots == 0
        finally:
            client.disconnect()

    def test_cooling_without_a_temperature_reading_just_waits(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            policy.observe([_obs(0x04, DXL_MASK, temperature_c=81.0)], now=0.0)
            actions = policy.observe(
                [_obs(0x04, DXL_MASK, temperature_c=None)], now=10.0)
            assert actions == []
            assert policy.record(1).cooling is True
        finally:
            client.disconnect()


class TestWhatGetsRecorded:
    def test_every_fault_records_cause_and_timing(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            policy.observe([_obs(0x20, DXL_MASK)], now=0.0)
            policy.record(1).last_raw = 0x20
            policy.observe([_obs(0x00, DXL_MASK)], now=12.0)

            event = policy.record(1).events[0]
            assert event.conditions == {EXCESS_LOAD}
            assert event.flag_names == {"overload"}
            assert event.stopped is True
            assert event.joint == "index_pip"
            assert event.at > 0
            assert event.duration_s == pytest.approx(12.0)
        finally:
            client.disconnect()

    def test_a_silent_motor_changes_nothing(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            assert policy.observe([_obs(None, DXL_MASK)], now=0.0) == []
            assert policy.records == {}
        finally:
            client.disconnect()


class TestTheTwoBudgetsAreSeparate:
    def test_thermal_recovery_is_unlimited_by_default(self):
        """A motor that cooled can work again, and the wait is self-limiting:
        it does not come back until it has cooled."""
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(client)
            now = 0.0
            for _ in range(8):
                policy.observe([_obs(0x04, DXL_MASK, temperature_c=81.0,
                                     torque_enabled=False)], now=now)
                now += 10.0
                policy.observe([_obs(0x04, DXL_MASK, temperature_c=60.0,
                                     torque_enabled=False)], now=now)
                now += 10.0
                policy.record(1).last_raw = 0
            record = policy.record(1)
            assert record.thermal_recoveries == 8
            assert record.thermal_given_up is False
            # Load budget untouched by any of it.
            assert record.reboots == 0
        finally:
            client.disconnect()

    def test_a_thermal_budget_can_be_set(self):
        client = _client(MockDynamixelClient)
        try:
            policy = MotorFaultPolicy(
                client, FaultPolicyConfig(thermal_giveup_after_reboots=2))
            now = 0.0
            for _ in range(4):
                policy.observe([_obs(0x04, DXL_MASK, temperature_c=81.0,
                                     torque_enabled=False)], now=now)
                now += 10.0
                policy.observe([_obs(0x04, DXL_MASK, temperature_c=60.0,
                                     torque_enabled=False)], now=now)
                now += 10.0
                policy.record(1).last_raw = 0
            assert policy.record(1).thermal_recoveries == 2
            assert policy.record(1).thermal_given_up is True
        finally:
            client.disconnect()


class TestCoolingHoldsTheMotorOff:
    def test_a_motor_that_re_energizes_itself_while_cooling_is_held_off(self):
        """Feetech's thermal protection tracks the live condition with about a
        degree of hysteresis, so the motor comes back on its own just under
        the limit and chatters there instead of cooling. The margin only
        means something if it is enforced."""
        client = _client(MockFeetechClient)
        try:
            policy = MotorFaultPolicy(client)
            # Overheats at 81 against an 80 limit and stops itself.
            policy.observe([_obs(0x04, FEETECH_MASK, temperature_c=81.0,
                                 torque_enabled=False)], now=0.0)
            assert policy.record(1).cooling is True

            # Drops a degree and the hardware re-energizes it.
            actions = policy.observe([_obs(0x04, FEETECH_MASK,
                                           temperature_c=79.0,
                                           torque_enabled=True)], now=10.0)
            assert _kinds(actions) == [HOLD_OFF]
            assert policy.record(1).cooling is True

            # Still held off well above the margin.
            actions = policy.observe([_obs(0x04, FEETECH_MASK,
                                           temperature_c=74.0,
                                           torque_enabled=True)], now=20.0)
            assert _kinds(actions) == [HOLD_OFF]

            # Margin met: cleared and brought back.
            actions = policy.observe([_obs(0x04, FEETECH_MASK,
                                           temperature_c=69.0,
                                           torque_enabled=True)], now=30.0)
            assert REBOOT in _kinds(actions)
            assert policy.record(1).cooling is False
        finally:
            client.disconnect()
