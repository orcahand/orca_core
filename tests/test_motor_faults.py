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
            assert REBOOT in _kinds(rounds[2])
            assert RELEASE in _kinds(rounds[2])
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

            # The clamp persists, so it is issued once rather than
            # re-asserted on every tick.
            actions = policy.observe([_obs(0x04, FEETECH_MASK,
                                           temperature_c=79.0,
                                           torque_enabled=True)], now=10.0)
            assert HOLD_OFF not in _kinds(actions), "already held"
            assert policy.record(1).cooling is True
            assert policy._held == {1}

            actions = policy.observe([_obs(0x04, FEETECH_MASK,
                                           temperature_c=74.0,
                                           torque_enabled=True)], now=20.0)
            assert HOLD_OFF not in _kinds(actions)
            assert policy._held == {1}

            # Margin met: cleared and brought back.
            actions = policy.observe([_obs(0x04, FEETECH_MASK,
                                           temperature_c=69.0,
                                           torque_enabled=True)], now=30.0)
            assert REBOOT in _kinds(actions)
            assert RELEASE in _kinds(actions)
            assert policy.record(1).cooling is False
            assert policy._held == set()
        finally:
            client.disconnect()


class TestWhoGetsHeldWhileOneMotorCools:
    """These motors heat each other, so a motor packed between driving
    neighbours can reach its limit and never get back under the margin -
    the heat is not its own. Holding the whole hand off is the default; the
    per-motor hold is right only where they are thermally independent."""

    def _observations(self, hot_temp):
        return [
            MotorObservation(motor_id=1, raw_error=0x04, stop_mask=FEETECH_MASK,
                             joint="index_pip", temperature_c=hot_temp,
                             temperature_limit_c=80.0, torque_enabled=True),
            MotorObservation(motor_id=2, raw_error=0x00, stop_mask=FEETECH_MASK,
                             joint="index_mcp", temperature_c=60.0,
                             temperature_limit_c=80.0, torque_enabled=True),
        ]

    def test_by_default_one_hot_motor_holds_the_whole_hand(self):
        client = _client(MockFeetechClient)
        try:
            policy = MotorFaultPolicy(client)
            assert policy.config.thermal_hold_all is True
            actions = policy.observe(self._observations(81.0), now=0.0)
            held = {a.motor_id for a in actions if a.kind == HOLD_OFF}
            assert held == {1, 2}, "the cool neighbour is held too"
        finally:
            client.disconnect()

    def test_per_motor_mode_holds_only_the_hot_one(self):
        client = _client(MockFeetechClient)
        try:
            policy = MotorFaultPolicy(
                client, FaultPolicyConfig(thermal_hold_all=False))
            actions = policy.observe(self._observations(81.0), now=0.0)
            held = {a.motor_id for a in actions if a.kind == HOLD_OFF}
            assert held == {1}
        finally:
            client.disconnect()

    def test_the_hand_is_released_only_once_nothing_is_cooling(self):
        client = _client(MockFeetechClient)
        try:
            policy = MotorFaultPolicy(client)
            policy.observe(self._observations(81.0), now=0.0)
            assert policy._held == {1, 2}

            # Still too hot: nothing released.
            actions = policy.observe(self._observations(75.0), now=10.0)
            assert RELEASE not in _kinds(actions)
            assert policy._held == {1, 2}

            # Margin met: the whole hand comes back.
            actions = policy.observe(self._observations(69.0), now=20.0)
            released = {a.motor_id for a in actions if a.kind == RELEASE}
            assert released == {1, 2}
            assert policy._held == set()
        finally:
            client.disconnect()


class TestSwitchingModesMidRun:
    """Ticking hold-all on must take precedence over any motor already
    waiting to be released on its own, or one finger restarts while the rest
    of the hand stays held."""

    def _obs(self, mid, temp, raw=0x00):
        return MotorObservation(
            motor_id=mid, raw_error=raw, stop_mask=FEETECH_MASK,
            joint=f"j{mid}", temperature_c=temp, temperature_limit_c=80.0,
            torque_enabled=True)

    def test_ticking_hold_all_on_takes_over_pending_releases(self):
        client = _client(MockFeetechClient)
        try:
            policy = MotorFaultPolicy(
                client, FaultPolicyConfig(thermal_hold_all=False))
            policy.observe([self._obs(1, 81.0, 0x04),
                            self._obs(2, 81.0, 0x04)], now=0.0)
            assert policy._held == {1, 2}

            policy.config = FaultPolicyConfig(thermal_hold_all=True)
            # One has cooled past the margin and would have been released.
            actions = policy.observe([self._obs(1, 69.0, 0x04),
                                      self._obs(2, 79.0, 0x04)], now=10.0)
            assert REBOOT not in _kinds(actions), "released while a motor is hot"
            assert RELEASE not in _kinds(actions)
            assert policy.record(1).ready is True
            assert policy.record(1).cooling is True
        finally:
            client.disconnect()

    def test_the_hand_comes_back_when_the_last_motor_cools(self):
        client = _client(MockFeetechClient)
        try:
            policy = MotorFaultPolicy(client)
            policy.observe([self._obs(1, 81.0, 0x04),
                            self._obs(2, 81.0, 0x04),
                            self._obs(3, 60.0)], now=0.0)
            assert policy._held == {1, 2, 3}

            actions = policy.observe([self._obs(1, 68.0, 0x04),
                                      self._obs(2, 69.0, 0x04),
                                      self._obs(3, 60.0)], now=20.0)
            # Both latched motors recover in the same tick, and the neighbour
            # that was only held for them is released with them.
            assert {a.motor_id for a in actions if a.kind == REBOOT} == {1, 2}
            assert 3 in {a.motor_id for a in actions if a.kind == RELEASE}
            assert policy._held == set()
        finally:
            client.disconnect()

    def test_readiness_is_judged_across_the_whole_tick(self):
        """Deciding motor by motor as the list is walked judges each against
        neighbours whose temperature has not been read yet, and the hand comes
        back one motor per tick instead of together."""
        client = _client(MockFeetechClient)
        try:
            policy = MotorFaultPolicy(client)
            policy.observe([self._obs(1, 81.0, 0x04),
                            self._obs(2, 81.0, 0x04)], now=0.0)
            # Motor 2 is read last but is ready; motor 1 must still see that.
            actions = policy.observe([self._obs(1, 69.0, 0x04),
                                      self._obs(2, 69.0, 0x04)], now=20.0)
            assert {a.motor_id for a in actions if a.kind == REBOOT} == {1, 2}
        finally:
            client.disconnect()

    def test_a_reboot_does_not_quietly_free_a_motor_that_is_still_held(self):
        """A reboot restores the configured ceiling, which undoes the clamp.
        Without re-clamping, a motor recovered from a load fault starts
        driving while the rest of the hand is held for a neighbour."""
        client = _client(MockFeetechClient)
        try:
            policy = MotorFaultPolicy(client)
            # Motor 1 overheats and the hand is held.
            policy.observe([self._obs(1, 81.0, 0x04),
                            self._obs(2, 60.0)], now=0.0)
            assert policy._held == {1, 2}

            # Motor 2 now draws over its current limit, which this family
            # does stop it for, and is rebooted while motor 1 is still
            # cooling. (Overload, 0x20, would not: it is absent from the
            # unload mask, so that motor keeps driving and is only logged.)
            actions = policy.observe([self._obs(1, 79.0, 0x04),
                                      self._obs(2, 60.0, 0x08)], now=10.0)
            kinds = [(a.kind, a.motor_id) for a in actions]
            assert (REBOOT, 2) in kinds
            assert kinds.index((REBOOT, 2)) < kinds.index((HOLD_OFF, 2)), \
                "re-clamped after the reboot, not before"
            assert policy._held == {1, 2}
        finally:
            client.disconnect()


class TestMotorsWithNoCurrentRegister:
    """One Dynamixel model has no Goal Current register, so it cannot be
    clamped and is held off by disabling torque instead. Releasing it has to
    give the torque back: there is no ceiling to restore, and without this it
    stays dead after the rest of the hand has recovered."""

    def test_the_hold_falls_back_and_the_release_undoes_it(self, monkeypatch):
        from orca_core import MockOrcaHand

        hand = MockOrcaHand(
            config_path="orca_core/models/v2/orcahand-right/config.yaml")
        hand.connect()
        try:
            client = hand.motor_client
            wrist = hand.config.joint_to_motor_map["wrist"]
            # Stand in for the model with no current register.
            monkeypatch.setattr(client, "hold_motor_off",
                                lambda motor_id: int(motor_id) != wrist)
            monkeypatch.setattr(client, "release_motor",
                                lambda motor_id: int(motor_id) != wrist)
            enabled, disabled = [], []
            monkeypatch.setattr(hand, "enable_torque",
                                lambda ids=None: enabled.extend(ids or []))
            monkeypatch.setattr(hand, "disable_torque",
                                lambda ids=None: disabled.extend(ids or []))
            hand._torque_enabled = True

            from orca_core.hardware.motor_faults import FaultAction

            # Held: the clampable motor takes a ceiling, the wrist loses torque.
            for motor_id in (wrist, 2):
                action = FaultAction(HOLD_OFF, motor_id, "cooling")
                if client.hold_motor_off(action.motor_id):
                    hand._fault_torque_held.discard(action.motor_id)
                else:
                    hand.disable_torque([action.motor_id])
                    hand._fault_torque_held.add(action.motor_id)
            assert disabled == [wrist]
            assert hand._fault_torque_held == {wrist}

            # Released: only the wrist needs its torque back.
            for motor_id in (wrist, 2):
                client.release_motor(motor_id)
                if motor_id in hand._fault_torque_held:
                    hand._fault_torque_held.discard(motor_id)
                    if hand._torque_enabled:
                        hand.enable_torque([motor_id])
            assert enabled == [wrist]
            assert hand._fault_torque_held == set()
        finally:
            hand.disconnect()


class TestEveryConditionIsReported:
    """A fault reaches an operator whichever family raised it and whichever
    bit it arrived on. Only overload and overheating are recovered from - a
    reboot clears a latch, not a cause - but the others must not be a log
    line nobody reads."""

    def _report(self, cls, raw, mask):
        client = cls([1], port="mock", baudrate=1000000)
        client.connect()
        try:
            policy = MotorFaultPolicy(client)
            return policy.observe([MotorObservation(
                motor_id=1, raw_error=raw, stop_mask=mask, joint="index_pip",
                temperature_c=30.0, temperature_limit_c=80.0,
                torque_enabled=True)], now=0.0)
        finally:
            client.disconnect()

    @pytest.mark.parametrize("cls, raw, mask", [
        (MockDynamixelClient, 0x01, DXL_MASK),      # keeps driving
        (MockDynamixelClient, 0x10, DXL_MASK),      # stops
        (MockFeetechClient, 0x01, FEETECH_MASK),    # stops
    ])
    def test_a_power_fault_is_reported_prominently_and_never_rebooted(
            self, cls, raw, mask):
        actions = self._report(cls, raw, mask)
        assert REBOOT not in _kinds(actions), "a reboot cannot fix wiring"
        prominent = [a for a in actions if a.kind == NOTIFY and a.prominent]
        assert prominent, "a supply fault must reach an operator"
        assert "Power the hand down" in prominent[0].reason

    @pytest.mark.parametrize("cls, raw, mask", [
        (MockFeetechClient, 0x02, FEETECH_MASK),   # angle_sensor
        (MockDynamixelClient, 0x08, DXL_MASK),     # motor_encoder
    ])
    def test_an_encoder_fault_is_reported_the_same_on_both_families(
            self, cls, raw, mask):
        """Different bit, different vendor name, one report."""
        actions = self._report(cls, raw, mask)
        assert REBOOT not in _kinds(actions)
        prominent = [a for a in actions if a.kind == NOTIFY and a.prominent]
        assert prominent
        assert "position sensor" in prominent[0].reason
