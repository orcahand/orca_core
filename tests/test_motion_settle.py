"""Arrival is a whole-chain condition, not a per-motor one.

A recorded motion is implicitly synchronous: letting one motor start its
next leg because it arrived first, while others are still travelling, is
not a faster version of the same move but a different one.
"""

import numpy as np
import pytest

from orca_core.hardware.motor_client import MotionTimeoutError, MotorClient
from orca_core.hardware.motor_factory import (
    mock_motor_client_class,
    motor_client_class,
)


def test_every_shipped_family_actually_waits():
    """A goal write is instantaneous only while the trajectory profile is
    zero. Once a profile is set the servo ramps, so no family can claim its
    motion is too fast to wait for."""
    for motor_type in ("dynamixel", "feetech"):
        assert motor_client_class(motor_type).waits_for_motion is True


def test_the_mocks_make_the_same_claim():
    for motor_type in ("dynamixel", "feetech"):
        real = motor_client_class(motor_type)
        mock = mock_motor_client_class(motor_type)
        assert mock.waits_for_motion == real.waits_for_motion
        assert mock.arrival_tolerance_rad == real.arrival_tolerance_rad


def test_a_tolerance_is_declared_and_is_coarser_than_one_encoder_count():
    """One count is about 0.0015 rad on both families; a tolerance at that
    scale would never be met on a real servo sitting in its dead zone."""
    assert MotorClient.arrival_tolerance_rad > 0.0015
    assert MotorClient.arrival_tolerance_rad < 0.1


class _Reader:
    def __init__(self, values):
        self.values = values

    def read(self):
        return self.values


class TestDynamixelArrival:
    """The flag alone lies in one direction and position alone in the other,
    so both are required."""

    def _client(self, moving, positions, goals):
        cls = motor_client_class("dynamixel")
        client = cls.__new__(cls)
        client.motor_ids = [1, 2]
        client._moving_reader = _Reader(np.array(moving))
        client._pos_vel_cur_reader = _Reader(
            (np.array(positions), np.zeros(2), np.zeros(2)))
        client._goal_positions = dict(goals)
        return client

    def test_a_motor_still_moving_is_unsettled(self):
        client = self._client([1, 0], [0.0, 0.0], {1: 0.0, 2: 0.0})
        assert client._unsettled_motors() == [1]

    def test_a_motor_stopped_short_of_its_goal_is_unsettled(self):
        """A servo stalled against a load clears its moving flag while short
        of the target. Advancing on that is the desynchronisation."""
        client = self._client([0, 0], [0.0, 0.0], {1: 0.0, 2: 1.0})
        assert client._unsettled_motors() == [2]

    def test_everything_stopped_and_in_tolerance_is_settled(self):
        client = self._client([0, 0], [0.0, 1.0], {1: 0.0, 2: 1.0})
        assert client._unsettled_motors() == []

    def test_within_tolerance_counts_as_arrived(self):
        tol = motor_client_class("dynamixel").arrival_tolerance_rad
        client = self._client([0, 0], [tol * 0.9, 0.0], {1: 0.0, 2: 0.0})
        assert client._unsettled_motors() == []

    def test_a_motor_never_commanded_here_is_judged_on_its_flag_alone(self):
        """Nothing to compare against is not the same as being wrong."""
        client = self._client([0, 0], [5.0, 5.0], {})
        assert client._unsettled_motors() == []

    def test_a_failed_poll_is_not_read_as_arrival(self):
        """Losing the bus mid-move must not look like the chain settled."""
        client = self._client([0, 0], [0.0, 0.0], {1: 0.0})

        def boom():
            raise OSError("bus")

        client._moving_reader.read = boom
        assert client._unsettled_motors() == [1, 2]

    def test_the_timeout_names_what_never_settled(self):
        client = self._client([1, 0], [0.0, 0.0], {1: 0.0, 2: 0.0})
        client.check_connected = lambda: None
        with pytest.raises(MotionTimeoutError) as caught:
            client.wait_for_motion_complete(timeout=0.05, poll_interval=0.01)
        assert "1" in str(caught.value)


class TestServoLimits:
    """A front-end cannot infer these: register widths and units differ by
    family, and zero means opposite things depending on the tunable."""

    def test_both_families_report_every_tunable(self):
        for motor_type in ("dynamixel", "feetech"):
            limits = motor_client_class(motor_type).servo_limits()
            assert set(limits) == {"gain", "velocity_rad_s",
                                   "acceleration_rad_s2"}
            for entry in limits.values():
                assert entry["max"] is not None
                assert entry["zero_means"]

    def test_the_families_really_do_differ(self):
        """If they agreed, reporting them would be pointless."""
        dxl = motor_client_class("dynamixel").servo_limits()
        fee = motor_client_class("feetech").servo_limits()
        assert dxl["gain"]["max"] != fee["gain"]["max"]
        assert (dxl["acceleration_rad_s2"]["max"]
                != fee["acceleration_rad_s2"]["max"])

    def test_zero_velocity_is_documented_as_no_cap_everywhere(self):
        """The one place the families disagree at the register level: a
        literal zero stops a Feetech. The client translates, so the reported
        meaning stays the same on both."""
        for motor_type in ("dynamixel", "feetech"):
            limits = motor_client_class(motor_type).servo_limits()
            assert limits["velocity_rad_s"]["zero_means"] == "no speed cap"

    def test_the_mocks_report_the_same_limits(self):
        for motor_type in ("dynamixel", "feetech"):
            assert (mock_motor_client_class(motor_type).servo_limits()
                    == motor_client_class(motor_type).servo_limits())


class TestFeetechProfileTranslation:
    """Goal speed 0 stops this family rather than uncapping it."""

    def _client(self):
        cls = mock_motor_client_class("feetech")
        client = cls([1])
        client.connect()
        return client

    def test_asking_for_no_cap_does_not_stop_the_motor(self):
        from orca_core.hardware.motor_client import ServoProfile

        client = self._client()
        client.write_servo_profile({1: ServoProfile(velocity_rad_s=0.0)})

        assert client.read_servo_profile([1])[1].velocity_rad_s == 0.0

    def test_a_real_cap_survives_the_round_trip(self):
        from orca_core.hardware.motor_client import ServoProfile

        client = self._client()
        client.write_servo_profile({1: ServoProfile(velocity_rad_s=2.0)})

        assert client.read_servo_profile([1])[1].velocity_rad_s == pytest.approx(2.0)

    def test_one_field_at_a_time_leaves_the_other_alone(self):
        from orca_core.hardware.motor_client import ServoProfile

        client = self._client()
        before = client.read_servo_profile([1])[1]
        client.write_servo_profile({1: ServoProfile(velocity_rad_s=3.0)})
        after = client.read_servo_profile([1])[1]

        assert after.acceleration_rad_s2 == before.acceleration_rad_s2

    def test_a_negative_limit_is_refused(self):
        from orca_core.hardware.motor_client import ServoProfile

        client = self._client()
        with pytest.raises(ValueError):
            client.write_servo_profile({1: ServoProfile(velocity_rad_s=-1.0)})


class _FakeSyncReader:
    """Stands in for GroupSyncRead over the limit block.

    ``answers`` maps a motor id to ``{address: raw}``; a motor absent from it
    is one that did not reply.
    """

    def __init__(self, answers, comm_ok=True):
        self.answers = answers
        self.comm_ok = comm_ok
        self.ids = []

    def addParam(self, mid):
        self.ids.append(mid)
        return True

    def txRxPacket(self):
        return 0 if self.comm_ok else -1

    def isAvailable(self, mid, address, length):
        return mid in self.answers

    def getData(self, mid, address, length):
        return self.answers[mid][address]

    def clearParam(self):
        self.ids.clear()


class TestDynamixelProfileCeiling:
    """Velocity Limit, not the register width, is what the servo honours.

    Hardware confirms the register does not clamp: Profile Velocity accepts
    and reads back 32767 on a motor whose Velocity Limit is 320. A front-end
    that trusts the register width offers a speed two orders of magnitude
    past anything the motor can do.
    """

    ADDR_ACC_LIMIT = 40
    ADDR_VEL_LIMIT = 44

    def _client(self, answers, comm_ok=True):
        import threading

        cls = motor_client_class("dynamixel")
        client = cls.__new__(cls)
        client.motor_ids = [1, 2]
        client._bus_lock = threading.RLock()
        client.port_handler = object()
        client.packet_handler = object()
        client._flush_input_buffer = lambda: None
        reader = _FakeSyncReader(answers, comm_ok)

        class _Dxl:
            COMM_SUCCESS = 0

            @staticmethod
            def GroupSyncRead(*_args):
                return reader

        client.dxl = _Dxl()
        return client

    def _limits(self, acc_raw, vel_raw, **kw):
        answers = {mid: {self.ADDR_ACC_LIMIT: acc_raw,
                         self.ADDR_VEL_LIMIT: vel_raw} for mid in (1, 2)}
        return self._client(answers, **kw).read_profile_limits([1, 2])

    def test_the_ceiling_is_far_below_the_register_width(self):
        """The numbers a real chain of these motors reports."""
        cls = motor_client_class("dynamixel")
        limits = self._limits(acc_raw=0, vel_raw=320)

        assert limits[1].velocity_rad_s == pytest.approx(7.674, abs=0.01)
        assert limits[1].velocity_rad_s < cls.profile_velocity_max_rad_s / 100

    def test_the_wrist_and_a_finger_may_disagree(self):
        """Which is why this is per motor and not a family constant."""
        answers = {1: {self.ADDR_ACC_LIMIT: 32767, self.ADDR_VEL_LIMIT: 306},
                   2: {self.ADDR_ACC_LIMIT: 0, self.ADDR_VEL_LIMIT: 320}}
        limits = self._client(answers).read_profile_limits([1, 2])

        assert limits[1].velocity_rad_s == pytest.approx(7.338, abs=0.01)
        assert limits[2].velocity_rad_s == pytest.approx(7.674, abs=0.01)

    def test_a_zero_limit_means_unbounded_not_stopped(self):
        """Opposite to what zero means in a goal-speed register, so reading it
        as "cannot move" would silently forbid every acceleration."""
        cls = motor_client_class("dynamixel")
        limits = self._limits(acc_raw=0, vel_raw=0)

        assert limits[1].velocity_rad_s == cls.no_load_speed_rad_s
        assert limits[1].acceleration_rad_s2 == cls.profile_acceleration_max_rad_s2

    def test_a_silent_bus_does_not_narrow_the_range(self):
        """A dropped packet must not be read as a slow motor: that would
        quietly cap a move the hardware would have allowed."""
        cls = motor_client_class("dynamixel")
        limits = self._limits(acc_raw=0, vel_raw=320, comm_ok=False)

        assert limits[1].velocity_rad_s == cls.no_load_speed_rad_s

    def test_a_motor_that_skips_its_turn_keeps_the_width(self):
        answers = {2: {self.ADDR_ACC_LIMIT: 0, self.ADDR_VEL_LIMIT: 320}}
        cls = motor_client_class("dynamixel")
        limits = self._client(answers).read_profile_limits([1, 2])

        assert limits[1].velocity_rad_s == cls.no_load_speed_rad_s
        assert limits[2].velocity_rad_s == pytest.approx(7.674, abs=0.01)

    def test_no_motors_is_not_a_bus_transaction(self):
        assert self._client({}).read_profile_limits([]) == {}


class TestFeetechProfileCeiling:
    """This family publishes no limit register, so the honest answer is the
    register width plus a statement that nothing enforces it."""

    def test_it_reports_a_speed_the_motor_can_reach(self):
        """A datasheet speed, not the 2512 rad/s the register holds."""
        cls = mock_motor_client_class("feetech")
        client = cls([1])
        client.connect()
        limits = client.read_profile_limits([1])

        assert limits[1].velocity_rad_s < cls.profile_velocity_max_rad_s / 100
        assert "no limit register" in cls.profile_ceiling_source

    def test_the_ceiling_follows_the_model_each_motor_reports(self):
        """One chain carries models whose no-load speeds differ by well over
        a factor of two, so a family constant would either not cap the slow
        motor or needlessly throttle the fast one."""
        from orca_core.hardware.feetech_client import FEETECH_NO_LOAD_RPM

        cls = motor_client_class("feetech")
        client = cls.__new__(cls)
        client.motor_ids = [1, 2]
        client._model_numbers = {1: 4106, 2: 6922}

        assert client.no_load_speed_rad_s_for(1) == pytest.approx(4.712, abs=0.01)
        assert client.no_load_speed_rad_s_for(2) == pytest.approx(10.472, abs=0.01)
        assert len(set(FEETECH_NO_LOAD_RPM.values())) > 1

    def test_a_motor_that_will_not_name_itself_gets_the_slowest(self):
        """The two errors are not symmetric: too low is merely sluggish, too
        high is indistinguishable from no cap at all."""
        from orca_core.hardware.feetech_client import FEETECH_NO_LOAD_RPM

        cls = motor_client_class("feetech")
        client = cls.__new__(cls)
        client.motor_ids = [1]
        client._model_numbers = {}

        assert client.no_load_speed_rad_s_for(1) == pytest.approx(
            min(FEETECH_NO_LOAD_RPM.values()) * 2 * np.pi / 60, abs=0.01)

    def test_the_two_families_do_not_claim_the_same_source(self):
        assert (motor_client_class("dynamixel").profile_ceiling_source
                != motor_client_class("feetech").profile_ceiling_source)


class TestDefaultProfile:
    """A default has to be defensible on every motor it lands on, and the
    motors differ, so it is derived per motor rather than chosen once."""

    def test_it_is_half_of_what_the_motor_can_do(self):
        for motor_type in ("dynamixel", "feetech"):
            cls = mock_motor_client_class(motor_type)
            client = cls([1])
            client.connect()
            ceiling = client.read_profile_limits([1])[1].velocity_rad_s
            default = client.default_profile([1])[1]

            assert default.velocity_rad_s == pytest.approx(ceiling / 2, abs=0.01)

    def test_the_default_is_reachable_on_both_families(self):
        """The point of the exercise: a default above the ceiling is silently
        clamped and is indistinguishable from asking for no cap at all."""
        for motor_type in ("dynamixel", "feetech"):
            cls = mock_motor_client_class(motor_type)
            client = cls([1])
            client.connect()
            default = client.default_profile([1])[1]

            assert 0 < default.velocity_rad_s < client.read_profile_limits([1])[1].velocity_rad_s
            assert default.velocity_rad_s < 20.0

    def test_acceleration_is_a_ramp_not_a_share_of_the_range(self):
        """The two families' acceleration ranges differ by a factor of 300, so
        the same fraction would mean two unrelated things; the same ramp does
        not, and it has to fit inside the smaller family's register."""
        dxl = motor_client_class("dynamixel")
        fee = motor_client_class("feetech")

        assert dxl.default_profile_acceleration_rad_s2 == fee.default_profile_acceleration_rad_s2
        assert dxl.default_profile_acceleration_rad_s2 < fee.profile_acceleration_max_rad_s2

    def test_the_ramp_reaches_the_default_speed_promptly(self):
        """A default that takes seconds to come up to speed is a default
        nobody would keep."""
        cls = mock_motor_client_class("dynamixel")
        client = cls([1])
        client.connect()
        default = client.default_profile([1])[1]

        assert default.velocity_rad_s / default.acceleration_rad_s2 < 0.6


class TestFeetechPerMotorProfile:
    """Each motor keeps its own profile. A single chain-wide acceleration
    meant setting one motor's quietly set every motor's."""

    def _client(self, ids):
        cls = mock_motor_client_class("feetech")
        client = cls(ids)
        client.connect()
        return client

    def test_setting_one_motors_acceleration_leaves_the_others(self):
        from orca_core.hardware.motor_client import ServoProfile

        client = self._client([1, 2, 3])
        before = client.read_servo_profile([2])[2].acceleration_rad_s2
        client.write_servo_profile({1: ServoProfile(acceleration_rad_s2=5.0)})
        after = client.read_servo_profile([1, 2, 3])

        assert after[1].acceleration_rad_s2 == pytest.approx(5.0, abs=0.2)
        assert after[2].acceleration_rad_s2 == pytest.approx(before)
        assert after[3].acceleration_rad_s2 == pytest.approx(before)

    def test_motors_can_hold_different_accelerations_at_once(self):
        from orca_core.hardware.motor_client import ServoProfile

        client = self._client([1, 2])
        client.write_servo_profile({
            1: ServoProfile(acceleration_rad_s2=5.0),
            2: ServoProfile(acceleration_rad_s2=20.0),
        })
        profile = client.read_servo_profile([1, 2])

        assert profile[1].acceleration_rad_s2 != profile[2].acceleration_rad_s2
