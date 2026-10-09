"""A signed goal current is a torque command, and is kept apart from the
goal-current *limit* a position mode runs under.

The limit is non-negative, clamped, and remembered so a reboot can restore the
ceiling. A torque command is signed, transient, and must never be remembered:
replaying a transient torque as a ceiling after a reboot would be a fault, not
a restoration.
"""

import math

import numpy as np
import pytest

from orca_core.constants import CURRENT, MODE_MAP, POSITION
from orca_core.hardware.motor_factory import mock_motor_client_class, motor_client_class


def _mock_dxl(ids=(1, 2)):
    client = mock_motor_client_class("dynamixel")(list(ids))
    client.connect()
    return client


class TestTheMockDynamixel:
    def test_a_signed_command_lands_with_its_sign(self):
        c = _mock_dxl()
        c.set_operating_mode([1, 2], MODE_MAP[CURRENT])
        c.write_goal_current_signed([1, 2], [120.0, -120.0])
        assert c._commanded_current_ma[1] == 120.0
        assert c._commanded_current_ma[2] == -120.0

    def test_magnitude_is_clamped_to_the_ceiling_keeping_the_sign(self):
        c = _mock_dxl()
        c.set_operating_mode([1], MODE_MAP[CURRENT])
        ceiling = c._current_ceiling_ma(1)
        c.write_goal_current_signed([1], [-(ceiling * 10)])
        assert c._commanded_current_ma[1] == -ceiling

    def test_refused_unless_the_motor_is_in_current_control_mode(self):
        """In any other mode the register is a ceiling, and a negative ceiling
        is nonsense the servo would read as a huge unsigned one."""
        c = _mock_dxl()
        c.set_operating_mode([1], MODE_MAP[POSITION])
        with pytest.raises(ValueError, match="not in current control mode"):
            c.write_goal_current_signed([1], [50.0])

    def test_refused_for_a_model_without_current_control(self):
        c = _mock_dxl()
        c.set_operating_mode([1], MODE_MAP[CURRENT])
        c._model_numbers = {1: 1080}   # an XC430: no current-control mode
        assert c.supports_current_control(1) is False
        with pytest.raises(ValueError, match="no current control mode"):
            c.write_goal_current_signed([1], [50.0])

    def test_non_finite_is_refused(self):
        c = _mock_dxl()
        c.set_operating_mode([1], MODE_MAP[CURRENT])
        with pytest.raises(ValueError, match="finite"):
            c.write_goal_current_signed([1], [math.nan])

    def test_the_limit_write_still_refuses_a_negative(self):
        """The two must stay distinct: the limit never grew a sign."""
        c = _mock_dxl()
        with pytest.raises(ValueError):
            c.write_desired_current([1], np.asarray([-50.0]))


class TestTheRealDynamixelClientWithoutABus:
    def _client(self):
        client = motor_client_class("dynamixel")([1])
        client._operating_modes = {1: MODE_MAP[CURRENT]}
        client._model_numbers = {1: 1220}
        return client

    def test_the_raw_value_is_signed_and_goes_to_the_goal_current_register(self, monkeypatch):
        from orca_core.hardware import dynamixel_client as dxl
        client = self._client()
        writes = []
        monkeypatch.setattr(client, "sync_write",
                            lambda ids, values, addr, size: writes.append((list(ids), list(values), addr, size)))
        client.write_goal_current_signed([1], [-150.0])
        assert writes == [([1], [-150], dxl.ADDR_GOAL_CURRENT, dxl.LEN_GOAL_CURRENT)]

    def test_a_torque_command_is_never_remembered_as_the_ceiling(self, monkeypatch):
        """_remember_ram is what lets a reboot restore the limit; a torque
        command going through it would be replayed as a ceiling later."""
        client = self._client()
        monkeypatch.setattr(client, "sync_write", lambda *a, **k: None)
        monkeypatch.setattr(client, "_remember_ram",
                            lambda *a, **k: pytest.fail("torque command was remembered as a ceiling"))
        client.write_goal_current_signed([1], [80.0])
        assert client._ram_settings.get(1, {}) == {}

    def test_an_unknown_mode_is_read_from_the_bus_before_deciding(self, monkeypatch):
        client = self._client()
        client._operating_modes = {}
        monkeypatch.setattr(client, "read_config_register",
                            lambda mid, key: MODE_MAP[CURRENT] if key == "operating_mode" else None)
        monkeypatch.setattr(client, "sync_write", lambda *a, **k: None)
        client.write_goal_current_signed([1], [10.0])
        assert client._operating_modes[1] == MODE_MAP[CURRENT]

    def test_the_wrist_model_reports_no_current_control(self):
        client = motor_client_class("dynamixel")([1])
        client._model_numbers = {1: 1080}
        assert client.supports_current_control(1) is False
        client._model_numbers = {1: 1220}
        assert client.supports_current_control(1) is True


class TestFeetech:
    def test_is_honest_about_not_driving_current_mode(self):
        for factory in (motor_client_class, mock_motor_client_class):
            c = factory("feetech")([1], *(() if factory is mock_motor_client_class else ("/dev/null", 1000000)))
            assert c.supports_current_control(1) is False
            with pytest.raises(ValueError, match="current control"):
                c.write_goal_current_signed([1], [10.0])


def test_every_client_offers_both_methods():
    from orca_core.hardware.motor_client import MotorClient
    for motor_type in ("dynamixel", "feetech"):
        for factory in (motor_client_class, mock_motor_client_class):
            cls = factory(motor_type)
            assert cls.write_goal_current_signed is not MotorClient.write_goal_current_signed
            assert hasattr(cls, "supports_current_control")
