"""The turntable driver on the mock Dynamixel with the rig's hard stop."""
import numpy as np
import pytest

from visual_calibration import turntable as tt
from visual_calibration.rig_config import RigConfig
from visual_calibration.turntable import Turntable, TurntableError


def cfg():
    return RigConfig.load().raw["turntable"]


def test_config_is_its_own_file():
    rig = RigConfig.load()
    assert rig.raw["turntable"]["motor"]["model"].startswith("XC430")
    assert rig.turntable_calibration_path.endswith("turntable_calibration.yaml")


def test_home_into_the_stop_then_move():
    t = Turntable.open(cfg(), mock=True)
    with pytest.raises(TurntableError, match="home"):
        t.move(30.0)
    s = t.home()
    assert s.homed and s.zero_from == "homing" and abs(s.angle - 1.0) < 0.5      # backed off the stop
    assert abs(t.zero - t.client._min_pos) < 0.06
    assert abs(t.move(135.0).angle - 135.0) < 0.1
    with pytest.raises(TurntableError, match="outside the travel"):
        t.move(330.0)
    t.move(10.0)
    t.client._max_motor_pos = t.zero + 1.0                     # something in the way at ~57°
    with pytest.raises(TurntableError, match="blocked"):
        t.move(100.0)
    t.close()


def test_calibrate_maps_motor_to_plate_and_survives_a_power_cycle():
    t = Turntable.open(cfg(), mock=True)
    cal = t.calibrate(save=False)
    assert abs(cal["travel_deg"] - 320.0) < 1.0 and cal["homing_spread_deg"] < 0.5
    assert len(cal["stops_motor_deg"]) == 22
    assert abs(cal["stops_motor_deg"][90.0] - cal["zero_motor_deg"] - 90.0) < 0.2
    assert 0 <= cal["zero_in_turn_ticks"] < 4096
    t.move(200.0)

    # Power cycle: the multi-turn count is gone, the in-turn position stays.
    client = t.client
    client._pos[t.motor_id] = float(client._pos[t.motor_id] % (2 * np.pi))
    client._min_pos, client._max_motor_pos = -100.0, 100.0
    again = Turntable(client, t.motor_id, cfg(), mock=True, calibration=cal)
    again._zero_from_calibration()
    s = again.state()
    assert s.homed and s.zero_from == "calibration" and abs(s.angle - 200.0) < 0.2


def test_calibrate_catches_a_wrong_gear_ratio():
    c = cfg()
    c = {**c, "motor": {**c["motor"], "gear_ratio": 2.0}}
    t = Turntable.open(c, mock=True)
    with pytest.raises(TurntableError, match="gear_ratio"):
        t.calibrate(save=False)


def test_find_rig_bus_takes_the_board_with_one_motor(monkeypatch):
    buses = {"/dev/hand": [{"id": i, "baud_rate": 1000000, "model_name": "XC330"} for i in range(1, 18)],
             "/dev/rig": [{"id": 3, "baud_rate": 57600, "model_name": "XC430-T240BB-T"}]}
    monkeypatch.setattr(tt, "_candidate_ports", lambda board, kind, serial_no: (list(buses), []))
    monkeypatch.setattr(tt, "_usb_serial", lambda port: "RIG123")

    class Scanner:
        def __init__(self, *a):
            pass

        def scan_for_motors(self, port, id_range, bauds):
            return buses[port]

    monkeypatch.setattr(tt, "motor_client_class", lambda kind: Scanner)
    bus = tt.find_rig_bus(cfg())
    assert (bus.port, bus.motor_id, bus.baudrate, bus.usb_serial) == ("/dev/rig", 3, 57600, "RIG123")
    with pytest.raises(TurntableError, match="exactly one motor"):
        tt.find_rig_bus(cfg(), motor_id=5)
    buses["/dev/rig"] = []
    with pytest.raises(TurntableError, match="/dev/rig: board found, no motor on its bus"):
        tt.find_rig_bus(cfg())


def test_probe_is_retried(monkeypatch):
    from orca_core.hardware.sensing import serial_discovery as sd
    replies = iter([None, None, sd.OrcaBoardInfo(role="motor")])
    monkeypatch.setattr(sd, "probe_orca_info", lambda port, timeout: next(replies))
    assert tt._board_role("/dev/x") == "motor"


def test_slow_and_soft_by_default():
    m = cfg()["motor"]
    assert m["speed_deg_s"] <= 10 and m["homing_speed_deg_s"] <= 5 and m["pwm"] <= 0.2 and m["stall_lag_deg"] <= 2
    t = Turntable.open(cfg(), mock=True, torque=True)
    assert t.client.pwm_fraction[t.motor_id] == pytest.approx(m["pwm"])
    assert t.client._servo_gains[t.motor_id].kp == m["position_p_gain"] < 700     # 700 hunts while holding
    assert t.torque and t.state().torque
    t.close()


def test_homing_without_a_stop_gives_up_and_releases():
    t = Turntable.open(cfg(), mock=True)
    t.client._min_pos, t.client._max_motor_pos = -100.0, 100.0          # no stop at all
    start = t._motor()
    with pytest.raises(TurntableError, match="no hard stop within 340"):
        t.home()
    assert abs(np.degrees(start - t._motor()) - 340.0) < 1.0 and not t.torque


def test_any_error_or_interrupt_releases_the_torque(monkeypatch):
    t = Turntable.open(cfg(), mock=True)
    t.home()
    t.client._max_motor_pos = t.zero + 1.0
    with pytest.raises(TurntableError, match="blocked"):
        t.move(100.0)
    assert not t.torque
    t.move(10.0, hold=True)                                            # torque back on for the next move
    assert t.torque

    def boom(*a, **kw):
        raise KeyboardInterrupt
    monkeypatch.setattr(t, "_ramp", boom)
    with pytest.raises(KeyboardInterrupt):
        t.move(20.0)
    assert not t.torque and not t.lock.locked()
    monkeypatch.setattr(t, "_ramp", lambda *a, **kw: (_ for _ in ()).throw(OSError("port gone")))
    with pytest.raises(TurntableError, match="port gone.*torque released"):
        t.move(20.0)
    assert not t.torque


def test_state_while_busy_does_not_touch_the_bus():
    t = Turntable.open(cfg(), mock=True)
    t.home()
    t.move(30.0)
    with t.lock:
        reads = []
        t.client.read_position_velocity_current = lambda: reads.append(1)
        s = t.state()
    assert s.busy and abs(s.angle - 30.0) < 0.1 and not reads


def test_uncalibrated_turntable_is_flagged():
    from visual_calibration.checks import turntable
    assert turntable(RigConfig.load(), {})[0]["code"] == "turntable_uncalibrated"
    assert turntable(RigConfig.load(), {"zero_in_turn_deg": 0.0}) == []


def test_connected_with_the_torque_off_reads_the_angle():
    t = Turntable.open(cfg(), mock=True)
    t.home()
    t.move(40.0)
    t.release()
    assert not t.torque and abs(t.state().angle - 40.0) < 0.1          # free plate, angle still read
    t.move(60.0)
    assert not t.torque and abs(t.angle() - 60.0) < 0.1
    t.close()
    free = Turntable.open(cfg(), mock=True, torque=False)
    assert not free.torque and not free.state().torque
    free.close()


def test_the_torque_is_off_whenever_the_plate_stops():
    """Connect, home, move, calibrate: torque on only while it turns; a sweep's photo holds it."""
    m = cfg()["motor"]
    assert m["hold_after_move"] is False
    t = Turntable.open(cfg(), mock=True)
    seen = []
    ramp = t._ramp
    t._ramp = lambda *a, **kw: (seen.append(t.torque), ramp(*a, **kw))[1]
    assert not t.torque
    t.home()
    assert not t.torque
    t.move(30.0)
    assert not t.torque and abs(t.angle() - 30.0) < 0.1
    t.move(45.0, hold=True)
    assert t.torque and t.state().torque
    t.move(50.0)
    assert not t.torque
    t.calibrate(save=False)
    assert not t.torque and seen and all(seen)                         # on while turning
    t.close()
    held = Turntable.open({**cfg(), "motor": {**m, "hold_after_move": True}}, mock=True)
    held.home()
    held.move(20.0)
    assert held.torque
    held.close()
