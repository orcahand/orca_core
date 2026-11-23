"""The rig's turntable: its own OH board with one XC430, driven through orca_core.

Config: the rig's `turntable.yaml` (not the hand's config.yaml). The board is
found by what is on its bus: the controller board (ORCA_ID? motor CDC, or a
plain Dynamixel adapter) with exactly one motor; the hand's board has 17. A
port held by a running hand cannot be probed and is skipped.

Multi-turn position mode with a PWM cap. Every move is a ramp of small goal
steps so the plate turns slowly and a blocked plate shows up as lag: the goal
runs ahead of the motor. Homing ramps toward the hard stop until that happens;
the stalled position is 0°; no stop within the travel + a margin is an error.
The XC430-T240BB-T has no current control, so the defaults are slow and soft,
and any error (or Ctrl-C) releases the torque. Whenever the plate stops, the
torque goes off (motor.hold_after_move: false): only a sweep holds it between
its stops (hold=True), and releases it at its end.

The motor cannot tell the two sides of the arm apart: with the direction
(gear_ratio's sign) wrong, homing ends on the other side and the travel still
measures right. The photos tell (visual_calibration.mounting: the sides come
round in the wrong order).

The motor calibration (`calibrate`) homes a few times, runs to the stop's other
side and saves `turntable_calibration.yaml`: the motor position of 0°, the real
travel, the motor position of every stop. With the motor driving the plate
directly, the motor's in-turn position (absolute, kept across power cycles)
then gives the plate angle at connect without homing, since the travel is
under one turn. Plate angles in degrees, positive away from the stop.
"""
from __future__ import annotations

import signal
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np
from orca_core.hardware.dynamixel_client import DEFAULT_POS_SCALE
from orca_core.hardware.mock_dynamixel_client import MockDynamixelClient
from orca_core.hardware.motor_client import ServoGains
from orca_core.hardware.motor_factory import create_motor_client, motor_client_class
from orca_core.utils.utils import read_yaml, write_yaml_atomic

DEFAULTS = {"speed_deg_s": 10.0, "homing_speed_deg_s": 5.0, "pwm": 0.2, "stall_lag_deg": 2.0,
            "search_margin_deg": 20.0, "position_p_gain": 400, "gear_ratio": 1.0, "operating_mode": 4, "type": "dynamixel",
            "hold_after_move": False}
BOARD_DEFAULTS = {"port": "auto", "usb_serial": None, "baudrates": [1000000, 3000000, 57600], "id_range": [0, 20]}
CAL_DEFAULTS = {"repeats": 3, "repeat_tol_deg": 0.5, "travel_tol_deg": 5.0, "backoff_deg": 1.0}
TICK_S = 0.05
IN_TURN_TOL_DEG = 2.0          # an angle this far past either end still counts as at the stop
STALL_TICKS = 3                # lag on this many ticks in a row: a stall (one bad read is not)
PROBE_TRIES, PROBE_TIMEOUT_S = 3, 0.5     # the motor CDC sometimes misses the first ORCA_ID?


class TurntableError(RuntimeError):
    pass


@dataclass
class TurntableState:
    connected: bool
    mock: bool
    homed: bool
    angle: float | None
    busy: bool
    port: str | None
    motor_id: int | None
    calibrated: bool = False
    zero_from: str | None = None       # "homing" or "calibration"
    torque: bool = False

    def as_dict(self) -> dict:
        return {k: (round(v, 2) if isinstance(v, float) else v) for k, v in self.__dict__.items()}


@dataclass(frozen=True)
class RigBus:
    port: str
    motor_id: int
    baudrate: int
    model: str | None = None
    usb_serial: str | None = None


class MockTurntableClient(MockDynamixelClient):
    """The mock Dynamixel with the rig's hard stop on both sides of the arm."""
    _min_pos = -1.3                      # rad, wherever the motor happened to power up
    _max_motor_pos = -1.3 + np.radians(320.4)


def _merged(cfg: Mapping) -> tuple[dict, dict, dict]:
    board = {**BOARD_DEFAULTS, **(cfg.get("board") or {})}
    motor = {**DEFAULTS, **(cfg.get("motor") or {})}
    return board, motor, {**CAL_DEFAULTS, **(cfg.get("motor_calibration") or {})}


def _board_role(port: str) -> str | None:
    """"motor" / "sensor" for an OH-board CDC, retried: a busy bridge misses a probe now and then."""
    from orca_core.hardware.sensing.serial_discovery import probe_orca_info

    for _ in range(PROBE_TRIES):
        info = probe_orca_info(port, timeout=PROBE_TIMEOUT_S)
        if info is not None:
            return info.role
    return None


def _candidate_ports(board: Mapping, motor_type: str, usb_serial: str | None) -> tuple[list[str], list[str]]:
    """Motor buses that could be the rig's (OH-board motor CDCs and plain adapters),
    and a note for every port passed over."""
    import serial.tools.list_ports
    from orca_core.constants import KNOWN_VIDS
    from orca_core.hardware.sensing.serial_discovery import port_in_use

    if board["port"] not in (None, "auto"):
        return [board["port"]], []
    out, notes = [], []
    for p in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device):
        if usb_serial and p.serial_number != usb_serial:
            continue
        if p.vid in KNOWN_VIDS["oh_board"]:
            if port_in_use(p.device):
                notes.append(f"{p.device}: held by another program")
            elif (role := _board_role(p.device)) == "motor":
                out.append(p.device)
            elif role is None:
                notes.append(f"{p.device}: an OH board that does not answer ORCA_ID?")
        elif p.vid in KNOWN_VIDS.get(motor_type, []):
            out.append(p.device)
    return out, notes


def _usb_serial(port: str) -> str | None:
    import serial.tools.list_ports
    return next((p.serial_number for p in serial.tools.list_ports.comports() if p.device == port), None)


def find_rig_bus(cfg: Mapping, port: str | None = None, motor_id: int | None = None,
                 pinned: Mapping | None = None) -> RigBus:
    """The bus with the rig's one motor. `pinned`: the motor calibration's board
    and motor, tried first; a board it names that is gone falls back to a scan."""
    board, motor, _ = _merged(cfg)
    if port:
        board = {**board, "port": port}
    want = motor_id if motor_id is not None else (None if motor["id"] in (None, "auto") else int(motor["id"]))
    pinned = (pinned or {}).get("board") or {}
    serials = [board["usb_serial"]] if board["usb_serial"] else [pinned.get("usb_serial"), None]
    scanner = motor_client_class(motor["type"])([], "scan", board["baudrates"][0])
    seen = []
    for serial_no in dict.fromkeys(serials):
        ports, notes = _candidate_ports(board, motor["type"], serial_no)
        seen += [n for n in notes if n not in seen]
        for p in ports:
            found = {m["id"]: m for m in scanner.scan_for_motors(p, tuple(board["id_range"]), list(board["baudrates"]))}
            note = (f"{p}: board found, no motor on its bus (motor power or cable?)" if not found else
                    f"{p}: {len(found)} motors (the hand's board?)" if len(found) > 1 else
                    f"{p}: motor {next(iter(found))}, not the asked id {want}")
            if note not in seen:
                seen.append(note)
            if len(found) != 1:
                continue
            (mid, m), = found.items()
            if want is not None and mid != want:
                continue
            return RigBus(p, int(mid), int(m["baud_rate"]), m.get("model_name"), _usb_serial(p))
    raise TurntableError("no board with exactly one motor found" + (f": {'; '.join(seen)}" if seen else
                         ": no motor board plugged in"))


class Turntable:
    def __init__(self, client, motor_id: int, cfg: Mapping, mock: bool = False,
                 bus: RigBus | None = None, calibration: Mapping | None = None):
        self.board_cfg, self.cfg, self.cal_cfg = _merged(cfg)
        self.client, self.motor_id, self.mock, self.bus = client, int(motor_id), mock, bus
        self.ratio = float(self.cfg["gear_ratio"])            # motor turns per plate turn; < 0: reversed
        self.travel = float(cfg["travel"])
        self.stops = cfg.get("stops") or {}
        self.calibration = dict(calibration or {})
        self.zero: float | None = None                        # motor rad at the hard stop
        self.zero_from: str | None = None
        self.lock = threading.Lock()
        self.torque = False
        self._last_motor: float | None = None

    @classmethod
    def open(cls, cfg: Mapping, mock: bool = False, port: str | None = None, motor_id: int | None = None,
             calibration_path: str | None = None, torque: bool | None = None) -> Turntable:
        """torque=False (the default with hold_after_move off): connected to read the angle only, the
        plate stays free; a move turns the torque on for its run."""
        _, motor, _ = _merged(cfg)
        cal = (read_yaml(calibration_path) or {}) if calibration_path and not mock else {}
        if mock:
            mid = motor_id or (1 if motor["id"] in (None, "auto") else int(motor["id"]))
            client = MockTurntableClient([mid], port or "mock")
            client._pos[mid] = 0.4
            t = cls(client, mid, cfg, mock=True, calibration=cal)
        else:
            bus = find_rig_bus(cfg, port, motor_id, cal)
            t = cls(create_motor_client(motor["type"], [bus.motor_id], bus.port, bus.baudrate), bus.motor_id, cfg,
                    bus=bus, calibration=cal)
        t.calibration_path = calibration_path
        t.client.connect()
        try:
            t.client.set_torque_enabled([t.motor_id], False)
            t.client.set_operating_mode([t.motor_id], int(motor["operating_mode"]))
            t.client.write_servo_gains({t.motor_id: ServoGains(kp=int(motor["position_p_gain"]))})
            t._torque(bool(motor["hold_after_move"]) if torque is None else torque)
            t._zero_from_calibration()
        except BaseException:
            t.close()
            raise
        return t

    def _torque(self, on: bool) -> None:
        """On: the PWM cap first, then torque, holding where the plate is."""
        if on and not self.torque:
            self.client.write_pwm_fraction([self.motor_id], float(self.cfg["pwm"]))
            self.client.set_torque_enabled([self.motor_id], True)
            self._hold()
        elif not on:
            self.client.set_torque_enabled([self.motor_id], False)
        self.torque = on

    def _op(self, fn, *a, hold: bool | None = None):
        """One motor operation at a time, torque on for it and off after it unless held (hold=None:
        motor.hold_after_move); any error or interrupt releases the torque."""
        if not self.lock.acquire(blocking=False):
            raise TurntableError("the turntable is busy")
        try:
            self._torque(True)
            out = fn(*a)
            if not (bool(self.cfg["hold_after_move"]) if hold is None else hold):
                self._torque(False)
            return out
        except BaseException as e:
            self.release()
            if isinstance(e, Exception) and not isinstance(e, TurntableError):
                raise TurntableError(f"{type(e).__name__}: {e} (torque released)") from e
            raise
        finally:
            self.lock.release()

    def release(self) -> None:
        """Torque off (the plate turns freely; the multi-turn position, so the zero, is kept)."""
        try:
            self._torque(False)
        except Exception:
            self.torque = False

    def close(self) -> None:
        try:
            self.release()
        finally:
            self.client.disconnect()

    def state(self) -> TurntableState:
        """While a move runs, the angle is its last reading (the bus belongs to the move)."""
        busy = self.lock.locked()
        return TurntableState(True, self.mock, self.zero is not None, self.peek_angle(), busy, getattr(self.client, "port_name", None), self.motor_id,
                              bool(self.calibration.get("zero_in_turn_deg") is not None), self.zero_from,
                              self.torque)

    def _motor(self) -> float:
        self._last_motor = float(self.client.read_position_velocity_current().position[0])
        return self._last_motor

    def _goal(self, rad: float) -> None:
        self.client.write_desired_pos([self.motor_id], np.array([rad]))

    def _hold(self) -> None:
        self._goal(self._motor())

    def peek_angle(self) -> float | None:
        """The plate angle for a poll: read when the bus is free, else the running move's last reading."""
        if self.zero is None:
            return None
        if not self.lock.acquire(blocking=False):
            return self.angle(cached=True)
        try:
            return self.angle()
        finally:
            self.lock.release()

    def angle(self, cached: bool = False) -> float:
        m = self._last_motor if cached and self._last_motor is not None else self._motor()
        return float(np.degrees(m - self.zero) / self.ratio)

    def _zero_from_calibration(self) -> None:
        """Direct drive: the in-turn position says where in the travel the plate is."""
        c = self.calibration
        if c.get("zero_in_turn_deg") is None or abs(abs(self.ratio) - 1.0) > 1e-9 or \
                int((c.get("motor") or {}).get("id", self.motor_id)) != self.motor_id:
            return
        a = ((np.degrees(self._motor()) - float(c["zero_in_turn_deg"])) * np.sign(self.ratio)) % 360.0
        if a > 360.0 - IN_TURN_TOL_DEG:
            a -= 360.0
        if a <= float(c.get("travel_deg", self.travel)) + IN_TURN_TOL_DEG:
            self.zero = self._motor() - np.radians(a) * self.ratio
            self.zero_from = "calibration"

    def _ramp(self, start: float, end: float, speed_deg_s: float, stop_on_stall: bool) -> bool:
        """Goal from start to end (motor rad) at speed (plate deg/s). True if it stalled."""
        step = np.radians(speed_deg_s * abs(self.ratio)) * TICK_S
        n = max(1, int(np.ceil(abs(end - start) / step)))
        lag = np.radians(float(self.cfg["stall_lag_deg"]) * abs(self.ratio))
        behind = 0
        for goal in np.linspace(start, end, n + 1)[1:]:
            self._goal(goal)
            time.sleep(0.0 if self.mock else TICK_S)
            off = abs(self._motor() - goal)
            behind = behind + 1 if off > lag else 0
            if behind >= STALL_TICKS:
                self._hold()
                if stop_on_stall:
                    return True
                at = f" at {self.angle():.1f}°" if self.zero is not None else ""
                raise TurntableError(f"the plate is blocked{at} (the motor {np.degrees(off) / abs(self.ratio):.1f}° "
                                     "behind its goal)")
        return False

    def _to_stop(self, direction: float) -> float:
        """Ramp toward a stop (-1: the hard stop, +1: the arm's other side), at most the
        travel + search_margin_deg, until it stalls. The motor rad there."""
        start, reach = self._motor(), self.travel + float(self.cfg["search_margin_deg"])
        if not self._ramp(start, start + direction * np.radians(reach) * self.ratio,
                          float(self.cfg["homing_speed_deg_s"]), stop_on_stall=True):
            raise TurntableError(f"no hard stop within {reach:g}°: is the fin or the arm missing, "
                                 "or the motor slipping on the plate?")
        time.sleep(0.0 if self.mock else 0.3)
        return self._motor()

    def _back_off(self, at: float, direction: float) -> None:
        self._goal(at - direction * np.radians(float(self.cal_cfg["backoff_deg"])) * self.ratio)
        time.sleep(0.0 if self.mock else 0.3)

    def home(self, hold: bool | None = None) -> TurntableState:
        """Turn toward the hard stop until it stalls there: that is 0°."""
        self._op(self._home, hold=hold)
        return self.state()

    def _home(self) -> None:
        self.zero = self._to_stop(-1.0)
        self.zero_from = "homing"
        self._back_off(self.zero, -1.0)

    def move(self, angle: float, settle_s: float = 0.0, hold: bool | None = None) -> TurntableState:
        """hold=True: the torque stays on at the angle (a sweep's photo); else per hold_after_move."""
        self._op(self._move, angle, settle_s, hold=hold)
        return self.state()

    def _move(self, angle: float, settle_s: float = 0.0) -> None:
        if self.zero is None:
            raise TurntableError("home the turntable first")
        if not 0.0 <= angle <= self.travel:
            raise TurntableError(f"{angle:g}° is outside the travel 0..{self.travel:g}°")
        target = self.zero + np.radians(angle) * self.ratio
        self._ramp(self._motor(), target, float(self.cfg["speed_deg_s"]), stop_on_stall=False)
        if not self.mock:
            self.client.wait_for_motion_complete(timeout=3.0)
            time.sleep(settle_s)

    def calibrate(self, save: bool = True) -> dict:
        return self._op(self._calibrate, save, hold=False)

    def _calibrate(self, save: bool) -> dict:
        """Which motor position is which plate angle: home `repeats` times, run to the
        far side of the stop, check both against the design, visit every stop."""
        c = self.cal_cfg
        zeros = []
        for i in range(int(c["repeats"])):
            if i:
                self._move(min(20.0, self.travel))
            self._home()
            zeros.append(self.zero)
        spread = float(np.degrees(np.ptp(zeros)) / abs(self.ratio))
        if spread > float(c["repeat_tol_deg"]):
            raise TurntableError(f"the hard stop moved {spread:.2f}° between homings (allowed {c['repeat_tol_deg']}°)")
        self.zero = float(np.mean(zeros))
        far = self._to_stop(+1.0)
        self._back_off(far, +1.0)
        travel = float(np.degrees(far - self.zero) / self.ratio)
        if abs(travel - self.travel) > float(c["travel_tol_deg"]):
            raise TurntableError(f"measured travel {travel:.1f}°, the design says {self.travel:g}°: "
                                 "check gear_ratio (sign: direction), or something is in the way")
        start, step, count = (float(self.stops.get(k, d)) for k, d in (("start", 0.0), ("step", 15.0), ("count", 1)))
        stops = {}
        for a in [start + k * step for k in range(int(count))]:
            self._move(a)
            stops[round(a, 3)] = round(float(np.degrees(self._motor())), 3)
        self._move(0.0)
        zero_deg = float(np.degrees(self.zero))
        result = {
            "calibrated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "board": {"port": getattr(self.client, "port_name", None),
                      "usb_serial": self.bus.usb_serial if self.bus else None},
            "motor": {"id": self.motor_id, "baudrate": getattr(self.client, "baudrate", None),
                      "model": self.bus.model if self.bus else None},
            "gear_ratio": self.ratio,
            # plate angle = (motor_deg - zero_motor_deg) / gear_ratio
            "zero_motor_deg": round(zero_deg, 3),
            "zero_in_turn_deg": round(zero_deg % 360.0, 3),
            "zero_in_turn_ticks": int(round((self.zero % (2 * np.pi)) / DEFAULT_POS_SCALE)) % 4096,
            "travel_deg": round(travel, 3),
            "travel_error_deg": round(travel - self.travel, 3),
            "homing_spread_deg": round(spread, 3),
            "stops_motor_deg": stops,
        }
        self.calibration, self.zero_from = result, "calibration"
        path = getattr(self, "calibration_path", None)
        if save and path:
            old = read_yaml(path) or {}
            history = list(old.pop("history", None) or [])
            if old:
                history.insert(0, old)
            write_yaml_atomic(path, {**result, "history": history[:10]})
        return result


def main() -> None:
    import argparse
    import json

    from visual_calibration.rig_config import RigConfig

    ap = argparse.ArgumentParser(description="The rig's turntable motor (turntable.yaml).")
    ap.add_argument("action", choices=["find", "home", "calibrate", "state", "move"])
    ap.add_argument("angle", nargs="?", type=float, help="move: plate angle, deg")
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--port")
    ap.add_argument("--id", type=int)
    args = ap.parse_args()
    rig = RigConfig.load()
    cfg = rig.raw["turntable"]
    _, motor, _ = _merged(cfg)
    print("motor settings:", {k: motor[k] for k in ("speed_deg_s", "homing_speed_deg_s", "pwm", "stall_lag_deg",
                                                      "search_margin_deg", "position_p_gain", "gear_ratio", "operating_mode")})
    if args.action == "find":
        print(find_rig_bus(cfg, args.port, args.id, read_yaml(rig.turntable_calibration_path) or {}))
        return
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    t = Turntable.open(cfg, mock=args.mock, port=args.port, motor_id=args.id,
                       calibration_path=rig.turntable_calibration_path)
    try:
        out = {"home": lambda: t.home().as_dict(), "calibrate": lambda: t.calibrate(save=not args.mock),
               "state": lambda: t.state().as_dict(), "move": lambda: t.move(args.angle).as_dict()}[args.action]()
        print(json.dumps(out, indent=1, default=str))
    finally:
        t.close()


if __name__ == "__main__":
    main()
