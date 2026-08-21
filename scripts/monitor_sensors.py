#!/usr/bin/env python
"""ORCA electrical monitor: live view of motors, joint encoders and tactile sensors.

Assembly / bring-up dashboard. Opens a Tkinter window that:

  * finds the hand's sensor board on its own and reconnects when the board or
    a sensor is unplugged and plugged back in;
  * scans the motor bus and shows every Dynamixel with its ID, the joint it
    drives (per hand side), live position, voltage and temperature;
  * shows every joint encoder grouped by finger with a live angle and a
    health verdict (live / no encoder / parity or chip error), so magnets and
    wiring can be checked joint by joint while the hand is assembled;
  * shows a card per tactile sensor with taxel count and live force, in a
    mode you pick with the radio buttons (Off / Resultant / Taxels /
    Combined). Switching the radio reconfigures the device live.

This is for watching the data. To decide whether the sensors are *healthy*,
run ``scripts/check_sensors.py`` instead.

Usage:
    uv run python scripts/monitor_sensors.py                # autodetect the port
    uv run python scripts/monitor_sensors.py --port /dev/cu.usbmodemXXXX
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import ttk

from orca_core.constants import FINGER_NAMES
from orca_core.hardware.dynamixel_client import DYNAMIXEL_MODELS
from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.joint_encoder_client import (
    EncodersNotAvailableError,
    JointEncoderClient,
)
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_ANGLE_ERROR_BIT,
    AUTO_ENC_ANGLE_MASK,
    AUTO_ENC_NUM_JOINTS,
    ENCODER_LSB_DEG,
    ENCODER_SLOT_TO_JOINT,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware.sensing.health import EncoderStreamHealth
from orca_core.hardware.sensing.serial_discovery import (
    baud_for_port,
    discover_sensing_ports,
    probe_orca_info,
)
from orca_core.hardware.tactile_client import NoSensorsAvailableError, TactileClient
from orca_core.utils.utils import get_model_path

REFRESH_MS = 100
MOTOR_SCAN_S = 3.0
MOTOR_BAUD = 1_000_000
MOTOR_IDS = tuple(range(1, 18))
ADDR_PRESENT_VOLTAGE, ADDR_PRESENT_TEMP = 144, 146
HEALTH_WINDOW_S = 1.0
TACTILE_RESCAN_S = 2.0
RECONNECT_DELAY_S = 1.0
FORCE_BAR_FULL_N = 10.0

# (resultant, taxels) flags per selectable mode. "Off" disables the stream.
MODES = {
    "Off":       None,
    "Resultant": (True, False),
    "Taxels":    (False, True),
    "Combined":  (True, True),
}

# Display order: finger -> joints proximal to distal. Wrist is its own column.
FINGER_JOINTS = {
    "thumb":  ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"],
    "index":  ["index_abd", "index_mcp", "index_pip"],
    "middle": ["middle_abd", "middle_mcp", "middle_pip"],
    "ring":   ["ring_abd", "ring_mcp", "ring_pip"],
    "pinky":  ["pinky_abd", "pinky_mcp", "pinky_pip"],
    "wrist":  ["wrist"],
}

C_BG, C_CARD, C_TEXT, C_MUTED = "#1e1f24", "#2a2c33", "#e8e8ea", "#8a8d96"
C_OK, C_OFF, C_ERR, C_WARN, C_ACCENT = "#3ccf7a", "#5a5d66", "#ff5c5c", "#ffb84d", "#4da3ff"
FONT, FONT_B, FONT_MONO, FONT_H = ("Helvetica", 12), ("Helvetica", 12, "bold"), ("Menlo", 13), ("Helvetica", 16, "bold")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    p.add_argument("--port", default=None,
                   help="Sensor serial port. Default: autodetect the board (and follow re-plugs).")
    p.add_argument("--baud", type=int, default=None,
                   help="Link baud. Default: auto-detect from the connected sensor.")
    p.add_argument("--start-mode", choices=list(MODES), default="Resultant",
                   help="Tactile mode to start in.")
    p.add_argument("--motor-port", default=None,
                   help="Motor serial port. Default: autodetect (any usbmodem that answers pings).")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Connection management (background thread; owns the serial link)
# ---------------------------------------------------------------------------


@dataclass
class Snapshot:
    """Everything the UI needs, copied under the manager lock."""
    connected: bool = False
    port: str | None = None
    baud: int | None = None
    board: str = ""
    message: str = "searching for board..."
    enc_reading: object = None
    enc_hz: float = 0.0
    tac_hz: float = 0.0
    tac_cfg: object = None
    tac_reading: object = None
    tac_status: str = ""
    link_resyncs: int = 0
    link_bad_lrc: int = 0
    stream_rearms: int = 0
    side: str | None = None


class SensorSession:
    """One open link plus its encoder and tactile clients."""

    def __init__(self, port: str, baud: int):
        self.port, self.baud = port, baud
        self.link = HandSerialLink(port=port, baudrate=baud)
        self.enc = JointEncoderClient(self.link)
        self.tac = TactileClient(self.link)
        self.tac_cfg = None
        self.mode_applied: str | None = None

    def open(self) -> None:
        self.link.connect()
        self.enc.connect()
        self.tac.connect()
        try:
            self.enc.start_stream(timeout=0.5)
        except EncodersNotAvailableError:
            pass  # frames may start later; the UI shows "no encoder frames" meanwhile
        self.tac_cfg = self.tac.get_tactile_configuration()

    @property
    def alive(self) -> bool:
        return self.link.is_connected and not self.link.is_port_dead

    def close(self) -> None:
        for fn in (self.tac.stop_stream, self.enc.disconnect, self.tac.disconnect, self.link.disconnect):
            try:
                fn()
            except Exception:
                pass


class ConnectionManager(threading.Thread):
    """Finds the board, keeps a session open, and re-opens it after unplugs.

    Runs all blocking serial work (discovery, register reads, mode switches)
    off the UI thread. The UI reads :meth:`snapshot` on a timer and requests
    tactile modes with :meth:`request_mode`.
    """

    def __init__(self, port: str | None, baud: int | None, mode: str):
        super().__init__(name="SensorConnectionManager", daemon=True)
        self._fixed_port, self._fixed_baud = port, baud
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._desired_mode = mode
        self._force_reconnect = False
        self._session: SensorSession | None = None
        self._snap = Snapshot()
        self._rate_t0 = time.monotonic()
        self._rate_enc0 = self._rate_tac0 = 0

    # ----- UI-facing API ---------------------------------------------------

    def snapshot(self) -> Snapshot:
        with self._lock:
            return Snapshot(**vars(self._snap))

    def request_mode(self, mode: str) -> None:
        with self._lock:
            self._desired_mode = mode

    def request_reconnect(self) -> None:
        with self._lock:
            self._force_reconnect = True

    def stop(self) -> None:
        self._stop.set()

    # ----- Thread body -----------------------------------------------------

    def run(self) -> None:
        try:
            while not self._stop.is_set():
                if self._session is None:
                    self._try_connect()
                    self._stop.wait(RECONNECT_DELAY_S)
                    continue
                self._service_session()
                self._stop.wait(0.05)
        finally:
            self._drop_session("closed")

    def _set(self, **fields) -> None:
        with self._lock:
            for k, v in fields.items():
                setattr(self._snap, k, v)

    def _try_connect(self) -> None:
        port, baud = self._fixed_port, self._fixed_baud
        if port is None:
            found = discover_sensing_ports()
            port = found.encoder or found.tactile
            if port is None:
                self._set(connected=False, message="searching for board... (none found)")
                return
            if baud is None and found.shared:
                baud = found.tactile_baudrate
        if baud is None:
            baud = baud_for_port(port)
        info = probe_orca_info(port)
        board, side = "", None
        if info is not None:
            side = info.side
            parts = [info.role, info.side or "?", f"HW={info.hw_version}", f"FW={info.fw_version}"]
            if info.hand_id:
                parts.append(f"ID={info.hand_id}")
            board = "  ".join(parts)
        self._set(message=f"connecting {port} @ {baud}...")
        session = SensorSession(port, baud)
        try:
            session.open()
        except Exception as e:
            session.close()
            self._set(connected=False, message=f"connect failed: {type(e).__name__}: {e}")
            return
        self._session = session
        self._last_rescan = 0.0
        self._rate_t0, self._rate_enc0, self._rate_tac0 = time.monotonic(), 0, 0
        self._set(connected=True, port=port, baud=baud, board=board, side=side, message="connected")

    def _drop_session(self, why: str) -> None:
        if self._session is not None:
            self._session.close()
            self._session = None
        self._set(connected=False, enc_reading=None, tac_reading=None, tac_cfg=None,
                  enc_hz=0.0, tac_hz=0.0, message=why)

    def _service_session(self) -> None:
        s = self._session
        with self._lock:
            force, mode = self._force_reconnect, self._desired_mode
            self._force_reconnect = False
        if force:
            self._drop_session("reconnecting...")
            return
        if not s.alive:
            self._drop_session(f"link lost ({s.link.port_error or 'port closed'}); waiting for board...")
            return

        now = time.monotonic()
        if now - self._last_rescan >= TACTILE_RESCAN_S:
            self._last_rescan = now
            self._rescan_tactile(s, mode)
        elif mode != s.mode_applied:
            self._apply_mode(s, mode)

        if s.enc.get_latest() is None:
            try:
                s.enc.start_stream(timeout=0.05)
            except EncodersNotAvailableError:
                pass

        enc_stats, tac_stats, link_stats = s.enc.get_stats(), s.tac.get_stats(), s.link.get_link_stats()
        dt = now - self._rate_t0
        if dt >= 0.5:
            self._set(enc_hz=(enc_stats.frames_ok - self._rate_enc0) / dt,
                      tac_hz=(tac_stats.frames_ok - self._rate_tac0) / dt)
            self._rate_t0, self._rate_enc0, self._rate_tac0 = now, enc_stats.frames_ok, tac_stats.frames_ok
        self._set(enc_reading=s.enc.get_latest(), tac_reading=s.tac.get_latest(),
                  tac_cfg=s.tac_cfg, link_resyncs=link_stats.bad_header_resyncs,
                  link_bad_lrc=sum(link_stats.frames_bad_lrc.values()),
                  stream_rearms=tac_stats.stream_rearms)

    def _rescan_tactile(self, s: SensorSession, mode: str) -> None:
        """Re-read the connected-sensors register so hot-plugged sensors appear."""
        try:
            cfg = s.tac._get_configuration()
        except Exception as e:
            self._set(tac_status=f"no tactile hub ({type(e).__name__})")
            if s.tac_cfg is not None:
                s.tac_cfg, s.mode_applied = None, None
            return
        changed = s.tac_cfg is None or cfg.connected != s.tac_cfg.connected
        s.tac_cfg = cfg
        s.tac._tactile_config = cfg
        if changed or mode != s.mode_applied:
            self._apply_mode(s, mode)

    def _apply_mode(self, s: SensorSession, mode: str) -> None:
        spec = MODES[mode]
        try:
            if spec is None:
                s.tac.stop_stream()
                self._set(tac_status="stream off")
            else:
                s.tac.start_stream(resultant=spec[0], taxels=spec[1], min_sensors=1)
                self._set(tac_status=f"streaming {mode.lower()}")
            s.mode_applied = mode
        except NoSensorsAvailableError:
            self._set(tac_status="no tactile sensors connected")
            s.mode_applied = mode
        except Exception as e:
            self._set(tac_status=f"{type(e).__name__}: {e}")


# ---------------------------------------------------------------------------
# Motor bus (background thread; read-only: pings, positions, voltage, temp)
# ---------------------------------------------------------------------------

ADDR_PRESENT_POSITION = 132
POSITION_UNIT_DEG = 360.0 / 4096
POSITION_POLL_S = 0.15


@dataclass
class MotorSnapshot:
    port: str | None = None
    message: str = "searching for motor bus..."
    motors: dict = field(default_factory=dict)  # id -> {"model", "volt", "temp", "pos"}


class MotorBus(threading.Thread):
    """Finds the Dynamixel bus, pings every motor on a slow loop and polls
    present positions on a fast one. Read-only: never torques or moves motors.
    The sensor port owned by :class:`ConnectionManager` is skipped.
    """

    def __init__(self, fixed_port: str | None, sensor_port_fn):
        super().__init__(name="MotorBus", daemon=True)
        self._fixed_port = fixed_port
        self._sensor_port_fn = sensor_port_fn
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._snap = MotorSnapshot()
        self._port: str | None = None
        self._handler = None
        self._packet = None
        self._motors: dict[int, dict] = {}

    def snapshot(self) -> MotorSnapshot:
        with self._lock:
            return MotorSnapshot(**vars(self._snap))

    def stop(self) -> None:
        self._stop.set()

    def _set(self, **fields) -> None:
        with self._lock:
            for k, v in fields.items():
                setattr(self._snap, k, v)

    def _candidates(self) -> list[str]:
        if self._fixed_port:
            return [self._fixed_port]
        sensor_port = self._sensor_port_fn()
        ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
        return [p for p in ports if p != sensor_port]

    def _open_bus(self) -> None:
        import dynamixel_sdk
        candidates = self._candidates()
        for port in candidates:
            handler = dynamixel_sdk.PortHandler(port)
            try:
                if not handler.openPort():
                    continue
                handler.setBaudRate(MOTOR_BAUD)
                packet = dynamixel_sdk.PacketHandler(2.0)
                if not any(packet.ping(handler, mid)[1] == 0 for mid in MOTOR_IDS):
                    handler.closePort()
                    continue
                self._port, self._handler, self._packet = port, handler, packet
                return
            except Exception:
                try:
                    handler.closePort()
                except Exception:
                    pass
        self._set(port=None, motors={},
                  message="no motors answering" if candidates else "searching for motor bus...")

    def _close_bus(self) -> None:
        if self._handler is not None:
            try:
                self._handler.closePort()
            except Exception:
                pass
        self._handler = self._packet = self._port = None
        self._motors = {}
        self._set(port=None, motors={})

    def _scan(self) -> None:
        motors = {}
        for mid in MOTOR_IDS:
            model, comm, _ = self._packet.ping(self._handler, mid)
            if comm != 0:
                continue
            volt = temp = None
            v, comm, _ = self._packet.read2ByteTxRx(self._handler, mid, ADDR_PRESENT_VOLTAGE)
            if comm == 0:
                volt = v / 10.0
            t, comm, _ = self._packet.read1ByteTxRx(self._handler, mid, ADDR_PRESENT_TEMP)
            if comm == 0:
                temp = t
            prev = self._motors.get(mid, {})
            motors[mid] = {"model": model, "volt": volt, "temp": temp, "pos": prev.get("pos")}
        if not motors:
            self._close_bus()
            return
        self._motors = motors
        self._set(port=self._port, motors=motors,
                  message=f"{len(motors)}/{len(MOTOR_IDS)} motors on {self._port}")

    def _poll_positions(self) -> None:
        for mid, motor in self._motors.items():
            raw, comm, _ = self._packet.read4ByteTxRx(self._handler, mid, ADDR_PRESENT_POSITION)
            if comm != 0:
                continue
            if raw >= 0x80000000:  # signed 32-bit (multi-turn positions go negative)
                raw -= 0x100000000
            motor["pos"] = raw * POSITION_UNIT_DEG
        self._set(motors={k: dict(v) for k, v in self._motors.items()})

    def run(self) -> None:
        next_scan = 0.0
        try:
            while not self._stop.is_set():
                if self._handler is None:
                    self._open_bus()
                    if self._handler is None:
                        self._stop.wait(MOTOR_SCAN_S)
                        continue
                    next_scan = 0.0
                try:
                    now = time.monotonic()
                    if now >= next_scan:
                        self._scan()
                        next_scan = now + MOTOR_SCAN_S
                        if self._handler is None:
                            continue
                    self._poll_positions()
                except Exception:
                    self._close_bus()
                    continue
                self._stop.wait(POSITION_POLL_S)
        finally:
            self._close_bus()


def motor_joint_map(side: str) -> dict[int, str]:
    """Motor ID -> joint name for hands of ``side``, from the packaged model."""
    import yaml
    path = get_model_path(model_name=f"orcahand-full-{side}")
    with open(os.path.join(path, "config.yaml")) as f:
        cfg = yaml.safe_load(f)
    return {abs(m): j for j, m in cfg["joint_to_motor_map"].items()}


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------


class Dot(tk.Canvas):
    """A small coloured status circle."""

    def __init__(self, parent, size=12, **kw):
        super().__init__(parent, width=size, height=size, highlightthickness=0, bg=kw.pop("bg", C_CARD))
        self._id = self.create_oval(1, 1, size - 1, size - 1, fill=C_OFF, outline="")

    def set(self, color: str) -> None:
        self.itemconfig(self._id, fill=color)


class JointRow:
    def __init__(self, parent, joint: str):
        self.frame = tk.Frame(parent, bg=C_CARD)
        self.dot = Dot(self.frame)
        self.dot.pack(side=tk.LEFT, padx=(2, 6))
        short = joint.split("_", 1)[1] if "_" in joint else joint
        tk.Label(self.frame, text=short, width=4, anchor="w", bg=C_CARD, fg=C_TEXT, font=FONT).pack(side=tk.LEFT)
        self.angle = tk.Label(self.frame, text="   --  ", width=8, anchor="e", bg=C_CARD, fg=C_TEXT, font=FONT_MONO)
        self.angle.pack(side=tk.LEFT)
        self.state = tk.Label(self.frame, text="", width=10, anchor="w", bg=C_CARD, fg=C_MUTED, font=("Helvetica", 10))
        self.state.pack(side=tk.LEFT, padx=(6, 2))

    def update(self, deg: float | None, verdict: str) -> None:
        color = {"live": C_OK, "no encoder": C_OFF, "parity": C_WARN, "chip error": C_ERR, "no frames": C_OFF}[verdict]
        self.dot.set(color)
        self.angle.config(text=f"{deg:6.1f}°" if deg is not None else "   --  ",
                          fg=C_TEXT if verdict == "live" else C_MUTED)
        self.state.config(text=verdict, fg=color if verdict != "no encoder" else C_MUTED)


class MotorRow:
    def __init__(self, parent, joint: str):
        self.frame = tk.Frame(parent, bg=C_CARD)
        self.dot = Dot(self.frame)
        self.dot.pack(side=tk.LEFT, padx=(2, 6))
        short = joint.split("_", 1)[1] if "_" in joint else joint
        tk.Label(self.frame, text=short, width=4, anchor="w", bg=C_CARD, fg=C_TEXT, font=FONT).pack(side=tk.LEFT)
        self.mid = tk.Label(self.frame, text="", width=3, anchor="e", bg=C_CARD, fg=C_MUTED, font=FONT_MONO)
        self.mid.pack(side=tk.LEFT)
        self.pos = tk.Label(self.frame, text="", width=8, anchor="e", bg=C_CARD, fg=C_TEXT, font=FONT_MONO)
        self.pos.pack(side=tk.LEFT)
        self.info = tk.Label(self.frame, text="", anchor="w", bg=C_CARD, fg=C_MUTED, font=("Helvetica", 10))
        self.info.pack(side=tk.LEFT, padx=(8, 2))

    def update(self, motor_id: int | None, motor: dict | None) -> None:
        self.mid.config(text=f"#{motor_id}" if motor_id else "--")
        if motor is None:
            self.dot.set(C_OFF)
            self.pos.config(text="   --  ", fg=C_MUTED)
            self.info.config(text="missing", fg=C_MUTED)
            return
        self.dot.set(C_OK)
        pos = motor.get("pos")
        self.pos.config(text=f"{pos:7.1f}\N{DEGREE SIGN}" if pos is not None else "   --  ", fg=C_TEXT)
        volt = f'{motor["volt"]:.1f}V' if motor["volt"] is not None else ""
        temp = f'{motor["temp"]}\N{DEGREE SIGN}C' if motor["temp"] is not None else ""
        self.info.config(text=" ".join(x for x in (volt, temp) if x), fg=C_TEXT)


class TactileCard:
    def __init__(self, parent, finger: str):
        self.frame = tk.Frame(parent, bg=C_CARD, padx=10, pady=8)
        head = tk.Frame(self.frame, bg=C_CARD)
        head.pack(fill=tk.X)
        self.dot = Dot(head)
        self.dot.pack(side=tk.LEFT, padx=(0, 6))
        tk.Label(head, text=finger.capitalize(), bg=C_CARD, fg=C_TEXT, font=FONT_B).pack(side=tk.LEFT)
        self.taxels = tk.Label(head, text="", bg=C_CARD, fg=C_MUTED, font=("Helvetica", 10))
        self.taxels.pack(side=tk.RIGHT)
        self.force = tk.Label(self.frame, text="--", bg=C_CARD, fg=C_MUTED, font=FONT_MONO, anchor="w")
        self.force.pack(fill=tk.X, pady=(6, 2))
        self.bar = tk.Canvas(self.frame, height=8, bg=C_BG, highlightthickness=0)
        self.bar.pack(fill=tk.X)
        self._bar_id = self.bar.create_rectangle(0, 0, 0, 8, fill=C_ACCENT, outline="")
        self.detail = tk.Label(self.frame, text="", bg=C_CARD, fg=C_MUTED, font=("Helvetica", 10), anchor="w")
        self.detail.pack(fill=tk.X, pady=(4, 0))

    def update(self, connected: bool, n_taxels: int, forces, taxels) -> None:
        self.dot.set(C_OK if connected else C_OFF)
        self.taxels.config(text=f"{n_taxels} taxels" if connected else "not connected")
        mag = 0.0
        if forces is not None:
            fx, fy, fz = forces
            mag = (fx * fx + fy * fy + fz * fz) ** 0.5
            self.force.config(text=f"{fx:+5.1f} {fy:+5.1f} {fz:+5.1f} N", fg=C_TEXT)
        elif taxels:
            mag = max(abs(t[2]) for t in taxels)
            self.force.config(text=f"max |fz| {mag:4.1f} N", fg=C_TEXT)
        else:
            self.force.config(text="--", fg=C_MUTED)
        if taxels:
            fzs = [abs(t[2]) for t in taxels]
            active = sum(1 for v in fzs if v > 0.1)
            self.detail.config(text=f"peak {max(fzs):.1f} N @ taxel {fzs.index(max(fzs))}  ·  {active} active")
        else:
            self.detail.config(text="")
        w = max(self.bar.winfo_width(), 1)
        self.bar.coords(self._bar_id, 0, 0, w * min(mag / FORCE_BAR_FULL_N, 1.0), 8)


class SensorMonitorUI:
    def __init__(self, root: tk.Tk, manager: ConnectionManager, motors: MotorBus, start_mode: str):
        self.root, self.manager, self.motors = root, manager, motors
        self.mode_var = tk.StringVar(value=start_mode)
        self.joint_rows: dict[str, JointRow] = {}
        self.finger_summ: dict[str, tk.Label] = {}
        self.tac_cards: dict[str, TactileCard] = {}
        self.motor_rows: dict[str, MotorRow] = {}
        self.motor_summ: dict[str, tk.Label] = {}
        self._joint_maps: dict[str, dict[int, str]] = {}
        self._health = EncoderStreamHealth()
        self._health_t0 = time.monotonic()
        self._last_verdicts = {s: "no frames" for s in range(AUTO_ENC_NUM_JOINTS)}
        self._last_enc_ts = None
        self._build()
        self._tick()

    # ----- Layout ------------------------------------------------------------

    def _build(self) -> None:
        r = self.root
        r.title("ORCA electrical monitor")
        r.configure(bg=C_BG)
        r.minsize(1020, 820)

        style = ttk.Style(r)
        style.theme_use("clam")
        style.configure("Mode.TRadiobutton", background=C_BG, foreground=C_TEXT, font=FONT)
        style.map("Mode.TRadiobutton", background=[("active", C_BG)])
        style.configure("Accent.TButton", font=FONT)

        # Header: title, connection status, board identity, reconnect
        head = tk.Frame(r, bg=C_BG, padx=14, pady=10)
        head.pack(fill=tk.X)
        tk.Label(head, text="ORCA electrical monitor", bg=C_BG, fg=C_TEXT, font=FONT_H).pack(side=tk.LEFT)
        ttk.Button(head, text="Reconnect", style="Accent.TButton",
                   command=self.manager.request_reconnect).pack(side=tk.RIGHT)
        self.conn_dot = Dot(head, size=14, bg=C_BG)
        self.conn_dot.pack(side=tk.LEFT, padx=(18, 6))
        self.conn_lbl = tk.Label(head, text="", bg=C_BG, fg=C_TEXT, font=FONT)
        self.conn_lbl.pack(side=tk.LEFT)
        self.health_lbl = tk.Label(head, text="", bg=C_BG, fg=C_MUTED, font=FONT_H)
        self.health_lbl.pack(side=tk.RIGHT, padx=(0, 16))

        sub = tk.Frame(r, bg=C_BG, padx=14)
        sub.pack(fill=tk.X)
        self.board_lbl = tk.Label(sub, text="", bg=C_BG, fg=C_MUTED, font=("Helvetica", 11))
        self.board_lbl.pack(side=tk.LEFT)
        self.stats_lbl = tk.Label(sub, text="", bg=C_BG, fg=C_MUTED, font=FONT_MONO)
        self.stats_lbl.pack(side=tk.RIGHT)

        # Encoders: one card per finger
        enc = tk.LabelFrame(r, text=" joint encoders ", bg=C_BG, fg=C_MUTED, font=FONT_B, bd=0, padx=10, pady=6)
        enc.pack(fill=tk.X, padx=14, pady=(10, 4))
        for col, (finger, joints) in enumerate(FINGER_JOINTS.items()):
            card = tk.Frame(enc, bg=C_CARD, padx=8, pady=8)
            card.grid(row=0, column=col, padx=4, sticky="nsew")
            enc.columnconfigure(col, weight=1)
            top = tk.Frame(card, bg=C_CARD)
            top.pack(fill=tk.X, pady=(0, 6))
            tk.Label(top, text=finger.capitalize(), bg=C_CARD, fg=C_TEXT, font=FONT_B).pack(side=tk.LEFT)
            summ = tk.Label(top, text="", bg=C_CARD, fg=C_MUTED, font=("Helvetica", 10))
            summ.pack(side=tk.RIGHT)
            self.finger_summ[finger] = summ
            for j in joints:
                row = JointRow(card, j)
                row.frame.pack(fill=tk.X, pady=1)
                self.joint_rows[j] = row
        self.enc_note = tk.Label(enc, text="", bg=C_BG, fg=C_MUTED, font=("Helvetica", 10), anchor="w")
        self.enc_note.grid(row=1, column=0, columnspan=len(FINGER_JOINTS), sticky="w", pady=(6, 0))

        # Motors: one card per finger, rows mirror the encoder layout
        mot = tk.LabelFrame(r, text=" motors ", bg=C_BG, fg=C_MUTED, font=FONT_B, bd=0, padx=10, pady=6)
        mot.pack(fill=tk.X, padx=14, pady=(4, 4))
        for col, (finger, joints) in enumerate(FINGER_JOINTS.items()):
            card = tk.Frame(mot, bg=C_CARD, padx=8, pady=8)
            card.grid(row=0, column=col, padx=4, sticky="nsew")
            mot.columnconfigure(col, weight=1)
            top = tk.Frame(card, bg=C_CARD)
            top.pack(fill=tk.X, pady=(0, 6))
            tk.Label(top, text=finger.capitalize(), bg=C_CARD, fg=C_TEXT, font=FONT_B).pack(side=tk.LEFT)
            summ = tk.Label(top, text="", bg=C_CARD, fg=C_MUTED, font=("Helvetica", 10))
            summ.pack(side=tk.RIGHT)
            self.motor_summ[finger] = summ
            for j in joints:
                row = MotorRow(card, j)
                row.frame.pack(fill=tk.X, pady=1)
                self.motor_rows[j] = row
        self.motor_note = tk.Label(mot, text="", bg=C_BG, fg=C_MUTED, font=("Helvetica", 10), anchor="w")
        self.motor_note.grid(row=1, column=0, columnspan=len(FINGER_JOINTS), sticky="w", pady=(6, 0))

        # Tactile: mode selector + one card per finger
        tac = tk.LabelFrame(r, text=" tactile sensors ", bg=C_BG, fg=C_MUTED, font=FONT_B, bd=0, padx=10, pady=6)
        tac.pack(fill=tk.BOTH, expand=True, padx=14, pady=(4, 12))
        modes = tk.Frame(tac, bg=C_BG)
        modes.pack(fill=tk.X, pady=(0, 6))
        for name in MODES:
            ttk.Radiobutton(modes, text=name, value=name, variable=self.mode_var, style="Mode.TRadiobutton",
                            command=lambda: self.manager.request_mode(self.mode_var.get())).pack(side=tk.LEFT, padx=(0, 12))
        self.tac_status = tk.Label(modes, text="", bg=C_BG, fg=C_MUTED, font=("Helvetica", 11))
        self.tac_status.pack(side=tk.RIGHT)
        cards = tk.Frame(tac, bg=C_BG)
        cards.pack(fill=tk.X)
        for col, finger in enumerate(FINGER_NAMES):
            card = TactileCard(cards, finger)
            card.frame.grid(row=0, column=col, padx=4, sticky="nsew")
            cards.columnconfigure(col, weight=1)
            self.tac_cards[finger] = card

    # ----- Refresh -----------------------------------------------------------

    def _tick(self) -> None:
        self._refresh(self.manager.snapshot())
        self.root.after(REFRESH_MS, self._tick)

    def _refresh(self, s: Snapshot) -> None:
        self.conn_dot.set(C_OK if s.connected else C_ERR)
        self.conn_lbl.config(text=f"{s.port} @ {s.baud}" if s.connected else s.message)
        self.board_lbl.config(text=s.board if s.connected else "")
        self.stats_lbl.config(
            text=f"enc {s.enc_hz:5.0f} Hz   tac {s.tac_hz:5.0f} Hz   resyncs {s.link_resyncs}   "
                 f"bad lrc {s.link_bad_lrc}   rearms {s.stream_rearms}" if s.connected else "")
        enc_live = self._refresh_encoders(s)
        motors_ok = self._refresh_motors(s)
        tac_ok = self._refresh_tactile(s)
        total = AUTO_ENC_NUM_JOINTS + len(MOTOR_IDS) + len(FINGER_NAMES)
        ok = enc_live + motors_ok + tac_ok
        pct = 100.0 * ok / total
        self.health_lbl.config(
            text=f"{pct:.0f}%  ({ok}/{total} working)",
            fg=C_OK if ok == total else (C_WARN if ok else C_ERR))

    def _refresh_encoders(self, s: Snapshot) -> int:
        reading = s.enc_reading
        now = time.monotonic()
        if reading is not None and reading.timestamp != self._last_enc_ts:
            self._health.update(reading)
            self._last_enc_ts = reading.timestamp
        if now - self._health_t0 >= HEALTH_WINDOW_S:
            if self._health.frames:
                self._last_verdicts = {slot: self._verdict(slot) for slot in range(AUTO_ENC_NUM_JOINTS)}
            self._health = EncoderStreamHealth()
            self._health_t0 = now
        if reading is None:
            self._last_verdicts = {slot: "no frames" for slot in range(AUTO_ENC_NUM_JOINTS)}

        live_total = 0
        for finger, joints in FINGER_JOINTS.items():
            live = 0
            for j in joints:
                slot = JOINT_TO_ENCODER_SLOT[j]
                verdict = self._last_verdicts[slot]
                deg = None
                if reading is not None:
                    deg = (int(reading.raw_counts[slot]) & AUTO_ENC_ANGLE_MASK) * ENCODER_LSB_DEG
                self.joint_rows[j].update(deg, verdict)
                live += verdict == "live"
            live_total += live
            self.finger_summ[finger].config(text=f"{live}/{len(joints)} live",
                                            fg=C_OK if live == len(joints) else C_MUTED)
        if reading is None:
            self.enc_note.config(text="no encoder frames — encoder bus not answering" if s.connected else "")
        else:
            self.enc_note.config(text=f"{live_total}/{AUTO_ENC_NUM_JOINTS} encoders live  ·  "
                                      f"error byte 0x{reading.error_byte:02X}  ·  "
                                      f"grey = no encoder on this slot, amber = parity errors, red = chip angle error")
        return live_total

    def _joint_map(self, side: str) -> dict[int, str]:
        if side not in self._joint_maps:
            try:
                self._joint_maps[side] = motor_joint_map(side)
            except Exception:
                self._joint_maps[side] = {}
        return self._joint_maps[side]

    def _refresh_motors(self, s: Snapshot) -> int:
        m = self.motors.snapshot()
        side = s.side or "right"
        id_to_joint = self._joint_map(side)
        joint_to_id = {j: i for i, j in id_to_joint.items()}
        total = 0
        for finger, joints in FINGER_JOINTS.items():
            present = 0
            for j in joints:
                mid = joint_to_id.get(j)
                motor = m.motors.get(mid) if mid else None
                self.motor_rows[j].update(mid, motor)
                present += motor is not None
            total += present
            self.motor_summ[finger].config(text=f"{present}/{len(joints)}",
                                           fg=C_OK if present == len(joints) else C_MUTED)
        side_note = f"{side} hand map" + ("" if s.side else " (assumed, board side unknown)")
        self.motor_note.config(text=f"{m.message}  ·  {side_note}" if m.port or not m.motors
                               else side_note)
        return total

    def _verdict(self, slot: int) -> str:
        rep = self._health.report(slot)
        if rep.healthy:
            return "live"
        if rep.parity_errors:
            return "parity"
        if rep.angle_error_flags:
            return "chip error"
        return "no encoder"

    def _refresh_tactile(self, s: Snapshot) -> int:
        self.tac_status.config(text=s.tac_status if s.connected else "")
        cfg, reading = s.tac_cfg, s.tac_reading
        forces = reading.forces.forces if reading and reading.forces else None
        taxels = reading.taxels.taxels if reading and reading.taxels else None
        n_connected = 0
        for finger, card in self.tac_cards.items():
            connected = bool(cfg and cfg.connected.get(finger))
            n_connected += connected
            n = cfg.num_taxels.get(finger, 0) if cfg else 0
            card.update(connected, n,
                        forces.get(finger) if forces and connected else None,
                        taxels.get(finger) if taxels and connected else None)
        return n_connected


def main() -> int:
    args = parse_args()
    manager = ConnectionManager(args.port, args.baud, args.start_mode)
    manager.start()
    motors = MotorBus(args.motor_port, lambda: manager.snapshot().port)
    motors.start()

    root = tk.Tk()
    SensorMonitorUI(root, manager, motors, start_mode=args.start_mode)

    def on_close():
        manager.stop()
        motors.stop()
        manager.join(timeout=3.0)
        motors.join(timeout=3.0)
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
