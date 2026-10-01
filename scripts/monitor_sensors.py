#!/usr/bin/env python
"""ORCA electrical monitor: live view of motors, joint encoders and tactile sensors.

Assembly / bring-up dashboard. Opens a Tkinter window that:

  * opens the sensing streams the hand's config declares, on the ports it
    names, and reconnects when the board or a sensor is unplugged and plugged
    back in;
  * connects to the motor bus the way a control session does (port, family,
    baud rate and IDs resolved by the package) and shows every configured
    motor with its ID, joint, live position, current and temperature;
  * shows every joint encoder grouped by finger with a live angle and a
    health verdict (live / no encoder / parity or chip error), so magnets and
    wiring can be checked joint by joint while the hand is assembled;
  * shows a card per tactile sensor with taxel count and live force, in a
    mode you pick with the radio buttons (Off / Resultant / Taxels /
    Combined). Switching the radio reconfigures the device live.

A stream counts towards the working/total score once the hand declares it, or
once a session opens one anyway — so a hand that declares tactile sensors and
no joint encoders is not graded against 17 encoders, and a hand that declares
no sensing at all is graded on its motors until a board turns up. ``--port``
forces both streams onto one link for board bring-up.

This is for watching the data. To decide whether the sensors are *healthy*,
run ``scripts/check_sensors.py`` instead.

Usage:
    uv run python scripts/monitor_sensors.py                # autodetect the hand
    uv run python scripts/monitor_sensors.py path/to/config.yaml
    uv run python scripts/monitor_sensors.py --port /dev/cu.usbmodemXXXX   # COM3 on Windows
"""
from __future__ import annotations

import argparse
import dataclasses
import math
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import ttk

from orca_core import OrcaHandTouchConfig
from orca_core.constants import FINGER_NAMES
from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.joint_encoder_client import (
    EncodersNotAvailableError,
    JointEncoderClient,
)
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_ANGLE_MASK,
    AUTO_ENC_NUM_JOINTS,
    ENCODER_LSB_DEG,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware.sensing.health import EncoderStreamHealth
from orca_core.hardware.sensing.serial_discovery import (
    baud_for_port,
    probe_orca_info,
    resolve_sensing_ports,
)
from orca_core.hardware.tactile_client import NoSensorsAvailableError, TactileClient
from orca_core.utils.cli import add_hand_arguments, create_hand_from_args

REFRESH_MS = 100
MOTOR_RECONNECT_S = 3.0
MOTOR_POLL_S = 1.0
MOTOR_MISSES_BEFORE_RECONNECT = 5
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

# Joint types proximal to distal, for ordering a finger's rows.
JOINT_TYPE_ORDER = ("cmc", "abd", "mcp", "pip", "dip")
WRIST = "wrist"


def finger_joints(joint_ids) -> dict[str, list[str]]:
    """Group a hand's joints by finger, proximal to distal, wrist last.

    Read off the config so a hand built with a different joint set still gets
    a column per finger it has and a row per joint, rather than a fixed table.
    """
    def order(joint: str) -> int:
        kind = joint.split("_", 1)[1]
        return JOINT_TYPE_ORDER.index(kind) if kind in JOINT_TYPE_ORDER else len(JOINT_TYPE_ORDER)

    groups: dict[str, list[str]] = {}
    for finger in FINGER_NAMES:
        joints = [j for j in joint_ids if j.startswith(f"{finger}_")]
        if joints:
            groups[finger] = sorted(joints, key=order)
    if WRIST in joint_ids:
        groups[WRIST] = [WRIST]
    return groups

C_BG, C_CARD, C_TEXT, C_MUTED = "#1e1f24", "#2a2c33", "#e8e8ea", "#8a8d96"
C_OK, C_OFF, C_ERR, C_WARN, C_ACCENT = "#3ccf7a", "#5a5d66", "#ff5c5c", "#ffb84d", "#4da3ff"
FONT, FONT_B, FONT_MONO, FONT_H = ("Helvetica", 12), ("Helvetica", 12, "bold"), ("Menlo", 13), ("Helvetica", 16, "bold")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    add_hand_arguments(p, feedback_flag=False)
    p.add_argument("--port", default=None,
                   help="Sensor serial port carrying both streams, for board bring-up. "
                        "Default: the ports the hand's config declares (and follow re-plugs).")
    p.add_argument("--baud", type=int, default=None,
                   help="Link baud, overriding the config. Default: the config's rates, "
                        "else auto-detect from the connected sensor.")
    p.add_argument("--start-mode", choices=list(MODES), default="Resultant",
                   help="Tactile mode to start in.")
    p.add_argument("--motor-port", default=None,
                   help="Motor serial port. Default: autodetect (any USB serial port that answers pings).")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Connection management (background thread; owns the serial link)
# ---------------------------------------------------------------------------


# One sensor stream's serial port and baud.
Target = tuple[str, int]


@dataclass
class Snapshot:
    """Everything the UI needs, copied under the manager lock."""
    connected: bool = False
    where: str = ""
    board: str = ""
    message: str = "searching for board..."
    enc_open: bool = False
    tac_open: bool = False
    enc_reading: object = None
    enc_hz: float = 0.0
    tac_hz: float = 0.0
    tac_cfg: object = None
    tac_reading: object = None
    tac_status: str = ""
    link_resyncs: int = 0
    link_bad_lrc: int = 0
    stream_rearms: int = 0


class SensorSession:
    """The links for one monitoring attempt plus the clients riding them.

    Both streams share a single link when they resolve to the same port, the
    way a hand with both sensing capabilities connects; a tactile sensor on
    its own adapter gets a link of its own. A stream the hand does not
    declare gets no client and no link.
    """

    def __init__(self, enc: Target | None, tac: Target | None,
                 finger_to_sensor_id: dict[str, int] | None = None):
        self.targets = [t for t in (enc, tac) if t is not None]
        self.bauds: dict[str, int] = {}
        self.links: dict[str, HandSerialLink] = {}
        # Encoder first: on a shared port its baud is the one that matters.
        for port, baud in self.targets:
            if port not in self.links:
                self.bauds[port] = baud
                self.links[port] = HandSerialLink(port=port, baudrate=baud)
        self._enc_port = enc[0] if enc else None
        self._tac_port = tac[0] if tac else None
        self.enc = JointEncoderClient(self.links[enc[0]]) if enc else None
        self.tac = (
            TactileClient(self.links[tac[0]], finger_to_sensor_id=finger_to_sensor_id)
            if tac else None
        )
        self.tac_cfg = None
        self.mode_applied: str | None = None

    @property
    def where(self) -> str:
        return "  ".join(f"{port} @ {baud}" for port, baud in self.bauds.items())

    @property
    def probe_port(self) -> str:
        """Port to read the board identity from: the encoder board when there is one."""
        return self.targets[0][0]

    def open(self) -> None:
        """Open every link and connect the clients on the ones that came up.

        A port that will not open (unplugged, held by another process) costs
        only its own stream: the other one is still worth watching.
        """
        for port, link in list(self.links.items()):
            try:
                link.connect()
            except Exception as e:
                print(f"WARNING: {port} did not open ({type(e).__name__}: {e})")
                self._drop_port(port)
        if not self.links:
            raise ConnectionError("no sensing port opened")
        if self.enc is not None:
            self.enc.connect()
        if self.tac is not None:
            self.tac.connect()
        if self.enc is not None:
            try:
                self.enc.start_stream(timeout=0.5)
            except EncodersNotAvailableError:
                pass  # frames may start later; the UI shows "no encoder frames" meanwhile
        if self.tac is not None:
            self.tac_cfg = self.tac.get_tactile_configuration()

    def _drop_port(self, port: str) -> None:
        """Forget a link that never opened, and the clients that rode it."""
        link = self.links.pop(port)
        self.bauds.pop(port, None)
        self.targets = [t for t in self.targets if t[0] != port]
        try:
            link.disconnect()
        except Exception:
            pass
        if self._enc_port == port:
            self.enc = None
        if self._tac_port == port:
            self.tac = None

    @property
    def alive(self) -> bool:
        return all(link.is_connected and not link.is_port_dead for link in self.links.values())

    @property
    def error(self) -> str:
        return next(
            (link.port_error for link in self.links.values() if link.port_error), ""
        )

    def link_counters(self) -> tuple[int, int]:
        """Resyncs and bad-LRC frames, summed over every open link."""
        stats = [link.get_link_stats() for link in self.links.values()]
        return (
            sum(st.bad_header_resyncs for st in stats),
            sum(sum(st.frames_bad_lrc.values()) for st in stats),
        )

    def close(self) -> None:
        fns = []
        if self.tac is not None:
            fns += [self.tac.stop_stream, self.tac.disconnect]
        if self.enc is not None:
            fns.append(self.enc.disconnect)
        fns += [link.disconnect for link in self.links.values()]
        for fn in fns:
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

    def __init__(self, config, encoder_joints, port: str | None,
                 baud: int | None, mode: str):
        super().__init__(name="SensorConnectionManager", daemon=True)
        self._fixed_port, self._fixed_baud = port, baud
        touch = config if isinstance(config, OrcaHandTouchConfig) else None
        # A config that declares no sensing says nothing about a hand being
        # brought up, so both streams stay searched for; one that declares a
        # stream narrows the search to it. --port forces both onto one link.
        unknown = touch is None and not config.has_joint_encoders
        self.declares_tactile = touch is not None
        self.declares_encoders = config.has_joint_encoders
        self.has_tactile = port is not None or unknown or self.declares_tactile
        self.has_encoders = port is not None or unknown or self.declares_encoders
        # Joints the config gives an encoder; None when it declares none, so
        # a stream found anyway is graded across every slot.
        self.encoder_joints = list(encoder_joints) if encoder_joints else None
        # Which sensor a finger's readings come from is per hand wiring.
        self._finger_to_sensor_id = (
            dict(touch.finger_to_sensor_id) if touch is not None else None
        )
        self._tactile_port = (
            "auto" if unknown else (touch.sensor_port if touch else "disabled")
        )
        self._tactile_baud = touch.sensor_baudrate if touch else "auto"
        self._encoder_port = (
            "auto" if unknown
            else (config.encoder_serial_port if config.has_joint_encoders else "disabled")
        )
        self._encoder_baud = config.encoder_baudrate
        # Only a port a human wrote into config.yaml is worth a second,
        # rediscovering attempt; "auto" already went through discovery.
        self._names_a_port = port is None and any(
            p not in ("auto", "disabled")
            for p in (self._tactile_port, self._encoder_port)
        )
        self._warned_fallback = False
        self._last_rescan = 0.0
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

    def _resolve_targets(self, pinned: bool) -> tuple[Target | None, Target | None]:
        """Encoder and tactile targets for one connect attempt.

        ``--port`` puts both streams on that port. Otherwise the hand's own
        declaration drives :func:`resolve_sensing_ports`, so a stream the
        config is silent about is never probed for. ``pinned=False`` re-runs
        discovery for a declared stream whose configured port is gone, which
        is how the monitor follows a board that comes back on a new path.
        """
        if self._fixed_port is not None:
            target = (self._fixed_port, self._fixed_baud or baud_for_port(self._fixed_port))
            return target, target

        def port_override(configured: str) -> str:
            if configured == "disabled" or pinned:
                return configured
            return "auto"

        ports = resolve_sensing_ports(
            tactile_override=port_override(self._tactile_port),
            encoder_override=port_override(self._encoder_port),
            tactile_baud_override=self._tactile_baud,
        )
        enc = tac = None
        if ports.encoder is not None:
            enc = (ports.encoder, self._fixed_baud or self._encoder_baud)
        if ports.tactile is not None:
            baud = self._fixed_baud or ports.tactile_baudrate or baud_for_port(ports.tactile)
            tac = (ports.tactile, baud)
        return enc, tac

    def _candidate_targets(self):
        """The configured ports, then discovery when one of them was a fixed path."""
        yield self._resolve_targets(pinned=True)
        if self._names_a_port:
            if not self._warned_fallback:
                self._warned_fallback = True
                print(
                    f"WARNING: config names sensors.port={self._tactile_port!r} / "
                    f"encoder_serial_port={self._encoder_port!r}, which did not "
                    "open; falling back to autodetection"
                )
            yield self._resolve_targets(pinned=False)

    def _board_identity(self, port: str) -> str:
        """One-line board identity for the header, or "" if nothing answered."""
        info = probe_orca_info(port)
        if info is None:
            return ""
        parts = [info.role, info.side or "?", f"HW={info.hw_version}", f"FW={info.fw_version}"]
        if info.hand_id:
            parts.append(f"ID={info.hand_id}")
        return "  ".join(parts)

    def _try_connect(self) -> None:
        try:
            self._connect_a_candidate()
        except Exception as e:  # noqa: BLE001 — a dead thread would freeze the UI
            self._set(connected=False,
                      message=f"connect failed: {type(e).__name__}: {e}")

    def _connect_a_candidate(self) -> None:
        tried: list[tuple[Target | None, Target | None]] = []
        for enc, tac in self._candidate_targets():
            if (enc, tac) in tried or (enc is None and tac is None):
                continue
            tried.append((enc, tac))
            session = SensorSession(enc, tac, self._finger_to_sensor_id)
            self._set(message=f"connecting {session.where}...")
            board = self._board_identity(session.probe_port)
            try:
                session.open()
            except Exception as e:
                session.close()
                self._set(connected=False,
                          message=f"connect failed: {type(e).__name__}: {e}")
                continue
            self._session = session
            self._last_rescan = 0.0
            self._rate_t0, self._rate_enc0, self._rate_tac0 = time.monotonic(), 0, 0
            self._set(connected=True, where=session.where, board=board,
                      enc_open=session.enc is not None,
                      tac_open=session.tac is not None, message="connected")
            return
        if not tried:
            self._set(connected=False, message="searching for board... (none found)")

    def _drop_session(self, why: str) -> None:
        if self._session is not None:
            self._session.close()
            self._session = None
        self._set(connected=False, enc_reading=None, tac_reading=None, tac_cfg=None,
                  enc_hz=0.0, tac_hz=0.0, enc_open=False, tac_open=False,
                  tac_status="", where="", message=why)

    def _service_session(self) -> None:
        s = self._session
        with self._lock:
            force, mode = self._force_reconnect, self._desired_mode
            self._force_reconnect = False
        if force:
            self._drop_session("reconnecting...")
            return
        if not s.alive:
            self._drop_session(f"link lost ({s.error or 'port closed'}); waiting for board...")
            return

        now = time.monotonic()
        if s.tac is not None:
            if now - self._last_rescan >= TACTILE_RESCAN_S:
                self._last_rescan = now
                self._rescan_tactile(s, mode)
            elif mode != s.mode_applied:
                self._apply_mode(s, mode)

        if s.enc is not None and s.enc.get_latest() is None:
            try:
                s.enc.start_stream(timeout=0.05)
            except EncodersNotAvailableError:
                pass

        enc_ok = s.enc.get_stats().frames_ok if s.enc is not None else 0
        tac_stats = s.tac.get_stats() if s.tac is not None else None
        tac_ok = tac_stats.frames_ok if tac_stats is not None else 0
        resyncs, bad_lrc = s.link_counters()
        dt = now - self._rate_t0
        if dt >= 0.5:
            self._set(enc_hz=(enc_ok - self._rate_enc0) / dt,
                      tac_hz=(tac_ok - self._rate_tac0) / dt)
            self._rate_t0, self._rate_enc0, self._rate_tac0 = now, enc_ok, tac_ok
        self._set(enc_reading=s.enc.get_latest() if s.enc is not None else None,
                  tac_reading=s.tac.get_latest() if s.tac is not None else None,
                  tac_cfg=s.tac_cfg, link_resyncs=resyncs, link_bad_lrc=bad_lrc,
                  stream_rearms=tac_stats.stream_rearms if tac_stats is not None else 0)

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


@dataclass
class MotorSnapshot:
    port: str | None = None
    message: str = "connecting to the motor bus..."
    motors: dict = field(default_factory=dict)      # id -> {"pos", "cur", "temp"}
    joint_map: dict = field(default_factory=dict)   # id -> joint, from the hand's config
    motor_ids: tuple = ()


class MotorBus(threading.Thread):
    """Reads every configured motor once a second through the hand's own
    motor client, so port, family, baud rate and IDs are resolved exactly as a
    control session resolves them. One sync read per poll for position and
    current, one for temperature. Read-only: never torques or moves motors.
    """

    def __init__(self, hand):
        super().__init__(name="MotorBus", daemon=True)
        self._hand = hand
        self._lock = threading.Lock()
        self._stop = threading.Event()
        cfg = hand.config
        self._snap = MotorSnapshot(
            joint_map={abs(m): j for j, m in cfg.joint_to_motor_map.items()},
            motor_ids=tuple(cfg.motor_ids),
        )
        self._connected = False
        self._misses = 0
        self._probe_pending = False

    def snapshot(self) -> MotorSnapshot:
        with self._lock:
            return MotorSnapshot(**vars(self._snap))

    def stop(self) -> None:
        self._stop.set()

    def _set(self, **fields) -> None:
        with self._lock:
            for k, v in fields.items():
                setattr(self._snap, k, v)

    def _bus_label(self) -> str:
        cfg = self._hand.config
        return f"{cfg.motor_type} @ {cfg.baudrate} baud on {cfg.port}"

    def connect(self) -> bool:
        ok, msg = self._hand.connect(interactive=False)
        if not ok:
            self._set(port=None, motors={}, message=msg)
            return False
        self._misses = 0
        self._set(port=self._hand.config.port, message=self._bus_label())
        return True

    def disconnect(self) -> None:
        try:
            self._hand.disconnect()
        except Exception:
            pass
        self._set(port=None, motors={})

    def _probe_motors(self) -> None:
        """One read per motor to name the ones not answering. Only runs in the
        cycle after a bus-wide read failed; a healthy bus never pays for it."""
        client = self._hand.motor_client
        answering = [m for m in self._snap.motor_ids if client.read_hardware_error(m) is not None]
        missing = [m for m in self._snap.motor_ids if m not in answering]
        with self._lock:
            kept = {m: self._snap.motors.get(m, {}) for m in answering}
        self._set(motors=kept,
                  message=f"{len(answering)}/{len(self._snap.motor_ids)} motors answering"
                          + (f", missing {missing}" if missing else "") + f", {self._bus_label()}")

    def poll_once(self) -> None:
        """Read every motor: position and current in one transaction, then
        temperature. A failed bus read is followed, next cycle, by a per-motor
        probe that names the silent motors."""
        if self._probe_pending:
            self._probe_pending = False
            self._probe_motors()
        state = self._hand.get_motor_state()
        if not self._hand.last_read_ok:
            self._misses += 1
            self._probe_pending = True
            if self._misses >= MOTOR_MISSES_BEFORE_RECONNECT:
                raise ConnectionError("motor bus stopped answering")
            self._set(message=f"bus read failed ({self._misses}); probing motors next cycle")
            return
        self._misses = 0
        temps = self._hand.get_motor_temp()
        motors = {}
        for i, mid in enumerate(self._snap.motor_ids):
            motors[mid] = {
                "pos": math.degrees(float(state.position[i])),
                "cur": float(state.current[i]),
                "temp": float(temps[i]) if temps is not None else None,
            }
        self._set(port=self._hand.config.port, motors=motors,
                  message=f"{len(motors)}/{len(self._snap.motor_ids)} motors, {self._bus_label()}")

    def run(self) -> None:
        try:
            while not self._stop.is_set():
                if not self._connected:
                    self._connected = self.connect()
                    if not self._connected:
                        self._stop.wait(MOTOR_RECONNECT_S)
                        continue
                try:
                    self.poll_once()
                except Exception as e:
                    self._set(message=f"motor bus lost: {e}; reconnecting")
                    self.disconnect()
                    self._connected = False
                    continue
                self._stop.wait(MOTOR_POLL_S)
        finally:
            if self._connected:
                self.disconnect()
                self._connected = False


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
        color = {"live": C_OK, "no encoder": C_OFF, "parity": C_WARN, "chip error": C_ERR,
                 "no frames": C_OFF, "n/a": C_OFF}[verdict]
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
        cur = f'{motor["cur"]:.0f}mA' if motor.get("cur") is not None else ""
        temp = f'{motor["temp"]:.0f}\N{DEGREE SIGN}C' if motor.get("temp") is not None else ""
        self.info.config(text=" ".join(x for x in (cur, temp) if x), fg=C_TEXT)


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
    def __init__(self, root: tk.Tk, manager: ConnectionManager, motors: MotorBus,
                 start_mode: str, layout: dict[str, list[str]]):
        self.root, self.manager, self.motors = root, manager, motors
        self.finger_joints = layout
        self.mode_var = tk.StringVar(value=start_mode)
        self.joint_rows: dict[str, JointRow] = {}
        self.finger_summ: dict[str, tk.Label] = {}
        self.tac_cards: dict[str, TactileCard] = {}
        self.motor_rows: dict[str, MotorRow] = {}
        self.motor_summ: dict[str, tk.Label] = {}
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
        for col, (finger, joints) in enumerate(self.finger_joints.items()):
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
        self.enc_note.grid(row=1, column=0, columnspan=len(self.finger_joints), sticky="w", pady=(6, 0))

        # Motors: one card per finger, rows mirror the encoder layout
        mot = tk.LabelFrame(r, text=" motors ", bg=C_BG, fg=C_MUTED, font=FONT_B, bd=0, padx=10, pady=6)
        mot.pack(fill=tk.X, padx=14, pady=(4, 4))
        for col, (finger, joints) in enumerate(self.finger_joints.items()):
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
        self.motor_note.grid(row=1, column=0, columnspan=len(self.finger_joints), sticky="w", pady=(6, 0))

        # Tactile: mode selector + one card per finger
        tac = tk.LabelFrame(r, text=" tactile sensors ", bg=C_BG, fg=C_MUTED, font=FONT_B, bd=0, padx=10, pady=6)
        tac.pack(fill=tk.BOTH, expand=True, padx=14, pady=(4, 12))
        modes = tk.Frame(tac, bg=C_BG)
        modes.pack(fill=tk.X, pady=(0, 6))
        for name in MODES:
            ttk.Radiobutton(modes, text=name, value=name, variable=self.mode_var, style="Mode.TRadiobutton",
                            state=tk.NORMAL if self.manager.has_tactile else tk.DISABLED,
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
        self.conn_lbl.config(text=s.where if s.connected else s.message)
        self.board_lbl.config(text=s.board if s.connected else "")
        self.stats_lbl.config(
            text=f"enc {s.enc_hz:5.0f} Hz   tac {s.tac_hz:5.0f} Hz   resyncs {s.link_resyncs}   "
                 f"bad lrc {s.link_bad_lrc}   rearms {s.stream_rearms}" if s.connected else "")
        # A panel counts once the hand declares its stream, or once a session
        # has opened one anyway, so the score never grades absent hardware.
        show_enc = self.manager.declares_encoders or s.enc_open
        show_tac = self.manager.declares_tactile or s.tac_open
        graded = self._graded_joints() if show_enc else []
        enc_live = self._refresh_encoders(s, graded)
        motors_ok = self._refresh_motors(s)
        tac_ok = self._refresh_tactile(s, show_tac)
        total = health_total(len(self.motors.snapshot().motor_ids), len(graded),
                             len(FINGER_NAMES) if show_tac else 0)
        ok = enc_live + motors_ok + tac_ok
        pct = 100.0 * ok / total
        self.health_lbl.config(
            text=f"{pct:.0f}%  ({ok}/{total} working)",
            fg=C_OK if ok == total else (C_WARN if ok else C_ERR))

    def _graded_joints(self) -> list[str]:
        """Joints the config gives an encoder, or every slotted joint when it
        names none. A joint the wire protocol has no slot for is never graded."""
        declared = self.manager.encoder_joints or list(self.joint_rows)
        return [j for j in declared if j in JOINT_TO_ENCODER_SLOT]

    def _refresh_encoders(self, s: Snapshot, graded: list[str]) -> int:
        if not graded:
            for row in self.joint_rows.values():
                row.update(None, "n/a")
            self.enc_note.config(
                text="" if self.manager.has_encoders
                else "this hand declares no joint encoders")
            return 0
        graded_set = set(graded)
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
        for finger, joints in self.finger_joints.items():
            live = counted = 0
            for j in joints:
                if j not in graded_set:
                    self.joint_rows[j].update(None, "n/a")
                    continue
                counted += 1
                slot = JOINT_TO_ENCODER_SLOT[j]
                verdict = self._last_verdicts[slot]
                deg = None
                if reading is not None:
                    deg = (int(reading.raw_counts[slot]) & AUTO_ENC_ANGLE_MASK) * ENCODER_LSB_DEG
                self.joint_rows[j].update(deg, verdict)
                live += verdict == "live"
            live_total += live
            self.finger_summ[finger].config(text=f"{live}/{counted} live",
                                            fg=C_OK if live == counted else C_MUTED)
        if reading is None:
            self.enc_note.config(text="no encoder frames — encoder bus not answering" if s.connected else "")
        else:
            self.enc_note.config(text=f"{live_total}/{len(graded)} encoders live  ·  "
                                      f"error byte 0x{reading.error_byte:02X}  ·  "
                                      f"grey = no encoder on this slot, amber = parity errors, red = chip angle error")
        return live_total

    def _refresh_motors(self, s: Snapshot) -> int:
        m = self.motors.snapshot()
        joint_to_id = {j: i for i, j in m.joint_map.items()}
        total = 0
        for finger, joints in self.finger_joints.items():
            present = 0
            for j in joints:
                mid = joint_to_id.get(j)
                motor = m.motors.get(mid) if mid else None
                self.motor_rows[j].update(mid, motor)
                present += motor is not None
            total += present
            self.motor_summ[finger].config(text=f"{present}/{len(joints)}",
                                           fg=C_OK if present == len(joints) else C_MUTED)
        self.motor_note.config(text=m.message)
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

    def _refresh_tactile(self, s: Snapshot, show: bool) -> int:
        if not show:
            self.tac_status.config(
                text="" if self.manager.has_tactile
                else "this hand declares no tactile sensors")
            for card in self.tac_cards.values():
                card.update(False, 0, None, None)
            return 0
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


def health_total(motors: int, encoders: int, tactile: int) -> int:
    """Denominator of the working/total score. Never zero: a hand with nothing
    configured still has to render a percentage."""
    return max(motors + encoders + tactile, 1)


def main() -> int:
    args = parse_args()
    # Motors only: the monitor opens the sensing ports itself, and detection
    # runs here, before that thread starts, so the two never race for them.
    # The config still carries the sensing declaration the monitor reads.
    hand = create_hand_from_args(args, engage_feedback=False, engage_sensors=False)
    if args.motor_port:
        hand.config = dataclasses.replace(hand.config, port=args.motor_port)
    manager = ConnectionManager(hand.config, hand.encoder_backed_joints,
                                args.port, args.baud, args.start_mode)
    manager.start()
    motors = MotorBus(hand)
    motors.start()

    root = tk.Tk()
    SensorMonitorUI(root, manager, motors, start_mode=args.start_mode,
                    layout=finger_joints(hand.config.joint_ids))

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
