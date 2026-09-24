#!/usr/bin/env python
"""Live view of the ORCA joint-encoder and tactile sensor streams.

Opens a Tkinter window that streams joint encoder angles for all 17 slots and
tactile forces in a mode you pick with the radio buttons:

        Off  |  Resultant  |  Taxels  |  Combined

Switching the radio reconfigures the device live. Each stream shows its
measured frame rate (Hz).

Passing a hand's ``config.yaml`` monitors exactly the streams that config
declares, on the ports it names — needed whenever the hand's sensing setup
differs from the default. Without one, the connector board is autodetected and
both streams are read from it.

This is for watching the data. To decide whether the sensors are *healthy*,
run ``scripts/check_sensors.py`` instead.

Usage:
    uv run python scripts/monitor_sensors.py                # autodetect the port
    uv run python scripts/monitor_sensors.py orca_core/models/v2/orcahand-touch-left/config.yaml
    uv run python scripts/monitor_sensors.py --port /dev/cu.usbmodemXXXX   # COM3 on Windows
"""
from __future__ import annotations

import argparse
import sys
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import ttk

from orca_core import OrcaHandTouchConfig, load_hand
from orca_core.constants import FINGER_NAMES
from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.tactile_client import TactileClient, NoSensorsAvailableError
from orca_core.hardware.joint_encoder_client import (
    JointEncoderClient,
    EncodersNotAvailableError,
)
from orca_core.hardware.sensing.serial_discovery import (
    baud_for_port,
    discover_sensing_ports,
    resolve_sensing_ports,
)
from orca_core.hardware.sensing.constants import (
    ENCODER_SLOT_TO_JOINT,
    ENCODER_LSB_DEG,
    AUTO_ENC_ANGLE_MASK,
    AUTO_ENC_ANGLE_ERROR_BIT,
    AUTO_ENC_NUM_JOINTS,
)

REFRESH_MS = 150

# (resultant, taxels) flags per selectable mode. "Off" disables the stream.
MODES = {
    "Off":       None,
    "Resultant": (True, False),
    "Taxels":    (False, True),
    "Combined":  (True, True),
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    p.add_argument(
        "config_path", nargs="?", default=None,
        help="Path to the hand model directory or config.yaml. Its sensors "
             "block and joint_encoder_joints decide which streams are opened, "
             "on which ports. Mutually exclusive with --port. Default: "
             "autodetect one connector board carrying both streams.",
    )
    p.add_argument("--port", default=None,
                   help="Sensor serial port carrying both streams, skipping the "
                        "config entirely. Default: autodetect the connector board.")
    p.add_argument("--baud", type=int, default=None,
                   help="Link baud, overriding the config. Default: the config's "
                        "rates, else auto-detect from the connected sensor.")
    p.add_argument("--start-mode", choices=list(MODES), default="Resultant",
                   help="Tactile mode to start in. Default: Resultant.")
    return p.parse_args()


# ----- Stream resolution ----------------------------------------------------


@dataclass(frozen=True)
class StreamTarget:
    """Serial port and baud carrying one sensor stream."""

    port: str
    baud: int


@dataclass(frozen=True)
class SensingTargets:
    """The streams to monitor, and the wiring map for the tactile one.

    Either stream is ``None`` when it is not configured or no port resolved
    for it; both point at the same port when one link carries both.
    """

    tactile: StreamTarget | None
    encoder: StreamTarget | None
    finger_to_sensor_id: dict[str, int] | None = None


def targets_from_port(port: str | None, baud: int | None) -> SensingTargets:
    """Point both streams at a single link, autodetecting ``port`` when ``None``.

    Exits when nothing is plugged in, since there is no config to fall back on.

    Returns:
        Targets whose tactile and encoder streams share one port and baud.
    """
    if port is None:
        discovered = discover_sensing_ports()
        port = discovered.encoder or discovered.tactile
        if port is None:
            sys.exit("No sensor port found; pass --port or a config.yaml.")
        print(f"autodetected port: {port}")
        if baud is None and discovered.shared:
            baud = discovered.tactile_baudrate
    if baud is None:
        baud = baud_for_port(port)
    target = StreamTarget(port, baud)
    return SensingTargets(tactile=target, encoder=target)


def targets_from_config(config, baud: int | None = None) -> SensingTargets:
    """Resolve the streams a hand's config declares against live discovery.

    A config with no ``sensors`` block leaves the tactile stream out, one with
    no ``joint_encoder_joints`` leaves the encoder stream out, and a declared
    stream whose port does not resolve is dropped too. ``baud`` overrides both
    the configured rates and detection.

    Args:
        config: An :class:`~orca_core.OrcaHandConfig` or a touch-capable
            subclass of it.
        baud: Forced link baud, or ``None`` to use the config's own rates.

    Returns:
        Targets for the declared streams, with the config's tactile wiring map.
    """
    has_tactile = isinstance(config, OrcaHandTouchConfig)
    ports = resolve_sensing_ports(
        tactile_override=config.sensor_port if has_tactile else "disabled",
        encoder_override=(
            config.encoder_serial_port if config.has_joint_encoders else "disabled"
        ),
        tactile_baud_override=config.sensor_baudrate if has_tactile else "auto",
    )

    tactile = None
    if ports.tactile is not None:
        # An explicit sensors.port skips discovery, so detect its baud directly.
        tactile_baud = baud or ports.tactile_baudrate or baud_for_port(ports.tactile)
        tactile = StreamTarget(ports.tactile, tactile_baud)
    encoder = None
    if ports.encoder is not None:
        encoder = StreamTarget(ports.encoder, baud or config.encoder_baudrate)

    return SensingTargets(
        tactile=tactile,
        encoder=encoder,
        finger_to_sensor_id=dict(config.finger_to_sensor_id) if has_tactile else None,
    )


class SensingLinks:
    """One serial link per distinct port named by a set of targets.

    Streams that share a port share a single link: two links on one device
    would steal each other's bytes. The encoder's baud wins such a port, its
    stream being the baud-critical one.
    """

    def __init__(self, targets: SensingTargets):
        self._by_port: dict[str, tuple[int, HandSerialLink]] = {}
        for target in (targets.encoder, targets.tactile):
            if target is None or target.port in self._by_port:
                continue
            self._by_port[target.port] = (
                target.baud, HandSerialLink(port=target.port, baudrate=target.baud),
            )
        self.tactile = self._link_for(targets.tactile)
        self.encoder = self._link_for(targets.encoder)
        self._warn_on_baud_clash(targets)

    def _link_for(self, target: StreamTarget | None) -> HandSerialLink | None:
        return self._by_port[target.port][1] if target is not None else None

    def _warn_on_baud_clash(self, targets: SensingTargets) -> None:
        if self.tactile is not None and self.tactile is self.encoder:
            opened = self._by_port[targets.tactile.port][0]
            if targets.tactile.baud != opened:
                print(f"WARNING: both streams share {targets.tactile.port}; using "
                      f"{opened} baud (tactile declares {targets.tactile.baud})")

    def connect(self) -> None:
        """Open every link, closing the ones already open if one fails."""
        try:
            for port, (baud, link) in self._by_port.items():
                print(f"connecting: {port} @ {baud}")
                link.connect()
        except Exception:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        """Close every link, so one that fails cannot strand the others."""
        for _, link in self._by_port.values():
            try:
                link.disconnect()
            except Exception as e:  # noqa: BLE001 — nothing left to recover
                print(f"WARNING: closing a link failed: {type(e).__name__}: {e}")


def build_clients(
    links: SensingLinks, targets: SensingTargets
) -> tuple[TactileClient | None, JointEncoderClient | None]:
    """Connect a client per stream that has a link, and start the encoder stream.

    Returns:
        The tactile and encoder clients, either of which is ``None`` when its
        stream was not resolved.
    """
    tactile = None
    if links.tactile is not None:
        tactile = TactileClient(
            links.tactile, finger_to_sensor_id=targets.finger_to_sensor_id
        )
        tactile.connect()
        print(f"tactile config: {tactile.get_tactile_configuration()}")

    encoder = None
    if links.encoder is not None:
        encoder = JointEncoderClient(links.encoder)
        encoder.connect()
        try:
            encoder.start_stream()
            print("encoder stream: OK")
        except EncodersNotAvailableError as e:
            print(f"WARNING: no encoder frames ({e}) — encoder panel will stay blank")
    return tactile, encoder


class SensorFeedbackUI:
    def __init__(self, root, tactile, encoder, start_mode):
        self.root = root
        self.tactile = tactile
        self.encoder = encoder

        cfg = tactile.get_tactile_configuration() if tactile else None
        self.connected = [f for f in FINGER_NAMES if cfg and cfg.connected.get(f)]
        self.num_taxels = dict(cfg.num_taxels) if cfg else {}

        self.mode_var = tk.StringVar(value=start_mode)
        self.status_var = tk.StringVar(value="")
        self.enc_hz_var = tk.StringVar(
            value="enc: -- Hz" if encoder else "enc: not configured")
        self.tac_hz_var = tk.StringVar(
            value="tac: -- Hz" if tactile else "tac: not configured")

        self.enc_labels: dict[int, ttk.Label] = {}
        self.tac_labels: dict[str, ttk.Label] = {}

        # Hz bookkeeping (frames_ok deltas over wall time).
        self._t0 = time.monotonic()
        self._enc0 = encoder.get_stats().frames_ok if encoder else 0
        self._tac0 = tactile.get_stats().frames_ok if tactile else 0

        self._build_ui()
        self._apply_mode()           # honour --start-mode
        self._schedule_refresh()

    # ----- UI construction --------------------------------------------------

    def _build_ui(self) -> None:
        self.root.title("ORCA sensor feedback")
        self.root.geometry("560x720")

        info = ttk.LabelFrame(self.root, text="link", padding=6)
        info.pack(fill=tk.X, padx=6, pady=4)
        conn = ", ".join(f"{f}({self.num_taxels.get(f, 0)})" for f in self.connected) or "none"
        ttk.Label(info, text=f"tactile connected: {conn}").pack(anchor=tk.W)
        hz_row = ttk.Frame(info)
        hz_row.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(hz_row, textvariable=self.enc_hz_var, width=18).pack(side=tk.LEFT)
        ttk.Label(hz_row, textvariable=self.tac_hz_var, width=18).pack(side=tk.LEFT)

        mode = ttk.LabelFrame(self.root, text="tactile mode (live switch)", padding=6)
        mode.pack(fill=tk.X, padx=6, pady=4)
        for name in MODES:
            ttk.Radiobutton(mode, text=name, value=name, variable=self.mode_var,
                            command=self._apply_mode,
                            state=tk.NORMAL if self.tactile else tk.DISABLED,
                            ).pack(side=tk.LEFT, padx=4)
        ttk.Label(mode, textvariable=self.status_var, foreground="#555").pack(
            side=tk.LEFT, padx=8)

        tac = ttk.LabelFrame(self.root, text="tactile readings", padding=6)
        tac.pack(fill=tk.X, padx=6, pady=4)
        if self.tactile is None:
            ttk.Label(tac, text="(tactile not configured)").pack(anchor=tk.W)
        elif not self.connected:
            ttk.Label(tac, text="(no tactile sensors connected)").pack(anchor=tk.W)
        for f in self.connected:
            row = ttk.Frame(tac)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=f, width=8).pack(side=tk.LEFT)
            lbl = ttk.Label(row, text="--", font=("Menlo", 11))
            lbl.pack(side=tk.LEFT)
            self.tac_labels[f] = lbl

        enc = ttk.LabelFrame(self.root, text="joint encoders (17 slots)", padding=6)
        enc.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
        if self.encoder is None:
            ttk.Label(enc, text="(joint encoders not configured)").pack(anchor=tk.W)
            return
        # Two columns of slots to keep the window compact.
        half = (AUTO_ENC_NUM_JOINTS + 1) // 2
        cols = ttk.Frame(enc)
        cols.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(cols)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        right = ttk.Frame(cols)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        for slot in range(AUTO_ENC_NUM_JOINTS):
            parent = left if slot < half else right
            row = ttk.Frame(parent)
            row.pack(fill=tk.X, pady=1)
            name = ENCODER_SLOT_TO_JOINT.get(slot, f"slot{slot}")
            ttk.Label(row, text=f"{slot:2d} {name}", width=14).pack(side=tk.LEFT)
            lbl = ttk.Label(row, text="--", width=12, font=("Menlo", 11))
            lbl.pack(side=tk.LEFT)
            self.enc_labels[slot] = lbl

    # ----- Mode switching ---------------------------------------------------

    def _apply_mode(self) -> None:
        if self.tactile is None:
            self.status_var.set("tactile not configured")
            return
        name = self.mode_var.get()
        spec = MODES[name]
        try:
            if spec is None:
                self.tactile.stop_stream()
                self.status_var.set("stream off")
            else:
                resultant, taxels = spec
                self.tactile.start_stream(
                    resultant=resultant, taxels=taxels, min_sensors=1)
                self.status_var.set(f"streaming {name.lower()}")
            # Reset the tactile-rate baseline so Hz reflects the new mode.
            self._tac0 = self.tactile.get_stats().frames_ok
            self._t0 = time.monotonic()
            if self.encoder is not None:
                self._enc0 = self.encoder.get_stats().frames_ok
        except NoSensorsAvailableError as e:
            self.status_var.set(f"no sensors: {e}")
        except Exception as e:  # noqa: BLE001 — surface anything in the UI
            self.status_var.set(f"error: {type(e).__name__}: {e}")

    # ----- Periodic refresh -------------------------------------------------

    def _schedule_refresh(self) -> None:
        self._refresh()
        self.root.after(REFRESH_MS, self._schedule_refresh)

    def _refresh(self) -> None:
        now = time.monotonic()
        dt = now - self._t0
        if dt >= 0.5:
            if self.encoder is not None:
                enc_n = self.encoder.get_stats().frames_ok
                self.enc_hz_var.set(f"enc: {(enc_n - self._enc0) / dt:6.0f} Hz")
                self._enc0 = enc_n
            if self.tactile is not None:
                tac_n = self.tactile.get_stats().frames_ok
                self.tac_hz_var.set(f"tac: {(tac_n - self._tac0) / dt:6.0f} Hz")
                self._tac0 = tac_n
            self._t0 = now

        self._refresh_encoders()
        self._refresh_tactile()

    def _refresh_encoders(self) -> None:
        reading = self.encoder.get_latest() if self.encoder else None
        if reading is None:
            return
        raw = reading.raw_counts
        for slot, lbl in self.enc_labels.items():
            r = int(raw[slot])
            deg = (r & AUTO_ENC_ANGLE_MASK) * ENCODER_LSB_DEG
            flag = ""
            if r & AUTO_ENC_ANGLE_ERROR_BIT:
                flag = " err"
            elif not bool(reading.parity_ok[slot]):
                flag = " par?"
            lbl.config(text=f"{deg:6.1f}°{flag}")

    def _refresh_tactile(self) -> None:
        reading = self.tactile.get_latest() if self.tactile else None
        forces = reading.forces if reading else None
        taxels = reading.taxels if reading else None
        for f, lbl in self.tac_labels.items():
            parts = []
            if forces and f in forces:
                fx, fy, fz = forces[f]
                parts.append(f"F=[{fx:+5.1f} {fy:+5.1f} {fz:+5.1f}]N")
            if taxels and f in taxels and taxels[f]:
                fzs = [abs(t[2]) for t in taxels[f]]
                peak = max(fzs)
                idx = fzs.index(peak)
                parts.append(f"taxels:{len(taxels[f])} max|fz|={peak:4.1f}N @{idx}")
            lbl.config(text="  ".join(parts) if parts else "--")


def resolve_targets(args: argparse.Namespace) -> SensingTargets:
    """Turn parsed arguments into the streams to monitor.

    Exits when the two port sources conflict, when the config declares no
    sensors at all, or when none of its declared ports resolve.

    Returns:
        The targets named by ``config_path``, else by ``--port``, else by
        autodetection.
    """
    if args.config_path and args.port:
        raise SystemExit("Pass either config_path or --port, not both.")
    if not args.config_path:
        return targets_from_port(args.port, args.baud)

    config = load_hand(config_path=args.config_path, engage_feedback=False).config
    if not (config.has_joint_encoders or isinstance(config, OrcaHandTouchConfig)):
        raise SystemExit(
            "This hand config declares no sensors: no joint_encoder_joints and "
            "no sensors block. Nothing to monitor."
        )
    targets = targets_from_config(config, args.baud)
    if targets.tactile is None and targets.encoder is None:
        raise SystemExit(
            "No sensor port resolved for this config. Check that the sensing "
            "link is plugged in, or name it in config.yaml (sensors.port / "
            "encoder_serial_port)."
        )
    return targets


def main() -> int:
    args = parse_args()
    targets = resolve_targets(args)

    links = SensingLinks(targets)
    links.connect()
    try:
        tactile, encoder = build_clients(links, targets)
    except Exception:
        links.disconnect()
        raise

    root = tk.Tk()
    SensorFeedbackUI(root, tactile, encoder, start_mode=args.start_mode)

    def on_close():
        try:
            if tactile is not None:
                tactile.stop_stream()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    try:
        root.mainloop()
    finally:
        try:
            if encoder is not None:
                encoder.disconnect()
            if tactile is not None:
                tactile.disconnect()
        finally:
            links.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
