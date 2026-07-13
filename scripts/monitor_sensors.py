#!/usr/bin/env python
"""Live view of the ORCA joint-encoder and tactile sensor streams.

Opens a Tkinter window that streams joint encoder angles for all 17 slots and
tactile forces in a mode you pick with the radio buttons:

        Off  |  Resultant  |  Taxels  |  Combined

Switching the radio reconfigures the device live. Each stream shows its
measured frame rate (Hz).

This is for watching the data. To decide whether the sensors are *healthy*,
run ``scripts/check_sensors.py`` instead.

Usage:
    uv run python scripts/monitor_sensors.py                # autodetect the port
    uv run python scripts/monitor_sensors.py --port /dev/cu.usbmodemXXXX
"""
from __future__ import annotations

import argparse
import os
import sys
import time
import tkinter as tk
from tkinter import ttk

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
    p.add_argument("--port", default=os.environ.get("PORT"),
                   help="Sensor serial port. Default: autodetect the connector board.")
    p.add_argument("--baud", type=int, default=None,
                   help="Link baud. Default: auto-detect from the connected sensor.")
    p.add_argument("--start-mode", choices=list(MODES), default="Resultant",
                   help="Tactile mode to start in.")
    return p.parse_args()


class SensorFeedbackUI:
    def __init__(self, root, link, tactile, encoder, start_mode):
        self.root = root
        self.link = link
        self.tactile = tactile
        self.encoder = encoder

        cfg = tactile.get_tactile_configuration()
        self.connected = [f for f in FINGER_NAMES if cfg and cfg.connected.get(f)]
        self.num_taxels = dict(cfg.num_taxels) if cfg else {}

        self.mode_var = tk.StringVar(value=start_mode)
        self.status_var = tk.StringVar(value="")
        self.enc_hz_var = tk.StringVar(value="enc: -- Hz")
        self.tac_hz_var = tk.StringVar(value="tac: -- Hz")

        self.enc_labels: dict[int, ttk.Label] = {}
        self.tac_labels: dict[str, ttk.Label] = {}

        # Hz bookkeeping (frames_ok deltas over wall time).
        self._t0 = time.monotonic()
        self._enc0 = encoder.get_stats().frames_ok
        self._tac0 = tactile.get_stats().frames_ok

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
                            command=self._apply_mode).pack(side=tk.LEFT, padx=4)
        ttk.Label(mode, textvariable=self.status_var, foreground="#555").pack(
            side=tk.LEFT, padx=8)

        tac = ttk.LabelFrame(self.root, text="tactile readings", padding=6)
        tac.pack(fill=tk.X, padx=6, pady=4)
        if not self.connected:
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
            enc_n = self.encoder.get_stats().frames_ok
            tac_n = self.tactile.get_stats().frames_ok
            self.enc_hz_var.set(f"enc: {(enc_n - self._enc0) / dt:6.0f} Hz")
            self.tac_hz_var.set(f"tac: {(tac_n - self._tac0) / dt:6.0f} Hz")
            self._enc0, self._tac0, self._t0 = enc_n, tac_n, now

        self._refresh_encoders()
        self._refresh_tactile()

    def _refresh_encoders(self) -> None:
        reading = self.encoder.get_latest()
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
        reading = self.tactile.get_latest()
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


def main() -> int:
    args = parse_args()

    port, baud = args.port, args.baud
    if port is None:
        discovered = discover_sensing_ports()
        port = discovered.encoder or discovered.tactile
        if port is None:
            sys.exit("No sensor port found; pass --port.")
        print(f"autodetected port: {port}")
        if baud is None and discovered.shared:
            baud = discovered.tactile_baudrate
    if baud is None:
        baud = baud_for_port(port)
    link = HandSerialLink(port=port, baudrate=baud)
    print(f"connecting: {port} @ {baud}")
    link.connect()

    tactile = TactileClient(link)
    encoder = JointEncoderClient(link)
    tactile.connect()
    encoder.connect()
    try:
        encoder.start_stream()
        print("encoder stream: OK")
    except EncodersNotAvailableError as e:
        print(f"WARNING: no encoder frames ({e}) — encoder panel will stay blank")

    cfg = tactile.get_tactile_configuration()
    print(f"tactile config: {cfg}")

    root = tk.Tk()
    ui = SensorFeedbackUI(root, link, tactile, encoder, start_mode=args.start_mode)

    def on_close():
        try:
            tactile.stop_stream()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    try:
        root.mainloop()
    finally:
        try:
            encoder.disconnect()
            tactile.disconnect()
        finally:
            link.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
