"""Tkinter slider controller for ring_mcp and ring_pip driven by
``OrcaHandJointFeedback``. Slider drags push targets onto
``hand._loop.set_target`` (degrees); a 100 ms timer reads the
encoder-measured joint angles and the trim correction for live display.
A tuning panel lets you retune Kp / Ki / correction_max / max_current
without restarting the session.

Usage:
    uv run python scripts/slider_ring.py \\
        orca_core/models/v2/orcahand_right/config.yaml --max-current 600
"""
from __future__ import annotations

import argparse
import dataclasses
import logging
import sys
import tkinter as tk
from tkinter import ttk

from orca_core.control.constants import (
    DEFAULT_CORRECTION_MAX_DEG,
    DEFAULT_KI,
    DEFAULT_KP,
)
from orca_core.hardware_hand_joint_feedback import (
    OrcaHandJointFeedback,
    OrcaHandJointFeedbackError,
)


SLIDER_JOINTS = ("ring_mcp", "ring_pip")
REFRESH_MS = 100


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    p.add_argument(
        "model_path", nargs="?", default=None,
        help="Path to the hand config.yaml.",
    )
    p.add_argument("--encoder-port", default=None)
    p.add_argument("--max-current", type=int, default=None)
    p.add_argument("--Kp", type=float, default=DEFAULT_KP)
    p.add_argument("--Ki", type=float, default=DEFAULT_KI)
    p.add_argument(
        "--correction-max-deg", type=float, default=DEFAULT_CORRECTION_MAX_DEG,
    )
    return p.parse_args()


def _apply_gains(
    hand: OrcaHandJointFeedback, kp: float, ki: float, corr_max_deg: float,
) -> str:
    hand._controller.set_gains(
        Kp=kp, Ki=ki,
        correction_max_deg=corr_max_deg,
        i_clamp_deg=corr_max_deg,
    )
    return f"Kp={kp} Ki={ki} correction_max={corr_max_deg:.1f}°"


class RingSliderUI:
    def __init__(
        self,
        root: tk.Tk,
        hand: OrcaHandJointFeedback,
        initial_gains: tuple[float, float, float],
    ):
        self.hand = hand
        self.root = root
        self.joint_values: dict[str, tk.DoubleVar] = {}
        self.measured_labels: dict[str, ttk.Label] = {}
        self.trim_labels: dict[str, ttk.Label] = {}

        self.kp_var = tk.StringVar(value=f"{initial_gains[0]}")
        self.ki_var = tk.StringVar(value=f"{initial_gains[1]}")
        self.corr_var = tk.StringVar(value=f"{initial_gains[2]}")
        self.max_current_var = tk.StringVar(value=f"{int(hand.config.max_current)}")
        self.apply_status_var = tk.StringVar(value="")

        self._seed_initial_targets()
        self._build_ui()
        self._schedule_refresh()

    def _seed_initial_targets(self) -> None:
        """Latch each slider to the current encoder-measured angle so
        opening the window doesn't yank the joint."""
        measured = self.hand._loop.get_measured_joints()
        for joint in SLIDER_JOINTS:
            self.joint_values[joint] = tk.DoubleVar(value=measured.get(joint, 0.0))

    def _build_ui(self) -> None:
        self.root.title("Ring slider")
        self.root.geometry("560x440")

        for joint in SLIDER_JOINTS:
            rom_min, rom_max = self.hand.config.joint_roms_dict[joint]
            row = ttk.Frame(self.root, padding=6)
            row.pack(fill=tk.X)

            ttk.Label(row, text=joint, width=10).pack(side=tk.LEFT)
            ttk.Label(row, text=f"{rom_min:+4.0f}°", width=6).pack(side=tk.LEFT)
            slider = ttk.Scale(
                row,
                from_=rom_min, to=rom_max,
                orient=tk.HORIZONTAL,
                variable=self.joint_values[joint],
                command=lambda v, j=joint: self._on_slider(j, v),
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ttk.Label(row, text=f"{rom_max:+4.0f}°", width=6).pack(side=tk.LEFT)

            target_label = ttk.Label(row, text="--", width=8)
            target_label.pack(side=tk.LEFT, padx=4)
            self.joint_values[joint].trace_add(
                "write",
                lambda *_, j=joint, lbl=target_label: lbl.config(
                    text=f"→{self.joint_values[j].get():+5.1f}°"
                ),
            )

        readback = ttk.LabelFrame(self.root, text="encoder", padding=6)
        readback.pack(fill=tk.X, padx=6, pady=6)
        for joint in SLIDER_JOINTS:
            row = ttk.Frame(readback)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=joint, width=10).pack(side=tk.LEFT)
            self.measured_labels[joint] = ttk.Label(row, text="meas: --", width=14)
            self.measured_labels[joint].pack(side=tk.LEFT)
            self.trim_labels[joint] = ttk.Label(row, text="", width=14)
            self.trim_labels[joint].pack(side=tk.LEFT)

        # trace_add only fires on writes after binding, so seed the labels
        # by manually firing each slider callback once.
        for joint in SLIDER_JOINTS:
            self._on_slider(joint, self.joint_values[joint].get())

        tuning = ttk.LabelFrame(self.root, text="tuning", padding=6)
        tuning.pack(fill=tk.X, padx=6, pady=6)
        for label, var in (
            ("Kp", self.kp_var),
            ("Ki", self.ki_var),
            ("correction_max (°)", self.corr_var),
            ("max_current (mA)", self.max_current_var),
        ):
            row = ttk.Frame(tuning)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=label, width=18).pack(side=tk.LEFT)
            entry = ttk.Entry(row, textvariable=var, width=12)
            entry.pack(side=tk.LEFT)
            entry.bind("<Return>", lambda _e: self._apply_tuning())

        button_row = ttk.Frame(tuning)
        button_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(button_row, text="Apply", command=self._apply_tuning).pack(
            side=tk.LEFT
        )
        ttk.Label(button_row, textvariable=self.apply_status_var).pack(
            side=tk.LEFT, padx=8
        )

    def _apply_tuning(self) -> None:
        """Read tuning entries, push gains + max_current to the running loop.
        Invalid input leaves prior values untouched and reports the error.
        """
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            corr = float(self.corr_var.get())
            mc = int(float(self.max_current_var.get()))
        except ValueError as exc:
            self.apply_status_var.set(f"parse error: {exc}")
            return

        if mc != int(self.hand.config.max_current):
            self.hand.config = dataclasses.replace(
                self.hand.config, max_current=mc,
            )
            try:
                self.hand.set_max_current(mc)
            except Exception as exc:
                self.apply_status_var.set(f"set_max_current failed: {exc}")
                return

        try:
            summary = _apply_gains(self.hand, kp, ki, corr)
        except Exception as exc:
            self.apply_status_var.set(f"set_gains failed: {exc}")
            return
        self.apply_status_var.set(f"applied: {summary}  max_current={mc}mA")

    def _on_slider(self, joint: str, value_str) -> None:
        try:
            value_deg = float(value_str)
        except ValueError:
            return
        try:
            self.hand._loop.set_target({joint: value_deg})
        except Exception as exc:
            logging.warning("set_target(%s=%.1f°) failed: %s", joint, value_deg, exc)

    def _schedule_refresh(self) -> None:
        self._refresh()
        self.root.after(REFRESH_MS, self._schedule_refresh)

    def _refresh(self) -> None:
        measured = self.hand._loop.get_measured_joints()
        correction = self.hand._loop.get_correction()
        for joint in SLIDER_JOINTS:
            self.measured_labels[joint].config(
                text=f"meas: {measured.get(joint, float('nan')):+6.1f}°"
            )
            self.trim_labels[joint].config(
                text=f"trim: {correction.get(joint, 0.0):+5.1f}°"
            )


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s"
    )

    hand = OrcaHandJointFeedback(config_path=args.model_path)
    overrides = {}
    if args.encoder_port is not None:
        overrides["encoder_serial_port"] = args.encoder_port
    if args.max_current is not None:
        overrides["max_current"] = args.max_current
    if overrides:
        hand.config = dataclasses.replace(hand.config, **overrides)

    try:
        success, msg = hand.connect()
    except OrcaHandJointFeedbackError as exc:
        print(f"FAIL: {exc}")
        return 1
    print(msg)
    if not success:
        return 1

    if args.max_current is not None:
        hand.set_max_current(args.max_current)
        print(f"  → max_current = {args.max_current} mA")

    initial_gains = (args.Kp, args.Ki, args.correction_max_deg)
    print(f"  → gains {_apply_gains(hand, *initial_gains)}")

    root = tk.Tk()
    RingSliderUI(root, hand, initial_gains)
    try:
        root.mainloop()
    finally:
        hand.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
