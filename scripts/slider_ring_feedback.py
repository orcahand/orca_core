"""Tkinter slider controller for ring_mcp and ring_pip driven by
``OrcaHandJointFeedback``. Targets are pushed straight onto
``hand._loop.set_target`` (radians), and a 100 ms timer reads back the
encoder-measured joint angles and the cascaded trim (when in cascaded
mode) for live display.

Usage:
    uv run python scripts/slider_ring_feedback.py \\
        orca_core/models/v2/orcahand_right/config.yaml \\
        --joint-control-mode cascaded --max-current 600
"""
from __future__ import annotations

import argparse
import dataclasses
import logging
import math
import sys
import tkinter as tk
from tkinter import ttk

from orca_core.control import CascadedJointController, JointPIDController
from orca_core.control.constants import (
    DEFAULT_CASCADED_CORRECTION_MAX_RAD,
    DEFAULT_CASCADED_KI_RAD_PER_RAD_S,
    DEFAULT_CASCADED_KP_RAD_PER_RAD,
    DEFAULT_KD_MA_S_PER_RAD,
    DEFAULT_KI_MA_PER_RAD_S,
    DEFAULT_KP_MA_PER_RAD,
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
    p.add_argument(
        "--joint-control-mode",
        choices=["current_pid", "cascaded"],
        default="cascaded",
    )
    p.add_argument("--max-current", type=int, default=None)
    p.add_argument(
        "--Kp", type=float, default=None,
        help="cascaded: rad/rad. current_pid: mA/rad. Default: mode constant.",
    )
    p.add_argument(
        "--Ki", type=float, default=None,
        help="cascaded: rad/rad/s. current_pid: mA/rad/s. Default: mode constant.",
    )
    p.add_argument(
        "--Kd", type=float, default=None,
        help="cascaded: correction_max in degrees. current_pid: Kd in mA·s/rad. "
             "Default: mode constant.",
    )
    return p.parse_args()


def _apply_gains_explicit(
    hand: OrcaHandJointFeedback, kp: float, ki: float, third_arg: float,
) -> str:
    """Push gain values onto the active controller. ``third_arg`` is
    correction_max in degrees for cascaded, or Kd in mA·s/rad for
    current_pid.
    """
    if isinstance(hand._pid, CascadedJointController):
        correction_max_rad = math.radians(third_arg)
        hand._pid.set_gains(
            Kp=kp, Ki=ki,
            correction_max_rad=correction_max_rad,
            i_clamp_rad=correction_max_rad,
        )
        return f"cascaded Kp={kp} Ki={ki} correction_max={third_arg:.1f}°"
    if isinstance(hand._pid, JointPIDController):
        i_max = float(hand.config.max_current)
        hand._pid.set_gains(
            Kp=kp, Ki=ki, Kd=third_arg,
            i_clamp_mA=i_max, i_max_mA=i_max,
        )
        return f"current_pid Kp={kp} Ki={ki} Kd={third_arg} i_max={i_max:.0f} mA"
    raise RuntimeError(f"unknown controller type: {type(hand._pid).__name__}")


def _resolve_initial_gains(
    hand: OrcaHandJointFeedback, args: argparse.Namespace,
) -> tuple[float, float, float]:
    """Pick startup gain values from CLI flags, falling back to mode defaults."""
    if isinstance(hand._pid, CascadedJointController):
        kp = DEFAULT_CASCADED_KP_RAD_PER_RAD if args.Kp is None else args.Kp
        ki = DEFAULT_CASCADED_KI_RAD_PER_RAD_S if args.Ki is None else args.Ki
        third = (
            math.degrees(DEFAULT_CASCADED_CORRECTION_MAX_RAD)
            if args.Kd is None else args.Kd
        )
    else:
        kp = DEFAULT_KP_MA_PER_RAD if args.Kp is None else args.Kp
        ki = DEFAULT_KI_MA_PER_RAD_S if args.Ki is None else args.Ki
        third = DEFAULT_KD_MA_S_PER_RAD if args.Kd is None else args.Kd
    return kp, ki, third


class RingSliderUI:
    def __init__(
        self,
        root: tk.Tk,
        hand: OrcaHandJointFeedback,
        initial_gains: tuple[float, float, float],
    ):
        self.hand = hand
        self.root = root
        self.cascaded = isinstance(hand._pid, CascadedJointController)
        self.joint_values: dict[str, tk.DoubleVar] = {}
        self.measured_labels: dict[str, ttk.Label] = {}
        self.trim_labels: dict[str, ttk.Label] = {}

        self.kp_var = tk.StringVar(value=f"{initial_gains[0]}")
        self.ki_var = tk.StringVar(value=f"{initial_gains[1]}")
        self.third_var = tk.StringVar(value=f"{initial_gains[2]}")
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
            value_deg = math.degrees(measured.get(joint, 0.0))
            self.joint_values[joint] = tk.DoubleVar(value=value_deg)

    def _build_ui(self) -> None:
        mode = self.hand.config.joint_control_mode
        self.root.title(f"Ring slider — {mode}")
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

        # Live readback panel.
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

        # Initial slider seed: trace_add only fires on writes after binding,
        # so seed the labels manually.
        for joint in SLIDER_JOINTS:
            v = self.joint_values[joint].get()
            self._on_slider(joint, v)

        third_label = (
            "correction_max (°)" if self.cascaded else "Kd (mA·s/rad)"
        )
        tuning = ttk.LabelFrame(self.root, text="tuning", padding=6)
        tuning.pack(fill=tk.X, padx=6, pady=6)
        for label, var in (
            ("Kp", self.kp_var),
            ("Ki", self.ki_var),
            (third_label, self.third_var),
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
            third = float(self.third_var.get())
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
            summary = _apply_gains_explicit(self.hand, kp, ki, third)
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
            self.hand._loop.set_target({joint: math.radians(value_deg)})
        except Exception as exc:
            logging.warning("set_target(%s=%.1f°) failed: %s", joint, value_deg, exc)

    def _schedule_refresh(self) -> None:
        self._refresh()
        self.root.after(REFRESH_MS, self._schedule_refresh)

    def _refresh(self) -> None:
        measured = self.hand._loop.get_measured_joints()
        correction = (
            self.hand._loop.get_correction() if self.cascaded else {}
        )
        for joint in SLIDER_JOINTS:
            meas_deg = math.degrees(measured.get(joint, float("nan")))
            self.measured_labels[joint].config(text=f"meas: {meas_deg:+6.1f}°")
            if self.cascaded:
                trim_deg = math.degrees(correction.get(joint, 0.0))
                self.trim_labels[joint].config(text=f"trim: {trim_deg:+5.1f}°")


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s"
    )

    hand = OrcaHandJointFeedback(config_path=args.model_path)
    overrides = {"joint_control_mode": args.joint_control_mode}
    if args.encoder_port is not None:
        overrides["encoder_serial_port"] = args.encoder_port
    if args.max_current is not None:
        overrides["max_current"] = args.max_current
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

    initial_gains = _resolve_initial_gains(hand, args)
    print(
        f"  → gains {_apply_gains_explicit(hand, *initial_gains)}"
    )

    root = tk.Tk()
    RingSliderUI(root, hand, initial_gains)
    try:
        root.mainloop()
    finally:
        hand.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
