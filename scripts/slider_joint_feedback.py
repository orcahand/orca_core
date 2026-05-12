"""Tkinter slider controller for the closed-loop ``OrcaHandJointFeedback``.

By default the UI shows one slider per encoder-backed joint; ``--fingers``
or ``--joints`` filters the set. Slider
drags push targets onto the running loop; a 100 ms timer reads the
encoder-measured joint angle + the PI trim correction for live display.
A tuning panel retunes Kp / Ki / correction_max / max_current without
restarting the session.

Usage:
    uv run python scripts/slider_joint_feedback.py \\
        orca_core/models/v2/orcahand_right/config.yaml --max-current 600
    uv run python scripts/slider_joint_feedback.py CONFIG --fingers ring
    uv run python scripts/slider_joint_feedback.py CONFIG --joints ring_mcp ring_pip
"""
from __future__ import annotations

import argparse
import dataclasses
import logging
import sys
import tkinter as tk
from tkinter import ttk
from typing import List

from orca_core.control.constants import (
    DEFAULT_CORRECTION_MAX_DEG,
    DEFAULT_KI,
    DEFAULT_KP,
)
from orca_core.hardware_hand_joint_feedback import (
    JointFeedbackConnectError,
    OrcaHandJointFeedback,
)
from orca_core.joint_position import OrcaJointPositions

FINGER_TO_JOINTS = {
    "thumb": ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"],
    "index": ["index_abd", "index_mcp", "index_pip"],
    "middle": ["middle_abd", "middle_mcp", "middle_pip"],
    "ring": ["ring_abd", "ring_mcp", "ring_pip"],
    "pinky": ["pinky_abd", "pinky_mcp", "pinky_pip"],
}
ALL_JOINTS = [j for joints in FINGER_TO_JOINTS.values() for j in joints]
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
    p.add_argument(
        "--fingers", nargs="+", choices=list(FINGER_TO_JOINTS.keys()),
        help="Show sliders for these fingers only.",
    )
    p.add_argument(
        "--joints", nargs="+", choices=ALL_JOINTS,
        help="Show sliders for these joints only.",
    )
    return p.parse_args()


def _resolve_joint_set(
    args: argparse.Namespace, hand: OrcaHandJointFeedback,
) -> List[str]:
    """Filter the hand's encoder-backed joints by --fingers / --joints."""
    if args.fingers and args.joints:
        raise SystemExit("Cannot specify both --fingers and --joints.")

    encoder_backed = list(hand._encoder_backed_joints())
    if not encoder_backed:
        raise SystemExit(
            "No encoder-backed joints configured on this hand "
            "(set joint_encoder_joints in config.yaml)."
        )

    if args.fingers is None and args.joints is None:
        return encoder_backed

    if args.fingers:
        requested = {j for finger in args.fingers for j in FINGER_TO_JOINTS[finger]}
    else:
        requested = set(args.joints)

    selected = [j for j in encoder_backed if j in requested]
    if not selected:
        raise SystemExit(
            f"No encoder-backed joints intersect the requested set "
            f"({sorted(requested)}). Encoder-backed: {encoder_backed}."
        )
    return selected


class JointFeedbackSliderUI:
    def __init__(
        self,
        root: tk.Tk,
        hand: OrcaHandJointFeedback,
        slider_joints: List[str],
        initial_gains: tuple[float, float, float],
    ):
        self.hand = hand
        self.root = root
        self.slider_joints = slider_joints
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
        measured = self.hand.get_measured_joints()
        for joint in self.slider_joints:
            self.joint_values[joint] = tk.DoubleVar(value=measured.get(joint, 0.0))

    def _build_ui(self) -> None:
        self.root.title("Joint-feedback slider")
        self.root.geometry("640x720")

        sliders_frame = ttk.LabelFrame(self.root, text="targets", padding=6)
        sliders_frame.pack(fill=tk.X, padx=6, pady=6)
        for joint in self.slider_joints:
            rom_min, rom_max = self.hand.config.joint_roms_dict[joint]
            row = ttk.Frame(sliders_frame, padding=4)
            row.pack(fill=tk.X)
            ttk.Label(row, text=joint, width=14).pack(side=tk.LEFT)
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
        for joint in self.slider_joints:
            row = ttk.Frame(readback)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=joint, width=14).pack(side=tk.LEFT)
            self.measured_labels[joint] = ttk.Label(row, text="meas: --", width=14)
            self.measured_labels[joint].pack(side=tk.LEFT)
            self.trim_labels[joint] = ttk.Label(row, text="", width=14)
            self.trim_labels[joint].pack(side=tk.LEFT)

        # trace_add only fires on writes after binding, so seed the labels
        # by manually firing each slider callback once.
        for joint in self.slider_joints:
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
            self.hand.set_pid_gains(Kp=kp, Ki=ki, correction_max_deg=corr)
        except Exception as exc:
            self.apply_status_var.set(f"set_pid_gains failed: {exc}")
            return
        self.apply_status_var.set(
            f"applied: Kp={kp} Ki={ki} correction_max={corr:.1f}°  max_current={mc}mA"
        )

    def _on_slider(self, joint: str, value_str) -> None:
        try:
            value_deg = float(value_str)
        except ValueError:
            return
        try:
            self.hand.set_joint_positions(
                OrcaJointPositions.from_dict({joint: value_deg}),
            )
        except Exception as exc:
            logging.warning("set_joint_positions(%s=%.1f°) failed: %s", joint, value_deg, exc)

    def _schedule_refresh(self) -> None:
        self._refresh()
        self.root.after(REFRESH_MS, self._schedule_refresh)

    def _refresh(self) -> None:
        measured = self.hand.get_measured_joints()
        correction = self.hand.get_loop_correction()
        for joint in self.slider_joints:
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
    except JointFeedbackConnectError as exc:
        print(f"FAIL: {exc}")
        return 1
    print(msg)
    if not success:
        return 1

    if args.max_current is not None:
        hand.set_max_current(args.max_current)
        print(f"  → max_current = {args.max_current} mA")

    slider_joints = _resolve_joint_set(args, hand)
    hand.set_pid_gains(
        Kp=args.Kp, Ki=args.Ki, correction_max_deg=args.correction_max_deg,
    )
    print(
        f"  → gains Kp={args.Kp} Ki={args.Ki} "
        f"correction_max={args.correction_max_deg:.1f}°"
    )
    print(f"  → sliders for {len(slider_joints)} joint(s): {slider_joints}")

    root = tk.Tk()
    JointFeedbackSliderUI(
        root, hand, slider_joints,
        initial_gains=(args.Kp, args.Ki, args.correction_max_deg),
    )
    try:
        root.mainloop()
    finally:
        hand.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
