"""Drive the ORCA Hand from the PC with on-screen sliders.

Joint space (default): the hand class is chosen from the config by
:func:`orca_core.load_hand`. A hand that declares joint encoders (and leaves
feedback enabled) is built as a closed-loop ``OrcaHandJointFeedback`` /
``OrcaHandFull`` and gets one slider per encoder-backed joint with live encoder
readback and a Kp / Ki / correction_max / max_current tuning panel;
``--fingers`` / ``--joints`` filters that slider set. Otherwise the hand is a
motor-only ``OrcaHand`` / ``OrcaHandTouch`` and gets one slider per joint plus
torque enable/disable.

Motor space (``--motor-space``): one slider per motor, each spanning a narrow
window around the motor's position at startup. This is a tendon bring-up aid
for nudging a single motor and watching its tendon respond, not a way to pose
the hand. It talks to the motor bus only — no encoders, no tactile.

Usage:
    uv run python scripts/manual_control.py CONFIG
    uv run python scripts/manual_control.py CONFIG --fingers ring
    uv run python scripts/manual_control.py CONFIG --joints ring_mcp ring_pip --max-current 600
    uv run python scripts/manual_control.py CONFIG --motor-space
"""
from __future__ import annotations

import argparse
import dataclasses
import logging
import math
import sys
import tkinter as tk
from tkinter import ttk
from typing import List

import numpy as np

from orca_core import JointGains, OrcaHandJointFeedback
from orca_core.utils.cli import add_hand_arguments, create_hand_from_args
from orca_core import JointFeedbackConnectError
from orca_core.hardware.joint_encoder_client import EncodersNotAvailableError
from orca_core.joint_position import OrcaJointPositions

REFRESH_MS = 100

# Half-width of each motor-space slider, in motor radians around the position
# read at startup. Deliberately tight: this mode is for precise nudges.
MOTOR_SLIDER_SPAN_RAD = 1.0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    add_hand_arguments(p)
    p.add_argument(
        "--motor-space", action="store_true",
        help="Slider per motor instead of per joint (tendon bring-up; motor bus only).",
    )
    p.add_argument("--encoder-port", default=None, help="Feedback hands only.")
    p.add_argument("--max-current", type=int, default=None, help="Feedback hands only.")
    p.add_argument(
        "--Kp", type=float, default=None,
        help="Feedback hands only. Overrides the configured gain on every joint.",
    )
    p.add_argument(
        "--Ki", type=float, default=None,
        help="Feedback hands only. Overrides the configured gain on every joint.",
    )
    p.add_argument(
        "--correction-max-deg", type=float, default=None,
        help="Feedback hands only. Overrides the configured clamp on every joint.",
    )
    p.add_argument(
        "--fingers", nargs="+",
        help="Show sliders for these fingers only (feedback hands).",
    )
    p.add_argument(
        "--joints", nargs="+",
        help="Show sliders for these joints only (feedback hands).",
    )
    return p.parse_args()


def _finger_joint_map(joint_ids: List[str]) -> dict[str, List[str]]:
    """Group the config's joint names by finger prefix ({finger}_{type}; bare wrist)."""
    mapping: dict[str, List[str]] = {}
    for joint in joint_ids:
        finger = joint.split("_", 1)[0]
        mapping.setdefault(finger, []).append(joint)
    return mapping


def _resolve_joint_set(
    args: argparse.Namespace, hand: OrcaHandJointFeedback,
) -> List[str]:
    """Filter the hand's encoder-backed joints by --fingers / --joints."""
    if args.fingers and args.joints:
        raise SystemExit("Cannot specify both --fingers and --joints.")

    encoder_backed = hand.encoder_backed_joints
    if not encoder_backed:
        raise SystemExit(
            "No encoder-backed joints configured on this hand "
            "(set joint_encoder_joints in config.yaml)."
        )

    if args.fingers is None and args.joints is None:
        return encoder_backed

    if args.fingers:
        finger_map = _finger_joint_map(hand.config.joint_ids)
        unknown = [f for f in args.fingers if f not in finger_map]
        if unknown:
            raise SystemExit(
                f"Unknown finger(s) {unknown}; this hand has {sorted(finger_map)}."
            )
        requested = {j for finger in args.fingers for j in finger_map[finger]}
    else:
        unknown = [j for j in args.joints if j not in hand.config.joint_ids]
        if unknown:
            raise SystemExit(
                f"Unknown joint(s) {unknown}; this hand has {hand.config.joint_ids}."
            )
        requested = set(args.joints)

    selected = [j for j in encoder_backed if j in requested]
    if not selected:
        raise SystemExit(
            f"No encoder-backed joints intersect the requested set "
            f"({sorted(requested)}). Encoder-backed: {encoder_backed}."
        )
    return selected


class HandControlUI:
    """Plain motor-only slider UI: one slider per joint + torque buttons."""

    def __init__(self, root, hand):
        self.hand = hand
        self.joint_roms = hand.config.joint_roms_dict
        self.joint_ids = hand.config.joint_ids
        self.joint_values = {joint: tk.DoubleVar() for joint in self.joint_ids}

        current_joint_positions = self.hand.get_joint_position().as_dict()
        for joint, pos in current_joint_positions.items():
            if joint in self.joint_values:
                self.joint_values[joint].set(pos)
        self.create_ui(root)

    def create_ui(self, root):
        root.title("Orca Hand Control")
        root.geometry("400x600")

        torque_frame = ttk.Frame(root)
        torque_frame.pack(pady=10)

        enable_button = ttk.Button(torque_frame, text="Enable Torque", command=self.enable_torque)
        enable_button.pack(side=tk.LEFT, padx=5)

        disable_button = ttk.Button(torque_frame, text="Disable Torque", command=self.disable_torque)
        disable_button.pack(side=tk.LEFT, padx=5)

        sliders_frame = ttk.Frame(root)
        sliders_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        for joint in self.joint_ids:
            rom_min, rom_max = self.joint_roms[joint]
            frame = ttk.Frame(sliders_frame)
            frame.pack(fill=tk.X, pady=5)

            label = ttk.Label(frame, text=joint, width=15)
            label.pack(side=tk.LEFT)

            slider = ttk.Scale(
                frame,
                from_=rom_min,
                to=rom_max,
                orient=tk.HORIZONTAL,
                variable=self.joint_values[joint],
                command=lambda value, joint=joint: self.update_joint_position(joint, value),
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

            value_label = ttk.Label(frame, text=f"{rom_min:.1f}", width=8)
            value_label.pack(side=tk.RIGHT)

            self.joint_values[joint].trace_add("write", lambda *args, joint=joint, label=value_label: self.update_value_label(joint, label))

    def enable_torque(self):
        self.hand.enable_torque()
        print("Torque enabled.")
        current_joint_positions = self.hand.get_joint_position().as_dict()
        for joint, pos in current_joint_positions.items():
            if joint in self.joint_values:
                self.joint_values[joint].set(pos)

    def disable_torque(self):
        self.hand.disable_torque()
        print("Torque disabled.")

    def update_joint_position(self, joint, value):
        try:
            joint_positions = {joint: float(value)}
            self.hand.set_joint_positions(joint_positions)
        except Exception as e:
            print(f"Error updating joint {joint}: {e}")

    def update_value_label(self, joint, label):
        value = self.joint_values[joint].get()
        label.config(text=f"{value:.1f}")


class MotorSliderUI:
    """Motor-space slider UI: one slider per motor over a narrow window around
    its startup position, plus torque buttons."""

    def __init__(self, root, hand):
        self.hand = hand
        self.motor_values = {motor: tk.DoubleVar() for motor in hand.config.motor_ids}
        self.create_ui(root)

    def create_ui(self, root):
        root.title("Orca Hand Motor Control")
        root.geometry("400x800")

        torque_frame = ttk.Frame(root)
        torque_frame.pack(pady=10)

        ttk.Button(torque_frame, text="Enable Torque", command=self.enable_torque).pack(
            side=tk.LEFT, padx=5
        )
        ttk.Button(torque_frame, text="Disable Torque", command=self.disable_torque).pack(
            side=tk.LEFT, padx=5
        )

        sliders_frame = ttk.Frame(root)
        sliders_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        current_motor_pos = self.hand.get_motor_pos(as_dict=True)
        for motor in self.hand.config.motor_ids:
            self.motor_values[motor].set(current_motor_pos[motor])

            frame = ttk.Frame(sliders_frame)
            frame.pack(fill=tk.X, pady=5)

            ttk.Label(frame, text=f"Motor {motor}", width=15).pack(side=tk.LEFT)

            slider = tk.Scale(
                frame,
                from_=current_motor_pos[motor] - MOTOR_SLIDER_SPAN_RAD,
                to=current_motor_pos[motor] + MOTOR_SLIDER_SPAN_RAD,
                orient=tk.HORIZONTAL,
                variable=self.motor_values[motor],
                command=lambda value, m=motor: self.update_motor_position(m, value),
                length=200,
                resolution=0.1,
                showvalue=False,
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

            value_label = ttk.Label(frame, text=f"{current_motor_pos[motor]:.1f}", width=8)
            value_label.pack(side=tk.RIGHT)

            self.motor_values[motor].trace_add(
                "write",
                lambda *args, m=motor, label=value_label: self.update_value_label(m, label),
            )

    def enable_torque(self):
        self.hand.enable_torque()
        print("Torque enabled.")

    def disable_torque(self):
        self.hand.disable_torque()
        print("Torque disabled.")

    def update_motor_position(self, motor, value):
        try:
            motor_ids = list(self.motor_values)
            positions = np.array([self.motor_values[m].get() for m in motor_ids])
            self.hand.write_motor_pos(motor_ids, positions)
            print(f"Updated motor {motor} to position: {float(value):.1f}")
        except Exception as e:
            print(f"Error updating motor {motor}: {e}")

    def update_value_label(self, motor, label):
        label.config(text=f"{self.motor_values[motor].get():.1f}")


class JointFeedbackSliderUI:
    """Closed-loop slider UI: encoder readback + live PI trim + tuning panel."""

    def __init__(
        self,
        root: tk.Tk,
        hand: OrcaHandJointFeedback,
        slider_joints: List[str],
        initial_gains: JointGains,
    ):
        self.hand = hand
        self.root = root
        self.slider_joints = slider_joints
        self.joint_values: dict[str, tk.DoubleVar] = {}
        self.measured_labels: dict[str, ttk.Label] = {}
        self.trim_labels: dict[str, ttk.Label] = {}

        # Seeded from the live gains, collapsed when uniform across joints
        # (the baseline otherwise). Apply pushes only the fields you edited,
        # so per-joint overrides survive an untouched panel.
        live = hand.get_pid_gains()

        def _seed(pick):
            values = {pick(g) for g in live.values()}
            return values.pop() if len(values) == 1 else pick(initial_gains)

        self._gain_seeds = {
            "kp": _seed(lambda g: g.kp),
            "ki": _seed(lambda g: g.ki),
            "corr": _seed(lambda g: g.correction_max_deg),
        }
        self.kp_var = tk.StringVar(value=f"{self._gain_seeds['kp']}")
        self.ki_var = tk.StringVar(value=f"{self._gain_seeds['ki']}")
        self.corr_var = tk.StringVar(value=f"{self._gain_seeds['corr']}")
        self.max_current_var = tk.StringVar(value=f"{int(hand.config.max_current)}")
        self.apply_status_var = tk.StringVar(value="")

        self._seed_initial_targets()
        self._build_ui()
        self._schedule_refresh()

    def _seed_initial_targets(self) -> None:
        """Latch each slider to the current measured angle so opening the
        window doesn't yank the joint. Joints the loop skipped (no encoder
        anchor) seed from the motor-derived pose instead of a fixed default.

        Sanitises the seed: a NaN or out-of-ROM reading (a slot that hasn't
        produced a clean frame yet) is clamped into range. ttk.Scale on
        macOS silently skips painting a thumb it can't position, which makes
        the slider look invisible even though it still works.
        """
        measured = self.hand.get_measured_joints()
        motor_pose = self.hand.get_joint_position().as_dict()
        for joint in self.slider_joints:
            rom_min, rom_max = self.hand.config.joint_roms_dict[joint]
            value = measured.get(joint, motor_pose.get(joint))
            if value is None or math.isnan(value):
                value = max(rom_min, min(rom_max, 0.0))
            value = max(rom_min, min(rom_max, value))
            self.joint_values[joint] = tk.DoubleVar(value=value)

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

        # Only edited fields are pushed (to every joint); untouched fields
        # keep whatever per-joint gains are in force.
        gain_kwargs = {}
        if kp != self._gain_seeds["kp"]:
            gain_kwargs["Kp"] = kp
        if ki != self._gain_seeds["ki"]:
            gain_kwargs["Ki"] = ki
        if corr != self._gain_seeds["corr"]:
            gain_kwargs["correction_max_deg"] = corr
        if gain_kwargs:
            try:
                self.hand.set_pid_gains(**gain_kwargs)
            except Exception as exc:
                self.apply_status_var.set(f"set_pid_gains failed: {exc}")
                return
            self._gain_seeds.update(kp=kp, ki=ki, corr=corr)

        parts = [f"{k}={v}" for k, v in gain_kwargs.items()] or ["gains unchanged"]
        self.apply_status_var.set(
            f"applied to all joints: {', '.join(parts)}  max_current={mc}mA"
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
        try:
            measured = self.hand.get_measured_joints()
            correction = self.hand.get_loop_correction()
        except RuntimeError as exc:
            for joint in self.slider_joints:
                self.measured_labels[joint].config(text="meas: e-stop")
                self.trim_labels[joint].config(text="trim: --")
            logging.warning("joint loop unavailable: %s", exc)
            return
        for joint in self.slider_joints:
            self.measured_labels[joint].config(
                text=f"meas: {measured.get(joint, float('nan')):+6.1f}°"
            )
            self.trim_labels[joint].config(
                text=f"trim: {correction.get(joint, 0.0):+5.1f}°"
            )


def _run_feedback_ui(args: argparse.Namespace, hand: OrcaHandJointFeedback) -> int:
    # connect() starts the loop but leaves the motors unpowered. The loop
    # latches its target to the measured pose, so powering up holds, not yanks.
    hand.enable_torque()
    hand.set_control_mode(hand.config.control_mode)
    hand.set_max_current(hand.config.max_current)
    print(f"  → torque on, {hand.config.control_mode}, "
          f"max_current = {int(hand.config.max_current)} mA")

    slider_joints = _resolve_joint_set(args, hand)
    # Unset flags leave config.yaml's per-joint gains alone; any flag given
    # applies to every joint, replacing whatever that joint was configured with.
    if any(g is not None for g in (args.Kp, args.Ki, args.correction_max_deg)):
        hand.set_pid_gains(
            Kp=args.Kp, Ki=args.Ki, correction_max_deg=args.correction_max_deg,
        )
    gains = hand.get_pid_gains()
    distinct = {(g.kp, g.ki, g.correction_max_deg) for g in gains.values()}
    if len(distinct) == 1:
        kp, ki, corr = distinct.pop()
        print(f"  → gains Kp={kp} Ki={ki} correction_max={corr:.1f}° (all joints)")
    else:
        print(f"  → per-joint gains active on {len(distinct)} distinct settings:")
        for joint, g in gains.items():
            print(f"      {joint:<12} Kp={g.kp} Ki={g.ki} "
                  f"correction_max={g.correction_max_deg:.1f}°")
    print(f"  → sliders for {len(slider_joints)} joint(s): {slider_joints}")

    root = tk.Tk()
    JointFeedbackSliderUI(
        root, hand, slider_joints,
        initial_gains=hand.config.joint_gains_baseline,
    )
    root.mainloop()
    return 0


def _run_motor_space(args: argparse.Namespace) -> int:
    # Motor-space nudging wants the motor bus alone, not the encoder and
    # tactile links the factory opens for a sensing hand.
    hand = create_hand_from_args(
        args, engage_feedback=False, engage_sensors=False
    )
    success, msg = hand.connect()
    print(msg)
    if not success:
        print("Failed to connect to the hand.")
        return 1

    try:
        # Minimal bring-up only: this mode runs pre-calibration on hands with
        # unseated tendons, so it must never calibrate or move the hand.
        hand.enable_torque()
        hand.set_control_mode(hand.config.control_mode)
        hand.set_max_current(hand.config.max_current)
        root = tk.Tk()
        MotorSliderUI(root, hand)
        root.mainloop()
        return 0
    finally:
        hand.disconnect()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s"
    )

    if args.motor_space:
        return _run_motor_space(args)

    hand = create_hand_from_args(args)
    overrides = {}
    if args.encoder_port is not None:
        overrides["encoder_serial_port"] = args.encoder_port
    if args.max_current is not None:
        overrides["max_current"] = args.max_current
    if overrides:
        hand.config = dataclasses.replace(hand.config, **overrides)

    try:
        success, msg = hand.connect()
    except (JointFeedbackConnectError, EncodersNotAvailableError) as exc:
        print(f"FAIL: {exc}")
        return 1
    print(msg)
    if not success:
        print("Failed to connect to the hand.")
        return 1

    try:
        if isinstance(hand, OrcaHandJointFeedback):
            return _run_feedback_ui(args, hand)
        root = tk.Tk()
        HandControlUI(root, hand)
        root.mainloop()
        return 0
    finally:
        hand.disconnect()


if __name__ == "__main__":
    sys.exit(main())
