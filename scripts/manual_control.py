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
window around the motor's position at startup, clamped to the motor's usable
travel. This is a tendon bring-up aid for nudging a single motor and watching
its tendon respond, not a way to pose the hand. The hand is still detected the
usual way, but the closed-loop controller stays disengaged.

With ``--all`` (or ``--hand`` given twice is not supported — attach exactly
the hands you want and use ``--all``), up to two attached hands share one
window as side-by-side columns, each driving its own hand independently.

Usage:
    uv run python scripts/manual_control.py CONFIG
    uv run python scripts/manual_control.py CONFIG --fingers ring
    uv run python scripts/manual_control.py CONFIG --joints ring_mcp ring_pip --max-current 600
    uv run python scripts/manual_control.py CONFIG --motor-space
    uv run python scripts/manual_control.py --all          # up to two hands, side by side
"""
from __future__ import annotations

import argparse
import dataclasses
import logging
import math
import sys
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk
from typing import List

import numpy as np

from orca_core import JointGains, OrcaHandJointFeedback
from orca_core import JointFeedbackConnectError
from orca_core.hardware.joint_encoder_client import EncodersNotAvailableError
from orca_core.joint_position import OrcaJointPositions
from orca_core.utils.cli import add_hand_arguments, create_fleet_from_args

REFRESH_MS = 100

# Half-width of each motor-space slider, in motor radians around the position
# read at startup. Deliberately tight: this mode is for precise nudges.
MOTOR_SLIDER_SPAN_RAD = 1.0

MAX_HANDS = 2

# Palette ported from orca_ui's frontend/src/theme/theme.css (same tokens,
# solid colors instead of translucent panels since ttk can't blend with the
# desktop behind it). Keep in sync by eye if that file's palette moves.
BG = "#1a1c2a"
PANEL = "#242739"
PANEL_BORDER = "#343850"
TEXT = "#dfe3e8"
TEXT_BRIGHT = "#ffffff"
DIM = "#7f8ea2"
ACCENT = "#22d3ee"
OK = "#34d399"
WARN = "#c8a870"
ERR = "#d4878a"

# Space Mono is orca_ui's font, loaded there via @fontsource; a desktop script
# can't bundle a webfont without adding a dependency, so this only picks it up
# if it happens to be installed as a system font and otherwise falls back to
# whatever monospace font the OS ships.
_MONO_CANDIDATES = [
    "Space Mono", "SF Mono", "Menlo", "Consolas", "DejaVu Sans Mono", "Courier New",
]


def _pick_font() -> str:
    available = set(tkfont.families())
    for name in _MONO_CANDIDATES:
        if name in available:
            return name
    return "TkFixedFont"


class ModernSlider(tk.Canvas):
    """Small, modern-looking horizontal slider: a rounded track filled up to
    the current value, dragged by a round thumb.

    ttk's only cross-platform-colorable theme (clam) draws a flat rectangle
    with no fill indicator, and pushing it further (a custom PhotoImage
    slider element) means fighting ttk's theme engine for something a canvas
    can just draw directly. No new dependency — tk.Canvas ships with Python.
    Drives (and is driven by) a ``tk.DoubleVar``, like ttk.Scale: dragging it
    calls ``var.set()`` (which fires any other trace on the same var, e.g. a
    value-label), and an external ``var.set()`` elsewhere redraws it too.
    """

    THUMB_R = 8
    TRACK_W = 6
    HEIGHT = 24

    def __init__(self, parent, from_, to, variable, command=None):
        super().__init__(parent, height=self.HEIGHT, bg=BG, highlightthickness=0, cursor="hand2")
        self._from = from_
        self._to = to
        self._var = variable
        self._command = command
        self._var.trace_add("write", lambda *_: self._redraw())
        self.bind("<Configure>", lambda _e: self._redraw())
        self.bind("<Button-1>", self._on_pointer)
        self.bind("<B1-Motion>", self._on_pointer)

    def _frac(self, value: float) -> float:
        span = self._to - self._from
        return 0.0 if span == 0 else min(1.0, max(0.0, (value - self._from) / span))

    def _redraw(self) -> None:
        width = self.winfo_width()
        if width <= 1:  # not yet laid out
            return
        self.delete("all")
        y = self.HEIGHT // 2
        x0, x1 = self.THUMB_R, width - self.THUMB_R
        value = min(self._to, max(self._from, self._var.get()))
        thumb_x = x0 + self._frac(value) * (x1 - x0)
        self.create_line(x0, y, x1, y, width=self.TRACK_W, capstyle=tk.ROUND, fill=PANEL)
        if thumb_x > x0:
            self.create_line(x0, y, thumb_x, y, width=self.TRACK_W, capstyle=tk.ROUND, fill=ACCENT)
        self.create_oval(
            thumb_x - self.THUMB_R, y - self.THUMB_R, thumb_x + self.THUMB_R, y + self.THUMB_R,
            fill=ACCENT, outline=BG, width=2,
        )

    def _on_pointer(self, event) -> None:
        width = self.winfo_width()
        x0, x1 = self.THUMB_R, width - self.THUMB_R
        frac = 0.0 if x1 <= x0 else min(1.0, max(0.0, (event.x - x0) / (x1 - x0)))
        value = self._from + frac * (self._to - self._from)
        self._var.set(value)
        if self._command is not None:
            self._command(value)


def _apply_theme(root: tk.Tk) -> None:
    """Dark/monospace ttk theme matching orca_ui's frontend/src/theme/theme.css.
    stdlib-only (ttk and PhotoImage both ship with Python).
    """
    font_family = _pick_font()
    root.configure(bg=BG)

    style = ttk.Style(root)
    style.theme_use("clam")  # only clam/alt honor color overrides cross-platform

    style.configure(".", background=BG, foreground=TEXT, font=(font_family, 10))
    style.configure("TFrame", background=BG)
    style.configure("TLabel", background=BG, foreground=TEXT)
    style.configure("TLabelframe", background=BG, bordercolor=PANEL_BORDER)
    style.configure(
        "TLabelframe.Label", background=BG, foreground=TEXT_BRIGHT,
        font=(font_family, 10, "bold"),
    )
    style.configure(
        "TButton", background=PANEL, foreground=TEXT, bordercolor=PANEL_BORDER,
        focuscolor=PANEL, padding=6,
    )
    style.map("TButton", background=[("active", "#2c3049")])
    style.configure(
        "TEntry", fieldbackground=PANEL, foreground=TEXT, bordercolor=PANEL_BORDER,
        insertcolor=TEXT,
    )

    style.configure(
        "Accent.TButton", background="#1b3a42", foreground=ACCENT, bordercolor="#2f5e68",
    )
    style.map(
        "Accent.TButton",
        background=[("disabled", PANEL), ("active", "#204552")],
        foreground=[("disabled", DIM), ("!disabled", ACCENT)],
        bordercolor=[("disabled", PANEL_BORDER)],
    )
    style.configure(
        "Danger.TButton", background="#3a2426", foreground=ERR, bordercolor="#5e3436",
    )
    style.map(
        "Danger.TButton",
        background=[("disabled", PANEL), ("active", "#452b2d")],
        foreground=[("disabled", DIM), ("!disabled", ERR)],
        bordercolor=[("disabled", PANEL_BORDER)],
    )

    style.configure("Header.TLabel", background=BG, foreground=TEXT_BRIGHT, font=(font_family, 13, "bold"))
    style.configure("Dim.TLabel", background=BG, foreground=DIM, font=(font_family, 9))
    style.configure("Ok.TLabel", background=BG, foreground=OK)
    style.configure("Warn.TLabel", background=BG, foreground=WARN, font=(font_family, 9, "bold"))
    style.configure("Err.TLabel", background=BG, foreground=ERR, font=(font_family, 9, "bold"))
    style.configure(
        "Banner.TLabel", background="#3a2f22", foreground=WARN,
        font=(font_family, 10, "bold"), padding=8,
    )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    add_hand_arguments(p, all_flag=True)
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
    return p


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


def _build_columns(root: tk.Tk, labels: List[str]) -> List[ttk.Frame]:
    """One frame per hand, side by side, separated by a vertical rule.

    A single hand gets no label and no rule — this is the zero-ceremony case,
    visually identical to before columns existed. With more than one, each
    column is headed by its hand's identity so the operator always knows
    which slider drives which physical hand.
    """
    container = ttk.Frame(root)
    container.pack(fill=tk.BOTH, expand=True)

    columns = []
    for i, label in enumerate(labels):
        grid_col = i * 2
        if i > 0:
            ttk.Separator(container, orient=tk.VERTICAL).grid(
                row=0, column=grid_col - 1, rowspan=2, sticky="ns", padx=6
            )
        if len(labels) > 1:
            ttk.Label(
                container, text=label, font=("", 11, "bold"), padding=(4, 4, 0, 2)
            ).grid(row=0, column=grid_col, sticky="w")
        col = ttk.Frame(container)
        col.grid(row=1, column=grid_col, sticky="nsew")
        container.columnconfigure(grid_col, weight=1)
        columns.append(col)
    return columns


class HandControlUI:
    """Plain motor-only slider UI: one slider per joint + torque buttons.

    A joint whose motor lacks calibration data can't be commanded in joint
    space (see ``_joint_calibrated``) — ``_joint_to_motor_pos`` would emit a
    ``None`` target that ``_set_motor_pos`` silently drops, so the slider
    would move with no effect. Those joints get a DIRECT-tagged slider that
    writes the motor position straight through instead (the same fallback
    ``--motor-space`` uses), rather than a slider that looks live but isn't.
    """

    BADGE_WIDTH = 8

    def __init__(self, parent: tk.Widget, hand):
        self.hand = hand
        self.joint_roms = hand.config.joint_roms_dict
        self.joint_ids = hand.config.joint_ids
        self.joint_values = {joint: tk.DoubleVar() for joint in self.joint_ids}
        self.direct_motor: dict[str, int] = {}  # joint -> motor_id, for uncalibrated joints
        self.create_ui(parent)

    def _joint_calibrated(self, joint: str) -> bool:
        """Whether this joint's motor has the calibration data
        ``_joint_to_motor_pos`` needs. Mirrors the check in
        ``OrcaHand._joint_to_motor_pos``.
        """
        motor_id = self.hand.config.joint_to_motor_map.get(joint)
        if motor_id is None:
            return False
        limits = self.hand.motor_limits_dict.get(motor_id) or [None, None]
        if any(limit is None for limit in limits):
            return False
        return bool(self.hand.calibration.joint_to_motor_ratios_dict.get(motor_id))

    def create_ui(self, parent: tk.Widget):
        # Window title/geometry are set once at the top level (main()), not
        # here — this may be one of two side-by-side hand columns, not the
        # whole window.
        _apply_theme(parent)

        ttk.Label(parent, text="ORCA HAND CONTROL", style="Header.TLabel").pack(
            anchor=tk.W, padx=12, pady=(12, 0)
        )

        uncalibrated = {j for j in self.joint_ids if not self._joint_calibrated(j)}
        if uncalibrated:
            ttk.Label(
                parent,
                text=(
                    f"⚠ {len(uncalibrated)}/{len(self.joint_ids)} joint(s) uncalibrated: "
                    "DIRECT motor control below. Run scripts/calibrate.py to fix."
                ),
                style="Banner.TLabel", wraplength=420, justify=tk.LEFT,
            ).pack(fill=tk.X, padx=12, pady=(8, 0))

        torque_frame = ttk.Frame(parent)
        torque_frame.pack(pady=12)

        self.enable_button = ttk.Button(
            torque_frame, text="Enable Torque", style="Accent.TButton", command=self.enable_torque
        )
        self.enable_button.pack(side=tk.LEFT, padx=5)

        self.disable_button = ttk.Button(
            torque_frame, text="Disable Torque", style="Danger.TButton", command=self.disable_torque
        )
        self.disable_button.pack(side=tk.LEFT, padx=5)

        self.torque_label = ttk.Label(torque_frame, text="TORQUE OFF", style="Dim.TLabel")
        self.torque_label.pack(side=tk.LEFT, padx=(10, 0))

        sliders_frame = ttk.Frame(parent)
        sliders_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=10)

        joint_positions = self.hand.get_joint_position().as_dict()
        motor_positions = self.hand.get_motor_pos(as_dict=True)

        for joint in self.joint_ids:
            calibrated = joint not in uncalibrated
            frame = ttk.Frame(sliders_frame)
            frame.pack(fill=tk.X, pady=3)

            # Fixed-width name + badge columns on every row, badge column
            # always present (blank when calibrated): the slider's expand=True
            # then claims identical space on every row regardless of state,
            # instead of a DIRECT-tagged row's slider being visibly shorter.
            ttk.Label(
                frame, text=joint, width=14, style="TLabel" if calibrated else "Warn.TLabel",
            ).pack(side=tk.LEFT)
            ttk.Label(
                frame, text="" if calibrated else "DIRECT", width=self.BADGE_WIDTH,
                style="Warn.TLabel", anchor=tk.W,
            ).pack(side=tk.LEFT, padx=(2, 4))

            if calibrated:
                rom_min, rom_max = self.joint_roms[joint]
                seed = joint_positions.get(joint)
                if seed is not None:
                    self.joint_values[joint].set(seed)
                slider = ModernSlider(
                    frame, from_=rom_min, to=rom_max,
                    variable=self.joint_values[joint],
                    command=lambda value, joint=joint: self.update_joint_position(joint, value),
                )
            else:
                motor_id = self.hand.config.joint_to_motor_map.get(joint)
                self.direct_motor[joint] = motor_id
                current = motor_positions.get(motor_id, 0.0)
                low, high, _clamped = _motor_slider_range(self.hand, motor_id, current)
                self.joint_values[joint].set(current)
                slider = ModernSlider(
                    frame, from_=low, to=high,
                    variable=self.joint_values[joint],
                    command=lambda value, joint=joint: self.update_joint_position(joint, value),
                )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)

            value_label = ttk.Label(
                frame, text=f"{self.joint_values[joint].get():.1f}", width=7, style="Dim.TLabel",
            )
            value_label.pack(side=tk.RIGHT)

            self.joint_values[joint].trace_add("write", lambda *args, joint=joint, label=value_label: self.update_value_label(joint, label))

        # Torque on by default: sliders that visibly do nothing until a
        # separate click is the exact confusion this UI now tries to avoid.
        self.enable_torque()

    def _set_torque_state(self, enabled: bool) -> None:
        """Reflect torque state in the buttons (grey out whichever action is
        already in effect) and a TORQUE ON/OFF label, so the state is visible
        without having to move a slider and see whether anything happens.
        """
        self.torque_enabled = enabled
        self.enable_button.state(["disabled" if enabled else "!disabled"])
        self.disable_button.state(["!disabled" if enabled else "disabled"])
        self.torque_label.config(
            text="TORQUE ON" if enabled else "TORQUE OFF",
            style="Ok.TLabel" if enabled else "Dim.TLabel",
        )

    def enable_torque(self):
        self.hand.enable_torque()
        print("Torque enabled.")
        joint_positions = self.hand.get_joint_position().as_dict()
        motor_positions = self.hand.get_motor_pos(as_dict=True)
        for joint, var in self.joint_values.items():
            if joint in self.direct_motor:
                var.set(motor_positions.get(self.direct_motor[joint], var.get()))
            else:
                pos = joint_positions.get(joint)
                if pos is not None:
                    var.set(pos)
        self._set_torque_state(True)

    def disable_torque(self):
        self.hand.disable_torque()
        print("Torque disabled.")
        self._set_torque_state(False)

    def update_joint_position(self, joint, value):
        try:
            if joint in self.direct_motor:
                self.hand.write_motor_pos([self.direct_motor[joint]], np.array([float(value)]))
            else:
                self.hand.set_joint_positions({joint: float(value)})
        except Exception as e:
            print(f"Error updating joint {joint}: {e}")

    def update_value_label(self, joint, label):
        value = self.joint_values[joint].get()
        label.config(text=f"{value:.1f}")


def _motor_slider_range(hand, motor: int, current: float) -> tuple[float, float, bool]:
    """Slider window around ``current``, clamped to the motor's usable travel.

    Calibrated hard limits win; otherwise the motor family's own position range
    bounds it. Returns ``(from_, to, clamped)``.
    """
    low = current - MOTOR_SLIDER_SPAN_RAD
    high = current + MOTOR_SLIDER_SPAN_RAD

    limits = hand.motor_limits_dict.get(motor) or [None, None]
    if all(limit is not None for limit in limits):
        bounds = (min(limits), max(limits))
    else:
        bounds = hand.motor_client.position_range_rad
    if bounds is None:
        return low, high, False

    clamped_low = max(low, min(bounds))
    clamped_high = min(high, max(bounds))
    if clamped_high <= clamped_low:
        return low, high, False
    return clamped_low, clamped_high, (clamped_low, clamped_high) != (low, high)


class MotorSliderUI:
    """Motor-space slider UI: one slider per motor over a narrow window around
    its startup position, plus torque buttons."""

    def __init__(self, parent: tk.Widget, hand):
        self.hand = hand
        self.motor_values = {motor: tk.DoubleVar() for motor in hand.config.motor_ids}
        self.create_ui(parent)

    def create_ui(self, parent: tk.Widget):
        # Window title/geometry are set once at the top level (main()), not
        # here — this may be one of two side-by-side hand columns, not the
        # whole window.
        _apply_theme(parent)

        ttk.Label(parent, text="ORCA HAND MOTOR CONTROL", style="Header.TLabel").pack(
            anchor=tk.W, padx=12, pady=(12, 0)
        )

        torque_frame = ttk.Frame(parent)
        torque_frame.pack(pady=12)

        self.enable_button = ttk.Button(
            torque_frame, text="Enable Torque", style="Accent.TButton", command=self.enable_torque
        )
        self.enable_button.pack(side=tk.LEFT, padx=5)
        self.disable_button = ttk.Button(
            torque_frame, text="Disable Torque", style="Danger.TButton", command=self.disable_torque
        )
        self.disable_button.pack(side=tk.LEFT, padx=5)

        self.torque_label = ttk.Label(torque_frame, text="TORQUE OFF", style="Dim.TLabel")
        self.torque_label.pack(side=tk.LEFT, padx=(10, 0))

        sliders_frame = ttk.Frame(parent)
        sliders_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=10)

        current_motor_pos = self.hand.get_motor_pos(as_dict=True)
        any_clamped = False
        for motor in self.hand.config.motor_ids:
            self.motor_values[motor].set(current_motor_pos[motor])
            low, high, clamped = _motor_slider_range(
                self.hand, motor, current_motor_pos[motor]
            )
            any_clamped = any_clamped or clamped

            frame = ttk.Frame(sliders_frame)
            frame.pack(fill=tk.X, pady=5)

            label = f"Motor {motor}*" if clamped else f"Motor {motor}"
            ttk.Label(frame, text=label, width=15).pack(side=tk.LEFT)

            slider = ModernSlider(
                frame,
                from_=low,
                to=high,
                variable=self.motor_values[motor],
                command=lambda value, m=motor: self.update_motor_position(m, value),
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)

            value_label = ttk.Label(frame, text=f"{current_motor_pos[motor]:.1f}", width=8, style="Dim.TLabel")
            value_label.pack(side=tk.RIGHT)

            self.motor_values[motor].trace_add(
                "write",
                lambda *args, m=motor, label=value_label: self.update_value_label(m, label),
            )

        if any_clamped:
            ttk.Label(
                sliders_frame, text="* range clamped to the motor's usable travel",
                style="Dim.TLabel",
            ).pack(anchor=tk.W, pady=(6, 0))

        # _run_motor_space() already enabled torque on the hand before this
        # UI was built; sync the buttons/label to that state.
        self.enable_torque()

    def _set_torque_state(self, enabled: bool) -> None:
        self.torque_enabled = enabled
        self.enable_button.state(["disabled" if enabled else "!disabled"])
        self.disable_button.state(["!disabled" if enabled else "disabled"])
        self.torque_label.config(
            text="TORQUE ON" if enabled else "TORQUE OFF",
            style="Ok.TLabel" if enabled else "Dim.TLabel",
        )

    def enable_torque(self):
        self.hand.enable_torque()
        print("Torque enabled.")
        self._set_torque_state(True)

    def disable_torque(self):
        self.hand.disable_torque()
        print("Torque disabled.")
        self._set_torque_state(False)

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
    """Closed-loop slider UI: one row per joint carries both the target
    slider and its live encoder readback (measured angle + PI trim) —
    everything about a joint stays in one line rather than two stacked
    sections, which is what made this window taller than a laptop screen.
    """

    def __init__(
        self,
        parent: tk.Widget,
        hand: OrcaHandJointFeedback,
        slider_joints: List[str],
        initial_gains: JointGains,
    ):
        self.hand = hand
        self.widget = parent
        self.slider_joints = slider_joints
        self.joint_values: dict[str, tk.DoubleVar] = {}
        self.readback_labels: dict[str, ttk.Label] = {}

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
        self._build_ui(parent)
        self._schedule_refresh()

    def _seed_initial_targets(self) -> None:
        """Latch each slider to the current measured angle so opening the
        window doesn't yank the joint. Joints the loop skipped (no encoder
        anchor) seed from the motor-derived pose instead of a fixed default.

        Sanitises the seed: a NaN or out-of-ROM reading (a slot that hasn't
        produced a clean frame yet) is clamped into range so the slider
        starts somewhere sane instead of off the visible track.
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

    def _build_ui(self, parent: tk.Widget) -> None:
        # Window title/geometry are set once at the top level (main()), not
        # here — this may be one of two side-by-side hand columns, not the
        # whole window.
        _apply_theme(parent)

        ttk.Label(parent, text="JOINT-FEEDBACK CONTROL", style="Header.TLabel").pack(
            anchor=tk.W, padx=10, pady=(10, 0)
        )

        sliders_frame = ttk.LabelFrame(parent, text="targets + encoder", padding=6)
        sliders_frame.pack(fill=tk.X, padx=6, pady=6)
        sliders_frame.columnconfigure(2, weight=1)

        for row, joint in enumerate(self.slider_joints):
            rom_min, rom_max = self.hand.config.joint_roms_dict[joint]
            ttk.Label(sliders_frame, text=joint, width=12).grid(
                row=row, column=0, sticky="w", pady=1
            )
            ttk.Label(sliders_frame, text=f"{rom_min:+4.0f}°", width=5, style="Dim.TLabel").grid(
                row=row, column=1
            )
            slider = ModernSlider(
                sliders_frame,
                from_=rom_min, to=rom_max,
                variable=self.joint_values[joint],
                command=lambda v, j=joint: self._on_slider(j, v),
            )
            slider.grid(row=row, column=2, sticky="ew", padx=4)
            ttk.Label(sliders_frame, text=f"{rom_max:+4.0f}°", width=5, style="Dim.TLabel").grid(
                row=row, column=3
            )
            target_label = ttk.Label(sliders_frame, text="--", width=7, style="Ok.TLabel")
            target_label.grid(row=row, column=4, padx=(6, 0))
            readback_label = ttk.Label(
                sliders_frame, text="m:--  t:--", width=15, font=("Menlo", 10), style="Dim.TLabel",
            )
            readback_label.grid(row=row, column=5, padx=(6, 0))
            self.readback_labels[joint] = readback_label

            self.joint_values[joint].trace_add(
                "write",
                lambda *_, j=joint, lbl=target_label: lbl.config(
                    text=f"→{self.joint_values[j].get():+5.1f}°"
                ),
            )

        # trace_add only fires on writes after binding, so seed the labels
        # by manually firing each slider callback once.
        for joint in self.slider_joints:
            self._on_slider(joint, self.joint_values[joint].get())

        tuning = ttk.LabelFrame(parent, text="tuning", padding=6)
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
        ttk.Button(button_row, text="Apply", style="Accent.TButton", command=self._apply_tuning).pack(
            side=tk.LEFT
        )
        ttk.Label(button_row, textvariable=self.apply_status_var, style="Dim.TLabel").pack(
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
        self.widget.after(REFRESH_MS, self._schedule_refresh)

    def _refresh(self) -> None:
        try:
            measured = self.hand.get_measured_joints()
            correction = self.hand.get_loop_correction()
        except RuntimeError as exc:
            for joint in self.slider_joints:
                self.readback_labels[joint].config(text="e-stop", style="Err.TLabel")
            logging.warning("joint loop unavailable: %s", exc)
            return
        for joint in self.slider_joints:
            m = measured.get(joint, float("nan"))
            t = correction.get(joint, 0.0)
            self.readback_labels[joint].config(
                text=f"m:{m:+5.1f}° t:{t:+4.1f}°", style="Dim.TLabel"
            )


def _hand_label(hand_id: str, hand) -> str:
    return f"{hand_id}  ({hand.config.type or '?'})"


def _run_feedback(parent: tk.Widget, args: argparse.Namespace, hand: OrcaHandJointFeedback) -> None:
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

    JointFeedbackSliderUI(
        parent, hand, slider_joints,
        initial_gains=hand.config.joint_gains_baseline,
    )


def _run_motor_space(args: argparse.Namespace) -> int:
    # Motor-space nudging wants the motor bus alone, not the encoder and
    # tactile links the factory opens for a sensing hand.
    fleet = create_fleet_from_args(args, engage_feedback=False, engage_sensors=False)
    _refuse_if_too_many(fleet)

    for hand in fleet:
        success, msg = hand.connect()
        print(msg)
        if not success:
            print("Failed to connect to the hand.")
            return 1

    try:
        root = tk.Tk()
        root.title("Orca Hand Motor Control")
        columns = _build_columns(
            root, [_hand_label(hand_id, hand) for hand_id, hand in zip(fleet.ids, fleet)]
        )
        for col, hand in zip(columns, fleet):
            # Minimal bring-up only: this mode runs pre-calibration on hands
            # with unseated tendons, so it must never calibrate or move the hand.
            hand.enable_torque()
            hand.set_control_mode(hand.config.control_mode)
            hand.set_max_current(hand.config.max_current)
            MotorSliderUI(col, hand)
        root.mainloop()
        return 0
    finally:
        for hand in fleet:
            hand.disconnect()


def _refuse_if_too_many(fleet) -> None:
    if len(fleet) > MAX_HANDS:
        raise SystemExit(
            f"manual_control.py shows at most {MAX_HANDS} hands side by side; "
            f"{len(fleet)} are attached ({', '.join(fleet.ids)}). Name one with "
            "--hand HAND_ID, or unplug the rest."
        )


def _feedback_only_overrides(parser: argparse.ArgumentParser, args: argparse.Namespace) -> list[str]:
    """Names of the feedback-only tuning flags the caller actually set."""
    return [
        flag
        for flag, dest in (("--Kp", "Kp"), ("--Ki", "Ki"),
                           ("--correction-max-deg", "correction_max_deg"))
        if getattr(args, dest) != parser.get_default(dest)
    ]


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s"
    )

    if args.motor_space:
        return _run_motor_space(args)

    fleet = create_fleet_from_args(args)
    _refuse_if_too_many(fleet)

    given = _feedback_only_overrides(parser, args)
    if given and not any(isinstance(hand, OrcaHandJointFeedback) for hand in fleet):
        print(
            f"FAIL: {', '.join(given)} tune the closed-loop controller, but no "
            f"attached hand loaded with joint feedback."
        )
        return 1

    overrides = {}
    if args.encoder_port is not None:
        overrides["encoder_serial_port"] = args.encoder_port
    if args.max_current is not None:
        overrides["max_current"] = args.max_current
    if overrides:
        for hand in fleet:
            hand.config = dataclasses.replace(hand.config, **overrides)

    for hand_id, hand in zip(fleet.ids, fleet):
        try:
            success, msg = hand.connect()
        except (JointFeedbackConnectError, EncodersNotAvailableError) as exc:
            print(f"[{hand_id}] FAIL: {exc}")
            return 1
        print(f"[{hand_id}] {msg}" if len(fleet) > 1 else msg)
        if not success:
            print(f"[{hand_id}] Failed to connect to the hand.")
            return 1

    try:
        any_feedback = any(isinstance(hand, OrcaHandJointFeedback) for hand in fleet)
        title = "Joint-feedback slider" if any_feedback else "Orca Hand Control"
        if len(fleet) > 1:
            title += " (2 hands)"

        root = tk.Tk()
        root.title(title)
        columns = _build_columns(
            root, [_hand_label(hand_id, hand) for hand_id, hand in zip(fleet.ids, fleet)]
        )
        for col, hand in zip(columns, fleet):
            if isinstance(hand, OrcaHandJointFeedback):
                _run_feedback(col, args, hand)
            else:
                HandControlUI(col, hand)
        root.mainloop()
        return 0
    finally:
        for hand in fleet:
            hand.disconnect()


if __name__ == "__main__":
    sys.exit(main())
