"""Health assessment for the hand's sensing hardware.

Turns raw stream data and link counters into structured verdicts a
front-end (terminal script or GUI) can render: per-encoder-slot stream
health, tactile wiring-mismatch inference from press peaks, and
likely-cause diagnosis of a silent or corrupt encoder link.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from orca_core.hardware.hand_serial_link import LinkStats
from orca_core.hardware.joint_encoder_client import EncoderStreamStats
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_ANGLE_MASK,
    AUTO_ENC_NUM_JOINTS,
)
from orca_core.hardware.sensing.types import EncoderReading

DEFAULT_RAIL_EDGE_LSB = 64
"""Angles within this many counts of either 14-bit rail count as "at a rail"."""

DEFAULT_RAIL_EDGE_FRACTION = 0.5
"""A slot at a rail for more than this fraction of a run reads as stuck/floating."""


@dataclass(frozen=True)
class EncoderSlotReport:
    """Health verdict for one encoder slot over an accumulation run.

    ``span`` is ``max - min`` of the observed 14-bit angle in counts;
    ``rail_fraction`` is the fraction of frames the angle sat within the
    rail-edge band. ``reason`` is ``"ok"`` when ``healthy``, else a
    human-readable failure cause.
    """

    slot: int
    frames: int
    span: int
    rail_fraction: float
    parity_errors: int
    angle_error_flags: int
    healthy: bool
    reason: str


class EncoderStreamHealth:
    """Accumulates encoder auto-stream readings into per-slot health verdicts.

    Feed every fresh :class:`EncoderReading` to :meth:`update`, then read a
    pass/fail verdict per slot from :meth:`report`. A slot fails on any
    parity error, any chip angle-error flag, or when its angle sat within
    ``rail_edge_lsb`` counts of either 14-bit rail for more than
    ``rail_edge_fraction`` of the run — the signature of a stuck or floating
    bus (all-0x0000, all-0x3FFF, or the alternating floating case) rather
    than a working encoder chip.
    """

    def __init__(
        self,
        rail_edge_lsb: int = DEFAULT_RAIL_EDGE_LSB,
        rail_edge_fraction: float = DEFAULT_RAIL_EDGE_FRACTION,
    ) -> None:
        self.rail_edge_lsb = rail_edge_lsb
        self.rail_edge_fraction = rail_edge_fraction
        self.frames = 0
        self._extreme = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint64)
        self._parity_bad = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint64)
        self._angle_err = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint64)
        self._angle_min = np.full(AUTO_ENC_NUM_JOINTS, AUTO_ENC_ANGLE_MASK, dtype=np.int64)
        self._angle_max = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.int64)

    def update(self, reading: EncoderReading) -> None:
        angle = reading.raw_counts.astype(np.uint16) & AUTO_ENC_ANGLE_MASK
        self.frames += 1
        self._extreme += (
            (angle <= self.rail_edge_lsb)
            | (angle >= AUTO_ENC_ANGLE_MASK - self.rail_edge_lsb)
        ).astype(np.uint64)
        self._parity_bad += (~np.asarray(reading.parity_ok, dtype=bool)).astype(np.uint64)
        self._angle_err += np.asarray(reading.angle_error, dtype=bool).astype(np.uint64)
        self._angle_min = np.minimum(self._angle_min, angle.astype(np.int64))
        self._angle_max = np.maximum(self._angle_max, angle.astype(np.int64))

    def report(self, slot: int) -> EncoderSlotReport:
        """Verdict for ``slot``. Fails with reason ``"no frames"`` when
        nothing was accumulated."""
        if not self.frames:
            return EncoderSlotReport(
                slot=slot, frames=0, span=0, rail_fraction=0.0,
                parity_errors=0, angle_error_flags=0,
                healthy=False, reason="no frames",
            )
        rail_fraction = float(self._extreme[slot]) / self.frames
        parity_errors = int(self._parity_bad[slot])
        angle_error_flags = int(self._angle_err[slot])
        span = int(self._angle_max[slot] - self._angle_min[slot])
        if parity_errors:
            healthy, reason = False, f"{parity_errors} parity errors"
        elif angle_error_flags:
            healthy, reason = False, f"{angle_error_flags} chip angle-error flags"
        elif rail_fraction > self.rail_edge_fraction:
            healthy, reason = False, f"stuck/floating bus ({rail_fraction * 100:.0f}% at a rail)"
        else:
            healthy, reason = True, "ok"
        return EncoderSlotReport(
            slot=slot, frames=self.frames, span=span, rail_fraction=rail_fraction,
            parity_errors=parity_errors, angle_error_flags=angle_error_flags,
            healthy=healthy, reason=reason,
        )


@dataclass(frozen=True)
class WiringMismatch:
    """A press on ``pressed_finger`` registered under ``sensed_finger`` instead.

    ``pressed_slot``/``sensed_slot`` are the ``finger_to_sensor_id`` entries
    of the two fingers; the likely fix is assigning ``sensed_slot`` to
    ``pressed_finger`` (and re-homing ``pressed_slot``).
    """

    pressed_finger: str
    sensed_finger: str
    pressed_peak_n: float
    sensed_peak_n: float
    pressed_slot: int | None
    sensed_slot: int | None


def detect_wiring_mismatch(
    pressed_finger: str,
    peaks: dict[str, float],
    finger_to_sensor_id: dict[str, int],
    threshold_n: float = 1.0,
) -> WiringMismatch | None:
    """Infer a ``finger_to_sensor_id`` miswiring from per-finger press peaks.

    ``peaks`` maps each finger to the peak ``|fz|`` observed while the
    operator pressed ``pressed_finger``. Returns a :class:`WiringMismatch`
    when the pressed finger stayed below ``threshold_n`` but another finger
    crossed it (the strongest one is reported), else ``None``.
    """
    pressed_peak = peaks.get(pressed_finger, 0.0)
    if pressed_peak >= threshold_n:
        return None
    others = {f: p for f, p in peaks.items() if f != pressed_finger and p >= threshold_n}
    if not others:
        return None
    sensed_finger = max(others, key=others.get)
    return WiringMismatch(
        pressed_finger=pressed_finger,
        sensed_finger=sensed_finger,
        pressed_peak_n=pressed_peak,
        sensed_peak_n=others[sensed_finger],
        pressed_slot=finger_to_sensor_id.get(pressed_finger),
        sensed_slot=finger_to_sensor_id.get(sensed_finger),
    )


LINK_CAUSE_WIRE_FORMAT_MISMATCH = "wire_format_mismatch"
LINK_CAUSE_UNROUTED_FRAMES = "unrouted_frames"
LINK_CAUSE_NO_FRAME_ALIGNMENT = "no_frame_alignment"
LINK_CAUSE_NO_BYTES = "no_bytes"


@dataclass(frozen=True)
class LikelyCause:
    """One likely cause of a dead or corrupt encoder stream.

    ``code`` is one of the ``LINK_CAUSE_*`` constants. ``description`` is
    pre-wrapped for terminal display; join its lines with a space for a
    single-line rendering.
    """

    code: str
    description: str


def diagnose_encoder_link(
    link_stats: LinkStats, stream_stats: EncoderStreamStats
) -> list[LikelyCause]:
    """Explain a dead or corrupt encoder stream from its link + stream counters.

    Returns the matching causes in check order; empty when the counters fit
    no known failure signature.
    """
    causes: list[LikelyCause] = []
    if stream_stats.frames_bad_effective_length > 0:
        causes.append(LikelyCause(
            code=LINK_CAUSE_WIRE_FORMAT_MISMATCH,
            description="Firmware/host wire-format mismatch: the firmware emits a\n"
                        "different joint count than the host expects. Re-flash it.",
        ))
    if link_stats.frames_dropped_no_handler:
        unhandled = ", ".join(f"0x{b:02X}" for b in link_stats.frames_dropped_no_handler)
        causes.append(LikelyCause(
            code=LINK_CAUSE_UNROUTED_FRAMES,
            description=f"Frames arriving on un-routed second-byte(s): {unhandled}.\n"
                        "Firmware may predate the current encoder header byte.",
        ))
    if (link_stats.bad_header_resyncs > 0
            and not link_stats.frames_routed
            and not link_stats.frames_dropped_no_handler):
        causes.append(LikelyCause(
            code=LINK_CAUSE_NO_FRAME_ALIGNMENT,
            description="Bytes arrive but never align to an AA-XX header. Wrong\n"
                        "baudrate, or you are reading the motor port by mistake.",
        ))
    if (link_stats.bad_header_resyncs == 0
            and not link_stats.frames_routed
            and not link_stats.frames_dropped_no_handler
            and link_stats.responses_received == 0):
        causes.append(LikelyCause(
            code=LINK_CAUSE_NO_BYTES,
            description="Zero bytes received. Board unpowered, cable unplugged, or\n"
                        "wrong serial port — try `ls /dev/cu.usbmodem*`.",
        ))
    return causes
