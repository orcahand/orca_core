# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Encoder slot-health and slot-identity preamble.

Two checks that gate any procedure trusting the joint encoders (the hardstop
calibration's encoder pass, and the vision calibration in particular):

1. **Health** — every encoder-backed slot streams with clean parity and no
   chip angle-error flag. A persistently flagged slot is a *state*, not a
   hardware defect: the remedy is a power cycle, and if it persists it is a
   firmware-side issue. A routine that starts in this state silently keeps
   stale anchors, so it must be caught before anything else runs.
2. **Identity** — each joint, wiggled alone at limited current, moves its
   *mapped* encoder slot and dominates every other slot's response. Catches
   crossed connectors and wrong slot maps on any community-built hand; those
   break calibration in a way no offset can fix.

Like the other maintenance routines this is interaction-free: progress is
reported through ``progress_callback({"event": ..., ...})`` and cooperative
cancellation through ``should_stop()``. Returns ``None`` on early stop; on
any exit torque is released and the configured current limit restored.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, TYPE_CHECKING

import numpy as np

from ..constants import CURRENT_BASED_POSITION, WRIST
from .calibration_routine import _read_motor_pos_checked
from ..hardware.sensing.constants import (
    ENCODER_COUNTS_PER_REV,
    ENCODER_LSB_DEG,
    JOINT_TO_ENCODER_SLOT,
)

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

HEALTH_SAMPLE_S = 2.0
HEALTH_MIN_FRAMES = 40
# A slot asserting angle-error on more than this fraction of frames is in the
# flagged state; occasional single-frame flags are normal chip noise.
FLAGGED_PCT = 5.0
PARITY_FAIL_PCT = 1.0

# Identity wiggle: relative motor move under the calibration current limit.
# ~0.35 rad of motor travel is a few joint degrees on every v2 ratio — far
# above encoder noise, small enough to be harmless against a hardstop.
WIGGLE_AMP_RAD = 0.35
SETTLE_S = 0.35
SAMPLE_FRAMES = 15
SAMPLE_TIMEOUT_S = 3.0
# The mapped slot must move at least this much and dominate every other slot
# by DOMINANCE_FACTOR. Tendon coupling legitimately moves neighbours a little,
# so identity is a dominance test, not an exclusivity test.
MIN_RESPONSE_DEG = 1.0
DOMINANCE_FACTOR = 3.0


def _count_delta(a: int, b: int) -> int:
    """Wrap-aware distance between two 14-bit encoder counts."""
    d = abs(a - b) % ENCODER_COUNTS_PER_REV
    return min(d, ENCODER_COUNTS_PER_REV - d)


@dataclass(frozen=True)
class SlotHealth:
    joint: str
    slot: int
    frames: int
    parity_fail_pct: float
    angle_error_pct: float
    verdict: str  # "ok" | "flagged" | "parity" | "no-frames"


@dataclass(frozen=True)
class SlotIdentity:
    joint: str
    expected_slot: int
    responses_deg: Dict[int, float]
    argmax_slot: Optional[int]
    dominance: float
    verdict: str  # "ok" | "mismapped" | "ambiguous" | "no-response" | "skipped-flagged"


@dataclass(frozen=True)
class SlotCheckResult:
    health: List[SlotHealth] = field(default_factory=list)
    identity: List[SlotIdentity] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return all(h.verdict == "ok" for h in self.health) and all(
            i.verdict == "ok" for i in self.identity
        )


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        pass


def _collect_frames(client, duration_s: float):
    """Distinct frames for ``duration_s``: (counts, parity_ok, angle_error) stacks."""
    counts, parity, err = [], [], []
    last_ts = None
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        reading = client.get_latest()
        if reading is not None and reading.timestamp != last_ts:
            last_ts = reading.timestamp
            counts.append(np.asarray(reading.raw_counts) & 0x3FFF)
            parity.append(np.asarray(reading.parity_ok, bool))
            err.append(np.asarray(reading.angle_error, bool))
        time.sleep(0.002)
    if not counts:
        return None
    return np.stack(counts), np.stack(parity), np.stack(err)


def _sample_median_counts(client, n: int = SAMPLE_FRAMES,
                          timeout_s: float = SAMPLE_TIMEOUT_S) -> Optional[np.ndarray]:
    """Per-slot median 14-bit count over ``n`` distinct frames; None on timeout.

    Chip-flagged frames still contribute counts — identity only needs motion,
    and health has already judged the flags.
    """
    rows = []
    last_ts = None
    deadline = time.monotonic() + timeout_s
    while len(rows) < n:
        reading = client.get_latest()
        if reading is not None and reading.timestamp != last_ts:
            last_ts = reading.timestamp
            rows.append(np.asarray(reading.raw_counts) & 0x3FFF)
        if time.monotonic() > deadline:
            return None
        time.sleep(0.002)
    return np.median(np.stack(rows), axis=0).astype(int)


def run_slot_check(
    hand: "OrcaHand",
    joint_encoder_client,
    *,
    joints: list[str] | None = None,
    identity: bool = True,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> SlotCheckResult | None:
    """Run the health and (optionally) identity checks and return the result.

    Args:
        hand: A connected :class:`~orca_core.OrcaHand`.
        joint_encoder_client: Started encoder stream client (``get_latest()``).
        joints: Restrict to these joints (default: every encoder-backed joint).
        identity: Also run the per-joint wiggle. Health-only when ``False``
            (e.g. mock hands, whose encoder stream is static by design).
        progress_callback: Optional ``callable(dict)`` with events
            ``slot_check_started``, ``slot_health``, ``identity_started``,
            ``slot_identity``, ``slot_check_done``, ``slot_check_aborted``.
        should_stop: Optional ``callable() -> bool``; return True to abort.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    checked = [
        j for j in hand._encoder_backed_joints()
        if joints is None or j in joints
    ]
    if not checked:
        raise ValueError("no encoder-backed joints selected for the slot check")

    _emit(progress_callback, "slot_check_started", joints=checked)

    # ---- health ------------------------------------------------------------
    frames = _collect_frames(joint_encoder_client, HEALTH_SAMPLE_S)
    health: list[SlotHealth] = []
    for joint in checked:
        slot = JOINT_TO_ENCODER_SLOT[joint]
        if frames is None or len(frames[0]) < HEALTH_MIN_FRAMES:
            n = 0 if frames is None else len(frames[0])
            entry = SlotHealth(joint, slot, n, 100.0, 100.0, "no-frames")
        else:
            _, parity, err = frames
            n = len(parity)
            parity_fail = 100.0 * float(np.mean(~parity[:, slot]))
            angle_err = 100.0 * float(np.mean(err[:, slot]))
            if parity_fail > PARITY_FAIL_PCT:
                verdict = "parity"
            elif angle_err > FLAGGED_PCT:
                verdict = "flagged"
            else:
                verdict = "ok"
            entry = SlotHealth(joint, slot, n, round(parity_fail, 2),
                               round(angle_err, 2), verdict)
        health.append(entry)
        _emit(progress_callback, "slot_health", joint=joint, slot=slot,
              verdict=entry.verdict, angle_error_pct=entry.angle_error_pct,
              parity_fail_pct=entry.parity_fail_pct)

    flagged = {h.joint for h in health if h.verdict != "ok"}

    if not identity:
        result = SlotCheckResult(health=health)
        _emit(progress_callback, "slot_check_done", passed=result.passed)
        return result

    # ---- identity ----------------------------------------------------------
    identities: list[SlotIdentity] = []
    slots_checked = sorted(JOINT_TO_ENCODER_SLOT[j] for j in checked)
    _emit(progress_callback, "identity_started", joints=checked)

    hand.set_control_mode(CURRENT_BASED_POSITION)
    hand.enable_torque()
    try:
        for joint in checked:
            if should_stop():
                _emit(progress_callback, "slot_check_aborted")
                return None
            expected = JOINT_TO_ENCODER_SLOT[joint]
            if joint in flagged:
                entry = SlotIdentity(joint, expected, {}, None, 0.0, "skipped-flagged")
                identities.append(entry)
                _emit(progress_callback, "slot_identity", joint=joint,
                      verdict=entry.verdict)
                continue

            motor_id = hand.config.joint_to_motor_map[joint]
            idx = hand.config.motor_id_to_idx_dict[motor_id]
            hand.set_max_current(
                hand.config.wrist_calibration_current if joint == WRIST
                else hand.config.calibration_current
            )

            start_pos = float(_read_motor_pos_checked(hand)[idx])
            base = _sample_median_counts(joint_encoder_client)
            entry = None
            if base is None:
                entry = SlotIdentity(joint, expected, {}, None, 0.0, "no-response")
            else:
                responses = np.zeros(len(base))
                for target in (WIGGLE_AMP_RAD, -WIGGLE_AMP_RAD):
                    hand._set_motor_pos({motor_id: start_pos + target})
                    time.sleep(SETTLE_S)
                    sample = _sample_median_counts(joint_encoder_client)
                    if sample is not None:
                        deltas = [
                            _count_delta(int(sample[s]), int(base[s])) * ENCODER_LSB_DEG
                            for s in range(len(base))
                        ]
                        responses = np.maximum(responses, deltas)
                hand._set_motor_pos({motor_id: start_pos})
                time.sleep(SETTLE_S)

                responses_deg = {s: round(float(responses[s]), 3) for s in slots_checked}
                mapped = responses[expected]
                others = [responses[s] for s in slots_checked if s != expected]
                top_other = max(others) if others else 0.0
                argmax_slot = int(max(slots_checked, key=lambda s: responses[s]))
                dominance = float(mapped / max(top_other, 1e-6))
                # "no-response" only when NO slot moved: a silent mapped slot
                # while another responds is the mismapped-wiring signature.
                if responses[argmax_slot] < MIN_RESPONSE_DEG:
                    verdict = "no-response"
                elif argmax_slot != expected or mapped < MIN_RESPONSE_DEG:
                    verdict = "mismapped"
                elif dominance < DOMINANCE_FACTOR:
                    verdict = "ambiguous"
                else:
                    verdict = "ok"
                entry = SlotIdentity(joint, expected, responses_deg, argmax_slot,
                                     round(dominance, 2), verdict)

            identities.append(entry)
            _emit(progress_callback, "slot_identity", joint=joint,
                  verdict=entry.verdict, expected_slot=expected,
                  argmax_slot=entry.argmax_slot, dominance=entry.dominance,
                  responses_deg=entry.responses_deg)
    finally:
        try:
            hand.set_max_current(hand.config.max_current)
            hand.disable_torque()
        except Exception as e:
            _emit(progress_callback, "cleanup_failed", error=str(e))

    result = SlotCheckResult(health=health, identity=identities)
    _emit(progress_callback, "slot_check_done", passed=result.passed)
    return result
