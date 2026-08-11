"""Checks that run before a measurement and refuse to start it.

Each one guards against a state that produces plausible-looking numbers rather
than an obvious failure. Those are the expensive ones: a run that crashes costs
an afternoon, a run that quietly records the wrong thing costs whatever is built
on it afterwards.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    ENCODER_LSB_DEG,
    JOINT_TO_ENCODER_SLOT,
)


class PreconditionError(RuntimeError):
    """A precondition failed. The message says what to do about it."""


# A slot flagging on more than this fraction of frames is reporting a condition,
# not a glitch, and nothing decoded from it can be trusted.
MAX_FLAGGED_FRACTION = 0.02

# Counts a slot cannot legitimately move between consecutive frames. At the
# board's emit period this is thousands of degrees per second, so a jump this
# size is corruption that happened to satisfy the single-bit parity check.
MAX_PLAUSIBLE_JUMP_COUNTS = 500

DEFAULT_HEALTH_FRAMES = 200
DEFAULT_HEALTH_TIMEOUT_S = 5.0


@dataclass(frozen=True)
class SlotHealth:
    """What one encoder slot did over a sample of frames."""

    slot: int
    joint: Optional[str]
    frames: int
    parity_bad: int
    angle_error: int
    impossible_jumps: int
    constant: bool

    @property
    def flagged_fraction(self) -> float:
        if self.frames == 0:
            return 1.0
        return max(self.parity_bad, self.angle_error) / self.frames

    @property
    def healthy(self) -> bool:
        return (
            self.flagged_fraction <= MAX_FLAGGED_FRACTION
            and self.impossible_jumps == 0
        )

    @property
    def complaint(self) -> str:
        parts = []
        if self.flagged_fraction > MAX_FLAGGED_FRACTION:
            parts.append(
                f"flagged on {self.flagged_fraction * 100:.0f}% of frames "
                f"(parity {self.parity_bad}, angle-error {self.angle_error})"
            )
        if self.impossible_jumps:
            parts.append(f"{self.impossible_jumps} implausible jumps")
        return "; ".join(parts)


@dataclass(frozen=True)
class EncoderHealthReport:
    frames_sampled: int
    duration_s: float
    slots: List[SlotHealth]

    @property
    def unhealthy(self) -> List[SlotHealth]:
        return [slot for slot in self.slots if not slot.healthy]

    @property
    def constant_slots(self) -> List[SlotHealth]:
        """Slots whose count never changed. Expected on an unmapped slot; on a
        mapped one it means a joint that is not being measured at all."""
        return [slot for slot in self.slots if slot.constant and slot.joint]

    def as_dict(self) -> Dict[str, object]:
        return {
            "frames_sampled": self.frames_sampled,
            "duration_s": self.duration_s,
            "slots": [
                {
                    "slot": slot.slot,
                    "joint": slot.joint,
                    "parity_bad": slot.parity_bad,
                    "angle_error": slot.angle_error,
                    "impossible_jumps": slot.impossible_jumps,
                    "constant": slot.constant,
                }
                for slot in self.slots
            ],
        }


def check_encoder_health(
    client,
    *,
    num_frames: int = DEFAULT_HEALTH_FRAMES,
    timeout_s: float = DEFAULT_HEALTH_TIMEOUT_S,
    poll_s: float = 0.001,
) -> EncoderHealthReport:
    """Sample distinct frames and report what each slot did.

    Consecutive reads of the same frame are ignored, so the count is frames
    seen, not polls taken.
    """
    if num_frames <= 0:
        raise ValueError("num_frames must be positive")

    counts = np.empty((num_frames, AUTO_ENC_NUM_JOINTS), dtype=np.int64)
    parity_bad = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.int64)
    angle_error = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.int64)

    slot_to_joint = {slot: joint for joint, slot in JOINT_TO_ENCODER_SLOT.items()}
    started = time.monotonic()
    deadline = started + timeout_s
    last_t = None
    seen = 0

    while seen < num_frames:
        reading = client.get_latest()
        if reading is not None and reading.timestamp != last_t:
            last_t = reading.timestamp
            counts[seen] = np.asarray(reading.raw_counts) & 0x3FFF
            parity_bad += ~np.asarray(reading.parity_ok)
            angle_error += np.asarray(reading.angle_error)
            seen += 1
            continue
        if time.monotonic() > deadline:
            raise PreconditionError(
                f"only {seen}/{num_frames} encoder frames arrived in {timeout_s}s. "
                "The encoder stream is stopped or the link is down; check the "
                "connection and that the stream was started."
            )
        time.sleep(poll_s)

    duration = time.monotonic() - started
    counts = counts[:seen]
    # Wrap-aware: a slot crossing zero moves by one count, not by a full turn.
    steps = np.diff(counts, axis=0)
    steps = (steps + 8192) % 16384 - 8192
    jumps = np.sum(np.abs(steps) > MAX_PLAUSIBLE_JUMP_COUNTS, axis=0)
    constant = np.all(steps == 0, axis=0) if seen > 1 else np.ones(
        AUTO_ENC_NUM_JOINTS, dtype=bool
    )

    return EncoderHealthReport(
        frames_sampled=seen,
        duration_s=duration,
        slots=[
            SlotHealth(
                slot=slot,
                joint=slot_to_joint.get(slot),
                frames=seen,
                parity_bad=int(parity_bad[slot]),
                angle_error=int(angle_error[slot]),
                impossible_jumps=int(jumps[slot]),
                constant=bool(constant[slot]),
            )
            for slot in range(AUTO_ENC_NUM_JOINTS)
        ],
    )


def require_healthy_encoders(
    client,
    joints: Optional[Sequence[str]] = None,
    **kwargs,
) -> EncoderHealthReport:
    """Refuse to proceed while any joint's encoder slot is reporting a fault.

    A slot that flags every frame is a state the hand can be power-cycled out
    of, not a defect to diagnose here. It matters because nothing downstream
    fails loudly: the control loop holds a flagged joint's previous angle, so
    its error stops changing and its correction winds steadily to the clamp, and
    calibration refuses to anchor a flagged joint and silently keeps whatever
    anchor was persisted before. Either way the numbers look reasonable and
    describe nothing.
    """
    report = check_encoder_health(client, **kwargs)
    watched = set(joints) if joints is not None else None

    faults = [
        slot
        for slot in report.unhealthy
        if slot.joint and (watched is None or slot.joint in watched)
    ]
    if faults:
        detail = "; ".join(f"{slot.joint} ({slot.complaint})" for slot in faults)
        raise PreconditionError(
            f"encoder slots are reporting faults: {detail}. Power-cycle the hand "
            "and retry. If it survives a restart, the joint needs attention "
            "before it can be measured."
        )
    return report


def require_open_loop(hand) -> None:
    """Refuse to proceed while a control loop owns the motors.

    Asserts the state rather than trusting that a connect argument had the
    intended effect: a study that shares the motors with a running loop measures
    the two of them fighting.
    """
    if getattr(hand, "_loop", None) is not None:
        raise PreconditionError(
            "a joint-feedback loop is running and would command the same motors. "
            "Reconnect without engaging feedback."
        )


def require_closed_loop(hand) -> None:
    """Refuse to proceed unless a control loop is running and still live."""
    loop = getattr(hand, "_loop", None)
    if loop is None:
        raise PreconditionError(
            "no joint-feedback loop is running; reconnect with feedback engaged."
        )
    if loop.get_stats()["fallback_active"]:
        raise PreconditionError(
            "the joint loop has e-stopped and its measurements are frozen. "
            "Reconnect: there is no way to restart it in place."
        )


def require_no_motion_profile(hand) -> Dict[int, int]:
    """Refuse to proceed while any motor is running a trapezoidal profile.

    Nothing in the package writes these registers, so they hold whatever is in
    motor EEPROM. A nonzero profile velocity makes the motor ramp to its target
    instead of chasing it, which turns any measured response time into a reading
    of that register.
    """
    client = getattr(hand, "_motor_client", None)
    reader = getattr(client, "read_profile_velocity", None)
    if reader is None:
        return {}
    velocities = {
        motor_id: int(value)
        for motor_id, value in zip(hand.config.motor_ids, reader(hand.config.motor_ids))
    }
    shaped = {mid: value for mid, value in velocities.items() if value}
    if shaped:
        raise PreconditionError(
            f"motors {sorted(shaped)} have a nonzero profile velocity {shaped}. "
            "The servo would ramp to each target rather than chase it, so any "
            "measured response time would be that register. Clear it first."
        )
    return velocities
