"""A synthetic joint, and the glue that puts one behind a mock hand.

Analysis code is validated against a plant whose parameters are known before it
is pointed at hardware, where nothing is known. That only works if the model can
express the behaviour being looked for, so the response here is second order
with the damping ratio free: a first-order or critically-damped model cannot
overshoot or ring, and a fit built against one would report a confident time
constant for a joint that oscillates.

Three effects are modelled, because each one is separately mistakable for
another:

* **Transport delay** — the command reaches the mechanism late.
* **Backlash** — motion reverses into slack before the joint follows, which
  looks exactly like transport delay if only one direction is ever measured.
* **Transmission gain** — the ratio the map was calibrated with differs from the
  joint's real one, so a commanded degree is not a measured degree.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    ENCODER_COUNTS_PER_REV,
    ENCODER_LSB_DEG,
    JOINT_TO_ENCODER_SLOT,
    joint_encoder_polarity_for_side,
)
from orca_core.hardware.sensing.types import EncoderReading


@dataclass
class PlantJoint:
    """One joint responding to a commanded angle.

    ``natural_frequency_hz`` and ``damping_ratio`` set the response shape; a
    damping ratio below one overshoots and rings at roughly the natural
    frequency. ``gain`` is measured degrees per commanded degree: 1.0 means the
    calibrated ratio matches the mechanism.
    """

    natural_frequency_hz: float = 8.0
    damping_ratio: float = 0.7
    backlash_deg: float = 0.0
    dead_time_s: float = 0.0
    gain: float = 1.0
    angle_deg: float = 0.0

    _driver_deg: float = field(default=0.0, repr=False)
    _driver_velocity: float = field(default=0.0, repr=False)
    _slack_center: Optional[float] = field(default=None, repr=False)
    _delay_line: deque = field(default_factory=deque, repr=False)

    def __post_init__(self) -> None:
        if self.natural_frequency_hz <= 0:
            raise ValueError("natural_frequency_hz must be positive")
        if self.damping_ratio < 0:
            raise ValueError("damping_ratio must be non-negative")
        if self.backlash_deg < 0:
            raise ValueError("backlash_deg must be non-negative")
        if self.dead_time_s < 0:
            raise ValueError("dead_time_s must be non-negative")

    @property
    def driver_deg(self) -> float:
        """Where the driving side sits. Equal to the joint angle only when there
        is no slack between them."""
        return self._driver_deg

    def reset(self, angle_deg: float = 0.0) -> None:
        self.angle_deg = float(angle_deg)
        self._driver_deg = float(angle_deg)
        self._driver_velocity = 0.0
        self._slack_center = None
        self._delay_line.clear()

    def step(self, command_deg: float, dt_s: float) -> float:
        """Advance by ``dt_s`` under ``command_deg`` and return the new angle."""
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        delayed = self._delay(command_deg * self.gain, dt_s)

        omega = 2.0 * math.pi * self.natural_frequency_hz
        # Semi-implicit: velocity first, then position, which stays stable at
        # step sizes where the explicit form would grow without bound.
        acceleration = (
            omega * omega * (delayed - self._driver_deg)
            - 2.0 * self.damping_ratio * omega * self._driver_velocity
        )
        self._driver_velocity += acceleration * dt_s
        self._driver_deg += self._driver_velocity * dt_s

        self.angle_deg = self._through_backlash(self._driver_deg)
        return self.angle_deg

    def _delay(self, command: float, dt_s: float) -> float:
        """Return the command from ``dead_time_s`` ago.

        The line is primed with the resting position, so a plant that has not
        been commanded yet holds still rather than lurching to whatever the
        first command happens to be.
        """
        if self.dead_time_s <= 0:
            return command
        depth = max(1, int(round(self.dead_time_s / dt_s)))
        line = self._delay_line
        while len(line) < depth:
            line.append(self._driver_deg)
        line.append(command)
        return line.popleft()

    def _through_backlash(self, driver: float) -> float:
        """Where the joint sits given where the driving side is.

        The slack lies between the two, so a reversal moves the driver across
        the gap before the joint answers at all. Measured in one direction only
        that is indistinguishable from transport delay, which is why a step
        measurement pairs a preload with its test step.
        """
        if self.backlash_deg <= 0:
            return driver
        half = self.backlash_deg / 2.0
        if self._slack_center is None:
            self._slack_center = driver
        elif driver > self._slack_center + half:
            self._slack_center = driver - half
        elif driver < self._slack_center - half:
            self._slack_center = driver + half
        return self._slack_center


def simulate_step(
    joint: PlantJoint,
    amplitude_deg: float,
    *,
    duration_s: float = 1.0,
    dt_s: float = 0.002,
    settle_s: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Hold at zero for ``settle_s``, step to ``amplitude_deg``, and record.

    Returns times relative to the step and the measured angle at each, sampled
    at ``dt_s``. Used to produce a response whose parameters are known, so a fit
    can be checked against them.
    """
    times: List[float] = []
    angles: List[float] = []
    t = -settle_s
    while t < 0:
        joint.step(0.0, dt_s)
        times.append(t)
        angles.append(joint.angle_deg)
        t += dt_s
    while t <= duration_s:
        joint.step(amplitude_deg, dt_s)
        times.append(t)
        angles.append(joint.angle_deg)
        t += dt_s
    return np.array(times), np.array(angles)


class PlantEncoderSource:
    """Encoder client backed by a plant driven by a mock hand's motor commands.

    Each read advances the plant by one period using whatever the loop last
    wrote to the motors, then encodes the resulting angles as a frame. That is
    the real causal order — command, mechanism, measurement — so a loop running
    against this sees the same delay structure it sees on a hand.
    """

    def __init__(
        self,
        hand,
        joints: Optional[Sequence[str]] = None,
        *,
        dt_s: float = 0.01,
        template: Optional[PlantJoint] = None,
        freshness_ms: float = 0.0,
    ):
        self._hand = hand
        self._joint_names = list(joints or hand._encoder_backed_joints())
        self._dt_s = float(dt_s)
        self._freshness_ms = float(freshness_ms)
        self.joints: Dict[str, PlantJoint] = {
            name: PlantJoint(**vars_of(template)) for name in self._joint_names
        }
        self._polarity = joint_encoder_polarity_for_side(hand.config.type)
        self._encoder_cal = hand.calibration.joint_encoder_calibration_dict
        self._reads = 0
        self._timestamp = 0.0

    @property
    def reads(self) -> int:
        return self._reads

    def angles(self) -> Dict[str, float]:
        return {name: joint.angle_deg for name, joint in self.joints.items()}

    def set_angles(self, angles: Dict[str, float]) -> None:
        for name, angle in angles.items():
            self.joints[name].reset(angle)

    def align_hand_to_plant(self) -> None:
        """Move the hand's motors to where the plant's angles say they are.

        Call before priming a loop. The loop captures a fixed motor-to-joint
        offset when it anchors, to start without lurching from wherever the
        motors happen to sit; the plant reads motor commands back through the
        map, which does not carry that offset. Unless the two agree at the
        start, every command the plant sees is displaced by it.
        """
        positions = self._hand._joint_to_motor_pos(self.angles())
        index = self._hand.config.motor_id_to_idx_dict
        joint_to_motor = self._hand.config.joint_to_motor_map
        self._hand._set_motor_pos(
            {
                joint_to_motor[name]: positions[index[joint_to_motor[name]]]
                for name in self.joints
                if name in joint_to_motor
            }
        )

    def advance(self) -> None:
        """Drive every joint with the hand's current motor command."""
        commanded = self._hand._motor_to_joint_pos(self._hand.get_motor_pos())
        for name, joint in self.joints.items():
            command = commanded.get(name)
            if command is None:
                continue
            joint.step(float(command), self._dt_s)

    def get_latest(self) -> EncoderReading:
        self._reads += 1
        self.advance()
        self._timestamp += self._dt_s
        return self._encode(self.angles())

    def _encode(self, angles: Dict[str, float]) -> EncoderReading:
        import time

        raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
        for name, angle in angles.items():
            slot = JOINT_TO_ENCODER_SLOT[name]
            polarity = self._polarity[name]
            anchor_count = self._encoder_cal[name].enc_at_anchor_count
            anchor_angle = self._hand.config.joint_roms_dict[name][1]
            delta = int(round((angle - anchor_angle) / (polarity * ENCODER_LSB_DEG)))
            raw[slot] = (anchor_count + delta) % ENCODER_COUNTS_PER_REV
        return EncoderReading(
            raw_counts=raw,
            parity_ok=np.ones(AUTO_ENC_NUM_JOINTS, dtype=bool),
            angle_error=np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool),
            error_byte=0,
            timestamp=time.monotonic() - self._freshness_ms / 1000.0,
        )


def vars_of(template: Optional[PlantJoint]) -> dict:
    """Constructor arguments reproducing ``template``, or the defaults."""
    if template is None:
        return {}
    return {
        "natural_frequency_hz": template.natural_frequency_hz,
        "damping_ratio": template.damping_ratio,
        "backlash_deg": template.backlash_deg,
        "dead_time_s": template.dead_time_s,
        "gain": template.gain,
    }
