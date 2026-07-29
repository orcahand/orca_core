"""Vectorised PI controller for the host-side joint loop.

Output is a per-joint correction in degrees, clipped to
``±correction_max_deg``. The loop thread adds the correction to the base
joint target before mapping to motor positions; the motor's internal PID
then tracks the corrected motor target on the motor encoder.

Conditional integration freezes the integrator when the output is at the
clamp and the error sign would push it further (anti-windup). No
derivative term — the inner motor loop is already damped, and a D term on
a quantised joint encoder at 100 Hz is mostly noise.

A second, much slower integral — the *bias* — accumulates the persistent
content of the fast one at ``bias_tau_s`` (minutes). The fast integral
converges to whatever constant offset the joint→motor map currently gets
wrong (slack buildup, calibration bias); parking that offset in the bias
keeps the fast integrator's headroom for genuine disturbances, stops large
target moves re-paying the same convergence transient, and gives the
open-loop watchdog fallback the one part of the trim that works without
feedback. No transfer bookkeeping is needed: as the bias takes over the DC
term the error shrinks and the fast integral unwinds through the loop
itself. With minutes against the PI's seconds and the motor PID's
milliseconds, the three are time-scale separated and cannot interact
dynamically. The bias is offset-only — the map slope is never adapted —
and is hard-clamped to ``±bias_max_deg``; a joint reaching the clamp logs
once, since offset beyond it means the hand wants recalibrating.
"""

import logging
import threading
from typing import Dict, Optional, Sequence, Union

import numpy as np

from .constants import (
    DEFAULT_BIAS_MAX_DEG,
    DEFAULT_BIAS_TAU_S,
    MAX_LOOP_DT_S,
    MIN_LOOP_DT_S,
)


logger = logging.getLogger(__name__)

ScalarOrArray = Union[float, np.ndarray]


class JointController:
    """Per-channel PI with shared scalar or per-joint vector gains.

    State mutation is lock-guarded so ``reset()`` / ``set_gains()`` called
    from other threads cannot interleave with the loop thread's ``step()``.
    """

    def __init__(
        self,
        num_joints: int,
        joint_names: Optional[Sequence[str]] = None,
        *,
        bias_tau_s: Optional[float] = DEFAULT_BIAS_TAU_S,
        bias_max_deg: float = DEFAULT_BIAS_MAX_DEG,
    ):
        if num_joints <= 0:
            raise ValueError("num_joints must be positive")
        if joint_names is not None and len(joint_names) != num_joints:
            raise ValueError(
                f"joint_names has {len(joint_names)} entries for {num_joints} joints"
            )
        if bias_tau_s is not None and bias_tau_s <= 0:
            raise ValueError("bias_tau_s must be positive, or None to disable")
        if bias_max_deg < 0:
            raise ValueError("bias_max_deg must be non-negative")
        self._num_joints = int(num_joints)
        self._joint_names = list(joint_names) if joint_names is not None else None
        self._bias_tau_s = None if bias_tau_s is None else float(bias_tau_s)
        self._bias_max_deg = float(bias_max_deg)
        self._lock = threading.Lock()
        self._Kp = np.zeros(self._num_joints)
        self._Ki = np.zeros(self._num_joints)
        self._correction_max_deg = np.zeros(self._num_joints)
        self._i_clamp_deg = np.zeros(self._num_joints)
        self._ierr = np.zeros(self._num_joints)
        self._bias = np.zeros(self._num_joints)
        self._bias_at_clamp = np.zeros(self._num_joints, dtype=bool)
        self._last_correction = np.zeros(self._num_joints)
        self._integral_frozen = False

    @property
    def num_joints(self) -> int:
        return self._num_joints

    @property
    def bias_deg(self) -> np.ndarray:
        """Per-joint slow feed-forward bias in degrees. The loop adds this to
        the joint target alongside the correction, including in its open-loop
        fallback, so it is not subject to ``correction_max_deg``."""
        with self._lock:
            return self._bias.copy()

    def set_gains(
        self,
        Kp: ScalarOrArray,
        Ki: ScalarOrArray,
        correction_max_deg: ScalarOrArray,
        i_clamp_deg: ScalarOrArray,
    ) -> None:
        """Set per-channel gains and clamps. Each value is a scalar
        (broadcast) or a ``(num_joints,)`` array. All values must be
        non-negative. The four arrays are validated and broadcast before any
        attribute is assigned, so a bad input never leaves the controller in
        a half-installed state."""
        kp = self._broadcast(Kp, "Kp")
        ki = self._broadcast(Ki, "Ki")
        corr_max = self._broadcast(correction_max_deg, "correction_max_deg")
        i_clamp = self._broadcast(i_clamp_deg, "i_clamp_deg")
        with self._lock:
            self._Kp = kp
            self._Ki = ki
            self._correction_max_deg = corr_max
            self._i_clamp_deg = i_clamp

    def step(
        self,
        target_deg: np.ndarray,
        measured_deg: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """Advance one cycle and return per-joint correction in degrees."""
        target = np.asarray(target_deg, dtype=np.float64)
        measured = np.asarray(measured_deg, dtype=np.float64)
        if target.shape != (self._num_joints,):
            raise ValueError(
                f"target must have shape ({self._num_joints},), got {target.shape}"
            )
        if measured.shape != (self._num_joints,):
            raise ValueError(
                f"measured must have shape ({self._num_joints},), got {measured.shape}"
            )
        dt_clamped = float(np.clip(dt, MIN_LOOP_DT_S, MAX_LOOP_DT_S))
        err = target - measured

        with self._lock:
            u_unsat = self._Kp * err + self._Ki * self._ierr

            if not self._integral_frozen:
                saturated = np.abs(u_unsat) >= self._correction_max_deg
                pushing_into_sat = np.sign(err) == np.sign(u_unsat)
                allow_integrate = ~(saturated & pushing_into_sat)
                new_ierr = np.where(
                    allow_integrate, self._ierr + err * dt_clamped, self._ierr
                )
                self._ierr = np.clip(new_ierr, -self._i_clamp_deg, self._i_clamp_deg)

            u = np.clip(
                self._Kp * err + self._Ki * self._ierr,
                -self._correction_max_deg,
                self._correction_max_deg,
            )
            self._last_correction = u

            if self._bias_tau_s is not None and not self._integral_frozen:
                # A channel at either clamp is a blocked or externally-loaded
                # joint, whose integral is holding a force rather than a map
                # error; banking that would both corrupt the bias and rob the
                # clamp warning of its meaning. A genuine map offset settles
                # the integral far below its clamp, so this rejects only the
                # abnormal cases.
                saturated = (np.abs(u) >= self._correction_max_deg) | (
                    np.abs(self._ierr) >= self._i_clamp_deg
                )
                leak = np.where(
                    saturated,
                    0.0,
                    self._Ki * self._ierr * (dt_clamped / self._bias_tau_s),
                )
                self._bias = np.clip(
                    self._bias + leak, -self._bias_max_deg, self._bias_max_deg
                )
                self._warn_new_clamp_hits()
        return u.copy()

    def reset(self) -> None:
        """Zero the integrator and the last-correction snapshot. The learned
        bias survives — it describes the hand, not the loop's run."""
        with self._lock:
            self._ierr = np.zeros(self._num_joints)
            self._last_correction = np.zeros(self._num_joints)
            self._integral_frozen = False

    def reset_bias(self) -> None:
        """Zero the slow feed-forward bias. Call after a fresh calibration, or
        when the learned offsets are suspect."""
        with self._lock:
            self._bias = np.zeros(self._num_joints)
            self._bias_at_clamp = np.zeros(self._num_joints, dtype=bool)

    def freeze_integral(self) -> None:
        with self._lock:
            self._integral_frozen = True

    def unfreeze_integral(self) -> None:
        with self._lock:
            self._integral_frozen = False

    @property
    def integral_frozen(self) -> bool:
        return self._integral_frozen

    def get_state(self) -> Dict[str, np.ndarray]:
        with self._lock:
            return {
                "ierr_deg": self._ierr.copy(),
                "last_correction_deg": self._last_correction.copy(),
                "bias_deg": self._bias.copy(),
                "bias_at_clamp": self._bias_at_clamp.copy(),
            }

    def _warn_new_clamp_hits(self) -> None:
        """Log once per joint that newly reaches the bias clamp. Caller holds
        the lock."""
        at_clamp = np.abs(self._bias) >= self._bias_max_deg - 1e-12
        newly = at_clamp & ~self._bias_at_clamp
        self._bias_at_clamp = at_clamp
        for idx in np.flatnonzero(newly):
            name = (
                self._joint_names[idx]
                if self._joint_names is not None
                else f"joint[{idx}]"
            )
            logger.warning(
                "slow feed-forward bias for %s hit the ±%.1f° clamp; the "
                "joint→motor map is off by more than the bias can absorb — "
                "consider recalibrating",
                name,
                self._bias_max_deg,
            )

    def _broadcast(self, value: ScalarOrArray, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        if array.ndim == 0:
            array = np.full(self._num_joints, float(array))
        elif array.shape != (self._num_joints,):
            raise ValueError(
                f"{name} must be scalar or shape ({self._num_joints},), got {array.shape}"
            )
        if np.any(array < 0):
            raise ValueError(f"{name} must be non-negative")
        return array
