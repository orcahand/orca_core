"""Vectorised PI controller for the host-side joint loop.

Output is a per-joint correction in degrees, clipped to
``±correction_max_deg``. The loop thread adds the correction to the base
joint target before mapping to motor positions; the motor's internal PID
then tracks the corrected motor target on the motor encoder.

Conditional integration freezes the integrator when the output is at the
clamp and the error sign would push it further. That is the only
anti-windup mechanism, and it makes ``correction_max_deg`` the effective
bound on stored integral: the freeze trips at
``ierr = correction_max_deg / Ki``, so the retained integral contribution
is ``correction_max_deg`` whatever Ki is.

That matters because stored integral can only be discharged by error of
the opposite sign — i.e. by overshooting. Anything the integrator banks
while a joint is held off target gets paid back on release, so
``correction_max_deg`` sets how big that lurch can be. Channels with
``Ki = 0`` hold no integral state at all, so raising Ki at runtime
starts from a clean integrator.

No derivative term — the inner motor loop is already damped, and a D term
on a quantised joint encoder at 100 Hz is mostly noise.
"""

import threading
from typing import Dict, Union

import numpy as np

from .constants import MAX_LOOP_DT_S, MIN_LOOP_DT_S


ScalarOrArray = Union[float, np.ndarray]


class JointController:
    """Per-channel PI with shared scalar or per-joint vector gains.

    State mutation is lock-guarded so ``reset()`` / ``set_gains()`` called
    from other threads cannot interleave with the loop thread's ``step()``.
    """

    def __init__(self, num_joints: int):
        if num_joints <= 0:
            raise ValueError("num_joints must be positive")
        self._num_joints = int(num_joints)
        self._lock = threading.Lock()
        self._Kp = np.zeros(self._num_joints)
        self._Ki = np.zeros(self._num_joints)
        self._correction_max_deg = np.zeros(self._num_joints)
        self._ierr = np.zeros(self._num_joints)
        self._last_correction = np.zeros(self._num_joints)
        self._integral_frozen = False

    @property
    def num_joints(self) -> int:
        return self._num_joints

    def set_gains(
        self,
        Kp: ScalarOrArray,
        Ki: ScalarOrArray,
        correction_max_deg: ScalarOrArray,
    ) -> None:
        """Set per-channel gains and clamps. Each value is a scalar
        (broadcast) or a ``(num_joints,)`` array. All values must be
        non-negative. The three arrays are validated and broadcast before any
        attribute is assigned, so a bad input never leaves the controller in
        a half-installed state."""
        kp = self._broadcast(Kp, "Kp")
        ki = self._broadcast(Ki, "Ki")
        corr_max = self._broadcast(correction_max_deg, "correction_max_deg")
        with self._lock:
            self._Kp = kp
            self._Ki = ki
            self._correction_max_deg = corr_max

    def get_gains(self) -> Dict[str, np.ndarray]:
        """Per-channel gains currently installed, in the controller's channel
        order. Copies, so callers can read them without holding the lock."""
        with self._lock:
            return {
                "Kp": self._Kp.copy(),
                "Ki": self._Ki.copy(),
                "correction_max_deg": self._correction_max_deg.copy(),
            }

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
                self._ierr = np.where(
                    allow_integrate, self._ierr + err * dt_clamped, self._ierr
                )
                # A channel with Ki=0 holds no integral state: error banked
                # during a pure-P phase would otherwise pin the correction at
                # the clamp for a long unwind once Ki is raised at runtime.
                self._ierr = np.where(self._Ki > 0.0, self._ierr, 0.0)

            u = np.clip(
                self._Kp * err + self._Ki * self._ierr,
                -self._correction_max_deg,
                self._correction_max_deg,
            )
            self._last_correction = u
        return u.copy()

    def reset(self) -> None:
        """Zero the integrator and the last-correction snapshot."""
        with self._lock:
            self._ierr = np.zeros(self._num_joints)
            self._last_correction = np.zeros(self._num_joints)
            self._integral_frozen = False

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
            }

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
