"""Vectorised per-joint PID controller for current-mode joint loops.

The control law uses derivative-on-measurement to avoid a derivative kick
on setpoint steps, a first-order low-pass filter on the derivative path
to keep encoder quantisation out of the motor command, and conditional
integration so the integrator only winds when it can drive the output
toward the setpoint.
"""

from __future__ import annotations

from typing import Dict, Union

import numpy as np

from .constants import MAX_LOOP_DT_S, MIN_LOOP_DT_S

ScalarOrArray = Union[float, np.ndarray]

DEFAULT_DERIVATIVE_FILTER_TAU_S = 0.005


class JointPIDController:
    """Per-channel PID with shared scalar or per-joint vector gains.

    Output is signed Goal_Current in milliamps. Motor polarity is applied
    by the loop thread, not here.
    """

    def __init__(self, num_joints: int):
        if num_joints <= 0:
            raise ValueError("num_joints must be positive")
        self._n = int(num_joints)
        self._Kp = np.zeros(self._n)
        self._Ki = np.zeros(self._n)
        self._Kd = np.zeros(self._n)
        self._i_clamp = np.zeros(self._n)
        self._i_max = np.zeros(self._n)
        self._derivative_filter_tau_s = DEFAULT_DERIVATIVE_FILTER_TAU_S
        self._target = np.zeros(self._n)
        self._ierr = np.zeros(self._n)
        self._prev_measured = np.zeros(self._n)
        self._prev_measured_valid = False
        self._derr_filtered = np.zeros(self._n)
        self._last_u = np.zeros(self._n)
        self._integral_frozen = False

    @property
    def num_joints(self) -> int:
        return self._n

    def set_gains(
        self,
        Kp: ScalarOrArray,
        Ki: ScalarOrArray,
        Kd: ScalarOrArray,
        i_clamp_mA: ScalarOrArray,
        i_max_mA: ScalarOrArray,
        derivative_filter_tau_s: float = DEFAULT_DERIVATIVE_FILTER_TAU_S,
    ) -> None:
        """Set per-channel gains and clamps.

        Each gain is a scalar (broadcast) or an array of length
        ``num_joints``. All values must be non-negative.

        ``derivative_filter_tau_s`` is the time constant of a first-order
        low-pass filter on the derivative path. Set to 0 to disable.
        """
        self._Kp = self._broadcast(Kp, "Kp")
        self._Ki = self._broadcast(Ki, "Ki")
        self._Kd = self._broadcast(Kd, "Kd")
        self._i_clamp = self._broadcast(i_clamp_mA, "i_clamp_mA")
        self._i_max = self._broadcast(i_max_mA, "i_max_mA")
        if derivative_filter_tau_s < 0:
            raise ValueError("derivative_filter_tau_s must be non-negative")
        self._derivative_filter_tau_s = float(derivative_filter_tau_s)

    def set_target(self, target_rad: np.ndarray) -> None:
        target = np.asarray(target_rad, dtype=np.float64)
        if target.shape != (self._n,):
            raise ValueError(f"target must have shape ({self._n},), got {target.shape}")
        self._target = target.copy()

    def get_target(self) -> np.ndarray:
        return self._target.copy()

    def step(self, measured_rad: np.ndarray, dt: float) -> np.ndarray:
        """Advance one cycle and return signed mA per channel."""
        measured = np.asarray(measured_rad, dtype=np.float64)
        if measured.shape != (self._n,):
            raise ValueError(f"measured must have shape ({self._n},), got {measured.shape}")
        dt_clamped = float(np.clip(dt, MIN_LOOP_DT_S, MAX_LOOP_DT_S))

        err = self._target - measured

        # Derivative on measurement: a setpoint step contributes nothing,
        # only real motion drives the D term.
        if self._prev_measured_valid:
            derr_raw = -(measured - self._prev_measured) / dt_clamped
        else:
            derr_raw = np.zeros(self._n)
        self._prev_measured = measured.copy()
        self._prev_measured_valid = True

        # First-order LPF on the derivative path.
        tau = self._derivative_filter_tau_s
        if tau > 0.0:
            alpha = dt_clamped / (tau + dt_clamped)
            self._derr_filtered = self._derr_filtered + alpha * (derr_raw - self._derr_filtered)
            derr = self._derr_filtered
        else:
            self._derr_filtered = derr_raw
            derr = derr_raw

        # Tentative output without the integral contribution, used to
        # decide whether the integrator should be allowed to wind further.
        u_no_i = self._Kp * err + self._Kd * derr
        u_unsat = u_no_i + self._Ki * self._ierr

        # Conditional integration: only update ierr when not saturated, or
        # when the err sign would unwind us out of saturation.
        if not self._integral_frozen:
            saturated = np.abs(u_unsat) >= self._i_max
            pushing_into_sat = np.sign(err) == np.sign(u_unsat)
            allow_integrate = ~(saturated & pushing_into_sat)
            new_ierr = np.where(
                allow_integrate, self._ierr + err * dt_clamped, self._ierr
            )
            self._ierr = np.clip(new_ierr, -self._i_clamp, self._i_clamp)

        u = np.clip(u_no_i + self._Ki * self._ierr, -self._i_max, self._i_max)
        self._last_u = u
        return u.copy()

    def reset(self) -> None:
        """Zero all internal state. Target is preserved."""
        self._ierr.fill(0.0)
        self._prev_measured.fill(0.0)
        self._prev_measured_valid = False
        self._derr_filtered.fill(0.0)
        self._last_u.fill(0.0)
        self._integral_frozen = False

    def freeze_integral(self) -> None:
        self._integral_frozen = True

    def unfreeze_integral(self) -> None:
        self._integral_frozen = False

    @property
    def integral_frozen(self) -> bool:
        return self._integral_frozen

    def get_state(self) -> Dict[str, np.ndarray]:
        return {
            "ierr": self._ierr.copy(),
            "prev_measured": self._prev_measured.copy(),
            "derr_filtered": self._derr_filtered.copy(),
            "last_u_mA": self._last_u.copy(),
        }

    def _broadcast(self, value: ScalarOrArray, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        if array.ndim == 0:
            array = np.full(self._n, float(array))
        elif array.shape != (self._n,):
            raise ValueError(
                f"{name} must be scalar or shape ({self._n},), got {array.shape}"
            )
        if np.any(array < 0):
            raise ValueError(f"{name} must be non-negative")
        return array
