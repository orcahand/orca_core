"""Slow adaptive feed-forward bias for the host-side joint loop.

The PI integrator converges to whatever constant offset the joint→motor
map currently gets wrong (slack buildup, calibration bias). This adapter
drains that persistent trim into a stored per-joint bias with a
minutes-scale time constant, so the integrator keeps its headroom for
genuine disturbances and large target moves don't pay a re-convergence
transient for the same offset every time.

Only offsets adapt — never the map slope — and only on cycles that pass
every gate:

* the target has been stationary for ``settle_s``;
* the filtered joint speed is below ``vel_max_deg_s``;
* the controller is not saturated (output clamp or integral clamp) — a
  blocked finger winds the integrator into its clamp, so this gate also
  rejects most sustained external-force cases;
* the integral trim exceeds ``deadband_deg`` (don't chase encoder noise).

The per-cycle transfer is ``integral_out * dt / tau_s`` and the
accumulated bias is hard-clamped to ``±max_bias_deg``; a joint first
hitting the clamp logs a warning, since offset beyond the clamp means
the hand wants recalibrating. With adaptation at minutes, PI at seconds
and the motor PID at milliseconds, the three loops are time-scale
separated and cannot interact dynamically.
"""

import logging
import threading
from typing import Dict, Optional, Sequence

import numpy as np

from .constants import (
    BIAS_ADAPT_DEADBAND_DEG,
    BIAS_ADAPT_MAX_DEG,
    BIAS_ADAPT_SETTLE_S,
    BIAS_ADAPT_TAU_S,
    BIAS_ADAPT_VEL_FILTER_TAU_S,
    BIAS_ADAPT_VEL_MAX_DEG_S,
)


logger = logging.getLogger(__name__)

_TARGET_CHANGE_EPS_DEG = 1e-9


class BiasAdapter:
    """Gated leaky transfer of persistent PI trim into a per-joint bias."""

    def __init__(
        self,
        num_joints: int,
        joint_names: Optional[Sequence[str]] = None,
        *,
        tau_s: float = BIAS_ADAPT_TAU_S,
        max_bias_deg: float = BIAS_ADAPT_MAX_DEG,
        deadband_deg: float = BIAS_ADAPT_DEADBAND_DEG,
        settle_s: float = BIAS_ADAPT_SETTLE_S,
        vel_max_deg_s: float = BIAS_ADAPT_VEL_MAX_DEG_S,
        vel_filter_tau_s: float = BIAS_ADAPT_VEL_FILTER_TAU_S,
    ):
        if num_joints <= 0:
            raise ValueError("num_joints must be positive")
        if joint_names is not None and len(joint_names) != num_joints:
            raise ValueError(
                f"joint_names has {len(joint_names)} entries for {num_joints} joints"
            )
        if tau_s <= 0:
            raise ValueError("tau_s must be positive")
        for name, value in (
            ("max_bias_deg", max_bias_deg),
            ("deadband_deg", deadband_deg),
            ("settle_s", settle_s),
            ("vel_max_deg_s", vel_max_deg_s),
        ):
            if value < 0:
                raise ValueError(f"{name} must be non-negative")
        if vel_filter_tau_s <= 0:
            raise ValueError("vel_filter_tau_s must be positive")

        self._num_joints = int(num_joints)
        self._joint_names = list(joint_names) if joint_names is not None else None
        self._tau_s = float(tau_s)
        self._max_bias_deg = float(max_bias_deg)
        self._deadband_deg = float(deadband_deg)
        self._settle_s = float(settle_s)
        self._vel_max_deg_s = float(vel_max_deg_s)
        self._vel_filter_tau_s = float(vel_filter_tau_s)

        # step() runs on the loop thread while reset()/bias_deg may be called
        # from the user thread (hand.reset_adaptive_bias / get_adaptive_bias);
        # the lock keeps a reset from being lost inside step()'s
        # read-modify-write of the bias.
        self._mutex = threading.Lock()
        self._bias = np.zeros(self._num_joints)
        self._settle_elapsed = np.zeros(self._num_joints)
        self._vel_filt = np.zeros(self._num_joints)
        self._at_clamp = np.zeros(self._num_joints, dtype=bool)
        self._prev_target: Optional[np.ndarray] = None
        self._prev_measured: Optional[np.ndarray] = None

    @property
    def num_joints(self) -> int:
        return self._num_joints

    @property
    def bias_deg(self) -> np.ndarray:
        with self._mutex:
            return self._bias.copy()

    def step(
        self,
        target_deg: np.ndarray,
        measured_deg: np.ndarray,
        integral_out_deg: np.ndarray,
        saturated: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """Advance one healthy loop cycle. Returns the per-joint amount
        (degrees) actually transferred into the bias this cycle; the caller
        drains the same amount from the controller integral so the total
        command does not jump."""
        target = self._check("target_deg", target_deg)
        measured = self._check("measured_deg", measured_deg)
        integral_out = self._check("integral_out_deg", integral_out_deg)
        sat = np.asarray(saturated, dtype=bool)
        if sat.shape != (self._num_joints,):
            raise ValueError(
                f"saturated must have shape ({self._num_joints},), got {sat.shape}"
            )
        dt = float(dt)
        with self._mutex:
            if dt <= 0 or self._prev_target is None or self._prev_measured is None:
                self._prev_target = target.copy()
                self._prev_measured = measured.copy()
                return np.zeros(self._num_joints)

            moved = np.abs(target - self._prev_target) > _TARGET_CHANGE_EPS_DEG
            self._settle_elapsed = np.where(moved, 0.0, self._settle_elapsed + dt)
            self._prev_target = target.copy()

            vel = (measured - self._prev_measured) / dt
            alpha = dt / (self._vel_filter_tau_s + dt)
            self._vel_filt += alpha * (vel - self._vel_filt)
            self._prev_measured = measured.copy()

            allow = (
                (self._settle_elapsed >= self._settle_s)
                & (np.abs(self._vel_filt) <= self._vel_max_deg_s)
                & ~sat
                & (np.abs(integral_out) > self._deadband_deg)
            )
            delta = np.where(allow, integral_out * (dt / self._tau_s), 0.0)
            new_bias = np.clip(
                self._bias + delta, -self._max_bias_deg, self._max_bias_deg
            )
            transferred = new_bias - self._bias
            self._bias = new_bias
            self._warn_new_clamp_hits()
            return transferred

    def reset(self) -> None:
        """Zero the bias and all gate state."""
        with self._mutex:
            self._bias.fill(0.0)
            self._settle_elapsed.fill(0.0)
            self._vel_filt.fill(0.0)
            self._at_clamp.fill(False)
            self._prev_target = None
            self._prev_measured = None

    def get_state(self) -> Dict[str, np.ndarray]:
        with self._mutex:
            return {
                "bias_deg": self._bias.copy(),
                "settle_elapsed_s": self._settle_elapsed.copy(),
                "vel_filt_deg_s": self._vel_filt.copy(),
                "at_clamp": self._at_clamp.copy(),
            }

    def _warn_new_clamp_hits(self) -> None:
        at_clamp = np.abs(self._bias) >= self._max_bias_deg - 1e-12
        newly = at_clamp & ~self._at_clamp
        if np.any(newly):
            for idx in np.flatnonzero(newly):
                name = (
                    self._joint_names[idx]
                    if self._joint_names is not None
                    else f"joint[{idx}]"
                )
                logger.warning(
                    "adaptive feed-forward bias for %s hit the ±%.1f° clamp; "
                    "the joint→motor map is off by more than the adapter can "
                    "absorb — consider recalibrating",
                    name,
                    self._max_bias_deg,
                )
        self._at_clamp = at_clamp

    def _check(self, name: str, value: np.ndarray) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        if array.shape != (self._num_joints,):
            raise ValueError(
                f"{name} must have shape ({self._num_joints},), got {array.shape}"
            )
        return array
