# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Pure detection logic for contact capture, unit-testable without hardware.

Two detectors:

- ``StallDetector`` — first contact while a weak pusher joint advances:
  the measured angle stops following the command (tracking error grows
  while measured velocity dies). Tendon stretch makes the stall gradual,
  so both conditions must hold for several consecutive steps; triggering
  on the ONSET keeps the contact force minimal, which is what keeps the
  parked finger unmoved and skin compression negligible.

- ``TipPressDetector`` — hand-guided fingertip press: both pads report a
  localisable resultant above threshold while the encoders are still
  (the operator is holding the pose). A cooldown prevents one press from
  spamming events.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class StallDetector:
    err_thresh_deg: float = 1.2    # commanded-minus-measured onset
    vel_thresh_deg: float = 0.06   # per step; below this the joint has stopped
    consecutive: int = 3

    _errs: list = field(default_factory=list)
    _meas: list = field(default_factory=list)

    def update(self, commanded_deg: float, measured_deg: float) -> bool:
        """Feed one settled step; True when first contact is established."""
        self._errs.append(commanded_deg - measured_deg)
        self._meas.append(measured_deg)
        if len(self._meas) < self.consecutive + 1:
            return False
        errs = np.abs(self._errs[-self.consecutive:])
        vels = np.abs(np.diff(self._meas[-(self.consecutive + 1):]))
        return bool((errs > self.err_thresh_deg).all()
                    and (vels < self.vel_thresh_deg).all())

    def reset(self) -> None:
        self._errs.clear()
        self._meas.clear()


@dataclass
class TipPressDetector:
    force_thresh_n: float = 0.35
    still_deg: float = 0.15        # max encoder change over the hold window
    hold_samples: int = 4          # consecutive qualifying samples
    cooldown_samples: int = 20

    _hold: int = 0
    _cooldown: int = 0
    _last_m: np.ndarray | None = None

    @property
    def hold(self) -> int:
        """Qualifying samples accumulated toward a capture — a live UI can
        show whether force or stillness is the blocker."""
        return self._hold

    def update(self, force_a: float, force_b: float,
               m: np.ndarray) -> bool:
        """Feed one sample (both pads' resultants + encoder row); True on
        a capture-worthy press."""
        if self._cooldown > 0:
            self._cooldown -= 1
            self._last_m = m
            return False
        still = (self._last_m is not None
                 and np.nanmax(np.abs(m - self._last_m)) < self.still_deg)
        self._last_m = m
        if min(force_a, force_b) >= self.force_thresh_n and still:
            self._hold += 1
        else:
            self._hold = 0
        if self._hold >= self.hold_samples:
            self._hold = 0
            self._cooldown = self.cooldown_samples
            return True
        return False


def taxel_centroid(positions: np.ndarray, forces: np.ndarray
                   ) -> tuple[np.ndarray, float]:
    """Force-weighted contact centroid in the sensor frame.

    ``positions`` (n, 3) static taxel geometry; ``forces`` (n, 3) per-taxel
    force vectors. Returns ``(centroid (3,), resultant_n)``. Weights use
    force magnitudes clipped at zero — a pressed pad reports a localized
    blob and the centroid interpolates below the 3.1 mm taxel pitch.
    """
    mag = np.linalg.norm(np.asarray(forces, float), axis=1)
    mag = np.clip(mag, 0.0, None)
    total = float(mag.sum())
    if total <= 0:
        return np.full(3, np.nan), 0.0
    centroid = (np.asarray(positions, float) * mag[:, None]).sum(axis=0) / total
    resultant = float(np.linalg.norm(np.asarray(forces, float).sum(axis=0)))
    return centroid, resultant
