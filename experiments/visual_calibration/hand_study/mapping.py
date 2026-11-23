"""orca_core's linear motor <-> joint map, rebuilt from one calibration run's files
(config.yaml + calibration.yaml), so each run's assumed mapping can be compared offline."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

QUARTER = np.pi / 4


class LinearMap:
    def __init__(self, run_dir: Path):
        cfg = yaml.safe_load((Path(run_dir) / "config.yaml").read_text())
        cal = yaml.safe_load((Path(run_dir) / "calibration.yaml").read_text())
        self.joints = list(cfg["joint_ids"])
        self.motor = {j: abs(int(m)) for j, m in cfg["joint_to_motor_map"].items()}
        self.inverted = {j: int(m) < 0 for j, m in cfg["joint_to_motor_map"].items()}
        self.rom = {j: tuple(map(float, r)) for j, r in cfg["joint_roms"].items()}
        self.limits = {int(k): tuple(map(float, v)) for k, v in cal["motor_limits"].items()}
        self.ratio = {int(k): float(v) for k, v in cal["joint_to_motor_ratios"].items()}

    def unwrap(self, j: str, m: float) -> float:
        lo, hi = self.limits[self.motor[j]]
        while m > hi + QUARTER:
            m -= 2 * np.pi
        while m < lo - QUARTER:
            m += 2 * np.pi
        return m

    def to_joint(self, j: str, m: float) -> float:
        mid = self.motor[j]
        d = (self.unwrap(j, m) - self.limits[mid][0]) / self.ratio[mid]
        lo, hi = self.rom[j]
        return hi - d if self.inverted[j] else lo + d

    def to_motor(self, j: str, q: float) -> float:
        mid = self.motor[j]
        lo, hi = self.rom[j]
        return self.limits[mid][0] + ((hi - q) if self.inverted[j] else (q - lo)) * self.ratio[mid]

    def joints_of(self, motor_ids, motor_pos) -> dict[str, float]:
        pos = dict(zip(motor_ids, motor_pos))
        return {j: self.to_joint(j, pos[self.motor[j]]) for j in self.joints}
