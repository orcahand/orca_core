"""orca_core's linear motor <-> joint map, rebuilt from one calibration run's archived files,
so each run's assumed mapping can be compared offline.

A run archived from orca_core 0.14 on holds ``hand_map.yaml`` (the hand's model and, per joint,
the motor, direction and range it ran with; see ``hand_map``) and the hand folder's
``calibration.yaml`` (motor limits) and ``joint_ranges.yaml`` (ranges measured on the hand); an
older run holds a full ``config.yaml`` instead. Limits, ratios and the
frame they map onto come from orca_core, as the hand puts them in force at connect (ratios an
older calibration.yaml stored are not read), and an archive whose map orca_core would not give is
refused. A joint orca_core holds uncalibrated is left out of ``joints_of``, as the hand reads it
as None."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

QUARTER = np.pi / 4
HAND_MAP = "hand_map.yaml"
HAND_FILES = ("calibration.yaml", "joint_ranges.yaml", "setup.yaml")


def _read(path: Path) -> dict:
    return (yaml.safe_load(path.read_text()) or {}) if path.exists() else {}


def calibration_in_force(config, calibration: dict) -> tuple[dict, dict, dict]:
    """(motor limits, ratios, joint frame) as an OrcaHand with `config` puts `calibration` in force:
    orca_core's calibration.in_force (a joint whose motor mapping changed loses its limits) and the
    frame of OrcaHand.effective_joint_roms_dict. Motors without limits or a ratio are left out."""
    from orca_core.calibration import FINGERPRINT, in_force, result_from_mapping
    from orca_core.hardware_hand import ROM_FRAME_CENTERED, ROM_FRAME_KEY
    result, _ = in_force(result_from_mapping(calibration, config.motor_ids), calibration.get(FINGERPRINT), config,
                         prefer_config=config.measured_joint_roms_dict)
    limits = {int(m): (float(v[0]), float(v[1])) for m, v in result.motor_limits_dict.items()
              if v and len(v) == 2 and None not in v}
    ratios = {int(m): float(r) for m, r in result.joint_to_motor_ratios_dict.items() if r}
    frame = {j: (float(r[0]), float(r[1])) for j, r in config.joint_roms_dict.items()}
    centered = calibration.get(ROM_FRAME_KEY) == ROM_FRAME_CENTERED
    for j, rom in result.joint_roms_measured_dict.items():
        if j not in frame or j in config.measured_joint_roms_dict:
            continue
        lo, hi = frame[j]
        half = ((rom[1] - rom[0]) - (hi - lo)) / 2.0
        frame[j] = (lo - half, hi + half) if centered else (float(rom[0]), float(rom[1]))
    return limits, ratios, frame


def orca_core_config(model: str, joint_ranges: dict | None = None):
    """orca_core's config of the packaged `model` with the hand's measured ranges in force."""
    from orca_core.hand_config import OrcaHandConfig
    config = OrcaHandConfig.from_config_path(model_name=model)
    return config.with_measured_ranges(joint_ranges) if joint_ranges else config


class LinearMap:
    def __init__(self, run_dir: Path):
        run_dir = Path(run_dir)
        cal = _read(run_dir / "calibration.yaml")
        if (run_dir / HAND_MAP).exists():
            ran = _read(run_dir / HAND_MAP)
            config = orca_core_config(ran["model"], _read(run_dir / "joint_ranges.yaml").get("joint_roms"))
            self._in_force(cal, config, list(ran["joints"]))
            off = [j for j, v in ran["joints"].items() if (abs(int(v["motor"])), bool(v["inverted"])) != (self.motor[j], self.inverted[j])
                   or not np.allclose(v["rom"], config.joint_roms_dict[j], atol=1e-6)]
            if off:
                raise ValueError(f"{run_dir}: orca_core's {ran['model']} with the archived joint ranges maps "
                                 f"{', '.join(off)} otherwise than the hand ran")
            return
        cfg = _read(run_dir / "config.yaml")
        if "joint_roms" not in cfg:
            raise ValueError(f"{run_dir} holds no {HAND_MAP} and no complete config.yaml: "
                             f"archive the run from the hand's folder (night.py archive_hand)")
        from orca_core.calibration import _derive_joint_to_motor_ratios
        self.limits = {int(k): tuple(map(float, v)) for k, v in (cal.get("motor_limits") or {}).items()
                       if v and len(v) == 2 and None not in v}
        self.joints = list(cfg["joint_ids"])
        self.motor = {j: abs(int(m)) for j, m in cfg["joint_to_motor_map"].items()}
        self.inverted = {j: int(m) < 0 for j, m in cfg["joint_to_motor_map"].items()}
        self.rom = {j: tuple(map(float, r)) for j, r in cfg["joint_roms"].items()}
        self.ratio = {m: r for m, r in _derive_joint_to_motor_ratios(cal.get("motor_limits") or {}, self.motor,
                                                                     self.rom).items() if r}

    @classmethod
    def of_hand(cls, folder: Path, model: str) -> LinearMap:
        """The map a hand of `model` runs with, from its folder's calibration.yaml and joint_ranges.yaml."""
        self = cls.__new__(cls)
        folder = Path(folder)
        self._in_force(_read(folder / "calibration.yaml"),
                       orca_core_config(model, _read(folder / "joint_ranges.yaml").get("joint_roms")))
        return self

    def _in_force(self, cal: dict, config, joints: list[str] | None = None) -> None:
        self.limits, self.ratio, frame = calibration_in_force(config, cal)
        self.joints = joints or list(config.joint_ids)
        self.motor = {j: abs(int(config.joint_to_motor_map[j])) for j in self.joints}
        self.inverted = {j: bool(config.joint_inversion_dict.get(j, False)) for j in self.joints}
        self.rom = {j: frame[j] for j in self.joints}

    def calibrated(self, j: str) -> bool:
        return self.motor[j] in self.limits and self.motor[j] in self.ratio

    def unwrap(self, j: str, m: float) -> float:
        lo, hi = self.limits[self.motor[j]]
        while m > hi + QUARTER:
            m -= 2 * np.pi
        while m < lo - QUARTER:
            m += 2 * np.pi
        return m

    def to_joint(self, j: str, m: float) -> float:
        mid = self.motor[j]
        if not self.calibrated(j):
            raise ValueError(f"{j} (motor {mid}) is not calibrated")
        d = (self.unwrap(j, m) - self.limits[mid][0]) / self.ratio[mid]
        lo, hi = self.rom[j]
        return hi - d if self.inverted[j] else lo + d

    def to_motor(self, j: str, q: float) -> float:
        mid = self.motor[j]
        lo, hi = self.rom[j]
        return self.limits[mid][0] + ((hi - q) if self.inverted[j] else (q - lo)) * self.ratio[mid]

    def joints_of(self, motor_ids, motor_pos) -> dict[str, float]:
        pos = dict(zip(motor_ids, motor_pos))
        return {j: self.to_joint(j, pos[self.motor[j]]) for j in self.joints if self.calibrated(j)}


def hand_map(info: dict) -> dict:
    """``hand_map.yaml`` from orca-ui's ``/api/hand/info``: the connected hand's id, folder, model
    and, per joint, its motor, direction and range in force (measured ranges included)."""
    hand = info.get("hand") or {}
    return {"hand": hand.get("id"), "folder": hand.get("folder"), "model": info.get("model_name"),
            "joint_ranges": (hand.get("joint_ranges") or {}).get("source", "model"),
            "joints": {j["id"]: {"motor": int(j["motor_id"]), "inverted": bool(j["inverted"]),
                                 "rom": [float(v) for v in j["rom"]]} for j in info["joints"]}}
