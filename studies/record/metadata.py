"""What a dataset has to carry to stay interpretable.

A run is worth keeping only if, months later, someone can still say which hand
it came from, what calibration was in force, which code produced it, and what
had been done to the tendons beforehand. None of that is recoverable
afterwards: ``calibration.yaml`` is not tracked and is rewritten by the next
calibration, gains come from three layered sources, and what the operator did
before the run leaves no trace at all.

So configuration is embedded verbatim rather than referenced, derived state is
recorded alongside the file it came from, and a run that moves the hand refuses
to start until its tendon history has been declared.
"""

from __future__ import annotations

import hashlib
import os
import platform
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

import orca_core
from orca_core.control import constants as control_constants
from orca_core.hardware.sensing.constants import (
    ENCODER_LSB_DEG,
    JOINT_TO_ENCODER_SLOT,
    joint_encoder_polarity_for_side,
)


# Modules whose contents change what a run means. Hashed so a dataset can be
# matched to the exact behaviour that produced it, dirty tree or not.
TRACKED_MODULES = (
    "control/joint_loop.py",
    "control/joint_controller.py",
    "control/constants.py",
)

SCHEDULING_PROBE_S = 0.5
SCHEDULING_PROBE_SLEEP_S = 0.001


class RitualNotDeclared(RuntimeError):
    """A run that moves the hand was started without its tendon history."""


@dataclass
class Ritual:
    """What was done to the hand before the run, and how long ago.

    Tension state moves hysteresis, the map's offset, and probably the response
    time. No code can observe it, so it is declared.
    """

    tensioned_at: Optional[float] = None
    jittered_at: Optional[float] = None
    calibrated_at: Optional[float] = None
    notes: str = ""

    def as_dict(self) -> Dict[str, Any]:
        now = time.time()
        return {
            "declared": True,
            "tensioned_at": self.tensioned_at,
            "jittered_at": self.jittered_at,
            "calibrated_at": self.calibrated_at,
            "minutes_since_tension": _minutes_since(self.tensioned_at, now),
            "minutes_since_jitter": _minutes_since(self.jittered_at, now),
            "minutes_since_calibration": _minutes_since(self.calibrated_at, now),
            "notes": self.notes,
        }


def _minutes_since(stamp: Optional[float], now: float) -> Optional[float]:
    if stamp is None:
        return None
    return round((now - stamp) / 60.0, 2)


def collect_metadata(
    hand,
    experiment: str,
    params: Optional[Dict[str, Any]] = None,
    *,
    moves_hand: bool,
    ritual: Optional[Ritual] = None,
) -> Dict[str, Any]:
    """Assemble the manifest for one run.

    ``hand`` may be ``None`` for a run that talks to the sensing link directly
    and never constructs a hand; the hand-specific sections are then absent
    rather than empty, so a reader can tell "not applicable" from "not
    recorded".

    ``moves_hand`` decides whether tendon history is required: a run that only
    listens has none to declare, and demanding it would train people to type
    something to get past the check.
    """
    if moves_hand and ritual is None:
        raise RitualNotDeclared(
            "this run commands the hand, so its tendon history must be declared: "
            "when it was last tensioned, jittered and calibrated. Pass a Ritual, "
            "or declare it absent explicitly if that is really the situation."
        )

    manifest = {
        "experiment": experiment,
        "params": dict(params or {}),
        "run_id": _run_id(experiment, hand),
        "clock": {
            "t0_mono": time.monotonic(),
            "t0_wall": time.time(),
            "timezone": time.strftime("%Z%z"),
        },
        "host": _host(),
        "code": _code(),
        "ritual": (ritual.as_dict() if ritual else {"declared": False}),
    }
    if hand is not None:
        manifest.update(
            hand=_hand_identity(hand),
            calibration=_calibration(hand),
            control=_control(hand),
            encoder=_encoder(hand),
        )
    return manifest


def _run_id(experiment: str, hand) -> str:
    stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
    side = getattr(getattr(hand, "config", None), "type", None) or "nohand"
    return f"{stamp}_{experiment}_{side}"


def _host() -> Dict[str, Any]:
    return {
        "platform": platform.platform(),
        "python": sys.version.split()[0],
        "switch_interval_s": sys.getswitchinterval(),
        "scheduling_noise": measure_scheduling_noise(),
    }


def measure_scheduling_noise(
    duration_s: Optional[float] = None,
    sleep_s: float = SCHEDULING_PROBE_SLEEP_S,
) -> Dict[str, float]:
    """How late this machine wakes a sleeping thread, with nothing else running.

    Every timestamp in a dataset is taken by a Python thread, so delivery jitter
    and scheduler jitter arrive mixed together. This is the floor: jitter at or
    below it says nothing about the hardware.
    """
    if duration_s is None:
        duration_s = SCHEDULING_PROBE_S
    deadline = time.monotonic() + duration_s
    overshoot: List[float] = []
    while time.monotonic() < deadline:
        start = time.monotonic()
        time.sleep(sleep_s)
        overshoot.append((time.monotonic() - start - sleep_s) * 1000.0)
    if not overshoot:
        return {}
    values = np.array(overshoot)
    return {
        "samples": int(values.size),
        "sleep_ms": sleep_s * 1000.0,
        "median_ms": float(np.median(values)),
        "p99_ms": float(np.percentile(values, 99)),
        "max_ms": float(values.max()),
    }


def _code() -> Dict[str, Any]:
    package_root = Path(orca_core.__file__).parent
    return {
        "git": _git(package_root.parent),
        # Which checkout is imported, which is not always the one being edited.
        "package_path": str(package_root),
        "module_hashes": {
            name: _sha256(package_root / name) for name in TRACKED_MODULES
        },
    }


def _git(repo: Path) -> Dict[str, Any]:
    def run(*args: str) -> Optional[str]:
        try:
            return subprocess.run(
                ["git", "-C", str(repo), *args],
                capture_output=True,
                text=True,
                timeout=5,
                check=True,
            ).stdout.strip()
        except (subprocess.SubprocessError, OSError):
            return None

    status = run("status", "--porcelain")
    return {
        "sha": run("rev-parse", "HEAD"),
        "branch": run("rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(status) if status is not None else None,
        "dirty_files": status.splitlines() if status else [],
    }


def _sha256(path: Path) -> Optional[str]:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _hand_identity(hand) -> Dict[str, Any]:
    config = hand.config
    config_path = Path(getattr(config, "config_path", "") or "")
    calibration_path = Path(getattr(config, "calibration_path", "") or "")
    return {
        "type": getattr(config, "type", None),
        "class": type(hand).__name__,
        "motor_type": getattr(config, "motor_type", None),
        "config_path": str(config_path),
        "config_sha256": _sha256(config_path),
        # Embedded, not referenced: calibration.yaml is untracked and the next
        # calibration overwrites it, so a path alone goes stale immediately.
        "config_yaml": _read_text(config_path),
        "calibration_path": str(calibration_path),
        "calibration_sha256": _sha256(calibration_path),
        "calibration_yaml": _read_text(calibration_path),
        "calibration_mtime": (
            calibration_path.stat().st_mtime if calibration_path.exists() else None
        ),
    }


def _read_text(path: Path) -> Optional[str]:
    if not path or not path.exists():
        return None
    return path.read_text()


def _calibration(hand) -> Dict[str, Any]:
    calibration = hand.calibration
    config_roms = hand.config.joint_roms_dict
    effective_roms = hand.effective_joint_roms_dict
    return {
        "calibrated": calibration.calibrated,
        "wrist_calibrated": calibration.wrist_calibrated,
        "motor_limits": _plain(calibration.motor_limits_dict),
        "joint_to_motor_ratios": _plain(calibration.joint_to_motor_ratios_dict),
        "encoder_anchors": {
            joint: entry.enc_at_anchor_count
            for joint, entry in calibration.joint_encoder_calibration_dict.items()
        },
        "joint_roms_measured": _plain(calibration.joint_roms_measured_dict),
        "effective_joint_roms": _plain(effective_roms),
        # The map runs in the effective frame while commands are clamped to the
        # config one, so where they differ a command cannot reach part of the
        # range the map is defined over.
        "rom_delta": {
            joint: [
                round(effective_roms[joint][0] - config_roms[joint][0], 4),
                round(effective_roms[joint][1] - config_roms[joint][1], 4),
            ]
            for joint in config_roms
            if joint in effective_roms
        },
        "wrap_offsets": _plain(getattr(hand, "_wrap_offsets_dict", None)),
    }


def _control(hand) -> Dict[str, Any]:
    constants = {
        name: getattr(control_constants, name)
        for name in dir(control_constants)
        if name.isupper()
    }
    gains: Dict[str, Any]
    loop_joints = None
    try:
        # Only a hand with a running loop can report installed gains; on any
        # other the configured values are what a loop would be given.
        gains = {
            joint: _plain(vars(entry))
            for joint, entry in hand.get_pid_gains().items()
        }
        loop_joints = list(hand.loop_joint_names or [])
        source = "installed"
    except (AttributeError, RuntimeError):
        gains = {
            joint: _plain(vars(hand.config.joint_gains_for(joint)))
            for joint in hand.config.joint_ids
        }
        source = "configured"
    return {
        "gains": gains,
        "gains_source": source,
        "loop_joints": loop_joints,
        "constants": _plain(constants),
    }


def _encoder(hand) -> Dict[str, Any]:
    side = getattr(hand.config, "type", None)
    try:
        polarity = _plain(joint_encoder_polarity_for_side(side))
    except (ValueError, KeyError):
        polarity = None
    return {
        "lsb_deg": ENCODER_LSB_DEG,
        "joint_to_slot": dict(JOINT_TO_ENCODER_SLOT),
        "polarity": polarity,
    }


def _plain(value: Any) -> Any:
    """Convert numpy scalars, arrays and containers into JSON-safe values."""
    if value is None:
        return None
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value
