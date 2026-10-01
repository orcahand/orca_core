"""The replay examples must ease onto the first recorded pose.

Playback starts from wherever ``init_joints()`` left the hand, so sending the
first waypoint straight out makes the hand snap to it at full speed. Both
examples interpolate that move like any other transition instead.
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
MODEL = "orcahand-right"


def _load(rel_path: str):
    path = REPO_ROOT / rel_path
    spec = importlib.util.spec_from_file_location(f"_example_{path.stem}", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    sys.path.insert(0, str(path.parent))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(path.parent))
    return module


@pytest.fixture(scope="module")
def poses():
    """A start pose and a distant target pose, both inside the config ROMs."""
    from orca_core import load_hand

    config = load_hand(mock=True, model_name=MODEL).config
    roms = config.joint_roms_dict
    joint_ids = config.joint_ids
    target = [float(roms[j][0] + 0.7 * (roms[j][1] - roms[j][0])) for j in joint_ids]
    neutral = [float(config.neutral_position.get(j, 0.0)) for j in joint_ids]
    tail = [(a + b) / 2 for a, b in zip(neutral, target)]
    return joint_ids, neutral, target, tail


def _write(path: Path, payload: dict) -> Path:
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return path


def _run(module, argv, joint_ids):
    """Run an example on a mock hand, returning every joint vector it commanded."""
    commands: list[list[float]] = []
    build = module.create_hand_from_args

    def spy(args, **overrides):
        hand = build(args, **overrides)
        send = hand._set_joint_positions

        def record(joint_pos):
            commands.append(joint_pos.as_list(joint_ids))
            return send(joint_pos)

        hand._set_joint_positions = record
        return hand

    module.create_hand_from_args = spy
    argv_backup = sys.argv
    sys.argv = argv
    try:
        assert module.main() == 0
    finally:
        module.create_hand_from_args = build
        sys.argv = argv_backup
    return commands


def _approach(commands, neutral, target):
    """Commands issued between settling on *neutral* and first reaching *target*."""
    arrival = next(
        i for i, pose in enumerate(commands) if np.allclose(pose, target, atol=1e-3)
    )
    settled = max(
        i for i, pose in enumerate(commands[:arrival])
        if np.allclose(pose, neutral, atol=1e-3)
    )
    return commands[settled + 1:arrival]


def test_waypoint_replay_eases_onto_the_first_waypoint(tmp_path, poses):
    joint_ids, neutral, target, tail = poses
    module = _load("examples/replay_angles.py")
    replay = _write(tmp_path / "wp.yaml", {
        "metadata": {"type": "discrete_waypoints", "joint_ids": joint_ids},
        "waypoints": [target, tail],
    })
    wrist = joint_ids.index("wrist")
    expected = list(target)
    expected[wrist] = 0.0  # replay pins the wrist

    commands = _run(module, [
        "replay_angles.py", "--mock", "--model-name", MODEL,
        "--replay-file", str(replay), "--step-time", "0.001", "--transition-time", "0.02",
    ], joint_ids)

    assert len(_approach(commands, neutral, expected)) > 1


def test_continuous_replay_eases_onto_the_first_frame(tmp_path, poses):
    joint_ids, neutral, target, tail = poses
    module = _load("examples/replay_continuous.py")
    replay = _write(tmp_path / "cont.yaml", {
        "metadata": {"type": "continuous", "sampling_frequency_hz": 1000.0,
                     "joint_ids": joint_ids},
        "angles": [target, tail],
    })

    commands = _run(module, [
        "replay_continuous.py", "--mock", "--model-name", MODEL,
        "--replay-file", str(replay), "--approach-time", "0.05",
    ], joint_ids)

    assert len(_approach(commands, neutral, target)) > 1
