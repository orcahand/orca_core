<div align="center" style="line-height: 1;">
  <a href="https://arxiv.org/abs/2504.04259" target="_blank"><img alt="arXiv" src="https://img.shields.io/badge/arXiv-2504.04259-B31B1B?logo=arxiv"/></a>
  <a href="https://discord.gg/xvGyxaccRa" target="_blank"><img alt="Discord" src="https://img.shields.io/badge/Discord-orcahand-7289da?logo=discord&logoColor=white&color=7289da"/></a>
  <a href="https://x.com/orcahand" target="_blank"><img alt="Twitter Follow" src="https://img.shields.io/twitter/follow/orcahand?style=social"/></a>
  <a href="https://orcahand.com" target="_blank"><img alt="Website" src="https://img.shields.io/badge/Website-orcahand.com-blue?style=flat&logo=google-chrome"/></a>
  <br>
  <a href="https://github.com/orcahand/orca_core" target="_blank"><img alt="GitHub stars" src="https://img.shields.io/github/stars/orcahand/orca_core?style=social"/></a>
  <a href="https://github.com/orcahand/orca_core/actions/workflows/test.yml" target="_blank"><img alt="Tests" src="https://github.com/orcahand/orca_core/actions/workflows/test.yml/badge.svg"/></a>
</div>

# Orca Core

`orca_core` is the Python control package for the ORCA Hand. The refactored codebase is organized around a small set of explicit objects:

- `OrcaHandConfig` loads the static hand description from `config.yaml`.
- `CalibrationResult` loads the mutable runtime state from `calibration.yaml`.
- `BaseHand` provides backend-agnostic joint-space helpers.
- `OrcaHand` adds hardware connection, torque, calibration, telemetry, and maintenance tasks.
- `OrcaJointPositions` is the typed container used for joint-space commands.

The practical workflow is:

1. Choose a model config.
2. Connect to the hardware.
3. Call `init_joints()` to enable torque, apply motor settings, calibrate when needed, and move to neutral.
4. Command joint-space poses through `set_joint_positions(...)`.

## Install

```sh
python -m venv .venv
source .venv/bin/activate
pip install -e .
```

## Quick Start

Start by checking the model config you want to use. The bundled models live under:

- `orca_core/models/v2/orcahand_right/config.yaml`
- `orca_core/models/v1/orcahand_right/config.yaml`
- `orca_core/models/v1/orcahand_left/config.yaml`

The refactored code expects `config_path` to point to the `config.yaml` file itself, not just the model directory.
Use the `v2` config as the canonical reference for the current schema. Older `v1` configs may still need the legacy `calib_*` keys renamed to `calibration_*`.

For a first-time hardware bring-up, the recommended operator flow is:

```sh
python scripts/setup.py orca_core/models/v2/orcahand_right/config.yaml
```

If the hand is already assembled and tensioned, the shorter path is:

```sh
python scripts/calibrate.py orca_core/models/v2/orcahand_right/config.yaml
python scripts/neutral.py orca_core/models/v2/orcahand_right/config.yaml
```

## Python Usage

```python
from orca_core import OrcaHand

hand = OrcaHand(config_path="orca_core/models/v2/orcahand_right/config.yaml")

success, message = hand.connect()
if not success:
    raise RuntimeError(message)

try:
    hand.init_joints()

    neutral = hand.config.neutral_position
    hand.set_joint_positions(
        {
            "thumb_mcp": neutral["thumb_mcp"] + 10,
            "index_mcp": neutral["index_mcp"] + 10,
            "middle_mcp": neutral["middle_mcp"] + 10,
        },
        num_steps=25,
        step_size=0.01,
    )

    print(hand.get_joint_position().as_dict())
finally:
    hand.stop_task()
    hand.disconnect()
```

### Notes on units

- Joint-space values use the same units as `joint_roms` and `neutral_position` in your model config. The bundled hand configs use degrees.
- Low-level motor position telemetry returned by `get_motor_pos()` is in radians.
- Motor current is reported in mA and motor temperature in C.

## Key Scripts

- `scripts/setup.py`: guided tension -> calibrate -> verify workflow.
- `scripts/calibrate.py`: run calibration and persist `calibration.yaml`.
- `scripts/tension.py`: hold the spools in place for tendon tensioning.
- `scripts/neutral.py`: move to the configured neutral pose.
- `scripts/zero.py`: move all configured joints to zero.
- `scripts/record_angles.py` / `scripts/replay_angles.py`: capture and replay waypoint-based motions.
- `scripts/record_continuous.py` / `scripts/replay_continuous.py`: capture and replay continuous trajectories.
- `scripts/slider_joint.py` / `scripts/slider_motor.py`: manual debugging UIs.

## Configuration and calibration files

Each hand model directory contains:

- `config.yaml`: static model and hardware metadata.
- `calibration.yaml`: calibration output written after calibration runs.

The refactored config loader expects canonical calibration keys such as:

- `calibration_current`
- `calibration_step_size`
- `calibration_step_period`
- `calibration_threshold`
- `calibration_num_stable`
- `calibration_sequence`

If you are migrating an older config that still uses `calib_*` keys, rename those fields before using it with the refactored API.

## Troubleshooting

### Serial port permissions on Linux

On Linux, the serial port is usually owned by the `dialout` group. If your user is not in that group, the hand may fail to connect.

Permanent fix:

```sh
sudo usermod -aG dialout $USER
```

Temporary fix:

```sh
sudo chmod 666 /dev/ttyUSB0
```

### Port mismatch

`OrcaHand.connect()` first tries the configured port, then attempts USB auto-detection, then falls back to an interactive chooser. If connection succeeds on a different port, the package updates `config.yaml` automatically.

## Documentation

The MkDocs site in `docs/` expands on:

- hardware setup
- the `config.yaml` schema
- calibration and tensioning workflow
- the package architecture
- the current Python API
