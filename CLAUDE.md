# Claude Code Instructions

## Project: ORCA Core

Control package for the ORCA Hand - a dexterous open-source robotic hand. Provides hardware abstraction, calibration tools, and high-level joint-space control.

## Project Structure

```
orca_core/
├── api/                          # FastAPI app exposing OrcaHand over HTTP (early/incomplete)
├── maintenance/                  # Interaction-free hardware routines (callback-driven)
│   ├── motor_chain.py               # Assign motor IDs/baud; progress + prompt callbacks
│   ├── calibration_routine.py       # run_calibration(): hardstop-drive + encoder-anchor pass
│   └── tensioning.py                # run_tension()/run_jitter(): tendon tensioning + seating
├── data/                         # Packaged content (demo_poses.yaml)
├── control/                      # Closed-loop joint control
│   ├── joint_loop.py                # JointLoopThread: PI loop + encoder-freshness watchdog
│   ├── joint_controller.py          # Per-joint PI controller
│   └── constants.py                 # Loop rate, watchdog tiers, PI defaults
├── hardware/                      # Hardware interfaces
│   ├── motor_client.py               # MotorClient base interface
│   ├── motor_factory.py              # motor_type → MotorClient class (lazy per-family imports)
│   ├── motor_resolution.py           # Connect-time driver probing + yaml persistence
│   ├── dynamixel_client.py           # Dynamixel motor control (bus-locked, thread-safe)
│   ├── feetech_client.py             # Feetech motor control
│   ├── feetech/                      # Vendored Feetech servo SDK (used by feetech_client)
│   ├── mock_dynamixel_client.py      # In-memory motor client for tests/dev
│   ├── hand_serial_link.py           # Framed serial link to the connector board (encoders/tactile)
│   ├── mock_hand_serial_link.py      # In-memory stand-in for HandSerialLink
│   ├── joint_encoder_client.py       # Joint-angle encoder stream client + anchor sampling
│   ├── tactile_client.py             # Tactile sensor register client
│   └── sensing/                      # Wire protocol, framing, port auto-discovery, sensor health checks
├── kinematics/                    # Rigid transforms, frames, forward kinematics
├── utils/                         # Shared utilities
│   ├── utils.py                      # Model-path resolution, YAML I/O (atomic writes), interpolation
│   └── cli.py                        # Shared argparse/lifecycle helpers for scripts and examples
├── models/                        # Hand configurations (YAML), versioned v1/ and v2/
├── base_hand.py                   # BaseHand: shared joint-space interface (ABC)
├── hardware_hand.py                # OrcaHand: the motor-only hand (+ MockOrcaHand)
├── hardware_hand_sensing.py        # Sensing variants: OrcaHandTouch / OrcaHandJointFeedback / OrcaHandFull (+ mocks)
├── hand_factory.py                 # load_hand()/detect_hand(): autodetect the hand, pick class + model
├── hand_config.py                  # Config dataclasses (BaseHandConfig, OrcaHandConfig, ...)
├── calibration.py                  # Calibration result types + YAML (de)serialization
├── joint_position.py               # OrcaJointPositions: typed joint-position container
├── demo_poses.py                   # load_demo_poses(): packaged demo pose data
├── version.py                      # LATEST_VERSION: default hand model version
└── constants.py                    # Control modes, protocol constants

scripts/              # Thin CLI front-ends. argparse + print + input(); no logic.
examples/             # Demo and record/replay scripts built on the public API
tools/                # Maintainer-only tools (regenerate packaged data)
tests/                # Unit tests
docs/                 # MkDocs site sources (docs/pages/) - partially stale, verify against code
```

Only `orca_core/` ships in the wheel. Anything both a script and an external
front-end (e.g. orca_ui) needs must live in the package: operations report
progress via `progress_callback` and request human action via `prompt_callback`,
so a terminal and a GUI drive the identical routine.

### Key Files

- [orca_core/hardware_hand.py](orca_core/hardware_hand.py) - `OrcaHand`: the main motor-only hand API
- [orca_core/hardware_hand_sensing.py](orca_core/hardware_hand_sensing.py) - `OrcaHandTouch` / `OrcaHandJointFeedback` / `OrcaHandFull`: the sensing-equipped variants
- [orca_core/hand_factory.py](orca_core/hand_factory.py) - `load_hand()`, the recommended entry point (picks the right class from `config.yaml`)
- [orca_core/base_hand.py](orca_core/base_hand.py) - Shared joint-space interface all hand backends implement
- [orca_core/hardware/dynamixel_client.py](orca_core/hardware/dynamixel_client.py) - Dynamixel motor control interface
- [orca_core/hardware/sensing/serial_discovery.py](orca_core/hardware/sensing/serial_discovery.py) - Auto-discovery of encoder/tactile serial ports
- [orca_core/models/*/config.yaml](orca_core/models/) - Hand configuration files
- [scripts/](scripts/) - Calibration, tensioning, and diagnostic scripts

### Hardware Components

| Component | Interface | Description |
|-----------|-----------|--------------|
| Dynamixel Motors | `dynamixel_client.py` | Servo motor control (typ. 17 motors), bus access is lock-serialized |
| Feetech Motors | `feetech_client.py` | Alternate servo motor backend |
| Joint Encoders | `joint_encoder_client.py` via `hand_serial_link.py` | Absolute joint-angle feedback, closed-loop control input |
| Tactile Sensors | `tactile_client.py` via `hand_serial_link.py` | Per-fingertip tactile sensing (Paxini) |
| Connector board link | `hand_serial_link.py`, `hardware/sensing/` | Framed serial protocol multiplexing encoder + tactile streams |
| Hand Configuration | `config.yaml` | Joint mapping, ROMs, calibration, feedback/sensor enablement |

---

## Workflow

### Git & PRs

- Work on feature branches: `feature/description` or `fix/description`
- Never push directly to `main`
- Create PRs for all changes

### Commit Guidelines

- Use conventional commits: `Add feature`, `Fix bug`, `Update docs`
- Keep messages concise and descriptive

---

## Code Style

**Keep code clean and readable:**

- Write concise, self-documenting code
- Avoid excessive comments - prefer clear variable/function names
- Only add comments for complex logic or non-obvious design decisions
- Remove commented-out code before committing
- Follow existing patterns in the codebase

**Comment length and content:**

- Max 1-2 lines per comment. Longer is only OK for headstrings on files, major classes, or major functions.
- Comments describe the current state of the code, never its history. Don't reference previous versions, past bugs, prior approaches that didn't work, commits, or plans/tasks - if it's not true of the code as it stands, it doesn't belong in a comment.
- Don't require hardware-specific insider knowledge to understand a comment. Refer to hardware generically rather than by internal codename.

---

## Development

### Virtual Environment

**IMPORTANT:** Use `uv` for Python commands and dependency management in this repo.

```bash
uv sync --group dev
```

This creates a local `.venv`. If you prefer an activated shell, use:

```bash
source .venv/bin/activate
```

Never install packages to the base/system Python environment.

### Setup

```bash
uv sync --group dev
```

### Testing

```bash
uv run pytest tests/
```

### Downstream consumers

`orca_core` is published on PyPI and imported by sibling repos checked out beside it
(`orca_ui`, `orca_teleop`, `orca_firmware`, `orca_stress_tests`, `orca_ros`). Their code is
invisible to this test suite.

**Never conclude a public symbol is unused from this repo alone.** A green suite here — and
even a green downstream suite — does not prove it. Check first:

```bash
uv run python tools/check_downstream.py --symbol MockOrcaHand   # who references it?
uv run python tools/check_downstream.py                          # do their imports resolve?
uv run python tools/check_downstream.py --run-tests              # do their suites still pass?
```

Missing sibling checkouts are skipped and reported, so this works with only `orca_core` present.

The public API surface is pinned in `tests/test_public_api_surface.py`. Changing it is a
breaking change: update the sets deliberately and bump the minor version in `pyproject.toml`.
Note that `release.yml` publishes to PyPI on every push to `main` carrying a new version.

### Common Scripts

Every script takes an optional positional `config_path` pointing at a model's `config.yaml` (defaults to the package's bundled model if omitted). Run `--help` on any script for its full flag list.

```bash
# Calibration workflow
uv run python scripts/tension.py orca_core/models/v2/orcahand-right/config.yaml
uv run python scripts/calibrate.py orca_core/models/v2/orcahand-right/config.yaml
uv run python scripts/neutral.py orca_core/models/v2/orcahand-right/config.yaml

# Manual control (sliders; --motor-space for per-motor tendon bring-up)
uv run python scripts/manual_control.py orca_core/models/v2/orcahand-right/config.yaml

# Sensor health check (config-driven; --port for board bring-up without a config)
uv run python scripts/check_sensors.py orca_core/models/v2/orcahand-touch-right/config.yaml
uv run python scripts/check_sensors.py --port /dev/cu.usbmodemXXXX

# Live sensor data view (autodetects the port; --port to override, no config_path)
uv run python scripts/monitor_sensors.py
```

### Configuration

All hand-specific settings are in `config.yaml`:
- Motor-to-joint mapping, joint ROMs (ranges of motion), neutral positions, calibration sequences
- `use_joint_feedback` + `joint_encoder_joints` + `encoder_serial_port` - enable the closed-loop joint-encoder controller (`load_hand` then returns `OrcaHandJointFeedback`/`OrcaHandFull`)
- `sensors:` block - enable tactile sensing (`load_hand` then returns `OrcaHandTouch`/`OrcaHandFull`)

---

## Technical Notes

### Control Modes

- `current_based_position` (recommended) - Position control with current feedback
- `position` - Direct position control
- `multi_turn_position` - Position control without a travel limit (used for the wrist)
- `current` - Direct current control
- `velocity` - Velocity control

### Joint Naming Convention

Format: `{finger}_{joint_type}`, except `wrist` which has no joint-type suffix.

Fingers: `thumb`, `index`, `middle`, `ring`, `pinky`
Joint types: `cmc`, `mcp`, `pip`, `dip`, `abd` (abduction)

Example: `index_mcp`, `thumb_cmc`
