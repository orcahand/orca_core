# Claude Code Instructions

## Project: ORCA Core

Control package for the ORCA Hand - a dexterous open-source robotic hand. Provides hardware abstraction, calibration tools, and high-level joint-space control.

## Project Structure

```
orca_core/
├── api/              # High-level control API
├── hardware/         # Hardware interfaces (motors)
│   ├── dynamixel_client.py      # Dynamixel motor control
├── utils/            # Shared utilities
├── configs/          # Hand configurations (YAML)
└── core.py           # Main OrcaHand API

scripts/              # CLI tools for calibration, demos
tests/                # Unit tests
docs/                 # Documentation
```

### Key Files

- [orca_core/core.py](orca_core/core.py) - Main `OrcaHand` class API
- [orca_core/hardware/dynamixel_client.py](orca_core/hardware/dynamixel_client.py) - Motor control interface
- [orca_core/configs/*/config.yaml](orca_core/configs/) - Hand configuration files
- [scripts/](scripts/) - Calibration and utility scripts

### Hardware Components

| Component | Interface | Description |
|-----------|-----------|-------------|
| Dynamixel Motors | `dynamixel_client.py` | Servo motor control (17 motors) |
| Hand Configuration | `config.yaml` | Joint mapping, ROMs, calibration |

---

## Workflow

### Git & PRs

- Work on feature branches: `feature/description` or `fix/description`
- Never push directly to `main`
- Create PRs for all changes

### Commit Guidelines

- Use conventional commits: `Add feature`, `Fix bug`, `Update docs`
- Keep messages concise and descriptive
- Do NOT add `Co-Authored-By` lines

---

## Code Style

**Keep code clean and readable:**

- Write concise, self-documenting code
- Avoid excessive comments - prefer clear variable/function names
- Only add comments for complex logic or non-obvious design decisions
- Remove commented-out code before committing
- Follow existing patterns in the codebase

---

## Development

### Virtual Environment

**IMPORTANT:** Always activate the virtual environment before running any Python commands or installing packages:

```bash
source venv/bin/activate
```

Never install packages to the base/system Python environment. All dependencies should be installed within the venv.

### Setup

```bash
python -m venv venv
source venv/bin/activate
pip install -e .
```

### Testing

```bash
source venv/bin/activate
pytest tests/
```

### Common Scripts

```bash
# Calibration workflow
python scripts/tension.py orca_core/configs/orcahand_v1_right
python scripts/calibrate.py orca_core/configs/orcahand_v1_right
python scripts/neutral.py orca_core/configs/orcahand_v1_right

# Manual control
python scripts/slider_joint.py orca_core/configs/orcahand_v1_right
python scripts/slider_motor.py orca_core/configs/orcahand_v1_right
```

### Configuration

All hand-specific settings are in `config.yaml`:
- Motor-to-joint mapping
- Joint ROMs (ranges of motion)
- Neutral positions
- Calibration sequences

---

## Technical Notes

### Control Modes

- `current_based_position` (recommended) - Position control with current feedback
- `position` - Direct position control
- `current` - Direct current control
- `velocity` - Velocity control

### Joint Naming Convention

Format: `{finger}_{joint_type}`

Fingers: `thumb`, `index`, `middle`, `ring`, `pinky`, `wrist`
Joint types: `mcp`, `pip`, `dip`, `abd` (abduction)

Example: `index_mcp`, `thumb_pip`

---

## Architecture

See [docs/architecture-diagram.md](docs/architecture-diagram.md) for visual diagrams of:
- System overview and component interactions
- Data flow and sequence diagrams
- Directory structure
- Calibration workflow
