# Scripts

The `scripts/` directory contains operator tools and debugging entrypoints built on top of the refactored package.

They are best understood as workflow helpers, not as the primary API surface. The primary API is the Python package itself.

## Recommended scripts

### Bring-up and maintenance

| Script | Purpose |
| --- | --- |
| `setup.py` | Guided setup workflow: tension, calibrate, neutral, motion test, repeat |
| `calibrate.py` | Run calibration and write `calibration.yaml` |
| `tension.py` | Hold spool tension during tendon maintenance |
| `neutral.py` | Move to the configured neutral pose |
| `zero.py` | Move every configured joint to zero |

### Recording and replay

| Script | Purpose |
| --- | --- |
| `record_angles.py` | Record waypoint-style poses |
| `replay_angles.py` | Replay waypoint-style motions |
| `record_continuous.py` | Record continuous joint trajectories |
| `replay_continuous.py` | Replay continuous trajectories |

### Debugging and inspection

| Script | Purpose |
| --- | --- |
| `slider_joint.py` | Joint-space debug UI |
| `slider_motor.py` | Motor-space debug UI |
| `check_motor.py` | Single-motor inspection tool |
| `test.py` | Manual hardware motion / monitoring script |
| `test_motor_latency.py` | Measure bus performance and optional Dynamixel optimizations |
| `jitter.py` | Apply a short oscillation to one motor for debugging |

### Demos and experiments

These scripts are useful examples but are less central than the workflow tools above:

- `main_demo.py`
- `main_demo_abduction.py`
- `overload_demo.py`
- `test_overload.py`
- `debug_overload.py`

## Typical usage patterns

### Full setup

```bash
python scripts/setup.py orca_core/models/v2/orcahand_right/config.yaml
```

### Calibration only

```bash
python scripts/calibrate.py orca_core/models/v2/orcahand_right/config.yaml
```

### Move to neutral

```bash
python scripts/neutral.py orca_core/models/v2/orcahand_right/config.yaml
```

### Hold tension during maintenance

```bash
python scripts/tension.py orca_core/models/v2/orcahand_right/config.yaml --move_motors
```

## Shared assumptions

Most scripts:

- accept a `config.yaml` path as their first positional argument
- construct an `OrcaHand`
- connect to the hardware
- perform one workflow
- disconnect on exit

Because of that, the scripts remain useful examples of how the package is intended to be used.
