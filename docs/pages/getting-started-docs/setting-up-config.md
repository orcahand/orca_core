# Setting Up `config.yaml`

`config.yaml` is the static description of a specific ORCA Hand build. The refactored loader turns it into an immutable `OrcaHandConfig` object, validates it, and uses it to drive every later stage of the workflow.

The most important rule is simple: make the config describe the hardware you actually assembled.

## Where the file lives

Each model directory contains a `config.yaml`, for example:

- `orca_core/models/v2/orcahand_right/config.yaml`
- `orca_core/models/v1/orcahand_left/config.yaml`

The `v2` model is the clearest example of the current canonical schema. Some older `v1` configs in the repository predate the calibration key rename and may need a small migration.

When you construct `OrcaHand`, pass the path to the file itself:

```python
from orca_core import OrcaHand

hand = OrcaHand(config_path="orca_core/models/v2/orcahand_right/config.yaml")
```

## Current schema

The refactored code expects the following top-level fields:

```yaml
version: 0.2.1
type: right
port: /dev/ttyUSB0
baudrate: 1000000
motor_type: dynamixel
max_current: 300
control_mode: current_based_position

motor_ids: [1, 2, 3, 4]
joint_ids: [wrist, thumb_cmc, thumb_abd, thumb_mcp]

joint_to_motor_map:
  wrist: -1
  thumb_cmc: 17
  thumb_abd: 14
  thumb_mcp: 15

joint_roms:
  wrist: [-65, 35]
  thumb_cmc: [-45, 33]

neutral_position:
  wrist: -8
  thumb_cmc: 0

calibration_current: 300
wrist_calibration_current: 100
calibration_step_size: 0.15
calibration_step_period: 0.0001
calibration_threshold: 0.01
calibration_num_stable: 10
calibration_sequence:
  - step: 1
    joints:
      thumb_cmc: flex
```

## What each section means

### Identity and communication

- `type`: left or right hand assembly
- `port`: serial device path
- `baudrate`: bus baudrate
- `motor_type`: currently selects the concrete motor backend

These fields determine how `OrcaHand.connect()` talks to the motor bus.

### Motor and joint layout

- `motor_ids`: the motor IDs expected on the bus
- `joint_ids`: the logical joint order for the hand model
- `joint_to_motor_map`: which motor drives which joint

`joint_to_motor_map` uses a signed motor ID convention:

- absolute value = physical motor ID
- negative sign = joint is inverted relative to the positive flexion convention

Internally, the loader converts this into:

- a normalized joint-to-motor map with absolute motor IDs
- a separate `joint_inversion_dict`

### Joint range and neutral pose

- `joint_roms`: minimum and maximum joint values for each joint
- `neutral_position`: the default pose used by `set_neutral_position()` and `init_joints()`

The bundled configs use degrees for these values, and joint-space commands should use the same units.

### Calibration settings

These fields govern the automatic calibration routine:

- `calibration_current`
- `wrist_calibration_current`
- `calibration_step_size`
- `calibration_step_period`
- `calibration_threshold`
- `calibration_num_stable`
- `calibration_sequence`

`calibration_sequence` is a list of steps. Each step names one or more joints and a direction (`flex` or `extend`).

## Migration from older config keys

Older configs and older docs often used `calib_*` names. The refactored code now expects canonical names.

Use this mapping when migrating an older config:

| Old key | New key |
| --- | --- |
| `calib_current` | `calibration_current` |
| `calib_step_size` | `calibration_step_size` |
| `calib_step_period` | `calibration_step_period` |
| `calib_threshold` | `calibration_threshold` |
| `calib_num_stable` | `calibration_num_stable` |
| `calib_sequence` | `calibration_sequence` |

## What you should usually edit

In a normal bring-up, the most common edits are:

- `port`
- `motor_type`
- `joint_to_motor_map`
- sometimes `neutral_position`

You should only edit calibration tuning parameters if you know why the defaults are not working for your build.

## Validation rules worth knowing

The loader validates that:

- every configured joint has a ROM
- every ROM entry corresponds to a known joint
- the number of joints, motor IDs, and mappings line up
- the configured control mode is supported
- calibration sequence entries refer to valid joints and directions

This means config mistakes are usually caught early, before motion commands run.
