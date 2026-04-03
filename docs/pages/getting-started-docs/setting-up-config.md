# Setting Up Config

Learn how to configure your ORCA Hand system settings. In the refactored codebase, the primary configuration for the hand is still managed through `config.yaml`, but the file is now loaded into a validated `OrcaHandConfig` object.

This file defines the static metadata needed to operate the hand: communication settings, motor and joint mappings, movement ranges, neutral pose, and the calibration procedure.

!!! Warning
    Be careful when editing this file. Incorrect settings can cause unexpected behavior and make debugging more difficult.

---

## `config.yaml` Structure Overview

### 1. General Settings

```yaml
version: 0.2.1
baudrate: 1000000
port: /dev/ttyUSB0
max_current: 300
type: right
control_mode: current_based_position
motor_type: dynamixel
```

**What should be changed?**

You should update `port` to match your system. `type` should match your physical hand. `motor_type` and `baudrate` should match the motor chain you actually assembled. `max_current` can be tuned for your task, but the default is usually a good starting point.

---

### 2. Motor and Joint IDs

```yaml
motor_ids: [1, 2, 3, ..., 17]
joint_ids: [wrist, thumb_cmc, thumb_abd, ..., pinky_pip]
```

**What should be changed?**

If you assigned IDs according to the intended assembly, this section should mostly already be correct. The key point is that the ordered `joint_ids` list is now also used to register the default ordering for `OrcaJointPositions`.

---

### 3. Joint to Motor Mapping and Inversion

```yaml
joint_to_motor_map:
  thumb_cmc: 17
  ring_mcp: -7
  wrist: -1
```

**What should be changed?**

This section defines which motor controls each joint and whether the joint direction is inverted.

- The **absolute value** is the motor ID.
- The **sign** indicates whether the joint is inverted relative to the positive flexion convention.

In the refactored loader, this mapping is normalized into:

- a joint-to-motor map with absolute motor IDs
- a separate `joint_inversion_dict`

So the sign still matters, but it is treated as explicit configuration metadata.

#### Hand-Specific Guidelines

The sign should follow the flexion direction of the joint:

- If flexion corresponds to the positive direction for your build, keep the motor ID positive.
- If flexion is inverted relative to that convention, use a negative motor ID.

For abduction joints and the thumb, use the same sign logic, but make sure your chosen positive direction is consistent with the actual hand motion you want the API to expose.

---

### 4. Joint Range of Motion (ROM)

```yaml
joint_roms:
  wrist: [-65, 35]
  thumb_cmc: [-45, 33]
  thumb_mcp: [-60, 90]
```

**What should be changed?**

Only change these values if your physical design or tendon routing differs from the bundled model. The refactored `BaseHand` clamps joint commands against these bounds automatically.

---

### 5. Neutral Position

```yaml
neutral_position:
  wrist: -8
  thumb_cmc: 0
  thumb_mcp: 33
```

**What should be changed?**

Adjust this section if you want a different default pose for `set_neutral_position()` and `init_joints()`.

---

### 6. Calibration Parameters

```yaml
calibration_current: 300
wrist_calibration_current: 100
calibration_step_size: 0.15
calibration_step_period: 0.0001
calibration_num_stable: 10
calibration_threshold: 0.01
```

**What should be changed?**

These parameters should usually be left alone unless you are tuning the calibration routine for a modified hardware setup.

The important change relative to older docs is that the canonical keys are now `calibration_*`, not `calib_*`.

---

### 7. Calibration Sequence

```yaml
calibration_sequence:
  - step: 1
    joints:
      thumb_cmc: flex
  - step: 2
    joints:
      thumb_cmc: extend
  - step: 3
    joints:
      wrist: flex
```

**What should be changed?**

If you need to calibrate joints in a different order, or group joints differently, you can adjust the sequence. If you are unsure, keep the bundled order.

An incomplete or inconsistent sequence can leave `calibration.yaml` only partially populated, which in turn affects motor-to-joint conversion later in the runtime.

---

## Additional Notes for the Refactored Codebase

- `OrcaHand` expects `config_path` to point to the `config.yaml` file itself.
- `calibration.yaml` lives next to `config.yaml` by default unless you override `calibration_path`.
- `BaseHandConfig` validates the joint-space portion of the file.
- `OrcaHandConfig` adds motor, control-mode, and calibration validation on top.
