# Setting Up Config

Learn how to configure your ORCA Hand system settings. The primary configuration for the ORCA Hand is managed through the `config.yaml` file located in the model-specific directory (e.g., `orca_core/models/orcahand_v1_right/config.yaml`).

This file defines parameters crucial for the hand's operation, including communication settings, motor and joint mappings, movement ranges, and calibration procedures.

!!! Warning
    Be careful when editing this file, incorrect settings can cause unexpected behavior and make debugging more difficult.

---

## `config.yaml` Structure Overview

### 1. General Settings

```yaml
version: 0.2.0
baudrate: 3000000
port: /dev/ttyUSB0
max_current: 400
type: right
control_mode: current_based_position
```

**What should be changed?**

You should change the `port` to match your system (Linux or macOS). Change `type` to righ or left depending on the hand assembly you have. `max_current` is set to value found to be sufficient you can adjust it depending on the needs of your tasks. The `baudrate` or `control_mode` should not be changed based on the current implemention in the repo. If you decide to change them you have to adapt the code accordingly. 

---

### 2. Motor and Joint Identifiers

```yaml
motor_ids: [1, 2, 3, ..., 17]
joint_ids: [thumb_mcp, thumb_abd, ..., wrist]
```

**What should be changed?**

If you Id'ed the servos as per our recomendation you should change nothing here.
to modify this section. 

---

### 3. Joint to Motor Mapping

```yaml
joint_to_motor_map:
  thumb_mcp: 4
  thumb_abd: 3
  ...
  wrist: 17
```

**What should be changed?**

If you assembled the hand per our recomendation (placing the servo motors to the top tower as shown in the guide) you should not need to change anything. Otherwise you should move each joint and find the corresponding servo Id. This mapping is different for left and right assemblies. 

---

### 4. Joint Inversion

```yaml
joint_inversion: ["thumb_mcp", "thumb_abd", ..., "wrist"]
```

**What should be changed?**

Again if the hand was assembled as shown in the guide nothing should be changed. The values in the list are different for left and right hand assemblies.

---

### 5. Joint Range of Motion (ROM)

```yaml
joint_roms:
  thumb_mcp: [-50, 50]
  ...
  wrist: [-50, 30]
```

**What should be changed?**

Unless you have modified the design, you shoud not change the values.

---

### 6. Neutral Position

```yaml
neutral_position:
  thumb_mcp: -13
  ...
  wrist: 0
```

**What should be changed?**

You can adjust this section if you want the hand to return to a different default pose.

---

### 7. Calibration Parameters

```yaml
calib_current: 350
calib_step_size: 0.1
calib_step_period: 0.001
calib_num_stable: 10
calib_threshold: 0.01
```

**What should be changed?**

These parameters should generally not be changed unless you have experience tuning the calibration behavior for a specific hardware modification.

---

### 8. Calibration Sequence

```yaml
calib_sequence:
    - step: 1
        joints:
        thumb_mcp: flex
    - step: 2
        joints:
        thumb_mcp: extend
    - step: 3
        joints:
        thumb_abd: flex
        ...
```

**What should be changed?**

If you want to specify the sequence or calibrate only specific joints you can adapt the sequence. If you are unsure, leave this section as is. An incomplete or incorrect sequence may will lead to erros when executing commands later.

---