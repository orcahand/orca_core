# Setting Up Config

Learn how to configure your ORCA Hand system settings. The primary configuration for the ORCA Hand is managed through the `config.yaml` file located in the model-specific directory (e.g., `orca_core/models/v2/orcahand-right/config.yaml`).

This file defines parameters crucial for the hand's operation, including communication settings, motor and joint mappings, movement ranges, and calibration procedures.

!!! Warning
    Be careful when editing this file, incorrect settings can cause unexpected behavior and make debugging more difficult.

---

## `config.yaml` Structure Overview

### 1. General Settings

```yaml
port: auto
baudrate: 1000000
max_current: 300
type: right
control_mode: current_based_position
```

**What should be changed?**

Change `type` to right or left depending on the hand assembly. `max_current` is set to a sane default; adjust if your tasks need more or less. `control_mode` should generally stay at `current_based_position`.

#### Driver settings: `port`, `baudrate`, `motor_type`

The bundled configs ship with `port: auto` and a `baudrate` pinned for the hand version (1M for v2, 3M for v1). Anything not pinned is auto-detected at connect time and persisted back to `config.yaml`:

- `port: auto` finds the serial adapter by USB VID, falling back to "the only adapter present" or an interactive picker.
- A missing `motor_type` is identified by pinging each motor family (Dynamixel vs Feetech) on the bus.
- A missing `baudrate` is probed from the family's known set (1M / 3M for Dynamixel, 1M for Feetech).

Override any of them explicitly if you need to:

```yaml
port: /dev/cu.usbmodemXXXX  # when multiple adapters are connected
baudrate: 1000000           # when motors are configured for a non-default rate
motor_type: feetech         # when probing might misidentify the bus
```

You won't normally need to — the script will tell you on the command line if a probe failed and asking for an override would help.

---

### 2. Motor and Joint Id's

```yaml
motor_ids: [1, 2, 3, ..., 17]
joint_ids: [thumb_mcp, thumb_abd, ..., wrist]
```

**What should be changed?**

If you ID'ed the servos as per our recommendation you should change nothing here.

---

### 3. Joint to Motor Mapping and Inversion

```yaml
joint_to_motor_map:
  thumb_mcp: -4
  thumb_abd: -3
  ...
  wrist: 17
```

**What should be changed?**

This section defines which motor (by ID) controls each joint and how the sign of the motor ID affects its rotation direction.

* The number is the **motor ID**.
* The **sign** is determined by direction of rotation when flexing a joint.

#### Hand-Specific Guidelines

Flexion decides the sign of an servo ID. 

  * If **flexion** moves the servo **CCW** then a **no sign** should be included to the motor ID. 

  * If **flexion** moves the servo **CW** then a **negative sign** should be included to the motor ID. 

**Right hand assembly:** Most joints flexions lead to **CW** rotation so mostly negative signs are found.

**Left hand assembly:** Most joints flexions lead to **CCW** rotation so mostly none/positive signs are found.

If tendon routing is done properly, all joints should have the same sign, except for `index_abd`, `middle_abd` and `pinky_abd` joints. 

> **Note:** For abduction joints, flexion refers to movement **away from the thumb** and for the thumb flexion is **towards the fingers**. 

---


For example:

```yaml
joint_to_motor_map:
  thumb_mcp: -4   # thumb_mcp flexion moves the motor with ID 4 CW
  index_abd: 14   # index_abd flexion moves the motor with ID 14 CCW
```

---

### 4. Joint Range of Motion (ROM)

```yaml
joint_roms:
  thumb_mcp: [-50, 50]
  ...
  wrist: [-50, 30]
```

**What should be changed?**

Unless you have modified the design, you should not change the values.

---

### 5. Neutral Position

```yaml
neutral_position:
  thumb_mcp: -13
  ...
  wrist: 0
```

**What should be changed?**

You can adjust this section if you want the hand to return to a different default pose.

---

### 6. Calibration Parameters

```yaml
calibration_current: 350
calibration_step_size: 0.1
calibration_step_period: 0.001
calibration_num_stable: 10
calibration_threshold: 0.01
```

**What should be changed?**

These parameters should generally not be changed unless you have experience tuning the calibration behavior for a specific hardware modification.

---

### 7. Calibration Sequence

```yaml
calibration_sequence:
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

If you want to specify the sequence or calibrate only specific joints you can adapt the sequence. If you are unsure, leave this section as is. An incomplete or incorrect sequence may lead to errors when executing commands later.

---

### 8. Motor Travel Baseline

```yaml
joint_motor_travel:
  index_mcp: 122.96
  index_pip: 102.13
  ...
calibration_travel_margin: 0.25
calibration_retry_current: 450
calibration_travel_retries: 1
```

`joint_motor_travel` records how far each joint's motor turns between that
joint's two hardstops, in degrees. It is keyed by **joint name**, not motor ID:
the travel is set by the joint's tendon spool diameter, so it belongs to the
joint, and calibration looks up whichever motor `joint_to_motor_map` assigns to
it. A hand whose motor IDs were assigned in a different order still reads the
right value.

Calibration uses it as a sanity check. After a joint has been driven onto both
of its hardstops, the routine compares the travel it just measured against this
baseline. An over-tensioned tendon makes the drive stall *before* the hardstop,
so the two limits come out too close together and every angle derived from them
is wrong. When the measurement falls more than `calibration_travel_margin`
(a fraction, default `0.25`) below the baseline, that joint alone is re-driven
at a higher current, then the nominal calibration current is restored for the
rest of the run. Travel *above* the baseline is reported but never re-driven —
more current cannot shorten a span; that points at a slipped tendon or a stale
baseline.

- `calibration_travel_margin` — accepted fractional deviation. Default `0.25`.
- `calibration_retry_current` — current (mA) the last re-drive uses. Must exceed
  `calibration_current`. Omit it and it defaults to 1.5x `calibration_current`.
- `calibration_travel_retries` — how many re-drives to attempt. Default `1`.
  With more than one, the current ramps evenly from `calibration_current` up to
  `calibration_retry_current`, so a joint that only needs a nudge is not driven
  at the full retry current on the first try. Set `0` to report the shortfall
  and never re-drive.

**What should be changed?**

Generate `joint_motor_travel` rather than writing it by hand, on a freshly
tensioned hand:

```bash
python scripts/measure_travel.py /path/to/orcahand-right/config.yaml
```

That drives every joint onto both hardstops, prints the measured travel per
joint, and writes the block. Add `--dry-run` to review the numbers first, or
`--from-calibration` to derive them from an existing clean `calibration.yaml`
without moving the hand. Compare joints that share a design — the four `_pip`
joints, the four `_mcp` joints — as a cross-check: one that reads well below its
siblings is usually over-tensioned rather than genuinely shorter.

Each calibration run also writes what it actually measured to
`calibration.yaml` under `motor_travel_measured:`, so successive runs can be
compared.

A joint omitted from `joint_motor_travel` has its travel measured and logged,
but not checked, and is never re-driven.

---