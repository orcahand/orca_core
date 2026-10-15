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
max_current: default
type: right
control_mode: current_based_position
```

**What should be changed?**

Change `type` to right or left depending on the hand assembly. `max_current: default` uses the motor family's own limit (300 mA on Dynamixel hands, 900 mA on Feetech hands); put a number there to pin a different limit for this hand. `control_mode` should generally stay at `current_based_position`.

#### Driver settings: `port`, `baudrate`, `motor_type`

The bundled configs leave all three on auto-detection: `port: auto`, and no `motor_type` or `baudrate` at all. They are resolved fresh on every connect and never written back to the file:

- `port: auto` finds the serial adapter by USB VID, falling back to "the only adapter present" or an interactive picker.
- `motor_type` is identified by pinging each motor family (Dynamixel vs Feetech) on the bus.
- `baudrate` is probed from the family's known set (1M / 3M for Dynamixel, 1M for Feetech).

Write a value into the file to override detection for that field:

```yaml
port: /dev/cu.usbmodemXXXX  # when multiple adapters are connected (COM3 on Windows)
baudrate: 1000000           # when motors are configured for a non-default rate
motor_type: feetech         # when probing might misidentify the bus
```

A value you write wins over what was detected, and the log says so whenever the two disagree. You won't normally need one — the script will tell you on the command line if a probe failed and asking for an override would help. Remove the line (or set `port: auto`) to go back to detection.

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

#### Hardware v2.1: a second map for the same model

A later hardware revision can route a tendon the other way without changing which motor drives which joint. Rather than fork the model, the config carries a second map for that revision:

```yaml
joint_to_motor_map:        # v2.0 boards
  thumb_mcp: 12
  thumb_dip: 11
  ...
joint_to_motor_map_v21:    # v2.1 boards -- same motors, only the sign may differ
  thumb_mcp: -12
  thumb_dip: -11
  ...
```

Which map is in force is decided once, when the config is built, from the hand's **hardware version** (200 for v2.0.0, 210 for v2.1.0, 211 for v2.1.1 -- one byte, `major*100 + minor*10 + patch`; boards provisioned earlier report 2 or 21, read as 200 and 210. A v2.1.1 hand is a v2.1 hand that also takes the ROMs listed in `joint_roms_v211`):

```yaml
hardware_version: auto     # take what the OH board reports (the packaged default)
hardware_version: 21       # pin it: a hand on a plain adapter has no board to ask
```

Precedence is the same as for `port` and `motor_type`: a value you write **wins** over what the board reports, and the clash is logged (`config.yaml pins hardware_version=21, so the detected 2 is not used`) so a stale copied config is never mistaken for a detection bug. `auto` defers to the board; with no board and no pin the base map runs.

`load_hand()` reads the board through detection. A caller that loads by `config_path` skips detection and passes the board's value as `detected_hardware_version=` -- it ranks as detection, so a pin still wins. Everything downstream then reads `joint_to_motor_map` and `joint_inversion` as usual.

Whichever map is chosen goes through the same checks as the base map (every joint defined, every motor in `motor_ids`). The two maps are hand-maintained copies, so keep them in step by hand. Right-hand v2 models carry no `_v21` map: the wiring change was a left-hand change. The version still matters on both sides, because it also decides whether the motor-travel features below apply.

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
calibration_current: default
calibration_step_size: 0.1
calibration_step_period: 0.001
calibration_num_stable: 10
calibration_threshold: 0.01
```

**What should be changed?**

`calibration_current: default` drives the joints to their hardstops at the motor family's own calibration current (300 mA on Dynamixel hands, 900 mA on Feetech hands). A number pins a different value; `wrist_calibration_current` follows it unless set.

On v2.1 hardware, `calibration_current_v21` sets a current per joint group for a motor family, replacing `calibration_current` for those joints: `abd` (the abductions and `thumb_cmc`), `mcp`, and `pip` (the PIPs and `thumb_dip`). The packaged models calibrate a v2.1 Dynamixel hand's abductions at 300 mA and its MCPs and PIPs at 500 mA; a family that is not listed, and every v2.0 hand, keeps `calibration_current`. Re-drives step up from the joint's own group current.

`guided_tension_currents` sets guided tensioning's currents per motor family, used by both `scripts/guided_tension.py` and the console: `hold` is the current each joint group (`abd`, `mcp`, `pip`) yields at while you move it through its range, and `tension` is the hold while you ratchet the spools. A joint named under `hold` keeps its own current over its group's and over `--hold-current`; the touch hands hold `thumb_mcp` at 100 mA. A family a config leaves out uses the package defaults (Dynamixel 200/140/100 mA, tension 400 mA; Feetech 380/320/280 mA, tension 580 mA).

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
joint_motor_travel_v21:   # packaged baseline, shared by every v2.1 hand
  index_mcp: 139.39
  index_pip: 131.04
  ...
# joint_motor_travel:     # this hand's own measurement, wins per joint
calibration_travel_margin: 0.25
calibration_travel_retries: 3
# calibration_max_current: <mA>   # optional headroom for the re-drive
```

**The baseline only applies to v2.1 hardware.** On a v2.0 hand, or one whose
version is unknown (no board and no `hardware_version` pin), there is no
baseline: calibration is a single plain pass with no travel check or re-drive,
and guided tensioning refuses to run (hold the motors and ratchet the spools
instead). The packaged models ship the baseline as `joint_motor_travel_v21`;
a `joint_motor_travel` block measured on your own hand takes precedence joint
by joint, but likewise only on v2.1.

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
(a fraction, default `0.25`) below the baseline, that joint alone is re-driven,
at a higher current when `calibration_max_current` grants headroom, then the
nominal calibration current is restored for the rest of the run. Travel *above* the baseline is reported but never re-driven —
more current cannot shorten a span; that points at a slipped tendon or a stale
baseline.

- `calibration_travel_margin` — accepted fractional deviation. Default `0.25`.
- `calibration_max_current` — ceiling (mA) the re-drive may use. Omit it and
  every re-drive repeats the sweep at `calibration_current`, which still helps
  a joint that only needed a second pass but gives no extra torque. Set it
  above `calibration_current` to let the re-drive escalate. Never below the
  motor family's protection limit is enforced by the client.
- `calibration_retry_current` — current (mA) the last re-drive aims for. Must
  exceed `calibration_current`. Omit it and it defaults to 1.5x
  `calibration_current`. Only reached when `calibration_max_current` allows it.
- `calibration_travel_retries` — how many re-drives to attempt. Default `3`.
  With more than one, the current ramps evenly from `calibration_current` up to
  the target, capped by `calibration_max_current`, so a joint that only needs
  a nudge is not driven at the full retry current on the first try. Set `0` to
  report the shortfall and never re-drive.

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

A joint with no baseline has its travel measured and logged,
but not checked, and is never re-driven.

---