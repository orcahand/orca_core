# Tensioning And Calibration

This page documents the current maintenance workflow used by the refactored package.

## Two files, two roles

The bring-up flow depends on two YAML files:

- `config.yaml` stores the static hand description and calibration procedure.
- `calibration.yaml` stores the results of running that procedure on a specific physical hand.

`OrcaHand` loads both:

- `OrcaHandConfig` from `config.yaml`
- `CalibrationResult` from `calibration.yaml`

## What calibration does

`hand.calibrate()` walks through the configured `calibration_sequence`, drives joints toward their mechanical limits, and writes the resulting values back to `calibration.yaml`.

The stored result includes:

- per-motor lower and upper limits
- per-motor joint-to-motor ratios
- a `calibrated` flag
- a `wrist_calibrated` flag

Partial progress is written during calibration, so an interrupted run is not fully lost.

## What `init_joints()` does

For most application code, you should prefer:

```python
hand.init_joints()
```

instead of manually calling each bring-up step.

`init_joints()`:

1. enables torque
2. applies the configured control mode
3. applies the configured current limit
4. calibrates if the hand is not yet calibrated
5. computes wrap offsets
6. moves to neutral

## Recommended workflow after assembly

The easiest operator path is the guided script:

```sh
python scripts/setup.py orca_core/models/v2/orcahand_right/config.yaml
```

That workflow repeats tensioning and calibration with motion checks in between.

## Manual workflow

If you want to run the steps yourself:

1. Tension the hand:

```sh
python scripts/tension.py orca_core/models/v2/orcahand_right/config.yaml --move_motors
```

2. Run calibration:

```sh
python scripts/calibrate.py orca_core/models/v2/orcahand_right/config.yaml
```

3. Move to neutral:

```sh
python scripts/neutral.py orca_core/models/v2/orcahand_right/config.yaml
```

## Tensioning guidance

The goal is to remove slack without overtightening.

Typical procedure:

1. Use `tension.py --move_motors` to drive the spools into a useful starting state.
2. Let `tension.py` hold the motors in place.
3. Tighten the top spools gradually with the mechanical tool.
4. Stop as soon as the tendons feel firm and consistent.

Too much tension can hurt motion quality and calibration quality.

## When to recalibrate

Run calibration again when:

- the hand has been re-strung
- tendon slack has changed noticeably
- the wrist has been serviced
- joint tracking no longer matches expectation

## Wrist calibration behavior

The wrist is handled slightly differently:

- wrist calibration status is tracked separately
- `--force-wrist` can force wrist recalibration
- if wrist steps are missing from the sequence, the code can append them when needed

This lets you skip unnecessary wrist passes during routine finger recalibration while still supporting a full bring-up when required.
