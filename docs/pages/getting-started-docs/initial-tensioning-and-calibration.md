# Calibration and Tensioning

## Calibration

In the refactored codebase, calibration is no longer just an informal script step. It produces a persistent `CalibrationResult`, loaded from `calibration.yaml`, which is then used by `OrcaHand` to convert between motor space and joint space.

Calibration should still be performed regularly, especially:

- After extended use
- After slack has built up in the tendons
- After tensioning, to restore precision

The calibration sequence is defined step-by-step in `config.yaml`, as explained in the [**Setting-up Config**](setting-up-config.md) guide. The canonical field name is now `calibration_sequence`.

**Be careful when modifying calibration steps**, because they determine which motor limits and joint-to-motor ratios get written into `calibration.yaml`.

## Tensioning

Tendons should be firm, but **not overtightened**. A small amount of give is acceptable.

To tension the hand:

1. If needed, move the servos into a suitable starting state with `tension.py --move_motors`. This uses the configured calibration current and then holds the motors in place.
2. Run the `tension.py` script located in the `scripts` folder. This locks the spools in place so the tendons can be tightened safely.
3. Using the included ratchet or your preferred maintenance tool, rotate the top spool gradually to remove slack.
4. Stop once the tendon feels firm and consistent.

> **Careful:** It’s easy to overtension the tendons. Overtensioning can interfere with performance and calibration quality. Tighten **only until the tendon feels firm**.

## Initial Calibration and Tensioning (After Assembly)

When the hand is first assembled, significant slack is often introduced during tendon spooling. That slack must be removed before you rely on the hand for motion quality or calibration accuracy.

Recommended steps:

1. Run the initial setup workflow or a full calibration pass.
2. Perform the tensioning procedure as described above.
3. Repeat calibration and tensioning **1-2 more times**, or until **no additional slack** appears after calibration completes.

This is exactly why the repository now ships a dedicated `scripts/setup.py` workflow: it combines repeated tensioning, calibration, neutral positioning, and a motion test into a single operator flow.

## Runtime Perspective

At runtime, the recommended sequence is:

1. `hand.connect()`
2. `hand.init_joints()`
3. `hand.set_joint_positions(...)`

`init_joints()` is the main convenience entrypoint because it:

- enables torque
- applies the configured control mode and current limit
- calibrates if needed
- computes wrap offsets
- moves the hand to neutral
