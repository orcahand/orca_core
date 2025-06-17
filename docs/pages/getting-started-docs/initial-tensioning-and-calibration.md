# Calibration and Tensioning

## Calibration

Calibration ensures the hand operates with high accuracy and precision. It should be performed regularly, especially:

- After extended use
- After slack has built up in the tendons
- After tensioning, to restore precision

The calibration sequence is defined step-by-step in the `config.py` file, as explained in the [**Setting-up Config**](setting-up-config.md) guide.

**Be careful when modifying calibration steps**, some are intended to run independently and may not function correctly if grouped or reordered.

## Tensioning

Tendons should be firm (not slack), but **not overtightened**. A small amount of give is acceptable.

To tension the hand:

1. Move all servos **fully counter-clockwise (CCW)**. Instead of moving all servos by hand you can also run `tension.py --move_motors` that turns all servos CCW using the calibration current specified in `config.py`.  
2. Run the `tension.py` script located in the `scripts` folder. This locks the servos in place and secures the bottom spools.
3. Using the included 3D-printed **ratchet** (found in the spools print file or included in the kit), rotate the **top spool of each servo clockwise** to tighten the tendons.
4. As you turn, you should **hear or feel a "clicking" sound**, this confirms the ratchet is engaged and the tendon is being tensioned.

> **Careful:** It’s easy to overtension the tendons when using the ratchet. Overtensioning can interfere with performance. Tighten **only until the tendon feels firm**.

## Initial Calibration and Tensioning (After Assembly)

When the hand is first assembled, because tendons are spooled by hand, significant slack is introduced. This must be corrected before using the hand for other tasks.

Recommended steps:

1. Run calibration (for all joints). Significant slack will accumulate during this step, this is expected, and calibration may seem ineffective at first.
2. Perform the tensioning procedure as described above.
3. Repeat steps 1 and 2 **1-2 more times**, or until **no additional slack** appears after calibration completes.

This ensures the tendons are properly tensioned and the hand is correctly calibrated.
