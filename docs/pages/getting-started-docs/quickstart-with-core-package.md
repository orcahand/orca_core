# Quickstart With `orca_core`

This page covers the current happy path for bringing up an ORCA Hand with the refactored package.

## 1. Install the package

```sh
python -m venv .venv
source .venv/bin/activate
pip install -e .
```

## 2. Choose the hand model

Pick the `config.yaml` file that matches your hardware. Bundled examples include:

- `orca_core/models/v2/orcahand_right/config.yaml`
- `orca_core/models/v1/orcahand_right/config.yaml`
- `orca_core/models/v1/orcahand_left/config.yaml`

`OrcaHand(config_path=...)` expects the full path to `config.yaml`.
For the current canonical schema, use the `v2` config as your reference. Older `v1` configs may still need the `calib_*` keys migrated to `calibration_*`.

## 3. Check the config before moving hardware

At minimum, confirm:

- `port` matches your machine
- `motor_type` matches your motor chain
- `motor_ids` and `joint_to_motor_map` match the physical build
- `neutral_position` is reasonable for your assembly
- calibration keys use the canonical refactored names such as `calibration_current` and `calibration_sequence`

## 4. Run the setup flow

For first-time bring-up or after re-stringing / major maintenance, use the guided workflow:

```sh
python scripts/setup.py orca_core/models/v2/orcahand_right/config.yaml
```

This script walks through repeated rounds of:

- tensioning
- calibration
- neutral positioning
- motion verification

If the hand is already assembled and only needs a fresh calibration:

```sh
python scripts/calibrate.py orca_core/models/v2/orcahand_right/config.yaml
python scripts/neutral.py orca_core/models/v2/orcahand_right/config.yaml
```

## 5. Minimal Python example

```python
from orca_core import OrcaHand

hand = OrcaHand(config_path="orca_core/models/v2/orcahand_right/config.yaml")

success, message = hand.connect()
if not success:
    raise RuntimeError(message)

try:
    hand.init_joints()

    neutral = hand.config.neutral_position
    hand.set_joint_positions(
        {
            "thumb_mcp": neutral["thumb_mcp"] + 10,
            "index_mcp": neutral["index_mcp"] + 8,
            "middle_mcp": neutral["middle_mcp"] + 8,
        },
        num_steps=25,
        step_size=0.01,
    )
finally:
    hand.stop_task()
    hand.disconnect()
```

## 6. Understand the main lifecycle

The recommended runtime sequence is:

1. `hand = OrcaHand(...)`
2. `hand.connect()`
3. `hand.init_joints()`
4. `hand.set_joint_positions(...)`
5. `hand.disconnect()`

`init_joints()` is the main convenience entrypoint. It:

- enables torque
- applies the configured control mode
- applies the configured current limit
- calibrates if needed
- computes wrap offsets
- moves the hand to neutral

## 7. Units and command shapes

- Joint-space commands accept `OrcaJointPositions`, `dict[str, float]`, or a 1-D numpy array.
- Joint-space values use the same units as `joint_roms` in the config. The bundled configs use degrees.
- Raw motor position telemetry is reported in radians.

## 8. Common helper scripts

- `scripts/tension.py`: hold spools in place during tendon tensioning
- `scripts/zero.py`: move every joint to zero
- `scripts/slider_joint.py`: debug joint-space control
- `scripts/slider_motor.py`: debug motor-space control

## 9. Linux serial permissions

If the hand fails to connect on Linux because of serial permissions:

```sh
sudo usermod -aG dialout $USER
```

Re-log after changing group membership.
