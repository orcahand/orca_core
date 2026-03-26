# ORCA Core API

This page documents the current Python-facing API that the refactored package is organized around.

## Recommended entrypoint

Most application code should start with `OrcaHand`:

```python
from orca_core import OrcaHand

hand = OrcaHand(config_path="orca_core/models/v2/orcahand_right/config.yaml")
success, message = hand.connect()

if success:
    hand.init_joints()
    hand.set_joint_positions({"index_mcp": 20, "middle_mcp": 20})
```

The main lifecycle is:

1. construct `OrcaHand`
2. `connect()`
3. `init_joints()`
4. send joint-space commands
5. `disconnect()`

## Public data objects

### `OrcaHandConfig`

`OrcaHandConfig` is the validated model description loaded from `config.yaml`.

::: orca_core.OrcaHandConfig

### `CalibrationResult`

`CalibrationResult` is the immutable snapshot loaded from `calibration.yaml`.

::: orca_core.CalibrationResult

### `OrcaJointPositions`

`OrcaJointPositions` is the typed container used to send and receive joint-space poses.

::: orca_core.OrcaJointPositions

## Hand classes

### `OrcaHand`

`OrcaHand` extends the shared joint-space interface with hardware lifecycle, calibration, telemetry, and maintenance tasks.

::: orca_core.OrcaHand

### `BaseHand`

`BaseHand` provides the backend-agnostic joint-space API used by real and test hands.

::: orca_core.base_hand.BaseHand

## Internal extension point

### `MotorClient`

`MotorClient` is the low-level contract that concrete motor backends implement. Most users do not need it directly, but it is the right abstraction if you are extending the hardware layer.

::: orca_core.hardware.motor_client.MotorClient

## Notes on units

- Joint-space commands follow the units used in the hand config. Bundled models use degrees.
- Raw motor telemetry is reported in radians.
- Motor current is reported in mA.

## Notes on accepted command shapes

`set_joint_positions(...)` accepts:

- `OrcaJointPositions`
- `dict[str, float | None]`
- `numpy.ndarray`

This makes it easy to use the same API from typed workflows, quick scripts, and numerical code.
