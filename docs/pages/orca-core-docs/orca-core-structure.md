# ORCA Core Structure

This page explains the current architecture of the refactored repository.

## High-level layout

- `orca_core/`
  The package itself.
- `scripts/`
  Operator and debugging entrypoints built on top of the package.
- `docs/`
  MkDocs source for this documentation site.
- `tests/`
  Unit and integration coverage for the refactored interfaces.

## Core package modules

### `orca_core/hand_config.py`

Defines the configuration dataclasses:

- `BaseHandConfig` for shared joint-space metadata
- `OrcaHandConfig` for hardware-backed hands

This module is responsible for:

- resolving model config paths
- loading `config.yaml`
- validating joint, motor, and calibration metadata
- normalizing signed motor mappings into absolute IDs plus inversion flags

### `orca_core/calibration.py`

Defines `CalibrationResult`, the immutable object that represents what has been learned about a physical hand after calibration.

### `orca_core/joint_position.py`

Defines `OrcaJointPositions`, the typed container used for joint-space commands and readback.

### `orca_core/base_hand.py`

Defines `BaseHand`, the backend-agnostic joint-space interface.

This is where shared behavior lives:

- clamping against ROM bounds
- accepting commands as typed objects, dicts, or numpy arrays
- interpolation between current and target pose
- named positions
- neutral and zero helpers

### `orca_core/hardware_hand.py`

Defines `OrcaHand`, the physical-hand implementation, plus `MockOrcaHand` for testing.

This module owns the hardware lifecycle:

- connect / disconnect
- torque control
- control mode selection
- current limits
- raw motor telemetry
- calibration
- motor-to-joint and joint-to-motor conversion
- maintenance helpers such as tensioning and jitter

### `orca_core/hardware/`

Defines the low-level motor client interface and concrete backends.

- `motor_client.py`: abstract contract
- `dynamixel_client.py`: Dynamixel implementation
- `feetech_client.py`: Feetech implementation
- `mock_dynamixel_client.py`: in-memory test backend

`OrcaHand` selects the correct backend based on `config.motor_type`.

### `orca_core/models/`

Bundled hand models and example configs. Each model directory contains:

- `config.yaml`
- optionally `calibration.yaml`

The refactored code treats these as versioned assets rather than ad hoc examples.

## Practical mental model

The package is easiest to understand as a pipeline:

1. `config.yaml` becomes `OrcaHandConfig`
2. `calibration.yaml` becomes `CalibrationResult`
3. `OrcaHand` combines those objects with a `MotorClient`
4. application code sends joint-space commands through `BaseHand` / `OrcaHand`

That split is the main conceptual improvement of the refactor: static model description, dynamic calibration state, and live hardware control are now distinct layers.
