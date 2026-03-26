# Motor Client Layer

The motor client layer is the hardware abstraction underneath `OrcaHand`.

Most users do not need to interact with it directly, but it matters if you are:

- adding support for a new motor family
- debugging low-level bus behavior
- comparing protocol-specific performance

## Role in the architecture

`OrcaHand` owns hand-level behavior such as calibration, neutral positioning, and joint-space commands.

`MotorClient` implementations own protocol-level behavior such as:

- opening the serial bus
- reading motor position / velocity / current
- reading temperatures
- enabling torque
- writing desired position or current
- setting operating modes

## Current implementations

- `DynamixelClient`
- `FeetechClient`
- `MockDynamixelClient` for tests and in-memory workflows

The specific implementation is chosen from `config.motor_type`.

## What stays above this layer

These responsibilities stay in `OrcaHand`, not in the motor client:

- converting between joint space and motor space
- interpreting signed joint mappings
- loading and applying calibration results
- clamping joint commands against configured ROMs
- coordinating higher-level tasks such as tensioning and jitter

## Extension contract

If you are implementing a new backend, `MotorClient` is the interface to follow.

::: orca_core.hardware.motor_client.MotorClient
