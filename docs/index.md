# ORCA Core

`orca_core` is the software layer that turns an ORCA Hand model description and calibration state into a controllable hardware interface.

The codebase is now organized around a few explicit concepts:

- `OrcaHandConfig` is the static description of a hand model, loaded from `config.yaml`.
- `CalibrationResult` is the mutable calibration state, loaded from `calibration.yaml`.
- `BaseHand` defines the shared joint-space interface.
- `OrcaHand` is the hardware-backed implementation used for real hands.
- `MotorClient` implementations handle the concrete motor bus protocol.

If you are new to the project, the shortest useful path through the docs is:

1. Set up the motor chain and choose the right model config.
2. Review the `config.yaml` schema and confirm it matches your hardware.
3. Run the setup or calibration workflow.
4. Use `OrcaHand` for joint-space control.

## What changed in the refactor

The older docs described a more script-centric package and used several outdated names. The current codebase is centered on:

- explicit config and calibration data objects
- a backend-agnostic `BaseHand`
- a hardware-specific `OrcaHand`
- typed joint payloads via `OrcaJointPositions`
- canonical config keys such as `calibration_sequence` instead of `calib_sequence`

## Recommended first steps

- Read the getting-started pages if you are bringing up hardware.
- Read the API page if you are integrating `orca_core` into another Python project.
- Read the repository structure page if you want the high-level architecture before diving into the source.
