# About Orca

## Welcome to the Orca Hand Core Documentation

This site documents the current refactored `orca_core` package on `main`.

The codebase is now structured around a few explicit concepts:

- `OrcaHandConfig` for the static hand model description in `config.yaml`
- `CalibrationResult` for the mutable calibration state in `calibration.yaml`
- `BaseHand` for the shared joint-space interface
- `OrcaHand` for the hardware-backed implementation
- `MotorClient` backends for the concrete motor bus protocol

The pages in this site keep the same layout as before, but the content now reflects the refactored package rather than the older script-first structure.

Whether you're configuring your first system or extending its capabilities, this documentation is intended to support development and experimentation with the control software.

!!! Info
    To ensure the hand functions as intended, we strongly recommend following the official guide on the [ORCA Hand Official site](https://orcahand.com) closely.
