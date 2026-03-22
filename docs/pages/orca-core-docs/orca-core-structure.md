# Orca Core Repository Structure

`orca_core` is organized around a stable framework surface plus operator tooling:

- `orca_core/core.py`: High-level `OrcaHand` controller built from a profile, runtime state store, and motor driver.
- `orca_core/profile.py`: Immutable hand profile types and loaders for built-in or external profiles.
- `orca_core/state.py`: Runtime state abstractions for calibration data and last-known connection settings.
- `orca_core/drivers.py`: Internal driver registry that resolves motor clients from profile driver config.
- `orca_core/cli.py`: Installed `orca-hand` CLI for calibration, tension, and setup workflows.
- `orca_core/hardware/`: Hardware-specific motor client implementations.
- `orca_core/models/`: Built-in official ORCA hand profiles shipped inside the package.

The legacy `scripts/` directory is now considered development and compatibility tooling. Installed users should prefer the packaged CLI commands over repo-local script execution.

The FastAPI module under `orca_core/api/` is an unsupported preview layer and is not part of the stable 1.0 framework contract.
