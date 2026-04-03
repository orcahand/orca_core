# Orca Core Repository Structure

This document outlines the current structure of the refactored `orca_core` repository.

*   **`orca_core/`**: The main Python package for Orca Hand control.
    *   `base_hand.py`: Defines the backend-agnostic joint-space interface shared by all hand implementations. It handles clamping, interpolation, named poses, and neutral / zero helpers.
    *   `hardware_hand.py`: Defines `OrcaHand`, the hardware-backed implementation, plus `MockOrcaHand` for testing and prototyping. This is where connection management, torque, calibration, motor telemetry, and maintenance tasks live. <br> [**OrcaHand Class methods**](orcahand-api.md)
    *   `hand_config.py`: Defines `BaseHandConfig`, `OrcaHandConfig`, and helpers such as `canonical_joint_ids`. This module loads and validates `config.yaml`.
    *   `calibration.py`: Defines `CalibrationResult`, the immutable snapshot loaded from `calibration.yaml`.
    *   `joint_position.py`: Defines `OrcaJointPositions`, the typed joint-space container used throughout the refactored API.
    *   **`hardware/`**: Low-level motor communication clients and their abstract contract.
        *   `motor_client.py`: Defines the abstract `MotorClient` interface.
        *   `dynamixel_client.py`: Implements the Dynamixel backend.
        *   `feetech_client.py`: Implements the Feetech backend.
        *   `mock_dynamixel_client.py`: Provides the in-memory mock motor client used by `MockOrcaHand`.
    *   **`models/`**: Bundled hand model assets.
        *   `config.yaml`: Defines the static hand description: motor IDs, joint IDs, mappings, ROMs, neutral pose, and calibration settings.
        *   `calibration.yaml`: Stores calibration output such as motor limits and joint-to-motor ratios.
    *   **`utils/`**: Helper functions for model-path resolution, YAML I/O, interpolation, and serial-port detection.
    *   **`api/`**: Contains an older FastAPI experiment. It is not the primary interface of the current refactored package.

*   **`scripts/`**: Standalone Python scripts built on top of the refactored package for calibration, tensioning, setup, testing, recording, replay, and debugging. <br> [**Explore Scripts**](orca-core-scripts.md)

*   **`tests/`**: Unit and integration tests covering the refactored hand, config, calibration, and script behavior.

*   **`docs/`**: Source files for the MkDocs documentation site.

*   **`replay_sequences/`**: Stores recorded or generated motion sequences for replay scripts.

*   `mkdocs.yml`: Configuration file for the MkDocs documentation generator.
