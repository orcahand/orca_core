# Orca Core Repository Structure

This document outlines the general structure of the `orca_core` repository.

*   **`orca_core/`**: The main python package for OrcaHand control.
    *   `core.py`: Contains the central *OrcaHand class*, which provides the high-level interface for controlling the robotic hand. This includes methods for connection, calibration, setting joint positions, and reading sensor data. <br> [**OrcaHand Class methods**](orcahand-api.md)
    *   **`api/`**: Contains the FastAPI application for exposing *OrcaHand* functionalities over a web API.
        *   `api.py`: Defines the FastAPI endpoints, request/response models, and integrates with the OrcaHand class. This is not properly implemented yet, please ignore.

    *   **`hardware/`**: Modules for interacting with specific hardware components.
        *   `dynamixel_client.py`: Implements the communication logic for Dynamixel servo motors.

    *   **`models/`**: Stores configuration files specific to different hand models. Each sub-directory typically represents a specific hand version (or just left-right) or configuration and contains:
        *   `config.yaml`: Defines static parameters of the hand, such as motor IDs, joint IDs, control modes, calibration sequences, and neutral positions.
        *   `calibration.yaml`: Stores calibration data, such as motor limits and joint-to-motor ratios, generated during the calibration process.
        
    *   **`utils/`**: Contains utility modules with helper functions.

*   **`scripts/`**: Contains standalone Python scripts for performing various operations with the *OrcaHand*, such as auto-calibration (*calibrate.py*), Tensioning (*tension.py*) etc. <br> [**Explore Scripts**](orca-core-scripts.md)

*   **`tests/`**: Includes unit tests and integration tests for the *orca_core* package to ensure code correctness and reliability (Under development).

*   **`docs/`**: Contains the generated documentation website (built using MkDocs). Ingore this directory. 
    
*   **`examples/sequences/`**: Recorded joint-space sequences for replay.
    *   *kapandji_opposition.yaml*: The Kapandji thumb-opposition test — the thumb tip touches each fingertip in turn. Recorded on an orcahand v1; `metadata.joint_ids` pins the joint order, so `replay_angles.py` refuses to run it on a hand whose ordering differs.

*   **`replay_sequences/`**: Stores YAML files that define sequences of joint positions or motor commands for the hand to replay.

*   `mkdocs.yml`: Configuration file for the MkDocs documentation generator. You can ingore this.