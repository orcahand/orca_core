# Quickstart with Core Package

Abstracts hardware and control with simple high level control methods in joint space. 

!!!  warning
This package is still under development and is not yet fully tested.!!! 

## Get Started

To get started with orca_core, follow these steps:

1. **(Recommended) Install orca_core using pip**:
    :::info 
    We are still uploading to PyPI. In the meantime please build from source!
    ```sh
    pip install orca_core
    ```

2. **(Optional) Build from source**:

    ```sh
    git clone git@github.com:orcahand/orca_core.git
    cd orca_core
    poetry install
    ```

3. **Download the `orca_config.yaml` file (e.g. orca_v1)**:
    ```sh
    curl -o orca_config.yaml https://raw.githubusercontent.com/orcahand/orca_models/main/orca_v1/orca_config.yaml
    ```
    :::danger
    Make sure to have the correct motors assigned to the joints in the `config.yaml` file
    :::

4. **Connect the hand and run the example usage**:
    ```python
    # Example usage
    from orca_core import OrcaHand

    hand = OrcaHand(orca_config="orca_config.yaml")
    status = hand.connect()
    print(status)
    hand.calibrate()

    # Set the desired joint positions to 0
    hand.set_joint_pos({joint: 0 for joint in hand.joint_ids})
    hand.disconnect()
    ```

## Installation

```bash
# Installation steps will go here
```

## Basic Usage

1. Import the package
2. Initialize the system
3. Run your first program

More content will be added here... 