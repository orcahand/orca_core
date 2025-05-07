[![PyPI version](https://badge.fury.io/py/orca_core.svg)](https://badge.fury.io/py/orca_core)
[![GitHub stars](https://img.shields.io/github/stars/orcahand/orca_core.svg?style=social)](https://github.com/yourusername/orca_core/stargazers)

OrcaHand class is used to abstract hardware, control the hand of the robot with simple high-level control methods in joint space.

# Orca Core

OrcaHand class is used to abtract hardware, control the hand of the robot with simple high level control methods in joint space. 

## Get Started

To get started with Orca Core, follow these steps:

1. **Clone the repository**:

    ```sh
    git clone git@github.com:orcahand/orca_core.git
    cd orca_core
    ```

2. **Install dependencies using Poetry**:

    ```sh
    poetry install
    ```

3. **Run the example usage**:

    ```python
    # Example usage
    from orca_core import OrcaHand

    hand = OrcaHand()
    status = hand.connect()
    print(status)
    hand.calibrate()

    # Set the desired joint positions to 0
    hand.set_joint_pos({joint: 0 for joint in hand.joint_ids})
    hand.disconnect()
    ```

## Config file

The configuration file `core/orca_config.yaml` is specific to the hand (currently the only hand we have) and defines everything from auto calibration, joint limits, and control settings. This file is crucial for the proper functioning of the OrcaHand class.
