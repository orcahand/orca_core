[![PyPI version](https://badge.fury.io/py/orca_core.svg)](https://badge.fury.io/py/orca_core)
[![GitHub stars](https://img.shields.io/github/stars/orcahand/orca_core?style=social)](https://github.com/orcahand/orca_core/stargazers)
[![Twitter Follow](https://img.shields.io/twitter/follow/orcahand?style=social)](https://x.com/orcahand)
[![Website](https://img.shields.io/badge/Website-orcahand.org-blue?style=flat&logo=google-chrome)](https://orcahand.com)


OrcaHand class is used to abstract hardware and control the hand of the robot with simple high-level control methods in joint space.

# Orca Core

OrcaHand class is used to abstract hardware and control the hand of the robot with simple high-level control methods in joint space.

## Get Started

To get started with Orca Core, follow these steps:

1. **Create a virtual environment** (recommended):

    ```sh
    python -m venv venv
    source venv/bin/activate
    ```

    You can also use **Poetry**, **pyenv**, **conda**, or any other environment manager if you prefer.

2. **Install dependencies**:

    ```sh
    pip install -e .
    ```

3. **Check the configuration file**:

    - Review the config file (e.g., `orca_core/orca_core/models/orcahand_v1_right/config.yaml`) and make sure it matches your hardware setup.

4. **Run the tension and calibration scripts**:

    ```sh
    python scripts/tension.py orca_core/orca_core/models/orcahand_v1_right
    python scripts/calibrate.py orca_core/orca_core/models/orcahand_v1_right
    ```

    Replace the path with your specific hand model folder if needed.

5. **Move the hand to the neutral position**:

    ```sh
    python scripts/neutral.py orca_core/orca_core/models/orcahand_v1_right
    ```

6. **Example usage: test.py**

    Here is a minimal example script you can use to test your setup:

    ```python
    from orca_core import OrcaHand
    import time

    hand = OrcaHand('orca_core/orca_core/models/orcahand_v1_right')
    status = hand.connect()
    print(status)
    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    hand.enable_torque()

    joint_dict = {
        "index_mcp": 90,
        "middle_pip": 30,
    }

    hand.set_joint_pos(joint_dict, num_steps=25, step_size=0.001)

    time.sleep(2)
    hand.disable_torque()
    hand.disconnect()
    ```

---

**Note:**  
- Always ensure your `config.yaml` matches your hardware and wiring.
- All scripts in the `scripts/` folder take the model path as their first argument.
- For more advanced usage, see the other scripts and the API documentation.

---
