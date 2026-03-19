Orca Core is the core control package of the ORCA Hand. It's used to abstract hardware, provide scripts for calibration and tensioning and to control the hand with simple high-level control methods in joint space.

## Get Started

To get started with Orca Core, follow these steps:

1. **Sync a local development environment with `uv`**:

    ```sh
    uv sync --group dev
    ```

    This creates a local `.venv` and installs the package plus development dependencies.

2. **Run commands through `uv`**:

    ```sh
    uv run pytest
    ```

    If you prefer an activated shell, you can still use:

    ```sh
    source .venv/bin/activate
    ```

    End users who do not use `uv` can still install the package with:

    ```sh
    pip install .
    ```

3. **Check the configuration file**:

    - Review the config file (e.g., `orca_core/models/orcahand_v1_right/config.yaml`) and make sure it matches your hardware setup.

4. **Run the tension and calibration scripts**:

    ```sh
    uv run python scripts/tension.py orca_core/models/orcahand_v1_right
    uv run python scripts/calibrate.py orca_core/models/orcahand_v1_right
    ```

    Replace the path with your specific hand model folder if needed.

5. **Move the hand to the neutral position**:

    ```sh
    uv run python scripts/neutral.py orca_core/models/orcahand_v1_right
    ```

6. **Example usage: test.py**

    Here is a minimal example script you can use to test your setup:

    ```python
    from orca_core import OrcaHand
    import time

    hand = OrcaHand("orca_core/models/orcahand_v1_right")
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
