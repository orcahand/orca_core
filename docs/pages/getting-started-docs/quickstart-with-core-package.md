Orca Core is the core control package of the ORCA Hand. In the refactored codebase, the main runtime object is `OrcaHand`, which combines a validated `OrcaHandConfig`, a `CalibrationResult`, and a concrete motor client backend.

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

    - Review the config file you want to use, such as `orca_core/models/v2/orcahand_right/config.yaml`.
    - When you pass `config_path` to `OrcaHand`, it must be the path to `config.yaml`.

4. **Run the tension and calibration scripts**:

    For first-time bring-up, prefer the full guided workflow:

    ```sh
    uv run python scripts/setup.py orca_core/models/v2/orcahand_right/config.yaml
    ```

    For targeted maintenance, use:

    ```sh
    uv run python scripts/tension.py orca_core/models/v2/orcahand_right/config.yaml --move_motors
    uv run python scripts/calibrate.py orca_core/models/v2/orcahand_right/config.yaml
    ```

5. **Move the hand to the neutral position**:

    ```sh
    uv run python scripts/neutral.py orca_core/models/v2/orcahand_right/config.yaml
    ```

6. **Example usage: test.py**

    Here is a minimal example script you can use to test your setup:

    ```python
    from orca_core import OrcaHand

    hand = OrcaHand(config_path="orca_core/models/v2/orcahand_right/config.yaml")
    status = hand.connect()
    print(status)
    if not status[0]:
        print("Failed to connect to the hand.")
        raise SystemExit(1)

    try:
        hand.init_joints()

        hand.set_joint_positions(
            {
                "index_mcp": 20,
                "middle_pip": 30,
            },
            num_steps=25,
            step_size=0.001,
        )

        print(hand.get_joint_position().as_dict())
    finally:
        hand.stop_task()
        hand.disconnect()
    ```

---

**Note:**
- The refactored API accepts joint commands as `OrcaJointPositions`, plain dicts, or 1-D numpy arrays.
- `init_joints()` is the recommended bring-up helper because it enables torque, applies the control mode, calibrates if needed, computes wrap offsets, and moves to neutral.
- All hardware-oriented scripts in the `scripts/` folder take the `config.yaml` path as their first argument.
- For more advanced usage, see the API reference page.

---
