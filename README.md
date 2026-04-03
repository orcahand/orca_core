<div align="center" style="line-height: 1;">
  <a href="https://arxiv.org/abs/2504.04259" target="_blank"><img alt="arXiv" src="https://img.shields.io/badge/arXiv-2504.04259-B31B1B?logo=arxiv"/></a>
  <a href="https://discord.gg/xvGyxaccRa" target="_blank"><img alt="Discord" src="https://img.shields.io/badge/Discord-orcahand-7289da?logo=discord&logoColor=white&color=7289da"/></a>
  <a href="https://x.com/orcahand" target="_blank"><img alt="Twitter Follow" src="https://img.shields.io/twitter/follow/orcahand?style=social"/></a>
  <a href="https://orcahand.com" target="_blank"><img alt="Website" src="https://img.shields.io/badge/Website-orcahand.com-blue?style=flat&logo=google-chrome"/></a>
  <br>
  <a href="https://github.com/orcahand/orca_core" target="_blank"><img alt="GitHub stars" src="https://img.shields.io/github/stars/orcahand/orca_core?style=social"/></a>
  <a href="https://github.com/orcahand/orca_core/actions/workflows/test.yml" target="_blank"><img alt="Tests" src="https://github.com/orcahand/orca_core/actions/workflows/test.yml/badge.svg"/></a>
</div>

Orca Core is the Python control package for the ORCA Hand. The current refactored codebase is organized around a small set of explicit pieces:

- `OrcaHandConfig` loads and validates the static hand description from `config.yaml`.
- `CalibrationResult` loads the mutable calibration state from `calibration.yaml`.
- `BaseHand` provides the shared joint-space interface.
- `OrcaHand` adds hardware connection, calibration, torque control, telemetry, and maintenance tasks.
- `OrcaJointPositions` is the typed container used for joint-space commands and readback.

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
    - When you pass `config_path` into `OrcaHand`, it must point to the `config.yaml` file itself.

4. **Run the setup, tensioning, and calibration flow**:

    For a first hardware bring-up, the most complete workflow is:

    ```sh
    uv run python scripts/setup.py orca_core/models/v2/orcahand_right/config.yaml
    ```

    For an already assembled hand that only needs maintenance:

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

        neutral = hand.config.neutral_position
        hand.set_joint_positions(
            {
                "thumb_mcp": neutral["thumb_mcp"] + 10,
                "index_mcp": neutral["index_mcp"] + 10,
                "middle_mcp": neutral["middle_mcp"] + 10,
            },
            num_steps=25,
            step_size=0.01,
        )

        print(hand.get_joint_position().as_dict())
    finally:
        hand.stop_task()
        hand.disconnect()
    ```

---

## Troubleshooting

### Serial Port Permissions (Linux)

On Linux, the serial port (e.g., `/dev/ttyUSB0`) is owned by the `dialout` group. If your user is not in this group, you will get a **permission denied** error and motors won't be detected.

**Permanent fix** (requires re-login):

```sh
sudo usermod -aG dialout $USER
```

**Temporary fix** (resets on reboot/replug):

```sh
sudo chmod 666 /dev/ttyUSB0
```

### Serial Port Path

Make sure the `port` field in your `config.yaml` matches your operating system:

| OS    | Example port                    |
|-------|---------------------------------|
| Linux | `/dev/ttyUSB0`                  |
| macOS | `/dev/tty.usbserial-XXXXXXXX`   |

---

**Note:**
- Always ensure your `config.yaml` matches your hardware and wiring.
- All hardware-oriented scripts in the `scripts/` folder expect a `config.yaml` path as their first argument.
- Joint-space commands use the same units as the values in `joint_roms` and `neutral_position`.
- For more advanced usage, see the rest of the docs and the API reference.

---
