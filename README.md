<div align="center" style="line-height: 1;">
  <a href="https://arxiv.org/abs/2504.04259" target="_blank"><img alt="arXiv" src="https://img.shields.io/badge/arXiv-2504.04259-B31B1B?logo=arxiv"/></a>
  <a href="https://discord.gg/xvGyxaccRa" target="_blank"><img alt="Discord" src="https://img.shields.io/badge/Discord-orcahand-7289da?logo=discord&logoColor=white&color=7289da"/></a>
  <a href="https://x.com/orcahand" target="_blank"><img alt="Twitter Follow" src="https://img.shields.io/twitter/follow/orcahand?style=social"/></a>
  <a href="https://orcahand.com" target="_blank"><img alt="Website" src="https://img.shields.io/badge/Website-orcahand.com-blue?style=flat&logo=google-chrome"/></a>
  <br>
  <a href="https://github.com/orcahand/orca_core" target="_blank"><img alt="GitHub stars" src="https://img.shields.io/github/stars/orcahand/orca_core?style=social"/></a>
  <a href="https://github.com/orcahand/orca_core/actions/workflows/test.yml" target="_blank"><img alt="Tests" src="https://github.com/orcahand/orca_core/actions/workflows/test.yml/badge.svg"/></a>
</div>

Orca Core is the core control package of the ORCA Hand. It's used to abstract hardware, provide scripts for calibration, tensioning and to control the hand with simple high-level control methods in joint space.

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

    - Review the config file (e.g., `orca_core/models/v2/orcahand_right/config.yaml`) and make sure it matches your hardware setup.

4. **Run the tension and calibration scripts**:

    ```sh
    uv run python scripts/tension.py orca_core/models/v2/orcahand_right/config.yaml
    uv run python scripts/calibrate.py orca_core/models/v2/orcahand_right/config.yaml
    ```

    Replace the path with your specific hand model folder if needed.

5. **Move the hand to the neutral position**:

    ```sh
    uv run python scripts/neutral.py orca_core/models/v2/orcahand_right/config.yaml
    ```

---

## Troubleshooting

### Serial Port Permissions (Linux)

On Linux, the serial port (e.g., `/dev/ttyACM0`) is owned by the `dialout` group. If your user is not in this group, you will get a **permission denied** error and motors won't be detected.

**Permanent fix** (requires re-login):

```sh
sudo usermod -aG dialout $USER
```

**Temporary fix** (resets on reboot/replug):

```sh
sudo chmod 666 /dev/ttyACM0
```

### Serial port, baudrate, and motor type

By default these are all **auto-detected** at connect time.

However, you can declare them explicitly in `config.yaml`. Useful when:

- **multiple hands are connected** at once → `port` disambiguates which one
- **motors run at a non-default baudrate** → `baudrate` skips the probe sweep
- **the auto-detection picks the wrong family** → `motor_type` forces a specific one

```yaml
# Optional overrides:   auto-detected if omitted
port: /dev/ttyACM0      # or /'dev/cu.usbmodemXXXX' on macOS
baudrate: 1000000       # 1M for v2; 3M for v1 
motor_type: dynamixel   # or 'feetech'
```

