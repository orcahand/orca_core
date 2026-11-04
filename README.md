<div align="center" style="line-height: 1;">
  <a href="https://arxiv.org/abs/2504.04259" target="_blank"><img alt="arXiv" src="https://img.shields.io/badge/arXiv-2504.04259-B31B1B?logo=arxiv"/></a>
  <a href="https://discord.gg/xvGyxaccRa" target="_blank"><img alt="Discord" src="https://img.shields.io/badge/Discord-orcahand-7289da?logo=discord&logoColor=white&color=7289da"/></a>
  <a href="https://x.com/orcahand" target="_blank"><img alt="Twitter Follow" src="https://img.shields.io/twitter/follow/orcahand?style=social"/></a>
  <a href="https://orcahand.com" target="_blank"><img alt="Website" src="https://img.shields.io/badge/Website-orcahand.com-blue?style=flat&logo=google-chrome"/></a>
  <br>
  <a href="https://github.com/orcahand/orca_core" target="_blank"><img alt="GitHub stars" src="https://img.shields.io/github/stars/orcahand/orca_core?style=social"/></a>
  <a href="https://github.com/orcahand/orca_core/actions/workflows/test.yml" target="_blank"><img alt="Tests" src="https://github.com/orcahand/orca_core/actions/workflows/test.yml/badge.svg"/></a>
</div>

Orca Core is the core control package of the ORCA Hand. It's used to abstract hardware, provide scripts for calibration, tensioning and to control the hand with simple high-level control methods in joint space. While working directly with core will let you interact with your ORCA Hand on a deep level, the Orca Ui repository (found at: https://github.com/orcahand/orca_ui) can let users get started right away, helping you setup/maintain your hand, tune its performance, and better understand its capabilities. 

## Experimental branch: `feature/improve-calibration`

This branch holds unreleased tensioning and calibration work, and will be merged into `main` in a later release. It reports
version 0.12.0, but it isn't the published 0.12.0.

**With the console:** you don't need this clone. The `merge/assembly-calibration-with-main` branch of
[orca_ui](https://github.com/orcahand/orca_ui/tree/merge/assembly-calibration-with-main#using-this-experimental-branch) installs
the matching commit by itself.

**From the terminal:**

```sh
git clone --branch feature/improve-calibration https://github.com/orcahand/orca_core.git
cd orca_core
uv sync --group dev
uv run python scripts/detect.py    # read-only: which model, serial and motor type is plugged in
```

Give each hand its own copy of its model's config, because calibration and tensioning records are saved beside the config you load:

```sh
mkdir -p ~/orca_bench/ser-0000/orcahand-right
cp orca_core/models/v2/orcahand-right/config.yaml ~/orca_bench/ser-0000/orcahand-right/
uv run python scripts/guided_tension.py ~/orca_bench/ser-0000/orcahand-right/config.yaml
uv run python scripts/calibrate.py ~/orca_bench/ser-0000/orcahand-right/config.yaml
```

Replace `ser-0000` and `orcahand-right` with what `detect.py` shows. Point everything else at that same `config.yaml`, including
teleoperation's `--model_path`. Tension first and calibrate last.

**What's different from `main`:**

- **Guided tensioning:** motors pull at `max_current`, settle where they stopped, then hold at each motor's ceiling while you ratchet.
  Avoid hard pushes on a finger while the motors hold, because they may loosen the tendon.
- **Records:** tensioning records live in `setup.yaml` beside the config. Older `last_tension.yaml` and `tension_progress.yaml` are still read.
- **Slack test and tension sweep:** these are new routines, with currents per motor type. Feetech values are provisional, and on Feetech
  a slack test pass can miss small slack.
- **Joint loop:** the slack test and sweep refuse to run while a joint-feedback loop is running.

## Get Started

To get started with Orca Core, follow these steps:

**Pre-installation steps**

1. **Git clone the orca_core repository**:
   ```
   #use any preferred method, ssh clone shown here
   git clone git@github.com:orcahand/orca_core.git
    ```

2. **Install uv (found at: https://docs.astral.sh/uv/getting-started/)**

**Installing the repository**

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
    source .venv/bin/activate      # macOS / Linux
    .venv\Scripts\activate         # Windows
    ```

    End users who do not use `uv` can still install the package with:

    ```sh
    pip install .
    ```
**Post-installation sample usage scripts**

1. **Check the configuration file**:

    - Review the config file (e.g., `orca_core/models/v2/orcahand-right/config.yaml`) and make sure it matches your hardware setup.

2. **Run the tension and calibration scripts**:

    ```sh
    uv run python scripts/tension.py orca_core/models/v2/orcahand-right/config.yaml
    uv run python scripts/calibrate.py orca_core/models/v2/orcahand-right/config.yaml
    ```

    Replace the path with your specific hand model folder if needed.

3. **Move the hand to the neutral position**:

    ```sh
    uv run python scripts/neutral.py orca_core/models/v2/orcahand-right/config.yaml
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

### Windows

Supported on Windows 10 and 11; no permission setup is needed. Ports are named `COM3`, `COM4`, ... and are listed with:

```sh
uv run python -m serial.tools.list_ports -v
```

Driver notes, latency settings and the first-connection checklist: [Running on Windows](docs/pages/getting-started-docs/windows.md).

### Serial port, baudrate, and motor type

By default these are all **auto-detected** at connect time.

However, you can declare them explicitly in `config.yaml`. Useful when:

- **multiple hands are connected** at once → `port` disambiguates which one
- **motors run at a non-default baudrate** → `baudrate` skips the probe sweep
- **the auto-detection picks the wrong family** → `motor_type` forces a specific one

```yaml
# Optional overrides:   auto-detected if omitted
port: /dev/ttyACM0      # /dev/cu.usbmodemXXXX on macOS, COM3 on Windows
baudrate: 1000000       # 1M for v2; 3M for v1
motor_type: dynamixel   # or 'feetech'
```

