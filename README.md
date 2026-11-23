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

## What's new in 0.14.0

- **Stop after a motor-bus error:** a USB error in the middle of a motor transaction no longer leaves the port marked busy
  ("Port is in use"), so Stop and disconnect still switch torque off and close the port.
- **Holding in place:** goal positions are rounded to the nearest motor step instead of cut off, so a motor told to hold
  where it is no longer pushes against a goal one step away and heats up.
- **Guided tensioning** reports when its live readings stop (`sampler_failed`) or torque can't be switched off at the end
  (`torque_off_failed`), and the terminal script prints these and motor faults. Asking to test a joint again is saved
  even when you go straight to the score.
- **`setup.yaml`:** writes from several programs on the same hand (the console and a script) no longer overwrite each
  other; they take turns through `setup.yaml.lock` beside it.
- **Manual calibration** asks for one joint at a time on configs without `manual_calibration_sequence:`, and
  calibration's drive-current table only lists joints measured in that run.

## What's new in 0.13.0

- **Guided tensioning** (`scripts/guided_tension.py`, v2.1 hands): motors pull at `max_current`, settle where they stopped, then
  hold at each motor's ceiling while you ratchet. Avoid hard pushes on a finger while the motors hold, because they may loosen the
  tendon. Its currents can be overridden per motor family under `guided_tension_currents:` in `config.yaml`.
- **Slack test and tension sweep:** new routines in `orca_core.maintenance` (`run_slack_test`, `run_tension_sweep`), with currents
  per motor family. They refuse to run while a joint-feedback loop is running. Feetech values are provisional, and on Feetech a
  slack test pass can miss small slack.
- **Manual calibration:** `run_calibration(manual=True)` asks you to move each joint to its stops by hand, following
  `manual_calibration_sequence:` in `config.yaml`, and lets you step back to redo a stop.
- **Hardware versions:** boards report three digits (`210` = v2.1.0, `211` = v2.1.1). Older boards that report `21` or `2` are still
  understood, and a v2.1.1 hand also gets `joint_roms_v211:`.
- **Records:** tensioning records live in `setup.yaml` beside the config. Older `last_tension.yaml` and `tension_progress.yaml` are
  still read.
- **Touch sensors:** the sensing link reads whatever bytes have arrived and ignores replies to earlier requests, so touch keeps
  working while the program is busy. Before 0.13.0, a busy program could lose touch until it reconnected.

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
      Joint ranges and the guided-tensioning currents every v2 model shares are in `orca_core/models/v2/shared.yaml`,
      merged under each model's `config.yaml`. To keep a model outside the package, write a complete
      copy with `orca_core.write_standalone_config("my-hand/config.yaml", model_name="orcahand-right")`.

2. **Run the tension and calibration scripts**:

    ```sh
    uv run python scripts/detect.py
    uv run python scripts/guided_tension.py orca_core/models/v2/orcahand-right/config.yaml
    uv run python scripts/calibrate.py orca_core/models/v2/orcahand-right/config.yaml
    ```

    `detect.py` only reads: it shows which model, motor type and hardware version is plugged in. Replace the path with that
    model's folder. Guided tensioning needs v2.1 hardware; on v2.0, use `scripts/tension.py` instead. Tension first and
    calibrate last.

    Calibration and tensioning records are saved in the hand's own folder, `~/.orca/hands/<hand id>/` (see
    [Where a hand's files live](#where-a-hands-files-live)), so several hands of one model can share its config.

3. **Move the hand to the neutral position**:

    ```sh
    uv run python scripts/neutral.py orca_core/models/v2/orcahand-right/config.yaml
    ```

---

## Where a hand's files live

Each physical hand keeps its own files (calibration, setup records, identity) in `~/.orca/hands/<hand id>/` (set
`ORCA_HOME` to move `~/.orca`); a model's `config.yaml` is only ever read. Files older versions kept beside `config.yaml`
are no longer read; calibrate each hand once. Details: [docs/pages/orca-core-docs/hand-files.md](docs/pages/orca-core-docs/hand-files.md).

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

