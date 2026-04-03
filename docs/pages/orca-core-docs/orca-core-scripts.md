# Scripts API Documentation

This document provides an overview of the available scripts in the `scripts` folder, updated to reflect the refactored package structure on `main`.

### Calibration Scripts

<details>
<summary><strong>calibrate.py</strong></summary>

Runs the hand calibration routine and writes the resulting motor limits and joint-to-motor ratios into `calibration.yaml`.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file (e.g., `/path/to/orcahand_right/config.yaml`). If not provided, the default packaged model is used.</li>
    <li><strong>--force-wrist</strong>: Force wrist calibration even if the wrist is already marked as calibrated.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/calibrate.py /path/to/orcahand_right/config.yaml --force-wrist
```
</details>

### Motor and Joint Check Scripts

<details>
<summary><strong>check_motor.py</strong></summary>

Checks a specific motor directly through the low-level motor client. This script is useful for isolating bus or single-motor issues outside the higher-level `OrcaHand` joint-space API.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>--port</strong> (<strong>str</strong>, optional): Serial port path.</li><br>
    <li><strong>--baudrate</strong> (<strong>int</strong>, optional): Motor bus baudrate.</li><br>
    <li><strong>--motor_id</strong> (<strong>int</strong>, optional): Motor ID to test.</li><br>
    <li><strong>--wrist</strong> (<strong>action</strong>, optional): Use wrist-friendly operating mode assumptions.</li><br>
    <li><strong>--reverse</strong> (<strong>action</strong>, optional): Sweep the target in the reverse direction.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/check_motor.py --motor_id 5 --port /dev/ttyUSB0
```
</details>

### Demonstration Scripts

<details>
<summary><strong>main_demo.py</strong></summary>

Runs a continuous hand demo using joint-space commands through `OrcaHand`.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/main_demo.py /path/to/orcahand_right/config.yaml
```
</details>

<details>
<summary><strong>main_demo_abduction.py</strong></summary>

Runs a demo focused on abduction motions using the current joint-space API.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/main_demo_abduction.py /path/to/orcahand_right/config.yaml
```
</details>

### Position Control Scripts

<details>
<summary><strong>neutral.py</strong></summary>

Moves the hand to the configured neutral pose using `set_neutral_position()`.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/neutral.py /path/to/orcahand_right/config.yaml
```
</details>

<details>
<summary><strong>zero.py</strong></summary>

Moves every configured joint to zero through the joint-space interface.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/zero.py /path/to/orcahand_right/config.yaml
```
</details>

### Recording and Replay Scripts

<details>
<summary><strong>record_angles.py</strong></summary>

Records waypoint-style joint poses from the current hand state and saves them to a replay file.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li><br>
    <li><strong>--output_dir</strong> (<strong>str</strong>, optional): Output directory for the replay sequence.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/record_angles.py /path/to/orcahand_right/config.yaml --output_dir my_recordings
```
</details>

<details>
<summary><strong>record_continuous.py</strong></summary>

Continuously records joint states from the hand at a chosen sampling rate.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li><br>
    <li><strong>--frequency</strong> (<strong>float</strong>, optional): Sampling frequency in Hz.</li><br>
    <li><strong>--duration</strong> (<strong>float</strong>, optional): Recording duration in seconds.</li><br>
    <li><strong>--output_dir</strong> (<strong>str</strong>, optional): Output directory.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/record_continuous.py /path/to/orcahand_right/config.yaml --frequency 100 --duration 10
```
</details>

<details>
<summary><strong>replay_angles.py</strong></summary>

Replays a waypoint-based motion sequence through `set_joint_positions(...)`.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li><br>
    <li><strong>--step_time</strong> (<strong>float</strong>, optional): Interpolation step time.</li><br>
    <li><strong>--replay_file</strong> (<strong>str</strong>, required): Replay sequence file.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/replay_angles.py /path/to/orcahand_right/config.yaml --replay_file my_sequence.yaml --step_time 0.01
```
</details>

<details>
<summary><strong>replay_continuous.py</strong></summary>

Replays a continuously recorded joint trajectory.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li><br>
    <li><strong>--replay_file</strong> (<strong>str</strong>, required): Replay file.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/replay_continuous.py /path/to/orcahand_right/config.yaml --replay_file continuous_sequence.yaml
```
</details>

### UI Control Scripts

<details>
<summary><strong>slider_joint.py</strong></summary>

Provides a Tkinter-based joint-space control UI built on top of `OrcaHand`.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/slider_joint.py /path/to/orcahand_right/config.yaml
```
</details>

<details>
<summary><strong>slider_motor.py</strong></summary>

Provides a Tkinter-based motor-space debugging UI for direct low-level control.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/slider_motor.py /path/to/orcahand_right/config.yaml
```
</details>

### Miscellaneous Scripts

<details>
<summary><strong>tension.py</strong></summary>

Enables torque and holds the hand in a maintenance-friendly state for tendon tensioning. With `--move_motors`, it first drives the motors using the configured calibration current.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li><br>
    <li><strong>--move_motors</strong>: Pre-position the motors before holding tension.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/tension.py /path/to/orcahand_right/config.yaml --move_motors
```
</details>

<details>
<summary><strong>setup.py</strong></summary>

Runs the full operator workflow for a real hand: repeated tensioning, calibration, neutral positioning, and a motion test.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/setup.py /path/to/orcahand_right/config.yaml
```
</details>

<details>
<summary><strong>test.py</strong></summary>

A manual hardware exercise script that connects to the hand, monitors temperatures, and cycles between open and closed poses.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/test.py /path/to/orcahand_right/config.yaml
```
</details>
