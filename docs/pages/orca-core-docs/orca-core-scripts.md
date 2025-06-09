---
title: "All Scripts"
sidebar_position: 3
---

# Scripts API Documentation

This document provides an overview of the available scripts in the `scripts` folder.

### Calibration Scripts

<details>
<summary><strong>calibrate.py</strong></summary>

Calibrates the ORCA Hand. This script reads the calibration sequence from the hand's configuration and applies it.

Args:
- <strong>model_path</strong> (<strong>str</strong>) – Path to the orcahand model folder (e.g., `/path/to/orcahand_v1_right`).

<strong>Example:</strong>
```bash
python scripts/calibrate.py /path/to/orcahand_v1_right
```
</details>

<details>
<summary><strong>calibrate_manual.py</strong></summary>

Manually calibrates the ORCA Hand. This script guides the user through a manual calibration process.

<strong>Args:</strong>
- <strong>model_path</strong> (<strong>str</strong>) – Path to the orcahand model folder (e.g., `/path/to/orcahand_v1_right`).

<strong>Example:</strong>
```bash
python scripts/calibrate_manual.py /path/to/orcahand_v1_right
```
</details>

### Motor and Joint Check Scripts

<details>
<summary><strong>check_motor.py</strong></summary>

Checks a specific motor (ID 2) by setting its operating mode to current-based position control, enabling torque, and then incrementally increasing its target position. It prints the current and target positions. This script appears to be hardcoded for a specific motor and setup.

<strong>Args:</strong>
- None

<strong>Example:</strong>
```bash
python scripts/check_motor.py
```
</details>

<details>
<summary><strong>check_wrist.py</strong></summary>

Checks the wrist motor (ID 2) by setting its operating mode to position control, enabling torque, and then incrementally increasing its target position. It prints the current and target positions. This script also appears to be hardcoded for a specific motor and setup.

<strong>Args:</strong>
- None

<strong>Example:</strong>
```bash
python scripts/check_wrist.py
```
</details>

### Demonstration Scripts

<details>
<summary><strong>main_demo.py</strong></summary>

Runs a demonstration of the ORCA Hand, making the fingers perform a wave-like motion. It initializes the hand, defines joint ranges, and then continuously updates joint positions to create the animation. The script uses a hardcoded model path.

<strong>Args:<strong>
- None

<strong>Example:<strong>
```bash
python scripts/main_demo.py
```
</details>

<details>
<summary><strong>main_demo_abduction.py</strong></summary>

Runs a demonstration of the ORCA Hand, similar to `main_demo.py`, but with a focus on abduction movements. It initializes the hand, defines joint ranges, and then continuously updates joint positions. The script uses a hardcoded model path.

<strong>Args:<strong>
- None

<strong>Example:<strong>
```bash
python scripts/main_demo_abduction.py
```
</details>

### Position Control Scripts

<details>
<summary><strong>neutral.py</strong></summary>

Moves the ORCA Hand to its neutral (home) position. It connects to the hand, enables torque, sets the neutral position, and then disables torque and disconnects.

<strong>Args:<strong>
- <strong>model_path<strong> (<strong>str<strong>) – Path to the hand model directory.

<strong>Example:<strong>
```bash
python scripts/neutral.py /path/to/orcahand_v1_right
```
</details>

<details>
<summary><strong>zero.py</strong></summary>

Moves the wrist joint (motor 17, typically the last motor ID) incrementally. It connects to the hand, enables torque, and then enters a loop to slightly increase the position of the last motor. The script seems to be intended for testing or fine-tuning the wrist motor.

<strong>Args:<strong>
- <strong>model_path<strong> (<strong>str<strong>) – Path to the orcahand model folder (e.g., `/path/to/orcahand_v1_right`).

<strong>Example:<strong>
```bash
python scripts/zero.py /path/to/orcahand_v1_right
```
</details>

### Recording and Replay Scripts

<details>
<summary><strong>record_angles.py</strong></summary>

Records a sequence of joint angle waypoints for the ORCA Hand. The user is prompted to press Enter to capture each waypoint. The recorded sequence is saved to a YAML file. This script uses a hardcoded model path.

<strong>Args:<strong>
- None (prompts for filename internally)

<strong>Example:<strong>
```bash
python scripts/record_angles.py
# Then enter the desired filename when prompted.
```
</details>

<details>
<summary><strong>record_continuous.py</strong></summary>

Continuously records joint angles from the ORCA Hand at a specified frequency and for an optional duration. The data is saved to a YAML file.

<strong>Args:<strong>
- <strong>model_path<strong> (<strong>str<strong>) – Path to the OrcaHand model.
- <strong>--frequency<strong> (<strong>float<strong>) – Sampling frequency in Hz (default: 50.0).
- <strong>--duration<strong> (<strong>float<strong>) – Recording duration in seconds (optional, records indefinitely if not set).
- <strong>--output_dir<strong> (<strong>str<strong>) – Directory to save the output file (default: ".").

<strong>Example:<strong>
```bash
python scripts/record_continuous.py /path/to/orcahand_v1_right --frequency 100 --duration 10 --output_dir ./replay_sequences
# Then enter a prefix for the filename when prompted.
```
</details>

<details>
<summary><strong>replay_angles.py</strong></summary>

Replays a recorded sequence of hand movements (waypoints) from a YAML file. It interpolates between waypoints for smooth motion.

<strong>Args:<strong>
- <strong>model_path<strong> (<strong>str<strong>) – Path to the orcahand model folder.
- <strong>--step_time<strong> (<strong>float<strong>) – Timestep for interpolation (default: 0.02).

<strong>Example:<strong>
```bash
python scripts/replay_angles.py /path/to/orcahand_v1_right --step_time 0.01
# Then enter the filename of the replay sequence when prompted.
```
</details>

<details>
<summary><strong>replay_continuous.py</strong></summary>

Replays continuously recorded hand joint movements from a YAML file. It attempts to match the original sampling frequency.

<strong>Args:<strong>
- <strong>model_path<strong> (<strong>str<strong>) – Path to the OrcaHand model folder.
- <strong>replay_file<strong> (<strong>str<strong>) – YAML file containing the recorded angles and metadata.

<strong>Example:<strong>
```bash
python scripts/replay_continuous.py /path/to/orcahand_v1_right ./replay_sequences/continuous_angles_YYYYMMDD_HHMMSS.yaml
```
</details>

### UI Control Scripts

<details>
<summary><strong>slider.py</strong></summary>

Provides a Tkinter-based GUI with sliders to control each joint of the ORCA Hand individually. It allows enabling/disabling torque and displays current joint values. This script uses a hardcoded model path ("models").

<strong>Args:<strong>
- None

<strong>Example:<strong>
```bash
python scripts/slider.py
```
</details>

<details>
<summary><strong>slider_joint.py</strong></summary>

Provides a Tkinter-based GUI with sliders to control each joint of the ORCA Hand. Similar to `slider.py` but takes the hand model path as an argument.

<strong>Args:<strong>
- <strong>hand_path<strong> (<strong>str<strong>) – Path to the hand model directory.

<strong>Example:<strong>
```bash
python scripts/slider_joint.py /path/to/orcahand_v1_right
```
</details>

<details>
<summary><strong>slider_motor.py</strong></summary>

Provides a Tkinter-based GUI with sliders to control each motor of the ORCA Hand individually. This allows for direct motor position control rather than joint-level control.

<strong>Args:<strong>
- <strong>hand_path<strong> (<strong>str<strong>) – Path to the hand model directory.

<strong>Example:<strong>
```bash
python scripts/slider_motor.py /path/to/orcahand_v1_right
```
</details>

### Miscellaneous Scripts

<details>
<summary><strong>tension.py</strong></summary>

Enables torque on the ORCA Hand and holds the current position, effectively creating tension in the tendons. The script runs until interrupted (Ctrl+C).

<strong>Args:<strong>
- <strong>model_path<strong> (<strong>str<strong>) – Path to the orcahand model folder.

<strong>Example:<strong>
```bash
python scripts/tension.py /path/to/orcahand_v1_left
```
</details>

<details>
<summary><strong>test.py</strong></summary>

A test script that connects to the ORCA Hand, enables torque, sets a specific pose for the index finger's MCP joint and middle finger's PIP joint, waits for 2 seconds, disables torque, and disconnects. This script appears to use a default `OrcaHand()` initialization, which might rely on a default model path or configuration.

<strong>Args:<strong>
- None

<strong>Example:<strong>
```bash
python scripts/test.py
```
</details>
