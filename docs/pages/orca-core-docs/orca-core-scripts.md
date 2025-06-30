# Scripts API Documentation

This document provides an overview of the available scripts in the `scripts` folder.

### Calibration Scripts

<details>
<summary><strong>calibrate.py</strong></summary>

Calibrates the ORCA Hand. This script reads the calibration sequence from the hand's configuration and applies it.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the orcahand model folder (e.g., `/path/to/orcahand_v1_right`). If not provided, the script will use the default model path.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/calibrate.py /path/to/orcahand_v1_right
```
</details>

<details>
<summary><strong>calibrate_manual.py</strong></summary>

Manually calibrates the ORCA Hand. This script guides the user through a manual calibration process.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the orcahand model folder (e.g., `/path/to/orcahand_v1_right`). If not provided, the script will use the default model path.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/calibrate_manual.py /path/to/orcahand_v1_right
```
</details>

### Motor and Joint Check Scripts

<details>
<summary><strong>check_motor.py</strong></summary>

Checks a specific motor by setting its operating mode and enabling torque. It then incrementally changes the motor's target position and prints the current and target positions. This script is useful for testing individual motor functionality.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>--port</strong> (<strong>str</strong>, optional): The serial port for the Dynamixel client (default: "/dev/tty.usbserial-FT9MISJT").</li><br>
    <li><strong>--baudrate</strong> (<strong>int</strong>, optional): The baud rate for the Dynamixel client (default: 3000000).</li><br>
    <li><strong>--motor_id</strong> (<strong>int</strong>, optional): The ID of the motor to check (default: 2).</li><br>
    <li><strong>--wrist</strong> (<strong>action</strong>, optional): If set, configures the motor for wrist operation (position control mode 3). Recommended for motor IDs 0 or 17.</li><br>
    <li><strong>--reverse</strong> (<strong>action</strong>, optional): If set, incrementally decreases the motor position; otherwise, increases it.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/check_motor.py --motor_id 5 --port /dev/ttyUSB0
```
</details>

### Demonstration Scripts

<details>
<summary><strong>main_demo.py</strong></summary>

Runs a demonstration of the ORCA Hand, making the fingers perform a wave-like motion. It initializes the hand, defines joint ranges, and then continuously updates joint positions to create the animation.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the orcahand model folder (e.g., `/path/to/orcahand_v1_right`). If not provided, the script will use the default model path.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/main_demo.py
```
</details>

<details>
<summary><strong>main_demo_abduction.py</strong></summary>

Runs a demonstration of the ORCA Hand, similar to `main_demo.py`, but with a focus on abduction movements. It initializes the hand, defines joint ranges, and then continuously updates joint positions.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the orcahand model folder (e.g., `/path/to/orcahand_v1_right`). If not provided, the script will use the default model path.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/main_demo_abduction.py
```
</details>

### Position Control Scripts

<details>
<summary><strong>neutral.py</strong></summary>

Moves the ORCA Hand to its neutral (home) position. It connects to the hand, enables torque, sets the neutral position, and then disables torque and disconnects.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the orcahand model folder (e.g., `/path/to/orcahand_v1_right`). If not provided, the script will use the default model path.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/neutral.py /path/to/orcahand_v1_right
```
</details>

<details>
<summary><strong>zero.py</strong></summary>

Moves all joints of the ORCA Hand to the zero position. It connects to the hand, enables torque, sets all joint positions to 0, waits for stabilization, then disables torque and disconnects.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the orcahand model folder (e.g., `/path/to/orcahand_v1_right`). If not provided, the script will use the default model path.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/zero.py /path/to/orcahand_v1_right
```
</details>

### Recording and Replay Scripts

<details>
<summary><strong>record_angles.py</strong></summary>

Records a sequence of joint angle waypoints for the ORCA Hand. The user is prompted to press Enter to capture each waypoint. The recorded sequence is saved to a YAML file in the `replay_sequences` directory (or a custom directory).

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the orcahand model folder (e.g., `/path/to/orcahand_v1_left`). If not provided, the script uses the default model path.</li><br>
    <li><strong>--output_dir</strong> (<strong>str</strong>, optional): Directory to save the replay sequence. Defaults to `replay_sequences/` at the project root.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/record_angles.py /path/to/orcahand_v1_left --output_dir my_recordings
# Then enter a filename prefix when prompted.
```
</details>

<details>
<summary><strong>record_continuous.py</strong></summary>

Continuously records joint angles from the ORCA Hand at a specified frequency and for an optional duration. The data is saved to a YAML file in the `replay_sequences` directory (or a custom directory).

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the orcahand model folder (e.g., `/path/to/orcahand_v1_left`). If not provided, uses the default model path.</li><br>
    <li><strong>--frequency</strong> (<strong>float</strong>, optional): Sampling frequency in Hz (default: 50.0).</li><br>
    <li><strong>--duration</strong> (<strong>float</strong>, optional): Recording duration in seconds. Records indefinitely if not set.</li><br>
    <li><strong>--output_dir</strong> (<strong>str</strong>, optional): Directory to save the output file. Defaults to `replay_sequences/` at the project root.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/record_continuous.py /path/to/orcahand_v1_right --frequency 100 --duration 10 --output_dir ./custom_replays
# Then enter a filename prefix when prompted.
```
</details>

<details>
<summary><strong>replay_angles.py</strong></summary>

Replays a recorded sequence of hand movements (waypoints) from a YAML file. It interpolates between waypoints for smooth motion and loops the sequence indefinitely.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the orcahand model folder (e.g., `/path/to/orcahand_v1`). If not provided, uses the default model path.</li><br>
    <li><strong>--step_time</strong> (<strong>float</strong>, optional): Timestep for interpolation (default: 0.02 seconds).</li><br>
    <li><strong>--replay_file</strong> (<strong>str</strong>, required): Path to the replay file. Can be an absolute/relative path, or a plain filename (searched in `project_root/replay_sequences/`).</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/replay_angles.py /path/to/orcahand_v1_right --replay_file my_capture_replay_sequence_TIMESTAMP.yaml --step_time 0.01
```
</details>

<details>
<summary><strong>replay_continuous.py</strong></summary>

Replays continuously recorded hand joint movements from a YAML file. It attempts to match the original sampling frequency.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the OrcaHand model folder (e.g., `/path/to/orcahand_v1_left`). If not provided, uses the default model path.</li><br>
    <li><strong>--replay_file</strong> (<strong>str</strong>, required): Path to the replay file. Can be an absolute/relative path, or a plain filename (searched in `project_root/replay_sequences/`).</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/replay_continuous.py /path/to/orcahand_v1_right --replay_file continuous_angles_YYYYMMDD_HHMMSS.yaml
```
</details>

### UI Control Scripts

<details>
<summary><strong>slider_joint.py</strong></summary>

Provides a Tkinter-based GUI with sliders to control each joint of the ORCA Hand individually. It allows enabling/disabling torque and displays current joint values.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the hand model directory (e.g., `/path/to/orcahand_v1_right`). If not provided, uses the default model path.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/slider_joint.py /path/to/orcahand_v1_right
```
</details>

<details>
<summary><strong>slider_motor.py</strong></summary>

Provides a Tkinter-based GUI with sliders to control each motor of the ORCA Hand individually. This allows for direct motor position control rather than joint-level control. Sliders have a small range for precise adjustments around the current motor position.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the hand model directory (e.g., `/path/to/orcahand_v1_right`). If not provided, uses the default model path.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/slider_motor.py /path/to/orcahand_v1_right
```
</details>

### Miscellaneous Scripts

<details>
<summary><strong>tension.py</strong></summary>

Enables torque on the ORCA Hand servos and holds the current position, effectively locking the bottom spools in order to be able to rachet the top spools. The script runs until interrupted (Ctrl+C).

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the orcahand model folder (e.g., `/path/to/orcahand_v1_left`). If not provided, uses the default model path.</li><br>
    <li><strong>--move_motors</strong>: Move all motors CCW with a the calibration current specified in `config.py` and then enable torque in order to hold the servos in position for tensioning.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/tension.py /path/to/orcahand_v1_left
```
</details>

<details>
<summary><strong>test.py</strong></summary>

A test script that connects to the ORCA Hand, enables torque, sets a specific pose for a few joints (index_mcp, middle_pip, pinky_abd), waits for 2 seconds, disables torque, and disconnects.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>model_path</strong> (<strong>str</strong>, optional): Path to the hand model directory (e.g., `/path/to/orcahand_v1_right`). If not provided, uses the default model path.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/test.py
```
</details>
