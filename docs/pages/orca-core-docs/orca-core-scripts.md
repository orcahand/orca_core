# Scripts API Documentation

This document provides an overview of the available scripts in the `scripts` folder.

### Calibration Scripts

<details>
<summary><strong>calibrate.py</strong></summary>

Calibrates the ORCA Hand. This script reads the calibration sequence from the hand's configuration and applies it.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file (e.g., `/path/to/orcahand-right/config.yaml`). If not provided, the script will use the default config path.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/calibrate.py /path/to/orcahand-right/config.yaml
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

### Demo Examples (`examples/`)

<details>
<summary><strong>main_demo.py</strong></summary>

Runs a demonstration of the ORCA Hand, making the fingers perform a wave-like motion. It initializes the hand, defines joint ranges, and then continuously updates joint positions to create the animation.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file (e.g., `/path/to/orcahand-right/config.yaml`). If not provided, the script will use the default config path.</li>
</ul>

<strong>Example:</strong>
```bash
python examples/main_demo.py
```
</details>

<details>
<summary><strong>main_demo_abduction.py</strong></summary>

Runs a demonstration of the ORCA Hand, similar to `main_demo.py`, but with a focus on abduction movements. It initializes the hand, defines joint ranges, and then continuously updates joint positions.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file (e.g., `/path/to/orcahand-right/config.yaml`). If not provided, the script will use the default config path.</li>
</ul>

<strong>Example:</strong>
```bash
python examples/main_demo_abduction.py
```
</details>

### Position Control Scripts

<details>
<summary><strong>neutral.py</strong></summary>

Moves the ORCA Hand to its neutral (home) position. It connects to the hand, enables torque, sets the neutral position, and then disables torque and disconnects.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file (e.g., `/path/to/orcahand-right/config.yaml`). If not provided, the script will use the default config path.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/neutral.py /path/to/orcahand-right/config.yaml
```
</details>

<details>
<summary><strong>zero.py</strong></summary>

Moves all joints of the ORCA Hand to the zero position. It connects to the hand, enables torque, sets all joint positions to 0, waits for stabilization, then disables torque and disconnects.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file (e.g., `/path/to/orcahand-right/config.yaml`). If not provided, the script will use the default config path.</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/zero.py /path/to/orcahand-right/config.yaml
```
</details>

### Recording and Replay Examples (`examples/`)

<details>
<summary><strong>record_angles.py</strong></summary>

Records a sequence of joint angle waypoints for the ORCA Hand. The user is prompted to press Enter to capture each waypoint. The recorded sequence is saved to a YAML file in the `replay_sequences` directory (or a custom directory).

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file (e.g., `/path/to/orcahand-left/config.yaml`). If not provided, the script uses the default config path.</li><br>
    <li><strong>--output-dir</strong> (<strong>str</strong>, optional): Directory to save the replay sequence. Defaults to `replay_sequences/` in the current working directory.</li>
</ul>

<strong>Example:</strong>
```bash
python examples/record_angles.py /path/to/orcahand-left/config.yaml --output-dir my_recordings
# Then enter a filename prefix when prompted.
```
</details>

<details>
<summary><strong>record_continuous.py</strong></summary>

Continuously records joint angles from the ORCA Hand at a specified frequency and for an optional duration. The data is saved to a YAML file in the `replay_sequences` directory (or a custom directory).

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file (e.g., `/path/to/orcahand-left/config.yaml`). If not provided, uses the default config path.</li><br>
    <li><strong>--frequency</strong> (<strong>float</strong>, optional): Sampling frequency in Hz (default: 50.0).</li><br>
    <li><strong>--duration</strong> (<strong>float</strong>, optional): Recording duration in seconds. Records indefinitely if not set.</li><br>
    <li><strong>--output-dir</strong> (<strong>str</strong>, optional): Directory to save the output file. Defaults to `replay_sequences/` in the current working directory.</li>
</ul>

<strong>Example:</strong>
```bash
python examples/record_continuous.py /path/to/orcahand-right/config.yaml --frequency 100 --duration 10 --output-dir ./custom_replays
# Then enter a filename prefix when prompted.
```
</details>

<details>
<summary><strong>replay_angles.py</strong></summary>

Replays a recorded sequence of hand movements (waypoints) from a YAML file. It interpolates between waypoints for smooth motion and plays the sequence once (pass <code>--loop</code> to repeat indefinitely).

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file (e.g., `/path/to/orcahand-right/config.yaml`). If not provided, uses the default config path.</li><br>
    <li><strong>--step-time</strong> (<strong>float</strong>, optional): Timestep for interpolation (default: 0.02 seconds).</li><br>
    <li><strong>--replay-file</strong> (<strong>str</strong>, required): Path to the replay file. Can be an absolute/relative path, or a plain filename (searched in `replay_sequences/` under the current working directory).</li>
</ul>

<strong>Example:</strong>
```bash
python examples/replay_angles.py /path/to/orcahand-right/config.yaml --replay-file my_capture_replay_sequence_TIMESTAMP.yaml --step-time 0.01
```
</details>

<details>
<summary><strong>replay_continuous.py</strong></summary>

Replays continuously recorded hand joint movements from a YAML file. It attempts to match the original sampling frequency.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file (e.g., `/path/to/orcahand-left/config.yaml`). If not provided, uses the default config path.</li><br>
    <li><strong>--replay-file</strong> (<strong>str</strong>, required): Path to the replay file. Can be an absolute/relative path, or a plain filename (searched in `replay_sequences/` under the current working directory).</li>
</ul>

<strong>Example:</strong>
```bash
python examples/replay_continuous.py /path/to/orcahand-right/config.yaml --replay-file continuous_angles_YYYYMMDD_HHMMSS.yaml
```
</details>

### UI Control Scripts

<details>
<summary><strong>manual_control.py</strong></summary>

Provides a Tkinter-based GUI with sliders to drive the ORCA Hand from the PC. By default the sliders are in joint space, one per joint, with torque enable/disable. On a hand with joint encoders the sliders cover the encoder-backed joints and gain a live encoder readback plus a Kp / Ki / correction_max / max_current tuning panel.

With <code>--motor-space</code> the sliders are one-per-motor over a narrow window around each motor's startup position, for nudging a single tendon during bring-up. That mode opens the motor bus only.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file (e.g., `/path/to/orcahand-right/config.yaml`). If not provided, uses the default config path.</li><br>
    <li><strong>--motor-space</strong>: One slider per motor instead of per joint.</li><br>
    <li><strong>--fingers</strong> / <strong>--joints</strong>: Restrict the slider set (joint-feedback hands).</li><br>
    <li><strong>--Kp</strong> / <strong>--Ki</strong> / <strong>--correction-max-deg</strong> / <strong>--max-current</strong>: Initial loop tuning (joint-feedback hands).</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/manual_control.py /path/to/orcahand-right/config.yaml
python scripts/manual_control.py /path/to/orcahand-right/config.yaml --motor-space
```
</details>

### Miscellaneous Scripts

<details>
<summary><strong>tension.py</strong></summary>

Enables torque on the ORCA Hand servos and holds the current position, effectively locking the bottom spools in order to be able to rachet the top spools. By default it first runs a short preconditioning flexion/extension motion before holding. The script runs until interrupted (Ctrl+C).

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file (e.g., `/path/to/orcahand-left/config.yaml`). If not provided, uses the default config path.</li><br>
    <li><strong>--move-motors</strong> / <strong>--no-move-motors</strong>: Run or skip the preconditioning motion before holding tension (default: enabled).</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/tension.py /path/to/orcahand-left/config.yaml
python scripts/tension.py /path/to/orcahand-left/config.yaml --no-move-motors
```
</details>


<details>
<summary><strong>stress_test.py</strong></summary>

Cycles the hand between fixed OPEN and CLOSE poses while monitoring motor temperatures, aborting if any motor exceeds the safe operating temperature. Supports <code>--mock</code>.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand config file. If not provided, uses the default config path.</li><br>
    <li><strong>--num-steps</strong>: Interpolation steps per move.</li><br>
    <li><strong>--step-size</strong>: Sleep between interpolation steps in seconds.</li><br>
    <li><strong>--hold</strong>: Seconds to hold each pose after motion completes (default: 2).</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/stress_test.py /path/to/orcahand-right/config.yaml --hold 1
```
</details>

### Sensing Diagnostic Scripts

The two sensing scripts split by intent: `monitor_sensors.py` shows you the data, `check_sensors.py` tells you whether the sensors are healthy.

<details>
<summary><strong>monitor_sensors.py</strong></summary>

Live view of the hand's sensing link: joint-encoder angles for all 17 slots plus tactile forces, in a mode you pick with radio buttons (Off / Resultant / Taxels / Combined). Each stream shows its measured frame rate. Autodetects the connector-board serial port; pass <code>--port</code> to override.

<strong>Example:</strong>
```bash
python scripts/monitor_sensors.py
python scripts/monitor_sensors.py --port /dev/cu.usbmodemXXXX
```
</details>

<details>
<summary><strong>check_sensors.py</strong></summary>

Pass/fail health check. Reads the hand's config and runs the checks its declared hardware supports: joint-encoder stream health (frame integrity, rate, per-slot parity / angle-error / stuck-bus detection — non-interactive), and tactile sensor checks (enumeration, stream rate, per-finger press response, zeroing, stream lifecycle — interactive). A hand with both runs both, encoders first. Motors need not be powered.

<code>--port</code> skips the config and runs the encoder checks against a raw serial port, for bringing up a connector board before a hand config exists.

<br><strong>Args:</strong><br>
<ul>
    <li><strong>config_path</strong> (<strong>str</strong>, optional): Path to the hand's config.yaml.</li><br>
    <li><strong>--port</strong>: Raw serial port; runs encoder checks only and ignores the config.</li><br>
    <li><strong>--encoder-duration</strong>: Seconds to sample the encoder stream (default 10).</li>
</ul>

<strong>Example:</strong>
```bash
python scripts/check_sensors.py orca_core/models/v2/orcahand-touch-right/config.yaml
python scripts/check_sensors.py --port /dev/cu.usbmodemXXXX
```
</details>

<details>
<summary><strong>examples/taxel_frames.py</strong></summary>

Streams per-taxel forces with 3D taxel positions in a chosen coordinate frame (sensor, fingertip, palm, base, world). Runs without hardware via <code>--mock</code>.

<strong>Example:</strong>
```bash
python examples/taxel_frames.py --mock --frame base
```
</details>
