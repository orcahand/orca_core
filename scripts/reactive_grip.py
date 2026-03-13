"""Reactive grip: sense contact and progressively close fingers around the object.

Uses resultant force feedback to detect which fingers are touching something,
then incrementally flexes the relevant joints to increase grip strength.

Usage:
    # Mock mode (no hardware)
    python scripts/reactive_grip.py --mock

    # Real hardware
    python scripts/reactive_grip.py orca_core/models/orcahand-touch
"""

import argparse
import time
import math
import signal
import sys

# Finger -> joints that flex to grip, ordered by priority.
# mcp = primary curl, pip = secondary curl, abd = squeeze inward
FINGER_GRIP_JOINTS = {
    "thumb": ["thumb_mcp", "thumb_dip", "thumb_abd"],
    "index": ["index_mcp", "index_pip", "index_abd"],
    "middle": ["middle_mcp", "middle_pip", "middle_abd"],
    "ring": ["ring_mcp", "ring_pip", "ring_abd"],
    "pinky": ["pinky_mcp", "pinky_pip", "pinky_abd"],
}

# How much to increment each joint per control step (degrees)
GRIP_STEP_DEG = 1.5

# Force thresholds (Newtons)
CONTACT_THRESHOLD = 0.5   # minimum force to count as contact
TARGET_FORCE = 3.0        # desired grip force per finger
MAX_FORCE = 6.0           # back off above this

# Control loop rate
LOOP_HZ = 30


def build_sensor_client(args):
    if args.mock:
        from orca_core.hardware.sensing.mock_sensor_client import MockSensorClient
        sensor = MockSensorClient(
            connected_sensors=["thumb", "index", "middle", "ring", "pinky"],
        )
        sensor.set_noise_level(0.05)
        return sensor

    from orca_core.hardware.sensing.sensor_client import SensorClient
    sensor_port = args.sensor_port or "/dev/ttyACM0"
    finger_map = {"thumb": 1, "index": 3, "middle": 0, "ring": 2, "pinky": 4}
    return SensorClient(port=sensor_port, finger_to_sensor_id=finger_map)


def build_hand(args):
    """Return (hand, joint_roms) or a mock equivalent."""
    if args.mock:
        return None, _mock_joint_roms()

    from orca_core import OrcaHand
    hand = OrcaHand(args.model_path)
    hand.init_joints()
    hand.set_neutral_position(num_steps=50, step_size=0.005)
    return hand, dict(hand.joint_roms_dict)


def _mock_joint_roms():
    """Minimal ROMs matching the orcahand-touch config."""
    return {
        "thumb_mcp": [-25, 100], "thumb_dip": [-15, 107], "thumb_abd": [-18, 55],
        "thumb_cmc": [-45, 33],
        "index_mcp": [-25, 100], "index_pip": [-15, 107], "index_abd": [-30, 25],
        "middle_mcp": [-25, 100], "middle_pip": [-15, 107], "middle_abd": [-27, 27],
        "ring_mcp": [-25, 100], "ring_pip": [-15, 107], "ring_abd": [-27, 27],
        "pinky_mcp": [-25, 100], "pinky_pip": [-15, 107], "pinky_abd": [-30, 30],
        "wrist": [-65, 35],
    }


def compute_grip_deltas(forces, joint_positions, joint_roms):
    """For each finger with contact, compute joint increments to tighten grip.

    Strategy:
    - Below CONTACT_THRESHOLD: no action (finger not touching).
    - Between CONTACT and TARGET: flex proportionally (bigger gap = bigger step).
    - Above TARGET: hold (no increment).
    - Above MAX_FORCE: back off (negative increment).

    Returns dict of {joint_name: delta_degrees}.
    """
    deltas = {}

    for finger, joints in FINGER_GRIP_JOINTS.items():
        if finger not in forces:
            continue

        fx, fy, fz = forces[finger]
        mag = math.sqrt(fx * fx + fy * fy + fz * fz)

        if mag < CONTACT_THRESHOLD:
            continue

        if mag > MAX_FORCE:
            # Back off: extend slightly
            gain = -0.5
        elif mag > TARGET_FORCE:
            # At target: hold
            continue
        else:
            # Ramp: scale 0..1 based on how far from target
            gap = (TARGET_FORCE - mag) / (TARGET_FORCE - CONTACT_THRESHOLD)
            gain = min(gap, 1.0)

        for joint in joints:
            if joint not in joint_roms:
                continue
            rom_min, rom_max = joint_roms[joint]
            current = joint_positions.get(joint, 0.0)

            # For abduction joints, squeeze toward 0 (neutral)
            if "abd" in joint:
                step = GRIP_STEP_DEG * gain * 0.5
                # Move toward 0
                if current > 0:
                    step = -step
            else:
                # Flex = move toward rom_max
                step = GRIP_STEP_DEG * gain

            new_pos = current + step
            new_pos = max(rom_min, min(rom_max, new_pos))
            deltas[joint] = new_pos - current

    return deltas


class MockHandState:
    """Tracks joint positions when running without real hardware."""

    def __init__(self, joint_roms):
        # Start with fingers slightly flexed so mock contact triggers sooner
        self.positions = {joint: 0.0 for joint in joint_roms}
        for finger in FINGER_GRIP_JOINTS:
            mcp = FINGER_GRIP_JOINTS[finger][0]
            if mcp in self.positions:
                self.positions[mcp] = 25.0
        self.joint_roms = joint_roms

    def get_joint_pos(self, as_list=False):
        if as_list:
            return list(self.positions.values())
        return dict(self.positions)

    def set_joint_pos(self, pos_dict, **kwargs):
        for joint, pos in pos_dict.items():
            if joint in self.joint_roms:
                rom_min, rom_max = self.joint_roms[joint]
                self.positions[joint] = max(rom_min, min(rom_max, pos))


def simulate_contact_forces(mock_sensor, joint_positions, step):
    """In mock mode, simulate increasing force as fingers flex.

    The more a finger's mcp is flexed, the more force we report,
    as if the finger is pressing against an object starting around 30 deg.
    """
    forces = {}
    for finger, joints in FINGER_GRIP_JOINTS.items():
        mcp_joint = joints[0]  # primary flexion joint
        pos = joint_positions.get(mcp_joint, 0.0)

        # Object contact starts at ~20 degrees of flexion
        if pos > 20:
            penetration = (pos - 20) / 80  # 0..1 over 20..100 deg range
            fz = penetration * 5.0  # up to 5N
            fx = penetration * 0.3
            fy = penetration * 0.2
        else:
            fz, fx, fy = 0.0, 0.0, 0.0

        forces[finger] = [fx, fy, fz]

    mock_sensor.set_mock_forces(forces)


def main():
    parser = argparse.ArgumentParser(description="Reactive grip controller")
    parser.add_argument("model_path", nargs="?", default="orca_core/models/orcahand-touch",
                        help="Path to hand model config")
    parser.add_argument("--mock", action="store_true", help="Run with mock hardware")
    parser.add_argument("--sensor-port", default=None, help="Sensor serial port")
    args = parser.parse_args()

    running = True

    def on_signal(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_signal)

    # Initialize
    sensor = build_sensor_client(args)
    hand, joint_roms = build_hand(args)
    mock_state = MockHandState(joint_roms) if args.mock else None

    sensor.connect()
    sensor.start_auto_stream(resultant=True, taxels=False)
    time.sleep(0.2)

    # Get starting positions
    if args.mock:
        joint_positions = mock_state.get_joint_pos()
    else:
        joint_positions = hand.get_joint_pos(as_list=False)

    print(f"Reactive grip started ({'mock' if args.mock else 'live'})")
    print(f"Contact threshold: {CONTACT_THRESHOLD} N, target: {TARGET_FORCE} N, max: {MAX_FORCE} N")
    print(f"Loop rate: {LOOP_HZ} Hz, step size: {GRIP_STEP_DEG} deg")
    print("Waiting for contact... (Ctrl+C to stop)\n")

    dt = 1.0 / LOOP_HZ
    step = 0
    last_print = 0

    while running:
        t0 = time.time()
        step += 1

        # In mock mode, simulate forces based on finger positions
        if args.mock:
            simulate_contact_forces(sensor, joint_positions, step)

        # Read latest sensor data
        forces, ts = sensor.get_auto_latest()
        if forces is None:
            time.sleep(dt)
            continue

        # Compute grip adjustments
        deltas = compute_grip_deltas(forces, joint_positions, joint_roms)

        # Apply deltas
        if deltas:
            new_positions = dict(joint_positions)
            for joint, delta in deltas.items():
                old = new_positions.get(joint, 0.0)
                new_pos = old + delta
                rom_min, rom_max = joint_roms[joint]
                new_positions[joint] = max(rom_min, min(rom_max, new_pos))

            if args.mock:
                mock_state.set_joint_pos(new_positions)
                joint_positions = mock_state.get_joint_pos()
            else:
                hand.set_joint_pos(new_positions)
                joint_positions = hand.get_joint_pos(as_list=False)

        # Print status periodically
        now = time.time()
        if now - last_print > 0.5:
            last_print = now
            active = []
            for finger in FINGER_GRIP_JOINTS:
                if finger in forces:
                    fx, fy, fz = forces[finger]
                    mag = math.sqrt(fx * fx + fy * fy + fz * fz)
                    if mag >= CONTACT_THRESHOLD:
                        mcp = FINGER_GRIP_JOINTS[finger][0]
                        active.append(f"{finger}: {mag:.1f}N @ {joint_positions.get(mcp, 0):.0f}deg")

            if active:
                print(f"[{step:>5}] " + " | ".join(active))

        elapsed = time.time() - t0
        if elapsed < dt:
            time.sleep(dt - elapsed)

    # Cleanup
    print("\nStopping...")
    sensor.stop_auto_stream()
    sensor.disconnect()

    if not args.mock and hand is not None:
        hand.set_neutral_position(num_steps=50, step_size=0.005)

    print("Done.")


if __name__ == "__main__":
    main()
