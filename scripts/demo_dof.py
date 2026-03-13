import time
import numpy as np
import argparse
from orca_core import OrcaHand

# Joint order for the demo
JOINT_ORDER = [
    "thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip",
    "index_abd", "index_mcp", "index_pip",
    "middle_abd", "middle_mcp", "middle_pip",
    "ring_abd", "ring_mcp", "ring_pip",
    "pinky_abd", "pinky_mcp", "pinky_pip",
    "wrist",
]

CYCLES = 3
CYCLE_TIME = 1.0  # seconds per flex/extend cycle
STEP_TIME = 0.02
ROM_SCALE = 1.0   # use full ROM
WRIST_SPEED_FACTOR = 10.0  # wrist moves this many times slower


def ease_in_out(t):
    return 0.5 * (1 - np.cos(np.pi * t))


def smooth_move(hand, joint, start, end, duration, step_time):
    """Smoothly move a single joint from start to end."""
    n_steps = max(int(duration / step_time), 1)
    start_time = time.time()
    for s in range(n_steps + 1):
        alpha = ease_in_out(s / n_steps)
        pos = start + alpha * (end - start)
        hand.set_joint_pos({joint: pos})
        target = start_time + s * step_time
        now = time.time()
        if now < target:
            time.sleep(target - now)


def actuate_joint(hand, joint, rom_min, rom_max, neutral, cycles, cycle_time, step_time):
    """Smoothly move from neutral, flex/extend for N cycles, return to neutral."""
    half_time = cycle_time / 2

    # Smooth start: neutral → rom_min
    smooth_move(hand, joint, neutral, rom_min, half_time, step_time)

    for cycle in range(cycles):
        # rom_min → rom_max
        smooth_move(hand, joint, rom_min, rom_max, half_time, step_time)
        # rom_max → rom_min
        smooth_move(hand, joint, rom_max, rom_min, half_time, step_time)

    # Smooth return: rom_min → neutral
    smooth_move(hand, joint, rom_min, neutral, half_time, step_time)


def main():
    parser = argparse.ArgumentParser(description="Demo all degrees of freedom of the ORCA Hand.")
    parser.add_argument("model_path", type=str, nargs="?", default=None,
                        help="Path to the orcahand model folder")
    parser.add_argument("--cycles", type=int, default=CYCLES,
                        help=f"Number of flex/extend cycles per joint (default: {CYCLES})")
    parser.add_argument("--cycle_time", type=float, default=CYCLE_TIME,
                        help=f"Seconds per flex/extend cycle (default: {CYCLE_TIME})")
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    hand.init_joints()
    time.sleep(1)

    joint_roms = hand.joint_roms_dict
    available = list(hand.joint_ids)
    demo_joints = [j for j in JOINT_ORDER if j in available]

    print(f"Starting DOF demo: {len(demo_joints)} joints, {args.cycles} cycles each\n")

    try:
        neutral_pos = hand.get_joint_pos(as_list=False)

        for joint in demo_joints:
            rom_min, rom_max = joint_roms[joint]
            mid = (rom_min + rom_max) / 2
            half_range = (rom_max - rom_min) / 2 * ROM_SCALE
            scaled_min = mid - half_range
            scaled_max = mid + half_range
            cycle_time = args.cycle_time * WRIST_SPEED_FACTOR if joint.startswith("wrist") else args.cycle_time
            neutral = neutral_pos.get(joint, mid)
            print(f"  {joint:16s}  ROM: [{scaled_min:.0f}, {scaled_max:.0f}]")
            actuate_joint(hand, joint, scaled_min, scaled_max, neutral, args.cycles, cycle_time, STEP_TIME)
            time.sleep(0.2)

        print("\nDOF demo complete.")

    except KeyboardInterrupt:
        print("\nDemo interrupted.")

    finally:
        hand.disable_torque()
        print("Torque disabled.")


if __name__ == "__main__":
    main()
