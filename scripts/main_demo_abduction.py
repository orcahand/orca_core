from orca_core import OrcaHand
import time
import numpy as np
import argparse # Added import

def main(): # Added main function
    parser = argparse.ArgumentParser(
        description="Run a demo of the ORCA Hand with abduction movements. Specify the path to the orcahand model folder."
    )
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1)"
    )
    args = parser.parse_args()

    # Initialize the hand
    hand = OrcaHand(args.model_path) # Replaced hardcoded path with args.model_path
    status = hand.connect()
    print(status)

    # Ensure the hand is connected
    if not status[0]:
        print("Failed to connect to the hand.")
        exit()

    # Joint ranges of motion (ROMs)
    joint_roms = {
        'thumb_mcp': [-50, 50],
        'thumb_abd': [-20, 42],
        'thumb_pip': [-12, 108],
        'thumb_dip': [-20, 112],
        'index_mcp': [-20, 95],
        'index_pip': [-20, 108],
        'index_abd': [-37, 37],
        'middle_mcp': [-20, 91],
        'middle_pip': [-20, 107],
        'ring_mcp': [-20, 91],
        'ring_pip': [-20, 107],
        'ring_abd': [-37, 37],
        'pinky_mcp': [-20, 98],
        'pinky_pip': [-20, 108],
        'pinky_abd': [-37, 37],
        'wrist': [-50, 30],
    }

    # Define the fingers and their joints
    fingers = [
        {'name': 'index', 'joints': ['index_mcp', 'index_pip']},
        {'name': 'middle', 'joints': ['middle_mcp', 'middle_pip']},
        {'name': 'ring', 'joints': ['ring_mcp', 'ring_pip']},
        {'name': 'pinky', 'joints': ['pinky_mcp', 'pinky_pip']},
    ]

    # Movement parameters
    period = 1.0  # Total time for one cycle (seconds)
    step_time = 0.005  # Time between updates (seconds)
    amplitude = 0.7  # Fraction of the ROM to use for finger movement
    thumb_amplitude = 0.25  # Fraction of the ROM to use for thumb movement
    phase_shift_factor = period / 20  # Phase shift between fingers (0 for no shift, period/4 for equal spacing)

    # Precompute the joint positions for each time step
    time_steps = np.arange(0, period, step_time)
    joint_positions = {finger['name']: [] for finger in fingers}
    thumb_positions = []

    for t in time_steps:
        # Compute positions for fingers
        for i, finger in enumerate(fingers):
            phase_shift = i * phase_shift_factor  # Apply the phase shift factor
            positions = {}
            for joint in finger['joints']:
                rom_min, rom_max = joint_roms[joint]
                center = (rom_min + rom_max) / 2
                range_ = (rom_max - rom_min) / 2
                positions[joint] = center + amplitude * range_ * np.sin(2 * np.pi * (t - phase_shift) / period)
            joint_positions[finger['name']].append(positions)

        # Compute positions for the thumb
        thumb_pos = {
            'thumb_mcp': (joint_roms['thumb_mcp'][0] + joint_roms['thumb_mcp'][1]) / 2
            + thumb_amplitude * (joint_roms['thumb_mcp'][1] - joint_roms['thumb_mcp'][0]) / 2
            * np.sin(2 * np.pi * t / period),
            'thumb_pip': (joint_roms['thumb_pip'][0] + joint_roms['thumb_pip'][1]) / 2
            + thumb_amplitude * (joint_roms['thumb_pip'][1] - joint_roms['thumb_pip'][0]) / 2
            * np.sin(2 * np.pi * t / period),
            'thumb_dip': (joint_roms['thumb_dip'][0] + joint_roms['thumb_dip'][1]) / 2
            + thumb_amplitude * (joint_roms['thumb_dip'][1] - joint_roms['thumb_dip'][0]) / 2
            * np.sin(2 * np.pi * t / period),
            'thumb_abd': 35,  # Constant position
            'wrist': -20,  # Constant position
            'pinky_abd': -20,  # Constant abduction
            'ring_abd': -10,   # Constant abduction
            'index_abd': 10, # Constant abduction
        }
        thumb_positions.append(thumb_pos)

    # Perform the movement in a loop
    try:
        while True:
            for t_idx, t in enumerate(time_steps):
                # Combine all joint positions for this time step
                current_positions = {}
                for finger in fingers:
                    current_positions.update(joint_positions[finger['name']][t_idx])
                current_positions.update(thumb_positions[t_idx])  # Add thumb, wrist, and abduction positions
                
                # Send the positions to the hand
                hand.set_joint_pos(current_positions)
                time.sleep(step_time)

    except KeyboardInterrupt:
        # Reset the hand to the neutral position on exit
        hand.set_joint_pos({joint: 0 for joint in hand.joint_ids})
        print("Demo stopped and hand reset.")

if __name__ == "__main__": # Added main execution block
    main()