from orca_core import OrcaHand
import yaml
import numpy as np
import time
import os

hand = OrcaHand()
status = hand.connect()
hand.enable_torque()

def convert_to_builtin(obj):
    if isinstance(obj, dict):
        return {k: convert_to_builtin(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_to_builtin(i) for i in obj]
    elif isinstance(obj, np.generic):  # catches numpy scalars like np.float64
        return obj.item()
    return obj

os.makedirs("configurations", exist_ok=True)
# for configuration in ["thumbs_up", "italian", "peace", "spiderman"]:
for configuration in ["spiderman"]:
    hand.disable_torque()

    user_input = input("Please set the hand to the configuration: " + configuration + ". Then press Enter to save, s to skip or q to quit.")
    if user_input == 'q':
        break
    if user_input == 's':
        continue
    hand.enable_torque()
    time.sleep(1)

    hand_pos = hand.get_joint_pos()
    with open(f'configurations/{configuration}.yaml', 'w') as file:
        yaml.dump(convert_to_builtin(hand_pos), file)

    print(f"Saved {configuration} hand position to configurations/{configuration}.yaml")

hand.disable_torque()
hand.disconnect()
