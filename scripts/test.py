from orca_core import OrcaHand

hand = OrcaHand()

status = hand.connect()
print(status)
if not status[0]:
    print("Failed to connect to the hand.")
    exit(1)
    
hand.enable_torque()

joint_dict = {
    "index_mcp": 90,
    "middle_pip": 30,
}

hand.set_joint_pos(joint_dict, num_steps = 25, step_size = 0.001)

time.sleep(2)
hand.disable_torque()

hand.disconnect()
import time