from orca_core import OrcaHand, Retargeter

hand = OrcaHand()
status = hand.connect()
print(status)
# hand.calibrate()

# Set the desired joint positions to 0
# hand.set_joint_pos({joint: 0 for joint in hand.joint_ids})

retargeter = Retargeter(hand)
