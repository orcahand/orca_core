from orca_core import OrcaHand, Retargeter

hand = OrcaHand('/Users/ccc/dev/orca/orca_core/orca_core/models/orcahand_v1_berkeley')
status = hand.connect()
print(status)

joints = hand.joint_ids

hand.set_joint_pos({joint: 0.0 for joint in joints})
