from orca_core import OrcaHand, Retargeter

hand = OrcaHand('/Users/ccc/dev/orca/orca_core/orca_core/models/orcahand_v1_berkeley')
status = hand.connect()
print(status)

hand.enable_torque()

while True:
    pass