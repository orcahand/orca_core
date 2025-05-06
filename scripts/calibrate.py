from orca_core import OrcaHand, Retargeter

hand = OrcaHand('/Users/ccc/dev/orca/orca_core/orca_core/models/orcahand_v1')
status = hand.connect()
print(status)

hand.calibrate()
