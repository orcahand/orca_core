from orca_core import OrcaHand, Retargeter
import time


hand = OrcaHand('/Users/ccc/dev/orca/orca_core/orca_core/models/orcahand_v1_berkeley')
status = hand.connect()
print(status)



while True:
    current_angles = hand.get_joint_pos()
    print(current_angles)
    time.sleep(0.1)  # Adjust the sleep time as needed

