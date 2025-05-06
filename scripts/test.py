from orca_core import OrcaHand, Retargeter

hand = OrcaHand('/Users/ccc/dev/orca/orca_core/orca_core/models/orcahand_v1')
status = hand.connect()
print(status)

while True:
    hand.disable_torque()
    position = hand.get_motor_pos()[10]
    print(f"Motor 11 position: {position}")