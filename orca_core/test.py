from orca_core import OrcaHand

if __name__ == "__main__":
    # Example usage:
    hand = OrcaHand()
    status = hand.connect()
    hand.enable_torque()
    hand.calibrate()

    hand.disable_torque()
    hand.disconnect()