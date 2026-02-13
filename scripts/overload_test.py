"""Flex multiple fingers and hold to trigger overload and test reactive recovery.

Uses position control (no current limit) so stalling triggers overload quickly.
Hold the fingers still while they flex to trigger the overload.
"""
import os
import time
import logging
import numpy as np
from orca_core import OrcaHand

def msg(text):
    os.write(2, (text + '\n').encode())

logging.basicConfig(level=logging.WARNING, format='%(asctime)s %(levelname)s %(message)s')

# index_mcp, index_pip, middle_mcp, middle_pip
TEST_MOTORS = [15, 16, 8, 9]
FLEX_AMOUNT_RAD = np.deg2rad(60)
HOLD_DURATION = 15.0

hand = OrcaHand()
hand.connect()
hand.enable_torque()
hand.set_control_mode('position', motor_ids=TEST_MOTORS)

start_positions = hand.get_motor_pos(as_dict=True)

msg("\n>>> HOLD INDEX + MIDDLE FINGERS to trigger overload! <<<\n")
msg(f"Motors under test: {TEST_MOTORS} (index_mcp, index_pip, middle_mcp, middle_pip)")
msg("Flexing...")
desired = {mid: start_positions[mid] - FLEX_AMOUNT_RAD for mid in TEST_MOTORS}
hand._set_motor_pos(desired)

msg(f"Holding flexed for {HOLD_DURATION}s...")
start = time.time()
while time.time() - start < HOLD_DURATION:
    elapsed = time.time() - start
    if int(elapsed) > int(elapsed - 0.1) and int(elapsed) % 2 == 0:
        msg(f"  [{elapsed:.0f}s]")
    hand._set_motor_pos(desired)
    time.sleep(0.01)

msg("\nReturning to start...")
hand._set_motor_pos(start_positions)
time.sleep(1)
msg("Done. Disabling torque.")
hand.disable_torque()
hand.disconnect()
