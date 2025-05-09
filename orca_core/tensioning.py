import time
from core import OrcaHand
import argparse
import math 
import sys
import termios
import tty
import select
import time

def get_key( timeout=0.1):
    """
    Read a single keypress in Linux without blocking.
    Returns the key pressed or None if no key was pressed within the timeout.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if ready:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def toggle_torque( controller):
    """
    Toggle torque state on the controller each time the SPACE bar is pressed.
    Press 'q' to exit the loop.
    
    :param controller: An object with enable_torque() and disable_torque() methods.
    """
    torque_enabled = False
    print("Press SPACE to toggle torque; press 'q' to quit.")
    while True:
        key = get_key()
        if key:
            if key == ' ' or key == '\n' or key == 'a':
                if torque_enabled:
                    controller.disable_torque()
                    print("Torque disabled")
                    torque_enabled = False
                    time.sleep(0.1)
                else:
                    controller.enable_torque()
                    print("Torque enabled")
                    torque_enabled = True
                    time.sleep(0.1)

            elif key.lower() == 'q':
                print("Exiting toggle loop.")
                break
            if key == 'p':
                print("Pressing 'p' key")
                print(controller.get_motor_pos())
        time.sleep(0.005)


if __name__ == "__main__":
    # Create a HandController instance without calibration
    hand_ctrl = OrcaHand()
    hand_ctrl.connect()
    toggle_torque(hand_ctrl)