from orca_core.hardware.dynamixel_client import DynamixelClient
import time

dxl_client = DynamixelClient([2], '/dev/tty.usbserial-FT9MISJT', 3000000)
dxl_client.connect()

# Set operating mode to current based position
dxl_client.set_operating_mode([2], 5)

# Enable torque
dxl_client.set_torque_enabled([2], True)

while True:
    pos = dxl_client.read_pos_vel_cur()[0]
    new_pos = pos + 0.1
    dxl_client.write_desired_pos([2], new_pos)
    print(f"Current Position: {pos}, Target Position: {new_pos}")
    time.sleep(0.5)
