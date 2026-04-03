# Setting Up the Dynamixel Servos

This guide walks you through preparing a Dynamixel motor chain for the current refactored `orca_core` workflow.

## Prerequisites

### **Install the Dynamixel Wizard**
   Use the [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) to assign unique IDs and update motor settings.

---

## Assigning Unique Motor IDs

Before using the motors, each one must have a **unique ID** that matches the model config you plan to use.

### Steps:

1. **Connect Hardware**
   - Connect the U2D2 to your PC using USB.
   - Use a 3-pin cable to connect U2D2 to the power and communication board you are using.
   - Power the chain.
   - Connect one motor at a time while assigning IDs.

2. **Launch Dynamixel Wizard**
   - Open the Wizard and go to **Options**.
   - Set:
     - Protocol: **2.0**
     - Baudrates that cover your expected setup, commonly **57600** and **3Mbps**
     - ID range that covers your hand assembly
   - Click **Scan**.

3. **Change the ID**
   - Assign a **unique ID** to each motor.
   - Label the motor physically so it can be matched against `joint_to_motor_map`.
   - Set the baudrate you want the chain to use.

4. **Repeat for All Motors**
   - Once every motor has a unique ID, connect them in the final daisy chain and verify they can all be discovered together.

---

## Verifying All Motors

After assigning IDs:

- Connect all motors in a daisy chain.
- Scan in the Dynamixel Wizard.
- Ensure all expected IDs are detected.
- Make sure the chosen baudrate matches the value in `config.yaml`.

---

## Matching the Refactored Config

The motor setup only becomes useful once it matches the hand config used by `OrcaHand`.

In particular, verify:

- `motor_ids` contains the IDs you assigned
- `joint_to_motor_map` matches the actual tendon routing and sign convention
- `baudrate` matches the chain
- `motor_type` is set correctly

The refactored loader normalizes signed motor IDs into:

- absolute motor IDs for the hardware mapping
- a separate inversion flag per joint

That means the sign in `joint_to_motor_map` still matters, but it is now treated as configuration metadata rather than as the raw hardware ID.

---

## USB Latency (Linux)

FTDI USB-serial adapters (like the U2D2) default to 16ms latency on Linux, which limits motor control to roughly 30 Hz. The current `DynamixelClient` and `FeetechClient` automatically try to enable low latency mode when connecting.

### Troubleshooting

If you experience slow communication, verify the latency setting:

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# Should output: 1 after connecting with the motor client
```

If low latency mode is not being set automatically, you can create a udev rule:

```bash
sudo tee /etc/udev/rules.d/99-usb-serial-low-latency.rules <<< 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"'
sudo udevadm control --reload-rules
```

---

Once the chain is visible and the config matches the hardware, you can move on to the `config.yaml` page and then the calibration workflow.
