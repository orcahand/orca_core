# Setting Up the Dynamixel Servos

This guide walks you through preparing your Dynamixel servos for use with the Orca hand control system.

## Prerequisites

### **Install the Dynamixel Wizard**  
   Use the [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) to assign unique IDs and update motor settings.

---

## Assigning Unique Motor IDs

Before using the motors, each one must have a **unique ID**.

### Steps:

1. **Connect Hardware**
   - Connect the U2D2 to your PC using USB.
   - Use a 3-pin cable to connect U2D2 to the Power Hub Board (PHB).
   - Power the PHB with a 12V adapter.
   - Connect one motor at a time to the PHB.
   - Switch on power (red LED should light up).

2. **Launch Dynamixel Wizard**
   - Open the Wizard and go to **Options**.
   - Set:
     - Protocol: **2.0**
     - Baudrate: **57600** and **3Mbps**
     - ID Range: **0–17**
   - Click **Scan**. Your motor should appear.

3. **Change the ID**
   - In the table, select the ID row (Address 7).
   - Assign a **unique ID** and label it physically on the motor.
   - Also update the **baudrate** to **3Mbps**.

4. **Repeat for All Motors**
   - Connect motors one-by-one (daisy-chain after setting unique IDs).

---

## Verifying All Motors

After assigning IDs:

- Connect all motors in a daisy chain.
- Scan in the Dynamixel Wizard.
- Ensure all are detected correctly.

---

## USB Latency (Linux)

FTDI USB-serial adapters (like the U2D2) default to 16ms latency on Linux, which limits motor control to ~30 Hz. The `DynamixelClient` and `FeetechClient` automatically enable low latency mode when connecting, achieving ~500 Hz control rates without any manual configuration.

### Troubleshooting

If you experience slow communication, verify the latency setting:

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# Should output: 1 (after connecting with DynamixelClient)
```

If low latency mode isn't being set automatically, you can create a udev rule:

```bash
# Create the udev rule
sudo tee /etc/udev/rules.d/99-usb-serial-low-latency.rules <<< 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"'

# Reload rules and replug the adapter
sudo udevadm control --reload-rules
```

---

Now you're ready to put your servos in the hand tower!