# Setting Up Dynamixel Motors

This page covers the hardware preparation required before `orca_core` can talk to a Dynamixel-based ORCA Hand.

## Goal

Before running the Python package, make sure:

- every motor has a unique ID
- every motor is on the expected baudrate
- the full chain is visible from the host machine

## Recommended tool

Use [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) to inspect the bus, assign IDs, and verify communication.

## Assign unique IDs

1. Connect the USB adapter and power hardware.
2. Connect one motor at a time.
3. Scan in Dynamixel Wizard.
4. Assign a unique ID and label the motor physically.
5. Set the target baudrate for the whole chain.
6. Repeat until all motors are configured.

After that, connect the motors in the full daisy chain and verify they all appear on the bus together.

## Match the software config

Once hardware IDs are assigned, update the ORCA model config so that:

- `motor_ids` matches the chain
- `joint_to_motor_map` matches the physical routing
- `baudrate` matches the configured motor baudrate

If these values do not match the actual chain, the higher-level hand API will not behave correctly.

## Linux latency optimization

On Linux, FTDI-based USB serial adapters often default to a high latency timer. The current `DynamixelClient` and `FeetechClient` attempt to enable low-latency mode automatically on connect.

You can verify the adapter latency with:

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

After the client enables low latency mode, this should typically read `1`.

If your platform or adapter does not persist that setting, you can install a udev rule:

```bash
sudo tee /etc/udev/rules.d/99-usb-serial-low-latency.rules <<< 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"'
sudo udevadm control --reload-rules
```

## After the hardware scan passes

Once all motors are visible and the config matches the chain, continue with:

1. config review
2. tensioning
3. calibration
4. neutral positioning
