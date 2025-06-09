# Setting Up the Dynamixel Servos

This guide walks you through preparing your Dynamixel servos for use with the Orca hand control system.

## Prerequisites

1. **Install the Dynamixel Wizard**  
   Use the [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) to assign unique IDs and update motor settings.

2. **Ensure Python 3 is installed**  
   On Linux:
   ```sh
   sudo apt update
   sudo apt install python3
   ```

3. **Install the Dynamixel SDK**  
   Either:
   ```sh
   sudo apt install python3-pip
   sudo pip3 install dynamixel-sdk
   ```
   Or follow the instructions from the [Robotis GitHub page](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/master/python).

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
     - ID Range: **0–12**
   - Click **Scan**. Your motor should appear.

3. **Change the ID**
   - In the table, select the ID row (Address 7).
   - Assign a **unique ID** and label it physically on the motor.
   - Also update the **baud rate** to **3Mbps**.

4. **Repeat for All Motors**
   - Connect motors one-by-one (or daisy-chain after setting unique IDs).

---

## Verifying All Motors

After assigning IDs:

- Connect all motors in a daisy chain.
- Scan in the Dynamixel Wizard.
- Ensure all are detected correctly.

---

Now you're ready to put your servos in the hand tower!