# Dynamixel Latency Optimization

This guide explains the communication latency optimizations available for Dynamixel motors in `test_motor_latency.py` and how they can be applied to improve control loop rates.

## Overview

The default Dynamixel configuration introduces significant communication overhead, especially with multiple motors. With 17 motors on a single bus at 3M baud, a naive read-write cycle takes ~12ms (~85 Hz). The optimizations below reduce this to ~4.5ms (~224 Hz).

| Test (17 motors) | Default | Optimized | Improvement |
|---|---|---|---|
| Sync read | 11.1 ms | 2.3 ms | 4.8x |
| Fast Sync read | — | 2.0 ms | 5.5x |
| Sync write | 0.7 ms | 0.3 ms | 2.3x |
| Full-hand control loop | ~12 ms | ~4.5 ms | 2.6x |
| **Max control rate** | **~85 Hz** | **~224 Hz** | **2.6x** |

---

## Optimizations

### 1. Return Delay Time = 0 (Register 9)

**What:** Each Dynamixel motor has a configurable delay before sending its status response packet. The default is 250 (500 microseconds).

**Why it matters:** With 17 motors responding individually, the default adds 17 x 500us = 8.5ms of pure waiting time per read cycle.

**Setting:** Register address 9, 1 byte, EEPROM (persists across reboots). Each unit = 2 microseconds. Setting to 0 means the motor responds immediately.

**Platform:** Cross-platform (motor hardware register).

**Source:** [Dynamixel Protocol 2.0 - Control Table](https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#return-delay-time9)

### 2. Status Return Level = 1 (Register 68)

**What:** Controls when a motor sends status response packets:

- `0` = Never respond (except PING)
- `1` = Respond to READ instructions only
- `2` = Respond to all instructions (default)

**Why it matters:** With level 2, every write command generates 17 status response packets that the controller must receive and process. With level 1, write commands are fire-and-forget — the motor executes immediately without sending a response, saving the entire write response time.

**Trade-off:** You lose write error feedback. The motor still executes the command, but you won't know if there was a communication error on write. For real-time control loops where you read state every cycle anyway, this is acceptable.

**Important:** When Status Return Level = 1, single-motor writes must use `write4ByteTxOnly` (send-only) instead of `write4ByteTxRx` (send-and-wait). Using `TxRx` will cause the SDK to wait for a response that never comes, hitting a ~34ms timeout.

**Platform:** Cross-platform (motor hardware register).

**Source:** [Dynamixel Protocol 2.0 - Control Table](https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#status-return-level68)

### 3. Fast Sync Read (Protocol 2.0 Instruction 0x8A)

**What:** Standard Sync Read sends one instruction packet but receives N individual status packets (one per motor). Fast Sync Read receives a single combined status packet containing all motor data.

**Why it matters:** Eliminates N-1 packet turnarounds on the bus. Instead of 17 separate receive operations, there is only 1. This is the single biggest improvement for multi-motor reads.

**Supported motors:**

- X430/X540 series: firmware v45+
- X330 series (including XC330-T288): firmware v46+
- P series: firmware v12+

**SDK API:** `GroupSyncRead.fastSyncRead()` in dynamixel_sdk 3.8.4+.

**Platform:** Cross-platform (Dynamixel Protocol 2.0 feature).

**Source:** [Dynamixel Protocol 2.0 - Fast Sync Read](https://emanual.robotis.com/docs/en/dxl/protocol2/#fast-sync-read-0x8a)

### 4. USB Latency Timer (Linux-specific)

**What:** FTDI USB-serial adapters (like the U2D2) have a latency timer that buffers data before sending it to the host. The Linux kernel default is 16ms.

**Why it matters:** A 16ms buffer delay on every USB transfer limits control to ~30 Hz regardless of baudrate. Setting it to 1ms is essential.

**How:** The `DynamixelClient` automatically calls `set_low_latency_mode(True)` via pyserial on connect. This sets the FTDI `latency_timer` to 1ms.

**Platform:** Linux only. The `set_low_latency_mode()` call uses the Linux `ASYNC_LOW_LATENCY` flag via `fcntl`. On macOS and Windows, this call is not available (the code handles this gracefully with a try/except). macOS and Windows have their own USB latency mechanisms through driver settings.

**Verification:**
```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# Should output: 1
```

**Permanent fix (udev rule):**
```bash
sudo tee /etc/udev/rules.d/99-dynamixel-latency.rules <<< 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"'
sudo udevadm control --reload-rules
```

**Source:** [FTDI Linux USB Latency - Granite Devices](https://granitedevices.com/wiki/FTDI_Linux_USB_latency), [DynamixelSDK FAQ](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/faq/)

---

## Usage

### Running the latency test with optimizations

```bash
# Default (no optimizations)
python scripts/test_motor_latency.py /dev/ttyUSB0 dynamixel 1 2 3 4 5

# With all optimizations enabled
python scripts/test_motor_latency.py /dev/ttyUSB0 dynamixel 1 2 3 4 5 --optimize
```

The `--optimize` flag:

1. Sets Return Delay Time = 0 on all motors
2. Sets Status Return Level = 1 on all motors
3. Uses `write4ByteTxOnly` for single-motor writes
4. Runs Fast Sync Read benchmarks (tests 10-11)
5. Restores all motor settings to defaults on exit

### Applying optimizations in your own code

```python
import dynamixel_sdk as dxl

port = dxl.PortHandler("/dev/ttyUSB0")
ph = dxl.PacketHandler(2.0)
port.openPort()
port.setBaudRate(3000000)

motor_ids = [1, 2, 3, 4, 5]

# Set optimizations (requires torque disabled, EEPROM registers)
for mid in motor_ids:
    ph.write1ByteTxRx(port, mid, 64, 0)   # Torque off
    ph.write1ByteTxRx(port, mid, 9, 0)    # Return Delay Time = 0
    ph.write1ByteTxRx(port, mid, 68, 1)   # Status Return Level = READ only
    ph.write1ByteTxRx(port, mid, 64, 1)   # Torque on

# Fast Sync Read
sync_read = dxl.GroupSyncRead(port, ph, 132, 4)  # Present Position
for mid in motor_ids:
    sync_read.addParam(mid)
sync_read.fastSyncRead()  # Single combined response packet
for mid in motor_ids:
    pos = sync_read.getData(mid, 132, 4)

# Sync Write (fire-and-forget, no status responses)
sync_write = dxl.GroupSyncWrite(port, ph, 116, 4)  # Goal Position
for mid, pos in zip(motor_ids, goal_positions):
    param = [
        dxl.DXL_LOBYTE(dxl.DXL_LOWORD(pos)),
        dxl.DXL_HIBYTE(dxl.DXL_LOWORD(pos)),
        dxl.DXL_LOBYTE(dxl.DXL_HIWORD(pos)),
        dxl.DXL_HIBYTE(dxl.DXL_HIWORD(pos)),
    ]
    sync_write.addParam(mid, param)
sync_write.txPacket()
sync_write.clearParam()
```

---

## Platform Compatibility Summary

| Optimization | Linux | macOS | Windows |
|---|---|---|---|
| Return Delay Time = 0 | Yes | Yes | Yes |
| Status Return Level = 1 | Yes | Yes | Yes |
| Fast Sync Read | Yes | Yes | Yes |
| USB latency timer = 1ms | Yes (automatic) | No (driver settings) | No (Device Manager) |

The motor register settings (Return Delay Time, Status Return Level) and protocol features (Fast Sync Read) are cross-platform since they are part of the Dynamixel hardware and Protocol 2.0 specification. The USB latency timer optimization is Linux-specific; macOS and Windows require their own platform-specific USB driver tuning.

---

## Benchmark Results (17 motors, 3M baud, Linux)

### Single Motor

| Test | Default | Optimized | Improvement |
|---|---|---|---|
| Read (spaced) | 1.893 ms | 0.998 ms | 1.9x |
| Read (burst) | 2.000 ms | 0.997 ms | 2.0x |
| Write (spaced) | 1.865 ms | 0.185 ms | 10.1x |
| Write (burst) | 1.998 ms | 1.025 ms | 1.9x |
| Round-trip | 3.885 ms | 1.372 ms | 2.8x |
| Control loop | 3.999 ms | 2.200 ms | 1.8x |

### Multi-Motor (17 motors)

| Test | Default | Optimized | Improvement |
|---|---|---|---|
| Sequential read | 19.176 ms | 17.218 ms | 1.1x |
| Sync read | 11.074 ms | 2.284 ms | 4.8x |
| Fast Sync read | — | 2.006 ms | 5.5x vs default |
| Sync write | 0.690 ms | 0.298 ms | 2.3x |
| Full-hand control loop | ~11.8 ms | 4.458 ms | 2.6x |

### Control Rate Estimates

| Configuration | Max Hz |
|---|---|
| Single motor (default) | 250 Hz |
| Single motor (optimized) | 455 Hz |
| Full hand 17 motors (default) | ~85 Hz |
| Full hand 17 motors (optimized) | 224 Hz |
