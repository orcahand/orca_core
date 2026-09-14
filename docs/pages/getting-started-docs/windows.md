# Running on Windows

orca_core runs on Windows 10 and 11. The hand is driven over a serial port through pyserial, which handles COM ports natively, and the control code itself has no platform-specific parts. This page covers the few places where Windows differs from macOS and Linux, and ends with the checklist for a first connection.

The test suite runs on every pull request on both Ubuntu and Windows, so the software path is verified continuously. Real-hardware behaviour on Windows is covered by the checklist at the bottom.

---

## 1. Install Python and uv

1. Python 3.10 or newer, either from python.org (tick **Add python.exe to PATH** in the installer) or

    ```powershell
    winget install Python.Python.3.12
    ```

2. uv:

    ```powershell
    winget install astral-sh.uv
    ```

3. From the repository folder, the same commands as on every other platform:

    ```powershell
    uv sync --group dev
    uv run pytest
    ```

    An activated shell, if you prefer one, is `.venv\Scripts\activate` (cmd) or `.venv\Scripts\Activate.ps1` (PowerShell).

Use **Windows Terminal** or PowerShell rather than the legacy `cmd.exe` window. The scripts switch the console to ANSI mode themselves, so colours and in-place redraws work in all three, but Windows Terminal is the one that is tested.

---

## 2. USB drivers

Which adapter you have depends on the hand version; look it up under **Device Manager > Ports (COM & LPT)** once the hand is plugged in.

| Device | Shows up as | Driver |
|---|---|---|
| ORCA controller board (v2 hands) | two `USB Serial Device (COMx)` entries, one for the motor bus and one for the sensing link | Built into Windows 10/11 (`usbser`). Nothing to install. |
| U2D2 or other FTDI adapter (Dynamixel) | `USB Serial Port (COMx)` | FTDI VCP. Windows Update installs it automatically when the PC is online; otherwise download it from FTDI. |
| Feetech adapter, CH340 | `USB-SERIAL CH340 (COMx)` | Windows Update usually handles it; otherwise the WCH CH341SER driver. |
| Feetech adapter, CP210x | `Silicon Labs CP210x USB to UART Bridge (COMx)` | Windows Update; otherwise the Silicon Labs VCP driver. |

If a device appears with a warning icon instead of a COM number, the driver did not bind. Right-click it, choose **Update driver**, and let Windows search online.

### FTDI latency timer (U2D2 only)

FTDI adapters buffer incoming bytes for 16 ms by default, which caps the control loop far below what the motors can do. On Linux the package lowers this itself; Windows exposes it only in Device Manager:

**Ports (COM & LPT) > USB Serial Port (COMx) > Properties > Port Settings > Advanced > Latency Timer (msec)**: set it to **1** and click OK.

The ORCA controller board and CH340/CP210x adapters have no such setting.

---

## 3. Find your COM port

```powershell
uv run python -m serial.tools.list_ports -v
```

This lists every serial port with its USB vendor and product IDs, the same information the auto-detection uses. A v2 hand shows two ports from the same board.

Port selection works exactly as on other platforms: `port: auto` in `config.yaml` finds the adapter by USB ID, and only asks you to choose when it cannot decide. On Windows that question is a numbered list rather than the arrow-key menu, because the Python builds for Windows ship without `curses`.

To pin a port by hand:

```yaml
port: COM3
```

---

## 4. Differences worth knowing

- **No permission setup.** There is no `dialout` group and nothing to `chmod`.
- **Ports are exclusive.** Windows lets only one process open a COM port at a time. A second script gets `PermissionError: Access is denied` immediately, where Linux and macOS would rely on the advisory lock the package takes. Close the other program (including a serial monitor or the Arduino IDE) and retry.
- **Port names are not paths.** `os.path.exists("COM3")` is always false. The package asks pyserial instead, so `resolve_port`, `wait_for_port` and the motor-chain configuration behave the same as on other platforms. Keep this in mind if you write your own tooling on top of orca_core.
- **Success beep** in `configure_motor_chain.py` uses the Windows system beep.

---

## 5. First connection checklist

Run this once per Windows machine before relying on it for calibration or assembly. Each step should behave exactly as it does on macOS or Linux.

1. `uv run pytest` passes.
2. Plug in the hand. `uv run python -m serial.tools.list_ports -v` shows it with a COM number and no warning icon in Device Manager.
3. Follow **Get Started** in the README (tension, calibrate, neutral) with the hand's `config.yaml`. Auto-detection should pick the port without being asked; if it asks, the numbered list appears and works.
4. Hands with sensing: `uv run python scripts/check_sensors.py <config.yaml>` reports the encoder stream and tactile checks as passing.
5. Assembly stations: `uv run python scripts/configure_motor_chain.py <config.yaml>` walks through the motors, waits for each plug and unplug, and beeps on success.

If any step differs from the macOS/Linux behaviour, open an issue on the orca_core repository with the step number, the Device Manager entry for the adapter, and the full console output.
