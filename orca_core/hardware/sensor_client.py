import time
import serial


class SensorClient:
    """Serial client for AS5600 angle sensors connected via Arduino.

    Protocol: send 'R\\n', receive CSV of raw 12-bit readings (0-4095).
    """

    RAW_TO_DEG = 360.0 / 4096.0

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200, num_sensors: int = 1):
        self.port = port
        self.baudrate = baudrate
        self.num_sensors = num_sensors
        self._serial: serial.Serial = None
        self._prev_angles: list[float] = [0.0] * num_sensors

    def connect(self):
        self._serial = serial.Serial(self.port, self.baudrate, timeout=2)
        time.sleep(2)  # Wait for Arduino boot
        self._serial.reset_input_buffer()
        self.read_angles_raw()  # Warm-up read to prime unwrap state

    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None

    def read_angles_raw(self) -> list[int]:
        self._serial.write(b"R\n")
        line = self._serial.readline().decode().strip()
        if not line:
            raise TimeoutError("No response from sensor Arduino")
        values = [int(v) for v in line.split(",")]
        if len(values) != self.num_sensors:
            raise ValueError(
                f"Expected {self.num_sensors} sensor values, got {len(values)}: {line!r}"
            )
        return values

    def read_angles_degrees(self) -> list[float]:
        raw = self.read_angles_raw()
        return [v * self.RAW_TO_DEG for v in raw]

    def read_angles_unwrapped(self) -> list[float]:
        angles = self.read_angles_degrees()
        unwrapped = []
        for i, angle in enumerate(angles):
            diff = angle - (self._prev_angles[i] % 360.0)
            if diff > 180.0:
                diff -= 360.0
            elif diff < -180.0:
                diff += 360.0
            self._prev_angles[i] += diff
            unwrapped.append(self._prev_angles[i])
        return unwrapped

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
