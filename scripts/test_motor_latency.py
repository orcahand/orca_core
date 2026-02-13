#!/usr/bin/env python
"""Test motor read/write latency for Feetech or Dynamixel motors.

Measures communication latency with various test patterns.

Usage:
    # Feetech motor
    python scripts/test_motor_latency.py /dev/ttyUSB0 feetech 1

    # Dynamixel motor
    python scripts/test_motor_latency.py /dev/ttyUSB0 dynamixel 1

    # Multiple motors for sync tests
    python scripts/test_motor_latency.py /dev/ttyUSB0 feetech 1 2 3

    # Custom iterations and baudrate
    python scripts/test_motor_latency.py /dev/ttyUSB0 dynamixel 1 -n 200 --baudrate 3000000
"""

import argparse
import signal
import statistics
import sys
import time
from abc import ABC, abstractmethod


WARMUP_ITERATIONS = 10


class MotorInterface(ABC):
    """Abstract interface for motor communication timing tests."""

    @abstractmethod
    def connect(self, low_latency: bool = True) -> bool:
        pass

    @abstractmethod
    def disconnect(self):
        pass

    @abstractmethod
    def setup_motor(self, motor_id: int) -> int:
        """Setup motor and return current position."""
        pass

    @abstractmethod
    def read_position(self, motor_id: int) -> int:
        pass

    @abstractmethod
    def write_position(self, motor_id: int, position: int):
        pass

    @abstractmethod
    def sync_read_positions(self, motor_ids: list) -> list:
        pass

    @abstractmethod
    def sync_write_positions(self, motor_ids: list, positions: list):
        pass


class FeetechInterface(MotorInterface):
    """Feetech motor interface for latency testing."""

    def __init__(self, port: str, baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.handler = None

    def connect(self, low_latency: bool = True) -> bool:
        from orca_core.hardware.feetech import PortHandler, sms_sts
        self.port_handler = PortHandler(self.port)
        self.port_handler.baudrate = self.baudrate
        if not self.port_handler.openPort():
            return False

        if low_latency and hasattr(self.port_handler, 'ser') and hasattr(self.port_handler.ser, 'set_low_latency_mode'):
            try:
                self.port_handler.ser.set_low_latency_mode(True)
                print("Enabled low latency mode for USB serial")
            except Exception as e:
                print(f"Failed to enable low latency mode: {e}")

        self.handler = sms_sts(self.port_handler)
        return True

    def disconnect(self):
        if self.port_handler:
            self.port_handler.closePort()

    def setup_motor(self, motor_id: int) -> int:
        self.handler.write1ByteTxRx(motor_id, 33, 0)  # Servo mode
        time.sleep(0.05)
        self.handler.write1ByteTxRx(motor_id, 40, 1)  # Torque enable
        time.sleep(0.05)
        pos_raw, _, _ = self.handler.read2ByteTxRx(motor_id, 56)
        return self.handler.scs_tohost(pos_raw, 15)

    def disable_torque(self, motor_id: int):
        self.handler.write1ByteTxRx(motor_id, 40, 0)

    def read_position(self, motor_id: int) -> int:
        pos_raw, _, _ = self.handler.read2ByteTxRx(motor_id, 56)
        return self.handler.scs_tohost(pos_raw, 15)

    def write_position(self, motor_id: int, position: int):
        self.handler.WritePosEx(motor_id, position, 60, 50, 500)

    def sync_read_positions(self, motor_ids: list) -> list:
        from orca_core.hardware.feetech import GroupSyncRead
        sync_read = GroupSyncRead(self.handler, 56, 2)
        for mid in motor_ids:
            sync_read.addParam(mid)
        sync_read.txRxPacket()
        positions = []
        for mid in motor_ids:
            pos_raw = sync_read.getData(mid, 56, 2)
            positions.append(self.handler.scs_tohost(pos_raw, 15))
        return positions

    def sync_write_positions(self, motor_ids: list, positions: list):
        self.handler.groupSyncWrite.clearParam()
        for mid, pos in zip(motor_ids, positions):
            self.handler.SyncWritePosEx(mid, pos, 60, 50, 500)
        self.handler.groupSyncWrite.txPacket()
        self.handler.groupSyncWrite.clearParam()


class DynamixelInterface(MotorInterface):
    """Dynamixel motor interface for latency testing."""

    ADDR_TORQUE_ENABLE = 64
    ADDR_OPERATING_MODE = 11
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132

    def __init__(self, port: str, baudrate: int = 3000000):
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None
        self.dxl = None

    def connect(self, low_latency: bool = True) -> bool:
        import dynamixel_sdk
        self.dxl = dynamixel_sdk
        self.port_handler = self.dxl.PortHandler(self.port)
        self.packet_handler = self.dxl.PacketHandler(2.0)  # Protocol 2.0
        if not self.port_handler.openPort():
            return False
        if not self.port_handler.setBaudRate(self.baudrate):
            return False

        if low_latency and hasattr(self.port_handler, 'ser') and hasattr(self.port_handler.ser, 'set_low_latency_mode'):
            try:
                self.port_handler.ser.set_low_latency_mode(True)
                print("Enabled low latency mode for USB serial")
            except Exception as e:
                print(f"Failed to enable low latency mode: {e}")

        return True

    def disconnect(self):
        if self.port_handler:
            self.port_handler.closePort()

    def setup_motor(self, motor_id: int) -> int:
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 0)
        time.sleep(0.05)
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.ADDR_OPERATING_MODE, 3)
        time.sleep(0.05)
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 1)
        time.sleep(0.05)
        pos, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, self.ADDR_PRESENT_POSITION)
        return pos

    def disable_torque(self, motor_id: int):
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 0)

    def read_position(self, motor_id: int) -> int:
        pos, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, self.ADDR_PRESENT_POSITION)
        return pos

    def write_position(self, motor_id: int, position: int):
        self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, self.ADDR_GOAL_POSITION, position)

    def sync_read_positions(self, motor_ids: list) -> list:
        sync_read = self.dxl.GroupSyncRead(self.port_handler, self.packet_handler, self.ADDR_PRESENT_POSITION, 4)
        for mid in motor_ids:
            sync_read.addParam(mid)
        sync_read.txRxPacket()
        positions = []
        for mid in motor_ids:
            pos = sync_read.getData(mid, self.ADDR_PRESENT_POSITION, 4)
            positions.append(pos)
        sync_read.clearParam()
        return positions

    def sync_write_positions(self, motor_ids: list, positions: list):
        sync_write = self.dxl.GroupSyncWrite(self.port_handler, self.packet_handler, self.ADDR_GOAL_POSITION, 4)
        for mid, pos in zip(motor_ids, positions):
            param = [
                self.dxl.DXL_LOBYTE(self.dxl.DXL_LOWORD(pos)),
                self.dxl.DXL_HIBYTE(self.dxl.DXL_LOWORD(pos)),
                self.dxl.DXL_LOBYTE(self.dxl.DXL_HIWORD(pos)),
                self.dxl.DXL_HIBYTE(self.dxl.DXL_HIWORD(pos)),
            ]
            sync_write.addParam(mid, param)
        sync_write.txPacket()
        sync_write.clearParam()


# Global for cleanup
interface = None
motor_ids = []


def cleanup():
    global interface, motor_ids
    if interface:
        for mid in motor_ids:
            try:
                interface.disable_torque(mid)
            except:
                pass
        interface.disconnect()


def signal_handler(sig, frame):
    print("\nInterrupted")
    cleanup()
    sys.exit(0)


def print_stats(name, times_ms):
    if len(times_ms) < 2:
        print(f"{name}: insufficient data")
        return
    print(f"{name}:")
    print(f"  avg: {statistics.mean(times_ms):.3f} ms")
    print(f"  min: {min(times_ms):.3f} ms  max: {max(times_ms):.3f} ms")
    print(f"  std: {statistics.stdev(times_ms):.3f} ms")
    p95_idx = min(int(len(times_ms) * 0.95), len(times_ms) - 1)
    print(f"  p50: {statistics.median(times_ms):.3f} ms  p95: {sorted(times_ms)[p95_idx]:.3f} ms")


def measure_read_latency(iface, motor_id, iterations, burst=False):
    """Measure position read latency."""
    for _ in range(WARMUP_ITERATIONS):
        iface.read_position(motor_id)

    times = []
    for _ in range(iterations):
        start = time.perf_counter()
        iface.read_position(motor_id)
        elapsed = (time.perf_counter() - start) * 1000
        times.append(elapsed)
        if not burst:
            time.sleep(0.01)
    return times


def measure_write_latency(iface, motor_id, positions, iterations, burst=False):
    """Measure position write latency."""
    for i in range(WARMUP_ITERATIONS):
        iface.write_position(motor_id, positions[i % len(positions)])
        if not burst:
            time.sleep(0.02)

    times = []
    for i in range(iterations):
        pos = positions[i % len(positions)]
        start = time.perf_counter()
        iface.write_position(motor_id, pos)
        elapsed = (time.perf_counter() - start) * 1000
        times.append(elapsed)
        if not burst:
            time.sleep(0.02)
    return times


def measure_roundtrip_latency(iface, motor_id, positions, iterations):
    """Measure write + immediate read latency."""
    for i in range(WARMUP_ITERATIONS):
        iface.write_position(motor_id, positions[i % len(positions)])
        iface.read_position(motor_id)

    times = []
    for i in range(iterations):
        pos = positions[i % len(positions)]
        start = time.perf_counter()
        iface.write_position(motor_id, pos)
        iface.read_position(motor_id)
        elapsed = (time.perf_counter() - start) * 1000
        times.append(elapsed)
        time.sleep(0.02)
    return times


def measure_control_loop_latency(iface, motor_id, positions, iterations):
    """Simulate control loop: read->write with no delays."""
    for i in range(WARMUP_ITERATIONS):
        iface.read_position(motor_id)
        iface.write_position(motor_id, positions[i % len(positions)])

    times = []
    for i in range(iterations):
        start = time.perf_counter()
        iface.read_position(motor_id)
        target = positions[i % len(positions)]
        iface.write_position(motor_id, target)
        elapsed = (time.perf_counter() - start) * 1000
        times.append(elapsed)
    return times


def measure_sequential_read_latency(iface, motor_ids, iterations):
    """Measure sequential reads of multiple motors."""
    if len(motor_ids) < 2:
        return None

    for _ in range(WARMUP_ITERATIONS):
        for mid in motor_ids:
            iface.read_position(mid)

    times = []
    for _ in range(iterations):
        start = time.perf_counter()
        for mid in motor_ids:
            iface.read_position(mid)
        elapsed = (time.perf_counter() - start) * 1000
        times.append(elapsed)
        time.sleep(0.01)
    return times


def measure_sync_read_latency(iface, motor_ids, iterations):
    """Measure sync read latency for multiple motors."""
    if len(motor_ids) < 2:
        return None

    for _ in range(WARMUP_ITERATIONS):
        iface.sync_read_positions(motor_ids)

    times = []
    for _ in range(iterations):
        start = time.perf_counter()
        iface.sync_read_positions(motor_ids)
        elapsed = (time.perf_counter() - start) * 1000
        times.append(elapsed)
        time.sleep(0.01)
    return times


def measure_sync_write_latency(iface, motor_ids, positions_dict, iterations):
    """Measure sync write latency for multiple motors."""
    if len(motor_ids) < 2:
        return None

    for _ in range(WARMUP_ITERATIONS):
        positions = [positions_dict[mid][0] for mid in motor_ids]
        iface.sync_write_positions(motor_ids, positions)

    times = []
    for i in range(iterations):
        positions = [positions_dict[mid][i % len(positions_dict[mid])] for mid in motor_ids]
        start = time.perf_counter()
        iface.sync_write_positions(motor_ids, positions)
        elapsed = (time.perf_counter() - start) * 1000
        times.append(elapsed)
        time.sleep(0.02)
    return times


def main():
    global interface, motor_ids

    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser(description="Test motor latency")
    parser.add_argument("port", help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument("motor_type", choices=["feetech", "dynamixel"], help="Motor type")
    parser.add_argument("motor_ids", type=int, nargs='+', help="Motor ID(s)")
    parser.add_argument("--iterations", "-n", type=int, default=100, help="Iterations per test")
    parser.add_argument("--baudrate", "-b", type=int, default=None, help="Baudrate (default: 1M for feetech, 3M for dynamixel)")
    parser.add_argument("--amplitude", "-a", type=int, default=100, help="Position amplitude for oscillation test (default: 100)")
    parser.add_argument("--no-low-latency", action="store_true", help="Disable low latency mode (for testing)")
    args = parser.parse_args()

    motor_ids = args.motor_ids
    iterations = args.iterations
    primary_motor = motor_ids[0]

    if args.baudrate:
        baudrate = args.baudrate
    else:
        baudrate = 1000000 if args.motor_type == "feetech" else 3000000

    print(f"Motor type: {args.motor_type.upper()}")
    print(f"Testing latency for motor(s) {motor_ids} on {args.port} @ {baudrate} baud")
    print(f"Low latency mode: {'enabled' if not args.no_low_latency else 'DISABLED'}")
    print(f"Iterations per test: {iterations} (+ {WARMUP_ITERATIONS} warmup)")
    print("=" * 60)

    if args.motor_type == "feetech":
        interface = FeetechInterface(args.port, baudrate)
    else:
        interface = DynamixelInterface(args.port, baudrate)

    low_latency = not args.no_low_latency
    if not interface.connect(low_latency=low_latency):
        print("Failed to open port")
        return

    amplitude = args.amplitude
    positions_dict = {}
    for mid in motor_ids:
        current_pos = interface.setup_motor(mid)
        if args.motor_type == "feetech":
            positions_dict[mid] = [
                max(500, current_pos - amplitude),
                min(3500, current_pos + amplitude)
            ]
        else:
            positions_dict[mid] = [
                max(0, current_pos - amplitude),
                min(4095, current_pos + amplitude)
            ]
        print(f"Motor {mid}: current={current_pos}, test range={positions_dict[mid]}")

    positions = positions_dict[primary_motor]
    print()

    try:
        print("=" * 60)
        print("SINGLE MOTOR TESTS")
        print("=" * 60)

        print("\n[1] Read latency (10ms between reads)...")
        read_times = measure_read_latency(interface, primary_motor, iterations, burst=False)
        print_stats("Read (spaced)", read_times)

        print("\n[2] Read latency (burst, no delays)...")
        read_burst_times = measure_read_latency(interface, primary_motor, iterations, burst=True)
        print_stats("Read (burst)", read_burst_times)

        print("\n[3] Write latency (20ms between writes)...")
        write_times = measure_write_latency(interface, primary_motor, positions, iterations, burst=False)
        print_stats("Write (spaced)", write_times)

        print("\n[4] Write latency (burst, no delays)...")
        write_burst_times = measure_write_latency(interface, primary_motor, positions, iterations, burst=True)
        print_stats("Write (burst)", write_burst_times)

        print("\n[5] Round-trip latency (write + read)...")
        rt_times = measure_roundtrip_latency(interface, primary_motor, positions, iterations)
        print_stats("Round-trip", rt_times)

        print("\n[6] Control loop simulation (read->write, no delays)...")
        loop_times = measure_control_loop_latency(interface, primary_motor, positions, iterations)
        print_stats("Control loop", loop_times)
        max_hz = 1000 / statistics.mean(loop_times)
        print(f"  Max control rate: {max_hz:.1f} Hz")

        if len(motor_ids) >= 2:
            print()
            print("=" * 60)
            print(f"MULTI-MOTOR TESTS ({len(motor_ids)} motors)")
            print("=" * 60)

            print(f"\n[7] Sequential read ({len(motor_ids)} motors)...")
            seq_read_times = measure_sequential_read_latency(interface, motor_ids, iterations)
            if seq_read_times:
                print_stats("Sequential read", seq_read_times)

            print(f"\n[8] Sync read ({len(motor_ids)} motors)...")
            sync_read_times = measure_sync_read_latency(interface, motor_ids, iterations)
            if sync_read_times:
                print_stats("Sync read", sync_read_times)
                speedup = statistics.mean(seq_read_times) / statistics.mean(sync_read_times)
                print(f"  Speedup vs sequential: {speedup:.2f}x")

            print(f"\n[9] Sync write ({len(motor_ids)} motors)...")
            sync_write_times = measure_sync_write_latency(interface, motor_ids, positions_dict, iterations)
            if sync_write_times:
                print_stats("Sync write", sync_write_times)

        print()
        print("=" * 60)
        print("SUMMARY")
        print("=" * 60)
        print(f"{'Test':<25} {'Avg (ms)':<12} {'Min (ms)':<12} {'Max (ms)':<12}")
        print("-" * 60)
        print(f"{'Read (spaced)':<25} {statistics.mean(read_times):<12.3f} {min(read_times):<12.3f} {max(read_times):<12.3f}")
        print(f"{'Read (burst)':<25} {statistics.mean(read_burst_times):<12.3f} {min(read_burst_times):<12.3f} {max(read_burst_times):<12.3f}")
        print(f"{'Write (spaced)':<25} {statistics.mean(write_times):<12.3f} {min(write_times):<12.3f} {max(write_times):<12.3f}")
        print(f"{'Write (burst)':<25} {statistics.mean(write_burst_times):<12.3f} {min(write_burst_times):<12.3f} {max(write_burst_times):<12.3f}")
        print(f"{'Round-trip':<25} {statistics.mean(rt_times):<12.3f} {min(rt_times):<12.3f} {max(rt_times):<12.3f}")
        print(f"{'Control loop':<25} {statistics.mean(loop_times):<12.3f} {min(loop_times):<12.3f} {max(loop_times):<12.3f}")

        if len(motor_ids) >= 2 and sync_read_times:
            print(f"{'Sync read':<25} {statistics.mean(sync_read_times):<12.3f} {min(sync_read_times):<12.3f} {max(sync_read_times):<12.3f}")
            print(f"{'Sequential read':<25} {statistics.mean(seq_read_times):<12.3f} {min(seq_read_times):<12.3f} {max(seq_read_times):<12.3f}")

        print()
        print(f"Estimated max control loop rate: {max_hz:.1f} Hz")

    finally:
        cleanup()
        print("\nDone!")


if __name__ == "__main__":
    main()
