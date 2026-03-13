#!/usr/bin/env python3
"""Debug diagnostic for sensor hardware register investigation"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from orca_core.hardware.sensing.sensor_client import SensorClient
import serial.tools.list_ports
import time

SENSOR_ADAPTER_VID = 0x28E9
SENSOR_ADAPTER_PID = 0x018A

def find_sensor_adapter():
    """Auto-detect sensor adapter port."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == SENSOR_ADAPTER_VID and port.pid == SENSOR_ADAPTER_PID:
            return port.device
    return None

def list_available_ports():
    """List all available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found!")
        return []

    print("\nAvailable serial ports:")
    for i, port in enumerate(ports, 1):
        marker = " <- Sensor Adapter" if (port.vid == SENSOR_ADAPTER_VID and port.pid == SENSOR_ADAPTER_PID) else ""
        print(f"  {i}. {port.device} - {port.description}{marker}")
    return [p.device for p in ports]

def diagnose_hardware_register(port=None):
    """Diagnose hardware register and payload behavior."""

    # Auto-detect port if not specified
    if port is None:
        print("Auto-detecting sensor adapter...")
        port = find_sensor_adapter()

        if port is None:
            print("❌ Could not auto-detect sensor adapter.")
            available = list_available_ports()
            if available:
                print("\nUsage: python app_debug.py [PORT]")
                print(f"Example: python app_debug.py {available[0]}")
            sys.exit(1)

        print(f"✓ Found sensor adapter at {port}\n")
    else:
        print(f"Using specified port: {port}\n")

    client = SensorClient(port=port)

    print("=== SENSOR HARDWARE DIAGNOSTIC ===\n")

    try:
        client.connect()
        print("✓ Connected to sensor adapter\n")

        # TEST 0: Check if any sensors are connected
        print("TEST 0: Basic Communication Check")
        print("-" * 50)
        try:
            num_taxels = client.read_num_taxels()
            print(f"✓ Sensor adapter is responding")
            print(f"  Taxel counts: {num_taxels}")
        except TimeoutError:
            print("❌ ERROR: Sensor adapter not responding to read commands")
            print("   Possible causes:")
            print("   - No sensors are physically connected")
            print("   - Sensor adapter needs reset (unplug/replug USB)")
            print("   - Communication issue with adapter")
            print("\nPlease ensure at least one sensor is connected and try again.\n")
            return
        except Exception as e:
            print(f"❌ ERROR: {e}")
            return

        # TEST 1: Read hardware register raw bytes
        print("\nTEST 1: Hardware Register Raw Data")
        print("-" * 50)
        try:
            data = client._read_register(0x0010, 4)
            print(f"Register 0x0010 (4 bytes): {data.hex()}")
            print(f"  Byte 0: 0x{data[0]:02x} = 0b{data[0]:08b}")
            print(f"  Byte 1: 0x{data[1]:02x} = 0b{data[1]:08b}")
            print(f"  Byte 2: 0x{data[2]:02x} = 0b{data[2]:08b}")
            print(f"  Byte 3: 0x{data[3]:02x} = 0b{data[3]:08b}")

            # Show bit positions being checked
            print(f"\nBit positions checked:")
            print(f"  Thumb:  byte[0] bit 2 = {bool(data[0] & (1 << 2))}")
            print(f"  Index:  byte[0] bit 6 = {bool(data[0] & (1 << 6))}")
            print(f"  Middle: byte[1] bit 2 = {bool(data[1] & (1 << 2))}")
            print(f"  Ring:   byte[1] bit 6 = {bool(data[1] & (1 << 6))}  <-- SUSPECT")
            print(f"  Pinky:  byte[2] bit 2 = {bool(data[2] & (1 << 2))}")

            # TEST 2: Try alternative bit positions for ring
            print(f"\nAlternative bit positions for byte[1] (ring finger):")
            for bit in range(8):
                print(f"  bit {bit}: {bool(data[1] & (1 << bit))}")
        except Exception as e:
            print(f"❌ Failed to read hardware register: {e}")

        # TEST 3: Read connected sensors (high-level API)
        print("\nTEST 2: Connected Sensors API")
        print("-" * 50)
        try:
            connected = client.read_connected_sensors()
            for finger, status in connected.items():
                marker = " <-- GHOST?" if finger == 'ring' and status else ""
                print(f"  {finger}: {'CONNECTED' if status else 'disconnected'}{marker}")
        except Exception as e:
            print(f"❌ Failed to read connected sensors: {e}")

        # TEST 4: Start auto-stream and check payload sizes
        print("\nTEST 3: Auto-Stream Payload Analysis")
        print("-" * 50)
        try:
            print("Starting auto-stream (resultant mode, using hardware register - not probing)...")
            client.start_auto_stream(resultant=True, taxels=False, use_probing=False)
            time.sleep(1.0)  # Let some frames accumulate

            # Get configuration
            config = client.get_sensor_configuration()
            print(f"Config says active sensors: {config.active_sensors}")
            print(f"Expected payload size: {config.expected_payload_size_resultant} bytes")

            # Sample actual payloads
            print(f"\nSampling 10 frames to check actual payload sizes...")
            for i in range(10):
                forces, ts = client.get_auto_latest()
                if forces:
                    print(f"  Frame {i+1}: received data for: {list(forces.keys())}")
                    for finger, values in forces.items():
                        frozen = "(FROZEN?)" if values == [0.0, 0.0, 0.0] else ""
                        print(f"    {finger}: {values} {frozen}")
                time.sleep(0.1)

            client.stop_auto_stream()
        except Exception as e:
            print(f"❌ Failed to run auto-stream test: {e}")

    finally:
        if client.is_connected:
            client.disconnect()
        print("\n=== DIAGNOSTIC COMPLETE ===\n")

if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else None
    diagnose_hardware_register(port)
