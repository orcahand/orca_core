"""Characterize macOS motor-bus <-> encoder-stream contention.

Connect motors, then start the encoder stream on the sibling CDC, then hammer
motor reads and report success rate + latency. Tells us whether the two CDCs
can coexist (stress test viable) or hard-fail (needs USB replug / Linux).
"""
import sys, time
from orca_core import load_hand
from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.joint_encoder_client import JointEncoderClient, EncodersNotAvailableError
from orca_core.hardware.sensing.serial_discovery import resolve_sensing_ports

cfg = sys.argv[1]
hand = load_hand(config_path=cfg, engage_feedback=False)
ok, msg = hand.connect()
print("motor connect:", ok, msg)

def hammer(label, n=20):
    good = bad = 0; lat = []
    for _ in range(n):
        t = time.monotonic()
        try:
            hand.get_motor_pos()
            good += 1; lat.append((time.monotonic()-t)*1000)
        except Exception:
            bad += 1
    ms = (sum(lat)/len(lat)) if lat else float("nan")
    mx = max(lat) if lat else float("nan")
    print(f"{label}: {good}/{n} ok, {bad} fail, mean={ms:.0f}ms max={mx:.0f}ms")

hammer("motor-only")

ports = resolve_sensing_ports(tactile_override="disabled", encoder_override="auto")
link = HandSerialLink(ports.encoder, baudrate=2_000_000)
link.connect()
client = JointEncoderClient(link)
client.connect()
try:
    client.start_stream(timeout=3.0)
    print("encoder stream up on", ports.encoder)
except EncodersNotAvailableError as e:
    print("encoder start failed:", e)

hammer("motor+encoder (1)")
time.sleep(0.5)
hammer("motor+encoder (2)")
r = client.get_latest()
print("encoder latest freshness_ms:", (r.freshness_ms if r else None))

client.stop_stream(); client.disconnect(); link.disconnect()
hammer("motor after encoder closed")
hand.disconnect()
