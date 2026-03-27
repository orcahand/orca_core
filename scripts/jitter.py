"""Quick jitter test on a single motor."""
import argparse
from orca_core import OrcaHand

parser = argparse.ArgumentParser()
parser.add_argument("--config-path", dest="config_path", type=str, default=None)
parser.add_argument("--motor", type=int, default=3)
parser.add_argument("--duration", type=float, default=10.0)
parser.add_argument("--amplitude", type=float, default=5.0)
parser.add_argument("--frequency", type=float, default=10.0)
args = parser.parse_args()

hand = OrcaHand(config_path=args.config_path)
hand.connect()
hand.enable_torque()
hand.set_control_mode('current_based_position')
hand.set_max_current(hand.max_current)

print(f"Jittering motor {args.motor} for {args.duration}s (amp={args.amplitude}°, freq={args.frequency}Hz)")
hand.jitter(motor_ids=[args.motor], amplitude=args.amplitude, frequency=args.frequency, duration=args.duration)

print("Done.")
hand.disable_torque()
hand.disconnect()
