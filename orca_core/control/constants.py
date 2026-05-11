"""Constants for the joint-control loop and watchdog."""

import math


DEFAULT_LOOP_HZ = 200

# PID dt is clamped to this band so a loop slip cannot blow up the integral or
# derivative term. Floor sits below the realistic loop budget; ceiling matches
# the watchdog hold tier so PID and watchdog stay coherent at slow loop rates.
MIN_LOOP_DT_S = 0.001
MAX_LOOP_DT_S = 0.050

# Encoder-freshness watchdog tiers, in milliseconds since the last frame.
WATCHDOG_WARN_MS = 10
WATCHDOG_HOLD_MS = 50
WATCHDOG_DROP_TORQUE_MS = 200
WATCHDOG_STOP_LOOP_MS = 1000

# PID gain defaults for the host-side joint loop. Conservative P-only seed for
# first-finger bring-up; Ki and Kd are tuned live once the finger is stable.
DEFAULT_KP_MA_PER_RAD = 200.0
DEFAULT_KI_MA_PER_RAD_S = 0.0
DEFAULT_KD_MA_S_PER_RAD = 0.0

# Cascaded outer-loop trim. Inner Dynamixel position PID handles motor-shaft
# tracking against the motor encoder; the host runs a slower PI on joint-encoder
# error and adds a per-joint correction (radians) to the base joint target. PI
# only — the inner loop is already damped, and a derivative on quantised joint
# encoders at this rate is mostly noise.
DEFAULT_CASCADED_LOOP_HZ = 100
DEFAULT_CASCADED_KP_RAD_PER_RAD = 1
DEFAULT_CASCADED_KI_RAD_PER_RAD_S = 12
DEFAULT_CASCADED_CORRECTION_MAX_RAD = math.radians(60)
DEFAULT_CASCADED_I_CLAMP_RAD = math.radians(15)
