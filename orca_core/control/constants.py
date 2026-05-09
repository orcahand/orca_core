"""Constants for the joint-control loop and watchdog."""

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
