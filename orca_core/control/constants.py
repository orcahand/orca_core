"""Constants for the joint-control loop and watchdog."""


DEFAULT_LOOP_HZ = 100

# Controller dt is clamped to this band so a loop slip cannot blow up the
# integral term. Floor sits below the realistic loop budget; ceiling
# matches the watchdog hold tier so controller and watchdog stay coherent
# at slow loop rates.
MIN_LOOP_DT_S = 0.001
MAX_LOOP_DT_S = 0.050

# Encoder-freshness watchdog tiers, in milliseconds since the last frame.
WATCHDOG_WARN_MS = 10
WATCHDOG_HOLD_MS = 50
WATCHDOG_DROP_TORQUE_MS = 200
WATCHDOG_STOP_LOOP_MS = 1000

# PI defaults for the outer joint loop. The motor's internal position PID
# handles motor-shaft tracking against the motor encoder; the host runs a
# slower PI on joint-encoder error and adds a per-joint correction
# (degrees) to the base joint target.
DEFAULT_KP = 1.0
DEFAULT_KI = 12.0
DEFAULT_CORRECTION_MAX_DEG = 60.0
DEFAULT_I_CLAMP_DEG = 15.0
