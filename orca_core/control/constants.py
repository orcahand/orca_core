"""Constants for the joint-control loop and watchdog."""


DEFAULT_LOOP_HZ = 100

# Controller dt clamp so a loop slip cannot blow up the integral term:
# floor below the realistic loop budget, ceiling at the watchdog hold tier.
MIN_LOOP_DT_S = 0.001
MAX_LOOP_DT_S = 0.050

# Encoder-freshness watchdog tiers, in ms since the last frame: warn,
# freeze integrator, drop the PI trim (base motor target only), stop the loop.
WATCHDOG_WARN_MS = 15
WATCHDOG_HOLD_MS = 50
WATCHDOG_HOLD_BASE_MS = 200
WATCHDOG_STOP_LOOP_MS = 1000

# Outer-loop PI defaults: the host trims residual joint-encoder error (deg)
# on top of the motor's internal position PID.
DEFAULT_KP = 1.0           # degrees of correction per degree of error
DEFAULT_KI = 12.0          # degrees of correction per degree·second of error
DEFAULT_CORRECTION_MAX_DEG = 60.0
DEFAULT_I_CLAMP_DEG = 15.0

# Freshness warning is rate-limited so a chronic stale-encoder condition
# doesn't drown the log file.
FRESHNESS_WARN_INTERVAL_S = 1.0

# Jitter monitor: loop-period ratios and consecutive-cycle streak lengths
# that trigger warn / e-stop; a single transient slow cycle is tolerated.
JITTER_WARN_RATIO = 2.0
JITTER_ESTOP_RATIO = 10.0
JITTER_WARN_CONSECUTIVE = 10
JITTER_ESTOP_CONSECUTIVE = 5

# Rate limit for the in-loop "step_once raised" log so a sick bus doesn't
# spam the handler.
LOOP_EXCEPTION_LOG_INTERVAL_S = 1.0
