"""Constants for the joint-control loop and watchdog."""


DEFAULT_LOOP_HZ = 100

# Controller dt is clamped to this band so a loop slip cannot blow up the
# integral term. Floor sits below the realistic loop budget; ceiling
# matches the watchdog hold tier so controller and watchdog stay coherent
# at slow loop rates.
MIN_LOOP_DT_S = 0.001
MAX_LOOP_DT_S = 0.050

# Encoder-freshness watchdog tiers, in milliseconds since the last frame.
# Tier-1: rate-limited warn. Tier-2: freeze the integrator. Tier-3: drop the
# PI trim and write only the base motor target (open-loop joint control via
# the motor's internal position PID). Tier-4: stop the loop entirely.
WATCHDOG_WARN_MS = 10
WATCHDOG_HOLD_MS = 50
WATCHDOG_HOLD_BASE_MS = 200
WATCHDOG_STOP_LOOP_MS = 1000

# PI defaults for the outer joint loop. The motor's internal position PID
# handles motor-shaft tracking against the motor encoder; the host runs a
# slower PI on joint-encoder error and adds a per-joint correction
# (degrees) to the base joint target.
DEFAULT_KP = 1.0           # degrees of correction per degree of error
DEFAULT_KI = 12.0          # degrees of correction per degree·second of error
DEFAULT_CORRECTION_MAX_DEG = 60.0
DEFAULT_I_CLAMP_DEG = 15.0

# Freshness warning is rate-limited so a chronic stale-encoder condition
# doesn't drown the log file.
FRESHNESS_WARN_INTERVAL_S = 1.0

# Jitter monitor: ratios of the target loop period and the consecutive-cycle
# streak lengths that trigger warn / e-stop. A single transient is tolerated;
# what matters is a sustained run of slow cycles.
JITTER_WARN_RATIO = 2.0
JITTER_ESTOP_RATIO = 10.0
JITTER_WARN_CONSECUTIVE = 10
JITTER_ESTOP_CONSECUTIVE = 5

# Rate limit for the in-loop "step_once raised" log so a sick bus doesn't
# spam the handler.
LOOP_EXCEPTION_LOG_INTERVAL_S = 1.0
