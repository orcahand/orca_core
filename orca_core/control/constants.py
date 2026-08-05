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
# on top of the motor's internal position PID. Per-joint overrides live in
# config.yaml under 'joint_control_gains'; see hand_config.JointGains.
#
# The correction is mapped to motor space through joint_to_motor_ratios, so a
# joint degree of correction nominally produces a joint degree of motion: Kp is
# dimensionless and Kp=1 would command the whole observed error every cycle.
# Only mechanical lag keeps that stable, so stiffly-coupled (fast) joints have
# the least margin and want the lowest Kp.
DEFAULT_KP = 0.5           # degrees of correction per degree of error
DEFAULT_KI = 5.0           # degrees of correction per degree·second of error

# Also the effective anti-windup bound: conditional integration freezes the
# integrator once the output saturates, so the retained integral contribution
# never exceeds this value regardless of Ki. Whatever is retained has to be
# discharged by overshooting, so keep it a trim, not full joint authority.
DEFAULT_CORRECTION_MAX_DEG = 15.0

# Per-joint default overrides for joints whose mechanics don't suit the scalars
# above. Each entry names only the gains it changes; the rest fall through.
# A hand's config.yaml 'joint_control_gains' block overrides this table in turn.
DEFAULT_JOINT_GAIN_OVERRIDES = {
    # Stiffer coupling than the fingers: they reach their travel in less motor
    # rotation, so they respond faster and carry less stability margin.
    "thumb_cmc": {"kp": 0.35},
    "thumb_abd": {"kp": 0.35},
}

# The wrist motor runs in multi_turn_position mode, which has no current
# limiting, so the loop clamps its commanded position to the calibrated
# travel plus this margin (joint-space degrees). Fingers stay unclamped:
# their tendon-stretch over-travel is deliberate and mode 5 caps current.
WRIST_MOTOR_TARGET_MARGIN_DEG = 3.0

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
