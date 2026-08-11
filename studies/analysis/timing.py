"""Statistics over a recorded encoder stream.

The question these answer is not "how fast is the stream" — the mean is easy and
uninformative. It is whether the gaps that matter are the board failing to send,
the wire losing a frame, or this host failing to wake a thread. Those have
different fixes and an inter-arrival histogram alone cannot tell them apart.

Two things separate them without any extra instrumentation:

* The board emits on a fixed period, so a gap caused by a **lost frame** is an
  integer multiple of that period. A gap caused by a **late host** is not.
* After a late wake-up the buffered frames are already waiting and drain in
  microseconds, so the gap that follows is near zero. After a real loss the next
  gap is an ordinary one.

Neither is conclusive alone; together they are hard to fool.
"""

from __future__ import annotations

from typing import Dict, Optional, Sequence

import numpy as np

from orca_core.hardware.sensing.constants import (
    AUTO_ENC_ANGLE_ERROR_BIT,
    AUTO_ENC_ANGLE_MASK,
    AUTO_ENC_NUM_JOINTS,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware.sensing.encoder_protocol import _check_even_parity

from ..preconditions import MAX_PLAUSIBLE_JUMP_COUNTS


# The connector board queues an encoder frame on this period.
BOARD_EMIT_PERIOD_MS = 2.0

# A gap of one period is a frame arriving on time. Anything from one and a half
# periods up means at least one frame did not arrive when the board sent it,
# which is the smallest event worth asking the question about — re-timing by a
# single period is the common case and a higher threshold steps straight over it.
LATE_FRAME_THRESHOLD_MS = 1.5 * BOARD_EMIT_PERIOD_MS


def inter_arrival_ms(table) -> np.ndarray:
    """Gaps between consecutive frame arrivals, in milliseconds."""
    return np.diff(table["t"]) * 1000.0


def distribution(values: np.ndarray) -> Dict[str, float]:
    """Percentiles that describe a tail, plus the mean that does not."""
    if values.size == 0:
        return {}
    return {
        "count": int(values.size),
        "mean_ms": float(values.mean()),
        "p50_ms": float(np.percentile(values, 50)),
        "p90_ms": float(np.percentile(values, 90)),
        "p99_ms": float(np.percentile(values, 99)),
        "p999_ms": float(np.percentile(values, 99.9)),
        "max_ms": float(values.max()),
        "min_ms": float(values.min()),
        "std_ms": float(values.std()),
    }


def observed_rate_hz(table) -> float:
    """Frames per second over the whole record."""
    times = table["t"]
    if times.size < 2:
        return float("nan")
    span = times[-1] - times[0]
    return float((times.size - 1) / span) if span > 0 else float("nan")


def period_multiples(
    gaps_ms: np.ndarray, period_ms: float = BOARD_EMIT_PERIOD_MS
) -> Dict[int, int]:
    """How many gaps land on each integer multiple of the emit period.

    A frame the board never sent, or the wire lost, leaves a gap of exactly two
    periods; two lost leave three. A host that woke late leaves a gap that is
    not a multiple of anything. So mass concentrated on the integers is loss,
    and mass spread between them is this machine.
    """
    if gaps_ms.size == 0:
        return {}
    multiples = np.rint(gaps_ms / period_ms).astype(int)
    values, counts = np.unique(multiples, return_counts=True)
    return {int(value): int(count) for value, count in zip(values, counts)}


def off_period_fraction(
    gaps_ms: np.ndarray,
    period_ms: float = BOARD_EMIT_PERIOD_MS,
    tolerance: float = 0.25,
) -> float:
    """Fraction of gaps that are not close to an integer multiple of the period.

    High means the delivery path is re-timing the stream — the arrivals no
    longer land where the board put them, so their spacing describes the path,
    not the board.
    """
    if gaps_ms.size == 0:
        return float("nan")
    multiples = gaps_ms / period_ms
    distance = np.abs(multiples - np.rint(multiples))
    return float(np.mean(distance > tolerance))


def lag1_conditional_mean(
    gaps_ms: np.ndarray, threshold_ms: float = LATE_FRAME_THRESHOLD_MS
) -> Dict[str, float]:
    """What follows a long gap.

    Near zero means the frames were already buffered and this host was late to
    read them. Near one emit period means the frames were genuinely not there.
    """
    if gaps_ms.size < 2:
        return {}
    late = gaps_ms[:-1] > threshold_ms
    if not np.any(late):
        return {"n_late": 0}
    following = gaps_ms[1:][late]
    return {
        "n_late": int(late.sum()),
        "threshold_ms": float(threshold_ms),
        "following_mean_ms": float(following.mean()),
        "following_p50_ms": float(np.percentile(following, 50)),
        # Reads as a verdict; both mechanisms are usually present at once, so
        # the fraction below matters more than the label.
        "reads_as": (
            "host late" if following.mean() < BOARD_EMIT_PERIOD_MS / 2 else "frames lost"
        ),
        "fraction_followed_by_short_gap": float(
            np.mean(following < BOARD_EMIT_PERIOD_MS / 2)
        ),
    }


def decode_cost_ms(table) -> Dict[str, float]:
    """Time between a frame arriving and its decoded form being available.

    Both stamps are taken on the delivery thread, so this is the decode cost
    plus whatever the interpreter did in between.
    """
    if "frame_t" not in table:
        return {}
    return distribution((table["frame_t"] - table["t"]) * 1000.0)


def slot_report(table, num_slots: int = AUTO_ENC_NUM_JOINTS) -> Dict[str, Dict]:
    """Per-slot integrity over the record.

    Parity and the chip's error bit live in the stored counts, so they are
    recomputed here rather than carried as extra columns — the same function the
    decoder uses, on the same bytes.
    """
    slot_to_joint = {slot: joint for joint, slot in JOINT_TO_ENCODER_SLOT.items()}
    raw = np.column_stack(
        [table[f"raw_s{slot:02d}"] for slot in range(num_slots)]
    ).astype(np.uint16)

    parity_ok = _check_even_parity(raw)
    angle_error = (raw & AUTO_ENC_ANGLE_ERROR_BIT) != 0
    angles = (raw & AUTO_ENC_ANGLE_MASK).astype(np.int64)
    steps = np.diff(angles, axis=0)
    steps = (steps + 8192) % 16384 - 8192

    frames = raw.shape[0]
    report = {}
    for slot in range(num_slots):
        report[f"s{slot:02d}"] = {
            "joint": slot_to_joint.get(slot),
            "frames": frames,
            "parity_bad": int(np.count_nonzero(~parity_ok[:, slot])),
            "angle_error": int(np.count_nonzero(angle_error[:, slot])),
            "impossible_jumps": int(
                np.count_nonzero(np.abs(steps[:, slot]) > MAX_PLAUSIBLE_JUMP_COUNTS)
            ),
            "counts_span": int(angles[:, slot].max() - angles[:, slot].min()),
            "constant": bool(np.all(steps[:, slot] == 0)) if frames > 1 else True,
        }
    return report


def sequence_gaps(table) -> Dict[str, int]:
    """Rows the recorder itself lost, from breaks in its own numbering."""
    seq = table["seq"].astype(np.int64)
    if seq.size < 2:
        return {"recorded": int(seq.size), "missing": 0}
    steps = np.diff(seq)
    return {
        "recorded": int(seq.size),
        "missing": int(np.sum(steps - 1)),
        "breaks": int(np.count_nonzero(steps != 1)),
    }


def watchdog_exposure(gaps_ms: np.ndarray, tiers: Dict[str, float]) -> Dict[str, int]:
    """How often the stream went stale enough to change the loop's behaviour.

    A gap wider than a tier means a running loop would have crossed it, whether
    or not a loop was running when this was recorded.
    """
    return {name: int(np.count_nonzero(gaps_ms > ms)) for name, ms in tiers.items()}


def summarise(table, tiers: Optional[Dict[str, float]] = None) -> Dict[str, object]:
    """Everything above, over one recorded stream."""
    gaps = inter_arrival_ms(table)
    from orca_core.control.constants import (
        WATCHDOG_HOLD_BASE_MS,
        WATCHDOG_HOLD_MS,
        WATCHDOG_STOP_LOOP_MS,
        WATCHDOG_WARN_MS,
    )

    if tiers is None:
        tiers = {
            "warn": WATCHDOG_WARN_MS,
            "hold": WATCHDOG_HOLD_MS,
            "hold_base": WATCHDOG_HOLD_BASE_MS,
            "stop": WATCHDOG_STOP_LOOP_MS,
        }
    return {
        "frames": len(table),
        "rate_hz": observed_rate_hz(table),
        "inter_arrival": distribution(gaps),
        "period_multiples": period_multiples(gaps),
        "off_period_fraction": off_period_fraction(gaps),
        "after_a_long_gap": lag1_conditional_mean(gaps),
        "decode_cost": decode_cost_ms(table),
        "watchdog_exposure": watchdog_exposure(gaps, tiers),
        "recorder": sequence_gaps(table),
        "slots": slot_report(table),
    }
