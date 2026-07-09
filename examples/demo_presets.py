"""Canned demo poses and the sequencer that plays them.

Poses are expressed as fractions of each joint's ROM, so one preset works
across hand models with different ranges.
"""

from typing import Final

from orca_core.constants import NUM_STEPS, STEP_SIZE


DEMO_POSE_FRACTIONS: Final[dict[str, dict[str, dict[str, float]]]] = {
    "main": {
        "open_hand": {
            "thumb_cmc": 0.70,
            "thumb_abd": 0.80,
            "thumb_mcp": 0.85,
            "thumb_dip": 0.75,
            "index_abd": 0.10,
            "middle_abd": 0.50,
            "ring_abd": 0.70,
            "pinky_abd": 0.85,
            "index_mcp": 0.15,
            "middle_mcp": 0.15,
            "ring_mcp": 0.15,
            "pinky_mcp": 0.15,
            "index_pip": 0.10,
            "middle_pip": 0.10,
            "ring_pip": 0.10,
            "pinky_pip": 0.10,
            "wrist": 0.30,
        },
        "power_grasp": {
            "thumb_cmc": 0.35,
            "thumb_abd": 0.55,
            "thumb_mcp": 0.20,
            "thumb_dip": 0.85,
            "index_mcp": 0.85,
            "middle_mcp": 0.85,
            "ring_mcp": 0.85,
            "pinky_mcp": 0.85,
            "index_pip": 0.90,
            "middle_pip": 0.90,
            "ring_pip": 0.90,
            "pinky_pip": 0.90,
            "wrist": 0.55,
        },
        "pinch": {
            "thumb_cmc": 0.45,
            "thumb_abd": 0.65,
            "thumb_mcp": 0.40,
            "thumb_dip": 0.75,
            "index_abd": 0.30,
            "index_mcp": 0.70,
            "index_pip": 0.75,
            "middle_mcp": 0.30,
            "middle_pip": 0.20,
            "ring_mcp": 0.20,
            "ring_pip": 0.15,
            "pinky_mcp": 0.20,
            "pinky_pip": 0.15,
            "wrist": 0.45,
        },
    },
    "abduction": {
        "fan_out": {
            "thumb_abd": 0.85,
            "index_abd": 0.10,
            "middle_abd": 0.50,
            "ring_abd": 0.80,
            "pinky_abd": 0.90,
            "wrist": 0.40,
        },
        "fan_in": {
            "thumb_abd": 0.25,
            "index_abd": 0.85,
            "middle_abd": 0.50,
            "ring_abd": 0.20,
            "pinky_abd": 0.15,
            "wrist": 0.55,
        },
        "spread_grasp": {
            "thumb_cmc": 0.55,
            "thumb_abd": 0.70,
            "thumb_mcp": 0.45,
            "thumb_dip": 0.65,
            "index_abd": 0.15,
            "middle_abd": 0.50,
            "ring_abd": 0.80,
            "pinky_abd": 0.90,
            "index_mcp": 0.65,
            "middle_mcp": 0.65,
            "ring_mcp": 0.65,
            "pinky_mcp": 0.65,
            "index_pip": 0.70,
            "middle_pip": 0.70,
            "ring_pip": 0.70,
            "pinky_pip": 0.70,
        },
    },
}


DEMO_SEQUENCES: Final[dict[str, tuple[str, ...]]] = {
    "main": ("open_hand", "power_grasp", "pinch"),
    "abduction": ("fan_out", "fan_in", "spread_grasp"),
}


def run_demo(
    hand,
    demo_name: str = "main",
    cycles: int = 1,
    num_steps: int = NUM_STEPS,
    step_size: float = STEP_SIZE,
    return_to_neutral: bool = True,
) -> tuple[str, ...]:
    """Play a named demo sequence on *hand* and return the pose names played."""
    if demo_name not in DEMO_POSE_FRACTIONS or demo_name not in DEMO_SEQUENCES:
        available = ", ".join(sorted(DEMO_SEQUENCES))
        raise ValueError(f"Unknown demo '{demo_name}'. Available demos: {available}.")

    poses = {
        name: hand.pose_from_fractions(fractions)
        for name, fractions in DEMO_POSE_FRACTIONS[demo_name].items()
    }
    sequence = DEMO_SEQUENCES[demo_name]

    for _ in range(cycles):
        for name in sequence:
            hand.set_joint_positions(poses[name], num_steps=num_steps, step_size=step_size)

        if return_to_neutral:
            hand.set_neutral_position(num_steps=num_steps, step_size=step_size)

    return sequence
