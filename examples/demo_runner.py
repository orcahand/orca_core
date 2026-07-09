"""Play a packaged demo sequence on a hand.

The poses themselves ship with orca_core
(:func:`orca_core.demo_poses.load_demo_poses`)
"""

from orca_core.constants import NUM_STEPS, STEP_SIZE
from orca_core.demo_poses import load_demo_poses


def run_demo(
    hand,
    demo_name: str = "main",
    cycles: int = 1,
    num_steps: int = NUM_STEPS,
    step_size: float = STEP_SIZE,
    return_to_neutral: bool = True,
) -> tuple[str, ...]:
    """Play a named demo sequence on *hand* and return the pose names played."""
    demos = load_demo_poses()
    if demo_name not in demos:
        available = ", ".join(sorted(demos))
        raise ValueError(f"Unknown demo '{demo_name}'. Available demos: {available}.")

    demo = demos[demo_name]
    poses = {
        name: hand.pose_from_fractions(fractions)
        for name, fractions in demo.pose_fractions.items()
    }

    for _ in range(cycles):
        for name in demo.sequence:
            hand.set_joint_positions(poses[name], num_steps=num_steps, step_size=step_size)

        if return_to_neutral:
            hand.set_neutral_position(num_steps=num_steps, step_size=step_size)

    return demo.sequence
