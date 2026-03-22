Orca Core is now a library-first framework for building ORCA Hand software without cloning this repository. Install the package, load a built-in or custom profile, and use the packaged CLI for operator workflows.

## Install

```sh
pip install orca_core
```

For local development in this repo, use:

```sh
uv sync --group dev
```

## Operator Workflows

Use the packaged CLI rather than repo-local `scripts/*.py`:

```sh
orca-hand doctor --profile orcahand_v1_right
orca-hand calibrate --profile orcahand_v1_right --auto-port
orca-hand neutral --profile orcahand_v1_right --auto-port
```

Runtime state such as calibration results and last-known port is stored separately from the immutable hand profile.

## Python Usage

```python
from orca_core import OrcaHand, load_profile

profile = load_profile("orcahand_v1_right")
hand = OrcaHand(profile=profile)

success, message = hand.connect()
print(message)
if not success:
    raise SystemExit(1)

hand.enable_torque()
hand.set_joint_pos({"index_mcp": 90, "middle_pip": 30}, num_steps=25, step_size=0.001)
hand.disable_torque()
hand.disconnect()
```

To use a custom profile that lives outside the package, load it from a directory or a `config.yaml` path:

```python
from orca_core import OrcaHand, load_profile_from_path

profile = load_profile_from_path("/path/to/custom_hand_profile")
hand = OrcaHand(profile=profile)
```
