<div align="center" style="line-height: 1;">
  <a href="https://arxiv.org/abs/2504.04259" target="_blank"><img alt="arXiv" src="https://img.shields.io/badge/arXiv-2504.04259-B31B1B?logo=arxiv"/></a>
  <a href="https://discord.gg/xvGyxaccRa" target="_blank"><img alt="Discord" src="https://img.shields.io/badge/Discord-orcahand-7289da?logo=discord&logoColor=white&color=7289da"/></a>
  <a href="https://x.com/orcahand" target="_blank"><img alt="Twitter Follow" src="https://img.shields.io/twitter/follow/orcahand?style=social"/></a>
  <a href="https://orcahand.com" target="_blank"><img alt="Website" src="https://img.shields.io/badge/Website-orcahand.com-blue?style=flat&logo=google-chrome"/></a>
  <br>
  <a href="https://github.com/orcahand/orca_core" target="_blank"><img alt="GitHub stars" src="https://img.shields.io/github/stars/orcahand/orca_core?style=social"/></a>
  <a href="https://github.com/orcahand/orca_core/actions/workflows/test.yml" target="_blank"><img alt="Tests" src="https://github.com/orcahand/orca_core/actions/workflows/test.yml/badge.svg"/></a>
</div>

Orca Core (`pip install orca_core`) is the core control package of the ORCA Hand ✋ 
Across the entire stack, `orca_core` is the unified entry point used to abstract hardware, provide scripts for hand calibration, controlling the hand, and more.

It is designed to be readily extensible and easily hackable. 
Don't be a bystander! If you see something that does not work, [open an issue](https://github.com/orcahand/orca_core/issues/new) or [submit a PR with your fix](https://github.com/orcahand/orca_core/compare)---`orca_core` is open source! ❤️

## Getting started

### Install

First, create a virtual environment.
We tested installs with Python 3.12, and newer versions of python might be less stable.

You can use `uv` ([installing uv](https://docs.astral.sh/uv/getting-started/installation/)) for virtualization and handle dependencies.

```sh
uv venv .venv --python 3.12
source .venv/bin/activate
uv pip install orca_core
```

Alternatively, you can use `conda` ([installing conda](https://www.anaconda.com/docs/getting-started/miniconda/install/overview)).

```sh
conda create -n orca python=3.12 -y
conda activate orca
pip install orca_core
```

We are constantly iterating on `orca_core`. If you want to keep up with the last and greatest, build `orca_core` from source.
```sh
uv pip install git+https://github.com/orcahand/orca_core.git
```

> [!WARNING]
> This will install the latest version from the main branch, and we are iterating fast. If you need stability, consider installing our latest stable release from PyPI.

### Connecting to the hand

[ ] TODO: both CLI and Python API

### Running calibration

[ ] TODO: both CLI and Python API

### Running tensioning

[ ] TODO: both CLI and Python API

### Running a sanity check: reaching neutral position

[ ] TODO: both CLI and Python API

That's it! You can now use ORCA hand 🙌

If you are running into issues with your installation, go ahead and [open an issue](https://github.com/orcahand/orca_core/issues/new).
If you have questions or are running into issues, feel free to reach out to us on [Discord](https://discord.gg/xvGyxaccRa) or [X](https://x.com/orcahand).

---

Made with ❤️ by the ORCA team