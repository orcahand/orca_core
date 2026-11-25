"""setup_rig.sh and rig-console.sh on a fake layout of checkouts, under macOS's bash 3.2: the
checkouts and branches it finds, where it stops, and the paths the console starts with."""
import os
import shutil
import subprocess
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
BASH = "/bin/bash" if Path("/bin/bash").exists() else shutil.which("bash")
BRANCHES = {"orca_core": "internal/rig-calibration", "orca_ui": "internal/rig-calibration",
            "orca_firmware": "internal/board-calibration-store", "orcahand_description": "main"}

pytestmark = pytest.mark.skipif(BASH is None or shutil.which("git") is None, reason="needs bash and git")


def layout(tmp_path, branches=BRANCHES, skip=()):
    """Checkouts side by side (empty git repos on their branches), the scripts in orca_core's."""
    for name, branch in branches.items():
        if name in skip:
            continue
        d = tmp_path / name
        d.mkdir()
        subprocess.run(["git", "init", "-q", "-b", branch, str(d)], check=True)
    exp = tmp_path / "orca_core" / "experiments" / "visual_calibration"
    exp.mkdir(parents=True)
    for script in ("setup_rig.sh", "rig-console.sh"):
        shutil.copy(HERE / script, exp / script)
    if "orca_ui" not in skip:
        (tmp_path / "orca_ui" / "dev").write_text("#!/bin/sh\nexit 0\n")
    return exp


def fake_tool(bin_dir: Path, name: str, body: str) -> None:
    bin_dir.mkdir(parents=True, exist_ok=True)
    (bin_dir / name).write_text("#!/bin/sh\n" + body + "\n")
    (bin_dir / name).chmod(0o755)


def run(script, tmp_path, *args, env=None):
    e = {k: v for k, v in os.environ.items() if not k.startswith(("ORCA_", "ORCAHAND"))}
    e["PATH"] = f"{tmp_path / 'bin'}:{e['PATH']}"
    return subprocess.run([BASH, str(script), *args], capture_output=True, text=True, env={**e, **(env or {})},
                          timeout=60)


def test_both_scripts_parse_under_bash_3_2_and_setup_explains_itself():
    for script in ("setup_rig.sh", "rig-console.sh"):
        assert subprocess.run([BASH, "-n", str(HERE / script)]).returncode == 0
        assert os.access(HERE / script, os.X_OK)
    out = subprocess.run([BASH, str(HERE / "setup_rig.sh"), "--help"], capture_output=True, text=True)
    assert out.returncode == 0 and "--no-firmware" in out.stdout and "rig-console.sh" in out.stdout
    bad = subprocess.run([BASH, str(HERE / "setup_rig.sh"), "--nope"], capture_output=True, text=True)
    assert bad.returncode == 2 and "unknown option --nope" in bad.stderr


def test_setup_stops_at_a_missing_checkout(tmp_path):
    exp = layout(tmp_path, skip=("orca_ui",))
    fake_tool(tmp_path / "bin", "uv", 'echo "uv 0.0.0"')
    r = run(exp / "setup_rig.sh", tmp_path)
    assert r.returncode == 1 and f"orca_ui: no git checkout at {tmp_path / 'orca_ui'}" in r.stderr


@pytest.mark.parametrize("repo", ["orca_core", "orca_ui"])
def test_orca_core_or_orca_ui_on_another_branch_stops_before_anything_changes(tmp_path, repo):
    exp = layout(tmp_path, branches={**BRANCHES, repo: "main"})
    fake_tool(tmp_path / "bin", "uv", f'echo "$*" >> {tmp_path / "uv.calls"}; echo "uv 0.0.0"')
    r = run(exp / "setup_rig.sh", tmp_path)
    assert r.returncode == 1 and "is on 'main', not internal/rig-calibration; nothing was changed" in r.stderr
    assert f"git -C {tmp_path / repo} switch internal/rig-calibration" in r.stderr
    assert (tmp_path / "uv.calls").read_text().split() == ["--version"]        # no uv sync, no install


def test_the_optional_checkouts_on_another_branch_are_warnings(tmp_path):
    exp = layout(tmp_path, branches={**BRANCHES, "orcahand_description": "v1-fix"}, skip=("orca_firmware",))
    fake_tool(tmp_path / "bin", "uv", 'case "$1" in --version) echo "uv 0.0.0";; *) exit 1;; esac')
    r = run(exp / "setup_rig.sh", tmp_path, "--no-firmware")
    assert f"ok    orca_core at {tmp_path / 'orca_core'} (internal/rig-calibration)" in r.stdout
    assert "WARN  orcahand_description at" in r.stdout and "is on 'v1-fix', not main" in r.stdout
    assert "WARN  orca_firmware: no git checkout" in r.stdout
    assert r.returncode == 1 and "uv sync in" in r.stderr                    # went on to the environment


def test_setup_without_uv_says_how_to_install_it(tmp_path):
    exp = layout(tmp_path)
    r = run(exp / "setup_rig.sh", tmp_path, env={"PATH": "/usr/bin:/bin"})
    if shutil.which("uv", path="/usr/bin:/bin"):
        pytest.skip("uv is installed system-wide")
    assert r.returncode == 1 and "uv not found: curl -LsSf https://astral.sh/uv/install.sh" in r.stderr


def test_the_console_starts_from_orca_uis_environment_with_the_rigs_paths(tmp_path):
    exp = layout(tmp_path)
    r = run(exp / "rig-console.sh", tmp_path)
    assert r.returncode == 1 and "run" in r.stderr and "setup_rig.sh first" in r.stderr
    venv = tmp_path / "orca_ui" / ".venv" / "bin"
    core = tmp_path / "orca_core" / "orca_core"
    core.mkdir()
    fake_tool(venv, "python", f'[ "$1" = -c ] && {{ echo "${{FAKE_CORE:-{core.resolve()}}}"; exit 0; }}\n'
                              'echo "pwd=$(pwd) args=$*"; env | grep -E "^ORCA" | sort')
    r = run(exp / "rig-console.sh", tmp_path, env={"FAKE_CORE": "/site-packages/orca_core"})
    assert r.returncode == 1 and "imports orca_core from '/site-packages/orca_core'" in r.stderr
    assert "re-run" in r.stderr and "setup_rig.sh" in r.stderr and "pwd=" not in r.stdout
    r = run(exp / "rig-console.sh", tmp_path, "--research-ui", env={"ORCA_FIRMWARE_REPO": "/elsewhere"})
    assert r.returncode == 0
    assert f"pwd={(tmp_path / 'orca_ui').resolve()} args=-m orca_ui --research-ui" in r.stdout
    assert f"ORCA_VISUAL_CALIBRATION={exp}" in r.stdout
    assert f"ORCAHAND_DESCRIPTION={tmp_path / 'orcahand_description'}" in r.stdout
    assert "ORCA_FIRMWARE_REPO" not in r.stdout                  # no cal_store.py there: no "Write to board"
    (tmp_path / "orca_firmware" / "tools" / "bench").mkdir(parents=True)
    (tmp_path / "orca_firmware" / "tools" / "bench" / "cal_store.py").write_text("")
    r = run(exp / "rig-console.sh", tmp_path)
    assert f"ORCA_FIRMWARE_REPO={tmp_path / 'orca_firmware'}" in r.stdout
