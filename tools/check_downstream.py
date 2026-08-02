# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Maintainer tool: check sibling repos against this orca_core working tree.

orca_core ships to PyPI and is consumed by sibling repos that live beside it.
Their code is invisible to our own test suite, so a symbol that looks unused
here may be load-bearing there. This tool answers two questions:

  * ``--imports`` (default): does every ``orca_core`` import in a sibling still
    resolve against this working tree? Fast, no installs.
  * ``--run-tests``: do the siblings' own suites still pass against this tree?
    Uses an ephemeral editable overlay, so their venvs are never modified.

  * ``--symbol NAME``: who outside orca_core references NAME? Run this before
    deleting anything public — a green test run does not prove a symbol is dead.

Siblings that are not checked out are skipped and reported as skipped, so this
works for anyone with only orca_core (and orca_firmware, which is private).

Usage:
    uv run python tools/check_downstream.py
    uv run python tools/check_downstream.py --run-tests
    uv run python tools/check_downstream.py --symbol MockOrcaHand
"""
from __future__ import annotations

import argparse
import ast
import importlib
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

ORCA_CORE_ROOT = Path(__file__).resolve().parent.parent

# Sibling repos that import orca_core, mapped to the uv flags their suite needs.
# None means import-check only: orca_teleop's suite hangs partway and pulls a
# multi-gigabyte CUDA toolchain first, so only its imports are checked here.
DOWNSTREAM = (
    ("orca_ui", []),
    ("orca_teleop", None),
    ("orca_firmware", None),
    ("orca_stress_tests", None),
    ("orca_ros", None),
)

SKIP_DIRS = {"node_modules", "__pycache__", "dist", "build"}


@dataclass
class Unresolved:
    path: Path
    lineno: int
    module: str
    name: str | None

    def __str__(self) -> str:
        target = f"{self.module}.{self.name}" if self.name else self.module
        return f"{self.path}:{self.lineno}  {target}"


def python_files(root: Path):
    """Yield the repo's own sources, skipping vendored and hidden trees."""
    for path in root.rglob("*.py"):
        parts = path.relative_to(root).parts
        if any(part.startswith(".") or part in SKIP_DIRS for part in parts):
            continue
        yield path


def orca_core_imports(path: Path):
    """Yield (lineno, module, name) for every orca_core import in *path*."""
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (SyntaxError, UnicodeDecodeError):
        return
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.level or not node.module or not node.module.startswith("orca_core"):
                continue
            for alias in node.names:
                yield node.lineno, node.module, alias.name
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.startswith("orca_core"):
                    yield node.lineno, alias.name, None


def check_imports(repo: Path) -> list[Unresolved]:
    """Resolve every orca_core import in *repo* against the local working tree."""
    failures = []
    for path in python_files(repo):
        for lineno, module, name in orca_core_imports(path):
            try:
                mod = importlib.import_module(module)
            except Exception:
                failures.append(Unresolved(path, lineno, module, None))
                continue
            # `from pkg import sub` may name a submodule rather than an attribute.
            if name and name != "*" and not hasattr(mod, name):
                try:
                    importlib.import_module(f"{module}.{name}")
                except Exception:
                    failures.append(Unresolved(path, lineno, module, name))
    return failures


def run_tests(repo: Path, uv_args: list) -> bool:
    """Run *repo*'s suite against this working tree via an ephemeral overlay."""
    cmd = ["uv", "run"]
    if (repo / ".venv").is_dir():
        cmd.append("--no-sync")
    else:
        cmd += uv_args
    cmd += ["--with-editable", str(ORCA_CORE_ROOT), "python", "-m", "pytest", "-q"]
    print(f"  $ {' '.join(cmd)}", flush=True)
    return subprocess.run(cmd, cwd=repo).returncode == 0


def find_symbol(repo: Path, symbol: str) -> list[str]:
    result = subprocess.run(
        ["git", "grep", "-n", "-w", symbol],
        cwd=repo,
        capture_output=True,
        text=True,
    )
    return [line for line in result.stdout.splitlines() if line.strip()]


def resolve_repos(repos_root: Path) -> tuple[list, list[str]]:
    present, missing = [], []
    for name, test_args in DOWNSTREAM:
        path = repos_root / name
        if path.is_dir():
            present.append((path, test_args))
        else:
            missing.append(name)
    return present, missing


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-tests", action="store_true", help="also run each sibling's test suite")
    parser.add_argument("--symbol", help="report references to a symbol instead of checking imports")
    parser.add_argument(
        "--repos-root",
        type=Path,
        default=ORCA_CORE_ROOT.parent,
        help="directory holding the sibling checkouts (default: orca_core's parent)",
    )
    args = parser.parse_args()

    present, missing = resolve_repos(args.repos_root)
    if not present:
        print(f"No sibling repos found under {args.repos_root} — nothing to check.")
        return 0

    if args.symbol:
        total = 0
        for repo, _ in present:
            hits = find_symbol(repo, args.symbol)
            total += len(hits)
            print(f"\n=== {repo.name} — {len(hits)} reference(s) ===")
            for hit in hits:
                print(f"  {hit}")
        if missing:
            print(f"\nSkipped (not checked out): {', '.join(missing)}")
        print(f"\n{args.symbol}: {total} reference(s) outside orca_core.")
        return 0

    failed = []
    for repo, test_args in present:
        print(f"\n=== {repo.name} ===", flush=True)
        unresolved = check_imports(repo)
        if unresolved:
            failed.append(repo.name)
            print(f"  {len(unresolved)} unresolved orca_core import(s):")
            for item in unresolved:
                print(f"    {item}")
        else:
            print("  imports: OK")

        if args.run_tests:
            if test_args is None:
                print("  tests: none")
            elif not run_tests(repo, test_args):
                failed.append(repo.name)

    if missing:
        print(f"\nSkipped (not checked out): {', '.join(missing)}")
    if failed:
        print(f"\nFAILED: {', '.join(sorted(set(failed)))}")
        return 1
    print("\nAll checked downstream repos are consistent with this working tree.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
