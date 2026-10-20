"""tools/check_downstream.py: the import walker."""

import importlib.util
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_tool():
    path = REPO_ROOT / "tools" / "check_downstream.py"
    spec = importlib.util.spec_from_file_location("_check_downstream", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["_check_downstream"] = module
    spec.loader.exec_module(module)
    return module


tool = _load_tool()


def test_unresolved_imports_are_reported_even_inside_functions(tmp_path):
    (tmp_path / "bench_server.py").write_text(
        "from orca_core import load_hand\n"
        "def handler():\n"
        "    if True:\n"
        "        from orca_core.hand_factory import no_such_name\n"
        "    import orca_core.no_such_module\n"
    )

    failures = tool.check_imports(tmp_path)

    assert sorted((f.lineno, f.module, f.name) for f in failures) == [
        (4, "orca_core.hand_factory", "no_such_name"),
        (5, "orca_core.no_such_module", None),
    ]
