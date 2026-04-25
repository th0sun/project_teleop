"""Import-discipline guard (R14).

The ``teaching_core`` package MUST NOT import from any robot-specific
adapter (``adapters.*``) or from a robot-specific runtime module
(``mg400_*``, ``ur_*``, ``franka_*``, ``dobot_*``, ``Dobot_*``).

This test scans every .py file under teaching_core/ and inspects the
``import``/``from ... import`` statements. If a forbidden symbol is
found, the test fails with the offending file and line.

This is the seam guard the proposal calls "import-linter test in CI"
for finding R14 (lifter quietly imports MG400 FK).
"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path
from typing import Iterable, List, Tuple


# Roots that the core layer must not depend on.
FORBIDDEN_TOP_LEVELS = (
    "adapters",
    "mg400_controller",
    "Dobot_TCP_IP_Python_V4",
    "MG400_Mock",
    "ur_robot_driver",
    "franka",
    "libfranka",
)


def _core_root() -> Path:
    # this file: <pkg>/test/test_import_discipline.py
    return Path(__file__).resolve().parent.parent / "teaching_core"


def _iter_py_files(root: Path) -> Iterable[Path]:
    for p in root.rglob("*.py"):
        if "__pycache__" in p.parts:
            continue
        yield p


def _imports_in_file(path: Path) -> List[Tuple[int, str]]:
    """Return (lineno, top-level module name) for every import in the file."""
    found: List[Tuple[int, str]] = []
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                top = alias.name.split(".", 1)[0]
                found.append((node.lineno, top))
        elif isinstance(node, ast.ImportFrom):
            if node.level and node.level > 0:
                # Relative import; never crosses package boundaries.
                continue
            if node.module is None:
                continue
            top = node.module.split(".", 1)[0]
            found.append((node.lineno, top))
    return found


class TestCoreImportDiscipline(unittest.TestCase):

    def test_core_does_not_import_adapters(self) -> None:
        violations: List[str] = []
        root = _core_root()
        self.assertTrue(root.is_dir(), f"core dir missing: {root}")
        for path in _iter_py_files(root):
            for lineno, top in _imports_in_file(path):
                if top in FORBIDDEN_TOP_LEVELS:
                    violations.append(
                        f"{path.relative_to(root.parent)}:{lineno}: "
                        f"forbidden import of '{top}'"
                    )
        if violations:
            self.fail(
                "teaching_core imports a forbidden adapter/runtime module:\n  "
                + "\n  ".join(violations)
            )


if __name__ == "__main__":
    unittest.main()
