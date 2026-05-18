#!/usr/bin/env bash
# .claude-handoff/run_tests.sh
#
# Run the project_teleop unittest suite without colcon.  The
# teleop-progress skill calls this when it needs a quick pass/fail
# summary; humans can run it the same way.
#
# Usage:
#   .claude-handoff/run_tests.sh                  # full sweep
#   .claude-handoff/run_tests.sh <test_file>      # one file
#
# Exit codes:
#   0 = all tests passed
#   non-zero = at least one test file errored or failed (see output)

set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG_DIR="${REPO_ROOT}/src/dobot_mg400/mg400_controller"
PROTO_DIR="${REPO_ROOT}/src/dobot_mg400/mg400_protocol"
CORE_DIR="${REPO_ROOT}/src/robot_teaching_core"

if [[ ! -d "${PKG_DIR}" ]]; then
  echo "[run_tests.sh] expected ${PKG_DIR} — repo layout changed?" >&2
  exit 2
fi

export PYTHONPATH=".:${PROTO_DIR}:${CORE_DIR}${PYTHONPATH:+:${PYTHONPATH}}"

cd "${PKG_DIR}" || exit 2

if [[ $# -ge 1 ]]; then
  # Allow either a path (`test/test_foo.py`) or a dotted module
  # (`test.test_foo`).  Path → execute directly so unittest's main()
  # handles it; dotted → unittest module mode.
  if [[ -f "$1" ]]; then
    exec python3 "$1" -v
  else
    exec python3 -m unittest "$1" -v
  fi
fi

# Full discover sweep.  ament_copyright / ament_flake8 / ament_pep257
# need a colcon-installed environment to import (they're ROS lint
# packages).  When the script runs outside that environment, those
# three files raise ImportError and would mask real failures, so we
# discover everything else and add the three lint suites only when
# they're actually importable.
if [[ ! -d test ]]; then
  echo "[run_tests.sh] no test/ dir under ${PKG_DIR}" >&2
  exit 2
fi

LINT_TESTS=(test_copyright.py test_flake8.py test_pep257.py)
KEEP=()
for f in test/test_*.py; do
  base="$(basename "$f")"
  skip=0
  for lint in "${LINT_TESTS[@]}"; do
    [[ "$base" == "$lint" ]] && skip=1
  done
  if [[ $skip -eq 0 ]]; then
    KEEP+=("$f")
  fi
done

# Run by passing each non-lint test file as a python invocation;
# unittest discover doesn't accept globs.  Use a small driver script.
python3 - "${KEEP[@]}" <<'PYDRIVER'
"""Driver: import each test file by absolute path so the `test/`
package's lint modules (ament_copyright etc.) cannot poison the
import.  Each loaded module's TestCases are added to a single suite,
then TextTestRunner reports + propagates a non-zero exit on failure.
"""
import importlib.util
import os
import sys
import unittest

loader = unittest.TestLoader()
suite = unittest.TestSuite()
skipped = []
for path in sys.argv[1:]:
    base = os.path.splitext(os.path.basename(path))[0]
    spec = importlib.util.spec_from_file_location(base, path)
    if spec is None or spec.loader is None:
        skipped.append((path, "spec_from_file_location returned None"))
        continue
    module = importlib.util.module_from_spec(spec)
    sys.modules[base] = module
    try:
        spec.loader.exec_module(module)
    except Exception as exc:
        skipped.append((path, f"import failed: {exc}"))
        continue
    suite.addTests(loader.loadTestsFromModule(module))

if skipped:
    print("[run_tests] SKIPPED:", file=sys.stderr)
    for path, reason in skipped:
        print(f"  {path}: {reason}", file=sys.stderr)

runner = unittest.TextTestRunner(verbosity=2)
result = runner.run(suite)
sys.exit(0 if result.wasSuccessful() else 1)
PYDRIVER
exit $?
