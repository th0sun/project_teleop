#!/usr/bin/env bash
# Launch the MG400 3-D Simulator
# Usage: ./run.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Create venv if missing
if [ ! -f ".venv/bin/python3" ]; then
    echo "[setup] Creating virtual environment..."
    python3 -m venv .venv
    .venv/bin/pip install -q -r requirements.txt
    echo "[setup] Done."
fi

exec .venv/bin/python3 main.py "$@"
