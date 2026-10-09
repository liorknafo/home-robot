#!/bin/zsh
set -e
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
exec python3 "$SIM_DIR/tests/run_physics.py"
