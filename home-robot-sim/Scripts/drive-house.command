#!/bin/zsh
set -e
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
python3 "$SIM_DIR/tests/house_route.py"
