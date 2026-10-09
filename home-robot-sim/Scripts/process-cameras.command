#!/bin/zsh
set -e
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
if [[ ! -x "$SIM_DIR/.venv/bin/python" ]]; then
  python3 -m venv "$SIM_DIR/.venv"
fi
if ! "$SIM_DIR/.venv/bin/python" -c 'import cv2, numpy, matplotlib' >/dev/null 2>&1; then
  "$SIM_DIR/.venv/bin/python" -m pip install -r "$SIM_DIR/controller/requirements.txt"
fi
"$SIM_DIR/.venv/bin/python" "$SIM_DIR/controller/stereo_mapping.py"
"$SIM_DIR/.venv/bin/python" "$SIM_DIR/controller/plot_vision.py"
"$SIM_DIR/.venv/bin/python" "$SIM_DIR/Scripts/build_report.py"
echo "Results are in reports/vision and simulation-review.html."
