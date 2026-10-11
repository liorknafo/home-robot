#!/usr/bin/env bash
# Refresh this public monorepo from the two working folders next to it:
#   ../home-robot-r1   (git repo: CAD, power & I/O board, harness, docs)  -> home-robot-r1/
#   ../home-robot-sim  (Unreal simulator + Python autonomy controller)    -> home-robot-sim/
# The two folders stay siblings here too, so home-robot-sim/Scripts/sync_robot_data.py still finds ../home-robot-r1/cad.
#
# Left out on purpose (see README "What is not in this repo"):
#   - manufacturer 3D models (NVIDIA, Pololu, Raspberry Pi, LDROBOT) and the generated meshes that contain their geometry;
#   - simulator recordings, logs, build output, caches and benchmark reports.
# Usage: tools/sync.sh [R1_REF]   (R1_REF defaults to the R1 repo's current HEAD; only committed files are exported)
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
src="$(cd "$here/.." && pwd)"
r1_src="$src/home-robot-r1"; sim_src="$src/home-robot-sim"
ref="${1:-HEAD}"
[ -d "$r1_src/.git" ] || { echo "missing $r1_src (git repo)" >&2; exit 1; }
[ -d "$sim_src" ] || { echo "missing $sim_src" >&2; exit 1; }

# 1. R1: committed files only, from git (no untracked scratch, no .superpowers workspace).
rm -rf "$here/home-robot-r1"; mkdir -p "$here/home-robot-r1"
git -C "$r1_src" archive "$ref" | tar -x -C "$here/home-robot-r1"
( cd "$here/home-robot-r1"
  # Manufacturer models: keep only the provenance note; tools/fetch_reference_models.sh downloads them.
  find reference-models -mindepth 1 ! -name SOURCE.txt -exec rm -rf {} + 2>/dev/null || true
  # Generated meshes that embed manufacturer geometry (rebuilt by cad/build_robot.py after the fetch).
  rm -f cad/assembly-meshes.json cad/home-robot-r1.glb design-review.html
)
echo "R1 exported from $(git -C "$r1_src" rev-parse --short "$ref") ($(git -C "$r1_src" rev-parse --abbrev-ref HEAD))" >&2

# 2. Simulator: everything except recordings, build output, caches, reports and the robot meshes.
mkdir -p "$here/home-robot-sim"
rsync -a --delete \
  --exclude '/Saved/' --exclude '/Intermediate/' --exclude '/DerivedDataCache/' --exclude '/Binaries/' \
  --exclude '/.git' --exclude '/.git/' --exclude '/.sim-lock/' \
  --exclude '/.venv/' --exclude '__pycache__/' --exclude '*.pyc' --exclude '.DS_Store' --exclude '*.log' \
  --exclude '/reports/' --exclude '/Content/RobotData/assembly-meshes.json' \
  "$sim_src/" "$here/home-robot-sim/"
mkdir -p "$here/home-robot-sim/reports" && touch "$here/home-robot-sim/reports/.gitkeep"

# 3. Guard: nothing over GitHub's 100 MB file limit, and no manufacturer model slipped through.
big=$(find "$here" -path "$here/.git" -prune -o -type f -size +95M -print)
[ -z "$big" ] || { echo "files too large for GitHub:" >&2; echo "$big" >&2; exit 1; }
leak=$(find "$here/home-robot-r1/reference-models" "$here/home-robot-sim" -type f \( -iname '*.stp' -o -iname '*jetson*' -o -iname 'LD14*' -o -iname 'pololu-*' -o -iname 'Pico-R3*' -o -iname '25d-*' \) 2>/dev/null)
[ -z "$leak" ] || { echo "manufacturer model found, not syncing:" >&2; echo "$leak" >&2; exit 1; }
echo "sync done: $(du -sh "$here" --exclude .git 2>/dev/null | cut -f1 || true)" >&2
