#!/bin/zsh
set -e
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ENGINE_DIR="${UE_ROOT:-/Users/Shared/Epic Games/UE_5.8}"
exec "$ENGINE_DIR/Engine/Build/BatchFiles/Mac/Build.sh" HomeRobotSimEditor Mac Development "$SIM_DIR/HomeRobotSim.uproject" -WaitMutex -NoHotReloadFromIDE -MaxParallelActions=4 -NoUBA
