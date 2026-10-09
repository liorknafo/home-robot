#!/bin/zsh
set -e
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ENGINE_DIR="${UE_ROOT:-/Users/Shared/Epic Games/UE_5.8}"
exec "$ENGINE_DIR/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor" "$SIM_DIR/HomeRobotSim.uproject" /Engine/Maps/Entry -game -windowed -ResX=1280 -ResY=800 -nosound "-Scenario=${1:-house}" "-FloorFinish=${2:-polished}"
