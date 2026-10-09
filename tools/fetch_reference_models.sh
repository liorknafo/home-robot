#!/usr/bin/env bash
# Download the manufacturer 3D models the CAD build imports. They are not redistributed in this repo:
# their licences don't clearly allow it. Provenance and checksums: home-robot-r1/reference-models/SOURCE.txt
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
dst="$here/home-robot-r1/reference-models"; tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT
mkdir -p "$dst"
get() { echo "fetch $2" >&2; curl -fsSL "$1" -o "$2"; }

# Pololu 25D 99:1 gearmotor with encoder (zip, one file used)
get https://www.pololu.com/file/0J1647/25d-gearmotor-3d-models.zip "$tmp/25d.zip"
unzip -o -j -q "$tmp/25d.zip" '*25d-metal-gearmotor-75-99-encoder.step' -d "$dst" || \
  unzip -o -j -q "$tmp/25d.zip" '*75-99*encoder*.step' -d "$dst"

# LDROBOT LD14 LiDAR (Waveshare D200 kit), checked against SOURCE.txt
get https://files.waveshare.com/upload/9/98/LD14.zip "$tmp/LD14.zip"
echo "bfee1fc762aceba867fee2075ba8a1c2f82bbe91937ae6fe5953a9bd2d0da1e5  $tmp/LD14.zip" | shasum -a 256 -c -
unzip -o -j -q "$tmp/LD14.zip" '*LD14.step' -d "$dst"

# Raspberry Pi Pico R3
get https://datasheets.raspberrypi.com/pico/Pico-R3-step.zip "$dst/pico-r3-step.zip"
mkdir -p "$dst/pico-r3" && unzip -o -j -q "$dst/pico-r3-step.zip" '*.step' -d "$dst/pico-r3"

# Pololu DRV8874 carrier (4035) and D24V50F5 regulator (reference only)
get https://www.pololu.com/file/0J1883/drv887x-single-brushed-dc-motor-driver-carrier.step "$dst/pololu-4035-drv8874-carrier.step"
get https://www.pololu.com/file/0J1437/d24v50f5-step-down-voltage-regulator.step "$dst/pololu-2851-d24v50f5.step"

cat >&2 <<'EOF'

Manual step (NVIDIA needs a developer login, so it can't be scripted):
  1. Download "Jetson Nano Developer Kit 3D model (B01)" from the NVIDIA Jetson Download Center.
  2. Save it as home-robot-r1/reference-models/Jetson_Nano_Dev_Kit_3D_b01.zip
     (expected SHA-256 73c30bf5712d177f735cab336bff24e518c864cd6f6070b90ee184ad229ac604).
  3. Unzip the .stp into home-robot-r1/reference-models/jetson-nano-b01/ and run
     python home-robot-r1/cad/simplify_jetson.py   (writes jetson-nano-b01-simplified.step/.json)
Then build the robot CAD (README "Build").
EOF
