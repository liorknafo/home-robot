# R1 home robot

A round (Ø 318 mm, 155 mm tall) differential-drive home robot built around an NVIDIA Jetson Nano, together with the Unreal
simulator and the Python autonomy stack it is developed against.

| Folder | What's in it |
|---|---|
| [`home-robot-r1/`](home-robot-r1) | Robot hardware: parametric CAD (CadQuery), printable parts, the 4-layer power & I/O board (KiCad, generated from Python and auto-routed), fab package for JLCPCB, harness and BOM, bring-up guide |
| [`home-robot-sim/`](home-robot-sim) | Unreal Engine 5.8 simulator (a furnished apartment, the robot with its LiDAR, camera, IMU, encoders, bumpers and cliff sensors) and the autonomy controller (LiDAR mapping, camera low-obstacle detection, exploration, go-to) |

Status: work in progress. Nothing has been ordered or printed yet.

## Robot at a glance

- **Compute:** Jetson Nano B01 on standoffs over the motors.
- **Board:** a 74 × 100 mm 4-layer power & I/O board under the Jetson. It carries:
  - a Raspberry Pi Pico for real-time I/O;
  - two Pololu DRV8874 motor drivers;
  - a GY-521 (MPU-6050) IMU;
  - fuses, a relay and connectors.
- **Sensors:**
  - an LDROBOT LD14 (D200) 2D LiDAR at 147.5 mm;
  - one camera at 107.5 mm;
  - four downward ToF cliff sensors;
  - a front bumper.
- **Drive:** Pololu 25D 99:1 encoder gearmotors.

## Key documents

**Hardware:**
- [`home-robot-r1/BRINGUP.md`](home-robot-r1/BRINGUP.md): assembly order and first power-up.
  - **J1 pin 1 is GND.**
  - Includes the IMU calibration.
- [`home-robot-r1/pcb/fab/ORDERING.md`](home-robot-r1/pcb/fab/ORDERING.md): exact JLCPCB order settings and a verify-before-ordering checklist.
- [`home-robot-r1/pcb/fab/`](home-robot-r1/pcb/fab): Gerbers, drill files, BOM, placement file, 1:1 PDF and board STEP.
- `home-robot-r1/parts-and-connections.json` and `cable-cut-list.json`: the BOM and the harness.

**Design history:** [`home-robot-sim/docs/superpowers/`](home-robot-sim/docs/superpowers) holds the specs and plans.

## What is not in this repo

- **Manufacturer 3D models:** NVIDIA Jetson Nano, Pololu motor and driver, Raspberry Pi Pico, LDROBOT LD14.
  - Their licences don't clearly allow redistribution, so they are fetched instead.
  - The same goes for the generated meshes that contain their geometry: `cad/assembly-meshes.json`, `cad/home-robot-r1.glb`, `design-review.html`, and the simulator's `Content/RobotData/assembly-meshes.json`.
  - Sources and checksums are in [`home-robot-r1/reference-models/SOURCE.txt`](home-robot-r1/reference-models/SOURCE.txt).
- **Simulator data:** recordings (`Saved/`), build output, caches and benchmark reports.

## Build

1. **Fetch the reference models:** `tools/fetch_reference_models.sh`. The NVIDIA Jetson model needs a manual download; the script prints the steps.
2. **Robot CAD:**
   - Python 3.14 with CadQuery 2.8 (`home-robot-r1/cad/requirements.txt`; 2.8.0 + cadquery-ocp 7.9.3.1.1 verified).
   - Run `python home-robot-r1/cad/build_robot.py`. It takes about 25 min, and writes the meshes, STEP, print STLs and validation report.
3. **Board (optional):**
   - KiCad 9 (its bundled Python runs the `pcb/` scripts and `ktest_*` tests).
   - Re-routing needs Java and Freerouting 2.x at `home-robot-r1/pcb/tools/freerouting.jar`. It isn't redistributed; download it from the Freerouting releases.
   - `pcb/route.py` with no arguments reproduces the committed routing.
4. **Simulator:**
   - Unreal Engine 5.8 (macOS): build with `home-robot-sim/Scripts/build.command`.
   - Sync the robot data: `python home-robot-sim/Scripts/sync_robot_data.py`. It reads `../home-robot-r1/cad` and copies the robot meshes and parameters.
   - Launch with `Scripts/launch.command house polished`.
   - Start the autonomy service: `Scripts/start-autonomy.command`, then open http://127.0.0.1:8771/autonomy.html.
   - Python deps: `home-robot-sim/controller/requirements.txt`.
5. **Tests:**
   - Simulator: `cd home-robot-sim && python -m unittest discover -s tests`, plus `tests/scenarios_round.py`.
   - Robot pure tests: `python -m unittest discover` in `home-robot-r1/pcb/tests` and `cad/`.

## Licence

MIT (see [`LICENSE`](LICENSE)) for everything in this repo: software, CAD, board design and documents. The manufacturer 3D models fetched by `tools/fetch_reference_models.sh` keep their own terms and are not part of this repo.

## Repository maintenance

This repo is a published snapshot of two working folders that live next to it. To refresh and push it:
`tools/sync.sh && git add -A && git commit -m "…" && git push`.
