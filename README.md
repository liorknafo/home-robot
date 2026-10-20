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
   - KiCad 9.0.x (its bundled Python, `$KPY` in the script headers, runs the `pcb/` scripts and `ktest_*` tests): on macOS
     `/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3`, on Windows
     `C:\Program Files\KiCad\9.0\bin\python.exe` (or `%LOCALAPPDATA%\Programs\KiCad\9.0\bin\python.exe` for a per-user install), on
     Linux the system `python3` with the distribution's KiCad 9 packages.
   - Re-routing needs Java 25 and Freerouting 2.4.1 at `home-robot-r1/pcb/tools/freerouting.jar`. It isn't redistributed; download it from the Freerouting releases
     (`pcb/tools.py` pins its sha256, `FREEROUTING_SHA256`).
   - `pcb/tools.py` finds these per OS. Override any of them with `KICAD_CLI`, `KICAD_PYTHON`, `KICAD9_SYMBOL_DIR`, `KICAD9_FOOTPRINT_DIR`,
     `KICAD9_3DMODEL_DIR`, `JAVA` (or `JAVA_HOME`) and `FREEROUTING_JAR`.
   - `pcb/route.py` with no arguments reproduces the committed routing with these versions; it records them in the board's `ROUTED_WITH`.
4. **Simulator:**
   - Unreal Engine 5.8 (macOS): build with `home-robot-sim/Scripts/build.command`.
   - Sync the robot data: `python home-robot-sim/Scripts/sync_robot_data.py`. It reads `../home-robot-r1/cad` and copies the robot meshes and parameters.
   - Launch with `Scripts/launch.command house polished`.
   - Start the autonomy service: `Scripts/start-autonomy.command`, then open http://127.0.0.1:8771/autonomy.html.
   - Python deps: `home-robot-sim/controller/requirements.txt`.
   - **Windows:** every `.command` script has a `.cmd` twin in `home-robot-sim/Scripts` (`build.cmd`, `launch.cmd house polished`,
     `start-autonomy.cmd`, `run-tests.cmd`, `drive-house.cmd`, `drive-square.cmd`, `process-cameras.cmd`).
     - One command runs everything: `.\home-robot-sim\Scripts\launch.cmd house polished` starts the simulator plus, in
       minimized windows, the autonomy service and object detection, then open http://127.0.0.1:8771/autonomy.html.
       Every launcher first stops its own earlier runs, so rerunning it never leaves duplicates (`NO_AUTONOMY=1` /
       `NO_VISION=1` skip a part).
     - Needs Unreal Engine 5.8 from the Epic Games Launcher (the scripts find it on any drive through the launcher's install record,
       or set `UE_ROOT`) and
       Visual Studio 2022 17.14+ with the "Game development with C++" workload, MSVC 14.38+ and Windows SDK 10.0.22621+.
     - The scripts make `home-robot-sim\.venv` on first use and run Python in UTF-8 mode (`PYTHONUTF8=1`). Set it yourself when you
       run the Python tools directly, e.g. the CAD build, or they read the UTF-8 sources in the ANSI code page.
     - `tools/fetch_reference_models.sh` runs in Git Bash. Verified with Python 3.13 + CadQuery 2.6.1 for the CAD build.
   - **Live object recognition (display only):** started by `launch.cmd`, or alone with `start-vision.cmd`. It runs
     YOLO26n-seg on each robot camera frame on the PC's NVIDIA GPU and shows boxes + masks under the dashboard's camera
     image. By default it runs a TensorRT engine for this PC's GPU, built on the fly the first time (about 1-2 minutes,
     `vision/ensure_engine.py`; the file name records the GPU and TensorRT version, so a new GPU or TensorRT gets a fresh
     build). It falls back to PyTorch when TensorRT is unavailable, or with `NO_TRT=1`.
     `start-vision.cmd --model <weights or .engine>` picks the model: any Ultralytics name (e.g. `yolo11n-seg.pt`), or
     `yolov12n-seg.pt`, fetched from the YOLOv12 authors' release and loaded through `vision/yolov12_compat.py`
     (Ultralytics publishes no YOLO12 segmentation weights). The robot does not act on these detections.
     The pipeline is PyTorch end to end (`vision/yoloseg`, no OpenCV) and is the same one the robot runs.
   - **On the Jetson Nano (TensorRT):** [`home-robot-sim/jetson/JETSON.md`](home-robot-sim/jetson/JETSON.md) goes step by step
     from the ONNX export on the PC to the FP16 engine and the live camera view on the Nano.
5. **Tests:**
   - Simulator: `cd home-robot-sim && python -m unittest discover -s tests`, plus `tests/scenarios_round.py`.
   - Robot pure tests: `cd home-robot-r1 && python -m unittest discover -s pcb/tests -t .` (or `python -m pytest pcb/tests`), and
     `python -m unittest discover` in `cad/`. The tests that need KiCad skip when it isn't found; `R1_REQUIRE_KICAD=1` makes them run (and fail).
   - Board tests in KiCad's Python: `cd home-robot-r1 && $KPY -m unittest pcb.tests.ktest_pcb pcb.tests.ktest_power pcb.tests.ktest_route`.

## Licence

MIT (see [`LICENSE`](LICENSE)) for everything in this repo: software, CAD, board design and documents. The manufacturer 3D models fetched by `tools/fetch_reference_models.sh` keep their own terms and are not part of this repo.

## Repository maintenance

This repo is a published snapshot of two working folders that live next to it. To refresh and push it:
`tools/sync.sh && git add -A && git commit -m "…" && git push`.
