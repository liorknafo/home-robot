@echo off
rem Live YOLO detection + segmentation of the robot camera for the dashboard (display only). Arguments pass through,
rem default YOLO26n-seg; e.g. --model yolov12n-seg.pt. Needs an NVIDIA GPU for real time; the first run downloads PyTorch
rem (~3 GB) and the weights.
setlocal
for %%I in ("%~dp0..") do set "SIM_DIR=%%~fI"
set "VPY=%SIM_DIR%\vision\.venv\Scripts\python.exe"
set "PYTHONUTF8=1"
if not exist "%VPY%" (
  py -3 -m venv "%SIM_DIR%\vision\.venv" || python -m venv "%SIM_DIR%\vision\.venv" || exit /b 1
)
"%VPY%" -c "import torch, ultralytics" >nul 2>&1 || "%VPY%" -m pip install -r "%SIM_DIR%\vision\requirements.txt" || exit /b 1
rem One vision service at a time (port 8772).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0_stop-previous.ps1" vision || exit /b 1
"%VPY%" "%SIM_DIR%\vision\vision_service.py" %*
