@echo off
rem Live YOLO detection + segmentation of the robot camera for the dashboard (display only). Arguments pass through.
rem Default: YOLO26n-seg as a TensorRT engine for this PC's GPU, built on the fly the first time (about 1-2 minutes;
rem vision\ensure_engine.py). Falls back to PyTorch when TensorRT or the build is not available, or with NO_TRT=1.
rem --model <weights or .engine> picks the model yourself, e.g. --model yolov12n-seg.pt (PyTorch).
rem Needs an NVIDIA GPU for real time; the first run downloads PyTorch, TensorRT (~5 GB together) and the weights.
setlocal
for %%I in ("%~dp0..") do set "SIM_DIR=%%~fI"
set "VPY=%SIM_DIR%\vision\.venv\Scripts\python.exe"
set "PYTHONUTF8=1"
if not exist "%VPY%" (
  py -3 -m venv "%SIM_DIR%\vision\.venv" || python -m venv "%SIM_DIR%\vision\.venv" || exit /b 1
)
"%VPY%" -c "import torch, ultralytics, tensorrt, onnx" >nul 2>&1 || "%VPY%" -m pip install -r "%SIM_DIR%\vision\requirements.txt" || exit /b 1
rem One vision service at a time (port 8772).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0_stop-previous.ps1" vision || exit /b 1
set "ENGINE="
set "MODEL_ARGS="
echo.%* | findstr /c:"--model" >nul && goto run
if defined NO_TRT goto run
for /f "usebackq tokens=1,* delims==" %%A in (`call "%VPY%" "%SIM_DIR%\vision\ensure_engine.py"`) do if "%%A"=="ENGINE" set "ENGINE=%%B"
if defined ENGINE (set MODEL_ARGS=--model "%ENGINE%") else echo Using PyTorch instead of TensorRT.
:run
"%VPY%" "%SIM_DIR%\vision\vision_service.py" %MODEL_ARGS% %*
