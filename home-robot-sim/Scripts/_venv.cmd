@echo off
rem Windows counterpart of the venv bootstrap in the .command scripts: makes .venv (Python 3.10+), installs
rem controller\requirements.txt when cv2/numpy/matplotlib are missing, and sets SIM_DIR and SIMPY for the caller.
for %%I in ("%~dp0..") do set "SIM_DIR=%%~fI"
set "SIMPY=%SIM_DIR%\.venv\Scripts\python.exe"
rem The sources and docs are UTF-8; Windows Python would otherwise read them in the ANSI code page.
set "PYTHONUTF8=1"
if not exist "%SIMPY%" (
  py -3 -m venv "%SIM_DIR%\.venv" || python -m venv "%SIM_DIR%\.venv" || exit /b 1
)
"%SIMPY%" -c "import cv2, numpy, matplotlib" >nul 2>&1 || "%SIMPY%" -m pip install -r "%SIM_DIR%\controller\requirements.txt" || exit /b 1
exit /b 0
