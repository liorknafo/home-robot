@echo off
rem Start the autonomy service on Windows (counterpart of start-autonomy.command); arguments pass through, e.g. --host 0.0.0.0
setlocal
call "%~dp0_venv.cmd" || exit /b 1
rem One autonomy service at a time (ports 8768 and 8771).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0_stop-previous.ps1" autonomy || exit /b 1
"%SIMPY%" "%SIM_DIR%\controller\autonomy_service.py" %*
