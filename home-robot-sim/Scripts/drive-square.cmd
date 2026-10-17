@echo off
rem Drive a 0.4 m square with camera capture on Windows (counterpart of drive-square.command). Start launch.cmd first.
setlocal
call "%~dp0_venv.cmd" || exit /b 1
"%SIMPY%" "%SIM_DIR%\controller\drive_demo.py" --side .4 --capture
