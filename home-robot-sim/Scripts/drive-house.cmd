@echo off
rem Drive the house route on Windows (counterpart of drive-house.command). Start launch.cmd first.
setlocal
call "%~dp0_venv.cmd" || exit /b 1
"%SIMPY%" "%SIM_DIR%\tests\house_route.py" %*
