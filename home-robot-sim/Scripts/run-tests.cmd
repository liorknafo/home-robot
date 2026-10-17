@echo off
rem Physics trials against the real Unreal process on Windows (counterpart of run-tests.command).
setlocal
call "%~dp0_ue.cmd" || exit /b 1
call "%~dp0_venv.cmd" || exit /b 1
"%SIMPY%" "%SIM_DIR%\tests\run_physics.py" %*
