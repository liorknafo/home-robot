@echo off
rem Process recorded camera pairs on Windows (counterpart of process-cameras.command).
setlocal
call "%~dp0_venv.cmd" || exit /b 1
"%SIMPY%" "%SIM_DIR%\controller\stereo_mapping.py" || exit /b 1
"%SIMPY%" "%SIM_DIR%\controller\plot_vision.py" || exit /b 1
"%SIMPY%" "%SIM_DIR%\Scripts\build_report.py" || exit /b 1
echo Results are in reports\vision and simulation-review.html.
