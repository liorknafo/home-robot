@echo off
rem Launch the simulator on Windows (counterpart of launch.command). Usage: launch.cmd [scenario=house] [floor finish=polished]
setlocal
call "%~dp0_ue.cmd" || exit /b 1
rem One simulator at a time: a second one would share UDP port 8767 and split the robot's commands between them.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0_stop-previous.ps1" sim || exit /b 1
rem The dashboard's autonomy service and object detection start alongside, each in its own minimized window and each
rem replacing any earlier run (an autonomy service left from a previous simulator keeps that run's robot pose and map).
rem Set NO_AUTONOMY=1 or NO_VISION=1 to skip one, e.g. NO_VISION on a PC without an NVIDIA GPU.
if not defined NO_AUTONOMY start "Home robot autonomy" /min "%~dp0start-autonomy.cmd"
if not defined NO_VISION start "Home robot vision" /min "%~dp0start-vision.cmd"
set "SCENARIO=%~1"
if "%SCENARIO%"=="" set "SCENARIO=house"
set "FINISH=%~2"
if "%FINISH%"=="" set "FINISH=polished"
"%UE_ROOT%\Engine\Binaries\Win64\UnrealEditor.exe" "%SIM_DIR%\HomeRobotSim.uproject" /Engine/Maps/Entry -game -windowed -ResX=1280 -ResY=800 -nosound "-Scenario=%SCENARIO%" "-FloorFinish=%FINISH%"
