@echo off
rem Build the editor target on Windows (counterpart of build.command). UE_ROOT overrides the engine folder.
setlocal
call "%~dp0_ue.cmd" || exit /b 1
call "%UE_ROOT%\Engine\Build\BatchFiles\Build.bat" HomeRobotSimEditor Win64 Development "-Project=%SIM_DIR%\HomeRobotSim.uproject" -WaitMutex -NoHotReloadFromIDE
