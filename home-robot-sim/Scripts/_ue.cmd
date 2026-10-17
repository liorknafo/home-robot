@echo off
rem Sets SIM_DIR and UE_ROOT for the Windows scripts. UE_ROOT, when already set, wins; otherwise the UE_5.8 install the
rem Epic Games Launcher recorded (any drive), else the launcher's default folder.
for %%I in ("%~dp0..") do set "SIM_DIR=%%~fI"
if defined UE_ROOT goto :check
for /f "usebackq delims=" %%P in (`powershell -NoProfile -Command "try{(((Get-Content -Raw \"$env:ProgramData\Epic\UnrealEngineLauncher\LauncherInstalled.dat\"|ConvertFrom-Json).InstallationList|Where-Object AppName -eq 'UE_5.8'|Select-Object -First 1).InstallLocation).Replace('/','\')}catch{}"`) do set "UE_ROOT=%%P"
if not defined UE_ROOT set "UE_ROOT=C:\Program Files\Epic Games\UE_5.8"
:check
if not exist "%UE_ROOT%\Engine\Build\BatchFiles\Build.bat" (
  echo Unreal Engine 5.8 not found at "%UE_ROOT%". Install it from the Epic Games Launcher or set UE_ROOT.
  exit /b 1
)
exit /b 0
