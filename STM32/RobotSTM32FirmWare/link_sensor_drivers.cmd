@echo off
setlocal
set "LINK=%~dp0sensor_drivers"
set "TARGET=%~dp0..\sensor_drivers"
if exist "%LINK%" (
  echo Junction already exists: %LINK%
  exit /b 0
)
mklink /J "%LINK%" "%TARGET%"
if errorlevel 1 (
  echo Failed to create junction. Run this script from an elevated prompt if needed.
  exit /b 1
)
echo Created junction: %LINK% -^> %TARGET%
