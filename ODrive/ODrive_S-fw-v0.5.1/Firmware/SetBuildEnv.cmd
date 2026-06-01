@echo off
REM ODrive firmware build — prepend local ARM GCC, GNU MCU build tools, Lua, and tup to PATH.
REM This file lives in Firmware\; tool folders are in the parent ODrive directory (two levels up).

set "ODRIVE_TOOLCHAIN_ROOT=%~dp0..\.."
if "%ODRIVE_TOOLCHAIN_ROOT:~-1%"=="\" set "ODRIVE_TOOLCHAIN_ROOT=%ODRIVE_TOOLCHAIN_ROOT:~0,-1%"

REM --- ARM GCC (7-2018-q2 win32 zip) ---
if exist "%ODRIVE_TOOLCHAIN_ROOT%\gcc-arm-none-eabi-7-2018-q2-update-win32\bin\arm-none-eabi-gcc.exe" (
  set "PATH=%ODRIVE_TOOLCHAIN_ROOT%\gcc-arm-none-eabi-7-2018-q2-update-win32\bin;%PATH%"
) else (
  echo WARNING: ARM GCC not found at gcc-arm-none-eabi-7-2018-q2-update-win32\bin
)

REM --- GNU MCU Eclipse Windows Build Tools (make, rm, etc.) ---
REM Typical extract: ...\gnu-mcu-eclipse-windows-build-tools-...\GNU MCU Eclipse\Build Tools\...\bin
if exist "%ODRIVE_TOOLCHAIN_ROOT%\gnu-mcu-eclipse-windows-build-tools-2.12-20190422-1053-win64\GNU MCU Eclipse\Build Tools\2.12-20190422-1053\bin\make.exe" (
  set "PATH=%ODRIVE_TOOLCHAIN_ROOT%\gnu-mcu-eclipse-windows-build-tools-2.12-20190422-1053-win64\GNU MCU Eclipse\Build Tools\2.12-20190422-1053\bin;%PATH%"
) else if exist "%ODRIVE_TOOLCHAIN_ROOT%\gnu-mcu-eclipse-windows-build-tools-2.12-20190422-1053-win64\bin\make.exe" (
  set "PATH=%ODRIVE_TOOLCHAIN_ROOT%\gnu-mcu-eclipse-windows-build-tools-2.12-20190422-1053-win64\bin;%PATH%"
) else if exist "%ODRIVE_TOOLCHAIN_ROOT%\GNU MCU Eclipse\Build Tools\2.12-20190422-1053\bin\make.exe" (
  set "PATH=%ODRIVE_TOOLCHAIN_ROOT%\GNU MCU Eclipse\Build Tools\2.12-20190422-1053\bin;%PATH%"
) else (
  echo WARNING: GNU MCU build tools not found — edit SetBuildEnv.cmd if your folder name differs.
)

REM --- Lua 5.5 (common extract layouts) ---
if exist "%ODRIVE_TOOLCHAIN_ROOT%\lua-5.5.0_Win32\bin\lua.exe" (
  set "PATH=%ODRIVE_TOOLCHAIN_ROOT%\lua-5.5.0_Win32\bin;%PATH%"
) else if exist "%ODRIVE_TOOLCHAIN_ROOT%\lua-5.5.0_Win32_bin\lua.exe" (
  set "PATH=%ODRIVE_TOOLCHAIN_ROOT%\lua-5.5.0_Win32_bin;%PATH%"
) else if exist "%ODRIVE_TOOLCHAIN_ROOT%\lua\lua.exe" (
  set "PATH=%ODRIVE_TOOLCHAIN_ROOT%\lua;%PATH%"
) else (
  echo WARNING: lua.exe not found — extract Lua 5.5 under ODrive or add a folder block below.
)

REM --- tup ---
if exist "%ODRIVE_TOOLCHAIN_ROOT%\tup\tup.exe" (
  set "PATH=%ODRIVE_TOOLCHAIN_ROOT%\tup;%PATH%"
) else if exist "%ODRIVE_TOOLCHAIN_ROOT%\tup.exe" (
  set "PATH=%ODRIVE_TOOLCHAIN_ROOT%;%PATH%"
) else (
  echo WARNING: tup.exe not found — put tup in ODrive\tup or ODrive\tup.exe.
)

echo.
echo ODrive tool PATH prepended from:
echo   %ODRIVE_TOOLCHAIN_ROOT%
echo.

REM noexit: use from an existing prompt so PATH stays in that window ^("call SetBuildEnv.cmd noexit"^)
if /i "%~1"=="noexit" exit /b 0

cmd /k
