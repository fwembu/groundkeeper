@echo off
rem Groundkeeper: install the newest test build on the FM-1, over USB. Double-click to run; run it again for each new build.
rem It fetches the build from https://github.com/fwembu/groundkeeper/releases/tag/test-build, then runs fm1_install.py,
rem which asks before it writes anything. With the GitHub CLI installed and signed in (gh auth login, once), it downloads
rem through that, which works while the repository is private; without it, a plain download, which needs it public. Needs Python 3.12 (python-rtmidi has no ready-made Windows
rem build for newer Pythons) and curl (part of Windows 10 and 11).
setlocal
title Groundkeeper installer
set "URL=https://github.com/fwembu/groundkeeper/releases/download/test-build"
set "DIR=%LOCALAPPDATA%\Groundkeeper"
set PYTHONUTF8=1
if not exist "%DIR%" mkdir "%DIR%"

echo.
echo   GROUNDKEEPER installer
echo   ----------------------
echo.

rem 1. Python 3.12
py -3.12 --version >nul 2>&1
if errorlevel 1 goto nopython

rem 2. the MIDI add-ons (only the first time)
py -3.12 -c "import mido, rtmidi" >nul 2>&1
if not errorlevel 1 goto fetch
echo   Installing the MIDI add-ons (only needed once)...
py -3.12 -m pip install --quiet mido python-rtmidi
if errorlevel 1 goto pipfail

:fetch
rem 3. the newest build
echo   Downloading the newest build...
where gh >nul 2>&1
if errorlevel 1 goto curlfetch
gh release download test-build -R fwembu/groundkeeper -D "%DIR%" --clobber -p groundkeeper.fwsc -p fm1_install.py -p BUILD.txt || goto ghfail
goto fetched
:curlfetch
curl -fsSL -o "%DIR%\groundkeeper.fwsc" "%URL%/groundkeeper.fwsc" || goto dlfail
curl -fsSL -o "%DIR%\fm1_install.py" "%URL%/fm1_install.py" || goto dlfail
curl -fsSL -o "%DIR%\BUILD.txt" "%URL%/BUILD.txt" || goto dlfail
:fetched
echo.
type "%DIR%\BUILD.txt"
echo.

rem 4. install
echo   Connect the FM-1 by USB and switch it on.
echo   Close anything else that uses MIDI: the Firmware Hub page, music software.
echo.
pause
echo.
py -3.12 "%DIR%\fm1_install.py" "%DIR%\groundkeeper.fwsc"
if errorlevel 1 goto installfail
echo.
echo   Done: the FM-1 is running the new build.
goto end

:nopython
echo   Python 3.12 was not found. Install it from
echo     https://www.python.org/downloads/release/python-31210/
echo   (under Files: "Windows installer (64-bit)"), then run this again.
goto end

:pipfail
echo.
echo   Installing the MIDI add-ons failed: see the message above.
goto end

:dlfail
echo.
echo   The download failed. If the repository is private, this needs the GitHub CLI:
echo     1. install it from https://cli.github.com
echo     2. open Command Prompt and type:  gh auth login   (once; it signs in through the browser)
echo     3. run this again
echo   Otherwise, check the internet connection and try again.
goto end

:ghfail
echo.
echo   The GitHub CLI could not download the build. If you have not signed in yet, open
echo   Command Prompt and type:  gh auth login   then run this again.
goto end

:installfail
echo.
echo   The install did not finish (see the message above). If the FM-1 says "not found":
echo   close other MIDI software, unplug and replug the FM-1, and run this again.
echo   If the FM-1 will not start: hold OCT- while switching it on (USB rescue), then run this again.

:end
echo.
pause
