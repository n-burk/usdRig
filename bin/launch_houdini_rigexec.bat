@echo off
rem Launches Houdini Indie (Steam) with the RigExec USD plugins, libraries,
rem and Python bindings built in build-houdini, plus the usdMayaRig Maya
rem ASCII file-format sidecar.
rem
rem Usage: launch_houdini_rigexec.bat [gui^|hbatch^|hython^|check] [args...]
rem
rem   gui     Start the Houdini GUI (default). Extra args are passed through.
rem   hbatch  Start hbatch (batch Houdini) with the RigExec environment.
rem   hython  Start hython (Houdini's Python) with the RigExec environment.
rem   check   Run the headless plugin verification (hython
rem           bin\check_houdini_rigexec.py) and report. No GUI.
rem
rem Environment overrides:
rem   HFS / HOUDINI_ROOT  Houdini install (default: Steam Houdini Indie)
rem   RIG                 repo root (default: this file's parent directory)
rem   INSTALL             RigExec install prefix (default:
rem                       %RIG%\build-houdini\install)
rem   MAYAINSTALL         usdMayaRig sidecar install prefix (default:
rem                       %RIG%\..\usdMayaRig\build-houdini\install)
rem   HOUDINI_APP         GUI binary name (default: hindie.steam.exe)
setlocal

if not defined RIG (for %%I in ("%~dp0..") do set "RIG=%%~fI")
if not defined HOUDINI_ROOT if defined HFS set "HOUDINI_ROOT=%HFS%"
if not defined HOUDINI_ROOT set "HOUDINI_ROOT=D:\SteamLibrary\steamapps\common\Houdini Indie"
if not defined INSTALL set "INSTALL=%RIG%\build-houdini\install"
if not defined MAYAINSTALL set "MAYAINSTALL=%RIG%\..\usdMayaRig\build-houdini\install"
if not defined HOUDINI_APP set "HOUDINI_APP=hindie.steam.exe"

if not exist "%HOUDINI_ROOT%\bin\hython.exe" (
    >&2 echo ERROR: no Houdini install at "%HOUDINI_ROOT%".
    >&2 echo        Set HFS=^<houdini-root^> and retry.
    exit /b 1
)
if not exist "%INSTALL%\lib\usd\rigExecSchema\resources\plugInfo.json" (
    >&2 echo ERROR: no RigExec install at "%INSTALL%".
    >&2 echo        Run bin\build_rigexec_houdini.bat first, or set
    >&2 echo        INSTALL=^<rigexec-install-prefix^>.
    exit /b 1
)
if not exist "%INSTALL%\lib\rigExecImaging.dll" (
    >&2 echo ERROR: "%INSTALL%\lib\rigExecImaging.dll" is missing.
    >&2 echo        Re-run bin\build_rigexec_houdini.bat.
    exit /b 1
)
if not exist "%MAYAINSTALL%\lib\usd\usdMayaRig\resources\plugInfo.json" (
    >&2 echo ERROR: no usdMayaRig sidecar install at "%MAYAINSTALL%".
    >&2 echo        Run ..\usdMayaRig\build_usdmayarig_houdini.bat first, or
    >&2 echo        set MAYAINSTALL=^<sidecar-install-prefix^>.
    exit /b 1
)

rem Scrub inherited entries that would shadow Houdini's embedded Python
rem 3.13 or its vendored USD before composing the child environment: a
rem directory shipping python3*.dll (CPython 3.10's python310.dll sits on a
rem default PATH), a foreign pxr package (the stock build's cp310 binaries
rem fail under 3.13 with "DLL load failed while importing _tf"), a
rem stock-build DLL dir whose file names collide with the Houdini twins, or
rem anything MoonRay. Our own directories are prepended after the scrub, so
rem they are unaffected; drops are reported to stderr. PYTHONHOME must be
rem empty or the embedded interpreter initializes against the wrong stdlib.
call :scrub PATH
call :scrub PYTHONPATH
call :scrub PXR_PLUGINPATH_NAME
set "PYTHONHOME="
set "PYTHONEXECUTABLE="
rem The RigExec DLLs live in install\lib (plain directory, not a Houdini
rem dso path), so they ride on PATH; the Python bindings and package through
rem PYTHONPATH. The USD plugins register through HOUDINI_USD_DSO_PATH:
rem Houdini assembles Plug's search list itself and does not honor
rem PXR_PLUGINPATH_NAME, which is still set for any stock-USD tool pointed
rem at this install. The trailing ;& keeps Houdini's own entries.
set "PATH=%INSTALL%\lib;%MAYAINSTALL%\lib;%HOUDINI_ROOT%\bin;%PATH%"
set "PXR_PLUGINPATH_NAME=%INSTALL%\lib\usd;%MAYAINSTALL%\lib\usd;%PXR_PLUGINPATH_NAME%"
set "HOUDINI_USD_DSO_PATH=%INSTALL%\lib\usd\rigExecSchema\resources;%INSTALL%\lib\usd\rigExecImaging\resources;%MAYAINSTALL%\lib\usd\usdMayaRig\resources;&"
set "PYTHONPATH=%INSTALL%\lib\python;%PYTHONPATH%"

set "MODE=%~1"
if not defined MODE set "MODE=gui"
if "%MODE%"=="gui" goto :gui
if "%MODE%"=="hbatch" goto :hbatch
if "%MODE%"=="hython" goto :hython
if "%MODE%"=="check" goto :check
>&2 echo ERROR: unknown mode "%MODE%". Use gui, hbatch, hython, or check.
exit /b 1

:gui
if not exist "%HOUDINI_ROOT%\bin\%HOUDINI_APP%" (
    >&2 echo ERROR: "%HOUDINI_ROOT%\bin\%HOUDINI_APP%" not found.
    >&2 echo        Set HOUDINI_APP=^<binary^> to choose another entry point.
    exit /b 1
)
rem A direct launch outside the Steam client still needs the Steam app id in
rem the environment, or the GUI exits immediately with code 3. An explicit
rem SteamAppId wins; otherwise it is read from the install.
if not defined SteamAppId (
    if exist "%HOUDINI_ROOT%\steam_appid.txt" (
        for /f "usebackq tokens=*" %%A in ("%HOUDINI_ROOT%\steam_appid.txt") do set "SteamAppId=%%A"
    )
)
if not defined SteamGameId if defined SteamAppId set "SteamGameId=%SteamAppId%"
shift
start "" "%HOUDINI_ROOT%\bin\%HOUDINI_APP%" %1 %2 %3 %4 %5 %6 %7 %8 %9
exit /b 0

:hbatch
shift
"%HOUDINI_ROOT%\bin\hbatch.exe" %1 %2 %3 %4 %5 %6 %7 %8 %9
exit /b %errorlevel%

:hython
shift
"%HOUDINI_ROOT%\bin\hython.exe" %1 %2 %3 %4 %5 %6 %7 %8 %9
exit /b %errorlevel%

:check
shift
"%HOUDINI_ROOT%\bin\hython.exe" "%RIG%\bin\check_houdini_rigexec.py" %1 %2 %3 %4 %5 %6 %7 %8 %9
exit /b %errorlevel%

rem Drops hostile segments from the named ;-separated variable (%1). A
rem segment goes when it ships versioned CPython DLLs, a pxr package, a
rem stock-build USD/RigExec/usdGen/usdMayaRig DLL, or anything MoonRay
rem (hdMoonray plugin or a moonray-named directory); every Houdini twin is
rem prepended after the scrub, so nothing of ours can match. Segments are
rem unquoted on the way through; a segment containing ! does not survive
rem delayed expansion (vanishingly rare in PATH-like variables).
:scrub
setlocal EnableDelayedExpansion
set "_SCRUB_OUT="
for %%S in ("!%~1:;=" "!") do (
    set "_SEG=%%~S"
    set "_DROP="
    if not "!_SEG!"=="" (
        if exist "!_SEG!\python3*.dll" set "_DROP=1"
        if exist "!_SEG!\pxr\__init__.py" set "_DROP=1"
        if exist "!_SEG!\usd_usd.dll" set "_DROP=1"
        if exist "!_SEG!\rigExec.dll" set "_DROP=1"
        if exist "!_SEG!\usdGen*.dll" set "_DROP=1"
        if exist "!_SEG!\usdMayaRig.dll" set "_DROP=1"
        if exist "!_SEG!\hdMoonray*.dll" set "_DROP=1"
        if /i not "!_SEG!"=="!_SEG:moonray=!" set "_DROP=1"
    )
    if defined _DROP (
        >&2 echo [%~n0] dropped hostile %~1 segment: !_SEG!
    ) else if not "!_SEG!"=="" (
        if defined _SCRUB_OUT (
            set "_SCRUB_OUT=!_SCRUB_OUT!;!_SEG!"
        ) else (
            set "_SCRUB_OUT=!_SEG!"
        )
    )
)
endlocal & set "%~1=%_SCRUB_OUT%"
exit /b 0
