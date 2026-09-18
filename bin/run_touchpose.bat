@echo off
rem Runs one of the plugin\touchPose scripts in the RigExec plugin environment.
rem
rem Usage: run_touchpose.bat <script> [args...]
rem   <script> is a name with or without .py, resolved first in
rem   plugin\touchPose and then in plugin\touchPose\spikes, so both
rem   `import_touch` and `r1_face_order` work without a path.
rem
rem Why it exists: same reason as run_biped.bat. These scripts import
rem `pxr` and `rigexec`, which need PYTHONPATH, PATH and
rem PXR_PLUGINPATH_NAME set the way _env.bat sets them; pointing
rem PXR_PLUGINPATH_NAME at the SOURCE schema resources instead of the
rem generated ones fails with a duplicate-plugin registration error.
rem
rem Examples:
rem   bin\run_touchpose.bat import_touch examples\biped\Biped.usda
rem   bin\run_touchpose.bat import_touch examples\biped\Biped.usda --list
rem   bin\run_touchpose.bat r1_face_order examples\biped\Biped.usda
setlocal EnableDelayedExpansion
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1

set "SCRIPT=%~1"
if not defined SCRIPT (
    >&2 echo Usage: run_touchpose.bat ^<script^> [args...]
    >&2 echo        scripts available in %RIG%\plugin\touchPose:
    for %%F in ("%RIG%\plugin\touchPose\*.py") do >&2 echo          %%~nF
    for %%F in ("%RIG%\plugin\touchPose\spikes\*.py") do >&2 echo          %%~nF
    exit /b 2
)
shift

set "TARGET=%RIG%\plugin\touchPose\%SCRIPT%"
if not exist "%TARGET%" set "TARGET=%RIG%\plugin\touchPose\%SCRIPT%.py"
if not exist "%TARGET%" set "TARGET=%RIG%\plugin\touchPose\spikes\%SCRIPT%"
if not exist "%TARGET%" set "TARGET=%RIG%\plugin\touchPose\spikes\%SCRIPT%.py"
if not exist "%TARGET%" (
    >&2 echo ERROR: no such script: %SCRIPT% ^(looked in plugin\touchPose and its spikes^)
    exit /b 1
)

rem %* still holds the script name after a shift, so the remaining
rem arguments are collected one at a time instead.
set "ARGS="
:collect
if "%~1"=="" goto :run
set "ARGS=!ARGS! %1"
shift
goto :collect

:run
"%PY%" "%TARGET%"!ARGS!
exit /b %errorlevel%
