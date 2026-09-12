@echo off
rem Runs one of the tools\biped scripts in the RigExec plugin environment.
rem
rem Usage: run_biped.bat <script> [args...]
rem   <script> is the name with or without .py, e.g. `set_ikfk` or
rem   `set_ikfk.py`; it is resolved inside tools\biped so you do not have to
rem   type the path.
rem
rem Why this exists: these scripts import both `pxr` and `rigexec`, which
rem needs PYTHONPATH, PATH and PXR_PLUGINPATH_NAME set the way _env.bat sets
rem them. Running `python tools\biped\set_ikfk.py` from a bare shell fails on
rem the import, and pointing PXR_PLUGINPATH_NAME at the SOURCE schema
rem resources instead of the generated ones fails differently, with a
rem duplicate-plugin registration error. _env.bat already gets both right.
rem
rem Examples:
rem   bin\run_biped.bat set_ikfk biped_full2.usda --show
rem   bin\run_biped.bat set_ikfk biped_full2.usda --limb arm_l --value 1
rem   bin\run_biped.bat verify_spine biped_full2.usda
rem   bin\run_biped.bat exec_stack biped_full2.usda --type constraint
setlocal EnableDelayedExpansion
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1

set "SCRIPT=%~1"
if not defined SCRIPT (
    >&2 echo Usage: run_biped.bat ^<script^> [args...]
    >&2 echo        scripts available in %RIG%\tools\biped:
    for %%F in ("%RIG%\tools\biped\*.py") do >&2 echo          %%~nF
    exit /b 2
)
shift

rem Accept `set_ikfk` and `set_ikfk.py` alike.
set "TARGET=%RIG%\tools\biped\%SCRIPT%"
if not exist "%TARGET%" set "TARGET=%RIG%\tools\biped\%SCRIPT%.py"
if not exist "%TARGET%" (
    >&2 echo ERROR: no such script: %SCRIPT% ^(looked in %RIG%\tools\biped^)
    exit /b 1
)

rem Collect the remaining arguments verbatim; %* still holds the script name
rem after a shift, so it cannot be used here.
set "ARGS="
:collect
if "%~1"=="" goto :run
set "ARGS=!ARGS! %1"
shift
goto :collect

:run
"%PY%" "%TARGET%"!ARGS!
exit /b %errorlevel%
