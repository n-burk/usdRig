@echo off
rem Runs a plugin\touchPose spike INSIDE a real usdview, via testusdview.
rem
rem Usage: run_touchpose_view.bat <spike> [stage.usda] [rendererDisplayName]
rem   e.g. run_touchpose_view.bat r2_overlay_visibility
rem        run_touchpose_view.bat r3_pick_cost examples\biped\Biped_layered.usda
rem
rem Two of the Phase 0 spikes cannot be answered from a script: R2 has to
rem count PIXELS the app actually drew (a primvar that is perfectly correct
rem at the scene index and invisible on screen is the exact failure mode
rem being tested for), and R3 has to read the deformed points back out of
rem the app's OWN terminal scene index, since that is the object a pick
rem would have to query.
rem
rem DELIBERATELY NO `cmake --build`, unlike run_testusdview_overlay.bat: an
rem open usdview holds a lock on build\rigExec.dll and the build fails on
rem the link step, taking the spike with it. Phase 0 adds no C++, so there
rem is nothing here that needs building.
setlocal EnableDelayedExpansion
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1

set "SPIKE=%~1"
if not defined SPIKE (
    >&2 echo Usage: run_touchpose_view.bat ^<spike^> [stage.usda] [renderer]
    for %%F in ("%RIG%\plugin\touchPose\spikes\*.py") do >&2 echo          %%~nF
    exit /b 2
)
set "TARGET=%RIG%\plugin\touchPose\spikes\%SPIKE%"
if not exist "%TARGET%" set "TARGET=%RIG%\plugin\touchPose\spikes\%SPIKE%.py"
if not exist "%TARGET%" (
    >&2 echo ERROR: no such spike: %SPIKE%
    exit /b 1
)

rem testusdview `exec`s the script, so it has no __file__ to locate the
rem package from. Handed over explicitly rather than guessed at.
set "TOUCHPOSE_PLUGIN_DIR=%RIG%\plugin\touchPose"

set "STAGE=%~2"
if not defined STAGE set "STAGE=%RIG%\examples\biped\Biped.usda"
set RENDERER_ARG=
if not "%~3"=="" set RENDERER_ARG=--renderer %~3

"%PY%" "%USD%\bin\testusdview" --testScript "%TARGET%" %RENDERER_ARG% "%STAGE%"
exit /b %errorlevel%
