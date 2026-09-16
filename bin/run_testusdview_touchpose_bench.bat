@echo off
rem TouchPose latency benchmark (tests\testUsdviewTouchPoseBench.py) in a
rem real usdview. The Windows twin of run_testusdview_touchpose_bench.sh,
rem which describes the numbers it prints.
rem
rem Usage: run_testusdview_touchpose_bench.bat [stage.usda]
rem   stage defaults to examples\biped\Biped_all.usda.
setlocal EnableDelayedExpansion
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1

set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%RIG%\plugin\touchPose"
set "PYTHONPATH=%RIG%\plugin\touchPose;%PYTHONPATH%"
set "TOUCHPOSE_PLUGIN_DIR=%RIG%\plugin\touchPose"

set "STAGE=%~1"
if not defined STAGE set "STAGE=%RIG%\examples\biped\Biped_all.usda"

"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewTouchPoseBench.py" "%STAGE%"
exit /b %errorlevel%
