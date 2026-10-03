@echo off
setlocal
call "%~dp0..\_env.bat"
call "%~dp0..\_require_python.bat"
if errorlevel 1 exit /b 1
set "RIGEXEC_EVALUATION_MODE="
set "RIGEXEC_DYNAMIC_RUNS_PROGRAM="
set "RIGEXEC_FRAME_CACHE=on"
set "RIGEXEC_FRAME_CACHE_VERIFY=0"
set "RIGEXEC_ENABLE_PARALLEL_EVAL=1"
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewWrinkleFrameCache.py" "%RIG%\docs\examples\wrinkle_mover.usda"
exit /b %errorlevel%
