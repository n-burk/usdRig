@echo off
setlocal
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1
set "RIGEXEC_EVALUATION_MODE="
set "RIGEXEC_DYNAMIC_RUNS_PROGRAM="
set "RIGEXEC_FRAME_CACHE=on"
set "RIGEXEC_FRAME_CACHE_VERIFY=0"
set "RIGEXEC_ENABLE_PARALLEL_EVAL=1"
set "STAGE=%~1"
if not defined STAGE set "STAGE=%RIG%\examples\ArmShotAnim.usda"
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewFrameCache.py" "%STAGE%"
exit /b %errorlevel%
