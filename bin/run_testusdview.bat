@echo off
rem Headless verification of the live RigExec Hydra integration.
rem Usage: run_testusdview.bat [rendererDisplayName]   (e.g. Embree)
call "%~dp0_vcvars.bat"
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1
cmake --build "%RIG%\build" >nul
if errorlevel 1 exit /b 1
set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewRigExec.py" %RENDERER_ARG% "%RIG%\examples\ArmShotAnim.usda"
exit /b %errorlevel%
