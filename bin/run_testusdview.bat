@echo off
rem Headless verification of the live RigExec Hydra integration.
rem Usage: run_testusdview.bat [rendererDisplayName]   (e.g. Embree)
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
call "%~dp0_env.bat"
cmake --build "%RIG%\build" >nul
if errorlevel 1 exit /b 1
set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1
python "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewRigExec.py" %RENDERER_ARG% "%RIG%\examples\ArmShotAnim.usda"
exit /b %errorlevel%
