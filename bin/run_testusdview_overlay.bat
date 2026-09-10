@echo off
rem Headless verification that the volumetric influence overlay reaches a
rem REAL usdview viewport (tests\testUsdviewVolumeWeightOverlay.py).
rem Usage: run_testusdview_overlay.bat [rendererDisplayName]   (e.g. Embree)
call "%~dp0_vcvars.bat"
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1
cmake --build "%RIG%\build" >nul
if errorlevel 1 exit /b 1
set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewVolumeWeightOverlay.py" %RENDERER_ARG% "%RIG%\examples\11_VolumeWeights.usda"
exit /b %errorlevel%
