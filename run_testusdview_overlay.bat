@echo off
rem Headless verification that the volumetric influence overlay reaches a
rem REAL usdview viewport (tests\testUsdviewVolumeWeightOverlay.py).
rem Usage: run_testusdview_overlay.bat [rendererDisplayName]   (e.g. Embree)
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
set USD=D:\work\usdRig\usd-install
set RIG=D:\work\usdRig\usdRig
set PATH=%RIG%\build;%USD%\bin;%USD%\lib;%PATH%
cmake --build %RIG%\build >nul
if errorlevel 1 exit /b 1
set PYTHONPATH=%RIG%\plugin\rigExecUsdview;%USD%\Lib\site-packages
set PXR_PLUGINPATH_NAME=%RIG%\build\usd\rigExecSchema\resources;%RIG%\build\usd\rigExecImaging\resources;%RIG%\plugin\rigExecUsdview
set RENDERER_ARG=
if not "%1"=="" set RENDERER_ARG=--renderer %1
python %USD%\bin\testusdview --testScript %RIG%\tests\testUsdviewVolumeWeightOverlay.py %RENDERER_ARG% %RIG%\examples\11_VolumeWeights.usda
exit /b %errorlevel%
