@echo off
rem Interactive usdview on a RigExec example with the live Hydra integration.
rem Usage: launch_usdview.bat [stage.usda] [rendererDisplayName]
rem   stage defaults to examples\ArmShotAnim.usda; renderer e.g. Embree.
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
set USD=D:\work\usdRig\usd-install
set RIG=D:\work\usdRig\usdRig
set PATH=%RIG%\build;%USD%\bin;%USD%\lib;%PATH%
cmake --build %RIG%\build >nul
if errorlevel 1 exit /b 1
set PYTHONPATH=%RIG%\plugin\rigExecUsdview;%USD%\Lib\site-packages
set PXR_PLUGINPATH_NAME=%RIG%\plugin\rigExecSchema\resources;%RIG%\plugin\rigExecImaging\resources;%RIG%\plugin\rigExecUsdview
set STAGE=%1
if "%STAGE%"=="" set STAGE=%RIG%\examples\ArmShotAnim.usda
set RENDERER_ARG=
if not "%2"=="" set RENDERER_ARG=--renderer %2
python %USD%\bin\usdview %RENDERER_ARG% %STAGE%
exit /b %errorlevel%
