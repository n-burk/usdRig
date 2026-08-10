@echo off
rem Curvenet authoring panel, verified against the puppetA character.
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
set USD=D:\work\usdRig\usd-install
set RIG=D:\work\usdRig\usdRig
set PATH=%RIG%\build;%USD%\bin;%USD%\lib;%PATH%
set PYTHONPATH=%RIG%\plugin\rigExecUsdview;%USD%\Lib\site-packages
set PXR_PLUGINPATH_NAME=%RIG%\build\usd\rigExecSchema\resources;%RIG%\build\usd\rigExecImaging\resources;%RIG%\plugin\rigExecUsdview
python %USD%\bin\testusdview --testScript %RIG%\tests\testUsdviewCurvenetPuppet.py D:\work\usdRig\chars\puppetA\puppetA_curvenet.usda
exit /b %errorlevel%
